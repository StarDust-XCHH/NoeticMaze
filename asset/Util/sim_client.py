import numpy as np
import matplotlib.pyplot as plt
import heapq
import socket
import sys
import struct

# ==========================================
# 1. 环境与物理引擎 (保持上帝视角用于生成物理雷达点，不参与控制)
# ==========================================
class MazeRoom:
    def __init__(self, size=5.0):
        self.size = size
        self.wall_segments = []
        true_environment = [
            [[1,0,0,1], [1,0,0,0], [1,0,1,0], [1,0,0,0], [1,1,1,0]],
            [[0,1,0,1], [0,0,1,1], [1,1,0,0], [0,1,0,1], [1,1,0,1]],
            [[0,1,0,1], [1,0,1,1], [0,1,1,0], [0,0,0,1], [0,1,0,0]],
            [[0,0,1,1], [1,0,0,0], [1,0,1,0], [0,1,1,0], [0,1,0,1]],
            [[1,0,1,1], [0,0,1,0], [1,0,1,0], [1,0,1,0], [0,1,1,0]]
        ]
        added_walls = set()
        for row in range(5):
            for col in range(5):
                cell = true_environment[row][col]
                x_min, x_max = col, col + 1
                y_min, y_max = 4 - row, 5 - row
                if cell[0] == 1: self._add_wall((x_min, y_max), (x_max, y_max), added_walls)
                if cell[1] == 1: self._add_wall((x_max, y_min), (x_max, y_max), added_walls)
                if cell[2] == 1: self._add_wall((x_min, y_min), (x_max, y_min), added_walls)
                if cell[3] == 1: self._add_wall((x_min, y_min), (x_min, y_max), added_walls)

    def _add_wall(self, p1, p2, added_walls):
        p1, p2 = tuple(map(float, p1)), tuple(map(float, p2))
        seg = tuple(sorted([p1, p2]))
        if seg not in added_walls:
            added_walls.add(seg)
            self._generate_straight_wall(seg[0], seg[1])

    def _generate_straight_wall(self, p1, p2, resolution=0.2):
        p1, p2 = np.array(p1), np.array(p2)
        dist = np.linalg.norm(p2 - p1)
        n_points = max(2, int(dist / resolution))
        t = np.linspace(0, 1, n_points)
        line_points = np.outer(1-t, p1) + np.outer(t, p2)
        for i in range(len(line_points)-1):
            self.wall_segments.append((line_points[i], line_points[i+1]))

    def get_local_lidar_scan(self, robot_pos, robot_yaw, v, w, scan_time, num_rays, max_range, drop_rate=0.08):
        local_scan = []
        global_valid = []
        angles = np.linspace(0, 2*np.pi, num_rays, endpoint=False)
        rx_start, ry_start = robot_pos

        for i, local_angle in enumerate(angles):
            if np.random.rand() < drop_rate:
                local_scan.append([0.0, 0.0])
                continue

            dt_ray = scan_time * (i / num_rays)
            current_yaw = robot_yaw + w * dt_ray
            if abs(w) > 1e-4:
                current_rx = rx_start + (v / w) * (np.sin(current_yaw) - np.sin(robot_yaw))
                current_ry = ry_start - (v / w) * (np.cos(current_yaw) - np.cos(robot_yaw))
            else:
                current_rx = rx_start + v * np.cos(robot_yaw) * dt_ray
                current_ry = ry_start + v * np.sin(robot_yaw) * dt_ray

            global_angle = current_yaw - local_angle
            dx, dy = np.cos(global_angle), np.sin(global_angle)

            min_dist = max_range
            hit = False
            for p1, p2 in self.wall_segments:
                denom = (p2[1] - p1[1]) * dx - (p2[0] - p1[0]) * dy
                if abs(denom) < 1e-6: continue
                ua = ((p2[0]-p1[0])*(current_ry-p1[1]) - (p2[1]-p1[1])*(current_rx-p1[0])) / denom
                ub = (dx*(current_ry-p1[1]) - dy*(current_rx-p1[0])) / denom
                if ua > 0 and 0 <= ub <= 1 and ua < min_dist:
                    min_dist = ua
                    hit = True

            if hit:
                dist_noise = min_dist + np.random.normal(0, 0.01)
                lx = dist_noise * np.cos(-local_angle)
                ly = dist_noise * np.sin(-local_angle)
                local_scan.append([lx, ly])
                global_valid.append([current_rx + dist_noise * dx, current_ry + dist_noise * dy])
            else:
                local_scan.append([0.0, 0.0])

        return np.array(local_scan), np.array(global_valid)

# ==========================================
# 2. 机器人认知系统：动态地图与规划
# ==========================================
class DynamicMap:
    def __init__(self, size=6.0, res=0.1):
        self.res = res
        self.size = size
        self.grid_size = int(size / res)
        # 0: 未知/空闲 (允许跨越), 1: 障碍物
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)

    def update(self, pose, local_scan):
        """将雷达点基于当前 ICP 位姿投影到地图，更新静态障碍物"""
        px, py, pth = pose
        for pt in local_scan:
            dist = np.linalg.norm(pt)
            if dist < 0.1: continue  # 忽略丢弃点或过近的噪点
            
            # 局部坐标转全局坐标
            gx = px + pt[0] * np.cos(pth) - pt[1] * np.sin(pth)
            gy = py + pt[0] * np.sin(pth) + pt[1] * np.cos(pth)
            
            idx_x, idx_y = int(gx / self.res), int(gy / self.res)
            if 0 <= idx_x < self.grid_size and 0 <= idx_y < self.grid_size:
                self.grid[idx_x, idx_y] = 1 # 标记障碍物

    def check_collision(self, pos, radius=0.22):
        """检测坐标点是否触碰障碍物（未知区域默认安全）"""
        px, py = pos
        cell_r = int(radius / self.res)
        cx, cy = int(px / self.res), int(py / self.res)
        
        # 超出地图边界视为墙壁
        if not (0 <= cx < self.grid_size and 0 <= cy < self.grid_size):
            return True 
            
        # 检查机器人半径内的像素
        for i in range(-cell_r, cell_r + 1):
            for j in range(-cell_r, cell_r + 1):
                if i**2 + j**2 <= cell_r**2:
                    nx, ny = cx + i, cy + j
                    if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                        if self.grid[nx, ny] == 1:
                            return True
        return False

def astar_plan(dynamic_map, start, goal):
    """基于动态地图进行规划，未知区域由于为 0 会被直接规划连线"""
    grid_res = dynamic_map.res
    def h(a, b): return np.linalg.norm(np.array(a) - np.array(b))

    # 【修复】：使用 np.round 替代直接 int 截断，将最大初始误差从 0.1m 缩小到 0.05m
    start_grid = tuple(np.round(np.array(start) / grid_res).astype(int))
    goal_grid = tuple(np.round(np.array(goal) / grid_res).astype(int))
    
    start_grid = tuple((np.array(start) / grid_res).astype(int))
    goal_grid = tuple((np.array(goal) / grid_res).astype(int))

    open_list = [(0, start_grid)]
    came_from = {}
    g_score = {start_grid: 0}

    while open_list:
        _, current = heapq.heappop(open_list)
        # 到达目标附近即可
        if h(current, goal_grid) < 1.5: 
            path = []
            while current in came_from:
                path.append(np.array(current) * grid_res)
                current = came_from[current]
            return path[::-1]

        for dx, dy in [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if not (0 <= neighbor[0] < dynamic_map.grid_size and 0 <= neighbor[1] < dynamic_map.grid_size): 
                continue
            
            # 使用构建的地图进行碰撞检测
            if dynamic_map.check_collision(np.array(neighbor) * grid_res, radius=0.22): 
                continue

            tg = g_score[current] + h(current, neighbor)
            if neighbor not in g_score or tg < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tg
                heapq.heappush(open_list, (tg + h(neighbor, goal_grid), neighbor))
    return None


def bresenham_line_check(dynamic_map, p1, p2, radius=0.22, ignore_start_dist=0.0): 
    """射线检测：判断两点之间的连线是否碰到障碍物"""
    dist = np.linalg.norm(p2 - p1)
    
    if dist < 0.1: 
        return False 
        
    steps = int(dist / (dynamic_map.res / 2)) 
    if steps <= 0: 
        return False
    
    for i in range(1, steps + 1):
        t = i / steps
        pt = p1 * (1 - t) + p2 * t
        
        # 【核心修复】：起步泥潭豁免。如果该点距离起点非常近，免除碰撞检测！
        # 允许小车从贴墙状态安全地“拔”出来
        if np.linalg.norm(pt - p1) < ignore_start_dist:
            continue
            
        if dynamic_map.check_collision(pt, radius=radius): 
            return True
            
    return False

def smooth_path(dynamic_map, path):
    """极限取直：砍掉路径中多余的节点，保留最长直线"""
    if not path or len(path) < 3: 
        return path
        
    smoothed = [path[0]]
    curr_idx = 0
    while curr_idx < len(path) - 1:
        furthest_visible = curr_idx + 1
        # 从最后面的节点开始往前倒推，找第一个能直线看得到的点
        for i in range(len(path) - 1, curr_idx, -1):
            if not bresenham_line_check(dynamic_map, path[curr_idx], path[i]):
                furthest_visible = i
                break
        smoothed.append(path[furthest_visible])
        curr_idx = furthest_visible
        
    return smoothed

def apply_odometry_noise(v, w, dt):
    v_noisy = v * 0.8 + np.random.normal(0, 0.02)
    w_noisy = w * 1.15 + 0.00391974 + np.random.normal(0, 0.05)
    return v_noisy, w_noisy

def send_binary_msg(sock, is_init, update_map, odom_guess, v, w, local_scan):
    points_flat = local_scan.flatten().astype(np.float32)
    num_points = len(local_scan)
    header = struct.pack('<BBfffffI', 1 if is_init else 0, 1 if update_map else 0,
                         float(odom_guess[0]), float(odom_guess[1]), float(odom_guess[2]),
                         float(v), float(w), num_points)
    points_data = struct.pack(f'<{len(points_flat)}f', *points_flat)
    sock.sendall(header + points_data)

def recv_binary_msg(sock):
    try:
        data = sock.recv(12)
        if len(data) < 12: return None
        return np.array(struct.unpack('<fff', data))
    except: return None

# ==========================================
# 3. 仿真主循环
# ==========================================
def run_simulation():
    FEWER_RAYS = 360
    LONGER_RANGE = 10.0
    LIDAR_HZ = 10.0
    SCAN_TIME = 1.0 / LIDAR_HZ

    try:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect(('127.0.0.1', 9999))
        print("[*] Connected to ICP Server.")
    except ConnectionRefusedError:
        print("[!] ERROR: Start your bridge proxy first!")
        sys.exit(1)

    world = MazeRoom(size=5.0)                   # 仅用于生成上帝视角的雷达数据
    robot_map = DynamicMap(size=6.0, res=0.1)    # 机器人用来规划的脑内地图

    start_pose = np.array([3.5, 0.5, np.pi])
    goal_pos = np.array([1.5, 2.5])
    true_pose, icp_pose, dead_reck_pose = start_pose.copy(), start_pose.copy(), start_pose.copy()

    # 初始路径规划
    # 初始路径规划与平滑 (注意这里套上了 smooth_path)
    raw_path = astar_plan(robot_map, icp_pose[:2], goal_pos)
    global_path = smooth_path(robot_map, raw_path) if raw_path else []
    
    path_idx, mission_phase = 0, 0
    v, w = 0.0, 0.0

    # 记录来时的实际轨迹（面包屑）
    breadcrumbs = [start_pose[:2].copy()]

    # 初始发包
    local_scan, current_scan_global = world.get_local_lidar_scan(true_pose[:2], true_pose[2], 0.0, 0.0, SCAN_TIME, FEWER_RAYS, LONGER_RANGE)
    send_binary_msg(client, True, True, true_pose, 0.0, 0.0, local_scan)
    icp_pose = recv_binary_msg(client)
    robot_map.update(icp_pose, local_scan)

    hist_true, hist_icp, hist_odom = [], [], []
    dt = 0.1 
    plt.ion(); fig, ax = plt.subplots(figsize=(7, 7))

    t = 0
    while mission_phase < 2:
        # --- 1. 获取上帝视角的物理反馈 ---
        local_scan, current_scan_global = world.get_local_lidar_scan(true_pose[:2], true_pose[2], v, w, SCAN_TIME, FEWER_RAYS, LONGER_RANGE)
        
        # 【恢复功能】：计算雷达有效点数
        valid_count = np.sum(np.linalg.norm(local_scan, axis=1) > 1e-4)

        # 真值推演
        true_pose[2] += w * dt
        true_pose[0] += v * np.cos(true_pose[2]) * dt
        true_pose[1] += v * np.sin(true_pose[2]) * dt

        # 里程计推演
        v_odom, w_odom = apply_odometry_noise(v, w, dt)
        dead_reck_pose[2] += w_odom * dt
        dead_reck_pose[0] += v_odom * np.cos(dead_reck_pose[2]) * dt
        dead_reck_pose[1] += v_odom * np.sin(dead_reck_pose[2]) * dt

        # --- 2. 与 C/MCU 层通信获取最新的 ICP 位姿 ---
        odom_guess = icp_pose + [v_odom*np.cos(icp_pose[2])*dt, v_odom*np.sin(icp_pose[2])*dt, w_odom*dt]
        send_binary_msg(client, False, (t % 5 == 0), odom_guess, v_odom, w_odom, local_scan)
        resp = recv_binary_msg(client)
        if resp is not None: icp_pose = resp

        # --- 3. 机器人认知更新：融合地图与路径拦截检测 ---
        # --- 3. 机器人认知更新：融合地图与局部路径拦截检测 ---
        robot_map.update(icp_pose, local_scan)
        
        # 【面包屑记录】：边走边撒下安全坐标点
        if mission_phase == 0 and np.linalg.norm(icp_pose[:2] - breadcrumbs[-1]) > 0.2:
            breadcrumbs.append(icp_pose[:2].copy())
            
        if global_path and path_idx < len(global_path):
            path_blocked = False
            
            # 【核心修复】：引入双阈值（Hysteresis）容差！
            # 规划时依然保证绝对安全的 0.22m，但动态监测时放宽到 0.18m。
            # 允许微小的网格漂移和切角，彻底消除死锁。
            chk_radius = 0.18 
            
            if bresenham_line_check(robot_map, icp_pose[:2], global_path[path_idx], radius=chk_radius, ignore_start_dist=0.25):
                path_blocked = True
            elif path_idx + 1 < len(global_path) and bresenham_line_check(robot_map, global_path[path_idx], global_path[path_idx+1], radius=chk_radius, ignore_start_dist=0.1):
                path_blocked = True
                
            if path_blocked:
                print(f"\n[!] 局部路径被阻挡 (触及 {chk_radius}m 红线)，重新规划并取直...")
                target_dest = goal_pos if mission_phase == 0 else start_pose[:2]
                new_path = astar_plan(robot_map, icp_pose[:2], target_dest)
                if new_path:
                    global_path = smooth_path(robot_map, new_path)
                    path_idx = 0
                else:
                    v, w = 0.0, 0.0 
                    print("[!] 无法找到新路径！")

        # --- 4. 双向纯跟踪速度解算 (允许倒车) ---
        if global_path and path_idx < len(global_path):
            target = global_path[path_idx]
            dist_to_target = np.linalg.norm(target - icp_pose[:2])
            
            # 到达当前路点，切换下一个
            # 到达当前路点，或者如果是刚规划出来的第一个点且离得很近，直接吃掉
            if dist_to_target < 0.2 or (path_idx == 0 and dist_to_target < 0.3):
                path_idx += 1
                v, w = 0.0, 0.0
            else:
                dx = target[0] - icp_pose[0]
                dy = target[1] - icp_pose[1]
                angle_to_target = np.arctan2(dy, dx)
                angle_err = (angle_to_target - icp_pose[2] + np.pi) % (2 * np.pi) - np.pi
                
                # 【判断是否需要倒车】：目标偏角 > 90度时
                is_reverse = False
                if abs(angle_err) > np.pi / 2:
                    is_reverse = True
                    # 翻转目标角度：把车尾当车头算角度偏差
                    angle_to_target = np.arctan2(-dy, -dx)
                    angle_err = (angle_to_target - icp_pose[2] + np.pi) % (2 * np.pi) - np.pi

                # 极限转角限制，超过阈值则原地调角度
                if abs(angle_err) > 0.4:
                    v = 0.0
                    w = np.sign(angle_err) * 0.4
                else:
                    # 如果倒车，速度给负数。角速度由于倒车的几何关系，保持原来的差值计算即可完成纠偏
                    v = -0.4 if is_reverse else 0.4
                    w = angle_err * 1.5
        else:
            if mission_phase == 0:
                mission_phase = 1
                # 【关键回程逻辑】：直接把来时的轨迹反转，然后拉直。完全不走未知区域！
                print("\n[*] 第一阶段完成！将根据来时已知轨迹拉直并倒车返航！")
                global_path = smooth_path(robot_map, breadcrumbs[::-1])
                path_idx = 0
            else:
                mission_phase = 2
                v, w = 0.0, 0.0 

        # --- 5. 可视化渲染 ---
        if t % 5 == 0:
            ax.cla()
            for p1, p2 in world.wall_segments: 
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-', lw=1, alpha=0.15)

            # 【修复点】：正确映射 x 和 y
            obs_x, obs_y = np.where(robot_map.grid == 1) 
            if len(obs_x) > 0:
                ax.plot(obs_x * robot_map.res, obs_y * robot_map.res, 'ks', markersize=3, label='Robot Mapped Walls')

            if global_path:
                path_arr = np.array(global_path)
                ax.plot(path_arr[:, 0], path_arr[:, 1], 'm-', lw=2, alpha=0.7, label='Dynamic A* Path')

            hist_true.append(true_pose.copy()); hist_icp.append(icp_pose.copy()); hist_odom.append(dead_reck_pose.copy())
            h_true, h_icp, h_odom = np.array(hist_true), np.array(hist_icp), np.array(hist_odom)

            ax.plot(h_true[:,0], h_true[:,1], 'g-', alpha=0.5, label='Ground Truth')
            ax.plot(h_odom[:,0], h_odom[:,1], 'b--', alpha=0.3, label='Dead Reckoning')
            ax.plot(h_icp[:,0], h_icp[:,1], 'r:', lw=2, label='ICP Pose')
            ax.arrow(icp_pose[0], icp_pose[1], 0.2*np.cos(icp_pose[2]), 0.2*np.sin(icp_pose[2]), head_width=0.1, color='r')

            # 【恢复功能】：图表标题显示有效点数
            ax.set_title(f"Dynamic A* + ICP Control | Frame {t:04d} | Valid Points: {valid_count}/360")
            ax.set_xlim(-0.5, 5.5); ax.set_ylim(-0.5, 5.5); ax.set_aspect('equal')
            ax.legend(loc='upper right', fontsize='small')
            plt.pause(0.01)
            
            # 【恢复功能】：终端打印有效点数和控制量
            print(f"\r[Sim] Frame {t:04d} | Valid: {valid_count:03d}/360 | v:{v:.2f} w:{w:.2f} | ICP:({icp_pose[0]:.2f}, {icp_pose[1]:.2f})", end="")
        t += 1

    # 【恢复功能】：最终误差统计报告
    final_drift = np.linalg.norm(true_pose[:2] - icp_pose[:2])
    odom_drift = np.linalg.norm(true_pose[:2] - dead_reck_pose[:2])
    print("\n\n" + "=" * 45)
    print("🎯 Final Report (Dynamic Map Exploration):")
    print("=" * 45)
    print(f"ICP Position Error: {final_drift:.4f} m")
    print(f"Pure Odom Error:    {odom_drift:.4f} m")
    print(f"Heading Error:      {np.degrees(abs(true_pose[2]-icp_pose[2]) % (2*np.pi)):.2f} degrees")
    print("=" * 45)

    client.close()
    plt.ioff(); plt.show()

if __name__ == "__main__":
    run_simulation()