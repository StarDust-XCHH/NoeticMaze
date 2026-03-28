import numpy as np
import matplotlib.pyplot as plt
import heapq
import socket
import sys
import struct

# ==========================================
# 1. 环境与物理引擎
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

    def check_collision(self, pos, radius=0.22):
        px, py = pos
        if px < radius or px > self.size - radius or py < radius or py > self.size - radius:
            return True
        for p1, p2 in self.wall_segments:
            l2 = (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2
            if l2 == 0: dist = np.hypot(px-p1[0], py-p1[1])
            else:
                t = max(0, min(1, ((px-p1[0])*(p2[0]-p1[0]) + (py-p1[1])*(p2[1]-p1[1])) / l2))
                proj_x = p1[0] + t * (p2[0]-p1[0])
                proj_y = p1[1] + t * (p2[1]-p1[1])
                dist = np.hypot(px-proj_x, py-proj_y)
            if dist < radius: return True
        return False

    # 【核心难度提升】：加入运动畸变 (Motion Skew) 模拟
    def get_local_lidar_scan(self, robot_pos, robot_yaw, v, w, scan_time, num_rays, max_range, drop_rate=0.08):
        local_scan = []      
        global_valid = []    

        angles = np.linspace(0, 2*np.pi, num_rays, endpoint=False)
        rx_start, ry_start = robot_pos

        for i, local_angle in enumerate(angles):
            # 1. 模拟随机丢点
            if np.random.rand() < drop_rate:
                local_scan.append([0.0, 0.0]) 
                continue

            # 2. 模拟运动畸变：计算这一束激光发射时的相对时间 (0 ~ scan_time)
            dt_ray = scan_time * (i / num_rays)
            
            # 3. 计算这一瞬间小车在世界坐标系下的真实位姿 (使用圆弧模型进行更精确的积分)
            current_yaw = robot_yaw + w * dt_ray
            if abs(w) > 1e-4:
                current_rx = rx_start + (v / w) * (np.sin(current_yaw) - np.sin(robot_yaw))
                current_ry = ry_start - (v / w) * (np.cos(current_yaw) - np.cos(robot_yaw))
            else:
                current_rx = rx_start + v * np.cos(robot_yaw) * dt_ray
                current_ry = ry_start + v * np.sin(robot_yaw) * dt_ray

            global_angle = local_angle + current_yaw
            dx, dy = np.cos(global_angle), np.sin(global_angle)

            min_dist = max_range
            hit = False
            for p1, p2 in self.wall_segments:
                # 注意：这里需要使用当前这根激光瞬间的 current_rx, current_ry 进行射线碰撞检测
                denom = (p2[1] - p1[1]) * dx - (p2[0] - p1[0]) * dy
                if abs(denom) < 1e-6: continue
                ua = ((p2[0]-p1[0])*(current_ry-p1[1]) - (p2[1]-p1[1])*(current_rx-p1[0])) / denom
                ub = (dx*(current_ry-p1[1]) - dy*(current_rx-p1[0])) / denom
                if ua > 0 and 0 <= ub <= 1 and ua < min_dist:
                    min_dist = ua
                    hit = True

            if hit:
                # 硬件测距噪声
                dist_noise = min_dist + np.random.normal(0, 0.01)
                
                # 4. 关键点：雷达硬件不知道车动了！它依旧按照最初的局部坐标系生成原始点云
                lx = dist_noise * np.cos(local_angle)
                ly = dist_noise * np.sin(local_angle)
                local_scan.append([lx, ly])
                
                # 全局画图用（展示变形后的点在全局真实位置的样子，会发现墙被“拉斜”了）
                global_valid.append([current_rx + dist_noise * dx, current_ry + dist_noise * dy])
            else:
                local_scan.append([0.0, 0.0])

        return np.array(local_scan), np.array(global_valid)

# ==========================================
# 2. 规划与辅助函数
# ==========================================
def astar_plan(world, start, goal, grid_res=0.2):
    def h(a, b): return np.linalg.norm(np.array(a) - np.array(b))
    start_grid = tuple((np.array(start) / grid_res).astype(int))
    goal_grid = tuple((np.array(goal) / grid_res).astype(int))
    open_list = [(0, start_grid)]; came_from = {}; g_score = {start_grid: 0}

    while open_list:
        _, current = heapq.heappop(open_list)
        if h(current, goal_grid) < 1.5:
            path = []
            while current in came_from:
                path.append(np.array(current) * grid_res)
                current = came_from[current]
            return path[::-1]

        for dx, dy in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if not (0 <= neighbor[0]*grid_res <= world.size and 0 <= neighbor[1]*grid_res <= world.size): continue
            if world.check_collision(np.array(neighbor)*grid_res, 0.22): continue

            tg = g_score[current] + h(current, neighbor)
            if neighbor not in g_score or tg < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tg
                heapq.heappush(open_list, (tg + h(neighbor, goal_grid), neighbor))
    return None

def apply_odometry_noise(v, w, dt):
    v_noisy = v * 0.8 + np.random.normal(0, 0.02)
    w_noisy = w * 1.15 + 0.00391974 + np.random.normal(0, 0.05)
    return v_noisy, w_noisy

def send_binary_msg(sock, is_init, update_map, odom_guess, local_scan):
    points_flat = local_scan.flatten().astype(np.float32)
    num_points = len(local_scan)
    header = struct.pack('<BBfffI', 1 if is_init else 0, 1 if update_map else 0,
                         float(odom_guess[0]), float(odom_guess[1]), float(odom_guess[2]), num_points)
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
    SCAN_TIME = 1.0 / LIDAR_HZ # 每圈扫描耗时0.1秒

    try:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect(('127.0.0.1', 9999))
        print("[*] Connected to ICP Server.")
    except ConnectionRefusedError:
        print("[!] ERROR: Start your C server proxy first!")
        sys.exit(1)

    world = MazeRoom(size=5.0)
    start_pose = np.array([3.5, 0.5, np.pi])
    goal_pos = np.array([1.5, 2.5])
    true_pose, icp_pose, dead_reck_pose = start_pose.copy(), start_pose.copy(), start_pose.copy()

    global_path = astar_plan(world, true_pose[:2], goal_pos)
    path_idx, mission_phase = 0, 0

    # 初始帧，速度均为0，无畸变
    local_scan, current_scan_global = world.get_local_lidar_scan(true_pose[:2], true_pose[2], 0.0, 0.0, SCAN_TIME, FEWER_RAYS, LONGER_RANGE)
    send_binary_msg(client, True, True, true_pose, local_scan)
    icp_pose = recv_binary_msg(client)

    hist_true, hist_icp, hist_odom = [], [], []
    dt = 0.1 # 控制周期与雷达周期一致，方便同步
    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))

    t = 0
    while mission_phase < 2:
        if path_idx < len(global_path):
            target = global_path[path_idx]
            dist_to_target = np.linalg.norm(target - true_pose[:2])
            if dist_to_target < 0.2:
                path_idx += 1; v, w = 0.0, 0.0
            else:
                angle_to_target = np.arctan2(target[1] - true_pose[1], target[0] - true_pose[0])
                angle_err = (angle_to_target - true_pose[2] + np.pi) % (2 * np.pi) - np.pi
                if abs(angle_err) > 0.5: v, w = 0.0, np.sign(angle_err) * 0.4
                else: v, w = 0.3, angle_err * 1.5
        else:
            if mission_phase == 0:
                mission_phase = 1; global_path = astar_plan(world, icp_pose[:2], start_pose[:2]); path_idx = 0; continue
            else: mission_phase = 2; v, w = 0.0, 0.0

        # --- 获取带畸变的雷达数据 ---
        # 传入当前的线速度v、角速度w。函数内会向后推演0.1s内发射每个射线的位姿
        local_scan, current_scan_global = world.get_local_lidar_scan(true_pose[:2], true_pose[2], v, w, SCAN_TIME, FEWER_RAYS, LONGER_RANGE)

        # 真实位姿向前推演 dt
        true_pose[2] += w * dt
        true_pose[0] += v * np.cos(true_pose[2]) * dt
        true_pose[1] += v * np.sin(true_pose[2]) * dt

        # 里程计位姿增加噪声
        v_odom, w_odom = apply_odometry_noise(v, w, dt)
        dead_reck_pose[2] += w_odom * dt
        dead_reck_pose[0] += v_odom * np.cos(dead_reck_pose[2]) * dt
        dead_reck_pose[1] += v_odom * np.sin(dead_reck_pose[2]) * dt

        valid_count = np.sum(np.linalg.norm(local_scan, axis=1) > 1e-4)
        print(f"\r[Sim] Frame {t:04d} | Sent 360 points (Valid: {valid_count})", end="")

        odom_guess = icp_pose + [v_odom*np.cos(icp_pose[2])*dt, v_odom*np.sin(icp_pose[2])*dt, w_odom*dt]
        send_binary_msg(client, False, (t % 10 == 0), odom_guess, local_scan)
        resp = recv_binary_msg(client)
        if resp is not None: icp_pose = resp

        if t % 5 == 0:
            ax.cla()
            for p1, p2 in world.wall_segments: ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-', lw=2)

            # 画出带畸变影响的雷达全局点云
            if len(current_scan_global) > 0:
                ax.plot(current_scan_global[:, 0], current_scan_global[:, 1], 'c.', markersize=3, alpha=0.6, label='Distorted LiDAR Scan')

            hist_true.append(true_pose.copy()); hist_icp.append(icp_pose.copy()); hist_odom.append(dead_reck_pose.copy())
            h_true, h_icp, h_odom = np.array(hist_true), np.array(hist_icp), np.array(hist_odom)

            ax.plot(h_true[:,0], h_true[:,1], 'g-', label='Ground Truth')
            ax.plot(h_odom[:,0], h_odom[:,1], 'b--', alpha=0.3, label='Dead Reckoning')
            ax.plot(h_icp[:,0], h_icp[:,1], 'r:', label='ICP Estimated')
            ax.arrow(true_pose[0], true_pose[1], 0.2*np.cos(true_pose[2]), 0.2*np.sin(true_pose[2]), head_width=0.1, color='g')

            ax.set_title(f"ICP + Motion Skew | Valid Points: {valid_count}/360")
            ax.set_xlim(-0.5, 5.5); ax.set_ylim(-0.5, 5.5); ax.set_aspect('equal')
            ax.legend(loc='upper right', fontsize='small')
            plt.pause(0.01)
        t += 1

    final_drift = np.linalg.norm(true_pose[:2] - icp_pose[:2])
    odom_drift = np.linalg.norm(true_pose[:2] - dead_reck_pose[:2])
    print("\n" + "=" * 40)
    print("🎯 Final Report (After Round Trip with Motion Skew):")
    print("=" * 40)
    print(f"ICP Position Error: {final_drift:.4f} m")
    print(f"Pure Odom Error:    {odom_drift:.4f} m")
    print(f"Heading Error:      {np.degrees(abs(true_pose[2]-icp_pose[2]) % (2*np.pi)):.2f} degrees")
    print("=" * 40)

    client.close()
    plt.ioff(); plt.show()

if __name__ == "__main__":
    run_simulation()