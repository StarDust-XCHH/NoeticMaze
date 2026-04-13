import tkinter as tk
from tkinter import scrolledtext, messagebox
import serial
import struct
import threading
import time
import numpy as np
import cv2
from PIL import Image, ImageTk
from collections import deque
import socket
import pickle
import zlib

# --- 协议配置 ---
SERIAL_PORT = 'COM17'
BAUD_RATE = 921600
HEX_SEND_HEADER = 0x5A5A
TYPE_CMD = 0x03
TYPE_ACK = 0x04
TYPE_STATUS = 0x01
TYPE_MAP_ICP = 0x05
TYPE_LIDAR_RAW = 0x02

# --- PID 收敛评估阈值 ---
CONVERGENCE_THRESHOLD = 2.0
ADJUSTING_THRESHOLD = 15.0

# --- 地图与渲染常量 ---
MAP_DIM = 600      # 画布放大到 600x600
MAP_RES = 0.02     # 2cm 分辨率
MAP_OFFSET = 3.5   # 中心偏移
STM32_MAP_SIZE = 250
MAP_PADDING = (MAP_DIM - STM32_MAP_SIZE) // 2

# --- 导航服务端配置 ---
NAV_SERVER_IP = '127.0.0.1'
NAV_SERVER_PORT = 8831

class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 机器人控制台 (点云观测 & 自动导航整合版)")
        self.root.geometry("1280x1020")

        # 状态初始化
        self.grid = np.full((MAP_DIM, MAP_DIM), 127, dtype=np.uint8)
        self.trajectory = deque(maxlen=2000)
        self.map_image_tk = None
        self.lidar_image_tk = None
        self.cached_color_map = cv2.flip(cv2.cvtColor(self.grid, cv2.COLOR_GRAY2RGB), 0)

        # 视图变换
        self.view_zoom = 1.0
        self.view_offset_x = 0.0
        self.view_offset_y = 0.0
        self.view_angle = 0.0
        self.is_panning = False
        self.is_rotating = False

        # 运动状态
        self.target_yaw_rate = 0.0
        self.target_linear_vel = 0.0
        self.last_send_time = 0.0

        # 导航状态
        self.current_pose = (0.0, 0.0, 0.0) # (x, y, theta)
        self.nav_goal = None                # 目标点 (x, y)
        self.auto_nav_enabled = False
        self.nav_path = []                  # 后端传回的规划路径
        self.cost_mask = None               # 后端传回的膨胀地图

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        except Exception as e:
            messagebox.showerror("串口错误", f"无法打开 {SERIAL_PORT}: {e}")
            self.root.destroy()
            return

        self.running = True

        # --- 新增点云点数状态变量 ---
        self.min_point_count = 9999  # 初始设为一个大值
        self.current_point_count = 0

        self.setup_ui()
        self.bind_keys()
        self.bind_mouse_events()
        self.render_map()
        self.root.focus_set()

        # 线程启动
        self.recv_thread = threading.Thread(target=self.receive_handler, daemon=True)
        self.recv_thread.start()
        self.nav_client_thread = threading.Thread(target=self.nav_client_task, daemon=True)
        self.nav_client_thread.start()

    def setup_ui(self):
        # 1. 状态显示区
        status_frame = tk.LabelFrame(self.root, text=" 机器人状态 ", padx=10, pady=10)
        status_frame.pack(fill="x", padx=10, pady=5)

        self.lbl_yaw = tk.Label(status_frame, text="Yaw: 0.00°", font=("Arial", 12, "bold"), fg="blue")
        self.lbl_yaw.grid(row=0, column=0, padx=10, rowspan=2)
        self.lbl_yaw_rate = tk.Label(status_frame, text="实际角速度: 0.00 °/s", font=("Arial", 10, "bold"), fg="darkgreen")
        self.lbl_yaw_rate.grid(row=0, column=1, padx=10, sticky="w")
        self.lbl_pos = tk.Label(status_frame, text="里程计 X: 0.00  Y: 0.00", font=("Arial", 10))
        self.lbl_pos.grid(row=0, column=2, padx=10)
        self.lbl_vel = tk.Label(status_frame, text="线速度: 0.00 m/s", font=("Arial", 10))
        self.lbl_vel.grid(row=1, column=2, padx=10)
        self.lbl_convergence = tk.Label(status_frame, text="收敛状况: 等待数据", font=("Arial", 10))
        self.lbl_convergence.grid(row=1, column=1, padx=10, sticky="w")

        # 2. 双屏布局区
        center_container = tk.Frame(self.root)
        center_container.pack(fill="x", padx=10, pady=5)

        # 左侧：ICP 地图
        map_frame = tk.LabelFrame(center_container, text=" ICP地图 (左平移|右旋转|滚轮缩放|Shift+左键设目标) ", padx=10, pady=5)
        map_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))
        self.lbl_icp_pose = tk.Label(map_frame, text="ICP 位姿 -> X: 0.000, Y: 0.000", font=("Arial", 11, "bold"), fg="purple")
        self.lbl_icp_pose.grid(row=0, column=0, sticky="w")
        self.lbl_nav_status = tk.Label(map_frame, text="导航目标: 未设置 | 状态: 手动", font=("Arial", 10, "bold"), fg="#FF8C00")
        self.lbl_nav_status.grid(row=1, column=0, sticky="w")
        self.map_display = tk.Label(map_frame, bg="black", width=MAP_DIM, height=MAP_DIM, cursor="crosshair")
        self.map_display.grid(row=2, column=0, pady=10)

        # 右侧：点云
        lidar_frame = tk.LabelFrame(center_container, text=" 原始雷达点云 (未补偿畸变) ", padx=10, pady=5)
        lidar_frame.pack(side="right", fill="both", expand=True, padx=(5, 0))
        # 修改：点云信息布局，增加点数显示
        info_sub_frame = tk.Frame(lidar_frame)
        info_sub_frame.grid(row=0, column=0, sticky="w")
        self.lbl_lidar_info = tk.Label(lidar_frame, text="频率: 0.0 Hz", font=("Arial", 11, "bold"), fg="#FF8C00")
        self.lbl_lidar_info.grid(row=0, column=0, sticky="w")

        # 新增：显示当前点数和记录的最小点数
        self.lbl_points_info = tk.Label(info_sub_frame, text="当前点数: 0 | 最小: 0", font=("Arial", 10, "bold"), fg="red")
        self.lbl_points_info.pack(side="left", padx=20)

        self.lidar_display = tk.Label(lidar_frame, bg="#111111", width=MAP_DIM, height=MAP_DIM)
        self.lidar_display.grid(row=2, column=0, pady=10)

        # 3. 控制区
        control_frame = tk.LabelFrame(self.root, text=" 导航与手动控制 ", padx=10, pady=5)
        control_frame.pack(fill="x", padx=10, pady=5)
        self.btn_auto_nav = tk.Button(control_frame, text="开启自动导航", bg="lightgray", width=15, command=self.toggle_auto_nav)
        self.btn_auto_nav.grid(row=0, column=0, padx=10)
        self.lbl_cmd_yaw = tk.Label(control_frame, text="下发角速: 0 °/s", font=("Arial", 10))
        self.lbl_cmd_yaw.grid(row=0, column=1, padx=10)
        self.lbl_cmd_vel = tk.Label(control_frame, text="下发线速: 0.00 m/s", font=("Arial", 10))
        self.lbl_cmd_vel.grid(row=0, column=2, padx=10)

        # 4. 日志
        self.log_area = scrolledtext.ScrolledText(self.root, state='disabled', height=6)
        self.log_area.pack(fill="both", expand=True, padx=10, pady=5)

    # --- 导航交互逻辑 ---
    def toggle_auto_nav(self):
        if not self.auto_nav_enabled:
            if self.nav_goal is None:
                messagebox.showwarning("提示", "请先按住 Shift+左键 设定目标点")
                return
            self.auto_nav_enabled = True
            self.btn_auto_nav.config(text="停止自动导航", bg="red", fg="white")
            self.write_log(">>> 自动导航模式开启", "green")
        else:
            self.auto_nav_enabled = False
            self.nav_path = []
            self.cost_mask = None
            self.btn_auto_nav.config(text="开启自动导航", bg="lightgray", fg="black")
            self.stop_robot()
            self.render_map()
            self.write_log(">>> 自动导航模式关闭")
        self.update_nav_ui()

    def update_nav_ui(self):
        goal_str = f"({self.nav_goal[0]:.2f}, {self.nav_goal[1]:.2f})" if self.nav_goal else "未设置"
        status_str = "自动导航中" if self.auto_nav_enabled else "手动控制"
        self.lbl_nav_status.config(text=f"目标: {goal_str} | 状态: {status_str}")

    def set_nav_goal(self, event):
        center = (MAP_DIM // 2, MAP_DIM // 2)
        M = cv2.getRotationMatrix2D(center, self.view_angle, self.view_zoom)
        M[0, 2] += self.view_offset_x
        M[1, 2] += self.view_offset_y
        inv_M = cv2.invertAffineTransform(M)
        pt_screen = np.array([event.x, event.y, 1.0])
        pt_map = inv_M.dot(pt_screen)
        px_x, px_y = pt_map[0], MAP_DIM - pt_map[1]
        phys_x = px_x * MAP_RES - MAP_OFFSET
        phys_y = px_y * MAP_RES - MAP_OFFSET
        self.nav_goal = (phys_x, phys_y)
        self.write_log(f"设置目标点: X={phys_x:.2f}, Y={phys_y:.2f}")

        if not self.auto_nav_enabled:
            self.auto_nav_enabled = True
            self.btn_auto_nav.config(text="停止自动导航", bg="red", fg="white")
            self.write_log(">>> 自动导航模式开启", "green")

        self.update_nav_ui()
        self.render_map()

    # --- 视觉与渲染 ---
    def bind_mouse_events(self):
        self.map_display.bind("<MouseWheel>", self.on_mouse_wheel)
        self.map_display.bind("<ButtonPress-1>", self.on_pan_start)
        self.map_display.bind("<B1-Motion>", self.on_pan_drag)
        self.map_display.bind("<ButtonRelease-1>", self.on_pan_end)
        self.map_display.bind("<ButtonPress-3>", self.on_rot_start)
        self.map_display.bind("<B3-Motion>", self.on_rot_drag)
        self.map_display.bind("<ButtonRelease-3>", self.on_rot_end)
        self.map_display.bind("<Double-Button-1>", self.reset_view)
        self.map_display.bind("<Shift-ButtonPress-1>", self.set_nav_goal)

    def on_mouse_wheel(self, event):
        if event.delta > 0: self.view_zoom *= 1.15
        else: self.view_zoom /= 1.15
        self.render_map()

    def on_pan_start(self, event): self.is_panning = True; self.pan_start_x, self.pan_start_y = event.x, event.y
    def on_pan_drag(self, event):
        if self.is_panning:
            self.view_offset_x += event.x - self.pan_start_x
            self.view_offset_y += event.y - self.pan_start_y
            self.pan_start_x, self.pan_start_y = event.x, event.y
            self.render_map()
    def on_pan_end(self, event): self.is_panning = False
    def on_rot_start(self, event): self.is_rotating = True; self.rot_start_x = event.x
    def on_rot_drag(self, event):
        if self.is_rotating:
            self.view_angle -= (event.x - self.rot_start_x) * 0.5
            self.rot_start_x = event.x
            self.render_map()
    def on_rot_end(self, event): self.is_rotating = False
    def reset_view(self, event=None):
        self.view_zoom, self.view_offset_x, self.view_offset_y, self.view_angle = 1.0, 0.0, 0.0, 0.0
        self.render_map()

    def render_map(self):
        if self.cached_color_map is None: return
        display_map = self.cached_color_map.copy()

        # =======================
        # 新增：直接渲染导航层数据
        # =======================
        # 1. 渲染膨胀地图层
        if self.cost_mask is not None:
            # 必须和 cached_color_map 一起翻转Y轴
            flipped_mask = cv2.flip(self.cost_mask, 0)
            # 深红色标记危险区域 [B, G, R] = [34, 34, 102]
            display_map[flipped_mask == 1] = [102, 34, 34]

            # 2. 渲染规划好的路径
        if self.nav_path and len(self.nav_path) > 1:
            pts = []
            for p in self.nav_path:
                px = int((p[0] + MAP_OFFSET) / MAP_RES)
                py = int((p[1] + MAP_OFFSET) / MAP_RES)
                # 镜像翻转 Y 轴匹配当前显示层
                flipped_py = MAP_DIM - py
                pts.append([px, flipped_py])
            pts = np.array(pts, np.int32).reshape((-1, 1, 2))
            cv2.polylines(display_map, [pts], False, (0, 255, 0), 2)

        # 3. 渲染目标点 (Target Goal)
        if self.nav_goal:
            gx_px = int((self.nav_goal[0] + MAP_OFFSET) / MAP_RES)
            gy_px = int((self.nav_goal[1] + MAP_OFFSET) / MAP_RES)
            gy_px_flipped = MAP_DIM - gy_px
            cv2.drawMarker(display_map, (gx_px, gy_px_flipped), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=15, thickness=2)

        # 视角仿射变换输出
        center = (MAP_DIM // 2, MAP_DIM // 2)
        M = cv2.getRotationMatrix2D(center, self.view_angle, self.view_zoom)
        M[0, 2] += self.view_offset_x
        M[1, 2] += self.view_offset_y
        transformed = cv2.warpAffine(display_map, M, (MAP_DIM, MAP_DIM), borderValue=(127, 127, 127))
        img = Image.fromarray(transformed)
        self.map_image_tk = ImageTk.PhotoImage(image=img)
        self.map_display.config(image=self.map_image_tk)

    # --- 核心解析逻辑 ---
    def receive_handler(self):
        buffer = b""
        while self.running:
            if self.ser.in_waiting > 0:
                buffer += self.ser.read(self.ser.in_waiting)
                if len(buffer) > 20000: buffer = buffer[-20000:]

                while len(buffer) >= 6:
                    idx_55aa = buffer.find(b'\xAA\x55')
                    idx_5a5a = buffer.find(b'\x5A\x5A')
                    indices = [i for i in [idx_55aa, idx_5a5a] if i != -1]
                    if not indices: buffer = buffer[-1:]; break

                    first_idx = min(indices)
                    if first_idx > 0: buffer = buffer[first_idx:]; continue

                    pkg_type = buffer[2]
                    # 状态包
                    if pkg_type == TYPE_STATUS and buffer.startswith(b'\xAA\x55'):
                        if len(buffer) < 32: break
                        d = struct.unpack('<Iffffff', buffer[3:31])
                        self.root.after(0, self.update_status_ui, d[4], d[5], d[6], d[1], d[2], d[3], d[0])
                        buffer = buffer[32:]
                    # 指令回显
                    elif pkg_type == TYPE_ACK and buffer.startswith(b'\x5A\x5A'):
                        if len(buffer) < 12: break
                        buffer = buffer[12:]
                    # 原始点云
                    elif pkg_type == TYPE_LIDAR_RAW and buffer.startswith(b'\xAA\x55'):
                        # 长度从 728 改为 732 (Header2 + Type1 + Dist720 + Time4 + Count4 + Sum1)
                        if len(buffer) < 732: break

                        # 校验和范围增加 4 字节点数 (从 index 3 到 731)
                        if (sum(buffer[3:731]) & 0xFF) == buffer[731]:
                            dists = struct.unpack('<360H', buffer[3:723])
                            st = struct.unpack('<f', buffer[723:727])[0]
                            pc = struct.unpack('<I', buffer[727:731])[0] # 解析点数 (uint32)

                            self.root.after(0, self.update_lidar_ui, dists, st, pc)
                        buffer = buffer[732:]
                    # ICP增量地图
                    elif pkg_type == TYPE_MAP_ICP and buffer.startswith(b'\xAA\x55'):
                        if len(buffer) < 5: break
                        d_len = struct.unpack('<H', buffer[3:5])[0]
                        total = 5 + d_len + 1
                        if len(buffer) < total: break
                        try:
                            ix, iy, it, dc = struct.unpack('<fffH', buffer[5:19])
                            pts = np.frombuffer(buffer[19:19+dc*3], dtype=np.uint8).reshape(-1, 3) if dc > 0 else np.array([])
                            self.root.after(0, self.update_map_ui, ix, iy, it, dc, d_len, pts)
                        except: pass
                        buffer = buffer[total:]
                    else: buffer = buffer[2:]
            time.sleep(0.001)


    def update_status_ui(self, yaw, yaw_rate, target_yaw_rate, x, y, vel, sweep):
        self.lbl_yaw.config(text=f"Yaw: {yaw:.2f}°")
        self.lbl_yaw_rate.config(text=f"实际角速: {yaw_rate:.2f} °/s")
        self.lbl_pos.config(text=f"里程 X: {x:.2f} Y: {y:.2f}")
        self.lbl_vel.config(text=f"线速: {vel:.2f} m/s")
        err = abs(target_yaw_rate - yaw_rate)
        c_text = "✅ 已收敛" if err < CONVERGENCE_THRESHOLD else "⚠️ 调节中"
        self.lbl_convergence.config(text=f"收敛状况: {c_text}", fg=("green" if err < CONVERGENCE_THRESHOLD else "orange"))


    # --- UI 更新函数 ---
    def update_lidar_ui(self, distances, scan_time, point_count):
        img = np.zeros((MAP_DIM, MAP_DIM, 3), dtype=np.uint8)
        center = MAP_DIM // 2
        max_mm = 6000.0
        scale = (MAP_DIM / 2) / max_mm

        d_arr = np.array(distances)
        ang = np.deg2rad(np.arange(360))
        mask = (d_arr > 30) & (d_arr < max_mm)
        rx = d_arr[mask] * np.cos(ang[mask])
        ry = d_arr[mask] * np.sin(ang[mask])
        ix = (center - ry * scale).astype(np.int32)
        iy = (center - rx * scale).astype(np.int32)
        valid = (ix>=0) & (ix<MAP_DIM) & (iy>=0) & (iy<MAP_DIM)
        img[iy[valid], ix[valid]] = (0, 255, 0)
        cv2.circle(img, (center, center), 3, (255,0,0), -1)

        pil_img = Image.fromarray(img)
        self.lidar_image_tk = ImageTk.PhotoImage(image=pil_img)
        self.lidar_display.config(image=self.lidar_image_tk)
        self.lbl_lidar_info.config(text=f"耗时: {scan_time:.4f}s | 频率: {(1/scan_time if scan_time>0 else 0):.1f}Hz")

        # 逻辑：更新最小点数记录（跳过开机可能不准的前几帧）
        if point_count > 0:
            if point_count < self.min_point_count:
                self.min_point_count = point_count

            # 变色提醒：如果当前点数掉得厉害，显示红色
            color = "red" if point_count < (self.min_point_count * 0.95) else "darkgreen"
            self.lbl_points_info.config(
                text=f"当前点数: {point_count} | 最小: {self.min_point_count}",
                fg=color
            )

    def update_map_ui(self, icp_x, icp_y, icp_theta, diff_count, payload_len, diff_pts):
        self.current_pose = (icp_x, icp_y, icp_theta)
        self.lbl_icp_pose.config(text=f"ICP X: {icp_x:.3f}, Y: {icp_y:.3f}, T: {icp_theta:.2f}")

        rgx = int((icp_x + MAP_OFFSET) / MAP_RES)
        rgy = int((icp_y + MAP_OFFSET) / MAP_RES)
        self.trajectory.append((rgx, rgy))

        if len(diff_pts) > 0:
            xs = diff_pts[:, 0].astype(np.int32)
            ys = diff_pts[:, 1].astype(np.int32)
            states = diff_pts[:, 2]

            xs += MAP_PADDING
            ys += MAP_PADDING

            valid = (xs >= 0) & (xs < MAP_DIM) & (ys >= 0) & (ys < MAP_DIM)
            xs, ys, states = xs[valid], ys[valid], states[valid]

            ray_start_x = int((icp_x + MAP_OFFSET) / MAP_RES)
            ray_start_y = int((icp_y + MAP_OFFSET) / MAP_RES)
            ray_mask = np.zeros_like(self.grid)

            for ox, oy in zip(xs, ys):
                cv2.line(ray_mask, (ray_start_x, ray_start_y), (ox, oy), 1, thickness=2)

            is_ray_path = (ray_mask == 1)
            valid_free_update = is_ray_path & (self.grid == 127)
            self.grid[valid_free_update] = 40

            obs_mask = (states == 2)
            if np.any(obs_mask):
                self.grid[ys[obs_mask], xs[obs_mask]] = 255

            free_mask = (states == 1)
            current_vals = self.grid[ys, xs]
            update_free_mask = free_mask & (current_vals == 127)

            if np.any(update_free_mask):
                self.grid[ys[update_free_mask], xs[update_free_mask]] = 40

        color_map = cv2.cvtColor(self.grid, cv2.COLOR_GRAY2RGB)

        if len(self.trajectory) > 1:
            pts = np.array(self.trajectory, np.int32).reshape((-1, 1, 2))
            cv2.polylines(color_map, [pts], False, (0, 100, 255), 1)

        if len(self.trajectory) > 0:
            cv2.circle(color_map, (rgx, rgy), 3, (255, 0, 0), -1)
            ltx = int(rgx + 8 * np.cos(icp_theta))
            lty = int(rgy + 8 * np.sin(icp_theta))
            cv2.line(color_map, (rgx, rgy), (ltx, lty), (0, 255, 0), 2)

        self.cached_color_map = cv2.flip(color_map, 0)
        self.render_map()

    # --- 网络与控制 ---
    def nav_client_task(self):
        while self.running:
            if self.auto_nav_enabled and self.nav_goal:
                try:
                    with socket.create_connection((NAV_SERVER_IP, NAV_SERVER_PORT), timeout=2.0) as s:
                        print("[+] 成功连接到导航服务端！")

                        while self.auto_nav_enabled and self.running:
                            data = {'pose': self.current_pose, 'goal': self.nav_goal, 'map': self.grid}
                            payload = zlib.compress(pickle.dumps(data))
                            s.sendall(struct.pack('<I', len(payload)) + payload)

                            header = s.recv(4)
                            if not header:
                                print("[-] 服务端主动断开了连接")
                                break

                            resp_len = struct.unpack('<I', header)[0]
                            resp_data = bytearray()
                            while len(resp_data) < resp_len:
                                packet = s.recv(resp_len - len(resp_data))
                                if not packet:
                                    break
                                resp_data.extend(packet)

                            if len(resp_data) == resp_len:
                                # 注意这里新增了 zlib 解压逻辑
                                resp = pickle.loads(zlib.decompress(resp_data))
                                self.target_linear_vel = round(resp.get('v', 0.0), 2)
                                self.target_yaw_rate = round(resp.get('w', 0.0), 1)

                                # 更新导航数据并强制触发画面重绘
                                self.nav_path = resp.get('path', [])
                                self.cost_mask = resp.get('cost_mask', None)

                                self.root.after(0, self.update_cmd_ui)
                                self.root.after(0, self.render_map)
                                self.send_command()

                            time.sleep(0.1)

                except ConnectionRefusedError:
                    print("[!] 无法连接到导航服务端: 目标计算机拒绝连接 (服务端没开？)")
                    time.sleep(1.0)
                except Exception as e:
                    print(f"[!] 导航网络通信异常: {e}")
                    time.sleep(1.0)
            else:
                time.sleep(0.2)

    def change_speed(self, v_delta=0.0, w_delta=0.0):
        if self.auto_nav_enabled: return
        self.target_linear_vel = max(-0.1, min(0.1, round(self.target_linear_vel + v_delta, 2)))
        self.target_yaw_rate = max(-360.0, min(360.0, round(self.target_yaw_rate + w_delta, 1)))
        self.update_cmd_ui()
        self.send_command()

    def stop_robot(self):
        if self.auto_nav_enabled:
            self.auto_nav_enabled = False
            self.nav_path = []
            self.cost_mask = None
            self.btn_auto_nav.config(text="开启自动导航", bg="lightgray", fg="black")
            self.write_log(">>> 紧急停止：自动导航模式已关闭", "red")
            self.update_nav_ui()

        self.target_linear_vel, self.target_yaw_rate = 0.0, 0.0
        self.update_cmd_ui()
        self.send_command()

    def update_cmd_ui(self):
        self.lbl_cmd_yaw.config(text=f"下发角速: {self.target_yaw_rate} °/s")
        self.lbl_cmd_vel.config(text=f"下发线速: {self.target_linear_vel:.2f} m/s")

    def send_command(self):
        try:
            p = struct.pack('<HBff', HEX_SEND_HEADER, TYPE_CMD, self.target_yaw_rate, self.target_linear_vel)
            self.ser.write(p + struct.pack('<B', sum(p) & 0xFF))
        except: pass

    def write_log(self, msg, color="black"):
        self.log_area.configure(state='normal')
        self.log_area.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {msg}\n", color)
        self.log_area.see(tk.END)
        self.log_area.configure(state='disabled')

    def bind_keys(self):
        self.root.bind('<w>', lambda e: self.change_speed(v_delta=0.02))
        self.root.bind('<s>', lambda e: self.change_speed(v_delta=-0.02))
        self.root.bind('<a>', lambda e: self.change_speed(w_delta=10.0))
        self.root.bind('<d>', lambda e: self.change_speed(w_delta=-10.0))
        self.root.bind('<space>', lambda e: self.stop_robot())

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotGUI(root)
    root.mainloop()