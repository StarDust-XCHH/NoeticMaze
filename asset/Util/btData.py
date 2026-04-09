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

# --- 协议配置 ---
SERIAL_PORT = 'COM17'
BAUD_RATE = 921600
HEX_SEND_HEADER = 0x5A5A
TYPE_CMD = 0x03
TYPE_ACK = 0x04
TYPE_STATUS = 0x01
TYPE_MAP_ICP = 0x05
TYPE_LIDAR_RAW = 0x02  # 【新增】原始雷达帧协议类型

# --- PID 收敛评估阈值 ---
CONVERGENCE_THRESHOLD = 2.0
ADJUSTING_THRESHOLD = 15.0

# --- 地图与渲染常量 ---
MAP_DIM = 600      # 画布放大到 600x600
MAP_RES = 0.02     # 2cm 分辨率
MAP_OFFSET = 3.5   # 中心偏移
STM32_MAP_SIZE = 250
MAP_PADDING = (MAP_DIM - STM32_MAP_SIZE) // 2  # 计算布局偏移: 175

class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 机器人控制台 (点云畸变观测 & ICP交互版)")
        # 【修改】大幅加宽窗口以容纳双屏并排显示
        self.root.geometry("1280x1020")

        # 地图渲染状态初始化
        self.grid = np.full((MAP_DIM, MAP_DIM), 127, dtype=np.uint8)
        self.trajectory = deque(maxlen=2000)
        self.map_image_tk = None
        self.lidar_image_tk = None # 【新增】原始雷达图像缓冲

        # 缓存渲染的基础地图，用于解耦数据更新和鼠标交互重绘
        self.cached_color_map = cv2.flip(cv2.cvtColor(self.grid, cv2.COLOR_GRAY2RGB), 0)

        # 视图变换状态 (缩放、平移、旋转)
        self.view_zoom = 1.0
        self.view_offset_x = 0.0
        self.view_offset_y = 0.0
        self.view_angle = 0.0

        self.is_panning = False
        self.is_rotating = False
        self.pan_start_x = 0
        self.pan_start_y = 0
        self.rot_start_x = 0

        # 目标速度变量与频率控制变量
        self.target_yaw_rate = 0.0
        self.target_linear_vel = 0.0
        self.last_send_time = 0.0

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        except Exception as e:
            messagebox.showerror("串口错误", f"无法打开 {SERIAL_PORT}: {e}")
            self.root.destroy()
            return

        self.running = True
        self.setup_ui()
        self.bind_keys()
        self.bind_mouse_events()

        # 初始化渲染第一帧
        self.render_map()

        self.root.focus_set()

        self.recv_thread = threading.Thread(target=self.receive_handler, daemon=True)
        self.recv_thread.start()

    def setup_ui(self):
        # 1. 状态显示区
        status_frame = tk.LabelFrame(self.root, text=" 机器人状态 (0x55AA - TYPE: 0x01) ", padx=10, pady=10)
        status_frame.pack(fill="x", padx=10, pady=5)

        self.lbl_yaw = tk.Label(status_frame, text="Yaw: 0.00°", font=("Arial", 12, "bold"), fg="blue")
        self.lbl_yaw.grid(row=0, column=0, padx=10, rowspan=2)

        self.lbl_yaw_rate = tk.Label(status_frame, text="实际角速度: 0.00 °/s", font=("Arial", 10, "bold"), fg="darkgreen")
        self.lbl_yaw_rate.grid(row=0, column=1, padx=10, sticky="w")

        self.lbl_target_rate = tk.Label(status_frame, text="预期角速度: 0.00 °/s", font=("Arial", 10), fg="gray")
        self.lbl_target_rate.grid(row=1, column=1, padx=10, sticky="w")

        self.lbl_error = tk.Label(status_frame, text="偏差(Error): 0.00 °/s", font=("Arial", 10, "bold"), fg="red")
        self.lbl_error.grid(row=2, column=1, padx=10, sticky="w", pady=(5, 0))

        self.lbl_convergence = tk.Label(status_frame, text="收敛状况: 等待数据", font=("Arial", 10, "bold"), fg="black")
        self.lbl_convergence.grid(row=3, column=1, padx=10, sticky="w")

        self.lbl_pos = tk.Label(status_frame, text="里程计 X: 0.00  Y: 0.00", font=("Arial", 10))
        self.lbl_pos.grid(row=0, column=2, padx=10)

        self.lbl_vel = tk.Label(status_frame, text="线速度: 0.00 m/s", font=("Arial", 10))
        self.lbl_vel.grid(row=1, column=2, padx=10)

        self.lbl_sweep = tk.Label(status_frame, text="Sweep: 0", font=("Arial", 10))
        self.lbl_sweep.grid(row=0, column=3, padx=10)

        # =========================================================================
        # 【新增】双屏并排布局区
        center_container = tk.Frame(self.root)
        center_container.pack(fill="x", padx=10, pady=5)

        # 左侧：ICP 累积地图面板
        map_title = " ICP位姿与地图增量 (左键平移 | 右键旋转 | 滚轮缩放) "
        map_frame = tk.LabelFrame(center_container, text=map_title, padx=10, pady=5)
        map_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        self.lbl_icp_pose = tk.Label(map_frame, text="ICP 位姿 -> X: 0.000, Y: 0.000, Theta: 0.000", font=("Arial", 11, "bold"), fg="purple")
        self.lbl_icp_pose.grid(row=0, column=0, padx=10, sticky="w")

        self.lbl_map_count = tk.Label(map_frame, text="最新接收栅格数: 0 (共 0 字节)", font=("Arial", 10))
        self.lbl_map_count.grid(row=1, column=0, padx=10, sticky="w")

        self.map_display = tk.Label(map_frame, bg="black", width=MAP_DIM, height=MAP_DIM, cursor="crosshair")
        self.map_display.grid(row=2, column=0, pady=10, padx=10)

        # 右侧：原始雷达点云面板
        lidar_frame = tk.LabelFrame(center_container, text=" 原始雷达点云观测 (TYPE: 0x02) ", padx=10, pady=5)
        lidar_frame.pack(side="right", fill="both", expand=True, padx=(5, 0))

        self.lbl_lidar_info = tk.Label(lidar_frame, text="物理扫描耗时: 0.0000 s | 真实频率: 0.0 Hz", font=("Arial", 11, "bold"), fg="#FF8C00")
        self.lbl_lidar_info.grid(row=0, column=0, padx=10, sticky="w")

        self.lbl_lidar_desc = tk.Label(lidar_frame, text="本面板展示未经过畸变补偿的纯原始坐标极点", font=("Arial", 10), fg="gray")
        self.lbl_lidar_desc.grid(row=1, column=0, padx=10, sticky="w")

        self.lidar_display = tk.Label(lidar_frame, bg="#111111", width=MAP_DIM, height=MAP_DIM)
        self.lidar_display.grid(row=2, column=0, pady=10, padx=10)
        # =========================================================================

        # 3. 键盘控制区
        control_frame = tk.LabelFrame(self.root, text=" 键盘控制区 (请保持英文输入法) ", padx=10, pady=5)
        control_frame.pack(fill="x", padx=10, pady=5)

        instruction_text = "操作说明: [W]线速度+0.02 | [S]线速度-0.02 | [A]角速度+10 | [D]角速度-10 | [空格]紧急停止"
        tk.Label(control_frame, text=instruction_text, font=("Arial", 10, "bold")).grid(row=0, column=0, columnspan=4, pady=5)

        tk.Label(control_frame, text="下发角速度:").grid(row=1, column=0, sticky="e", padx=(20, 0))
        self.lbl_cmd_yaw = tk.Label(control_frame, text="0 °/s", font=("Arial", 12, "bold"), fg="blue", width=8, anchor="w")
        self.lbl_cmd_yaw.grid(row=1, column=1, sticky="w")

        tk.Label(control_frame, text="下发线速度:").grid(row=1, column=2, sticky="e", padx=(20, 0))
        self.lbl_cmd_vel = tk.Label(control_frame, text="0.00 m/s", font=("Arial", 12, "bold"), fg="blue", width=8, anchor="w")
        self.lbl_cmd_vel.grid(row=1, column=3, sticky="w")

        # 4. 日志回显区
        log_frame = tk.LabelFrame(self.root, text=" 系统日志 ")
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.log_area = scrolledtext.ScrolledText(log_frame, state='disabled', height=5)
        self.log_area.pack(fill="both", expand=True)

    # ========================== 鼠标交互事件 (仅针对左侧地图) ==========================
    def bind_mouse_events(self):
        self.map_display.bind("<MouseWheel>", self.on_mouse_wheel)
        self.map_display.bind("<Button-4>", self.on_mouse_wheel)
        self.map_display.bind("<Button-5>", self.on_mouse_wheel)

        self.map_display.bind("<ButtonPress-1>", self.on_pan_start)
        self.map_display.bind("<B1-Motion>", self.on_pan_drag)
        self.map_display.bind("<ButtonRelease-1>", self.on_pan_end)

        self.map_display.bind("<ButtonPress-3>", self.on_rot_start)
        self.map_display.bind("<B3-Motion>", self.on_rot_drag)
        self.map_display.bind("<ButtonRelease-3>", self.on_rot_end)

        self.map_display.bind("<Double-Button-1>", self.reset_view)

    def on_mouse_wheel(self, event):
        if event.delta > 0 or getattr(event, 'num', 0) == 4:
            self.view_zoom *= 1.15
        elif event.delta < 0 or getattr(event, 'num', 0) == 5:
            self.view_zoom /= 1.15
        self.render_map()

    def on_pan_start(self, event):
        self.is_panning = True
        self.pan_start_x = event.x
        self.pan_start_y = event.y

    def on_pan_drag(self, event):
        if self.is_panning:
            dx = event.x - self.pan_start_x
            dy = event.y - self.pan_start_y
            self.view_offset_x += dx
            self.view_offset_y += dy
            self.pan_start_x = event.x
            self.pan_start_y = event.y
            self.render_map()

    def on_pan_end(self, event):
        self.is_panning = False

    def on_rot_start(self, event):
        self.is_rotating = True
        self.rot_start_x = event.x

    def on_rot_drag(self, event):
        if self.is_rotating:
            dx = event.x - self.rot_start_x
            self.view_angle -= dx * 0.5
            self.rot_start_x = event.x
            self.render_map()

    def on_rot_end(self, event):
        self.is_rotating = False

    def reset_view(self, event=None):
        self.view_zoom = 1.0
        self.view_offset_x = 0.0
        self.view_offset_y = 0.0
        self.view_angle = 0.0
        self.render_map()

    def render_map(self):
        if self.cached_color_map is None:
            return

        center = (MAP_DIM // 2, MAP_DIM // 2)
        M = cv2.getRotationMatrix2D(center, self.view_angle, self.view_zoom)

        M[0, 2] += self.view_offset_x
        M[1, 2] += self.view_offset_y

        transformed_map = cv2.warpAffine(self.cached_color_map, M, (MAP_DIM, MAP_DIM), borderValue=(127, 127, 127))

        img = Image.fromarray(transformed_map)
        self.map_image_tk = ImageTk.PhotoImage(image=img)
        self.map_display.config(image=self.map_image_tk)
    # =========================================================================

    def bind_keys(self):
        self.root.bind('<KeyPress-w>', lambda e: self.change_speed(v_delta=0.02))
        self.root.bind('<KeyPress-W>', lambda e: self.change_speed(v_delta=0.02))
        self.root.bind('<KeyPress-s>', lambda e: self.change_speed(v_delta=-0.02))
        self.root.bind('<KeyPress-S>', lambda e: self.change_speed(v_delta=-0.02))

        self.root.bind('<KeyPress-a>', lambda e: self.change_speed(w_delta=10.0))
        self.root.bind('<KeyPress-A>', lambda e: self.change_speed(w_delta=10.0))
        self.root.bind('<KeyPress-d>', lambda e: self.change_speed(w_delta=-10.0))
        self.root.bind('<KeyPress-D>', lambda e: self.change_speed(w_delta=-10.0))

        self.root.bind('<space>', lambda e: self.stop_robot())

    def change_speed(self, v_delta=0.0, w_delta=0.0):
        current_time = time.time()
        if current_time - self.last_send_time < 0.05:
            return

        new_v = round(self.target_linear_vel + v_delta, 2)
        new_w = round(self.target_yaw_rate + w_delta, 1)

        new_v = max(-0.1, min(0.1, new_v))
        new_w = max(-360.0, min(360.0, new_w))

        if new_v != self.target_linear_vel or new_w != self.target_yaw_rate:
            self.target_linear_vel = new_v
            self.target_yaw_rate = new_w
            self.update_cmd_ui()
            self.send_command()
            self.last_send_time = current_time

    def stop_robot(self):
        if self.target_linear_vel != 0.0 or self.target_yaw_rate != 0.0:
            self.target_linear_vel = 0.0
            self.target_yaw_rate = 0.0
            self.update_cmd_ui()
            self.send_command()
            self.write_log("🚨 已触发急停: 速度归零", color="red")

    def update_cmd_ui(self):
        self.lbl_cmd_yaw.config(text=f"{self.target_yaw_rate} °/s")
        self.lbl_cmd_vel.config(text=f"{self.target_linear_vel:.2f} m/s")

    def write_log(self, msg, color="black"):
        self.log_area.configure(state='normal')
        self.log_area.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        self.log_area.see(tk.END)
        self.log_area.configure(state='disabled')

    def send_command(self):
        try:
            yaw_rate = self.target_yaw_rate
            linear_vel = self.target_linear_vel
            packet = struct.pack('<HBff', HEX_SEND_HEADER, TYPE_CMD, yaw_rate, linear_vel)
            checksum = sum(packet) & 0xFF
            full_packet = packet + struct.pack('<B', checksum)
            self.ser.write(full_packet)
            self.write_log(f"发送指令 -> 角速度: {yaw_rate}°/s, 线速度: {linear_vel} m/s")
        except Exception as e:
            self.write_log(f"发送串口错误: {e}")

    # ========================== 核心数据接收与分发 ==========================
    def receive_handler(self):
        buffer = b""
        while self.running:
            if self.ser.in_waiting > 0:
                buffer += self.ser.read(self.ser.in_waiting)

                if len(buffer) > 16384:
                    buffer = buffer[-16384:]

                while len(buffer) >= 6:
                    idx_55aa = buffer.find(b'\xAA\x55')
                    idx_5a5a = buffer.find(b'\x5A\x5A')

                    indices = [i for i in [idx_55aa, idx_5a5a] if i != -1]
                    if not indices:
                        buffer = buffer[-1:]
                        break

                    first_idx = min(indices)
                    if first_idx > 0:
                        buffer = buffer[first_idx:]
                        continue

                    pkg_type = buffer[2]

                    if pkg_type == TYPE_STATUS and buffer.startswith(b'\xAA\x55'):
                        if len(buffer) < 32: break
                        try:
                            payload = buffer[3:31]
                            d = struct.unpack('<Iffffff', payload)
                            self.root.after(0, self.update_status_ui, d[4], d[5], d[6], d[1], d[2], d[3], d[0])
                        except Exception as e:
                            pass
                        buffer = buffer[32:]

                    elif pkg_type == TYPE_ACK and buffer.startswith(b'\x5A\x5A'):
                        if len(buffer) < 12: break
                        try:
                            _, _, r_yaw_rate, r_linear_vel, _ = struct.unpack('<HBffB', buffer[:12])
                            self.root.after(0, self.write_log, f"收到回显 ✔: 角速度={r_yaw_rate:.1f}°/s, 线速度={r_linear_vel:.2f}m/s")
                        except Exception as e:
                            pass
                        buffer = buffer[12:]

                    # 【新增】解析原始雷达数据 (728 字节新协议)
                    elif pkg_type == TYPE_LIDAR_RAW and buffer.startswith(b'\xAA\x55'):
                        if len(buffer) < 728:
                            break

                        # Checksum 计算范围：Payload (360个uint16 + 1个float)
                        expected_chk = sum(buffer[3:727]) & 0xFF
                        actual_chk = buffer[727]

                        if expected_chk == actual_chk:
                            try:
                                # 解析 360 个距离数据 (720 字节) 和 1 个耗时数据 (4 字节)
                                distances = struct.unpack('<360H', buffer[3:723])
                                scan_time = struct.unpack('<f', buffer[723:727])[0]
                                self.root.after(0, self.update_lidar_ui, distances, scan_time)
                            except Exception as e:
                                print(f"Raw Lidar Parse Error: {e}")

                        buffer = buffer[728:]

                    # 解析 ICP 增量地图 (保留不变)
                    elif pkg_type == TYPE_MAP_ICP and buffer.startswith(b'\xAA\x55'):
                        if len(buffer) < 5: break
                        data_len = struct.unpack('<H', buffer[3:5])[0]
                        total_tx_size = 5 + data_len + 1

                        if len(buffer) < total_tx_size:
                            break

                        try:
                            payload_head = buffer[5 : 5+14]
                            icp_x, icp_y, icp_theta, diff_count = struct.unpack('<fffH', payload_head)

                            if diff_count > 0:
                                diff_bytes = buffer[19 : 19 + diff_count*3]
                                diff_pts = np.frombuffer(diff_bytes, dtype=np.uint8).reshape(-1, 3)
                            else:
                                diff_pts = np.array([])

                            self.root.after(0, self.update_map_ui, icp_x, icp_y, icp_theta, diff_count, data_len, diff_pts)

                        except Exception as e:
                            pass

                        buffer = buffer[total_tx_size:]

                    else:
                        buffer = buffer[2:]

            time.sleep(0.002)

    # ========================== 【新增】原始雷达点云极速渲染 ==========================
    def update_lidar_ui(self, distances, scan_time):
        # 1. 更新顶部文字信息
        freq = (1.0 / scan_time) if scan_time > 0.001 else 0.0
        self.lbl_lidar_info.config(text=f"物理扫描耗时: {scan_time:.4f} s | 真实频率: {freq:.1f} Hz")

        # 2. 准备黑底画布
        img = np.zeros((MAP_DIM, MAP_DIM, 3), dtype=np.uint8)
        center = MAP_DIM // 2

        # 3. 绘制同心圆距离参考线 (1米~5米)
        max_dist_mm = 6000.0   # 设置画布边缘对应最大量程 6 米
        scale = (MAP_DIM / 2) / max_dist_mm
        for r_m in range(1, 6):
            r_px = int(r_m * 1000 * scale)
            cv2.circle(img, (center, center), r_px, (40, 40, 40), 1)
        cv2.line(img, (center, 0), (center, MAP_DIM), (40, 40, 40), 1)
        cv2.line(img, (0, center), (MAP_DIM, center), (40, 40, 40), 1)

        # 4. 向量化计算极坐标到像素系的映射
        dist_array = np.array(distances)
        angles_rad = np.deg2rad(np.arange(360))

        # 过滤无效噪点
        valid_mask = (dist_array > 30) & (dist_array < max_dist_mm)
        valid_dists = dist_array[valid_mask]
        valid_angles = angles_rad[valid_mask]

        # 物理系：X轴正向(车头)为0度，逆时针角度增加。映射到图像系：
        # 车头朝上 (-Y方向), 车左朝左 (-X方向)
        robot_x = valid_dists * np.cos(valid_angles)
        robot_y = valid_dists * np.sin(valid_angles)

        img_x = (center - robot_y * scale).astype(np.int32)
        img_y = (center - robot_x * scale).astype(np.int32)

        in_bounds = (img_x >= 0) & (img_x < MAP_DIM) & (img_y >= 0) & (img_y < MAP_DIM)
        img_x = img_x[in_bounds]
        img_y = img_y[in_bounds]

        # 5. 打点 (RGB模式：绿色)
        img[img_y, img_x] = (0, 255, 0)

        # 6. 画一个红色的车头中心标志
        cv2.circle(img, (center, center), 4, (255, 0, 0), -1)

        # 7. 刷新到 UI
        img_pil = Image.fromarray(img)
        self.lidar_image_tk = ImageTk.PhotoImage(image=img_pil)
        self.lidar_display.config(image=self.lidar_image_tk)
    # =========================================================================

    def update_map_ui(self, icp_x, icp_y, icp_theta, diff_count, payload_len, diff_pts):
        self.lbl_icp_pose.config(text=f"ICP 匹配位姿 -> X: {icp_x:.3f}, Y: {icp_y:.3f}, Theta: {icp_theta:.3f} rad")
        self.lbl_map_count.config(text=f"最新接收栅格数: {diff_count} 个 (当前包 Payload 长度: {payload_len} 字节)")

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

        flipped_map = cv2.flip(color_map, 0)
        self.cached_color_map = flipped_map
        self.render_map()

    def update_status_ui(self, yaw, yaw_rate, target_yaw_rate, x, y, vel, sweep):
        self.lbl_yaw.config(text=f"Yaw: {yaw:.2f}°")
        self.lbl_yaw_rate.config(text=f"实际角速度: {yaw_rate:.2f} °/s")
        self.lbl_target_rate.config(text=f"预期角速度: {target_yaw_rate:.2f} °/s")
        self.lbl_pos.config(text=f"里程计 X: {x:.2f}  Y: {y:.2f}")
        self.lbl_vel.config(text=f"线速度: {vel:.2f} m/s")
        self.lbl_sweep.config(text=f"Sweep: {sweep}")

        error = target_yaw_rate - yaw_rate
        abs_error = abs(error)

        self.lbl_error.config(text=f"偏差(Error): {error:+.2f} °/s")

        if target_yaw_rate == 0.0 and abs_error < CONVERGENCE_THRESHOLD:
            self.lbl_convergence.config(text="收敛状况: 已静止", fg="green")
        elif abs_error <= CONVERGENCE_THRESHOLD:
            self.lbl_convergence.config(text="收敛状况: ✅ 已收敛", fg="green")
        elif abs_error <= ADJUSTING_THRESHOLD:
            self.lbl_convergence.config(text="收敛状况: ⚠️ 调节中...", fg="#FF8C00")
        else:
            self.lbl_convergence.config(text="收敛状况: ❌ 偏差过大", fg="red")

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotGUI(root)
    root.mainloop()