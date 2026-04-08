import tkinter as tk
from tkinter import scrolledtext, messagebox
import serial
import struct
import threading
import time

# --- 协议配置 ---
SERIAL_PORT = 'COM17'
BAUD_RATE = 921600
HEX_SEND_HEADER = 0x5A5A
TYPE_CMD = 0x03
TYPE_ACK = 0x04
TYPE_STATUS = 0x01

# --- PID 收敛评估阈值 (可根据你的系统要求修改) ---
CONVERGENCE_THRESHOLD = 2.0  # 偏差在 2.0 °/s 以内认为已收敛
ADJUSTING_THRESHOLD = 15.0   # 偏差在 15.0 °/s 以内认为正在调节，大于此值认为偏差过大

class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 机器人 16进制控制台 (角速度PID调参版)")
        self.root.geometry("680x600") # 稍微加高加宽以容纳更多状态显示

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
        except Exception as e:
            messagebox.showerror("串口错误", f"无法打开 {SERIAL_PORT}: {e}")
            self.root.destroy()
            return

        self.running = True
        self.setup_ui()

        self.recv_thread = threading.Thread(target=self.receive_handler, daemon=True)
        self.recv_thread.start()

    def setup_ui(self):
        # 1. 状态显示区
        status_frame = tk.LabelFrame(self.root, text=" 机器人状态 (0x55AA) ", padx=10, pady=10)
        status_frame.pack(fill="x", padx=10, pady=5)

        self.lbl_yaw = tk.Label(status_frame, text="Yaw: 0.00°", font=("Arial", 12, "bold"), fg="blue")
        self.lbl_yaw.grid(row=0, column=0, padx=10, rowspan=2)

        # 角速度与调参监控区
        self.lbl_yaw_rate = tk.Label(status_frame, text="实际角速度: 0.00 °/s", font=("Arial", 10, "bold"), fg="darkgreen")
        self.lbl_yaw_rate.grid(row=0, column=1, padx=10, sticky="w")

        self.lbl_target_rate = tk.Label(status_frame, text="预期角速度: 0.00 °/s", font=("Arial", 10), fg="gray")
        self.lbl_target_rate.grid(row=1, column=1, padx=10, sticky="w")

        # 【新增】偏差值显示
        self.lbl_error = tk.Label(status_frame, text="偏差(Error): 0.00 °/s", font=("Arial", 10, "bold"), fg="red")
        self.lbl_error.grid(row=2, column=1, padx=10, sticky="w", pady=(5, 0))

        # 【新增】收敛状况评估
        self.lbl_convergence = tk.Label(status_frame, text="收敛状况: 等待数据", font=("Arial", 10, "bold"), fg="black")
        self.lbl_convergence.grid(row=3, column=1, padx=10, sticky="w")

        # 其他状态
        self.lbl_pos = tk.Label(status_frame, text="X: 0.00  Y: 0.00", font=("Arial", 10))
        self.lbl_pos.grid(row=0, column=2, padx=10)

        self.lbl_vel = tk.Label(status_frame, text="线速度: 0.00 m/s", font=("Arial", 10))
        self.lbl_vel.grid(row=1, column=2, padx=10)

        self.lbl_sweep = tk.Label(status_frame, text="Sweep: 0", font=("Arial", 10))
        self.lbl_sweep.grid(row=0, column=3, padx=10)

        # 2. 指令输入区
        input_frame = tk.LabelFrame(self.root, text=" 发送 16 进制指令 (0x5A5A) ", padx=10, pady=10)
        input_frame.pack(fill="x", padx=10, pady=5)

        tk.Label(input_frame, text="预期角速度(°/s):").grid(row=0, column=0)
        self.ent_yaw_rate = tk.Entry(input_frame, width=10)
        self.ent_yaw_rate.grid(row=0, column=1, padx=5)
        self.ent_yaw_rate.insert(0, "45.0") # 默认转速 45度/秒

        tk.Label(input_frame, text="预期线速度(m/s):").grid(row=0, column=2)
        self.ent_linear_vel = tk.Entry(input_frame, width=10)
        self.ent_linear_vel.grid(row=0, column=3, padx=5)
        self.ent_linear_vel.insert(0, "0.0")

        self.btn_send = tk.Button(input_frame, text="发送指令", command=self.send_command, bg="orange", width=10)
        self.btn_send.grid(row=0, column=4, padx=10)

        # 3. 日志回显区
        log_frame = tk.LabelFrame(self.root, text=" 系统日志 (ACK 确认区) ")
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.log_area = scrolledtext.ScrolledText(log_frame, state='disabled', height=15)
        self.log_area.pack(fill="both", expand=True)

    def write_log(self, msg, color="black"):
        self.log_area.configure(state='normal')
        self.log_area.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        self.log_area.see(tk.END)
        self.log_area.configure(state='disabled')

    def send_command(self):
        try:
            yaw_rate = float(self.ent_yaw_rate.get())
            linear_vel = float(self.ent_linear_vel.get())
            # 打包结构: Header(H), Type(B), yaw_rate(f), linear_vel(f)
            packet = struct.pack('<HBff', HEX_SEND_HEADER, TYPE_CMD, yaw_rate, linear_vel)
            checksum = sum(packet) & 0xFF
            full_packet = packet + struct.pack('<B', checksum)
            self.ser.write(full_packet)
            self.write_log(f"已发送指令: 角速度={yaw_rate}°/s, 线速度={linear_vel} m/s")
        except ValueError:
            messagebox.showwarning("输入错误", "请输入有效数字")

    def receive_handler(self):
        buffer = b""
        while self.running:
            if self.ser.in_waiting > 0:
                buffer += self.ser.read(self.ser.in_waiting)

                if len(buffer) > 4096:
                    buffer = buffer[-4096:]

                while len(buffer) >= 12:
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

                    if len(buffer) < 3: break
                    pkg_type = buffer[2]

                    # --- 解析 A: 机器人状态包 (32 字节) ---
                    if pkg_type == TYPE_STATUS and buffer.startswith(b'\xAA\x55'):
                        if len(buffer) < 32: break
                        try:
                            payload = buffer[3:31]
                            d = struct.unpack('<Iffffff', payload)
                            self.root.after(0, self.update_status_ui, d[4], d[5], d[6], d[1], d[2], d[3], d[0])
                        except Exception as e:
                            print(f"Status Parse Error: {e}")
                        buffer = buffer[32:]

                        # --- 解析 B: ACK 确认包 (12字节) ---
                    elif pkg_type == TYPE_ACK and buffer.startswith(b'\x5A\x5A'):
                        if len(buffer) < 12: break
                        try:
                            _, _, r_yaw_rate, r_linear_vel, _ = struct.unpack('<HBffB', buffer[:12])
                            self.root.after(0, self.write_log, f"收到回显 ✔: 角速度={r_yaw_rate:.1f}°/s, 线速度={r_linear_vel:.2f}m/s")
                        except Exception as e:
                            print(f"ACK Parse Error: {e}")
                        buffer = buffer[12:]

                    else:
                        buffer = buffer[2:]

            time.sleep(0.002)

    def update_status_ui(self, yaw, yaw_rate, target_yaw_rate, x, y, vel, sweep):
        # 基础数据更新
        self.lbl_yaw.config(text=f"Yaw: {yaw:.2f}°")
        self.lbl_yaw_rate.config(text=f"实际角速度: {yaw_rate:.2f} °/s")
        self.lbl_target_rate.config(text=f"预期角速度: {target_yaw_rate:.2f} °/s")
        self.lbl_pos.config(text=f"X: {x:.2f}  Y: {y:.2f}")
        self.lbl_vel.config(text=f"线速度: {vel:.2f} m/s")
        self.lbl_sweep.config(text=f"Sweep: {sweep}")

        # 【新增】计算偏差 (Error)
        error = target_yaw_rate - yaw_rate
        abs_error = abs(error)

        # 动态更新偏差 UI
        # 为了醒目，正负偏差保持原样，但可以根据正负号判断是超调还是不足
        self.lbl_error.config(text=f"偏差(Error): {error:+.2f} °/s")

        # 【新增】判定收敛状况
        if target_yaw_rate == 0.0 and abs_error < CONVERGENCE_THRESHOLD:
            # 目标为0且偏差很小，视作静止
            self.lbl_convergence.config(text="收敛状况: 已静止", fg="green")
        elif abs_error <= CONVERGENCE_THRESHOLD:
            self.lbl_convergence.config(text="收敛状况: ✅ 已收敛", fg="green")
        elif abs_error <= ADJUSTING_THRESHOLD:
            self.lbl_convergence.config(text="收敛状况: ⚠️ 调节中...", fg="#FF8C00") # 深橙色
        else:
            self.lbl_convergence.config(text="收敛状况: ❌ 偏差过大", fg="red")


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotGUI(root)
    root.mainloop()