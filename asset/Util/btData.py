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

class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 机器人 16进制控制台")
        self.root.geometry("600x550")

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

        self.lbl_yaw = tk.Label(status_frame, text="Yaw: 0.00°", font=("Arial", 14, "bold"), fg="blue")
        self.lbl_yaw.grid(row=0, column=0, padx=20)

        self.lbl_pos = tk.Label(status_frame, text="X: 0.00  Y: 0.00", font=("Arial", 10))
        self.lbl_pos.grid(row=0, column=1, padx=20)

        self.lbl_sweep = tk.Label(status_frame, text="Sweep: 0", font=("Arial", 10))
        self.lbl_sweep.grid(row=0, column=2, padx=20)

        # 2. 指令输入区
        input_frame = tk.LabelFrame(self.root, text=" 发送 16 进制指令 (0x5A5A) ", padx=10, pady=10)
        input_frame.pack(fill="x", padx=10, pady=5)

        tk.Label(input_frame, text="角度:").grid(row=0, column=0)
        self.ent_angle = tk.Entry(input_frame, width=10)
        self.ent_angle.grid(row=0, column=1, padx=5)
        self.ent_angle.insert(0, "90.0")

        tk.Label(input_frame, text="速度:").grid(row=0, column=2)
        self.ent_speed = tk.Entry(input_frame, width=10)
        self.ent_speed.grid(row=0, column=3, padx=5)
        self.ent_speed.insert(0, "0.0")

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
            angle = float(self.ent_angle.get())
            speed = float(self.ent_speed.get())
            packet = struct.pack('<HBff', HEX_SEND_HEADER, TYPE_CMD, angle, speed)
            checksum = sum(packet) & 0xFF
            full_packet = packet + struct.pack('<B', checksum)
            self.ser.write(full_packet)
            self.write_log(f"已发送指令: {angle}°, {speed} rps")
        except ValueError:
            messagebox.showwarning("输入错误", "请输入有效数字")

    def receive_handler(self):
        buffer = b""
        while self.running:
            if self.ser.in_waiting > 0:
                buffer += self.ser.read(self.ser.in_waiting)

                # 限制缓冲区大小防止溢出
                if len(buffer) > 4096:
                    buffer = buffer[-4096:]

                while len(buffer) >= 12:
                    # 寻找可能的包头位置
                    idx_55aa = buffer.find(b'\xAA\x55') # STM32 0x55AA 的内存序
                    idx_5a5a = buffer.find(b'\x5A\x5A') # STM32 0x5A5A 的内存序

                    # 找出最近的包头位置
                    indices = [i for i in [idx_55aa, idx_5a5a] if i != -1]
                    if not indices:
                        buffer = buffer[-1:] # 没找到，只保留最后一个字节
                        break

                    first_idx = min(indices)
                    if first_idx > 0:
                        buffer = buffer[first_idx:] # 丢弃包头前的垃圾数据
                        continue

                    # 确认 Type 字节存在
                    if len(buffer) < 3: break
                    pkg_type = buffer[2]

                    # --- 解析 A: 机器人状态包 (28字节) ---
                    if pkg_type == TYPE_STATUS and buffer.startswith(b'\xAA\x55'):
                        if len(buffer) < 28: break
                        try:
                            # 严格跳过头 3 字节 (Header 2 + Type 1)
                            payload = buffer[3:27]
                            # 解析: I(Sweep), f*5 (X, Y, Vel, Yaw, Rate)
                            d = struct.unpack('<Ifffff', payload)
                            self.root.after(0, self.update_status_ui, d[4], d[1], d[2], d[0])
                        except Exception as e:
                            print(f"Status Parse Error: {e}")
                        buffer = buffer[28:]

                    # --- 解析 B: ACK 确认包 (12字节) ---
                    elif pkg_type == TYPE_ACK and buffer.startswith(b'\x5A\x5A'):
                        if len(buffer) < 12: break
                        try:
                            # 解析: Header(H), Type(B), Angle(f), Speed(f), Checksum(B)
                            _, _, r_angle, r_speed, _ = struct.unpack('<HBffB', buffer[:12])
                            self.root.after(0, self.write_log, f"收到回显 ✔: 角度={r_angle:.1f}, 速度={r_speed:.2f}")
                        except Exception as e:
                            print(f"ACK Parse Error: {e}")
                        buffer = buffer[12:]

                    else:
                        # 虽有包头但类型不对或数据损坏，挪出一位继续找
                        buffer = buffer[2:]

            time.sleep(0.002) # 提高采样率

    def update_status_ui(self, yaw, x, y, sweep):
        self.lbl_yaw.config(text=f"Yaw: {yaw:.2f}°")
        self.lbl_pos.config(text=f"X: {x:.2f}  Y: {y:.2f}")
        self.lbl_sweep.config(text=f"Sweep: {sweep}")

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotGUI(root)
    root.mainloop()