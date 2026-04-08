import socket
import serial
import struct
import time
import numpy as np
import cv2
from collections import deque

COM_PORT = 'COM10'
BAUD_RATE = 1500000
HOST = '0.0.0.0'
PORT = 9999

MAP_DIM = 250
MAP_RES = 0.02
MAP_OFFSET = 0.0

class SLAMRenderer:
    def __init__(self):
        # 127 代表 MAP_UNKNOWN (未知区域)
        self.grid = np.full((MAP_DIM, MAP_DIM), 127, dtype=np.uint8)
        self.trajectory = deque(maxlen=2000)
        
        # 状态记录
        self.last_icp_time = 0.0  
        self.max_icp_time = 0.0   
        self.rx, self.ry, self.rt = 0.0, 0.0, 0.0
        self.diff_count_last_frame = 0 # 记录最后一帧传了多少个变化栅格
        
        cv2.namedWindow("Bresenham SLAM Monitor", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Bresenham SLAM Monitor", 800, 500)

    def update_pose(self, rx, ry, rt):
        """仅更新位姿与轨迹"""
        self.rx, self.ry, self.rt = rx, ry, rt
        rgx = int((rx + MAP_OFFSET) / MAP_RES)
        rgy = int((ry + MAP_OFFSET) / MAP_RES)
        self.trajectory.append((rgx, rgy))

    def update_map(self, diff_pts):
            if len(diff_pts) > 0:
                xs = diff_pts[:, 0]
                ys = diff_pts[:, 1]
                states = diff_pts[:, 2]

                valid = (xs >= 0) & (xs < MAP_DIM) & (ys >= 0) & (ys < MAP_DIM)
                xs, ys, states = xs[valid], ys[valid], states[valid]

                # --- 修改逻辑开始 ---
                # 如果你希望永久保留障碍物（观察重影/偏差）
                # 我们只在 state == 2 (障碍物) 时更新像素
                
                # 方案 A：标准更新（你现在的逻辑，会互相覆盖）
                # pixel_values = np.full_like(states, 127, dtype=np.uint8)
                # pixel_values[states == 1] = 0
                # pixel_values[states == 2] = 255
                # self.grid[ys, xs] = pixel_values

                # 方案 B：障碍物持久化（观察偏差神器）
                # 只提取障碍物点位
                obs_mask = (states == 2)
                if np.any(obs_mask):
                    # 仅将障碍物点设为白色，且不清除之前的白色
                    self.grid[ys[obs_mask], xs[obs_mask]] = 255
                    
                # 可选：将空地设为深灰色而不是纯黑，以便区分“从未扫描”和“扫描为空”
                free_mask = (states == 1)
                # 只有当该点之前是未知（127）时，才设为黑色（0）
                # 这样已经生成的白色障碍物就不会被空地信号抹除
                update_free_mask = free_mask & (self.grid[ys, xs] == 127)
                self.grid[ys[update_free_mask], xs[update_free_mask]] = 40 # 深灰色
                # --- 修改逻辑结束 ---
    def render(self):
        """独立渲染 UI"""
        color_map = cv2.cvtColor(self.grid, cv2.COLOR_GRAY2BGR)

        # 绘制轨迹
        if len(self.trajectory) > 1:
            pts = np.array(self.trajectory, np.int32).reshape((-1, 1, 2))
            cv2.polylines(color_map, [pts], False, (255, 100, 0), 1)

        # 绘制当前机器人位置与朝向
        if len(self.trajectory) > 0:
            rgx, rgy = self.trajectory[-1]
            cv2.circle(color_map, (rgx, rgy), 3, (0, 0, 255), -1)
            ltx = int(rgx + 8 * np.cos(self.rt))
            lty = int(rgy + 8 * np.sin(self.rt))
            cv2.line(color_map, (rgx, rgy), (ltx, lty), (0, 255, 0), 2)

        flipped_map = cv2.flip(color_map, 0)

        # 绘制信息面板
        info_panel = np.zeros((MAP_DIM, 160, 3), dtype=np.uint8)
        font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1

        cv2.putText(info_panel, "[ SLAM INFO ]", (10, 20), font, scale, (200, 200, 200), thick)
        cv2.putText(info_panel, f"Pose X: {self.rx:.2f}", (10, 40), font, scale, (255, 255, 255), thick)
        cv2.putText(info_panel, f"Pose Y: {self.ry:.2f}", (10, 55), font, scale, (255, 255, 255), thick)
        cv2.putText(info_panel, f"Angle:  {np.degrees(self.rt):.1f} deg", (10, 70), font, scale, (255, 255, 255), thick)
        
        cv2.putText(info_panel, f"ICP:    {self.last_icp_time:.1f} ms", (10, 85), font, scale, (0, 165, 255), thick)
        cv2.putText(info_panel, f"Max ICP:{self.max_icp_time:.1f} ms", (10, 100), font, scale, (0, 0, 255), thick) 

        cv2.putText(info_panel, "[ CLOUD DATA ]", (10, 165), font, scale, (200, 200, 200), thick)
        # 【新增】：显示当前帧更新了多少个栅格，直观感受省流效果
        cv2.putText(info_panel, f"Map Diffs: {self.diff_count_last_frame}", (10, 185), font, scale, (0, 255, 0), thick)
        if self.diff_count_last_frame == 0:
            cv2.putText(info_panel, f"(Bandwidth Saved!)", (10, 200), font, 0.4, (0, 200, 0), 1)

        display = np.hstack((flipped_map, info_panel))
        cv2.imshow("Bresenham SLAM Monitor", display)
        cv2.waitKey(1)

def calculate_checksum(data: bytes) -> int:
    checksum = 0
    for byte in data: checksum ^= byte
    return checksum

def pack_for_dma(header_data: bytes, points_data: bytes) -> bytes:
    payload = header_data + points_data
    frame_body = struct.pack('<BI', 0x01, len(payload)) + payload
    return b'\xAA\x55' + frame_body + bytes([calculate_checksum(frame_body)]) + b'\x5A\xA5'

class SerialDecoder:
    def __init__(self): self.buffer = bytearray()
    
    def feed_and_decode(self, data: bytes):
        self.buffer.extend(data)
        results = []
        while len(self.buffer) >= 10:
            head_idx = self.buffer.find(b'\xAA\x55')
            if head_idx == -1: self.buffer.clear(); break
            if head_idx > 0: self.buffer = self.buffer[head_idx:]
            if len(self.buffer) < 10: break

            msg_type = self.buffer[2]
            payload_len = struct.unpack('<I', self.buffer[3:7])[0]
            total_len = 10 + payload_len

            if len(self.buffer) < total_len: break
            frame = self.buffer[:total_len]
            if frame[-2:] != b'\x5A\xA5': self.buffer = self.buffer[2:]; continue

            if calculate_checksum(frame[2:-3]) == frame[-3]:
                payload = frame[7:-3]
                
                # 【保留】：解析位姿
                if msg_type == 0x02:
                    if len(payload) == 12:
                        rx, ry, rt = struct.unpack('<fff', payload)
                        results.append(('POSE', (rx, ry, rt, payload))) 
                
                # 【保留】：解析耗时
                elif msg_type == 0x03:
                    if len(payload) == 4:
                        results.append(('TIME', struct.unpack('<f', payload)[0]))
                
                # 【核心替换】：解析 Type 0x06 增量地图包
                elif msg_type == 0x06:
                    if len(payload) >= 4:
                        diff_cnt = struct.unpack('<I', payload[:4])[0]
                        expected_len = 4 + diff_cnt * 3
                        if len(payload) >= expected_len:
                            # 直接利用 numpy 将连续的字节流 reshape 为 (N, 3) 的矩阵
                            diff_pts = np.frombuffer(payload[4:expected_len], dtype=np.uint8).reshape(-1, 3)
                            results.append(('MAP_DIFF', diff_pts))
            
            self.buffer = self.buffer[total_len:]
            
        return results

def start_proxy():
    print(f"[*] 启动串口: {COM_PORT} @ {BAUD_RATE}")
    try: ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0)
    except Exception as e: print(f"[-] 串口打开失败: {e}"); return

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    print(f"[*] 中转服务就绪，监听 TCP 端口 {PORT}...")

    decoder = SerialDecoder()
    renderer = SLAMRenderer()

    while True:
        client, addr = server.accept()
        print(f"[+] 客户端 {addr} 已连接")
        try:
            while True:
                header_data = client.recv(26)
                if not header_data or len(header_data) < 26: break
                _, _, _, _, _, _, _, num_points = struct.unpack('<BBfffffI', header_data)

                points_size = num_points * 8
                points_data = bytearray()
                while len(points_data) < points_size:
                    chunk = client.recv(min(points_size - len(points_data), 4096))
                    if not chunk: break
                    points_data.extend(chunk)

                if len(points_data) < points_size: break
                ser.write(pack_for_dma(header_data, points_data))
                ser.flush()

                pose_back_payload = None
                timeout_start = time.time()
                
                while time.time() - timeout_start < 2.0:
                    if ser.in_waiting > 0:
                        parsed_frames = decoder.feed_and_decode(ser.read(ser.in_waiting))
                        for f_type, f_data in parsed_frames:
                            if f_type == 'TIME': 
                                renderer.last_icp_time = f_data
                                if f_data > renderer.max_icp_time:
                                    renderer.max_icp_time = f_data
                                    print(f"[!] 记录刷新: 发现最长 ICP 耗时 -> {f_data:.2f} ms")
                            
                            elif f_type == 'POSE': 
                                rx, ry, rt, pose_back_payload = f_data
                                renderer.update_pose(rx, ry, rt)
                                
                            elif f_type == 'MAP_DIFF': 
                                renderer.update_map(f_data)
                    
                    # 只要收到了 Pose 就跳出内层等待，赶紧发给 ROS 客户端
                    if pose_back_payload:
                        break
                
                # 无论是否收到地图增量，每一帧都强制渲染一下 UI (为了更新位姿和闪烁的轨迹)
                renderer.render()

                if pose_back_payload: 
                    client.sendall(pose_back_payload)

        except Exception as e: print(f"\n[-] 连接异常: {e}")
        finally: client.close(); print(f"[-] 客户端 {addr} 断开")

if __name__ == '__main__':
    try: start_proxy()
    except KeyboardInterrupt: cv2.destroyAllWindows()