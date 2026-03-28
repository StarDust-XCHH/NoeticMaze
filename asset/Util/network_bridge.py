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
        self.grid = np.full((MAP_DIM, MAP_DIM), 127, dtype=np.uint8)
        self.trajectory = deque(maxlen=2000)
        cv2.namedWindow("Bresenham SLAM Monitor", cv2.WINDOW_NORMAL)
        # 【修改点1】：因为改成了左右拼接，宽度变长了，建议修改默认窗口比例
        cv2.resizeWindow("Bresenham SLAM Monitor", 800, 500)

    def update(self, rx, ry, rt, hits, frees):
        if len(frees) > 0:
            valid_f = frees[(frees[:, 0] >= 0) & (frees[:, 0] < MAP_DIM) &
                            (frees[:, 1] >= 0) & (frees[:, 1] < MAP_DIM)]
            if len(valid_f) > 0:
                fx, fy = valid_f[:, 0], valid_f[:, 1]
                curr_vals = self.grid[fy, fx].astype(np.int16)
                self.grid[fy, fx] = np.clip(curr_vals - 64, 0, 255).astype(np.uint8)

        if len(hits) > 0:
            valid_h = hits[(hits[:, 0] >= 0) & (hits[:, 0] < MAP_DIM) &
                           (hits[:, 1] >= 0) & (hits[:, 1] < MAP_DIM)]
            if len(valid_h) > 0:
                self.grid[valid_h[:, 1], valid_h[:, 0]] = 255

        rgx, rgy = int((rx + MAP_OFFSET) / MAP_RES), int((ry + MAP_OFFSET) / MAP_RES)
        self.trajectory.append((rgx, rgy))

        color_map = cv2.cvtColor(self.grid, cv2.COLOR_GRAY2BGR)

        if len(self.trajectory) > 1:
            pts = np.array(self.trajectory, np.int32).reshape((-1, 1, 2))
            cv2.polylines(color_map, [pts], False, (255, 100, 0), 1)

        cv2.circle(color_map, (rgx, rgy), 3, (0, 0, 255), -1)
        ltx, lty = int(rgx + 8 * np.cos(rt)), int(rgy + 8 * np.sin(rt))
        cv2.line(color_map, (rgx, rgy), (ltx, lty), (0, 255, 0), 2)

        # 翻转地图
        flipped_map = cv2.flip(color_map, 0)

        # ==========================================
        # 【修改点2】：创建独立的右侧信息面板
        # 调整了 Y 坐标行距，确保在 MAP_DIM 较小(如 250) 时也能完整显示
        # ==========================================
        info_panel = np.zeros((MAP_DIM, 160, 3), dtype=np.uint8)

        # 字体和基础设置
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.45
        thick = 1

        # [ SLAM INFO ] 区域 (Y: 20 ~ 70)
        cv2.putText(info_panel, "[ SLAM INFO ]", (10, 20), font, scale, (200, 200, 200), thick)
        cv2.putText(info_panel, f"Pose X: {rx:.2f}", (10, 40), font, scale, (255, 255, 255), thick)
        cv2.putText(info_panel, f"Pose Y: {ry:.2f}", (10, 55), font, scale, (255, 255, 255), thick)
        cv2.putText(info_panel, f"Angle:  {np.degrees(rt):.1f} deg", (10, 70), font, scale, (255, 255, 255), thick)

        # [ MAP CONFIG ] 区域 (Y: 100 ~ 135)
        cv2.putText(info_panel, "[ MAP CONFIG ]", (10, 100), font, scale, (200, 200, 200), thick)
        cv2.putText(info_panel, f"Size: 5x5 m", (10, 120), font, scale, (0, 255, 255), thick)
        cv2.putText(info_panel, f"Res:  2 cm", (10, 135), font, scale, (0, 255, 255), thick)

        # [ CLOUD DATA ] 区域 (Y: 165 ~ 200)
        cv2.putText(info_panel, "[ CLOUD DATA ]", (10, 165), font, scale, (200, 200, 200), thick)
        cv2.putText(info_panel, f"Hits:  {len(hits)}", (10, 185), font, scale, (0, 255, 0), thick)
        # 现在最后一行最大 Y 坐标为 200，在 250 的高度内安全显示
        cv2.putText(info_panel, f"Frees: {len(frees)}", (10, 200), font, scale, (0, 255, 0), thick)


        # ==========================================
        # 【修改点3】：将地图和面板横向拼接
        # ==========================================
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
                if msg_type == 0x02: results.append(('POSE', payload))
                elif msg_type == 0x03:
                    try: results.append(('TIME', struct.unpack('<f', payload)[0]))
                    except: pass
                elif msg_type == 0x05:
                    if len(payload) >= 20:
                        rx, ry, rt, h_cnt, f_cnt = struct.unpack('<fffII', payload[:20])
                        hits_bytes, frees_bytes = h_cnt * 4, f_cnt * 4
                        if 20 + hits_bytes + frees_bytes <= len(payload):
                            hits = np.frombuffer(payload[20:20+hits_bytes], dtype=np.uint16).reshape(-1, 2)
                            frees = np.frombuffer(payload[20+hits_bytes:20+hits_bytes+frees_bytes], dtype=np.uint16).reshape(-1, 2)
                            results.append(('MAP', (rx, ry, rt, hits, frees)))
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
                # 【修改 2】：解包格式匹配发送端 '<BBfffffI'，用占位符忽略前面的参数，准确提取 num_points
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

                pose_back = None
                timeout_start = time.time()
                while pose_back is None and (time.time() - timeout_start < 2.0):
                    if ser.in_waiting > 0:
                        parsed_frames = decoder.feed_and_decode(ser.read(ser.in_waiting))
                        for f_type, f_data in parsed_frames:
                            if f_type == 'TIME': pass # 静默耗时，不然刷屏太快
                            elif f_type == 'POSE': pose_back = f_data
                            elif f_type == 'MAP': renderer.update(*f_data)

                if pose_back: client.sendall(pose_back)

        except Exception as e: print(f"\n[-] 连接异常: {e}")
        finally: client.close(); print(f"[-] 客户端 {addr} 断开")

if __name__ == '__main__':
    try: start_proxy()
    except KeyboardInterrupt: cv2.destroyAllWindows()