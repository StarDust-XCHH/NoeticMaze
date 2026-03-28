import serial
import time

class LidarRunner:
    def __init__(self, port='COM11', baudrate=460800): # 根据实际情况修改端口和波特率
        # 1. 初始化串口
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"成功连接雷达: {port}")
        except Exception as e:
            print(f"连接失败: {e}")
            exit()

        # 预计算的角度偏移 (与 C 代码一致)
        self.ANGLE_OFFSETS = [
            0.000,  0.705,  1.410,  2.115,  2.820,  3.525,  4.230,  4.935,  5.640,  6.345,
            7.050,  7.755,  8.460,  9.165,  9.870, 10.575, 11.280, 11.985, 12.690, 13.395,
            14.100, 14.805, 15.510, 16.215, 16.920, 17.625, 18.330, 19.035, 19.740, 20.445,
            21.150, 21.855, 22.560, 23.265, 23.970, 24.675, 25.380, 26.085, 26.790, 27.495
        ]

        self.last_start_angle = 0.0
        self.current_sweep_points = []
        self.frame_len = 84
        self.running = True

    def start_scan(self):
        # 发送你提供的开启指令
        start_cmd = bytes([0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22])
        self.ser.write(start_cmd)
        print("已发送开启扫描指令...")

    def parse_loop(self):
        buffer = bytearray()

        try:
            while self.running:
                # 2. 读取串口数据 (每次读 1024 字节提高效率)
                if self.ser.in_waiting > 0:
                    chunk = self.ser.read(self.ser.in_waiting)
                    buffer.extend(chunk)

                # 3. 寻找帧头并解析
                while len(buffer) >= self.frame_len:
                    # 检查帧头特征 (0xA0 0x50 掩码匹配)
                    if (buffer[0] & 0xF0) == 0xA0 and (buffer[1] & 0xF0) == 0x50:
                        frame = buffer[:self.frame_len]

                        # 解析这一帧
                        if self.decode_frame(frame):
                            # 返回 True 代表一圈结束
                            self.print_sweep_result()
                            self.current_sweep_points = []

                        # 移除已处理的帧
                        del buffer[:self.frame_len]
                    else:
                        # 没匹配到帧头，跳过一个字节继续找
                        del buffer[0]

        except KeyboardInterrupt:
            print("\n停止扫描...")
            self.running = False
            self.ser.close()

    def decode_frame(self, frame):
        # 校验和
        expected_checksum = ((frame[1] & 0x0F) << 4) | (frame[0] & 0x0F)
        calculated_checksum = 0
        for i in range(2, self.frame_len):
            calculated_checksum ^= frame[i]

        if calculated_checksum != expected_checksum:
            return False

        # 起始角度
        angle_raw = ((frame[3] & 0x7F) << 8) | frame[2]
        start_angle = angle_raw * 0.015625

        # 一圈结束检测 (角度回绕)
        is_sweep_done = False
        if start_angle < self.last_start_angle and (self.last_start_angle - start_angle) > 100.0:
            is_sweep_done = True
        self.last_start_angle = start_angle

        # 解析 40 个采样点
        for i in range(40):
            base = 4 + (i * 2)
            dist = (frame[base + 1] << 8) | frame[base]

            # 只保留有效距离的点 (可根据需要调整范围)
            if 150 < dist < 8000:
                angle = (start_angle + self.ANGLE_OFFSETS[i]) % 360.0
                self.current_sweep_points.append((round(angle, 2), dist))

        return is_sweep_done

    def print_sweep_result(self):
        count = len(self.current_sweep_points)
        timestamp = time.strftime("%H:%M:%S")
        print(f"[{timestamp}] 一圈扫描完成 | 总采样点数: {count}")
        if count > 0:
            # 打印前3个点看看数据对不对
            print(f"   样例点: {self.current_sweep_points[:3]}")

if __name__ == "__main__":
    # 请根据设备管理器里的显示修改 'COM3' (Windows) 或 '/dev/ttyUSB0' (Linux)
    # 波特率通常为 115200 或 230400，请确认你的雷达参数
    lidar = LidarRunner(port='COM3', baudrate=115200)
    lidar.start_scan()
    lidar.parse_loop()