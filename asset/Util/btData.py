import serial
import struct
import time

# 配置参数
PORT = 'COM17'
BAUD_RATE = 921600

class RobotDecoder:
    def __init__(self, port, baud):
        try:
            # 删除了引起报错的 set_buffer_size 行
            self.ser = serial.Serial(port, baud, timeout=1)
            print(f"成功连接到串口: {port} (波特率: {baud})")
        except Exception as e:
            print(f"无法打开串口: {e}")
            exit()

    def calc_checksum(self, data):
        """ 计算单字节校验和 """
        return sum(data) & 0xFF

    def run(self):
        print("开始接收数据，按 Ctrl+C 停止...")
        buffer = b""

        try:
            while True:
                # 按照要求延迟 1ms 降低刷屏速度和 CPU 占用
                time.sleep(0.001)

                if self.ser.in_waiting > 0:
                    # 一次性读取所有缓冲区数据
                    buffer += self.ser.read(self.ser.in_waiting)

                # 解析逻辑：寻找包头 0x55AA
                while len(buffer) >= 3:
                    # STM32 内存中的 0x55AA 发送到串口通常是 [0xAA, 0x55] (小端)
                    idx = buffer.find(b'\xAA\x55')
                    if idx == -1:
                        # 也尝试一下大端序的情况
                        idx = buffer.find(b'\x55\xAA')

                    if idx == -1:
                        buffer = buffer[-1:] # 保留最后一个字节，防止切断潜在的包头
                        break

                    # 检查类型位
                    if len(buffer) < idx + 3:
                        break

                    pkt_type = buffer[idx + 2]

                    # ==========================================
                    # 机器人状态包 (Type 0x01) - 28字节
                    # ==========================================
                    if pkt_type == 0x01:
                        pkt_len = 28
                        if len(buffer) < idx + pkt_len:
                            break

                        payload = buffer[idx + 3 : idx + 27]
                        received_checksum = buffer[idx + 27]

                        if self.calc_checksum(payload) == received_checksum:
                            # 解析: I(uint32), f(float) * 5
                            data = struct.unpack('<Ifffff', payload)
                            print(f"[STAT] 圈数:{data[0]:<4} | X:{data[1]:>7.2f} Y:{data[2]:>7.2f} | Yaw:{data[4]:>7.2f}")

                        buffer = buffer[idx + pkt_len:]

                    # ==========================================
                    # 雷达数据包 (Type 0x02) - 724字节
                    # ==========================================
                    elif pkt_type == 0x02:
                        pkt_len = 724
                        if len(buffer) < idx + pkt_len:
                            break

                        payload = buffer[idx + 3 : idx + 723]
                        received_checksum = buffer[idx + 723]

                        if self.calc_checksum(payload) == received_checksum:
                            # 解析 360 个 uint16 (q15_t)
                            distances = struct.unpack(f'<{360}H', payload)
                            valid_points = sum(1 for d in distances if d > 0)
                            print(f"  [LIDAR] 接收到雷达包，有效点数: {valid_points}")

                        buffer = buffer[idx + pkt_len:]

                    else:
                        # 如果类型不对，跳过当前包头继续找
                        buffer = buffer[idx + 1:]

        except KeyboardInterrupt:
            print("\n解析程序已停止。")
        finally:
            if self.ser.is_open:
                self.ser.close()

if __name__ == "__main__":
    decoder = RobotDecoder(PORT, BAUD_RATE)
    decoder.run()