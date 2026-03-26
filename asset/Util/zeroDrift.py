import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

def init_serial(port_name, baud_rate):
    # 初始化串口连接
    try:
        ser = serial.Serial(port_name, baud_rate, timeout=1)
        print("成功打开串口")
        return ser
    except Exception as e:
        print("无法打开串口")
        exit(1)

def parse_line(line_str):
    # 解析串口数据并返回最后一位yaw值
    try:
        parts = line_str.strip().split(',')
        if len(parts) == 3:
            return float(parts[2])
    except ValueError:
        pass
    return None

def wait_for_stabilization(ser, s_time, s_threshold):
    # 上电后等待数据收敛稳定
    print("正在等待IMU数据稳定...")
    buffer = deque(maxlen=50)
    start_stable_time = None

    while True:
        line = ser.readline().decode('utf-8', errors='ignore')
        yaw = parse_line(line)

        if yaw is not None:
            buffer.append(yaw)

            if len(buffer) == buffer.maxlen:
                yaw_range = max(buffer) - min(buffer)

                if yaw_range < s_threshold:
                    if start_stable_time is None:
                        start_stable_time = time.time()
                    elif time.time() - start_stable_time >= s_time:
                        yaw_zero = np.mean(buffer)
                        print("数据已稳定初始零点基准值设定完成")
                        return yaw_zero
                else:
                    start_stable_time = None

def measure_drift(ser, yaw_zero, max_duration):
    # 测量实际运行时长内的零漂数据
    print("开始零漂测试请保持设备绝对静止...")

    start_time = time.time()
    time_data = []
    yaw_data = []

    last_print_time = start_time

    try:
        while True:
            current_time = time.time()
            elapsed = current_time - start_time

            if elapsed >= max_duration:
                break

            line = ser.readline().decode('utf-8', errors='ignore')
            yaw = parse_line(line)

            if yaw is not None:
                relative_yaw = yaw - yaw_zero
                time_data.append(elapsed)
                yaw_data.append(relative_yaw)

                if current_time - last_print_time >= 60:
                    print("测试进行中已经记录一分钟数据...")
                    last_print_time = current_time

    except KeyboardInterrupt:
        print("测试被手动中断将处理已收集的数据")

    return np.array(time_data), np.array(yaw_data)

def analyze_and_optimize(time_data, yaw_data):
    # 分析数据并计算最终的线性漂移补偿率
    i = 0
    count = len(time_data)

    if count < 2:
        print("收集的数据不足以进行分析")
        return

    coefficients = np.polyfit(time_data, yaw_data, 1)
    drift_rate = coefficients[0]

    print("测试完成零漂分析结果如下")
    print("计算得出的每秒漂移率为:", drift_rate)

    plt.figure(figsize=(10, 5))
    plt.plot(time_data, yaw_data, label='Raw Drift', color='blue', alpha=0.6)

    fit_line = np.poly1d(coefficients)
    plt.plot(time_data, fit_line(time_data), label='Fitted Line', color='red', linestyle='--')

    plt.title('IMU Yaw Angle Zero Drift Over Real Task Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Drift (Degrees)')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    # 使用局部变量代替全局常量
    port_str = 'COM10'
    baud = 115200
    duration_time = 15 * 60  # 修改为你实际单次运行的最大时长，例如15分钟
    stable_time_req = 5.0
    stable_threshold_req = 0.5

    serial_port = init_serial(port_str, baud)

    try:
        zero_reference = wait_for_stabilization(serial_port, stable_time_req, stable_threshold_req)
        t_data, y_data = measure_drift(serial_port, zero_reference, duration_time)
        analyze_and_optimize(t_data, y_data)

    finally:
        serial_port.close()
        print("串口已安全关闭")