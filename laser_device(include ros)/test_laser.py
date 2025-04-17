import serial
import time
from collections import deque


class LaserController:
    def __init__(self):
        self.command_queue = [
            {'type': 'laser', 'cmd': [0x80, 0x06, 0x05, 0x01], 'resp': [0x80, 0x06, 0x85], 'delay': 0.3},
            {'type': 'resolution', 'cmd': [0xFA, 0x04, 0x0C, 0x02], 'resp': [0xFA, 0x04, 0x8C], 'delay': 0.3},
            {'type': 'frequency', 'cmd': [0xFA, 0x04, 0x0A, 0x14], 'resp': [0xFA, 0x04, 0x8A], 'delay': 0.3},
            {'type': 'continuous', 'cmd': [0x80, 0x06, 0x03], 'resp': [0x80, 0x06, 0x83], 'delay': 1.0}  # 增加延时
        ]
        self.filter = deque(maxlen=4)

    def calc_checksum(self, data):
        return (~sum(data) + 1) & 0xFF

    def validate_packet(self, data, expected_header):
        """强化数据包验证逻辑"""
        if len(data) < len(expected_header) + 1:  # 包头+至少1字节数据+校验
            return False, "Incomplete packet"

        # 校验包头
        if bytes(data[:len(expected_header)]) != bytes(expected_header):
            return False, f"Header mismatch {data[:3].hex()}"

        # 校验码验证
        calculated_cs = self.calc_checksum(data[:-1])
        return (data[-1] == calculated_cs), f"Checksum {'OK' if data[-1] == calculated_cs else 'Mismatch'}"

    def send_command(self, ser, command):
        """带流量控制的命令发送"""
        full_cmd = command['cmd'] + [self.calc_checksum(command['cmd'])]
        ser.reset_input_buffer()  # 清空接收缓存
        ser.write(bytes(full_cmd))
        print(f"Sent: {bytes(full_cmd).hex().upper()}")

        # 动态等待响应
        start_time = time.time()
        resp_buffer = bytearray()
        while time.time() - start_time < 2:  # 超时2秒
            resp_buffer += ser.read(ser.in_waiting or 1)

            # 尝试验证每个可能的数据包起始位置
            for i in range(len(resp_buffer)):
                possible_packet = resp_buffer[i:]
                is_valid, msg = self.validate_packet(possible_packet, command['resp'])
                if is_valid:
                    print(f"Valid response: {possible_packet.hex().upper()}")
                    return True
            time.sleep(0.01)

        print("Timeout waiting for response")
        return False


if __name__ == "__main__":
    ser = serial.Serial('COM15', 9600, timeout=1)
    controller = LaserController()

    try:
        # 分步初始化（带重试机制）
        for step in controller.command_queue:
            retry = 3
            while retry > 0:
                if controller.send_command(ser, step):
                    print(f"{step['type']} config success")
                    time.sleep(step['delay'])  # 重要：等待设备稳定
                    break
                else:
                    retry -= 1
                    print(f"Retry remaining: {retry}")
            else:
                raise RuntimeError(f"Failed to configure {step['type']}")

        # 数据接收循环（处理连续测量数据）
        while True:
            raw = ser.read_until(b'\x83')  # 特征字节
            if raw:
                # 尝试解析数据包
                packet = raw + ser.read(ser.in_waiting)
                is_valid, msg = controller.validate_packet(packet, [0x80, 0x06, 0x83])
                if is_valid and len(packet) >= 7:
                    ascii_data = bytes(packet[3:-1]).decode('ascii', 'ignore')
                    try:
                        distance = float(ascii_data)
                        controller.filter.append(distance)
                        print(f"Filtered: {sum(controller.filter) / len(controller.filter):.4f}m")
                    except ValueError:
                        print(f"Invalid data: {ascii_data}")

    except KeyboardInterrupt:
        ser.close()