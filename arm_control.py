import serial
import struct
import time

# 串口配置
SERIAL_PORT = 'COM23 ' # '/dev/ttyAMA0'  # 根据实际情况修改
BAUD_RATE = 115200

# 完成标志词
COMPLETE_FLAG = b"MOVE_COMPLETE\r\n"


def calculate_checksum(data):
    """计算校验和（前15字节之和）"""
    return sum(data) & 0xFF


def pack_command(x, y, z, grip=0):
    """将坐标打包成STM32期望的16字节格式"""
    # 帧头 + 长度
    packet = bytearray([0xAA, 16])
    # X坐标 (float, 4字节)
    packet.extend(struct.pack('<f', x))
    # Y坐标 (float, 4字节)
    packet.extend(struct.pack('<f', y))
    # Z坐标 (float, 4字节)
    packet.extend(struct.pack('<f', z))
    # 抓取状态 (1字节)
    packet.append(grip & 0xFF)
    # 校验和 (1字节)
    packet.append(calculate_checksum(packet))
    return packet


def wait_for_complete(ser):
    """等待STM32返回完成标志"""
    buffer = bytearray()
    while True:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            buffer.extend(data)
            if COMPLETE_FLAG in buffer:
                print("收到动作完成标志!")
                buffer = buffer[buffer.find(COMPLETE_FLAG) + len(COMPLETE_FLAG):]
                return
        time.sleep(0.01)


def main():
    print("树莓派机械臂控制程序启动...")

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"成功连接到串口: {SERIAL_PORT}")

        # 定义要执行的指令序列
        commands = [
            (-90, 45, 100, 0),  # 移动到上方位置
            (-90, 45, 30, 0),  # 移动到下方位置
            (-90, 45, 30, 1),
            (-90, 45, 100, 1),
            (-90, -45, 100, 1),
            (-90, -45, 30, 1),
            (-90, -45, 30, 0),
            (-90, -45, 100, 0),
            (-90, -45, 100, 1),
        ]

        # 依次执行每个指令
        for i, (x, y, z, grip) in enumerate(commands, 1):
            print(f"\n--- 执行指令 {i} ---")
            print(f"目标位置: X={x}, Y={y}, Z={z}, 抓取状态={grip}")

            # 打包并发送指令
            packet = pack_command(x, y, z, grip)
            ser.write(packet)
            print("指令已发送")

            # 等待动作完成
            wait_for_complete(ser)

        print("\n所有指令执行完成!")

    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("\n程序退出")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("串口已关闭")


if __name__ == "__main__":
    main()