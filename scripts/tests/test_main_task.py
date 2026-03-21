#!/usr/bin/env python3
"""
Main_task.prs 테스트 스크립트
- 레지스터 맵:
  - 301 (x), 302 (y), 303 (z): 위치 (mm, int16)
  - 304 (Rx), 305 (Ry), 306 (Rz): 회전 (deg×10, int16)
  - 307-312: x2, y2, z2, Rx2, Ry2, Rz2 (미사용)
  - 351 (task_number): 명령 코드
  - 352 (task_done): 상태 (0=Idle, 1=Running, 2=Done, 3=Error)
"""

import time
from pymodbus.client import ModbusTcpClient

# 연결 설정
ROBOT_IP = "192.168.0.29"
ROBOT_PORT = 1502  # Main_task 포트

# 레지스터 주소
REG_X = 301
REG_Y = 302
REG_Z = 303
REG_RX = 304
REG_RY = 305
REG_RZ = 306
REG_COMMAND = 351    # task_number
REG_STATUS = 352     # task_done

# 상태 코드
STATUS_IDLE = 0
STATUS_RUNNING = 1
STATUS_DONE = 2
STATUS_ERROR = 3


def to_int16(value):
    """int16 → uint16 변환 (음수 처리)"""
    if value < 0:
        return value + 65536
    return int(value)


def wait_for_completion(client, timeout=10.0):
    """로봇 동작 완료 대기"""
    start = time.time()
    while time.time() - start < timeout:
        rr = client.read_holding_registers(REG_STATUS, 1)
        if rr.isError():
            print("상태 읽기 실패")
            return False
        status = rr.registers[0]
        if status == STATUS_DONE:
            print("완료 (status=2)")
            return True
        elif status == STATUS_ERROR:
            print("오류 발생 (status=3)")
            return False
        elif status == STATUS_RUNNING:
            print(".", end="", flush=True)
        time.sleep(0.1)
    print("\n타임아웃")
    return False


def send_command(client, cmd, x=0, y=0, z=0, rx=0, ry=0, rz=0):
    """명령 전송 (포즈 데이터 포함)"""
    # ±180° 정규화
    rx = rx % 360
    if rx > 180: rx -= 360
    ry = ry % 360
    if ry > 180: ry -= 360
    rz = rz % 360
    if rz > 180: rz -= 360

    # 포즈 데이터 쓰기 (위치는 mm×10, 회전은 deg×10)
    pose_data = [
        to_int16(int(round(x * 10))),
        to_int16(int(round(y * 10))),
        to_int16(int(round(z * 10))),
        to_int16(int(round(rx * 10))),  # deg → deg×10
        to_int16(int(round(ry * 10))),
        to_int16(int(round(rz * 10))),
    ]
    client.write_registers(REG_X, pose_data)

    # 명령 전송
    client.write_registers(REG_COMMAND, [cmd])
    print(f"명령 {cmd} 전송 (x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz})")

    return wait_for_completion(client)


def print_menu():
    print("\n" + "=" * 50)
    print("Main_task.prs 테스트 메뉴")
    print("=" * 50)
    print(" 1: Go Home (var.p(100)으로 이동)")
    print(" 2: Error 테스트")
    print("10: X축 이동 (상대)")
    print("11: Y축 이동 (상대)")
    print("12: Z축 이동 (상대)")
    print("13: XYZ 동시 이동 (상대)")
    print("14: Rx 회전")
    print("15: Ry 회전")
    print("16: Rz 회전")
    print("17: RxRyRz 동시 회전")
    print("20: 절대 위치 이동 (6DOF)")
    print("30: Gripper Ungrip (열기)")
    print("31: Gripper Grip (닫기)")
    print("32: Gripper 전체 시퀀스")
    print(" r: 현재 상태 읽기")
    print(" q: 종료")
    print("=" * 50)


def main():
    print(f"로봇 연결 중... ({ROBOT_IP}:{ROBOT_PORT})")
    client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT, timeout=1.0)

    if not client.connect():
        print("연결 실패!")
        return

    print("연결 성공!")

    try:
        while True:
            print_menu()
            cmd = input("\n선택: ").strip().lower()

            if cmd == 'q':
                break

            elif cmd == 'r':
                # 상태 읽기
                rr = client.read_holding_registers(REG_COMMAND, 2)
                if not rr.isError():
                    print(f"command(351)={rr.registers[0]}, status(352)={rr.registers[1]}")
                else:
                    print("읽기 실패")

            elif cmd == '1':
                send_command(client, 1)

            elif cmd == '2':
                send_command(client, 2)

            elif cmd == '10':
                x = float(input("X 이동량 (mm): "))
                send_command(client, 10, x=x)

            elif cmd == '11':
                y = float(input("Y 이동량 (mm): "))
                send_command(client, 11, y=y)

            elif cmd == '12':
                z = float(input("Z 이동량 (mm): "))
                send_command(client, 12, z=z)

            elif cmd == '13':
                x = float(input("X 이동량 (mm): "))
                y = float(input("Y 이동량 (mm): "))
                z = float(input("Z 이동량 (mm): "))
                send_command(client, 13, x=x, y=y, z=z)

            elif cmd == '14':
                rx = float(input("Rx 회전량 (deg): "))
                send_command(client, 14, rx=rx)

            elif cmd == '15':
                ry = float(input("Ry 회전량 (deg): "))
                send_command(client, 15, ry=ry)

            elif cmd == '16':
                rz = float(input("Rz 회전량 (deg): "))
                send_command(client, 16, rz=rz)

            elif cmd == '17':
                rx = float(input("Rx 회전량 (deg): "))
                ry = float(input("Ry 회전량 (deg): "))
                rz = float(input("Rz 회전량 (deg): "))
                send_command(client, 17, rx=rx, ry=ry, rz=rz)

            elif cmd == '20':
                print("절대 위치 입력 (mm, deg):")
                x = float(input("  X (mm): "))
                y = float(input("  Y (mm): "))
                z = float(input("  Z (mm): "))
                rx = float(input("  Rx (deg): "))
                ry = float(input("  Ry (deg): "))
                rz = float(input("  Rz (deg): "))
                send_command(client, 20, x=x, y=y, z=z, rx=rx, ry=ry, rz=rz)

            elif cmd == '30':
                send_command(client, 30)

            elif cmd == '31':
                send_command(client, 31)

            elif cmd == '32':
                send_command(client, 32)

            else:
                print("잘못된 입력입니다.")

    finally:
        client.close()
        print("\n연결 해제됨")


if __name__ == "__main__":
    main()
