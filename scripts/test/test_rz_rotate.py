#!/usr/bin/env python3
"""
Rz 회전 테스트: 현재 99.627도에서 90도로 변경 (-9.6도 상대 회전)
"""

import time
from pymodbus.client import ModbusTcpClient

ROBOT_IP = "192.168.0.29"
ROBOT_PORT = 1502

REG_RZ = 306
REG_COMMAND = 351
REG_STATUS = 352


def to_int16(value):
    """int16 → uint16 변환"""
    if value < 0:
        return int(value + 65536)
    return int(value)


def main():
    print(f"로봇 연결 중... ({ROBOT_IP}:{ROBOT_PORT})")
    client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT, timeout=1.0)

    if not client.connect():
        print("연결 실패!")
        return

    print("연결 성공!")

    try:
        # 현재 상태 확인
        rr = client.read_holding_registers(address=REG_COMMAND, count=2)
        if not rr.isError():
            print(f"현재 상태: command={rr.registers[0]}, status={rr.registers[1]}")

        # Rz 상대 회전: -9.6도 (현재 99.627 → 90도)
        rz_deg = -9.6
        rz_scaled = int(rz_deg * 10)  # deg×10 = -96
        rz_reg = to_int16(rz_scaled)

        print(f"\nRz 회전: {rz_deg}도 (레지스터 값: {rz_reg})")

        # Rz 레지스터에 쓰기
        client.write_registers(REG_RZ, [rz_reg])
        print(f"레지스터 306 (Rz) = {rz_reg} 전송 완료")

        # command 16 (Rz 회전) 전송
        client.write_registers(REG_COMMAND, [16])
        print("command=16 (Rz 회전) 전송 완료")

        # 완료 대기
        print("로봇 동작 대기 중", end="")
        for _ in range(100):  # 10초 타임아웃
            time.sleep(0.1)
            rr = client.read_holding_registers(address=REG_STATUS, count=1)
            if rr.isError():
                print("\n상태 읽기 실패")
                break
            status = rr.registers[0]
            if status == 2:  # Done
                print("\n완료!")
                break
            elif status == 3:  # Error
                print("\n오류 발생!")
                break
            elif status == 1:  # Running
                print(".", end="", flush=True)
        else:
            print("\n타임아웃")

        # 최종 상태 확인
        rr = client.read_holding_registers(address=REG_COMMAND, count=2)
        if not rr.isError():
            print(f"최종 상태: command={rr.registers[0]}, status={rr.registers[1]}")

    finally:
        client.close()
        print("\n연결 해제됨")


if __name__ == "__main__":
    main()
