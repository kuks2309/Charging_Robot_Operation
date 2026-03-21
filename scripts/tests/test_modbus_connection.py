#!/usr/bin/env python3
"""
Modbus 연결 테스트 - 포트 5001/5002 확인
로봇 IP: 192.168.0.39
"""

import time
import struct
from pymodbus.client import ModbusTcpClient

ROBOT_IP = "192.168.0.39"
PORTS = [1502]


def read_float32(registers, offset):
    """2개 레지스터를 IEEE 754 float32로 변환"""
    low = registers[offset]
    high = registers[offset + 1]
    byte_data = high.to_bytes(2, 'big') + low.to_bytes(2, 'big')
    return struct.unpack('>f', byte_data)[0]


def test_port(ip, port):
    print(f"\n{'='*50}")
    print(f"  포트 {port} 테스트")
    print(f"{'='*50}")

    client = ModbusTcpClient(ip, port=port, timeout=3.0)

    # 1. TCP 연결
    connected = client.connect()
    print(f"[1] TCP 연결: {'성공' if connected else '실패'}")
    if not connected:
        return False

    # 2. 레지스터 읽기 테스트 (여러 주소)
    test_registers = [
        (351, 1, "명령 레지스터"),
        (352, 1, "상태 레지스터"),
        (301, 6, "좌표 레지스터 (301-306)"),
        (158, 12, "TCP Pose (158-169)"),
    ]

    success = False
    for addr, count, name in test_registers:
        try:
            result = client.read_holding_registers(address=addr, count=count)
            if hasattr(result, 'registers'):
                print(f"[2] {name} (addr={addr}): {result.registers}")
                success = True
            else:
                print(f"[2] {name} (addr={addr}): 응답 없음")
        except Exception as e:
            print(f"[2] {name} (addr={addr}): 오류 - {e}")

    # 3. TCP Pose float 변환
    if success:
        try:
            result = client.read_holding_registers(address=158, count=12)
            if hasattr(result, 'registers') and len(result.registers) == 12:
                x = read_float32(result.registers, 0)
                y = read_float32(result.registers, 2)
                z = read_float32(result.registers, 4)
                rx = read_float32(result.registers, 6)
                ry = read_float32(result.registers, 8)
                rz = read_float32(result.registers, 10)
                print(f"\n[3] TCP Position:")
                print(f"    X={x:8.2f}  Y={y:8.2f}  Z={z:8.2f}")
                print(f"    Rx={rx:7.2f}  Ry={ry:7.2f}  Rz={rz:7.2f}")
        except Exception:
            pass

    # 4. 쓰기 테스트 (상태 읽기만 - 안전)
    if success:
        try:
            result = client.read_holding_registers(address=352, count=1)
            if hasattr(result, 'registers'):
                status = result.registers[0]
                status_map = {0: "Idle", 1: "Running", 2: "Done", 3: "Error"}
                print(f"\n[4] 로봇 상태: {status} ({status_map.get(status, '알 수 없음')})")
        except Exception:
            pass

    client.close()
    print(f"\n>> 포트 {port}: {'Modbus 통신 성공' if success else 'Modbus 응답 없음'}")
    return success


def main():
    print(f"로봇 Modbus 연결 테스트")
    print(f"IP: {ROBOT_IP}")
    print(f"테스트 포트: {PORTS}")

    results = {}
    for port in PORTS:
        results[port] = test_port(ROBOT_IP, port)

    # 요약
    print(f"\n{'='*50}")
    print(f"  결과 요약")
    print(f"{'='*50}")
    for port, ok in results.items():
        print(f"  포트 {port}: {'통신 성공' if ok else '응답 없음'}")

    if not any(results.values()):
        print("\n  모든 포트에서 Modbus 응답 없음!")
        print("  확인사항:")
        print("  1. 로봇 티치펜던트에서 Main_task.prs 실행 중인지")
        print("  2. 모드버스 서버 '시작' 버튼 눌렀는지")
        print("  3. 외부 제어 모드 활성화 여부")


if __name__ == "__main__":
    main()
