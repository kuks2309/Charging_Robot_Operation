#!/usr/bin/env python3
"""
status 레지스터 실시간 모니터링

사용법:
  python3 /home/amap/Project/KAIST/Charging_Robot/scripts/debug_status.py
"""

import time
from pymodbus.client import ModbusTcpClient

ROBOT_IP = "192.168.0.29"
ROBOT_PORT = 1502

client = ModbusTcpClient(ROBOT_IP, port=ROBOT_PORT, timeout=1.0)
client.connect()

print("status(352) 모니터링 시작... (Ctrl+C로 종료)")
print("0=Idle, 1=Running, 2=Done, 3=Error")
print("-" * 40)

try:
    prev_status = -1
    while True:
        rr = client.read_holding_registers(address=351, count=2)
        if not rr.isError():
            cmd = rr.registers[0]
            status = rr.registers[1]
            if status != prev_status or cmd != 0:
                print(f"command={cmd}, status={status}")
                prev_status = status
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\n종료")
finally:
    client.close()
