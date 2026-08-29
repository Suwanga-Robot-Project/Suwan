"""
특정 모터가 "왜 안 움직이는지" 직접 확인하는 진단 도구.

Present Position(추측성 판단)만 보는 대신, 서보가 직접 보고하는
Load(부하)/Temperature(온도)/Voltage(전압)를 함께 읽어서 확정적으로 판단.

  - Load가 최대치까지 치솟음        → 기구적으로 막혀서 스톨(과부하) = 확정
  - Load는 낮은데 위치가 안 바뀜   → Present Position 읽기 자체가 잘못됨(주소 문제) = 확정
  - Temperature가 계속 오름         → 막힌 채로 계속 힘주고 있다는 방증

STS/SCS 서보 표준 레지스터 주소(추정, 이전 tilt_status_diagnostic.py와 동일 체계):
  Present Position    : 56 (2byte)
  Present Speed        : 58 (2byte)
  Present Load          : 60 (2byte)
  Present Voltage      : 62 (1byte)
  Present Temperature   : 63 (1byte)
  Moving 상태            : 66 (1byte, 1=움직이는 중, 0=정지)
"""

import time
from scservo_sdk import *

DEVICENAME_RIGHT = "COM14"  # 문제되는 모터가 오른팔이면 이거, 왼팔이면 COM12로 바꾸세요
BAUDRATE = 1000000
PROTOCOL_END = 0

ADDR_GOAL_POSITION = 42
ADDR_TORQUE_ENABLE = 40
TORQUE_ENABLE = 1
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_MOVING = 66

TARGET_MOTOR_ID = 10  # ⚠️ 문제되는 모터 ID로 바꾸세요 (오른팔 2번째 = 10번)


def read2(portHandler, packetHandler, motor_id, addr):
    val, result, error = packetHandler.read2ByteTxRx(portHandler, motor_id, addr)
    return val if result == COMM_SUCCESS else None


def read1(portHandler, packetHandler, motor_id, addr):
    val, result, error = packetHandler.read1ByteTxRx(portHandler, motor_id, addr)
    return val if result == COMM_SUCCESS else None


def print_status(portHandler, packetHandler, motor_id, label=""):
    pos = read2(portHandler, packetHandler, motor_id, ADDR_PRESENT_POSITION)
    load = read2(portHandler, packetHandler, motor_id, ADDR_PRESENT_LOAD)
    voltage = read1(portHandler, packetHandler, motor_id, ADDR_PRESENT_VOLTAGE)
    temp = read1(portHandler, packetHandler, motor_id, ADDR_PRESENT_TEMPERATURE)
    moving = read1(portHandler, packetHandler, motor_id, ADDR_MOVING)

    # Load는 STS 계열에서 부호+크기 비트(최상위 비트가 방향)인 경우가 많음 — 크기만 추출
    load_magnitude = (load & 0x3FF) if load is not None else None

    print(
        f"{label:12s} pos={pos!s:>6} load={load_magnitude!s:>5} "
        f"voltage={voltage!s:>4} temp={temp!s:>4}°C moving={moving}"
    )


def main():
    portHandler = PortHandler(DEVICENAME_RIGHT)
    packetHandler = PacketHandler(PROTOCOL_END)

    if not portHandler.openPort():
        print(f"포트({DEVICENAME_RIGHT}) 열기 실패")
        return
    portHandler.setBaudRate(BAUDRATE)

    # ===== 결정적으로 빠져있던 부분: 토크 활성화 =====
    # 이게 없으면 goal position 명령을 줘도 서보가 완전히 무시함
    # (힘없이 축만 풀려있는 상태 — pos 안 바뀌고 load도 안 오르는 지금 증상과 정확히 일치)
    result, error = packetHandler.write1ByteTxRx(
        portHandler, TARGET_MOTOR_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
    )
    if result != COMM_SUCCESS:
        print(
            f">>> [경고] 모터{TARGET_MOTOR_ID}번 토크 활성화 실패: {packetHandler.getTxRxResult(result)}"
        )
    else:
        print(f">>> 모터{TARGET_MOTOR_ID}번 토크 활성화 완료")

    print(f">>> 모터{TARGET_MOTOR_ID}번 진단 시작 (Ctrl+C로 종료)")
    print(
        ">>> 목표 tick을 입력하면 그쪽으로 명령을 보내고, 5초간 부하/온도를 관찰합니다.\n"
    )

    print_status(portHandler, packetHandler, TARGET_MOTOR_ID, "[현재 상태]")

    try:
        while True:
            raw = input(
                "\n목표 tick 입력 (그냥 엔터=현재상태만 확인, q=종료): "
            ).strip()
            if raw.lower() == "q":
                break

            if raw:
                try:
                    target = int(raw)
                except ValueError:
                    print("숫자를 입력하세요.")
                    continue

                packetHandler.write2ByteTxRx(
                    portHandler, TARGET_MOTOR_ID, ADDR_GOAL_POSITION, target
                )
                print(
                    f">>> 모터{TARGET_MOTOR_ID}번에 {target} 명령 전송, 5초간 관찰..."
                )

                for i in range(10):
                    time.sleep(0.5)
                    print_status(
                        portHandler,
                        packetHandler,
                        TARGET_MOTOR_ID,
                        f"  t={i*0.5+0.5:.1f}s",
                    )
            else:
                print_status(portHandler, packetHandler, TARGET_MOTOR_ID, "[현재 상태]")

    except KeyboardInterrupt:
        pass

    portHandler.closePort()
    print("\n=== 종료 ===")
    print("\n[판정 기준]")
    print("  - load 값이 계속 높게(예: 500~1000 근처) 유지되고 pos가 안 바뀜")
    print("    → 기구적으로 막혀서 스톨(과부하) 확정. 배선/기구 점검 필요")
    print("  - load는 낮은데(0~100) pos가 명령과 안 맞음")
    print(
        "    → Present Position 주소(56)가 잘못됐거나 다른 문제. 통신/주소 재검증 필요"
    )
    print("  - temp가 계속 상승")
    print("    → 막힌 채로 계속 힘주고 있다는 증거 (이전 팬틸트 사례와 동일 패턴)")


if __name__ == "__main__":
    main()
