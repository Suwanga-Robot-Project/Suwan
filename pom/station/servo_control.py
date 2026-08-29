"""
실제 서보(SCServo) 제어 모듈 — 왼팔/오른팔이 서로 다른 포트로 연결됨.
tool_changer_fsm.py의 move_arm_to()에서 이 모듈을 사용.

⚠️ _scservo_sdk 폴더가 pom/station의 상위 또는 동일 레벨에 있다고 가정
   (스크린샷 기준: pom/_scservo_sdk, pom/station/*.py)
   폴더 위치가 다르면 아래 sys.path.append 경로를 맞게 수정하세요.
"""

import sys
import os

# _scservo_sdk를 import 가능하게 경로 추가 (pom/_scservo_sdk 기준)
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(_THIS_DIR, ".."))  # pom/ 를 path에 추가
sys.path.append(os.path.join(_THIS_DIR, "..", "_scservo_sdk"))

from scservo_sdk import *  # PortHandler, PacketHandler 등

DEVICENAME_LEFT = "COM12"
DEVICENAME_RIGHT = "COM14"
BAUDRATE = 1000000
PROTOCOL_END = 0

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = (
    56  # ⚠️ 추정값 — 실기로 검증 안 됨, 이상한 값 나오면 이 주소 재확인 필요
)
TORQUE_ENABLE = 1

# tick 리스트의 인덱스(0~6)와 1:1 대응 — [모터1,모터2,...,모터7] 순서 그대로
MOTORS_LEFT = [1, 2, 3, 4, 5, 6, 7]
MOTORS_RIGHT = [9, 10, 11, 12, 13, 14, 15]

portHandlerLeft = PortHandler(DEVICENAME_LEFT)
portHandlerRight = PortHandler(DEVICENAME_RIGHT)
packetHandler = PacketHandler(PROTOCOL_END)

_initialized = False


def _open_port(port_handler, port_name, label):
    if not port_handler.openPort():
        raise RuntimeError(f"{label} 포트({port_name}) 열기 실패")
    if not port_handler.setBaudRate(BAUDRATE):
        raise RuntimeError(f"{label} 보드레이트({BAUDRATE}) 설정 실패")
    print(f">>> {label} 포트({port_name}) 연결 성공")


def init_servos():
    """포트 열기 + 양팔 전체 모터 토크 ON. 프로그램 시작 시 한 번만 호출."""
    global _initialized
    if _initialized:
        return

    _open_port(portHandlerLeft, DEVICENAME_LEFT, "왼팔")
    _open_port(portHandlerRight, DEVICENAME_RIGHT, "오른팔")

    for motor_id in MOTORS_LEFT:
        result, error = packetHandler.write1ByteTxRx(
            portHandlerLeft, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if result != COMM_SUCCESS:
            print(
                f"  [경고] 왼팔 모터{motor_id}번 토크 ON 실패: {packetHandler.getTxRxResult(result)}"
            )

    for motor_id in MOTORS_RIGHT:
        result, error = packetHandler.write1ByteTxRx(
            portHandlerRight, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if result != COMM_SUCCESS:
            print(
                f"  [경고] 오른팔 모터{motor_id}번 토크 ON 실패: {packetHandler.getTxRxResult(result)}"
            )

    print(">>> 양팔 전체 모터 토크 ON 완료")
    _initialized = True


def move_arm_to(arm_side, ticks):
    """
    arm_side: 'left' 또는 'right'
    ticks: [tick1, tick2, ..., tick7] — MOTORS_LEFT/MOTORS_RIGHT 순서와 1:1 대응
    """
    if not _initialized:
        raise RuntimeError("init_servos()를 먼저 호출해야 합니다")

    port_handler = portHandlerLeft if arm_side == "left" else portHandlerRight
    motor_ids = MOTORS_LEFT if arm_side == "left" else MOTORS_RIGHT

    for motor_id, tick in zip(motor_ids, ticks):
        if tick is None:
            continue
        result, error = packetHandler.write2ByteTxRx(
            port_handler, motor_id, ADDR_GOAL_POSITION, int(tick)
        )
        if result != COMM_SUCCESS:
            print(
                f"  [경고] {arm_side} 모터{motor_id}번 이동 실패: "
                f"{packetHandler.getTxRxResult(result)}"
            )


def read_current_ticks(arm_side):
    """
    지금 서보가 실제로 어디 있는지 읽어옴 (Present Position).
    반환: [tick1~tick7] 리스트, 읽기 실패한 모터는 None으로 채움.

    ⚠️ ADDR_PRESENT_POSITION(56)이 추정값이라 이상한 값(예: 음수, 65000대,
       0 근처로 전부 몰림 등)이 나오면 그 주소가 틀렸을 가능성이 큽니다.
       그런 경우 호출부에서 None 처리하거나 NEUTRAL로 대체하는 게 안전합니다.
    """
    if not _initialized:
        raise RuntimeError("init_servos()를 먼저 호출해야 합니다")

    port_handler = portHandlerLeft if arm_side == "left" else portHandlerRight
    motor_ids = MOTORS_LEFT if arm_side == "left" else MOTORS_RIGHT

    ticks = []
    for motor_id in motor_ids:
        pos, result, error = packetHandler.read2ByteTxRx(
            port_handler, motor_id, ADDR_PRESENT_POSITION
        )
        if result != COMM_SUCCESS:
            print(
                f"  [경고] {arm_side} 모터{motor_id}번 현재위치 읽기 실패: "
                f"{packetHandler.getTxRxResult(result)}"
            )
            ticks.append(None)
        else:
            ticks.append(pos)
    return ticks


def close_servos():
    """프로그램 종료 시 포트 정리."""
    portHandlerLeft.closePort()
    portHandlerRight.closePort()
    print(">>> 서보 포트 닫음")


if __name__ == "__main__":
    # 단독 실행 시 연결 테스트만
    init_servos()
    print("연결 테스트 완료. Ctrl+C로 종료하세요.")
