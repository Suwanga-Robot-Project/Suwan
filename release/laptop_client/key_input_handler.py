"""
키캡 눌림 상태(key1~key5)를 받아서 왼팔/오른팔 각각의 목표 스테이션으로 변환한다.

확정된 매핑:
  1번 = 오른팔 미세그리퍼 (fine)
  2번 = 오른팔 니퍼그리퍼 (nipper)
  3번 = 왼팔 기본(디폴트)그리퍼 (default)
  4번 = 왼팔 바이스그리퍼 (vise)
  5번 = 전체 탈거 (양팔 다 target_gripper=None, 빈손으로)

⚠️ 이 함수는 "실시간으로 계속 눌려있는지"만 판단하고, "새로 눌린 순간(엣지)"인지는
   판단하지 않는다. 엣지 감지는 호출부(run_gripper_changer.py)에서 처리한다 —
   그래야 버튼을 계속 누르고 있어도 한 번만 실행된다.
"""

# 전체탈거(5번) 전용 특수 타겟값. tool_changer_fsm.ArmSwapFSM.update()에서
# 이 값이 들어오면 "빈손(target_gripper=None)으로 가라"는 뜻으로 해석함.
# (파이썬 None은 "이번 프레임에 새 입력 없음"이라는 뜻으로 이미 쓰이고 있어서,
#  "명시적으로 빈손 만들기"는 별도 값으로 구분해야 함)
DROP_ALL = "__drop_all__"

KEY_TO_STATION = {
    1: ("right", "fine"),
    2: ("right", "nipper"),
    3: ("left", "default"),
    4: ("left", "vise"),
}


def parse_key_input(key1, key2, key3, key4, key5):
    """
    각 인자는 bool (True=눌림, 이번 순간 상태).
    반환: (left_target, right_target)
      - 해당 팔에 눌린 키가 없으면 None
      - 5번(전체탈거)이 눌리면 양팔 다 DROP_ALL
    """
    if key5:
        return DROP_ALL, DROP_ALL

    pressed_keys = []
    if key1:
        pressed_keys.append(1)
    if key2:
        pressed_keys.append(2)
    if key3:
        pressed_keys.append(3)
    if key4:
        pressed_keys.append(4)

    left_target = None
    right_target = None
    for k in pressed_keys:
        arm_side, station = KEY_TO_STATION[k]
        if arm_side == "left" and left_target is None:
            left_target = station
        elif arm_side == "right" and right_target is None:
            right_target = station

    return left_target, right_target
