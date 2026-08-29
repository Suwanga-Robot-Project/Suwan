import socket

PI_HOST = "192.168.1.104"  # ⚠️ 실제 라즈베리파이 IP로 확인/수정 필요
PI_PORT = 5006
SOCKET_TIMEOUT = 15  # 초 — 하강이 오래 걸릴 수 있어 넉넉하게
CONNECT_CHECK_TIMEOUT = 1.5  # 초 — import 시점 빠른 연결 확인용 (짧게)


def _check_reachable():
    """
    모듈을 import하는 시점에 라파가 실제로 켜져있고 응답 가능한지 짧게 확인.
    여기서 실패하면 예외를 던져서, tool_changer_fsm.py의 import가 실패하게 만들고
    → 자동으로 lift_control_sim(가짜)으로 넘어가게 함.
    이게 없으면 라파가 꺼져있어도 이 모듈 import 자체는 그냥 성공해버려서,
    실제 하강을 시도하는 순간에야 연결 에러가 나며 멈추게 됨 (더 늦게 발견됨).
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(CONNECT_CHECK_TIMEOUT)
        s.connect((PI_HOST, PI_PORT))  # 여기서 실패하면 예외 발생 → import 실패


_check_reachable()  # 모듈 로드 시점에 바로 확인


def _send_command(command: str) -> str:
    """라파의 lift_server.py에 명령 전송하고 응답 받기."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(SOCKET_TIMEOUT)
        s.connect((PI_HOST, PI_PORT))
        s.sendall(command.encode("utf-8"))
        response = s.recv(1024).decode("utf-8")
    return response


def descend_until_bottom_switch():
    """하단 리미트 스위치가 눌릴 때까지 하강(라파에 원격 요청). 걸린 시간(초)을 반환."""
    print(f">>> [원격] 라파({PI_HOST})에 하강 명령 전송...")
    response = _send_command("DESCEND")
    if response.startswith("OK:"):
        elapsed = float(response.split(":", 1)[1])
        print(f">>> [원격] 하강 완료 (경과 {elapsed:.2f}초)")
        return elapsed
    else:
        raise RuntimeError(f"라파 하강 실패: {response}")


def ascend_full(descend_seconds):
    """descend_seconds만큼 상승(라파에 원격 요청)."""
    print(f">>> [원격] 라파에 {descend_seconds:.2f}초 상승 명령 전송...")
    response = _send_command(f"ASCEND:{descend_seconds}")
    if response != "OK":
        raise RuntimeError(f"라파 상승 실패: {response}")
    print(">>> [원격] 상승 완료")


def ascend_clearance(seconds):
    """짧게 살짝만 상승(라파에 원격 요청)."""
    print(f">>> [원격] 라파에 {seconds:.2f}초 클리어런스 상승 명령 전송...")
    response = _send_command(f"CLEARANCE:{seconds}")
    if response != "OK":
        raise RuntimeError(f"라파 클리어런스 실패: {response}")
    print(">>> [원격] 클리어런스 완료")


if __name__ == "__main__":
    # 단독 실행 시 연결 테스트
    t = descend_until_bottom_switch()
    ascend_full(t)
