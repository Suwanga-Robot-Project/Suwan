# Suwan
수완(手腕): 고령근로자의 산업현장 안전 제고를 위한 이동형 양팔로봇

## 목표 달성 관리 도구

계획 대비 구현 단계, 목표 충족 여부, 증빙과 다음 조치를 관리합니다.
Python 3.10 이상을 사용합니다. 아래 의존성 안내는 목표 관리 도구에 해당하며,
기존 로봇 제어·펌웨어 실행 환경은 각 모듈의 조건을 따릅니다.

### 설치와 실행

저장소 최상위 폴더에서 가상환경을 생성합니다.

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirement.txt
python tools/goal_tracker/goal_tracker.py report
```

Windows PowerShell에서는 활성화 명령을 `.venv\Scripts\Activate.ps1`로 바꿉니다.
활성화하지 않고 실행하려면 macOS/Linux에서 `.venv/bin/python`을 사용하세요.

생성 결과: [목표 달성 현황표](tools/goal_tracker/achievement_report.md).
현재 외부 패키지는 필요 없으며, 추후 도입할 의존성은 `requirement.txt`에 버전 조건과 함께 추가합니다.
`.venv` 폴더는 Git에서 제외합니다. 환경 종료는 `deactivate`입니다.

### 목표 수정 및 시험

```sh
python tools/goal_tracker/goal_tracker.py update G05 --status assessed --level 0 --attainment not_met --result "가스 센서 미구현 확인" --evidence "확인한 자료와 위치" --as-of 2026-10-29
python tools/goal_tracker/goal_tracker.py report
python -m unittest discover -s tools/goal_tracker -p 'test_*.py' -v
```

위 수정 예시는 실제 확인 결과로 바꿔 사용하세요. 시험은 임시 데이터만 수정합니다.
상세 데이터 구조와 평가 원칙은 [사용 안내](tools/goal_tracker/README.md)를 참고하세요.
