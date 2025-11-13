# ROS 2 Topic Visualization with Rerun (Python)

`p13_rerun_bridge`는 Python으로 작성된 ROS 2 노드로, P13 계열 차량에서 사용하는 주요 토픽을 [Rerun](https://www.rerun.io/) Viewer에 실시간으로 전달합니다. 센서 계열 메시지는 3D 시각화로, 그 외 메시지는 시계열 그래프로 자동 분류합니다.

## 프로젝트 구조

모듈화된 패키지 구조 덕분에 각 역할을 명확히 구분할 수 있습니다.

```
src/
├── p13_rerun_bridge.py   # 실행 진입점 (CLI)
└── rerun_bridge/
    ├── bridge.py         # ROS 2 노드 구현체
    ├── config.py         # 환경 변수 기반 설정값 로직
    ├── flatten.py        # 메시지 평탄화 유틸리티
    ├── handlers.py       # 메시지 타입별 Rerun 로거
    ├── main.py           # 초기화 및 실행 흐름
    ├── topics.py         # 구독 토픽 정의
    └── utils.py          # 공용 헬퍼
```

각 함수에는 한국어 주석이 포함되어 있어 동작을 쉽게 파악할 수 있습니다. 기본 단위 이름(차량 접두사)은 환경 변수로 변경 가능합니다.

## 실행 전 준비 사항

로컬 환경에서 브리지를 실행하려면 다음 도구가 필요합니다.

- ROS 2 Humble 이상
- Python 3.10 이상 (ROS 2에서 사용하는 버전과 동일 권장)
- rerun Python SDK (`pip install --user rerun-sdk`)
- Rerun Viewer (`pip install rerun-sdk` 설치 시 제공되는 `rerun` CLI)

권장되는 초기 설정 절차는 아래와 같습니다.

```bash
source /opt/ros/humble/setup.bash
python3 -m venv .venv         # 선택 사항이지만 권장
source .venv/bin/activate     # 가상환경을 만들었다면 활성화
pip install --upgrade pip
pip install rerun-sdk         # rerun SDK 설치 (최초 1회)
```

프로젝트에서 추가 Python 의존성이 있다면 `pip install -r requirements.txt`로 설치하세요.

## 실행 방법

1. ROS 2 환경을 소싱합니다.
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. (선택) 차량 접두사를 지정합니다.
   ```bash
   export P13_NAME=P13B
   ```

3. 브리지를 실행합니다. 진입점 스크립트를 직접 호출하거나 모듈 실행 방식을 사용할 수 있습니다.
   ```bash
   python3 src/p13_rerun_bridge.py
   # 또는
   python3 -m rerun_bridge.main
   ```

4. 별도 터미널에서 Rerun Viewer를 실행해 ROS 데이터를 확인합니다.
   ```bash
   rerun
   ```

기본적으로 브리지는 `127.0.0.1:9876`에서 실행 중인 Viewer에 `connect` 모드로 연결합니다. 다른 포트나 호스트를 사용하려면 `RERUN_MODE`와 `RERUN_SERVER` 환경 변수를 조정하세요.

## 주요 환경 변수

| 변수 | 설명 | 기본값 |
| ---- | ---- | ------ |
| `P13_NAME` | `/P13/` 접두사를 대체할 차량 이름 | `P13` |
| `RERUN_MODE` | `connect` 또는 `spawn`. `spawn`은 Viewer를 자동 실행 | `connect` |
| `RERUN_SERVER` | `connect` 모드에서 사용할 Rerun 서버 주소 | `127.0.0.1:9876` |

## 문제 해결

- **Viewer가 자동으로 열리지 않는 경우**: `rerun` CLI가 PATH에 있는지 확인하고 별도 터미널에서 직접 실행합니다.
- **ROS 토픽이 보이지 않을 때**: 브리지 노드와 동일한 ROS 2 도메인(`ROS_DOMAIN_ID`)을 사용하고 있는지 확인합니다.
- **원격 머신에서 Viewer를 실행할 때**: `RERUN_SERVER`를 Viewer가 실행 중인 호스트 주소로 바꿉니다. 예) `export RERUN_SERVER=192.168.0.10:9876`

Docker 관련 파일과 스크립트는 제공하지 않으므로 로컬 환경에서 직접 실행하는 구성을 권장합니다.
