# ROS 2 Topic Visualization with Rerun (Python)

`p13_rerun_bridge`는 Python으로 작성된 ROS 2 노드로, P13 계열 차량에서 사용하는 주요 토픽을 [Rerun](https://www.rerun.io/) Viewer에 실시간으로 전달합니다. 센서 계열 메시지는 3D 시각화로, 그 외 메시지는 시계열 그래프로 자동 분류합니다.

`P13_NAME` 환경 변수로 차량 접두사(예: `P13`, `P13B`, ...)를 지정하면 `/P13/...`로 고정된 토픽 이름을 해당 값으로 치환합니다.

## 실행 방법

### 사전 준비

로컬 환경에서 노드를 실행하려면 다음 도구가 설치되어 있어야 합니다.

- ROS 2 Humble 이상
- Python 3.10 이상 (ROS 2 기본 Python 버전과 동일하게 맞추는 것을 권장)
- rerun Python SDK (`pip install --user rerun-sdk`)
- Rerun Viewer (`pip install --user rerun-sdk` 설치 시 함께 제공되는 `rerun` CLI)

ROS 2 Humble이 설치되어 있다면 아래 명령으로 워크스페이스를 초기화하고 필요한 Python 패키지를 설치할 수 있습니다.

```bash
source /opt/ros/humble/setup.bash
python3 -m venv .venv         # 선택 사항이지만 권장
source .venv/bin/activate     # 가상환경을 만들었다면 활성화
pip install --upgrade pip
pip install -r requirements.txt  # 필요 패키지를 명시했다면
pip install rerun-sdk         # rerun SDK 설치 (최초 1회)
```

> `requirements.txt`가 없다면 `pip install rerun-sdk`만 실행해도 충분합니다.

### 노드 실행

1. ROS 2 환경을 소싱합니다.
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. (선택) `P13_NAME` 환경 변수를 원하는 차량 이름으로 설정합니다.
   ```bash
   export P13_NAME=P13B
   ```

3. 브리지를 실행합니다.
   ```bash
   python3 src/p13_rerun_bridge.py
   ```

4. 별도 터미널에서 Rerun Viewer를 실행하고, 브리지 노드가 전송하는 데이터를 확인합니다.
   ```bash
   rerun
   ```

Rerun Viewer는 기본적으로 `127.0.0.1:9876` 포트로 접속 대기합니다. 브리지 노드는 동일 포트로 데이터를 전송하므로 추가 설정 없이 시각화를 확인할 수 있습니다. 다른 포트를 사용하고 싶다면 `RERUN_SERVER` 환경 변수를 변경하세요.

## 환경 변수 요약

| 변수 | 설명 | 기본값 |
| ---- | ---- | ------ |
| `P13_NAME` | `/P13/` 접두사를 대체할 차량 이름 | `P13` |
| `RERUN_MODE` | `connect`(기본) 또는 `spawn`. `connect`는 지정한 Viewer 서버에 접속 | `connect` |
| `RERUN_SERVER` | `connect` 모드에서 사용할 Rerun 서버 주소 | `127.0.0.1:9876` |
| `ROS_WORKSPACE` | 필요 시 추가로 소싱할 ROS 2 오버레이 워크스페이스 경로 | 빈 문자열 |
| `ROS_DISTRO` | 사용할 ROS 2 배포판 | `humble` |

## 문제 해결

- **Viewer가 자동으로 열리지 않는 경우**: `rerun` CLI가 PATH에 있는지 확인하고 별도 터미널에서 직접 실행합니다.
- **ROS 토픽이 보이지 않을 때**: 브리지 노드와 동일한 ROS 2 도메인(`ROS_DOMAIN_ID`)을 사용하고 있는지 확인합니다.
- **원격 머신에서 Viewer를 실행할 때**: `RERUN_SERVER` 환경 변수를 Viewer가 실행 중인 호스트의 주소로 바꿉니다. 예) `export RERUN_SERVER=192.168.0.10:9876`

Docker 관련 파일과 스크립트는 더 이상 제공하지 않으며, 모든 작업은 로컬 환경에서 직접 수행합니다.
