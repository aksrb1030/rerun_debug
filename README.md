# ROS 2 Topic Visualization with Rerun

`p13_rerun_bridge`는 C++로 작성된 ROS 2 노드로, P13 계열 차량에서 사용하는 주요 토픽을 [Rerun](https://www.rerun.io/) Viewer에 실시간으로 전달합니다. 센서 계열 메시지(`sensor_msgs`)는 3D 시각화로, 그 외 메시지는 시계열 그래프로 자동 분류합니다.

`P13_NAME` 환경 변수로 차량 접두사(예: `P13`, `P13B`, ...)를 지정하면 `/P13/...`로 고정된 토픽 이름을 해당 값으로 치환합니다.

## 빌드

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

빌드가 끝나면 실행 파일 `build/p13_rerun_bridge`가 생성됩니다.

## 실행

```bash
source /opt/ros/humble/setup.bash
./build/p13_rerun_bridge
```

환경 변수

| 변수 | 설명 | 기본값 |
| ---- | ---- | ------ |
| `P13_NAME` | `/P13/` 접두사를 대체할 차량 이름 | `P13` |
| `RERUN_MODE` | `connect`(기본) 또는 `spawn`. `connect`는 지정한 Viewer 서버에 접속 | `connect` |
| `RERUN_SERVER` | `connect` 모드에서 사용할 Rerun 서버 주소 | `127.0.0.1:9876` |
| `ROS_WORKSPACE` | Docker Compose에서 추가로 소싱할 ROS 2 오버레이 워크스페이스 경로 | 빈 문자열 |

## 요구사항

- ROS 2 Humble 이상 (노드 실행)
- rerun C++ SDK (`rerun::rerun` 라이브러리)
- Rerun Viewer (`rerun` CLI)

## Docker 이미지 빌드 및 실행

ROS 2와 Rerun SDK를 컨테이너에서 바로 사용할 수 있도록 아키텍처별 Dockerfile과 docker-compose 파일을 제공합니다. 두 Dockerfile은
공식 `ros:${ROS_DISTRO}-ros-base` 이미지를 기반으로 하여 `/opt/ros/${ROS_DISTRO}` 환경이 즉시 구성된 상태에서 빌드가 진행됩니다.

### AMD64 (x86_64)

```bash
docker build -f docker/Dockerfile.amd64 -t rerun-p13-bridge:amd64 .
```

### ARM64 (aarch64)

```bash
docker build -f docker/Dockerfile.arm64 -t rerun-p13-bridge:arm64 .
```

이미지를 실행하려면 다음과 같이 실행합니다.

```bash
docker run --rm --net=host \
  -e P13_NAME=P13B \
  -e RERUN_MODE=connect \
  -e RERUN_SERVER=host.docker.internal:9876 \
  rerun-p13-bridge:amd64
```

ARM64 이미지는 태그만 `arm64`로 바꾸면 동일한 명령으로 실행할 수 있습니다.

### Docker Compose

AMD64

```bash
docker compose -f docker/docker-compose.amd64.yml up --build
```

ARM64

```bash
docker compose -f docker/docker-compose.arm64.yml up --build
```

Compose 실행 시 `P13_NAME`, `RERUN_MODE`, `RERUN_SERVER`, `ROS_WORKSPACE`, `ROS_DISTRO` 등의 환경 변수를 `.env` 파일이나 셸 변수로 전달해 동작을 조정할 수 있습니다.

### Docker 컨테이너 안에서 Rerun Viewer 실행

GUI 애플리케이션인 Rerun Viewer를 Docker 안에서 실행하려면, 호스트의 디스플레이 서버에 접근할 수 있도록 몇 가지 추가 설정이 필요합니다.

1. 호스트에서 X11 접근 권한을 부여합니다 (Linux 기준).
   ```bash
   xhost +local:docker
   ```

2. Viewer를 실행할 때는 다음과 같이 디스플레이 소켓을 공유하고 `DISPLAY` 변수를 전달합니다.
   ```bash
   docker run --rm \
     -e DISPLAY=$DISPLAY \
     -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
     rerun-p13-bridge:amd64 rerun
   ```

   위 명령은 컨테이너 안에서 `rerun` CLI를 실행해 Viewer를 띄웁니다. 필요하다면 `--device /dev/dri` 등을 추가로 지정해 GPU 가속을 사용할 수 있습니다.

3. Viewer가 실행된 후, 동일한 이미지를 이용해 브리지 컨테이너를 기동하면 컨테이너 내부에서 데이터를 주고받을 수 있습니다.
   ```bash
   docker run --rm rerun-p13-bridge:amd64
   ```

X11이 아닌 환경(Wayland, macOS, Windows)에서는 각 플랫폼에 맞는 GUI 포워딩 설정이 필요합니다.
