# Sensor Visualization with Rerun

이 프로젝트는 [Rerun](https://www.rerun.io/) C++ SDK를 이용해 가상의 센서 데이터를 실시간으로 시각화하는 예제입니다. 온도, 위치, 속도 정보를 생성해 Rerun Viewer로 스트리밍합니다.

## 요구사항

- C++20을 지원하는 컴파일러 (예: GCC 12+, Clang 15+)
- CMake 3.21 이상
- Rerun C++ SDK (패키지 이름: `rerun_sdk`). `vcpkg`, `pip install rerun-sdk`, 또는 직접 빌드한 SDK의 CMake 패키지를 사용해 설치할 수 있습니다.
- Rerun Viewer (`rerun` CLI)

## 빌드 방법

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

`rerun_sdk` 패키지가 표준 CMake 경로에 설치되어 있지 않은 경우 `CMAKE_PREFIX_PATH`를 추가로 지정해야 할 수 있습니다.

예시:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/path/to/rerun-sdk
cmake --build build
```

## 실행 방법

먼저 Rerun Viewer를 실행합니다.

```bash
rerun
```

다음으로 예제 프로그램을 실행하면, 가상의 3개 센서에 대한 위치·속도·온도 정보가 스트리밍되어 Viewer에서 확인할 수 있습니다.

```bash
./build/sensor_visualizer
```

프로그램은 약 300프레임 동안 데이터를 전송한 후 종료됩니다. Viewer에서 센서 위치(`sensors/positions`), 속도 벡터(`sensors/velocity`), 그리고 온도 로그(`sensors/temperatures`)를 확인할 수 있습니다.

## Docker 이미지 빌드

로컬 환경에서 의존성을 직접 설치하고 싶지 않은 경우 제공된 Dockerfile을 이용해 애플리케이션을 빌드할 수 있습니다. 아키텍처별로 Dockerfile이 분리되어 있으니, 사용 중인 플랫폼에 맞는 파일을 선택하세요.

### AMD64 (x86_64)

```bash
docker build -f docker/Dockerfile.amd64 -t rerun-sensor-visualizer:amd64 .
```

### ARM64 (aarch64)

```bash
docker build -f docker/Dockerfile.arm64 -t rerun-sensor-visualizer:arm64 .
```

빌드가 완료되면 다음과 같이 컨테이너를 실행할 수 있습니다.

```bash
docker run --rm rerun-sensor-visualizer:amd64
```

ARM64 이미지도 동일한 명령으로 실행할 수 있으며, 필요에 따라 태그만 `arm64`로 바꿔주면 됩니다.

### Docker 컨테이너 안에서 Rerun Viewer까지 실행하고 싶은 경우

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
     rerun-sensor-visualizer:amd64 rerun
   ```

   위 명령은 컨테이너 안에서 `rerun` CLI를 실행해 Viewer를 띄웁니다. 필요하다면 `--device /dev/dri` 등을 추가로 지정해 GPU 가속을 사용할 수 있습니다.

3. Viewer가 실행된 후, 동일한 이미지를 이용해 센서 스트리머를 기동하면 컨테이너 내부에서 데이터를 주고받을 수 있습니다.
   ```bash
   docker run --rm rerun-sensor-visualizer:amd64 ./build/sensor_visualizer
   ```

`ros2 bag play`와 같은 다른 도구를 호스트에서 실행하면서, 시각화와 데이터 스트리밍을 모두 컨테이너에 격리시키고 싶을 때 유용한 방법입니다. X11이 아닌 환경(Wayland, macOS, Windows)에서는 각 플랫폼에 맞는 GUI 포워딩 설정이 필요합니다.
