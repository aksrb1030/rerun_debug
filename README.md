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
