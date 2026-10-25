# SLAM-RGBD

## Prerequisite
```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config \
  libopencv-dev \
  libglfw3-dev libglew-dev freeglut3-dev mesa-common-dev libglu1-mesa-dev libx11-dev \
  libopenni2-dev \
  libzmq3-dev
```

`libzmq3-dev` is optional. If it is not installed, the project builds with the
ZMQ publisher stub disabled.

## Build & Execution (Ubuntu)
```bash
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
cmake --build . --parallel
./Youth # ESC 로 종료
```


This repository will include source code about indoor SLAM algorithm using astra-depth-camera.

This code flow will be like under explanation.

1. First, sensor module will be connect with variable sensors. And send data buffer to the logging module. So, thread will be executed in infinite while loop.

2. Second, logging module receive that data buffer and toss it algorithm module.

3. Third, after algorithm will be exectued, then send some sort of result of algorithms like slam, etc.. to the viewer module.

4. Finally, the viewer module will be send the result including raw data. And the viewer module will visualize that results in GUI using OpenGL.

## Split RGB-D workspace

The left panel shows live RGB (`1`) or a depth colour preview (`2`, 0–5 m;
black means invalid depth). Images retain their aspect ratio. The right panel
is reserved for processed 3D results and starts in **Waiting for SLAM backend**.
The FPS counter measures received complete RGB-D frames.

- `V`: toggle the existing experimental visual odometry worker (not full SLAM).
- `X`: reset its accumulated map.
- In the right panel: left drag pans, right drag rotates, wheel zooms.
- `R` / `S`: start / stop recording; `P`: play the last viewer-created recording.
- `Esc`: exit. Terminal recording controls remain available.

A future algorithm worker can call `viewerPublishSlamResult()` in
`ViewerModule/viewerModule.h` to publish a copied snapshot of map points,
trajectory vertices, status and processing FPS. It requires no OpenGL context.
Positions use metres in OpenGL coordinates (+X right, +Y up, forward -Z),
colours use floats in [0,1]. Limits are 300,000 map points and 10,000 trajectory
vertices; invalid sizes are rejected. Empty arrays with a null status clear
the result. Stop the experimental VO preview before connecting another backend.
Camera calibration, RGB/depth registration and a complete SLAM backend are
not implemented by this viewer change.
