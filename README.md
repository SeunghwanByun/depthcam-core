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
- In the right panel: left drag orbits, right drag pans, wheel adjusts viewing distance.
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

Recording feedback: the footer shows `REC | Saved: N frames | MiB` and the
recording path (long paths are shortened visually). The count advances after
successful frame writes and `fflush`; this is not an `fsync` durability guarantee.
`S` stops recording and retains the final count. File-open/write errors appear as
`RECORDING ERROR`. Viewer `R` saves `recording_YYYYMMDD_HHMMSS.bin` in the process
working directory. Terminal-selected recordings also update this footer.

## Replay a recording and build a VO map

From the repository root (no camera required):

```bash
./build/Youth --playback recording_20261025_163209.bin
```

The left panel replays the recorded RGB/Depth; VO starts automatically and the
right panel accumulates its map. The footer shows replay frames and completed
VO frames. Playback uses recorded millisecond intervals (gaps over one second
are capped); processing can make playback slower than real time. Replay waits
for VO processing instead of dropping frames. At EOF, the final image and map
remain on screen. `P` restarts the selected file with a fresh map, `S` stops
playback and retains the map, `M` exports it to `<recording path>.ply`, and `Esc`
exits. Restart without `--playback` to return to live camera capture.

A recording selected using terminal menu option `3`, or the viewer's `P` key,
also starts VO when its first complete frame arrives. `V` can disable/re-enable
VO. Playback and live camera frames are kept separate.

To process a whole recording without opening a viewer:

```bash
./build/recording_to_map recording_20261025_163209.bin build/output/recording_20261025_163209.ply
```

This validates the frame sizes and EOF marker, processes every frame, and writes
an ASCII PLY with XYZ in metres and RGB colours. The summary reports frame count,
map points, all-zero depth frames, and tracking-lost frames. The current VO uses
approximate Astra intrinsics (scaled from 640x480), stores at most 1,000,000 map
points, and displays up to 300,000 sampled points. It is not full SLAM: sparse
texture, motion blur and tracking loss can produce incomplete or inaccurate
maps; there is no loop closure or global drift correction. Recordings and map
artifacts are local and excluded from Git.

Validation: `python3 tests/test_recording_to_map.py` checks blank-frame handling
and rejects truncated, mismatched-size, missing-EOF and trailing-data recordings.


## Camera-follow navigation

The right panel follows the last accepted VO camera pose by default. It draws a
camera frustum, a translucent image plane using the RGB frame that belongs to
that pose, a cyan trajectory, camera-local XYZ axes and a fixed world orientation
compass. X is red, Y green, Z blue. Orange means tracking is lost: the pose/image
stay at the last accepted frame, rather than pretending a new pose is known.
Coordinates match the existing negated XYZ map convention.

- Left drag: orbit around the followed camera; right drag: screen-space pan.
- Wheel: smooth proportional dolly (0.25–30 m), without scaling map geometry.
- `F`: toggle camera follow / free orbit around the frozen camera anchor.
- `C`: toggle the camera's own viewpoint / external chase view. Camera view hides
  its own frustum and image plane so they cannot obscure the scene.
- `H`: return to the default follow view; `I`: toggle the RGB image plane.
- `V` and `X` still control VO and reset the map.

View transitions are smoothed and mouse sensitivity accounts for viewport size
and distance. VO rejects non-finite PnP motion or steps exceeding 0.5 m / 45 degrees
per frame. This conservative handheld-camera guard avoids catastrophic pose
jumps; fast motion or widely spaced frames can instead report tracking loss.
Poses use SQPnP RANSAC plus Levenberg-Marquardt refinement, which stays stable
on planar scenes such as walls and corridors. After a lost frame, tracking keeps
matching against the last accepted frame for up to 15 frames before re-anchoring.
Map points are added only on keyframes (5 cm / 5 degrees of motion) and
deduplicated on a 3 cm voxel grid, so long recordings keep mapping instead of
filling the point budget with repeated observations.
It does not add SLAM relocalization or repair accumulated drift.

Camera pose/image consistency and lost-tracking/reset behavior are tested by
`ctest --test-dir build --output-on-failure`.
