# 1. Ubuntu 24.04 이미지를 베이스로 설정
FROM ubuntu:24.04

# 2. 필수 패키지 설치
RUN apt update && apt install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    libopencv-dev \
    libglfw3-dev \
    libglew-dev \
    freeglut3-dev \
    mesa-common-dev \
    libglu1-mesa-dev \
    libx11-dev \
    libopenni2-dev \
    libzmq3-dev \
    && rm -rf /var/lib/apt/lists/*

# 3. 작업 디렉토리 설정
WORKDIR /Project/Astra-Drone-SLAM

# 4. 소스 코드 복사
COPY ./ /Project/Astra-Drone-SLAM/

# 5. 소스 코드 컴파일
RUN cmake -S. -Bbuild/Release -DCMAKE_BUILD_TYPE=Release
RUN cmake --build build/Release --parallel

# 6. 프로그램 실행 명령어 설정
CMD ["./build/Release/Youth"]
