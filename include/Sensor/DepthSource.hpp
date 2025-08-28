#pragma once

#include <cstdint>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>

namespace Youth::Sensor
{
    // 단일 채널(depth mm) 또는 8비트 intensity를 GL R8 텍스처로 넘기자(간단).
    class DepthSource
    {
    public:
        struct Frame
        {
            int width = 0, height = 0;
            // 시각화용 normalized 8-bit (0~1 -> 0~255), GL_R8로 업로드
            std::vector<uint8_t> intensity;
        };

        bool start();
        void stop();

        bool latestFrame(Frame &out); // 최신 프레임 스냅샷

        // 백엔드 선택은 CMake 옵션으로
    private:
        void run();
        std::atomic<bool> running_{false};
        std::thread th_;
        std::mutex mtx_;
        Frame frame_;
    };
}