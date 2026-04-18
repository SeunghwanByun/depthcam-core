#include "Sensor/DepthSource.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>

using namespace Youth::Sensor;

static inline uint8_t normalizeDepth(uint16_t mm, uint16_t min_mm, uint16_t max_mm)
{
    if (mm <= min_mm)
        return 0;
    if (mm > max_mm)
        return 255;

    return (uint8_t)((mm - min_mm) * 255 / (max_mm - min_mm));
}

bool DepthSource::start()
{
    if (running_)
        return true;
    running_ = true;

    th_ = std::thread(&DepthSource::run, this);
    return true;
}

void DepthSource::stop()
{
    running_ = false;
    if (th_.joinable())
        th_.join();
}

bool DepthSource::latestFrame(Frame &out)
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (frame_.width == 0 || frame_.height == 0)
        return false;
    out = frame_;
    return true;
}

// ====== BACKEND 1 : OpenNI2 ======
#if defined(USE_OPENNI2)
#include <OpenNI.h>
void DepthSource::run()
{
    const auto initStatus = openni::OpenNI::initialize();
    if (initStatus != openni::STATUS_OK)
    {
        std::cerr << "[DepthSource] OpenNI2 init failed: "
                  << openni::OpenNI::getExtendedError() << '\n';
        running_ = false;
        return;
    }

    openni::Device dev;
    if (dev.open(openni::ANY_DEVICE) != openni::STATUS_OK)
    {
        std::cerr << "[DepthSource] Failed to open depth device: "
                  << openni::OpenNI::getExtendedError() << '\n';
        running_ = false;
        openni::OpenNI::shutdown();
        return;
    }

    const auto &deviceInfo = dev.getDeviceInfo();
    std::cout << "[DepthSource] Connected device: "
              << deviceInfo.getVendor() << ' '
              << deviceInfo.getName() << " ("
              << deviceInfo.getUri() << ")\n";

    openni::VideoStream depth;
    if (depth.create(dev, openni::SENSOR_DEPTH) != openni::STATUS_OK)
    {
        std::cerr << "[DepthSource] Failed to create depth stream: "
                  << openni::OpenNI::getExtendedError() << '\n';
        dev.close();
        running_ = false;
        openni::OpenNI::shutdown();
        return;
    }

    openni::VideoMode vm = depth.getVideoMode();
    vm.setResolution(640, 480);
    vm.setFps(30);
    vm.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
    if (depth.setVideoMode(vm) != openni::STATUS_OK)
    {
        std::cerr << "[DepthSource] Failed to set depth video mode: "
                  << openni::OpenNI::getExtendedError() << '\n';
        depth.destroy();
        dev.close();
        running_ = false;
        openni::OpenNI::shutdown();
        return;
    }

    if (depth.start() != openni::STATUS_OK)
    {
        std::cerr << "[DepthSource] Failed to start depth stream: "
                  << openni::OpenNI::getExtendedError() << '\n';
        depth.destroy();
        dev.close();
        running_ = false;
        openni::OpenNI::shutdown();
        return;
    }

    const uint16_t min_mm = 500, max_mm = 5000;
    bool firstFrameLogged = false;

    while (running_)
    {
        openni::VideoFrameRef depthFrame;
        if (depth.readFrame(&depthFrame) == openni::STATUS_OK && depthFrame.isValid())
        {
            const int w = depthFrame.getWidth();
            const int h = depthFrame.getHeight();
            const auto *p = static_cast<const uint16_t *>(depthFrame.getData());
            Frame f;
            f.width = w;
            f.height = h;
            f.intensity.resize(w * h);
            for (int i = 0; i < w * h; ++i)
            {
                f.intensity[i] = normalizeDepth(p[i], min_mm, max_mm);
            }
            {
                std::lock_guard<std::mutex> lk(mtx_);
                frame_ = std::move(f);
            }

            if (!firstFrameLogged)
            {
                std::cout << "[DepthSource] First depth frame received: "
                          << w << 'x' << h << '\n';
                firstFrameLogged = true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    depth.stop();
    depth.destroy();
    dev.close();
    openni::OpenNI::shutdown();
}

// ====== BACKEND 2 : Orbbec Astra SDK ======
#elif defined(USE_ASTRA_SDK)
#include <astra/astra.hpp>
void DepthSource::run()
{
    astra::initialize();
    astra::StreamSet sensor;
    auto reader = sensor.create_reader();
    auto depth = reader.stream<astra::DepthStream>();
    depth.set_mode(astra::ImageStreamMode(
        640, 480, 30, ASTRA_PIXEL_FORMAT_DEPTH_MM));
    depth.start();

    const uint16_t min_mm = 500, max_mm = 5000;
    while (running_)
    {
        auto astraFrame = reader.get_latest_frame();
        auto df = astraFrame.get<astra::DepthFrame>();
        if (df.is_valid())
        {
            int w = df.width();
            int h = df.height();
            Frame f;
            f.width = w;
            f.height = h;
            f.intensity.resize(w * h);
            auto *data = (const uint16_t *)df.data();
            for (int i = 0; i < w * h; ++i)
            {
                uint16_t mm = static_cast<uint16_t>(data[i] > 0 ? data[i] : 0);
                f.intensity[i] = normalizeDepth(mm, min_mm, max_mm);
            }
            {
                std::lock_guard<std::mutex> lk(mtx_);
                frame_ = std::move(f);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    astra::terminate();
}

// ====== BACKEND 3 : Dummy (No SDK) ======
#else
void DepthSource::run()
{
    // SDK가 없을 때도 뷰어 파이프라인 테스트할 수 있게 가짜 그라디언트 생성
    const int w = 640, h = 480;
    while (running_)
    {
        Frame f;
        f.width = w;
        f.height = h;
        f.intensity.resize(w * h);
        static int t = 0;
        ++t;
        for (int y = 0; y < h; ++y)
        {
            for (int x = 0; x < w; ++x)
            {
                f.intensity[y * w + x] = static_cast<uint8_t>((x + t) % 256);
            }
        }
        {
            std::lock_guard<std::mutex> lk(mtx_);
            frame_ = std::move(f);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
#endif
