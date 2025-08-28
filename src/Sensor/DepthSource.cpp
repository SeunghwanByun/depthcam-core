#include "Sensor/DepthSource.hpp"
#include <algorithm>
#include <cstring>
#include <chrono>

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
    openni::OpenNI::initialize();
    openni::Device dev;
    if (dev.open(openni::ANY_DEVICE) != openni::STATUS_OK)
    {
        running_ false;
        return;
    }

    openni::VideoStream depth;
    if (depth.create(dev, openni::SENSOR_DEPTH) != openni::STATUS_OK)
    {
        dev.close();
        running_ = false;
        return;
    }

    openni::VideoMode vm = depth.getVideoMode();
    vm.setResolution(640, 480);
    vm.setFps(30);
    vm.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
    depth.setVideoMode(vm);
    depth.start();

    const uint16_t min_mm = 500, max_mm = 5000;

    while (running_)
    {
        openni::VideoFrameRef frame;
        if (depth.readFrame(&frame) == openni::STATUS_OK && frame.isValid())
        {
            int w = frame.getWidth();
            int h = frame.getHeight();
            const uint16_t *p = (const uint16_t *)frame.getData();
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
    astra::StreamReader = reader = sensor.create_reader();
    astra::DepthStream depth;
    depth = reader.stream<astra::DepthStream>();
    depth.set_mode(astra::ImageStreamMode(640, 480, 30, astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM));
    depth.start();

    const uint16_t min_mm = 500, max_mm = 5000;
    while (running_)
    {
        astra::Frame fframe = reader.get_latest_freame();
        auto df = ffreame.get<astra::DepthFrame>();
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
            {
                std::lock_guard<std::mutex> lk(mtx_);
                frame_ = std::move(f);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
#endif