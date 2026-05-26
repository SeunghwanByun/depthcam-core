// astra_wrapper.cpp

#include "astra_wrapper.h"
#include <iostream>
#include <string>
#include <vector>

#if defined(USE_ASTRA_SDK)
#include <astra/astra.hpp>
#include <astra/capi/astra.h>   // for astra_update()
#elif defined(USE_OPENNI2)
#include <OpenNI.h>
#else
#error "Either USE_ASTRA_SDK or USE_OPENNI2 must be defined for SensorModuleLib."
#endif

// For WebGL
// #include <nlohmann/json.hpp>

extern "C" {

static std::string g_lastError;

static void setLastError(const std::string& message)
{
    g_lastError = message;
}

const char* GetAstraLastError(void)
{
    return g_lastError.c_str();
}

struct AstraContext {
#if defined(USE_ASTRA_SDK)
    astra::StreamSet* streamSet;
    astra::StreamReader* reader;
#elif defined(USE_OPENNI2)
    openni::Device device;
    openni::VideoStream depthStream;
    openni::VideoStream colorStream;
    std::vector<int16_t> depthBuffer;
    std::vector<uint8_t> colorBuffer;
#endif
};

AstraContext_t* InitializeAstraObj()
{
    setLastError("");
#if defined(USE_ASTRA_SDK)
    // Initial Astra SDK
    astra::initialize();
    AstraContext_t* context = new AstraContext_t;
    context->streamSet = new astra::StreamSet();
    context->reader = new astra::StreamReader(context->streamSet->create_reader());
    context->reader->stream<astra::DepthStream>().start();
    context->reader->stream<astra::ColorStream>().start();
    return context;
#else
    const auto initStatus = openni::OpenNI::initialize();
    if (initStatus != openni::STATUS_OK)
    {
        setLastError(std::string("OpenNI2 initialize failed: ") +
                     openni::OpenNI::getExtendedError());
        std::cerr << g_lastError << std::endl;
        return NULL;
    }

    AstraContext_t* context = new AstraContext_t;

    if (context->device.open(openni::ANY_DEVICE) != openni::STATUS_OK)
    {
        setLastError(std::string("OpenNI2 device open failed: ") +
                     openni::OpenNI::getExtendedError());
        std::cerr << g_lastError << std::endl;
        delete context;
        openni::OpenNI::shutdown();
        return NULL;
    }

    if (context->depthStream.create(context->device, openni::SENSOR_DEPTH) != openni::STATUS_OK)
    {
        setLastError(std::string("OpenNI2 depth stream create failed: ") +
                     openni::OpenNI::getExtendedError());
        std::cerr << g_lastError << std::endl;
        context->device.close();
        delete context;
        openni::OpenNI::shutdown();
        return NULL;
    }

    if (context->colorStream.create(context->device, openni::SENSOR_COLOR) != openni::STATUS_OK)
    {
        setLastError(std::string("OpenNI2 color stream create failed: ") +
                     openni::OpenNI::getExtendedError());
        std::cerr << g_lastError << std::endl;
        context->depthStream.destroy();
        context->device.close();
        delete context;
        openni::OpenNI::shutdown();
        return NULL;
    }

    openni::VideoMode depthMode = context->depthStream.getVideoMode();
    depthMode.setResolution(640, 480);
    depthMode.setFps(30);
    depthMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
    context->depthStream.setVideoMode(depthMode);

    openni::VideoMode colorMode = context->colorStream.getVideoMode();
    colorMode.setResolution(640, 480);
    colorMode.setFps(30);
    colorMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
    context->colorStream.setVideoMode(colorMode);

    if (context->depthStream.start() != openni::STATUS_OK ||
        context->colorStream.start() != openni::STATUS_OK)
    {
        setLastError(std::string("OpenNI2 stream start failed: ") +
                     openni::OpenNI::getExtendedError());
        std::cerr << g_lastError << std::endl;
        context->colorStream.destroy();
        context->depthStream.destroy();
        context->device.close();
        delete context;
        openni::OpenNI::shutdown();
        return NULL;
    }

    const auto& deviceInfo = context->device.getDeviceInfo();
    setLastError("");
    std::cout << "OpenNI2 device connected: "
              << deviceInfo.getVendor() << ' ' << deviceInfo.getName()
              << " (" << deviceInfo.getUri() << ")" << std::endl;

    return context;
#endif
}

void TerminateAstraObj(AstraContext_t* context)
{
#if defined(USE_ASTRA_SDK)
    delete context->reader;
    delete context->streamSet;
    astra::terminate();
#else
    if (context)
    {
        context->colorStream.stop();
        context->depthStream.stop();
        context->colorStream.destroy();
        context->depthStream.destroy();
        context->device.close();
    }
    openni::OpenNI::shutdown();
#endif
    delete context;
}

// For OpenGL - Depth
const int16_t* GetDepthDataAstraOpenGL(AstraContext_t* context, int* width, int* height)
{
#if defined(USE_ASTRA_SDK)
    astra_update();
    astra::Frame frame = context->reader->get_latest_frame(ASTRA_TIMEOUT_RETURN_IMMEDIATELY);
    auto depthFrame = frame.get<astra::DepthFrame>();

    if (depthFrame.is_valid())
    {
        *width = depthFrame.width();
        *height = depthFrame.height();
        return depthFrame.data();
    }else{
        setLastError("Get Depth Frame Failed");
        std::cout << "Get Depth Frame Failed...!" << std::endl;
    }
    return NULL;
#else
    openni::VideoFrameRef frame;
    if (context->depthStream.readFrame(&frame) == openni::STATUS_OK && frame.isValid())
    {
        *width = frame.getWidth();
        *height = frame.getHeight();
        const auto* depthData = static_cast<const int16_t*>(frame.getData());
        context->depthBuffer.assign(depthData, depthData + (*width * *height));
        return context->depthBuffer.data();
    }
    setLastError("OpenNI2 depth frame read failed");
    std::cout << "Get Depth Frame Failed...!" << std::endl;
    return NULL;
#endif
}

// For OpenGL - Color
const uint8_t* GetColorDataAstraOpenGL(AstraContext_t* context, int* width, int* height)
{
#if defined(USE_ASTRA_SDK)
    astra::Frame frame = context->reader->get_latest_frame(ASTRA_TIMEOUT_RETURN_IMMEDIATELY);
    auto colorFrame = frame.get<astra::ColorFrame>();

    if (colorFrame.is_valid())
    {
        *width = colorFrame.width();
        *height = colorFrame.height();
        const astra::RgbPixel* rgbData = colorFrame.data();

        // RgbPixel 포인터를 uint8_t 포인터로 변환
        return reinterpret_cast<const uint8_t*>(rgbData);
    }else{
        setLastError("Get Color Frame Failed");
        std::cout << "Get Color Frame Failed...!" << std::endl;
    } 
    return NULL;
#else
    openni::VideoFrameRef frame;
    if (context->colorStream.readFrame(&frame) == openni::STATUS_OK && frame.isValid())
    {
        *width = frame.getWidth();
        *height = frame.getHeight();
        const auto* rgbData = static_cast<const uint8_t*>(frame.getData());
        context->colorBuffer.assign(rgbData, rgbData + (*width * *height * 3));
        return context->colorBuffer.data();
    }
    setLastError("OpenNI2 color frame read failed");
    std::cout << "Get Color Frame Failed...!" << std::endl;
    return NULL;
#endif
}

// // For WebGL - Depth data to JSON format
// const char* GetDepthDataAstraWebGL(AstraContext_t* context)
// {
//     int width, height;
//     const int16_t* depthData = GetDepthDataAstraOpenGL(context, &width, &height);
//     
//     if (depthData)
//     {
//         nlohmann::json jsonData;
//         for (int y = 0; y < height; ++y)
//         {
//             for (int x = 0; x < width; ++x)
//             {
//                 int index = y * width + x;
//                 int depthValue = depthData[index];

//                 if (depthValue > 0)
//                 {
//                     float z = depthValue / 1000.0f;
//                     float x_pos = (x - width / 2) * z / 570.3f;
//                     float y_pos = (y - height / 2) * z / 570.3f;

//                     jsonData.push_back({{"x", x_pos}, {"y", -y_pos}, {"z", -z}});
//                 }
//             }
//         }
//         
//         // 생성된 JSON 데이터를 콘솔에 출력
//         std::string jsonString = jsonData.dump();
//         std::cout << "Generated JSON Data: " << jsonString << std::endl;
//         char* jsonDataCStr = new char[jsonString.length() + 1];
//         std::strcpy(jsonDataCStr, jsonString.c_str());
//         return jsonDataCStr;  // 웹 서버에서 JSON 문자열로 반환
//     }
//     return nullptr;
// }

// // For WebGL - Color data to JSON format
// const char* GetColorDataAstraWebGL(AstraContext_t* context)
// {
//     int width, height;
//     const uint8_t* colorData = GetColorDataAstraOpenGL(context, &width, &height);
//     
//     if (colorData)
//     {
//         nlohmann::json jsonData;
//         for (int y = 0; y < height; ++y)
//         {
//             for (int x = 0; x < width; ++x)
//             {
//                 int index = (y * width + x) * 3;
//                 float r = colorData[index] / 255.0f;
//                 float g = colorData[index + 1] / 255.0f;
//                 float b = colorData[index + 2] / 255.0f;

//                 jsonData.push_back({{"r", r}, {"g", g}, {"b", b}});
//             }
//         }
//         // 생성된 JSON 데이터를 콘솔에 출력
//         std::string jsonString = jsonData.dump();
//         std::cout << "Generated JSON Data: " << jsonString << std::endl;
//         char* jsonDataCStr = new char[jsonString.length() + 1];
//         std::strcpy(jsonDataCStr, jsonString.c_str());
//         return jsonDataCStr;  // 웹 서버에서 JSON 문자열로 반환
//     }
//     return nullptr;
// }
} // extern "C"
