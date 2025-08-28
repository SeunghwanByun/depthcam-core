#include "Viewer.hpp"
#include "Sensor/DepthSource.hpp"
#include "GL/UploadTex.hpp"
#include <iostream>

int main()
{
    try
    {
        Youth::Viewer::Config config;
        config.width = 1280;
        config.height = 720;
        config.title = "Youth Viewer";
        config.vsync = true;

        Youth::Viewer viewer(config);
        if (!viewer.ok())
        {
            std::cerr << "Failed to create viewer.\n";
            return 1;
        }

        Youth::Sensor::DepthSource sensor;
        sensor.start();

        static Youth::GL::UploadTex uploadTex;

        viewer.setDrawCallback([&]()
                               {
            Youth::Sensor::DepthSource::Frame frame;
            if(sensor.latestFrame(frame)){
                uploadTex.ensure(frame.width, frame.height);
                uploadTex.update(frame.intensity.data());
                viewer.drawTexture2D(uploadTex.id(), frame.width, frame.height, false);
            }else{
                // 아직 프레임 없으면 기본 삼각형
                // viewer 내부 기본 drawTriangle() 이 호출되게 하려면 여기서 아무것도 안하면 됨.
            } });

        viewer.run();
        sensor.stop();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Fatal: " << e.what() << '\n';
        return 2;
    }
    return 0;
}