#ifndef VIEWER_H
#define VIEWER_H

#include <GL/glew.h>
#include <GL/glut.h>

// viewerModule.h 에 콜백 타입 및 설정 함수 추가
typedef void (*ExitCallbackFunc)(void);
void setExitCallback(ExitCallbackFunc callback);
void setViewerStatusText(const char* text);

// Set before initialization; P replays this file without needing a new recording.
void setViewerPlaybackFile(const char* path);
void initViewerModule();
void stopViewerModule();
void* viewerModule(void* id);

int isViewerModuleRunning(void);
void requestStopViewerModule();

// Publish an owned snapshot from an algorithm worker. No OpenGL calls occur here.
// xyzrgb: 6 floats/point, xyz: 3 floats/path vertex. Units: metres,
// OpenGL coordinates (+X right, +Y up, camera looks toward -Z), RGB in [0,1].
// Maximum 300000 map points / 10000 path vertices; oversized input is rejected.
// Pass empty arrays to clear. status is a short ASCII label, fps is processing FPS.
#ifdef __cplusplus
extern "C" {
#endif
int viewerPublishSlamResult(const float* xyzrgb, int pointCount,
                           const float* xyz, int pathCount,
                           const char* status, float fps);
#ifdef __cplusplus
}
#endif

#endif // VIEWER_H
