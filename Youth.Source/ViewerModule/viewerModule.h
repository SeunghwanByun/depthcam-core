#ifndef VIEWER_H
#define VIEWER_H

#include <GL/glew.h>
#include <GL/glut.h>

// viewerModule.h 에 콜백 타입 및 설정 함수 추가
typedef void (*ExitCallbackFunc)(void);
void setExitCallback(ExitCallbackFunc callback);
void setViewerStatusText(const char* text);

void initViewerModule();
void stopViewerModule();
void* viewerModule(void* id);

int isViewerModuleRunning(void);
void requestStopViewerModule();

#endif // VIEWER_H
