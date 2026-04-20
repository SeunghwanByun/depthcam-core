#include "viewerModule.h"
#include "../frameDefinitions.h"
#ifdef YOUTH_ZMQ_ENABLED
#include "../ZmqPublisher/zmq_publisher.h"
#endif
#include <pthread.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <string.h>
#include <mqueue.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

// Monitor Resolution Information
#ifdef _WIN32
#include <windows.h>
#elif __linux__
#include <X11/Xlib.h>
#endif

// Viewer Status Variables
GLuint depthTexture;
GLuint colorTexture;

float angleX = 0.0f;
float angleY = 0.0f;
float distance = 2.0f;
int lastMouseX = 0, lastMouseY = 0;
float zoom = 1.0f;
int isDragging = 0;

// Message Queue Handle
mqd_t mqReceive = -1;

// GLFW Window
GLFWwindow* window;

// 종료 콜백 함수 포인터
static ExitCallbackFunc exitCallback = NULL;
static pthread_mutex_t viewerStatusMutex = PTHREAD_MUTEX_INITIALIZER;
static char viewerStatusText[256] = "Waiting for sensor";

// Data Buffer
int16_t* depthDataBuffer = NULL;
uint8_t* colorDataBuffer = NULL;
int currentWidth = 0;
int currentHeight = 0;
int hasNewData = 0;
// int currentFrameId = 0;

// Thread Control Variable
int viewerIsRunning = 1;
pthread_mutex_t dataMutex = PTHREAD_MUTEX_INITIALIZER;

// 청크 수신 상태 구조체
typedef struct{
  int frameId;
  int width;
  int height;
  uint32_t timestamp;
  int receivedDepthChunks;
  int totalDepthChunks;
  int receivedColorChunks;
  int totalColorChunks;
  char* depthDataBuffer;
  char* colorDataBuffer;
  int isComplete;
} FrameReceiveStatus;

FrameReceiveStatus receiveStatus = {0}; // 정적 초기화 추가

// Initialize OpenGL
void initOpenGL()
{
    glGenTextures(1, &depthTexture);
    glBindTexture(GL_TEXTURE_2D, depthTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);
}

// 콜백 설정 함수
void setExitCallback(ExitCallbackFunc callback){
  exitCallback = callback;
}

void setViewerStatusText(const char* text){
  pthread_mutex_lock(&viewerStatusMutex);
  snprintf(viewerStatusText, sizeof(viewerStatusText), "%s", text ? text : "");
  pthread_mutex_unlock(&viewerStatusMutex);
}

// 안전한 메모리 할당 헬퍼 함수
static void* safe_malloc(size_t size){
  void* ptr = malloc(size);
  if(!ptr){
    perror("malloc failed");
    return NULL;
  }
  memset(ptr, 0, size); // 안전하게 0으로 초기화
  return ptr;
}

// Frame Receive Status Initialization
void initFrameReceiveStatus(FrameReceiveStatus* status, int frameId, uint32_t timestamp, int width, int height){
  // 기존 버퍼를 해제하기 전에 새 버퍼 할당
  int depthDataSize = width * height * sizeof(int16_t);
  int colorDataSize = width * height * 3 * sizeof(uint8_t);

  char* newDepthBuffer = safe_malloc(depthDataSize);
  char* newColorBuffer = safe_malloc(colorDataSize);

  if(!newDepthBuffer || !newColorBuffer){
    if(newDepthBuffer) free(newDepthBuffer);
    if(newColorBuffer) free(newColorBuffer);

    // 실패 시 기준 버퍼 유지
    return;
  }

  // 이전 버퍼 해제
  if(status->depthDataBuffer){
    free(status->depthDataBuffer);
  }
  if(status->colorDataBuffer){
    free(status->colorDataBuffer);
  }

  status->frameId = frameId;
  status->timestamp = timestamp;
  status->width = width;
  status->height = height;
  status->receivedDepthChunks = 0;
  status->totalDepthChunks = 0;
  status->receivedColorChunks = 0;
  status->totalColorChunks = 0;
  status->isComplete = 0;
  status->depthDataBuffer = newDepthBuffer;
  status->colorDataBuffer = newColorBuffer;

  // // 이전 버퍼 해제
  // if(status->depthDataBuffer){
  //   free(status->depthDataBuffer);
  // }
  //
  // if(status->colorDataBuffer){
  //   free(status->colorDataBuffer);
  // }
  //
  // // Allocate new buffer
  // int depthDataSize = width * height * sizeof(int16_t);
  // int colorDataSize = width * height * 3 * sizeof(uint8_t);
  //
  // status->depthDataBuffer = (char*)malloc(depthDataSize);
  // status->colorDataBuffer = (char*)malloc(colorDataSize);
  //
  // if(!status->depthDataBuffer || !status->colorDataBuffer){
  //   perror("malloc frame buffers");
  //   if(status->depthDataBuffer) free(status->depthDataBuffer);
  //   if(status->colorDataBuffer) free(status->colorDataBuffer);
  //   status->depthDataBuffer = NULL;
  //   status->colorDataBuffer = NULL;
  // }
}

// Data Receiving Thread
void* dataReceiveThread(void* arg){
  (void)arg;
  printf("Viewer data receive thread started...\n");

  // Message Queue Feature Setting
  struct mq_attr attr;
  attr.mq_flags = 0;
  attr.mq_maxmsg = 10;
  attr.mq_msgsize = MAX_MSG_SIZE;
  attr.mq_curmsgs = 0;

  // Open Message Queue
  mqReceive = mq_open(MQ_LOGGER_TO_VIEWER, O_RDONLY, 0644, &attr);
  if(mqReceive == (mqd_t) - 1){
    perror("mq_open receive");
    return NULL;
  }

  // Message Buffer
  char* msgBuffer = safe_malloc(MAX_MSG_SIZE);
  if(!msgBuffer){
    mq_close(mqReceive);
    return NULL;
  }

  // Initialize Receive status - 구조체 필드 명시적 초기화
  memset(&receiveStatus, 0, sizeof(FrameReceiveStatus));
  receiveStatus.depthDataBuffer = NULL;
  receiveStatus.colorDataBuffer = NULL;

  // Main Loop
  while(viewerIsRunning){
    ssize_t byteRead = mq_receive(mqReceive, msgBuffer, MAX_MSG_SIZE, NULL);

    if(byteRead > 0){
      MessageHeader* header = (MessageHeader*)msgBuffer;

      // 메세지 타입에 따라 처리
      switch(header->msgType){
        case MSG_TYPE_METADATA:
          // 새 프레임 정보 수신
          pthread_mutex_lock(&dataMutex);
          initFrameReceiveStatus(&receiveStatus, header->frameId, header->timestamp, header->width, header->height);
          pthread_mutex_unlock(&dataMutex);
          break;

        case MSG_TYPE_DEPTH_DATA:
          // 깊이 데이터 청크 수신
          pthread_mutex_lock(&dataMutex);

          if(header->frameId == receiveStatus.frameId && receiveStatus.depthDataBuffer){
            int maxDataPerMsg = MAX_MSG_SIZE - sizeof(MessageHeader);
            int offset = header->chunkIndex * maxDataPerMsg;
            size_t depthBufferBytes = (size_t)receiveStatus.width * (size_t)receiveStatus.height * sizeof(int16_t);

            // 범위 체크 추가
            if((size_t)(offset + header->dataSize) <= depthBufferBytes){
              memcpy(receiveStatus.depthDataBuffer + offset, msgBuffer + sizeof(MessageHeader), header->dataSize);

              receiveStatus.receivedDepthChunks++;
              receiveStatus.totalDepthChunks = header->totalChunks;
            }else{
              printf("Warning: Depth data chunk exceeds buffer size, ignoring\n");
            }
          }

          pthread_mutex_unlock(&dataMutex);
          break;

        case MSG_TYPE_COLOR_DATA:
          // 색상 데이터 청크 수신
          pthread_mutex_lock(&dataMutex);

          if(header->frameId == receiveStatus.frameId && receiveStatus.colorDataBuffer){
            int maxDataPerMsg = MAX_MSG_SIZE - sizeof(MessageHeader);
            int offset = header->chunkIndex * maxDataPerMsg;
            size_t colorBufferBytes = (size_t)receiveStatus.width * (size_t)receiveStatus.height * 3U * sizeof(uint8_t);

            // 범위 체크 추가
            if((size_t)(offset + header->dataSize) <= colorBufferBytes){
              memcpy(receiveStatus.colorDataBuffer + offset, msgBuffer + sizeof(MessageHeader), header->dataSize);

              receiveStatus.receivedColorChunks++;
              receiveStatus.totalColorChunks = header->totalChunks;
            }else{
              printf("Warning: Color data chunk exceeds buffer size, lgnoring\n");
            }
          }

          pthread_mutex_unlock(&dataMutex);
          break;
      }

      // 프레임 완성 여부 확인
      pthread_mutex_lock(&dataMutex);

      if(receiveStatus.totalDepthChunks > 0 && receiveStatus.totalColorChunks > 0 && receiveStatus.receivedDepthChunks == receiveStatus.totalDepthChunks && receiveStatus.receivedColorChunks == receiveStatus.totalColorChunks){
        // 프레임이 완성됨 - 메인 버퍼로 복사

        if(currentWidth != receiveStatus.width || currentHeight != receiveStatus.height){
          // 버퍼 크기 변경 필요
          int16_t* newDepthBuffer = (int16_t*)safe_malloc(receiveStatus.width * receiveStatus.height * sizeof(int16_t));
          uint8_t* newColorBuffer = (uint8_t*)safe_malloc(receiveStatus.width * receiveStatus.height * 3 * sizeof(uint8_t));

          if(!newDepthBuffer || !newColorBuffer){
            if(newDepthBuffer) free(newDepthBuffer);
            if(newColorBuffer) free(newColorBuffer);
            pthread_mutex_unlock(&dataMutex);
            continue;
          }

          // 이전 버퍼 해제
          if(depthDataBuffer) free(depthDataBuffer);
          if(colorDataBuffer) free(colorDataBuffer);

          depthDataBuffer = newDepthBuffer;
          colorDataBuffer = newColorBuffer;
          currentWidth = receiveStatus.width;
          currentHeight = receiveStatus.height;
        }

        if(depthDataBuffer && colorDataBuffer){
          // 데이터 복사
          memcpy(depthDataBuffer, receiveStatus.depthDataBuffer, currentWidth * currentHeight * sizeof(int16_t));
          memcpy(colorDataBuffer, receiveStatus.colorDataBuffer, currentWidth * currentHeight * 3 * sizeof(uint8_t));

          hasNewData = 1;

#ifdef YOUTH_ZMQ_ENABLED
          {
            uint64_t ts_ns = (uint64_t)receiveStatus.timestamp * 1000000ULL;
            zmq_pub_depth_frame(ts_ns,
                                (uint16_t)currentWidth, (uint16_t)currentHeight,
                                depthDataBuffer, colorDataBuffer);
          }
#endif
        }

        // 상태 초기화를 위한 플래그
        receiveStatus.isComplete = 1;
      }

      pthread_mutex_unlock(&dataMutex);
    }else{
      // 에러 발생 시 짧은 대기 후 재시도
      usleep(10000); // 10ms 
      continue;
    }
  }

  // 정리
  free(msgBuffer);

  pthread_mutex_lock(&dataMutex);
  if(receiveStatus.depthDataBuffer){
    free(receiveStatus.depthDataBuffer);
    receiveStatus.depthDataBuffer = NULL;
  }

  if(receiveStatus.colorDataBuffer){
    free(receiveStatus.colorDataBuffer);
    receiveStatus.colorDataBuffer = NULL;
  }
  pthread_mutex_unlock(&dataMutex);

  mq_close(mqReceive);

  printf("Data receive thread terminated\n");
  return NULL;
}

// 3D Rendring Function
void display_3d_color(){
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glLoadIdentity();
  glTranslatef(0.0f, 0.0f, -distance);
  glRotatef(angleX, 1.0f, 0.0f, 0.0f);
  glRotatef(angleY, 0.0f, 1.0f, 0.0f);

  glScalef(zoom, zoom, zoom);

  pthread_mutex_lock(&dataMutex);
  
  if(hasNewData && depthDataBuffer && colorDataBuffer && currentWidth > 0 && currentHeight > 0){
    glBegin(GL_POINTS);

    for(int y = 0; y < currentHeight; ++y){
      for(int x = 0; x < currentWidth; ++x){
        int index = y * currentWidth + x;
        int depthValue = depthDataBuffer[index];

        if(depthValue > 0){
          // Calculate 3D Coordinate (Using Simple Camera Model)
          float z_pos = depthValue / 1000.0f; // from mm to m
          float x_pos = (x - currentWidth / 2) * z_pos  / 570.3f; // 570.3f is focal distance of Astra camera
          float y_pos = (y - currentHeight / 2) * z_pos / 570.3f;

          // Set color using color data (RGB order)
          int colorIndex = index * 3; // RGB consist of 3 values.
          float r = colorDataBuffer[colorIndex] / 255.0f;
          float g = colorDataBuffer[colorIndex + 1] / 255.0f;
          float b = colorDataBuffer[colorIndex + 2] / 255.0f;

          glColor3f(r, g, b);
          glVertex3f(-x_pos, -y_pos, -z_pos);
        }
      }
    }

    glEnd();
  }

  pthread_mutex_unlock(&dataMutex);

  if(window){
    glfwSwapBuffers(window);
  }
}

// GLFW Error Callback
void error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

// 윈도우 종료 콜백
void window_close_callback(GLFWwindow* window){
  (void)window;
  printf("Window close callback triggered\n");
  viewerIsRunning = 0;

  // 외부 프로그램에 종료 신호 보내기
  if(exitCallback){
    exitCallback();
  }
}

// Keyboard Callback
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
  (void)scancode;
  (void)mods;
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
    printf("ESC key pressed, setting window should close\n");
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  }

  /* 외부 프로그램에 종료 신호 보내기는 window_close_callback 에서 처리됨
   * 여기에서는 중복 호출하지 않음. */
  // // 외부 프로그램에 종료 신호 보내기 (전역변수나 신호 사용)
  // if(exitCallback){
  //   exitCallback(); // 메인 프로그램에 등록된 콜백 호출
  // }
}

// Mouse Button Callback
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
  (void)mods;
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    if (action == GLFW_PRESS) {
      isDragging = 1;
      glfwGetCursorPos(window, (double*)&lastMouseX, (double*)&lastMouseY);
    } else if (action == GLFW_RELEASE) {
      isDragging = 0;
    }
  }
}

// Mouse Moving Callback
void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
    (void)window;
    if (isDragging) {
        int dx = xpos - lastMouseX;
        int dy = ypos - lastMouseY;

        angleX += dy * 0.05f;
        angleY += dx * 0.05f;

        lastMouseX = xpos;
        lastMouseY = ypos;
    }
}

// Scroll Callback
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    (void)window;
    (void)xoffset;
    if (yoffset > 0) {
        // Zoom in
        zoom *= 1.1f;
    } else {
        // Zoom out
        zoom *= 0.9f;
    }
}

// Window Size Change Callback
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
  (void)window;
  if(width <= 0 || height <= 0) return;

  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0, (double)width / (double)height, 0.1, 100.0);
  glMatrixMode(GL_MODELVIEW);
}

// Viewer Thread Function
void* viewerThread(void* id){
  (void)id;
  printf("Initializing GLFW...\n");

  if (!glfwInit()) {
    fprintf(stderr, "Failed to initialize GLFW\n");
    viewerIsRunning = 0; // 초기화 실패 시 바로 실행 중지 플래그 설정
    return NULL;
  }
    
  glfwSetErrorCallback(error_callback);
    
  // Get Monitor Resolution
  int screenWidth = 800; // Set default value.
  int screenHeight = 600;

#ifdef _WIN32
  screenWidth = GetSystemMetrics(SM_CXSCREEN);
  screenHeight = GetSystemMetrics(SM_CYSCREEN);
#elif __linux__
  Display* d = XOpenDisplay(NULL);
  if(d){
    Screen* s = DefaultScreenOfDisplay(d);
    screenWidth = s->width;
    screenHeight = s->height;
    XCloseDisplay(d);
  }
#endif

  // // 너무 큰 창 크기 방지
  // screenWidth = screenWidth > 1920 ? 1920 : screenWidth;
  // screenHeight = screenHeight > 1080 ? 1080 : screenHeight;
  //
  // // 창 크기를 화면 크기의 80%로 설정
  // screenWidth = (int)(screenWidth * 0.8);
  // screenHeight = (int)(screenHeight * 0.8);

  // //  GLFW window creation
  // glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  // glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
  // glfwWindowHint(GLFW_SAMPLES, 4);

  window = glfwCreateWindow(screenWidth, screenHeight, "3D Viewer", NULL, NULL);
  if (!window) {
      fprintf(stderr, "Failed to create GLFW window\n");
      glfwTerminate();
      viewerIsRunning = 0;
      return NULL;
  }
    
  glfwMakeContextCurrent(window);
  glfwSetWindowCloseCallback(window, window_close_callback);
  glfwSetKeyCallback(window, key_callback);
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetCursorPosCallback(window, cursor_position_callback);
  glfwSetScrollCallback(window, scroll_callback);
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    
  // Initialize GLEW
  GLenum err = glewInit();
  if (err != GLEW_OK) {
      fprintf(stderr, "GLEW initialization error: %s\n", glewGetErrorString(err));
      glfwDestroyWindow(window);
      glfwTerminate();
      viewerIsRunning = 0;
      return NULL;
  }
    
  initOpenGL();

  // OpenGL 상태 설정
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_POINT_SMOOTH);
  glPointSize(2.0f);

  // Set up initial perspective
  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  framebuffer_size_callback(window, width, height);

  // Start Data Receiving Thread
  pthread_t receiveThreadId;
  if(pthread_create(&receiveThreadId, NULL, dataReceiveThread, NULL) != 0){
    fprintf(stderr, "Failed to create data receive thread\n");
    glfwDestroyWindow(window);
    glfwTerminate();
    viewerIsRunning = 0;
    return NULL;
  }

  // Main loop
  while (viewerIsRunning && window && !glfwWindowShouldClose(window)) {
    char windowTitle[320];
    pthread_mutex_lock(&viewerStatusMutex);
    snprintf(windowTitle, sizeof(windowTitle), "3D Viewer - %s", viewerStatusText);
    pthread_mutex_unlock(&viewerStatusMutex);
    glfwSetWindowTitle(window, windowTitle);

    display_3d_color();
    glfwPollEvents();

    // 약간의 CPU 시간 양보
    usleep(10000); // 10ms
  }

  printf("Exited viewer main loop, viewerIsRunning=%d, window=%p\n", viewerIsRunning, window);

  // 정리
  viewerIsRunning = 0;
  pthread_join(receiveThreadId, NULL);
  
  pthread_mutex_lock(&dataMutex);
  if(depthDataBuffer){
    free(depthDataBuffer);
    depthDataBuffer = NULL;
  }
  if(colorDataBuffer){
    free(colorDataBuffer);
    colorDataBuffer = NULL;
  }

  if(window){
    glfwDestroyWindow(window);
    window = NULL;
  }
  
  glfwTerminate();

  printf("Viewer thread terminated\n");
  return NULL;
}

pthread_t viewer_thread_id;
void initViewerModule(){
  printf("Initializing viewer module...\n");
  viewerIsRunning = 1;
  setViewerStatusText("Waiting for sensor data");
#ifdef YOUTH_ZMQ_ENABLED
  zmq_pub_init("tcp://*:5558");
#endif
  pthread_create(&viewer_thread_id, NULL, viewerThread, NULL);
}

void stopViewerModule(){
  printf("Stopping viewer module...\n");
  viewerIsRunning = 0;

  // 윈도우가 열려있다면 닫기 요청
  if(window){
    glfwSetWindowShouldClose(window, GLFW_TRUE);

    // 약간의 시간을 주어 창이 닫히도록 함
    usleep(100000); // 100ms
  }

  pthread_join(viewer_thread_id, NULL);
#ifdef YOUTH_ZMQ_ENABLED
  zmq_pub_destroy();
#endif
  printf("Viewer module stopped\n");
}

int isViewerModuleRunning(void){
  return viewerIsRunning;
}

void requestStopViewerModule(){
  viewerIsRunning = 0;
}
