#include "viewerModule.h"
#include "../frameDefinitions.h"
#include "../AlgorithmModule/voModule.h"
#ifdef YOUTH_ZMQ_ENABLED
#include "../ZmqPublisher/zmq_publisher.h"
#endif
#include <pthread.h>
#include <GL/glew.h>
#include <GL/glut.h>
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <string.h>
#include <mqueue.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

// Monitor Resolution Information
#ifdef _WIN32
#include <windows.h>
#elif __linux__
#include <X11/Xlib.h>
#endif

// Viewer Status Variables
GLuint depthTexture;
GLuint colorTexture;

// Camera state
float angleX = 0.0f;
float angleY = 0.0f;
float distance = 2.0f;
float zoom = 1.0f;
float panX = 0.0f;
float panY = 0.0f;

// Mouse drag state (left = pan, right = rotate)
int isDragging = 0;
int lastMouseX = 0, lastMouseY = 0;
int isDraggingRight = 0;
int lastRightX = 0, lastRightY = 0;

// Window dimensions (updated in framebuffer_size_callback)
static int windowWidth = 800;
static int windowHeight = 600;

// Message Queue Handle
mqd_t mqReceive = -1;

// GLFW Window
GLFWwindow* window;

// Exit callback
static ExitCallbackFunc exitCallback = NULL;
static pthread_mutex_t viewerStatusMutex = PTHREAD_MUTEX_INITIALIZER;
static char viewerStatusText[256] = "Waiting for sensor";

// Recording / Playback state (tracked locally in viewer)
static int viewerIsRecording = 0;
static int viewerIsPlayback = 0;
static char lastRecordedFilename[256] = "";

// VO state tracked in viewer
static int viewerVoActive = 0;  // 0 = off, 1 = on

// VO map display buffer — updated incrementally (only when map grows)
#define VO_MAX_DISPLAY_POINTS 300000
static float  s_voDisplayBuf[VO_MAX_DISPLAY_POINTS * 6];
static int    s_voDisplayCount  = 0;
static int    s_voLastSyncCount = 0;  // how many points were copied last time

// Data Buffer
int16_t* depthDataBuffer = NULL;
uint8_t* colorDataBuffer = NULL;
int currentWidth = 0;
int currentHeight = 0;
int hasNewData = 0;

// Thread Control Variable
int viewerIsRunning = 1;
pthread_mutex_t dataMutex = PTHREAD_MUTEX_INITIALIZER;

// Chunk receive state
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

FrameReceiveStatus receiveStatus = {0};

// Send a control command to LoggingModule via the control message queue
static void viewerSendControlCommand(int command, const char* filename){
  struct mq_attr attr;
  attr.mq_flags = 0;
  attr.mq_maxmsg = 10;
  attr.mq_msgsize = MAX_MSG_SIZE;
  attr.mq_curmsgs = 0;

  mqd_t mqCtrl = mq_open(MQ_CONTROL_QUEUE, O_WRONLY, 0644, &attr);
  if(mqCtrl == (mqd_t)-1){
    perror("viewer: mq_open control");
    return;
  }

  MessageHeader msg;
  memset(&msg, 0, sizeof(MessageHeader));
  msg.msgType = MSG_TYPE_CONTROL;
  msg.ctrlCommand = command;

  if(filename){
    strncpy(msg.filename, filename, sizeof(msg.filename) - 1);
    msg.filename[sizeof(msg.filename) - 1] = '\0';
  }

  if(mq_send(mqCtrl, (char*)&msg, sizeof(MessageHeader), 0) == -1){
    perror("viewer: mq_send control");
  }

  mq_close(mqCtrl);
}

// Initialize OpenGL
void initOpenGL()
{
    glGenTextures(1, &depthTexture);
    glBindTexture(GL_TEXTURE_2D, depthTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void setExitCallback(ExitCallbackFunc callback){
  exitCallback = callback;
}

void setViewerStatusText(const char* text){
  pthread_mutex_lock(&viewerStatusMutex);
  snprintf(viewerStatusText, sizeof(viewerStatusText), "%s", text ? text : "");
  pthread_mutex_unlock(&viewerStatusMutex);
}

static void* safe_malloc(size_t size){
  void* ptr = malloc(size);
  if(!ptr){
    perror("malloc failed");
    return NULL;
  }
  memset(ptr, 0, size);
  return ptr;
}

void initFrameReceiveStatus(FrameReceiveStatus* status, int frameId, uint32_t timestamp, int width, int height){
  int depthDataSize = width * height * sizeof(int16_t);
  int colorDataSize = width * height * 3 * sizeof(uint8_t);

  char* newDepthBuffer = safe_malloc(depthDataSize);
  char* newColorBuffer = safe_malloc(colorDataSize);

  if(!newDepthBuffer || !newColorBuffer){
    if(newDepthBuffer) free(newDepthBuffer);
    if(newColorBuffer) free(newColorBuffer);
    return;
  }

  if(status->depthDataBuffer) free(status->depthDataBuffer);
  if(status->colorDataBuffer) free(status->colorDataBuffer);

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
}

// Data Receiving Thread
void* dataReceiveThread(void* arg){
  (void)arg;
  printf("Viewer data receive thread started...\n");

  struct mq_attr attr;
  attr.mq_flags = 0;
  attr.mq_maxmsg = 10;
  attr.mq_msgsize = MAX_MSG_SIZE;
  attr.mq_curmsgs = 0;

  mqReceive = mq_open(MQ_LOGGER_TO_VIEWER, O_RDONLY, 0644, &attr);
  if(mqReceive == (mqd_t) - 1){
    perror("mq_open receive");
    return NULL;
  }

  char* msgBuffer = safe_malloc(MAX_MSG_SIZE);
  if(!msgBuffer){
    mq_close(mqReceive);
    return NULL;
  }

  memset(&receiveStatus, 0, sizeof(FrameReceiveStatus));
  receiveStatus.depthDataBuffer = NULL;
  receiveStatus.colorDataBuffer = NULL;

  while(viewerIsRunning){
    ssize_t byteRead = mq_receive(mqReceive, msgBuffer, MAX_MSG_SIZE, NULL);

    if(byteRead > 0){
      MessageHeader* header = (MessageHeader*)msgBuffer;

      switch(header->msgType){
        case MSG_TYPE_METADATA:
          pthread_mutex_lock(&dataMutex);
          initFrameReceiveStatus(&receiveStatus, header->frameId, header->timestamp, header->width, header->height);
          pthread_mutex_unlock(&dataMutex);
          break;

        case MSG_TYPE_DEPTH_DATA:
          pthread_mutex_lock(&dataMutex);

          if(header->frameId == receiveStatus.frameId && receiveStatus.depthDataBuffer){
            int maxDataPerMsg = MAX_MSG_SIZE - sizeof(MessageHeader);
            int offset = header->chunkIndex * maxDataPerMsg;
            size_t depthBufferBytes = (size_t)receiveStatus.width * (size_t)receiveStatus.height * sizeof(int16_t);

            if((size_t)(offset + header->dataSize) <= depthBufferBytes){
              memcpy(receiveStatus.depthDataBuffer + offset, msgBuffer + sizeof(MessageHeader), header->dataSize);
              receiveStatus.receivedDepthChunks++;
              receiveStatus.totalDepthChunks = header->totalChunks;
            }
          }

          pthread_mutex_unlock(&dataMutex);
          break;

        case MSG_TYPE_COLOR_DATA:
          pthread_mutex_lock(&dataMutex);

          if(header->frameId == receiveStatus.frameId && receiveStatus.colorDataBuffer){
            int maxDataPerMsg = MAX_MSG_SIZE - sizeof(MessageHeader);
            int offset = header->chunkIndex * maxDataPerMsg;
            size_t colorBufferBytes = (size_t)receiveStatus.width * (size_t)receiveStatus.height * 3U * sizeof(uint8_t);

            if((size_t)(offset + header->dataSize) <= colorBufferBytes){
              memcpy(receiveStatus.colorDataBuffer + offset, msgBuffer + sizeof(MessageHeader), header->dataSize);
              receiveStatus.receivedColorChunks++;
              receiveStatus.totalColorChunks = header->totalChunks;
            }
          }

          pthread_mutex_unlock(&dataMutex);
          break;
      }

      pthread_mutex_lock(&dataMutex);

      if(receiveStatus.totalDepthChunks > 0 && receiveStatus.totalColorChunks > 0 &&
         receiveStatus.receivedDepthChunks == receiveStatus.totalDepthChunks &&
         receiveStatus.receivedColorChunks == receiveStatus.totalColorChunks){

        if(currentWidth != receiveStatus.width || currentHeight != receiveStatus.height){
          int16_t* newDepthBuffer = (int16_t*)safe_malloc(receiveStatus.width * receiveStatus.height * sizeof(int16_t));
          uint8_t* newColorBuffer = (uint8_t*)safe_malloc(receiveStatus.width * receiveStatus.height * 3 * sizeof(uint8_t));

          if(!newDepthBuffer || !newColorBuffer){
            if(newDepthBuffer) free(newDepthBuffer);
            if(newColorBuffer) free(newColorBuffer);
            pthread_mutex_unlock(&dataMutex);
            continue;
          }

          if(depthDataBuffer) free(depthDataBuffer);
          if(colorDataBuffer) free(colorDataBuffer);

          depthDataBuffer = newDepthBuffer;
          colorDataBuffer = newColorBuffer;
          currentWidth = receiveStatus.width;
          currentHeight = receiveStatus.height;
        }

        if(depthDataBuffer && colorDataBuffer){
          memcpy(depthDataBuffer, receiveStatus.depthDataBuffer, currentWidth * currentHeight * sizeof(int16_t));
          memcpy(colorDataBuffer, receiveStatus.colorDataBuffer, currentWidth * currentHeight * 3 * sizeof(uint8_t));

          hasNewData = 1;

          // Feed frame to VO worker (non-blocking drop if queue full)
          if(viewerVoActive && voIsRunning()){
            voProcessFrame(depthDataBuffer, colorDataBuffer,
                           currentWidth, currentHeight, receiveStatus.timestamp);
          }

#ifdef YOUTH_ZMQ_ENABLED
          {
            uint64_t ts_ns = (uint64_t)receiveStatus.timestamp * 1000000ULL;
            zmq_pub_depth_frame(ts_ns,
                                (uint16_t)currentWidth, (uint16_t)currentHeight,
                                depthDataBuffer, colorDataBuffer);
          }
#endif
        }

        receiveStatus.isComplete = 1;
      }

      pthread_mutex_unlock(&dataMutex);
    }else{
      usleep(10000);
      continue;
    }
  }

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

// Draw a string using GLUT bitmap fonts at the given raster position
static void drawBitmapString(float x, float y, void* font, const char* str){
  glRasterPos2f(x, y);
  for(const char* c = str; *c; c++){
    glutBitmapCharacter(font, *c);
  }
}

// Draw HUD overlay: recording/playback status + key hints
static void drawHUD(){
  int w = windowWidth;
  int h = windowHeight;
  if(w <= 0 || h <= 0) return;

  glDisable(GL_DEPTH_TEST);
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, w, 0, h);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  // Status indicator box (top-left)
  float bx = 15.0f, by = (float)h - 38.0f, bw = 22.0f, bh = 22.0f;

  if(viewerIsRecording){
    glColor3f(0.9f, 0.15f, 0.15f); // red
  }else if(viewerIsPlayback){
    glColor3f(0.15f, 0.85f, 0.15f); // green
  }else{
    glColor3f(0.45f, 0.45f, 0.45f); // grey
  }

  glBegin(GL_QUADS);
  glVertex2f(bx,      by);
  glVertex2f(bx + bw, by);
  glVertex2f(bx + bw, by + bh);
  glVertex2f(bx,      by + bh);
  glEnd();

  // Status text next to box
  glColor3f(1.0f, 1.0f, 1.0f);
  if(viewerIsRecording){
    char msg[300];
    snprintf(msg, sizeof(msg), " REC  %s", lastRecordedFilename);
    drawBitmapString(bx + bw + 4, by + 5, GLUT_BITMAP_HELVETICA_18, msg);
  }else if(viewerIsPlayback){
    char msg[300];
    snprintf(msg, sizeof(msg), " PLAY  %s", lastRecordedFilename);
    drawBitmapString(bx + bw + 4, by + 5, GLUT_BITMAP_HELVETICA_18, msg);
  }else{
    drawBitmapString(bx + bw + 4, by + 5, GLUT_BITMAP_HELVETICA_18, " IDLE");
  }

  // VO status (second row)
  if(viewerVoActive && voIsRunning()){
    float voBy = by - 28.0f;
    int trackSt = voGetTrackingStatus();
    if(trackSt == 1){
      glColor3f(0.2f, 0.9f, 1.0f);  // cyan = tracking OK
    }else if(trackSt == 2){
      glColor3f(1.0f, 0.6f, 0.1f);  // orange = lost
    }else{
      glColor3f(0.6f, 0.6f, 0.6f);
    }

    glBegin(GL_QUADS);
    glVertex2f(bx,      voBy);
    glVertex2f(bx + bw, voBy);
    glVertex2f(bx + bw, voBy + bh);
    glVertex2f(bx,      voBy + bh);
    glEnd();

    glColor3f(1.0f, 1.0f, 1.0f);
    char voMsg[64];
    snprintf(voMsg, sizeof(voMsg), " VO  pts:%d  frm:%d",
             voGetMapPointCount(), voGetFrameCount());
    drawBitmapString(bx + bw + 4, voBy + 5, GLUT_BITMAP_HELVETICA_18, voMsg);
  }

  // Key hints at the bottom
  glColor3f(0.75f, 0.75f, 0.75f);
  drawBitmapString(10, 10, GLUT_BITMAP_HELVETICA_12,
    "R:Record  P:Playback  S:Stop  V:VO toggle  X:VO reset  |  Left:Pan  Right:Rotate  Scroll:Zoom  ESC:Quit");

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glEnable(GL_DEPTH_TEST);
}

// 3D Rendering
void display_3d_color(){
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glLoadIdentity();
  // Pan in camera space, then pull back by distance, then rotate, then scale
  glTranslatef(panX, panY, -distance);
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
          float z_pos = depthValue / 1000.0f;
          float x_pos = (x - currentWidth * 0.5f) * z_pos  / 570.3f;
          float y_pos = (y - currentHeight * 0.5f) * z_pos / 570.3f;

          int colorIndex = index * 3;
          float r = colorDataBuffer[colorIndex]     / 255.0f;
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

  // ── VO accumulated map ────────────────────────────────────────────────────
  // When VO is active and has data, draw the accumulated map instead of / in
  // addition to the live cloud.  We use a vertex array for performance.
  if(viewerVoActive && voIsRunning()){
    int currentMapCount = voGetMapPointCount();
    if(currentMapCount != s_voLastSyncCount && currentMapCount > 0){
      s_voDisplayCount  = voGetMapPoints(s_voDisplayBuf, VO_MAX_DISPLAY_POINTS);
      s_voLastSyncCount = currentMapCount;
    }

    if(s_voDisplayCount > 0){
      glPointSize(1.5f);
      glEnableClientState(GL_VERTEX_ARRAY);
      glEnableClientState(GL_COLOR_ARRAY);

      // Each point: [x, y, z, r, g, b] — stride = 6 floats
      glVertexPointer(3, GL_FLOAT, 6 * sizeof(float), s_voDisplayBuf);
      glColorPointer(3,  GL_FLOAT, 6 * sizeof(float), s_voDisplayBuf + 3);
      glDrawArrays(GL_POINTS, 0, s_voDisplayCount);

      glDisableClientState(GL_VERTEX_ARRAY);
      glDisableClientState(GL_COLOR_ARRAY);
      glPointSize(2.0f);
    }
  }

  drawHUD();

  if(window){
    glfwSwapBuffers(window);
  }
}

// GLFW Error Callback
void error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

// Window close callback
void window_close_callback(GLFWwindow* win){
  (void)win;
  printf("Window close callback triggered\n");
  viewerIsRunning = 0;

  if(exitCallback){
    exitCallback();
  }
}

// Keyboard Callback
void key_callback(GLFWwindow* win, int key, int scancode, int action, int mods)
{
  (void)scancode;
  (void)mods;

  if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS){
    printf("ESC key pressed\n");
    glfwSetWindowShouldClose(win, GLFW_TRUE);
    return;
  }

  // R: start recording (auto timestamped filename)
  if(key == GLFW_KEY_R && action == GLFW_PRESS){
    if(!viewerIsRecording && !viewerIsPlayback){
      time_t t = time(NULL);
      struct tm* tm_info = localtime(&t);
      strftime(lastRecordedFilename, sizeof(lastRecordedFilename),
               "recording_%Y%m%d_%H%M%S.bin", tm_info);
      viewerSendControlCommand(CTRL_CMD_START_RECORD, lastRecordedFilename);
      viewerIsRecording = 1;
      printf("Recording started: %s\n", lastRecordedFilename);
    }else if(viewerIsRecording){
      printf("Already recording. Press S to stop.\n");
    }else{
      printf("Playback in progress. Press S to stop first.\n");
    }
  }

  // P: start playback of last recorded file
  if(key == GLFW_KEY_P && action == GLFW_PRESS){
    if(!viewerIsPlayback && !viewerIsRecording){
      if(lastRecordedFilename[0] != '\0'){
        viewerSendControlCommand(CTRL_CMD_START_PLAYBACK, lastRecordedFilename);
        viewerIsPlayback = 1;
        printf("Playback started: %s\n", lastRecordedFilename);
      }else{
        printf("No recording available. Press R to record first.\n");
      }
    }else if(viewerIsPlayback){
      printf("Already playing. Press S to stop.\n");
    }else{
      printf("Recording in progress. Press S to stop first.\n");
    }
  }

  // S: stop recording or playback
  if(key == GLFW_KEY_S && action == GLFW_PRESS){
    if(viewerIsRecording){
      viewerSendControlCommand(CTRL_CMD_STOP_RECORD, NULL);
      viewerIsRecording = 0;
      printf("Recording stopped: %s\n", lastRecordedFilename);
    }
    if(viewerIsPlayback){
      viewerSendControlCommand(CTRL_CMD_STOP_PLAYBACK, NULL);
      viewerIsPlayback = 0;
      printf("Playback stopped\n");
    }
  }

  // V: toggle Visual Odometry on / off
  if(key == GLFW_KEY_V && action == GLFW_PRESS){
    if(!viewerVoActive){
      // Astra camera intrinsics
      voInit(570.3f, 570.3f, 320.0f, 240.0f);
      viewerVoActive = 1;
      s_voDisplayCount  = 0;
      s_voLastSyncCount = 0;
      printf("VO started\n");
    }else{
      voStop();
      viewerVoActive    = 0;
      s_voDisplayCount  = 0;
      s_voLastSyncCount = 0;
      printf("VO stopped\n");
    }
  }

  // X: reset VO map (keep VO running but clear accumulated points)
  if(key == GLFW_KEY_X && action == GLFW_PRESS){
    if(viewerVoActive && voIsRunning()){
      voReset();
      s_voDisplayCount  = 0;
      s_voLastSyncCount = 0;
      printf("VO map reset\n");
    }
  }
}

// Mouse Button Callback
// Left button  → pan
// Right button → rotate / tilt
void mouse_button_callback(GLFWwindow* win, int button, int action, int mods)
{
  (void)mods;
  double cx, cy;

  if(button == GLFW_MOUSE_BUTTON_LEFT){
    if(action == GLFW_PRESS){
      isDragging = 1;
      glfwGetCursorPos(win, &cx, &cy);
      lastMouseX = (int)cx;
      lastMouseY = (int)cy;
    }else if(action == GLFW_RELEASE){
      isDragging = 0;
    }
  }

  if(button == GLFW_MOUSE_BUTTON_RIGHT){
    if(action == GLFW_PRESS){
      isDraggingRight = 1;
      glfwGetCursorPos(win, &cx, &cy);
      lastRightX = (int)cx;
      lastRightY = (int)cy;
    }else if(action == GLFW_RELEASE){
      isDraggingRight = 0;
    }
  }
}

// Mouse Move Callback
// Left drag  → pan (matching lidar-core 0.002/zoom scale)
// Right drag → rotate (matching lidar-core 0.2 deg/px)
void cursor_position_callback(GLFWwindow* win, double xpos, double ypos)
{
  (void)win;

  if(isDragging){
    int dx = (int)xpos - lastMouseX;
    int dy = (int)ypos - lastMouseY;

    // Pan in camera space; divide by zoom so pan feels same speed at any zoom level
    float safeZoom = (zoom > 0.01f) ? zoom : 0.01f;
    panX -= dx * 0.002f / safeZoom;
    panY += dy * 0.002f / safeZoom;

    lastMouseX = (int)xpos;
    lastMouseY = (int)ypos;
  }

  if(isDraggingRight){
    int dx = (int)xpos - lastRightX;
    int dy = (int)ypos - lastRightY;

    // Rotation sensitivity 0.2 deg/px — matches lidar-core viewer
    angleY += dx * 0.2f;
    angleX += dy * 0.2f;

    lastRightX = (int)xpos;
    lastRightY = (int)ypos;
  }
}

// Scroll Callback — additive zoom matching lidar-core step feel
void scroll_callback(GLFWwindow* win, double xoffset, double yoffset)
{
  (void)win;
  (void)xoffset;

  const float step = 0.1f;
  if(yoffset > 0){
    zoom += step;
    if(zoom > 20.0f) zoom = 20.0f;
  }else if(yoffset < 0){
    zoom -= step;
    if(zoom < 0.05f) zoom = 0.05f;
  }
}

// Framebuffer resize callback
void framebuffer_size_callback(GLFWwindow* win, int width, int height)
{
  (void)win;
  if(width <= 0 || height <= 0) return;

  windowWidth  = width;
  windowHeight = height;

  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0, (double)width / (double)height, 0.1, 100.0);
  glMatrixMode(GL_MODELVIEW);
}

// Viewer Thread
void* viewerThread(void* id){
  (void)id;
  printf("Initializing GLFW...\n");

  // Initialize GLUT for bitmap font rendering (no mainloop used)
  int fakeArgc = 0;
  char* fakeArgv[] = {NULL};
  glutInit(&fakeArgc, fakeArgv);

  if(!glfwInit()){
    fprintf(stderr, "Failed to initialize GLFW\n");
    viewerIsRunning = 0;
    return NULL;
  }

  glfwSetErrorCallback(error_callback);

  int screenWidth = 800;
  int screenHeight = 600;

#ifdef _WIN32
  screenWidth  = GetSystemMetrics(SM_CXSCREEN);
  screenHeight = GetSystemMetrics(SM_CYSCREEN);
#elif __linux__
  Display* d = XOpenDisplay(NULL);
  if(d){
    Screen* s = DefaultScreenOfDisplay(d);
    screenWidth  = s->width;
    screenHeight = s->height;
    XCloseDisplay(d);
  }
#endif

  window = glfwCreateWindow(screenWidth, screenHeight, "3D Viewer", NULL, NULL);
  if(!window){
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

  GLenum err = glewInit();
  if(err != GLEW_OK){
    fprintf(stderr, "GLEW initialization error: %s\n", glewGetErrorString(err));
    glfwDestroyWindow(window);
    glfwTerminate();
    viewerIsRunning = 0;
    return NULL;
  }

  initOpenGL();

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_POINT_SMOOTH);
  glPointSize(2.0f);

  int width, height;
  glfwGetFramebufferSize(window, &width, &height);
  framebuffer_size_callback(window, width, height);

  // Start data receive thread
  pthread_t receiveThreadId;
  if(pthread_create(&receiveThreadId, NULL, dataReceiveThread, NULL) != 0){
    fprintf(stderr, "Failed to create data receive thread\n");
    glfwDestroyWindow(window);
    glfwTerminate();
    viewerIsRunning = 0;
    return NULL;
  }

  printf("Viewer ready. R=Record  P=Playback  S=Stop  | Left drag=Pan  Right drag=Rotate  Scroll=Zoom\n");

  // Main loop
  while(viewerIsRunning && window && !glfwWindowShouldClose(window)){
    // Update window title with sensor + recording/playback state
    char windowTitle[512];
    char stateStr[128] = "";
    if(viewerIsRecording){
      snprintf(stateStr, sizeof(stateStr), " [REC]");
    }else if(viewerIsPlayback){
      snprintf(stateStr, sizeof(stateStr), " [PLAY]");
    }
    if(viewerVoActive && voIsRunning()){
      int trackSt = voGetTrackingStatus();
      const char* voSuffix = (trackSt == 1) ? " [VO:OK]" : (trackSt == 2) ? " [VO:LOST]" : " [VO]";
      strncat(stateStr, voSuffix, sizeof(stateStr) - strlen(stateStr) - 1);
    }

    pthread_mutex_lock(&viewerStatusMutex);
    snprintf(windowTitle, sizeof(windowTitle), "3D Viewer%s - %s", stateStr, viewerStatusText);
    pthread_mutex_unlock(&viewerStatusMutex);
    glfwSetWindowTitle(window, windowTitle);

    display_3d_color();
    glfwPollEvents();

    usleep(10000); // 10ms
  }

  printf("Exited viewer main loop\n");

  viewerIsRunning = 0;
  pthread_join(receiveThreadId, NULL);

  // Stop VO if it was running
  if(viewerVoActive && voIsRunning()){
    voStop();
    viewerVoActive = 0;
  }

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

  pthread_mutex_unlock(&dataMutex);
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

  if(window){
    glfwSetWindowShouldClose(window, GLFW_TRUE);
    usleep(100000);
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
