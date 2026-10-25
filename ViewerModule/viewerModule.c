#define _GNU_SOURCE

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
#include <stdatomic.h>

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
static int windowWidth = 1280;
static int windowHeight = 720;

static int showDepth = 0;
static unsigned long inputFrames = 0;
static float inputFps = 0;
static uint8_t* previewPixels = NULL;
static size_t previewCapacity = 0;

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
static atomic_int viewerVoActive = 0;  // 0 = off, 1 = on

// VO map display buffer — updated incrementally (only when map grows)
#define VO_MAX_DISPLAY_POINTS 300000
static float  s_voDisplayBuf[VO_MAX_DISPLAY_POINTS * 6];
static int    s_voDisplayCount  = 0;
static int    s_voLastSyncCount = 0;  // how many points were copied last time

static pthread_mutex_t resultMutex = PTHREAD_MUTEX_INITIALIZER;
static float resultPoints[VO_MAX_DISPLAY_POINTS * 6];
static float resultPath[10000 * 3];
static int resultPointCount=0, resultPathCount=0, resultAvailable=0;
static float resultFps=0;
static char resultStatus[96]="Waiting";

int viewerPublishSlamResult(const float* points, int count, const float* path,
                           int pathCount, const char* status, float fps){
  if(count<0 || count>VO_MAX_DISPLAY_POINTS || pathCount<0 || pathCount>10000 ||
     (count && !points) || (pathCount && !path)) return 0;
  pthread_mutex_lock(&resultMutex);
  if(count) memcpy(resultPoints,points,(size_t)count*6*sizeof(float));
  if(pathCount) memcpy(resultPath,path,(size_t)pathCount*3*sizeof(float));
  resultPointCount=count; resultPathCount=pathCount;
  resultAvailable=status!=NULL || count>0 || pathCount>0;
  resultFps=fps;
  snprintf(resultStatus,sizeof(resultStatus),"%s",status?status:"Waiting");
  pthread_mutex_unlock(&resultMutex);
  return 1;
}

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
  unsigned char* depthChunkMarks;
  unsigned char* colorChunkMarks;
  char* depthDataBuffer;
  char* colorDataBuffer;
  int isComplete;
} FrameReceiveStatus;

FrameReceiveStatus receiveStatus = {0};

static void setTimeoutFromNow(struct timespec* ts, long timeoutMs){
  clock_gettime(CLOCK_REALTIME, ts);
  ts->tv_sec += timeoutMs / 1000;
  ts->tv_nsec += (timeoutMs % 1000) * 1000000L;

  if(ts->tv_nsec >= 1000000000L){
    ts->tv_sec += 1;
    ts->tv_nsec -= 1000000000L;
  }
}

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
  if(width <= 0 || height <= 0){
    return;
  }

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
  if(status->depthChunkMarks) free(status->depthChunkMarks);
  if(status->colorChunkMarks) free(status->colorChunkMarks);

  status->frameId = frameId;
  status->timestamp = timestamp;
  status->width = width;
  status->height = height;
  status->receivedDepthChunks = 0;
  status->totalDepthChunks = 0;
  status->receivedColorChunks = 0;
  status->totalColorChunks = 0;
  status->depthChunkMarks = NULL;
  status->colorChunkMarks = NULL;
  status->isComplete = 0;
  status->depthDataBuffer = newDepthBuffer;
  status->colorDataBuffer = newColorBuffer;
}

static int markChunkReceived(unsigned char** marks, int* receivedCount,
                             int* totalChunks, int chunkIndex, int newTotalChunks){
  if(newTotalChunks <= 0 || chunkIndex < 0 || chunkIndex >= newTotalChunks){
    return 0;
  }

  if(*marks == NULL){
    *marks = (unsigned char*)calloc((size_t)newTotalChunks, sizeof(unsigned char));
    if(!*marks){
      perror("calloc chunk marks");
      return 0;
    }
    *totalChunks = newTotalChunks;
  }

  if(*totalChunks != newTotalChunks){
    return 0;
  }

  if((*marks)[chunkIndex]){
    return 1;
  }

  (*marks)[chunkIndex] = 1;
  (*receivedCount)++;
  return 1;
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
  receiveStatus.depthChunkMarks = NULL;
  receiveStatus.colorChunkMarks = NULL;

  while(viewerIsRunning){
    struct timespec receiveTimeout;
    setTimeoutFromNow(&receiveTimeout, 100);
    ssize_t byteRead = mq_timedreceive(mqReceive, msgBuffer, MAX_MSG_SIZE, NULL, &receiveTimeout);

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

            if(offset >= 0 && header->dataSize > 0 &&
               (size_t)(offset + header->dataSize) <= depthBufferBytes){
              memcpy(receiveStatus.depthDataBuffer + offset, msgBuffer + sizeof(MessageHeader), header->dataSize);
              markChunkReceived(&receiveStatus.depthChunkMarks,
                                &receiveStatus.receivedDepthChunks,
                                &receiveStatus.totalDepthChunks,
                                header->chunkIndex,
                                header->totalChunks);
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

            if(offset >= 0 && header->dataSize > 0 &&
               (size_t)(offset + header->dataSize) <= colorBufferBytes){
              memcpy(receiveStatus.colorDataBuffer + offset, msgBuffer + sizeof(MessageHeader), header->dataSize);
              markChunkReceived(&receiveStatus.colorChunkMarks,
                                &receiveStatus.receivedColorChunks,
                                &receiveStatus.totalColorChunks,
                                header->chunkIndex,
                                header->totalChunks);
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
          inputFrames++;

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
    }else if(errno != ETIMEDOUT && errno != EINTR){
      perror("mq_timedreceive viewer");
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
  if(receiveStatus.depthChunkMarks){
    free(receiveStatus.depthChunkMarks);
    receiveStatus.depthChunkMarks = NULL;
  }
  if(receiveStatus.colorChunkMarks){
    free(receiveStatus.colorChunkMarks);
    receiveStatus.colorChunkMarks = NULL;
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

// All layout dimensions are framebuffer pixels; pointer hit tests use window pixels.
static void overlayProjection(void){
  glViewport(0, 0, windowWidth, windowHeight);
  glDisable(GL_DEPTH_TEST);
  glMatrixMode(GL_PROJECTION); glLoadIdentity();
  gluOrtho2D(0, windowWidth, 0, windowHeight);
  glMatrixMode(GL_MODELVIEW); glLoadIdentity();
}

static void panelRect(float x, float y, float w, float h){
  glBegin(GL_QUADS);
  glVertex2f(x,y); glVertex2f(x+w,y); glVertex2f(x+w,y+h); glVertex2f(x,y+h);
  glEnd();
}

static void drawHUD(void){
  overlayProjection();
  int half = windowWidth / 2;
  glColor3f(.09f,.12f,.17f);
  panelRect(0,windowHeight-84,windowWidth,84);
  panelRect(0,0,windowWidth,56);
  glColor3f(.22f,.29f,.36f); panelRect(half-1,56,2,windowHeight-140);
  glColor3f(.3f,.85f,.85f);
  drawBitmapString(20,windowHeight-30,GLUT_BITMAP_HELVETICA_18,"LIVE INPUT");
  drawBitmapString(half+20,windowHeight-30,GLUT_BITMAP_HELVETICA_18,"SLAM WORKSPACE");
  char text[256];
  pthread_mutex_lock(&dataMutex);
  snprintf(text,sizeof(text),"%s  |  %d x %d  |  %.1f FPS",showDepth?"DEPTH (0-5m)":"RGB",currentWidth,currentHeight,inputFps);
  pthread_mutex_unlock(&dataMutex);
  glColor3f(.75f,.8f,.86f);
  drawBitmapString(20,windowHeight-60,GLUT_BITMAP_HELVETICA_12,text);
  snprintf(text,sizeof(text),"%s  |  points %d  |  frames %d",
    viewerVoActive ? (voGetTrackingStatus()==1?"Experimental VO: tracking":voGetTrackingStatus()==2?"Experimental VO: lost":"Experimental VO: initializing") : "Waiting for SLAM backend",
    viewerVoActive?voGetMapPointCount():0,viewerVoActive?voGetFrameCount():0);
  drawBitmapString(half+20,windowHeight-60,GLUT_BITMAP_HELVETICA_12,text);
  pthread_mutex_lock(&resultMutex);
  if(resultAvailable){
    glColor3f(.09f,.12f,.17f); panelRect(half+10,windowHeight-78,half-20,30);
    glColor3f(.3f,.85f,.85f);
    snprintf(text,sizeof(text),"%s | %.1f FPS | %d points",resultStatus,resultFps,resultPointCount);
    drawBitmapString(half+20,windowHeight-60,GLUT_BITMAP_HELVETICA_12,text);
  }
  int waiting=!viewerVoActive && !resultAvailable;
  pthread_mutex_unlock(&resultMutex);
  if(waiting){
    glColor3f(.75f,.8f,.86f);
    drawBitmapString(half+30,windowHeight/2+20,GLUT_BITMAP_HELVETICA_18,"SLAM backend not connected");
    drawBitmapString(half+30,windowHeight/2-10,GLUT_BITMAP_HELVETICA_12,"Map results will appear here. V: experimental VO preview");
  }
  glColor3f(.75f,.8f,.86f);
  drawBitmapString(18,34,GLUT_BITMAP_HELVETICA_12,"1:RGB  2:Depth  R:Record  P:Playback  S:Stop  ESC:Quit");
  drawBitmapString(half+18,34,GLUT_BITMAP_HELVETICA_12,"V:VO  X:Reset  Left:Pan  Right:Rotate  Wheel:Zoom");
  pthread_mutex_lock(&viewerStatusMutex);
  drawBitmapString(18,14,GLUT_BITMAP_HELVETICA_12,viewerStatusText);
  pthread_mutex_unlock(&viewerStatusMutex);
  glColor3f(viewerIsRecording?1:.4f,viewerIsRecording?.3f:.8f,.4f);
  drawBitmapString(half+18,14,GLUT_BITMAP_HELVETICA_12,viewerIsRecording?"RECORDING":viewerIsPlayback?"PLAYBACK":"LIVE");
}

void display_3d_color(void){
  if(windowWidth<2 || windowHeight<142) return;
  glClearColor(.035f,.05f,.075f,1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  overlayProjection();
  int half=windowWidth/2, areaHeight=windowHeight-140;
  static double fpsTime=0;
  static unsigned long fpsFrames=0;
  double now=glfwGetTime();
  pthread_mutex_lock(&dataMutex);
  if(now-fpsTime>=1){
    inputFps=(inputFrames-fpsFrames)/(now-fpsTime);
    fpsFrames=inputFrames; fpsTime=now;
  }
  if(hasNewData && currentWidth>0 && currentHeight>0 && depthDataBuffer && colorDataBuffer){
    size_t size=(size_t)currentWidth*currentHeight*3;
    if(size>previewCapacity){
      uint8_t* next=realloc(previewPixels,size);
      if(next){previewPixels=next;previewCapacity=size;}
    }
    const uint8_t* pixels=colorDataBuffer;
    if(showDepth && previewCapacity>=size){
      for(size_t i=0;i<size/3;i++){
        unsigned int d=(uint16_t)depthDataBuffer[i];
        unsigned int v=d>5000?255:d*255/5000;
        previewPixels[3*i]=d?(uint8_t)v:0;
        previewPixels[3*i+1]=d?(uint8_t)(255-v):0;
        previewPixels[3*i+2]=d?180:0;
      }
      pixels=previewPixels;
    }
    glBindTexture(GL_TEXTURE_2D,depthTexture);
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,currentWidth,currentHeight,0,GL_RGB,GL_UNSIGNED_BYTE,pixels);
    float scale=(float)(half-32)/currentWidth;
    if(scale*currentHeight>areaHeight-24) scale=(float)(areaHeight-24)/currentHeight;
    float w=currentWidth*scale,h=currentHeight*scale,x=(half-w)/2,y=56+(areaHeight-h)/2;
    glEnable(GL_TEXTURE_2D); glColor3f(1,1,1);
    glBegin(GL_QUADS);
    glTexCoord2f(0,1);glVertex2f(x,y);glTexCoord2f(1,1);glVertex2f(x+w,y);
    glTexCoord2f(1,0);glVertex2f(x+w,y+h);glTexCoord2f(0,0);glVertex2f(x,y+h);
    glEnd();glDisable(GL_TEXTURE_2D);
  }
  pthread_mutex_unlock(&dataMutex);
  glViewport(half,56,windowWidth-half,areaHeight);
  glEnable(GL_DEPTH_TEST);
  glMatrixMode(GL_PROJECTION);glLoadIdentity();
  gluPerspective(45,(double)(windowWidth-half)/areaHeight,.1,100);
  glMatrixMode(GL_MODELVIEW);glLoadIdentity();
  glTranslatef(panX,panY,-distance);
  glRotatef(angleX,1,0,0);glRotatef(angleY,0,1,0);glScalef(zoom,zoom,zoom);
  glColor3f(.13f,.20f,.25f);
  glBegin(GL_LINES);
  for(int i=-10;i<=10;i++){
    glVertex3f(i*.5f,-1,-5);glVertex3f(i*.5f,-1,5);
    glVertex3f(-5,-1,i*.5f);glVertex3f(5,-1,i*.5f);
  }
  glEnd();
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

  pthread_mutex_lock(&resultMutex);
  if(resultAvailable){
    glEnableClientState(GL_VERTEX_ARRAY); glEnableClientState(GL_COLOR_ARRAY);
    glVertexPointer(3,GL_FLOAT,6*sizeof(float),resultPoints);
    glColorPointer(3,GL_FLOAT,6*sizeof(float),resultPoints+3);
    glDrawArrays(GL_POINTS,0,resultPointCount);
    glDisableClientState(GL_COLOR_ARRAY);
    glColor3f(.3f,.9f,.8f); glLineWidth(2);
    glVertexPointer(3,GL_FLOAT,0,resultPath);
    glDrawArrays(GL_LINE_STRIP,0,resultPathCount);
    glDisableClientState(GL_VERTEX_ARRAY); glLineWidth(1);
  }
  pthread_mutex_unlock(&resultMutex);
  drawHUD();
  if(window) glfwSwapBuffers(window);
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

  if(action == GLFW_PRESS && key == GLFW_KEY_1) showDepth = 0;
  if(action == GLFW_PRESS && key == GLFW_KEY_2) showDepth = 1;

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
  int ww, wh;
  glfwGetWindowSize(win, &ww, &wh);
  glfwGetCursorPos(win, &cx, &cy);
  if(action == GLFW_PRESS && (cx < ww/2 || cy < 84.0*wh/windowHeight || cy > wh-56.0*wh/windowHeight)) return;

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

  double cx,cy; int ww,wh;
  glfwGetWindowSize(win,&ww,&wh); glfwGetCursorPos(win,&cx,&cy);
  if(cx<ww/2 || cy<84.0*wh/windowHeight || cy>wh-56.0*wh/windowHeight) return;
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

  window = glfwCreateWindow(screenWidth>1440?1440:screenWidth, screenHeight>900?900:screenHeight, "RGB-D Workspace", NULL, NULL);
  if(!window){
    fprintf(stderr, "Failed to create GLFW window\n");
    glfwTerminate();
    viewerIsRunning = 0;
    return NULL;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  glfwSetWindowSizeLimits(window, 1000, 500, GLFW_DONT_CARE, GLFW_DONT_CARE);
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
    snprintf(windowTitle, sizeof(windowTitle), "RGB-D Workspace%s - %s", stateStr, viewerStatusText);
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

  free(previewPixels); previewPixels=NULL; previewCapacity=0;
  glDeleteTextures(1,&depthTexture);
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
