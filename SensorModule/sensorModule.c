#define _GNU_SOURCE

#include "sensorModule.h"
#include "astra_wrapper.h"
#include "../frameDefinitions.h"
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <mqueue.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>
#include <stdarg.h>

// Sensor Context
AstraContext_t* sensorContext = NULL;

// Message Queue Handle
mqd_t mqSend = -1;

// Allow queue backpressure, but bound waits during startup and shutdown.
static int sendSensorMessage(const char* data, size_t size){
  struct timespec deadline;
  clock_gettime(CLOCK_REALTIME, &deadline);
  deadline.tv_nsec += 100000000L;
  if(deadline.tv_nsec >= 1000000000L){
    deadline.tv_sec++;
    deadline.tv_nsec -= 1000000000L;
  }
  return mq_timedsend(mqSend, data, size, 0, &deadline);
}

// Thread Control Variable;
static int sensorIsRunning = 1;
static int sensorIsReady = 0;

// 에러 카운터
static int errorCounter = 0;
static const int MAX_CONSECUTIVE_ERRORS = 5;

// 뮤텍스
static pthread_mutex_t sensorMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t sensorStatusMutex = PTHREAD_MUTEX_INITIALIZER;
static char sensorStatus[256] = "Sensor module not started";

static void setSensorStatus(const char* fmt, ...)
{
  va_list args;
  pthread_mutex_lock(&sensorStatusMutex);
  va_start(args, fmt);
  vsnprintf(sensorStatus, sizeof(sensorStatus), fmt, args);
  va_end(args);
  pthread_mutex_unlock(&sensorStatusMutex);
}

// Get Current Time (ms)
static uint32_t getCurrentTimeMs(){
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint32_t)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

// 안전한 메모리 할당 헬퍼 함수
static void* safe_malloc(size_t size){
  void*ptr = malloc(size);
  if(!ptr){
    perror("malloc failed");
    return NULL;
  }

  memset(ptr, 0, size); // 안전하게 0으로 초기화
  return ptr;
}

// 안전한 Astra 센서 초기화
static AstraContext_t* safeInitializeAstraObj(){
  printf("Attempting to initialize Astra sensor...\n");
  setSensorStatus("Attempting to initialize Astra sensor");

  // 여러 번 시도
  for(int attempt = 1; attempt <= 3; attempt++){
    setSensorStatus("Initializing sensor (attempt %d/3)", attempt);
    AstraContext_t* context = InitializeAstraObj();
    if(context){
      printf("Astra sensor initialized successfully on attempt %d\n", attempt);
      setSensorStatus("Sensor initialized successfully");
      return context;
    }

    printf("Astra initialization attempt %d failed, retrying...\n", attempt);
    setSensorStatus("%s", GetAstraLastError());
    usleep(500000); //0.5초 대기
  }

  printf("Failed to initialize Astra sensor after multiple attemps\n");
  if(strlen(GetAstraLastError()) == 0){
    setSensorStatus("Failed to initialize Astra sensor");
  }
  return NULL;
}

void* sensorLoop(void* arg){
  (void)arg;
  printf("Sensor thread started...\n");
  setSensorStatus("Sensor thread started");

  // Message Queue Feature Setting
  struct mq_attr attr;
  attr.mq_flags = 0;
  attr.mq_maxmsg = 10;
  attr.mq_msgsize = MAX_MSG_SIZE;
  attr.mq_curmsgs = 0;

  // Message Queue
  mqSend = mq_open(MQ_SENSOR_TO_LOGGER, O_WRONLY, 0644, &attr);
  if(mqSend == (mqd_t) - 1){
    perror("mq_open send");
    sensorIsRunning = 0;
    sensorIsReady = 0;
    setSensorStatus("Failed to open sensor->logger message queue");
    return NULL;
  }

  // Initialize Sensor
  printf("Initializing Astra sensor ...\n");
  sensorContext = safeInitializeAstraObj();
  
  if(!sensorContext){
    printf("Failed to initialize Astra sensor\n");
    mq_close(mqSend);
    sensorIsRunning = 0;
    sensorIsReady = 0;
    if(strlen(GetAstraLastError()) > 0){
      setSensorStatus("%s", GetAstraLastError());
    }else{
      setSensorStatus("Failed to initialize Astra sensor");
    }
    return NULL;
  }

  // 데이터 청크를 위한 버퍼 할당
  char* msgBuffer = safe_malloc(MAX_MSG_SIZE);
  if(!msgBuffer){
    perror("malloc message buffer");
    TerminateAstraObj(sensorContext);
    mq_close(mqSend);
    sensorIsRunning = 0;
    sensorIsReady = 0;
    setSensorStatus("Failed to allocate sensor message buffer");
    return NULL;
  }

  int frameId = 0;
  errorCounter = 0;

  // Main Loop
  while(sensorIsRunning){
    int width = 0, height = 0;

    pthread_mutex_lock(&sensorMutex);
    // Get Data from Sensor
    const int16_t* depthData = GetDepthDataAstraOpenGL(sensorContext, &width, &height);
    const uint8_t* colorData = GetColorDataAstraOpenGL(sensorContext, &width, &height);
    pthread_mutex_unlock(&sensorMutex);

    if(depthData && colorData && width > 0 && height > 0){
      errorCounter = 0; // 성공적으로 데이터를 받았으므로 에러 카운터 리셋
      if(!sensorIsReady){
        sensorIsReady = 1;
        setSensorStatus("Sensor ready: streaming %dx%d", width, height);
      }

      frameId++;
      uint32_t timestamp = getCurrentTimeMs();

      // 1. 메타데이터 메세지 전송
      MessageHeader* header = (MessageHeader*)msgBuffer;
      header->msgType = MSG_TYPE_METADATA;
      header->width = width;
      header->height = height;
      header->chunkIndex = 0;
      header->totalChunks = 0; // 메타데이터에는 의미 없음
      header->dataSize = 0;
      header->frameId = frameId;
      header->timestamp = timestamp;

      if(sendSensorMessage(msgBuffer, sizeof(MessageHeader)) == -1){
        perror("mq_send metadata");
        continue;
      }

      // 2. 깊이 데이터 청크 전송
      int depthDataSize = width * height * sizeof(int16_t);
      int maxDataPerMsg = MAX_MSG_SIZE - sizeof(MessageHeader);
      int totalDepthChunks = (depthDataSize + maxDataPerMsg - 1) / maxDataPerMsg;

      int depthChunkesSuccess = 1;
      for(int i = 0; i < totalDepthChunks && depthChunkesSuccess; i++){
        header->msgType = MSG_TYPE_DEPTH_DATA;
        header->chunkIndex = i;
        header->totalChunks = totalDepthChunks;
        header->frameId = frameId;
        header->timestamp = timestamp;

        int offset = i * maxDataPerMsg;
        int chunkSize = (i == totalDepthChunks - 1) ? (depthDataSize - offset) : maxDataPerMsg;

        // 안전성 검사 추가
        if(offset < 0 || offset + chunkSize > depthDataSize){
          printf("Error: depth chunk offset calculation error\n");
          depthChunkesSuccess = 0;
          break;
        }

        header->dataSize = chunkSize;

        // 데이터 복사
        memcpy(msgBuffer + sizeof(MessageHeader), ((const char*)depthData) + offset, chunkSize);

        if(sendSensorMessage(msgBuffer, sizeof(MessageHeader) + chunkSize) == -1){
          if(errno != EAGAIN){
            perror("mq_send depth chunk");
          }
          depthChunkesSuccess = 0;
          break;
        }
      }

      // 깊이 데이터 전송에 실패했으면 다음 반복으로
      if(!depthChunkesSuccess) continue;

      // 3. 색상 데이터 청크 전송
      int colorDataSize = width * height * 3 * sizeof(uint8_t);
      int totalColorChunks = (colorDataSize + maxDataPerMsg - 1) / maxDataPerMsg;

      for(int i = 0; i < totalColorChunks; i++){
        header->msgType = MSG_TYPE_COLOR_DATA;
        header->chunkIndex = i;
        header->totalChunks = totalColorChunks;
        header->frameId = frameId;
        header->timestamp = timestamp;

        int offset = i * maxDataPerMsg;
        int chunkSize = (i == totalColorChunks - 1) ? (colorDataSize - offset) : maxDataPerMsg;

        // 안전성 검사 추가
        if(offset < 0 || offset + chunkSize > colorDataSize){
          printf("Error: color chunk offset calculation error\n");
          break;
        }

        header->dataSize = chunkSize;

        // 데이터 복사
        memcpy(msgBuffer + sizeof(MessageHeader), ((const char*)colorData) + offset, chunkSize);

        if(sendSensorMessage(msgBuffer, sizeof(MessageHeader) + chunkSize) == -1){
          if(errno != EAGAIN){
            perror("mq_send color chunk");
          }
          break;
        }
      }
    }else{
      // 데이터를 가져오는데 실패
      sensorIsReady = 0;
      errorCounter++;
      printf("Failed to get data from sensor (attempt %d/%d)\n", errorCounter, MAX_CONSECUTIVE_ERRORS);
      if(strlen(GetAstraLastError()) > 0){
        setSensorStatus("%s", GetAstraLastError());
      }else{
        setSensorStatus("Failed to get frames from sensor (%d/%d)", errorCounter, MAX_CONSECUTIVE_ERRORS);
      }

      if(errorCounter >= MAX_CONSECUTIVE_ERRORS){
        printf("Too many consecutive failures, attempting to reinitialize sensor...\n");
        setSensorStatus("Reinitializing sensor after repeated frame failures");

        // 센서 재초기화 시도
        pthread_mutex_lock(&sensorMutex);
        if(sensorContext){
          TerminateAstraObj(sensorContext);
          sensorContext = NULL;
        }

        // 5초 정도 대기 후 재시도
        usleep(5000000);

        sensorContext = safeInitializeAstraObj();
        pthread_mutex_unlock(&sensorMutex);

        if(!sensorContext){
          printf("Failed to reinitialize sensor, terminating sensor thread\n");
          sensorIsRunning = 0;
          sensorIsReady = 0;
          if(strlen(GetAstraLastError()) > 0){
            setSensorStatus("%s", GetAstraLastError());
          }else{
            setSensorStatus("Failed to reinitialize sensor");
          }
          break;
        }

        errorCounter = 0;
      }
    }

    // Data Collection Speed Control (About 30 FPS)
    usleep(33333);
  }

  // 메모리 해제
  free(msgBuffer);

  // Terminate Sensor
  pthread_mutex_lock(&sensorMutex);
  if(sensorContext){
    TerminateAstraObj(sensorContext);
    sensorContext = NULL;
  }
  sensorIsReady = 0;

  // Close Message Queue
  if(mqSend != (mqd_t) - 1){
    mq_close(mqSend);
    mqSend = (mqd_t) - 1;
  }

  printf("Sensor thread termination\n");
  if(sensorIsRunning == 0 && strlen(sensorStatus) == 0){
    setSensorStatus("Sensor thread terminated");
  }
  return NULL;
}

pthread_t sensor_thread_id;

void initSensorModule(){
  printf("Initializing sensor module...\n");
  sensorIsRunning = 1;
  sensorIsReady = 0;
  setSensorStatus("Starting sensor module");
  if(pthread_create(&sensor_thread_id, NULL, sensorLoop, NULL) != 0){
    perror("Failed to create sensor thread");
    sensorIsRunning = 0;
    setSensorStatus("Failed to create sensor thread");
  }
}

void stopSensorModule(){
  printf("Stopping sensor module...\n");
  sensorIsRunning = 0;
  sensorIsReady = 0;
  setSensorStatus("Stopping sensor module");

  // 스레드가 적절히 종료될 때까지 대기
  struct timespec timeout;
  clock_gettime(CLOCK_REALTIME, &timeout);
  timeout.tv_sec += 3; // 3초 타임아웃 

  int joined = (pthread_timedjoin_np(sensor_thread_id, NULL, &timeout) == 0);
  if(!joined){
    printf("Sensor thread did not terminate within timeout; leaving SDK cleanup to the sensor thread.\n");
    return;
  }

  // 센서 컨텍스트 정리
  pthread_mutex_lock(&sensorMutex);
  if(sensorContext){
    TerminateAstraObj(sensorContext);
    sensorContext = NULL;
  }
  pthread_mutex_unlock(&sensorMutex);

  // 메세지 큐 정리
  if(mqSend != (mqd_t) - 1){
    mq_close(mqSend);
    mqSend = (mqd_t) - 1;
  }

  printf("Sensor module stopped\n");
  setSensorStatus("Sensor module stopped");
}

int isSensorModuleRunning(void){
  return sensorIsRunning;
}

int isSensorModuleReady(void){
  return sensorIsReady;
}

const char* getSensorModuleStatus(void){
  return sensorStatus;
}

void requestStopSensorModule(){
  sensorIsRunning = 0;
}
