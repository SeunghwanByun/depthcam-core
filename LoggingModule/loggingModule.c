#define _GNU_SOURCE

#include "loggingModule.h"
#include "../frameDefinitions.h"
#include <pthread.h>
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
#include <sys/time.h>

// 스레드 제어 변수
static atomic_int loggingIsRunning = 1;
static int isRecordingData = 0;
static atomic_int isPlaybackActive = 0;
static atomic_int isPassThroughEnabled = 1; // 기본적으로 패스스루 활성화
uint32_t playbackFrameCounter = 0;


// 스레드 ID 
static pthread_t logger_thread_id;
static pthread_t playback_thread_id;

// 메세지 큐 핸들
static mqd_t mqFromSensor = -1;
static mqd_t mqToViewer = -1;
static mqd_t mqControl = -1;

// 파일 핸들
static FILE* recordFile = NULL;
static FILE* playbackFile = NULL;

// 뮤텍스
static pthread_mutex_t recordMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t playbackMutex = PTHREAD_MUTEX_INITIALIZER;

// 파일명 저장 변수
static char currentRecordFilename[256];
static char currentPlaybackFilename[256];
static PlaybackStatus playbackStatus={0};
static int playbackGeneration=0;
void getPlaybackStatus(PlaybackStatus* status){
  if(!status) return;
  pthread_mutex_lock(&playbackMutex);
  *status=playbackStatus;
  pthread_mutex_unlock(&playbackMutex);
}


// 프레임 카운터
static uint32_t frameCounter = 0;
static RecordingStatus recordingStatus = {0};

void getRecordingStatus(RecordingStatus* status){
  if(!status) return;
  pthread_mutex_lock(&recordMutex);
  *status = recordingStatus;
  pthread_mutex_unlock(&recordMutex);
}

// 기타 상태 변수
static int receivedDepthFrame = 0;
static int receivedColorFrame = 0;

// 버퍼
static char* depthBuffer = NULL;
static char* colorBuffer = NULL;
static int currentWidth = 0;
static int currentHeight = 0;
static int depthBufferSize = 0;
static int colorBufferSize = 0;

static void setTimeoutFromNow(struct timespec* ts, long timeoutMs){
  clock_gettime(CLOCK_REALTIME, ts);
  ts->tv_sec += timeoutMs / 1000;
  ts->tv_nsec += (timeoutMs % 1000) * 1000000L;

  if(ts->tv_nsec >= 1000000000L){
    ts->tv_sec += 1;
    ts->tv_nsec -= 1000000000L;
  }
}

static void closeRecordFileLocked(void){
  if(recordFile){
    FrameHeader endHeader = {0};
    endHeader.frameType = FRAME_TYPE_END_OF_FILE;
    fwrite(&endHeader, sizeof(FrameHeader), 1, recordFile);
    fclose(recordFile);
    recordFile = NULL;
  }
  isRecordingData = 0;
  recordingStatus.active = 0;
}

static int ensurePlaybackBuffer(char** buffer, size_t* capacity, size_t requiredSize){
  if(requiredSize == 0){
    return 1;
  }

  if(*capacity >= requiredSize){
    return 1;
  }

  char* newBuffer = (char*)realloc(*buffer, requiredSize);
  if(!newBuffer){
    perror("Failed to allocate playback buffer");
    return 0;
  }

  *buffer = newBuffer;
  *capacity = requiredSize;
  return 1;
}

// 버퍼 재할당 함수
static int reallocateBuffers(int width, int height){
  int depthSize = width * height * sizeof(int16_t);
  int colorSize = width * height * 3 * sizeof(uint8_t);

  // 깊이 버퍼 할당/재할당
  if(depthBuffer == NULL || depthBufferSize < depthSize){
    char* newBuffer = (char*)realloc(depthBuffer, depthSize);
    if(newBuffer == NULL){
      perror("Failed to allocate depth buffer");
      return 0;
    }
    depthBuffer = newBuffer;
    depthBufferSize = depthSize;
  }

  // 색상 버퍼 할당/재할당
  if(colorBuffer == NULL || colorBufferSize < colorSize){
    char* newBuffer = (char*)realloc(colorBuffer, colorSize);
    if(newBuffer == NULL){
      perror("Failed to allocate color buffer");
      return 0;
    }
    colorBuffer = newBuffer;
    colorBufferSize = colorSize;
  }

  currentWidth = width;
  currentHeight = height;

  return 1;
}

// 데이터 저장 함수
static void saveFrameToFile(int frameId, uint32_t timestamp){
  if(!recordFile) return;

  pthread_mutex_lock(&recordMutex);

  // 프레임 헤더 생성
  FrameHeader header;
  header.frameId = frameId;
  header.timestamp = timestamp;
  header.frameType = FRAME_TYPE_DEPTH_COLOR;
  header.width = currentWidth;
  header.height = currentHeight;
  header.depthDataSize = currentWidth * currentHeight * sizeof(int16_t);
  header.colorDataSize = currentWidth * currentHeight * 3 * sizeof(uint8_t);
  header.reserved = 0;

  if(fwrite(&header, sizeof(header), 1, recordFile) == 1 &&
     fwrite(depthBuffer, header.depthDataSize, 1, recordFile) == 1 &&
     fwrite(colorBuffer, header.colorDataSize, 1, recordFile) == 1 &&
     fflush(recordFile) == 0){
    recordingStatus.frames = ++frameCounter;
    recordingStatus.bytes += sizeof(header) + header.depthDataSize + header.colorDataSize;
  }else{
    perror("Recording write failed");
    recordingStatus.writeError = 1;
    fclose(recordFile);
    recordFile = NULL;
    isRecordingData = 0;
    recordingStatus.active = 0;
  }

  pthread_mutex_unlock(&recordMutex);
}

// 로거 스레드 함수
static void* loggerThread(void* arg){
  (void)arg;
  printf("Logger thread started...\n");

  // 메세지 큐 속성 설정
  struct mq_attr attr;
  attr.mq_flags = 0;
  attr.mq_maxmsg = 10;
  attr.mq_msgsize = MAX_MSG_SIZE;
  attr.mq_curmsgs = 0;

  // 센서로부터 데이터를 받는 메세지 큐 열기
  mqFromSensor = mq_open(MQ_SENSOR_TO_LOGGER, O_RDONLY, 0644, &attr);
  if(mqFromSensor == (mqd_t) - 1){
    perror("mq_open from sensor");
    return NULL;
  }

  // 뷰어로 데이터를 보내는 메세지 큐 열기
  mqToViewer = mq_open(MQ_LOGGER_TO_VIEWER, O_WRONLY, 0644, &attr);
  if(mqToViewer == (mqd_t)-1){
    perror("mq_open to viewer");
    mq_close(mqFromSensor);
    return NULL;
  }

  // 제어 메세지 큐 열기
  mqControl = mq_open(MQ_CONTROL_QUEUE, O_RDONLY | O_NONBLOCK, 0644, &attr);
  if(mqControl == (mqd_t) - 1){
    perror("mq_open control");
    mq_close(mqFromSensor);
    mq_close(mqToViewer);
    return NULL;
  }

  // 메세지 버퍼 할당
  char* msgBuffer = (char*)malloc(MAX_MSG_SIZE);
  if(!msgBuffer){
    perror("malloc message buffer");
    mq_close(mqFromSensor);
    mq_close(mqToViewer);
    mq_close(mqControl);
    return NULL;
  }

  // 깊이 및 색상 데이터 버퍼 초기화
  depthBuffer = NULL;
  colorBuffer = NULL;
  depthBufferSize = 0;
  colorBufferSize = 0;

  // 메인 루프
  while(loggingIsRunning){
    // 제어 메세지 확인(non-blocking)
    ssize_t ctrlBytes = mq_receive(mqControl, msgBuffer, MAX_MSG_SIZE, NULL);
    // printf("ctrlBytes : %d\n", ctrlBytes);
    if(ctrlBytes > 0){
      // printf("testsets\n");
      MessageHeader* header = (MessageHeader*)msgBuffer;

      if(header->msgType == MSG_TYPE_CONTROL){
        printf("Ctrl Command Receive: %d\n", header->ctrlCommand);

        switch(header->ctrlCommand){
          case CTRL_CMD_START_RECORD:
            // 녹화 시작 명령 처리
            if(!isRecordingData){

              pthread_mutex_lock(&recordMutex);
              strncpy(currentRecordFilename, header->filename, sizeof(currentRecordFilename) -1);
              currentRecordFilename[sizeof(currentRecordFilename) - 1] = '\0';

              memset(&recordingStatus, 0, sizeof(recordingStatus));
              snprintf(recordingStatus.path, sizeof(recordingStatus.path), "%s", currentRecordFilename);
              recordFile = fopen(currentRecordFilename, "wb");
              if(recordFile){
                isRecordingData = 1;
                frameCounter = 0;
                recordingStatus.active = 1;
                char* absolutePath = realpath(currentRecordFilename, NULL);
                if(absolutePath){
                  snprintf(recordingStatus.path, sizeof(recordingStatus.path), "%s", absolutePath);
                  free(absolutePath);
                }
                printf("Started recording to: %s\n", currentRecordFilename);
              }else{
                recordingStatus.writeError = 1;
                perror("Failed to open record file");
              }

              pthread_mutex_unlock(&recordMutex);
            }
            break;

          case CTRL_CMD_STOP_RECORD:
            // 녹화 중지 명령 처리
            if(isRecordingData){
              pthread_mutex_lock(&recordMutex);
              closeRecordFileLocked();
              printf("Stopped recording. %u frames saved.\n", frameCounter);
              pthread_mutex_unlock(&recordMutex);
            }
            break;

          case CTRL_CMD_START_PLAYBACK:
            printf("Receive Command\n");

            // 플레이백 시작 명령 처리
            pthread_mutex_lock(&playbackMutex);
            if(playbackFile){fclose(playbackFile);playbackFile=NULL;}
            playbackGeneration++;
            playbackStatus.state=1;
            playbackStatus.frames=0;
            snprintf(playbackStatus.path,sizeof(playbackStatus.path),"%s",header->filename);
            strncpy(currentPlaybackFilename, header->filename, sizeof(currentPlaybackFilename) - 1);
            currentPlaybackFilename[sizeof(currentPlaybackFilename) - 1] = '\0';

            // 패스스루 비활성화
            isPassThroughEnabled = 0;

            // 플레이백 활성화
            isPlaybackActive = 1;

            // 프레임 카운터 초기화
            playbackFrameCounter = 0;

            printf("Starting playback from: %s\n", currentPlaybackFilename);
            pthread_mutex_unlock(&playbackMutex);
            break;

          case CTRL_CMD_STOP_PLAYBACK:
            // 플레이백 중지 명령 처리
            pthread_mutex_lock(&playbackMutex);

            isPlaybackActive = 0;
            if(playbackFile){
              fclose(playbackFile);
              playbackFile = NULL;
            }

            // 패스스루 다시 활성화
            isPassThroughEnabled = 0; // Freeze the completed map until explicitly returning live.
            playbackStatus.state = 3;

            printf("Playback stopped\n");
            pthread_mutex_unlock(&playbackMutex);
            break;
        }
      }
    }

    // 센서 데이터 수신. 타임아웃을 둬서 종료 플래그를 주기적으로 확인한다.
    struct timespec receiveTimeout;
    setTimeoutFromNow(&receiveTimeout, 100);
    ssize_t bytesRead = mq_timedreceive(mqFromSensor, msgBuffer, MAX_MSG_SIZE, NULL, &receiveTimeout);

    if(bytesRead > 0){
      MessageHeader* header = (MessageHeader*)msgBuffer;

      // 뷰어로 데이터 직접 전달 (패스스루)
      if(isPassThroughEnabled){
        struct timespec sendTimeout;
        setTimeoutFromNow(&sendTimeout, 100);
        if(mq_timedsend(mqToViewer, msgBuffer, bytesRead, 0, &sendTimeout) == -1){
          perror("mq_send to viewer");
        }
      }

      // 메세지 타입에 따라 처리
      switch(header->msgType){
        case MSG_TYPE_METADATA:
          // 메타데이터 메세지 처리
          if(!reallocateBuffers(header->width, header->height)){
            continue;
          }

          // 프레임 수신 상태 초기화
          receivedDepthFrame = 0;
          receivedColorFrame = 0;
          break;

        case MSG_TYPE_DEPTH_DATA:
          // 깊이 데이터 청크 처리
          if(header->chunkIndex == 0){

            // 새 프레임 시작
            receivedDepthFrame = 0;
          }

          if(depthBuffer){
            // 데이터 복사
            int offset = header->chunkIndex * (MAX_MSG_SIZE - sizeof(MessageHeader));
            int copySize = header->dataSize;

            // printf("width : %d, height : %d\n", header->width, header->height);

            if(offset + copySize <= depthBufferSize){
              memcpy(depthBuffer + offset, msgBuffer + sizeof(MessageHeader), copySize);
            }

            // 마지막 청크 확인
            if(header->chunkIndex == header->totalChunks - 1){
              receivedDepthFrame = 1;
            }
          }
          break;

        case MSG_TYPE_COLOR_DATA:
          // 색상 데이터 청크 처리
          if(header->chunkIndex == 0){
            // 새 프레임 시작
            receivedColorFrame = 0;
          }

          if(colorBuffer){
            // 데이터 복사
            int offset = header->chunkIndex * (MAX_MSG_SIZE - sizeof(MessageHeader));
            int copySize = header->dataSize;

            if(offset + copySize <= colorBufferSize){
              memcpy(colorBuffer + offset, msgBuffer + sizeof(MessageHeader), copySize);
            }

            // 마지막 청크 확인
            if(header->chunkIndex == header->totalChunks - 1){
               receivedColorFrame = 1;
            } 
          }
          break;
      }

      // 깊이 및 색상 데이터가 모두 수신되면 프레임 저장 (녹화 모드인 경우)
      if(receivedDepthFrame && receivedColorFrame && isRecordingData){
        saveFrameToFile(header->frameId, header->timestamp);
        receivedDepthFrame = 0;
        receivedColorFrame = 0;
      }
    }else if(errno != ETIMEDOUT && errno != EINTR){
      perror("mq_timedreceive from sensor");
    }
  }

  // 정리
  free(msgBuffer);

  if(depthBuffer){
    free(depthBuffer);
    depthBuffer = NULL;
  }

  if(colorBuffer){
    free(colorBuffer);
    colorBuffer = NULL;
  }

  // 메세지 큐 닫기
  if(mqFromSensor != (mqd_t)-1){
    mq_close(mqFromSensor);
  }

  if(mqToViewer != (mqd_t) - 1){
    mq_close(mqToViewer);
  }

  if(mqControl != (mqd_t) - 1){
    mq_close(mqControl);
  }

  // 녹화 중이었다면 파일 닫기
  pthread_mutex_lock(&recordMutex);

  closeRecordFileLocked();

  pthread_mutex_unlock(&recordMutex);

  printf("Logger thread termninated\n");

  return NULL;
}

// 파일에서 하나의 프레임 읽기
static int readFrameFromFile(FILE* file, FrameHeader* header,
                             char** depthData, size_t* depthCapacity,
                             char** colorData, size_t* colorCapacity){
  // 헤더 읽기
  size_t headerRead = fread(header, sizeof(FrameHeader), 1, file);
  if(headerRead != 1){
    if(feof(file)){
      printf("End of file reached\n");
    }else{
      perror("Error reading frame header");
    }

    return 0;
  }

  // 종료 마커 확인
  if(header->frameType == FRAME_TYPE_END_OF_FILE){
    printf("End of file marker found\n");
    return 2;
  }

  if(header->width == 0 || header->height == 0 ||
     header->frameType != FRAME_TYPE_DEPTH_COLOR ||
     (uint64_t)header->width*header->height > 4096ULL*4096 ||
     header->depthDataSize != (uint64_t)header->width*header->height*2 ||
     header->colorDataSize != (uint64_t)header->width*header->height*3){
    printf("Invalid frame header: width=%u, height=%u, depth=%u, color=%u\n",
           header->width, header->height, header->depthDataSize, header->colorDataSize);
    return 0;
  }

  if(!ensurePlaybackBuffer(depthData, depthCapacity, header->depthDataSize) ||
     !ensurePlaybackBuffer(colorData, colorCapacity, header->colorDataSize)){
    return 0;
  }

  // 깊이 데이터 읽기
  size_t depthRead = fread(*depthData, 1, header->depthDataSize, file);
  if(depthRead != header->depthDataSize){
    perror("Error reading depth data");
    return 0;
  }

  // 색상 데이터 읽기
  size_t colorRead = fread(*colorData, 1, header->colorDataSize, file);
  if(colorRead != header->colorDataSize){
    perror("Error reading color data");
    return 0;
  }

  return 1;
}

// 청크로 나누어 데이터 전송
static int sendDataInChunks(mqd_t mqdes, int msgType, int frameId, uint32_t timestamp, int width, int height, const char* data, int dataSize){

  char* msgBuffer = (char*)malloc(MAX_MSG_SIZE);
  if(!msgBuffer){
    perror("malloc chunk buffer");
    return 0;
  }

  int maxDataPerMsg = MAX_MSG_SIZE - sizeof(MessageHeader);
  int totalChunks = (dataSize + maxDataPerMsg - 1) / maxDataPerMsg;

  for(int i = 0; i < totalChunks; i++){
    MessageHeader* header = (MessageHeader*)msgBuffer;
    memset(header,0,sizeof(*header));
    header->ctrlCommand=playbackGeneration;
    header->msgType = msgType;
    header->width = width;
    header->height = height;
    header->chunkIndex = i;
    header->totalChunks = totalChunks;
    header->frameId = frameId;
    header->timestamp = timestamp;

    int offset = i * maxDataPerMsg;
    int chunkSize = (i == totalChunks - 1) ? (dataSize - offset) : maxDataPerMsg;

    header->dataSize = chunkSize;

    // 데이터 복사
    memcpy(msgBuffer + sizeof(MessageHeader), data + offset, chunkSize);

    // 전송
    struct timespec deadline;
    setTimeoutFromNow(&deadline, 5000);
    if(!loggingIsRunning || mq_timedsend(mqdes, msgBuffer, sizeof(MessageHeader)+chunkSize, 0, &deadline) == -1){

      perror("mq_send chunk");
      free(msgBuffer);
      return 0;
    }
  }

  free(msgBuffer);
  return 1;
}

// 메타 데이터 전송
static int sendMetadata(mqd_t mqdes, int frameId, uint32_t timestamp, int width, int height){
  MessageHeader header={0};
  header.ctrlCommand=playbackGeneration;
  header.msgType = MSG_TYPE_METADATA;
  header.width = width;
  header.height = height;
  header.chunkIndex = 0;
  header.totalChunks = 0;
  header.dataSize = 0;
  header.frameId = frameId;
  header.timestamp = timestamp;

  struct timespec deadline;
  setTimeoutFromNow(&deadline,5000);
  if(mq_timedsend(mqdes, (char*)&header, sizeof(MessageHeader), 0, &deadline) == -1){
    perror("mq_send metadata");
    return 0;
  }
  return 1;
}

// 재생 스레드 함수
static void* playbackThread(void* arg){
  (void)arg;
  printf("Playback thread started...\n");

  // 메세지 큐 속성 설정
  struct mq_attr attr;
  attr.mq_flags = 0;
  attr.mq_maxmsg = 10;
  attr.mq_msgsize = MAX_MSG_SIZE;
  attr.mq_curmsgs = 0;

  // 뷰어로 데이터를 보내는 메세지 큐 열기
  mqd_t mqPlaybackToViewer = mq_open(MQ_LOGGER_TO_VIEWER, O_WRONLY, 0644, &attr);
  if(mqPlaybackToViewer == (mqd_t) - 1){
    perror("mq_open playback to viewer");
    return NULL;
  }

  // 데이터 버퍼
  char* playbackDepthBuffer = NULL;
  char* playbackColorBuffer = NULL;
  size_t playbackDepthCapacity = 0;
  size_t playbackColorCapacity = 0;

  uint32_t previousTimestamp=0;
  int lastGeneration=-1;
  while(loggingIsRunning){
    if(!isPlaybackActive){usleep(10000);continue;}
    pthread_mutex_lock(&playbackMutex);
    if(!isPlaybackActive){pthread_mutex_unlock(&playbackMutex);continue;}
    if(!playbackFile){
      playbackFile=fopen(currentPlaybackFilename,"rb");
      if(!playbackFile){
        perror("Playback open failed"); playbackStatus.state=4;isPlaybackActive=0;
        pthread_mutex_unlock(&playbackMutex);continue;
      }
    }
    FrameHeader header;
    int readStatus=readFrameFromFile(playbackFile,&header,
      &playbackDepthBuffer,&playbackDepthCapacity,&playbackColorBuffer,&playbackColorCapacity);
    if(readStatus!=1){
      fclose(playbackFile);playbackFile=NULL;isPlaybackActive=0;
      playbackStatus.state=readStatus==2?2:4;
      printf("Playback %s: %u frames. Map retained.\n",readStatus==2?"completed":"error",playbackStatus.frames);
      pthread_mutex_unlock(&playbackMutex);continue;
    }
    int generation=playbackGeneration;
    uint32_t delay=lastGeneration==generation ? header.timestamp-previousTimestamp : 0;
    previousTimestamp=header.timestamp;lastGeneration=generation;
    pthread_mutex_unlock(&playbackMutex);
    // Timestamp pacing; cap long acquisition gaps at one second.
    if(delay>1000)delay=1000;
    while(delay && loggingIsRunning && isPlaybackActive){
      unsigned step=delay>10?10:delay;usleep(step*1000);delay-=step;
    }
    pthread_mutex_lock(&playbackMutex);
    if(!isPlaybackActive || generation!=playbackGeneration){pthread_mutex_unlock(&playbackMutex);continue;}
    int sent=sendMetadata(mqPlaybackToViewer,header.frameId,header.timestamp,header.width,header.height) &&
      sendDataInChunks(mqPlaybackToViewer,MSG_TYPE_DEPTH_DATA,header.frameId,header.timestamp,header.width,header.height,playbackDepthBuffer,header.depthDataSize) &&
      sendDataInChunks(mqPlaybackToViewer,MSG_TYPE_COLOR_DATA,header.frameId,header.timestamp,header.width,header.height,playbackColorBuffer,header.colorDataSize);
    if(sent){playbackStatus.frames=++playbackFrameCounter;}
    else {playbackStatus.state=4;isPlaybackActive=0;fclose(playbackFile);playbackFile=NULL;}
    pthread_mutex_unlock(&playbackMutex);
  }

  // 정리
  free(playbackDepthBuffer);
  free(playbackColorBuffer);

  mq_close(mqPlaybackToViewer);

  pthread_mutex_lock(&playbackMutex);
  if(playbackFile){
    fclose(playbackFile);
    playbackFile = NULL;

  }
  isPlaybackActive = 0;
  pthread_mutex_unlock(&playbackMutex);

  printf("Playback thread terminated\n");
  return NULL;
}

// 초기화 함수
void initLoggingModule(){
  printf("Initializing logging module...\n");

  // 메세지 큐 생성/열기 (다른 모듈에서 사용할 수 있도록)
  struct mq_attr attr;
  attr.mq_flags = 0;
  attr.mq_maxmsg = 10;
  attr.mq_msgsize = MAX_MSG_SIZE;
  attr.mq_curmsgs = 0;

  /*
   * Generate Message Queues.
   */
  
  // 센서->로거 큐 생성
  mqd_t mqTemp = mq_open(MQ_SENSOR_TO_LOGGER, O_CREAT | O_RDWR, 0644, &attr);
  if(mqTemp != (mqd_t) - 1){
    mq_close(mqTemp);
  }

  // 로거->뷰어 큐 생성
  mqTemp = mq_open(MQ_LOGGER_TO_VIEWER, O_CREAT | O_RDWR, 0644, &attr);
  if(mqTemp != (mqd_t) - 1){
    mq_close(mqTemp);
  }

  // 제어 큐 생성
  mqTemp = mq_open(MQ_CONTROL_QUEUE, O_CREAT | O_RDWR, 0644, &attr);
  if(mqTemp != (mqd_t) - 1){
    mq_close(mqTemp);
  }

  // 변수 초기화
  loggingIsRunning = 1;
  isRecordingData = 0;
  isPlaybackActive = 0;
  isPassThroughEnabled = 1;
  receivedDepthFrame = 0;
  receivedColorFrame = 0;
  recordFile = NULL;
  playbackFile = NULL;
  currentRecordFilename[0] = '\0';
  currentPlaybackFilename[0] = '\0';

  // 로거 스레드 시작
  pthread_create(&logger_thread_id, NULL, loggerThread, NULL);

  // 재생 스레드 시작
  pthread_create(&playback_thread_id, NULL, playbackThread, NULL);

  printf("Logging module initialized\n");
}

// 종료 함수
void stopLoggingModule(){
  printf("Stopping logging module...\n");

  // 먼저 로거 스레드가 제어 명령을 처리할 수 있게 녹화/재생 종료를 요청한다.
  stopRecording();
  stopPlayback();
  usleep(200000);

  // 스레드 종료 플레그 설정
  loggingIsRunning = 0;

  // 스레드 종료 대기
  printf("Waiting for logger thread to terminate...\n");
  pthread_join(logger_thread_id, NULL);

  printf("Waiting for playback thread to terminate...\n");
  pthread_join(playback_thread_id, NULL);

  printf("Logging module stopped\n");
}

// 녹화 시작 함수
int startRecording(const char* filename){
  if(isRecordingData){
    printf("Already recording to: %s\n", currentRecordFilename);
    return 0;
  }

  // 제어 명령 전송
  sendControlCommand(CTRL_CMD_START_RECORD, filename);
  return 1;
}

// 녹화 중지 함수
void stopRecording(){
  if(isRecordingData){
    sendControlCommand(CTRL_CMD_STOP_RECORD, NULL);
  }
}

// 재생 시작 함수
int startPlayback(const char* filename){
  if(isPlaybackActive){
    printf("Already playing: %s\n", currentPlaybackFilename);
    return 0;
  }

  // 제어 명령 전송
  sendControlCommand(CTRL_CMD_START_PLAYBACK, filename);
  return 1;
}

// 재생 중지 함수
void stopPlayback(){
  if(isPlaybackActive){
    sendControlCommand(CTRL_CMD_STOP_PLAYBACK, NULL);
  }
}

// 녹화 상태 확인
int isRecording(){
  return isRecordingData;
}

// 재생 상태 확인
int isPlayingBack(){
  return isPlaybackActive;
}

int isLoggingModuleRunning(void){
  return loggingIsRunning;
}

// 제어 명령 전송 함수
void sendControlCommand(int command, const char* filename){
  // 메세지 큐 열기
  mqd_t mqCtrl = mq_open(MQ_CONTROL_QUEUE, O_WRONLY, 0644, NULL);
  if(mqCtrl == (mqd_t) - 1){
    perror("mq_open control for command");
    return;
  }

  // 명령 메세지 생성
  MessageHeader msg;
  memset(&msg, 0, sizeof(MessageHeader));

  msg.msgType = MSG_TYPE_CONTROL;
  msg.ctrlCommand = command;

  if(filename){
    strncpy(msg.filename, filename, sizeof(msg.filename) - 1);
    msg.filename[sizeof(msg.filename) - 1] = '\0';
  }

  // 메세지 전송
  if(mq_send(mqCtrl, (char*)&msg, sizeof(MessageHeader), 0) == -1){
    perror("mq_send control command");
  }

  // 메세지 큐 닫기
  mq_close(mqCtrl);
}
