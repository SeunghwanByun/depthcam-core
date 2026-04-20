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
#include <sys/time.h>

// 스레드 제어 변수
static int loggingIsRunning = 1;
static int isRecordingData = 0;
static int isPlaybackActive = 0;
static int isPassThroughEnabled = 1; // 기본적으로 패스스루 활성화
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

// 프레임 카운터
static uint32_t frameCounter = 0;

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

// 현재 시간 가져오기 (밀리초)
static uint32_t getCurrentTimeMS(){
  struct timeval tv;

  gettimeofday(&tv, NULL);
  return (uint32_t)(tv.tv_sec * 1000 + tv.tv_usec / 1000);
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

  // 헤더 쓰기
  fwrite(&header, sizeof(FrameHeader), 1, recordFile);

  // 깊이 데이터 쓰기
  fwrite(depthBuffer, header.depthDataSize, 1, recordFile);

  // 색상 데이터 쓰기
  fwrite(colorBuffer, header.colorDataSize, 1, recordFile);

  // 파일 버퍼 플러시
  fflush(recordFile);

  pthread_mutex_unlock(&recordMutex);
}

// 로거 스레드 함수
static void* loggerThread(void* arg){
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

              recordFile = fopen(currentRecordFilename, "wb");
              if(recordFile){
                isRecordingData = 1;
                frameCounter = 0;
                printf("Started recording to: %s\n", currentRecordFilename);
              }else{
                perror("Failed to open record file");
              }

              pthread_mutex_unlock(&recordMutex);
            }
            break;

          case CTRL_CMD_STOP_RECORD:
            // 녹화 중지 명령 처리
            if(isRecordingData){
              pthread_mutex_lock(&recordMutex);
              isRecordingData = 0;
              if(recordFile){
                // 종료 마커 쓰기
                FrameHeader endHeader = {0};
                endHeader.frameType = FRAME_TYPE_END_OF_FILE;
                fwrite(&endHeader, sizeof(FrameHeader), 1, recordFile);

                fclose(recordFile);
                recordFile = NULL;
                printf("Stopped recording. %u frames saved.\n", frameCounter);
              }
              pthread_mutex_unlock(&recordMutex);
            }
            break;

          case CTRL_CMD_START_PLAYBACK:
            printf("Receive Command\n");

            // 플레이백 시작 명령 처리
            pthread_mutex_lock(&playbackMutex);
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
            isPassThroughEnabled = 1;

            printf("Playback stopped\n");
            pthread_mutex_unlock(&playbackMutex);
            break;
        }
      }
    }

    // 센서 데이터 수신 (blocking)
    ssize_t bytesRead = mq_receive(mqFromSensor, msgBuffer, MAX_MSG_SIZE, NULL);

    if(bytesRead > 0){
      MessageHeader* header = (MessageHeader*)msgBuffer;

      // 뷰어로 데이터 직접 전달 (패스스루)
      if(isPassThroughEnabled){
        if(mq_send(mqToViewer, msgBuffer, bytesRead, 0) == -1){
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
        frameCounter++;
      }
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

  if(recordFile){
    fclose(recordFile);
    recordFile = NULL;
    isRecordingData = 0;
  }

  pthread_mutex_unlock(&recordMutex);

  printf("Logger thread termninated\n");

  return NULL;
}

// 파일에서 하나의 프레임 읽기
static int readFrameFromFile(FILE* file, FrameHeader* header, char* depthData, char* colorData, int maxSize){
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
    return 0;
  }

  // 데이터 크기 유효성 검사
  if(header->depthDataSize > maxSize || header->colorDataSize > maxSize){
    printf("Frame data too large: depth=%u, color=%u, max=%d\n", header->depthDataSize, header->colorDataSize, maxSize);
    return 0;
  }

  // 깊이 데이터 읽기
  size_t depthRead = fread(depthData, 1, header->depthDataSize, file);
  if(depthRead != header->depthDataSize){
    perror("Error reading depth data");
    return 0;
  }

  // 색상 데이터 읽기
  size_t colorRead = fread(colorData, 1, header->colorDataSize, file);
  if(colorRead != header->colorDataSize){
    perror("Error reading color data");
    return 0;
  }

  return 1;
}

// 청크로 나누어 데이터 전송
static void sendDataInChunks(mqd_t mqdes, int msgType, int frameId, uint32_t timestamp, int width, int height, const char* data, int dataSize){

  char* msgBuffer = (char*)malloc(MAX_MSG_SIZE);
  if(!msgBuffer){
    perror("malloc chunk buffer");
    return;
  }

  int maxDataPerMsg = MAX_MSG_SIZE - sizeof(MessageHeader);
  int totalChunks = (dataSize + maxDataPerMsg - 1) / maxDataPerMsg;

  for(int i = 0; i < totalChunks; i++){
    MessageHeader* header = (MessageHeader*)msgBuffer;
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
    if(mq_send(mqdes, msgBuffer, sizeof(MessageHeader) + chunkSize, 0) == -1){

      perror("mq_send chunk");
      break;
    }
  }

  free(msgBuffer);
}

// 메타 데이터 전송
static void sendMetadata(mqd_t mqdes, int frameId, uint32_t timestamp, int width, int height){
  MessageHeader header;
  header.msgType = MSG_TYPE_METADATA;
  header.width = width;
  header.height = height;
  header.chunkIndex = 0;
  header.totalChunks = 0;
  header.dataSize = 0;
  header.frameId = frameId;
  header.timestamp = timestamp;

  if(mq_send(mqdes, (char*)&header, sizeof(MessageHeader), 0) == -1){
    perror("mq_send metadata");
  }
}

// 재생 스레드 함수
static void* playbackThread(void* arg){
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

  // 제어 메세지를 위한 임시 버퍼
  char controlBuffer[MAX_MSG_SIZE];

  // 데이터 버퍼
  char* playbackDepthBuffer = NULL;
  char* playbackColorBuffer = NULL;
  int maxBufferSize = 1024 * 1024; // 1MB 초기 할당

  
  playbackDepthBuffer = (char*)malloc(maxBufferSize);
  playbackColorBuffer = (char*)malloc(maxBufferSize);

  if(!playbackDepthBuffer || !playbackColorBuffer){
    perror("malloc playback buffers");
    free(playbackDepthBuffer);
    free(playbackColorBuffer);
    mq_close(mqPlaybackToViewer);
    return NULL;
  }

  while(loggingIsRunning){
    // 플레이백 모드가 아니면 대기
    if(!isPlaybackActive){
      // printf("Playback is Not Activated!\n");
      usleep(100000); // 100ms 대기
      continue;
    }

    pthread_mutex_lock(&playbackMutex);

    // 파일 열기
    if(!playbackFile){
      playbackFile = fopen(currentPlaybackFilename, "rb");
      if(!playbackFile){
        perror("Failed to open playback file");
        isPlaybackActive = 0;
        pthread_mutex_unlock(&playbackMutex);
        continue;
      }

      printf("Started playback from: %s\n", currentPlaybackFilename);
    }

    // 프레임 읽기
    FrameHeader header;
    if(!readFrameFromFile(playbackFile, &header, playbackDepthBuffer, playbackColorBuffer, maxBufferSize)){
      // 파일 끝이거나 오류 발생
      fclose(playbackFile);
      playbackFile = NULL;
      isPlaybackActive = 0;
      printf("Playback completed or error occurred. Total frames played: %u\n", playbackFrameCounter);
      pthread_mutex_unlock(&playbackMutex);
      continue;
    }

    // 프레임 카운터 증가 및 출력
    playbackFrameCounter++;
    printf("Playing frame #%u (ID: %u, timestamp: %u)\n", playbackFrameCounter, header.frameId, header.timestamp);

    pthread_mutex_unlock(&playbackMutex);

    // 메타데이터 전송
    sendMetadata(mqPlaybackToViewer, header.frameId, header.timestamp, header.width, header.height);

    // 깊이 데이터 전송
    sendDataInChunks(mqPlaybackToViewer, MSG_TYPE_DEPTH_DATA, header.frameId, header.timestamp, header.width, header.height, playbackDepthBuffer, header.depthDataSize);

    // 색상 데이터 전송
    sendDataInChunks(mqPlaybackToViewer, MSG_TYPE_COLOR_DATA, header.frameId, header.timestamp, header.width, header.height, playbackColorBuffer, header.colorDataSize);

    // 프레임 레이트 조절 (30fps, 약 33ms)
    usleep(33333);
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
  mqd_t mqTemp = mq_open(MQ_SENSOR_TO_LOGGER, O_CREAT, 0644, &attr);
  if(mqTemp != (mqd_t) - 1){
    mq_close(mqTemp);
  }

  // 로거->뷰어 큐 생성
  mqTemp = mq_open(MQ_LOGGER_TO_VIEWER, O_CREAT, 0644, &attr);
  if(mqTemp != (mqd_t) - 1){

    mq_close(mqTemp);
  }

  // 제어 큐 생성
  mqTemp = mq_open(MQ_CONTROL_QUEUE, O_CREAT, 0644, &attr);
  if(mqTemp != (mqd_t) - 1){
    mq_close(mqTemp);
  }

  // 변수 초기화
  loggingIsRunning = 1;
  isRecordingData = 0;
  isPlaybackActive = 0;
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

  // 스레드 종료 플레그 설정
  loggingIsRunning = 0;

  // 녹화 중지
  stopRecording();

  // 재생 중지
  stopPlayback();

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
  if(!isRecordingData){
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
