#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <mqueue.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <pthread.h>
#include <sys/time.h>

#include "frameDefinitions.h"
#include "SensorModule/sensorModule.h"
#include "ViewerModule/viewerModule.h"
#include "LoggingModule/loggingModule.h"

#define SENSOR_READY_TIMEOUT_MS 5000

// 종료 시그널 핸들링
volatile int keepRunning = 1;

// 사용자 명령 처리를 위한 플래그
static int isMenuMode = 0;
static int offlinePlayback = 0;

// 종료 과정 상태 추적
static int shutdownRequested = 0;
static pthread_mutex_t shutdownMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t shutdownCond = PTHREAD_COND_INITIALIZER;

// 타임아웃 설정용 타이머
static timer_t shutdownTimerId;

// 종료 핸들러
void intHandler(int dummy){
  keepRunning = 0;
  printf("\nShutdown signal received. Closing application...\n");
  fflush(stdout);
}

// 뷰어에서 호출될 종료 콜백
void exitHandler(void){
  keepRunning = 0;
  printf("Received shutdown signal from viewer. Closing application...\n");
  fflush(stdout);
}

// 타이머 만료 시 호출될 함수
void timer_handler(union sigval sv){
  printf("\nShutdown timeout occurred! Forcing exit...\n");
  fflush(stdout);

  // 정리 없이 강제 종료
  exit(1);
}

// 메인 메뉴 표시
void displayMenu(){
  printf("\n");
  printf("==== 3D Point Cloud Viewer & Logger ====\n");
  printf("1. Start Recording\n");
  printf("2. Stop Recording\n");
  printf("3. Start Playback\n");
  printf("4. Stop Playback\n");
  printf("5. Toggle Menu Mode\n");
  printf("0. Exit\n");
  printf("Enter your choice: ");
  fflush(stdout);
}

// 사용자 입력 처리
void processUserInput(){
  int choice;
  char filename[256];
  char buffer[512];
  
  if(fgets(buffer, sizeof(buffer), stdin) == NULL){
    return;
  }

  // 메뉴 모드에서는 숫자 입력 처리
  if(isMenuMode){
    choice = atoi(buffer);

    switch(choice){
      case 0:
        // 종료
        keepRunning = 0;
        break;

      case 1:
        // 녹화 시작
        printf("Enter filename to save: ");
        fflush(stdout);
        if(fgets(buffer, sizeof(buffer), stdin)){
          // 개행 문자 제거
          buffer[strcspn(buffer, "\n")] = 0;

          // 확장자 확인 및 추가
          if(strstr(buffer, ".bin") == NULL){
            strncat(buffer, ".bin", sizeof(buffer) - strlen(buffer) - 1);
          }

          // 녹화 시작 명령 전송
          sendControlCommand(CTRL_CMD_START_RECORD, buffer);
          printf("Starting recording to %s\n", buffer);
        }
        break;

      case 2:
        // 녹화 중지
        sendControlCommand(CTRL_CMD_STOP_RECORD, NULL);
        printf("Stopping recording\n");
        break;

      case 3:
        // 재생 시작
        printf("Enter filename to play: ");
        fflush(stdout);
        if(fgets(buffer, sizeof(buffer), stdin)){
          // 개행 문자 제거
          buffer[strcspn(buffer, "\n")] = 0;

          // 확장자 확인 및 추가
          if(strstr(buffer, ".bin") == NULL){
            strncat(buffer, ".bin", sizeof(buffer) - strlen(buffer) - 1);
          }

          // 재생 시작 명령 전송
          sendControlCommand(CTRL_CMD_START_PLAYBACK, buffer);
          printf("Starting playback from %s\n", buffer);
        }
        break;

      case 4:
        // 재생 중지
        sendControlCommand(CTRL_CMD_STOP_PLAYBACK, NULL);
        printf("Stopping playback\n");
        break;

      case 5:
        // 메뉴 모드 토글
        isMenuMode = 0;
        printf("Menu mode disabled. Press Enter for menu.\n");
        break;

      default:
        printf("Invalid choice!\n");
        break;
    }

    // if(isMenuMode && keepRunning){
    //   displayMenu();
    // }
  }else{
    // 메뉴 모드가 아닌 경우, 엔터 키 입력하면 메뉴 활성화
    isMenuMode = 1;
    displayMenu();
  }
}

// 타이머 설정
void setupShutdownTimer(int seconds){
  struct sigevent sev;
  struct itimerspec its;

  // 타이머 이벤트 설정
  memset(&sev, 0, sizeof(struct sigevent));
  sev.sigev_notify = SIGEV_THREAD;
  sev.sigev_notify_function = timer_handler;

  // 타이머 생성
  if(timer_create(CLOCK_REALTIME, &sev, &shutdownTimerId) == -1){
    perror("timer_create");
    return;
  }

  // 타이머 간격 설정
  its.it_value.tv_sec = seconds;
  its.it_value.tv_nsec = 0;
  its.it_interval.tv_sec = 0;
  its.it_interval.tv_nsec = 0;

  // 타이머 시작
  if(timer_settime(shutdownTimerId, 0, &its, NULL) == -1){
    perror("timer_settime");
  }
}

// 모든 메세지 큐 정리 - 안전한 버전
void cleanupMessageQueues(){
  printf("Clean up all Message Queues\n");
  
  // 각 메시지 큐를 안전하게 제거
  // 이미 제거된 큐에 대한 unlink는 에러를 반환하지만 무시함
  if(mq_unlink(MQ_SENSOR_TO_LOGGER) == -1) {
    if(errno != ENOENT) { // ENOENT는 "존재하지 않음" 에러
      perror("mq_unlink MQ_SENSOR_TO_LOGGER failed");
    }
  }
  
  if(mq_unlink(MQ_LOGGER_TO_VIEWER) == -1) {
    if(errno != ENOENT) {
      perror("mq_unlink MQ_LOGGER_TO_VIEWER failed");
    }
  }
  
  if(mq_unlink(MQ_CONTROL_QUEUE) == -1) {
    if(errno != ENOENT) {
      perror("mq_unlink MQ_CONTROL_QUEUE failed");
    }
  }
  
  // 짧은 대기로 메시지 큐가 완전히 제거되도록 함
  usleep(100000); // 0.1초
}

// 안전한 종료 시퀀스
void safeShutdown(){
  printf("Initializing safe shutdown sequence...\n");

  // 종료 타이머 설정 (10초 후 강제 종료)
  setupShutdownTimer(10);

  // 먼저 녹화/재생 중지 요청
  sendControlCommand(CTRL_CMD_STOP_RECORD, NULL);
  sendControlCommand(CTRL_CMD_STOP_PLAYBACK, NULL);

  // 약간의 지연을 두어 명령이 처리되도록 함
  usleep(500000); // 0.5초

  // 모듈 종료 요청 (역순)
  stopViewerModule();

  stopLoggingModule();  // 원래 코드에서는 잘못된 stopSensorModule()을 호출함

  if(!offlinePlayback) stopSensorModule();

  // 타이머 취소
  timer_delete(shutdownTimerId);

  // 메세지 큐 정리
  cleanupMessageQueues();

  printf("Shutdown completed successfully\n");
}

int main(int argc, char** argv)
{
  const char* playbackPath=NULL;
  if(argc==3 && strcmp(argv[1],"--playback")==0){
    playbackPath=argv[2];offlinePlayback=1;
    if(strlen(playbackPath)>=256 || access(playbackPath,R_OK)!=0){
      fprintf(stderr,"Playback path must be readable and shorter than 256 bytes\n");return 1;
    }
  }else if(argc!=1){fprintf(stderr,"Usage: %s [--playback recording.bin]\n",argv[0]);return 2;}
  printf("Starting Youth\n");
  
  // 종료 시그널 핸들러 등록
  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = intHandler;
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);

  // 기존 메세지 큐 정리 (이전 실행에서 남은 것들)
  cleanupMessageQueues();

  /* Logging system will be here. */
  // 로깅 모듈 초기화 (먼저 초기화하여 메세지 큐 생성)
  initLoggingModule();

  // 약간의 지연 후 다른 모듈 초기화
  usleep(200000); // 0.1초

  /* Connect sensor and get sensor data. */
  if(!offlinePlayback){
  initSensorModule();

  // 센서가 실제로 프레임을 받기 시작할 때까지 대기
  int waitedMs = 0;
  while(waitedMs < SENSOR_READY_TIMEOUT_MS && isSensorModuleRunning() && !isSensorModuleReady()) {
    usleep(100000);
    waitedMs += 100;
  }

  if(!isSensorModuleRunning() || !isSensorModuleReady()) {
    printf("Sensor module failed to become ready: %s\n", getSensorModuleStatus());
    if(!offlinePlayback) stopSensorModule();
    stopLoggingModule();
    cleanupMessageQueues();
    return 1;
  }

  }

  if(playbackPath) setViewerPlaybackFile(playbackPath);

  /* Algorithm module. */
  // pthread_t algorithm_thread_id;
  // pthread_create(&algorithm_thread_id, NULL, algorithmModule, NULL);

  /* Generate 3D Viewer. */
  // 약간의 지연 후 뷰어 모듈 초기화 (센서가 준비되도록)
  usleep(200000);

  initViewerModule();
  setViewerStatusText(offlinePlayback?"Offline recording playback":getSensorModuleStatus());

  // 종료 콜백 설정
  setExitCallback(exitHandler);

  // 모듈 초기화 확인
  usleep(500000); // 0.5초
  if(!isViewerModuleRunning()) {
    printf("Viewer module failed to initialize! Exiting...\n");
    if(!offlinePlayback) stopSensorModule();
    stopLoggingModule();
    return 1;
  }

  if(playbackPath) startPlayback(playbackPath);

  printf("All modules initialized. Press Enter to show menu.\n");

  // 사용자 입력을 비동기적으로 처리하기 위한 준비
  struct timeval tv;
  fd_set readfds;
  int stdin_fd = fileno(stdin);

  // 프로그램이 종료될 때까지 대기
  // printf("Application running. Press Ctrl+C or close the window to exit.\n");
  while(keepRunning){
    setViewerStatusText(offlinePlayback?"Offline recording playback":getSensorModuleStatus());

    // 사용자 입력 처리 설정
    FD_ZERO(&readfds);
    FD_SET(stdin_fd, &readfds);

    // 타임아웃 설정 (100ms)
    tv.tv_sec = 0;
    tv.tv_usec = 100000;

    // 입력 확인
    int ret = select(stdin_fd + 1, &readfds, NULL, NULL, &tv);

    if(ret > 0 && FD_ISSET(stdin_fd, &readfds)){
      // 사용자 입력 처리
      processUserInput();
    }

    // 모듈 상태 확인
    if(!isLoggingModuleRunning()){
      printf("Logging module stopped unexpectedly!\n");
      keepRunning = 0;
    }

    if(!offlinePlayback && !isSensorModuleRunning()){
      printf("Sensor module stopped unexpectedly! %s\n", getSensorModuleStatus());
      keepRunning = 0;
    }

    if(!isViewerModuleRunning()){
      printf("Viewer module stopped unexpectedly!\n");
      keepRunning = 0;
    }
  }

  printf("Shutting down application...\n");
  
  safeShutdown();
  
  printf("Application terminated successfully\n");
    
  return 0;
}
