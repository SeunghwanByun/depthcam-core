#ifndef FRAME_DEFINITIONS_H
#define FRAME_DEFINITIONS_H

#include <stdint.h>

// 프레임 타입 정의
#define FRAME_TYPE_DEPTH_COLOR 1
#define FRAME_TYPE_END_OF_FILE 0xFF

// 프레임 헤더 구조체
typedef struct{
  uint32_t frameId;   // 프레임 ID 
  uint32_t timestamp; // 타임 스템프 (ms)
  uint16_t frameType; // 프레임 타입
  uint16_t width;     // 이미지 너비 
  uint16_t height;    // 이미지 높이
  uint32_t depthDataSize; // 깊이 데이터 크기
  uint32_t colorDataSize; // 색상 데이터 크기
  uint32_t reserved;      // 추가 정보를 위한 예약 필드
} FrameHeader;

// 센서 데이터 메세지 구조체 (메세지 공유)
typedef struct{
  int width;
  int height;
  int frameId;
  uint32_t timestamp;
  int dataSize;
  // 실제 데이터는 메세지 큐에 별도로 전송됨
} SensorDataMsg;

// 메세지 타입 정의
#define MSG_TYPE_METADATA 1 
#define MSG_TYPE_DEPTH_DATA 2
#define MSG_TYPE_COLOR_DATA 3
#define MSG_TYPE_CONTROL 4

// 제어 메세지 명령어
#define CTRL_CMD_START_RECORD 1
#define CTRL_CMD_STOP_RECORD 2
#define CTRL_CMD_START_PLAYBACK 3
#define CTRL_CMD_STOP_PLAYBACK 4

// 메세지 헤더 구조체
typedef struct{
  int msgType; // 메세지 타입 
  int width;  // 이미지 너비
  int height; // 이미지 높이
  int chunkIndex; // 청크 인덱스
  int totalChunks;  // 전체 청크 수
  int dataSize; // 이 메세지의 데이터 크기
  int frameId;  // 프레임 식별자
  uint32_t timestamp; // 타임스템프
  int ctrlCommand;  // 제어 명령 (MSG_TYPE_CONTROL 에서 사용)
  char filename[256]; // 파일명 (제어 명령에서 사용)
}MessageHeader;

// 메세지 큐 이름 정의
#define MQ_SENSOR_TO_LOGGER "/sensor_logger_queue"
#define MQ_LOGGER_TO_VIEWER "/logger_viewer_queue"
#define MQ_CONTROL_QUEUE "/control_queue"

// 메세지 최대 크기
#define MAX_MSG_SIZE 8192

#endif // FRAME_DEFINITIONS_H
