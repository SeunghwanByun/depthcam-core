#ifndef SLAM_H
#define SLAM_H

#ifdef __cplusplus
extern "C" {
#endif

// SLAM 모듈 초기화
// config_file: ORB_SLAM3 설정 파일 경로
// vocabulary_file: ORB 어휘 파일 경로
void initSlamModule(const char* config_file, const char* vocabulary_file);

// SLAM 모듈 종료
void stopSlamModule();

// 프레임 처리 함수
// depth_data: 16비트 깊이 데이터
// color_data: RGB 색상 데이터 (RGB 순서)
// width, height: 이미지 크기
// timestamp: 타임스템프 (밀리초)
// 반환값: 성공 시 1, 실패 시 0
int processSlamFrame(const int16_t* depth_data, const uint8_t* color_data, int width, int height, uint32_t timestamp);

// 생성된 맵 저장
// map_file: 맵 저장 파일 경로
// 반환값: 성공 시 1, 실패 시 0
int saveSlamMap(const char* map_file);

// SLAM 상태 확인
// 반환값: 동작 중이면 1, 아니면 0
int isSlamModuleRunning();

// 현재 맵 정보 얻기
// 반환값: 맵 포인트 수
int getSlamMapPoints();

// 맵핑 리셋
void resetSlam();

#ifdef __cplusplus
}
#endif


#endif // SLAM_H
