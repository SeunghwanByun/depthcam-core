#include "SLAM.h"
#include <iostream>
#include <thread>
#include <mutex>
#include <queue>
#include <memory>
#include <opencv2/opencv.hpp>

// ORB-SLAM3 헤더 포함
#include <System.h>
#include <Tracking.h>
#include <MapPoint.h>

// SLAM 시스템 변수
static std::shared_ptr<ORB_SLAM3::System> slam_system = nullptr;
static std::mutex slam_mutex;
static bool slam_running = false;
static std::thread processing_thread;

// 프레임 큐와 관련 변수
struct FrameData{
  cv::Mat depth;
  cv::Mat rgb;
  double timestamp;
};

static std::queue<FrameData> frame_queue;
static std::mutex queue_mutex;
static bool process_frames = true;

// 프레임 처리 스레드 함수
void processFramesThread(){
  std::cout << "SLAM processing thread started" << std::endl;

  while(process_frames){
    FrameData current_frame;
    bool has_frame = false;

    // 큐에서 프레임 가져오기
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      if(!frame_queue.empty()){
        current_frame = frame_queue.front();
        frame_queue.pop();
        has_frame = true;
      }
    }

    // 프레임 처리
    if(has_frame){
      std::lock_guard<std::mutex> lock(slam_mutex);
      if(slam_system && slam_running){
        // ORB-SLAM3 에 프레임 전달
        slam_system->TrackRGBD(current_frame.rgb, current_frame.depth, current_frame.timestamp);
      }
    }else{
        // 프레임이 없으면 잠시 대기
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  std::cout << "SLAM processing thread stopped" << std::endl;
}

extern "C" {

void initSlamModule(const char* config_file, const char* vocabulary_file){
  std::cout << "Initializing ORB-SLAM3 module..." << std::endl;

  if(slam_running){
    std::cout <<"SLAM module is already running" << std::endl;
    return;
  }

  try{
    // ORB-SLAM3 시스템 초기화
    std::lock_guard<std::mutex> lock(slam_mutex);
    slam_system = std::make_shared<ORB_SLAM3::System>(
        vocabulary_file,           // ORB 어휘 파일
        config_file,               // 설정 파일
        ORB_SLAM::System::RGBD,    // 센서 타입
        true                       // 시각화 활성화
    );

    // 프레임 처리 스레드 시작
    process_frames = true;
    processing_thread = std::thread(processFramesThread);

    slam_running = true;
    std::cout << "ORB-SLAM3 initialized successfully" << std::endl;
  }catch (const std::exception& e){
    std::cerr << "Failed to initialize ORB-SLAM3: " << e.what() << std::endl;
    slam_system = nullptr;
  }
}

void stopSlamModule() {
  std::cout << "Stopping ORB-SLAM3 module..." << std::endl;

  // 프레임 처리 스레드 중지
  process_frames = false;
  if(processing_thread.joinable()){
    processing_thread.join();
  }

  // ORB-SLAM3 시스템 종료
  {
    std::lock_guard<std::mutex> lock(slam_mutex);
    if(slam_system){
      slam_system->Shutdown();
      slam_system = nullptr;
    }
  }

  // 큐 비우기
  {
    std::lock_guard<std::mutex> lock(queue_mutex);
    std::queue<FrameData> empty;
    std::swap(frame_queue, empty);
  }

  slam_running = false;
  std::cout << "ORB-SLAM3 module stopped" << std::endl;
}

int processSlamFrame(const int16_t* depth_data, const uint8_t* color_data, int width, int height, uint32_t timestamp){
  if(!slam_running || !slam_system){
    return 0;
  }

  try{
    // 깊이 데이터를 OpenCV 행렬로 변환
    cv::Mat depth_mat(height, width, CV_16UC1);
    memcpy(depth_mat.data, depth_data, width * height * sizeof(int16_t));

    // 색상 데이터를 OpenCV 행렬로 변환 (RGB -> BGF 변환)
    cv::Mat rgb_mat(height, width, CV_8UC3);
    for(int y = 0; y < height; ++y){
      for(int x = 0; x < width; ++x){
        int src_idx = (y * width + x) * 3;
        int dst_idx = y * rgb_mat.step + x * 3;

        // RGB -> BGR 순서 변환
        rgb_mat.data[dst_idx + 2] = color_data[src_idx];     // R -> B
        rbg_mat.data[dst_idx + 1] = color_data[src_idx + 1]; // G -> G
        rgb_mat.data[dst_idx] = color_data[src_idx +2];      // B -> R
      }
    }

    // 타임스템프를 초 단위로 변환
    double timestamp_sec = timestamp; 

    // 깊이 이미지를 32비트 부동소수점으로 변환 (mm -> m 단위 변환)
    cv::Mat depth_float;
    depth_map.convertTo(depth_float, CV_32F, 1.0/1000.0);

    // 프레임 큐에 추가
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      frame_queue.push({depth_float, rgb_mat, timestamp_sec});

      // 큐가 너무 커지지 않도록 오래된 프레임 제거
      if(frame_queue.size() > 10){
        std::cout << "Warning: Frame queue is getting large (" << frame_queue.size() << ")" << std::endl;
        while(frame_queue.size() > 5){
          frame_queue.pop();
        }
      }
    }

    return 1;
  } catch(const std::exception& e){
    std::cerr << "Error processing frame: " << e.what() << std::endl;
  }
}

int saveSlamMap(const char* map_file){
  if(!slam_running || !slam_system){
    std::cerr << "SLAM system is not running" << std::endl;
    return 0;
  }

  try{
    std::lock_guard<std::mutex> lock(slam_mutex);

    // 맵 저장 (ORB-SLAM3 형식)
    slam_system->SaveTrajectoryTUM(std::string(map_file) + "_trajectory.txt");
    slam_system->SaveKeyFrameTrajectoryTUM(std::string(map_file) + "_keyframes.txt");

    // 포인트 클라우드 저장 (추가 구현 필요)
    // ORB-SLAM3의 맵 포인트를 PCL 포인트 클라우드로 변환하여 저장
    std::cout << "Map saved to " << map_file << std::endl;
    return 1;
  }catch(const std::exception& e){
    std::cerr << "Error saving map: " << e.what() << std::endl;
    return 0;
  }
}

int isSlamModuleRunning(){
  return slam_running ? 1 : 0;
}

int getSlamMapPoints(){
  if(!slam_running || !slam_system){
    return 0;
  }

  std::lock_guard<std::mutex> lock(slam_mutex);
  // ORB-SLAM3의 맵 포인트 수 얻기 (이 부분은 ORB-SLAM3 API 에 따라 달라질 수 있음)
  // 참고로 이 부분은 ORB-SLAM3의 실제 API 에 따라 수정 필요
  const auto map = slam_system->GetMap();
  if(!map){
    return 0;
  }

  return map->GetAllMapPoints().size();
}

void resetSlam(){
  if(!slam_running || !slam_system){
    return;
  }

  std::lock_guard<std::mutex> lock(slam_mutex);
  slam_system->Reset();
  std::cout << "SLAM system reset" << std::endl;
}

} // extern "C"
