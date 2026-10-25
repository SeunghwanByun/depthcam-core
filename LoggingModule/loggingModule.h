#ifndef LOGGING_MODULE_H
#define LOGGING_MODULE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int active;
  int writeError;
  uint32_t frames;
  uint64_t bytes;
  char path[1024];
} RecordingStatus;

// Thread-safe snapshot; frame count advances only after successful write + fflush.
void getRecordingStatus(RecordingStatus* status);

typedef struct {
  int state; // 0 live, 1 playing, 2 complete, 3 stopped, 4 error
  uint32_t frames;
  char path[256];
} PlaybackStatus;
void getPlaybackStatus(PlaybackStatus* status);

// Initialize Logging Module
void initLoggingModule();

// Stop Logging Module
void stopLoggingModule();

// Start Record with file name
int startRecording(const char* filename);

// Stop Recording
void stopRecording();

// Start Playback of recording file
int startPlayback(const char* filename);

// Stop Playback
void stopPlayback();

// Check Recording Status 
int isRecording();

// Check Play Status
int isPlayingBack();

// Check Logging Module Execution Status 
int isLoggingModuleRunning();

void sendControlCommand(int command, const char* filename);

#ifdef __cplusplus
}
#endif

#endif // LOGGING_MODULE_H
