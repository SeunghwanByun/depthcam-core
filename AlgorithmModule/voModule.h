#ifndef VO_MODULE_H
#define VO_MODULE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize VO with camera intrinsics and start worker thread.
void voInit(float fx, float fy, float cx, float cy);

// Stop worker thread and free resources.
void voStop(void);

int voIsRunning(void);

// Feed a new RGB-D frame (non-blocking; drops frame if queue is full).
// depth16 : int16 depth values in millimetres, row-major
// rgb     : packed uint8 RGB (R,G,B order), row-major
void voProcessFrame(const int16_t* depth16, const uint8_t* rgb,
                    int width, int height, uint32_t timestamp_ms);

// Copy at most max_points accumulated map points into dst.
// Each point is 6 floats: [x, y, z, r, g, b]
// Coordinates are pre-negated to match the viewer's OpenGL convention.
// Returns number of points written.
int voGetMapPoints(float* dst, int max_points);

// Fast query — no copy.
int voGetMapPointCount(void);

// Tracking status: 0 = idle, 1 = tracking OK, 2 = tracking lost
int voGetTrackingStatus(void);

// Number of frames processed so far.
int voGetFrameCount(void);

// Wait until all submitted frames finish. Call from a single producer.
void voWaitIdle(void);
// Save current map as ASCII PLY; returns 1 on success.
int voSaveMap(const char* filename);

// Reset accumulated map and restart tracking from scratch.
void voReset(void);

#ifdef __cplusplus
}
#endif

#endif // VO_MODULE_H
