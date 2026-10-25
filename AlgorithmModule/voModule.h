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

// Thread-safe last accepted camera pose and matching RGB image. Column-major
// matrix maps local viewer coordinates to the same negated world as map points.
// Pose/image remain at the last accepted frame if tracking is lost.
typedef struct {
    float cameraToWorld[16];
    float fx, fy, cx, cy;
    int width, height, valid;
    unsigned int revision;
} VoCameraState;
int voGetCameraState(VoCameraState* state, uint8_t* rgb, int rgbCapacity,
                     float* trajectory, int maxVertices);

// Per-frame feature pipeline of the most recently processed frame, for debug
// overlays. Pixel coordinates are in the VO input image. prevX/prevY is the
// matched feature in the reference (last accepted) frame, valid for state >= 1.
enum { VO_FEAT_UNMATCHED = 0, VO_FEAT_NO_DEPTH = 1, VO_FEAT_DEPTH = 2, VO_FEAT_INLIER = 3 };
enum { VO_REASON_OK = 0, VO_REASON_INIT, VO_REASON_NO_FEATURES, VO_REASON_FEW_MATCHES,
       VO_REASON_FEW_DEPTH, VO_REASON_PNP_FAILED, VO_REASON_IMPLAUSIBLE };
typedef struct { float x, y, prevX, prevY; int state; } VoDebugFeature;
typedef struct {
    int width, height;
    int features, matches, withDepth, inliers;
    int reason;           // VO_REASON_*
    unsigned int frame;   // increments for every processed frame
} VoDebugStats;
// Returns the number of features written (up to maxFeatures).
int voGetDebugFrame(VoDebugStats* stats, VoDebugFeature* features, int maxFeatures);

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
