#include "voModule.h"

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include <atomic>
#include <cstring>
#include <mutex>
#include <queue>
#include <thread>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <condition_variable>
#include <unordered_set>
#include <cmath>

// ─── Tuning constants ────────────────────────────────────────────────────────

static const int   ORB_N_FEATURES    = 500;
static const float LOWE_RATIO        = 0.75f;
static const float RANSAC_THRESH_PX  = 3.0f;
static const int   RANSAC_ITER       = 200;
static const int   MIN_INLIERS       = 15;   // min PnP inliers to accept a pose
static const float MIN_DEPTH_M       = 0.30f;
static const float MAX_DEPTH_M       = 7.00f;
static const int   DEPTH_SAMPLE_STEP = 8;    // add 1 map point every NxN depth pixels
static const int   MAX_QUEUE_SIZE    = 3;    // drop frames if worker is behind
static const int   MAX_MAP_POINTS    = 1000000;
// Conservative handheld RGB-D motion limits per accepted frame. Degenerate
// depth/PnP fits must not teleport the map or the following viewer camera.
static const double MAX_STEP_METRES = 0.5;
static const double MAX_STEP_RADIANS = 0.7853981633974483; // 45 degrees
// Map points are inserted only on keyframes and deduplicated per voxel, so a
// long or static recording does not exhaust MAX_MAP_POINTS with repeats.
static const double KEYFRAME_METRES  = 0.05;
static const double KEYFRAME_RADIANS = 0.0872664625997165; // 5 degrees
static const float  MAP_VOXEL_M      = 0.03f;
// Consecutive lost frames before re-anchoring tracking on the current frame.
static const int    MAX_LOST_FRAMES  = 15;

// ─── Camera intrinsics ───────────────────────────────────────────────────────

static float g_fx = 570.3f, g_fy = 570.3f;
static float g_cx = 320.0f, g_cy = 240.0f;

// ─── Map storage ─────────────────────────────────────────────────────────────
// Layout per point: [x, y, z, r, g, b] (coords pre-negated for viewer)

static float*      g_mapBuf   = nullptr;
static int         g_mapCount = 0;
static std::mutex  g_mapMtx;
static VoCameraState g_camera = {};
static std::vector<uint8_t> g_cameraRgb;
static std::vector<float> g_trajectory;
static unsigned int g_cameraRevision = 0;
static std::unordered_set<int64_t> g_mapVoxels;

// Debug snapshot of the last processed frame, guarded by g_debugMtx.
static std::mutex g_debugMtx;
static VoDebugStats g_debugStats = {};
static std::vector<VoDebugFeature> g_debugFeatures;

// ─── Worker thread state ─────────────────────────────────────────────────────

struct FrameJob {
    cv::Mat color;    // CV_8UC3 RGB
    cv::Mat depth16;  // CV_16SC1 (or CV_16UC1), millimetres
    uint32_t ts_ms;
};

static std::queue<FrameJob> g_jobQueue;
static std::mutex           g_queueMtx;
static std::condition_variable g_idle;
static bool g_busy = false;
static std::thread          g_worker;
static std::atomic<bool>    g_running{false};

// ─── Tracking state (written by worker, read by API callers) ─────────────────

static std::atomic<int> g_status{0};     // 0=idle, 1=ok, 2=lost
static std::atomic<int> g_frameCount{0};

// ─── Per-frame VO state (only accessed by worker thread) ─────────────────────

static cv::Mat g_prevGray;
static cv::Mat g_prevDepthF;                    // CV_32F, metres
static std::vector<cv::KeyPoint> g_prevKps;
static cv::Mat g_prevDesc;
static cv::Mat g_T_w_c;                         // 4×4 CV_64F, camera→world
static cv::Mat g_T_w_kf;                        // pose of the last map keyframe
static int     g_lostFrames = 0;

// ─── Helpers ─────────────────────────────────────────────────────────────────

static int64_t voxelKey(float x, float y, float z)
{
    const int64_t mask = (1 << 21) - 1;
    int64_t ix = (int64_t)std::floor(x / MAP_VOXEL_M) & mask;
    int64_t iy = (int64_t)std::floor(y / MAP_VOXEL_M) & mask;
    int64_t iz = (int64_t)std::floor(z / MAP_VOXEL_M) & mask;
    return (ix << 42) | (iy << 21) | iz;
}

// True when the camera moved far enough from the last keyframe to add map points.
static bool isNewKeyframe()
{
    if (g_T_w_kf.empty()) return true;
    cv::Mat delta = g_T_w_kf.inv() * g_T_w_c;
    cv::Mat rvec;
    cv::Rodrigues(delta(cv::Rect(0, 0, 3, 3)), rvec);
    return cv::norm(delta(cv::Rect(3, 0, 1, 3))) >= KEYFRAME_METRES ||
           cv::norm(rvec) >= KEYFRAME_RADIANS;
}

// Publishes the accepted pose/image every frame; adds map points on keyframes.
static void addFrameToMap(const cv::Mat& colorRGB, const cv::Mat& depthF)
{
    std::lock_guard<std::mutex> lk(g_mapMtx);

    const double* Tw = g_T_w_c.ptr<double>(0); // row-major 4×4
    // Map storage negates all three world coordinates. Local camera coordinates
    // use the same convention: S*T*S has R unchanged and translation -t.
    for(int row=0;row<4;row++) for(int col=0;col<4;col++)
        g_camera.cameraToWorld[col*4+row] = (float)g_T_w_c.at<double>(row,col);
    for(int axis=0;axis<3;axis++) g_camera.cameraToWorld[12+axis] *= -1;
    g_camera.fx=g_fx;g_camera.fy=g_fy;g_camera.cx=g_cx;g_camera.cy=g_cy;
    g_camera.width=colorRGB.cols;g_camera.height=colorRGB.rows;
    g_camera.valid=1;g_camera.revision=++g_cameraRevision;
    g_cameraRgb.assign(colorRGB.data,colorRGB.data+colorRGB.total()*3);
    if(g_trajectory.size()>=30000)g_trajectory.erase(g_trajectory.begin(),g_trajectory.begin()+3000);
    for(int axis=0;axis<3;axis++)g_trajectory.push_back(g_camera.cameraToWorld[12+axis]);

    if (!isNewKeyframe()) return;
    g_T_w_kf = g_T_w_c.clone();

    for (int row = 0; row < depthF.rows && g_mapCount < MAX_MAP_POINTS; row += DEPTH_SAMPLE_STEP) {
        const float* drow  = depthF.ptr<float>(row);
        const uchar* crow  = colorRGB.ptr<uchar>(row);

        for (int col = 0; col < depthF.cols && g_mapCount < MAX_MAP_POINTS; col += DEPTH_SAMPLE_STEP) {
            float d = drow[col];
            if (d < MIN_DEPTH_M || d > MAX_DEPTH_M) continue;

            // 3-D point in current camera frame
            double xc = (col - g_cx) * d / g_fx;
            double yc = (row - g_cy) * d / g_fy;
            double zc = d;

            // Transform to world frame:  P_w = T_w_c * [xc,yc,zc,1]^T
            double xw = Tw[0]*xc + Tw[1]*yc + Tw[2]*zc  + Tw[3];
            double yw = Tw[4]*xc + Tw[5]*yc + Tw[6]*zc  + Tw[7];
            double zw = Tw[8]*xc + Tw[9]*yc + Tw[10]*zc + Tw[11];

            // Store with negated coords to match viewer's OpenGL convention
            // (viewer draws live cloud as glVertex3f(-x, -y, -z) in camera space)
            if (!g_mapVoxels.insert(voxelKey(-(float)xw, -(float)yw, -(float)zw)).second)
                continue;
            float* slot = g_mapBuf + (size_t)g_mapCount * 6;
            slot[0] = -(float)xw;
            slot[1] = -(float)yw;
            slot[2] = -(float)zw;

            // Color: RGB order input → store as r,g,b
            int cpx = col * 3;
            slot[3] = crow[cpx    ] / 255.0f;  // R
            slot[4] = crow[cpx + 1] / 255.0f;  // G
            slot[5] = crow[cpx + 2] / 255.0f;  // B

            ++g_mapCount;
        }
    }
}

static cv::Mat makeTFromRvecTvec(const cv::Mat& rvec, const cv::Mat& tvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    tvec.reshape(1, 3).copyTo(T(cv::Rect(3, 0, 1, 3)));
    return T;
}

static void publishDebug(int width, int height, int matches, int withDepth, int inliers,
                         int reason, std::vector<VoDebugFeature>& features)
{
    std::lock_guard<std::mutex> lock(g_debugMtx);
    unsigned int frame = g_debugStats.frame + 1;
    g_debugStats = {width, height, (int)features.size(), matches, withDepth, inliers, reason, frame};
    g_debugFeatures.swap(features);
}

// ─── Worker thread ────────────────────────────────────────────────────────────

static void workerLoop()
{
    auto orb     = cv::ORB::create(ORB_N_FEATURES);
    auto matcher = cv::BFMatcher::create(cv::NORM_HAMMING, false);

    // Camera matrix (no distortion — Astra is pre-rectified)
    cv::Mat K = (cv::Mat_<double>(3, 3)
        << g_fx,    0, g_cx,
              0, g_fy, g_cy,
              0,    0,    1);

    g_T_w_c = cv::Mat::eye(4, 4, CV_64F);
    g_T_w_kf = cv::Mat();
    g_lostFrames = 0;
    g_status = 0;

    while (g_running) {
        // ── Dequeue ──────────────────────────────────────────────────────────
        FrameJob job;
        {
            std::unique_lock<std::mutex> lk(g_queueMtx);
            if (g_jobQueue.empty()) {
                lk.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            job = std::move(g_jobQueue.front());
            g_jobQueue.pop();
            g_busy = true;
        }

        // Every exit path (including early continue) signals completed processing.
        struct Completion {
            ~Completion(){
                std::lock_guard<std::mutex> lock(g_queueMtx);
                g_busy = false;
                ++g_frameCount;
                g_idle.notify_all();
            }
        } completion;
        int frameIdx = g_frameCount + 1;

        // ── Preprocess ───────────────────────────────────────────────────────
        cv::Mat depthF;
        job.depth16.convertTo(depthF, CV_32F, 1.0 / 1000.0);  // mm → m

        cv::Mat gray;
        cv::cvtColor(job.color, gray, cv::COLOR_RGB2GRAY);

        // ── Feature detection ─────────────────────────────────────────────────
        std::vector<cv::KeyPoint> kps;
        cv::Mat desc;
        orb->detectAndCompute(gray, cv::noArray(), kps, desc);

        std::vector<VoDebugFeature> dbg(kps.size());
        for (size_t i = 0; i < kps.size(); i++)
            dbg[i] = {kps[i].pt.x, kps[i].pt.y, 0, 0, VO_FEAT_UNMATCHED};

        if (kps.empty() || desc.empty()) {
            publishDebug(gray.cols, gray.rows, 0, 0, 0, VO_REASON_NO_FEATURES, dbg);
            g_prevGray   = gray;
            g_prevDepthF = depthF;
            g_prevKps    = {};
            g_prevDesc   = cv::Mat();
            g_status     = 2;
            continue;
        }

        // ── First frame — initialise ──────────────────────────────────────────
        if (g_prevGray.empty() || g_prevDesc.empty()) {
            addFrameToMap(job.color, depthF);
            g_prevGray   = gray;
            g_prevDepthF = depthF;
            g_prevKps    = kps;
            g_prevDesc   = desc;
            g_status     = 1;
            publishDebug(gray.cols, gray.rows, 0, 0, 0, VO_REASON_INIT, dbg);
            printf("[VO] frame %d: first frame, map=%d pts\n", frameIdx, g_mapCount);
            continue;
        }

        // ── KNN matching + Lowe's ratio test ─────────────────────────────────
        std::vector<std::vector<cv::DMatch>> knnMatches;
        matcher->knnMatch(g_prevDesc, desc, knnMatches, 2);

        std::vector<cv::DMatch> good;
        good.reserve(knnMatches.size());
        for (auto& m : knnMatches) {
            if (m.size() >= 2 && m[0].distance < LOWE_RATIO * m[1].distance)
                good.push_back(m[0]);
        }

        // ── Build 3-D (prev frame) → 2-D (current frame) correspondences ─────
        std::vector<cv::Point3f> pts3D;
        std::vector<cv::Point2f> pts2D;
        std::vector<int> pts3DFeature;      // current-frame feature index per 3-D point
        pts3D.reserve(good.size());
        pts2D.reserve(good.size());

        for (auto& m : good) {
            const cv::KeyPoint& kp = g_prevKps[m.queryIdx];
            VoDebugFeature& f = dbg[m.trainIdx];
            f.prevX = kp.pt.x; f.prevY = kp.pt.y; f.state = VO_FEAT_NO_DEPTH;
            int px = (int)kp.pt.x;
            int py = (int)kp.pt.y;
            if (px < 0 || py < 0 || px >= g_prevDepthF.cols || py >= g_prevDepthF.rows)
                continue;
            float d = g_prevDepthF.at<float>(py, px);
            if (d < MIN_DEPTH_M || d > MAX_DEPTH_M) continue;

            pts3D.push_back({(kp.pt.x - g_cx) * d / g_fx,
                             (kp.pt.y - g_cy) * d / g_fy,
                             d});
            pts2D.push_back(kps[m.trainIdx].pt);
            pts3DFeature.push_back(m.trainIdx);
            f.state = VO_FEAT_DEPTH;
        }

        // ── PnP pose estimation ───────────────────────────────────────────────
        bool poseOk = false;
        int reason = (int)good.size() < MIN_INLIERS ? VO_REASON_FEW_MATCHES : VO_REASON_FEW_DEPTH;
        int inlierCount = 0;
        if ((int)pts3D.size() >= MIN_INLIERS) {
            cv::Mat rvec, tvec, inliers;
            // SQPnP is stable on planar scenes (walls, floors) where ITERATIVE's
            // homography initialisation flips to wildly wrong poses.
            bool ok = cv::solvePnPRansac(
                pts3D, pts2D, K, cv::Mat(),
                rvec, tvec,
                false,            // useExtrinsicGuess
                RANSAC_ITER,
                RANSAC_THRESH_PX,
                0.999,
                inliers,
                cv::SOLVEPNP_SQPNP) && inliers.rows >= MIN_INLIERS;

            reason = ok ? VO_REASON_IMPLAUSIBLE : VO_REASON_PNP_FAILED;
            if (ok) {
                inlierCount = inliers.rows;
                for (int i = 0; i < inliers.rows; i++)
                    dbg[pts3DFeature[inliers.at<int>(i)]].state = VO_FEAT_INLIER;
                std::vector<cv::Point3f> in3D;
                std::vector<cv::Point2f> in2D;
                for (int i = 0; i < inliers.rows; i++) {
                    in3D.push_back(pts3D[inliers.at<int>(i)]);
                    in2D.push_back(pts2D[inliers.at<int>(i)]);
                }
                cv::solvePnPRefineLM(in3D, in2D, K, cv::Mat(), rvec, tvec);
            }

            bool plausible = ok && cv::checkRange(rvec) && cv::checkRange(tvec) &&
                cv::norm(tvec) <= MAX_STEP_METRES && cv::norm(rvec) <= MAX_STEP_RADIANS;
            if(ok && !plausible)
                printf("[VO] rejected implausible motion at frame %d\n", frameIdx);
            if (plausible) {
                // T_c1_c0: transforms prev-cam coords → curr-cam coords
                cv::Mat T_c1_c0 = makeTFromRvecTvec(rvec, tvec);

                // T_w_c_new = T_w_c_old * T_c0_c1 = T_w_c_old * inv(T_c1_c0)
                g_T_w_c = g_T_w_c * T_c1_c0.inv();

                addFrameToMap(job.color, depthF);
                poseOk = true;
                reason = VO_REASON_OK;
                g_status = 1;
                g_lostFrames = 0;

                printf("[VO] frame %d: %d inliers / %d matches, map=%d pts\n",
                       frameIdx, inliers.rows, (int)good.size(), g_mapCount);
            }
        }

        publishDebug(gray.cols, gray.rows, (int)good.size(), (int)pts3D.size(),
                     inlierCount, reason, dbg);

        if (!poseOk) {
            g_status = 2;
            printf("[VO] frame %d: tracking lost (%d pts3D, %d matches)\n",
                   frameIdx, (int)pts3D.size(), (int)good.size());
            // Keep matching against the last accepted frame, whose pose is known.
            // Only after a long loss re-anchor, accepting a gap in the trajectory.
            if (++g_lostFrames < MAX_LOST_FRAMES) continue;
            g_lostFrames = 0;
        }

        // ── Store previous frame ──────────────────────────────────────────────
        g_prevGray   = gray;
        g_prevDepthF = depthF;
        g_prevKps    = kps;
        g_prevDesc   = desc;
    }

    printf("[VO] worker stopped\n");
}

// ─── Public API ───────────────────────────────────────────────────────────────

void voInit(float fx, float fy, float cx, float cy)
{
    if (g_running) return;

    g_fx = fx; g_fy = fy;
    g_cx = cx; g_cy = cy;

    {
        std::lock_guard<std::mutex> lock(g_mapMtx);
        g_mapBuf = new float[(size_t)MAX_MAP_POINTS * 6]();
        g_mapCount = 0;
        g_camera = {};
        g_cameraRgb.clear();g_trajectory.clear();
        g_mapVoxels.clear();
    }
    {
        std::lock_guard<std::mutex> lock(g_debugMtx);
        g_debugStats = {};
        g_debugFeatures.clear();
    }

    g_prevGray   = cv::Mat();
    g_prevDepthF = cv::Mat();
    g_prevKps.clear();
    g_prevDesc   = cv::Mat();
    g_frameCount = 0;
    g_status     = 0;

    g_running = true;
    g_worker  = std::thread(workerLoop);

    printf("[VO] initialised  fx=%.1f fy=%.1f cx=%.1f cy=%.1f\n", fx, fy, cx, cy);
}

void voStop(void)
{
    if (!g_running) return;
    g_running = false;
    if (g_worker.joinable()) g_worker.join();

    // drain queue
    std::lock_guard<std::mutex> lk(g_queueMtx);
    while (!g_jobQueue.empty()) g_jobQueue.pop();

    std::lock_guard<std::mutex> mapLock(g_mapMtx);
    delete[] g_mapBuf;
    g_mapBuf   = nullptr;
    g_mapCount = 0;
    g_camera = {};g_cameraRgb.clear();g_trajectory.clear();
    g_mapVoxels.clear();
    {
        std::lock_guard<std::mutex> lock(g_debugMtx);
        g_debugStats = {};
        g_debugFeatures.clear();
    }
    g_status   = 0;

    printf("[VO] stopped\n");
}

int voIsRunning(void)
{
    return g_running ? 1 : 0;
}

void voProcessFrame(const int16_t* depth16, const uint8_t* rgb,
                    int width, int height, uint32_t timestamp_ms)
{
    if (!g_running) return;

    // Drop frame if queue is already full
    {
        std::lock_guard<std::mutex> lk(g_queueMtx);
        if ((int)g_jobQueue.size() >= MAX_QUEUE_SIZE) return;
    }

    FrameJob job;
    job.ts_ms = timestamp_ms;

    // Wrap raw buffers as cv::Mat — clone to own the data
    cv::Mat colorWrap(height, width, CV_8UC3, const_cast<uint8_t*>(rgb));
    job.color = colorWrap.clone();

    cv::Mat depthWrap(height, width, CV_16SC1, const_cast<int16_t*>(depth16));
    job.depth16 = depthWrap.clone();

    {
        std::lock_guard<std::mutex> lk(g_queueMtx);
        g_jobQueue.push(std::move(job));
    }
}

int voGetMapPoints(float* dst, int max_points)
{
    std::lock_guard<std::mutex> lk(g_mapMtx);
    int n = std::min(g_mapCount, max_points);
    if (n > 0 && g_mapBuf && dst){
        // Sample across the full map when the viewer's display budget is smaller.
        for(int i=0;i<n;i++){
            size_t source=(size_t)i*g_mapCount/n;
            std::memcpy(dst+i*6,g_mapBuf+source*6,6*sizeof(float));
        }
    }
    return n;
}

int voGetCameraState(VoCameraState* state, uint8_t* rgb, int capacity,
                     float* trajectory, int maxVertices){
    std::lock_guard<std::mutex> lock(g_mapMtx);
    if(state)*state=g_camera;
    if(rgb && capacity>=0 && (size_t)capacity>=g_cameraRgb.size())
        std::copy(g_cameraRgb.begin(),g_cameraRgb.end(),rgb);
    int count=std::min(std::max(0,maxVertices),(int)g_trajectory.size()/3);
    if(trajectory && count)std::copy(g_trajectory.end()-count*3,g_trajectory.end(),trajectory);
    return count;
}

int voGetDebugFrame(VoDebugStats* stats, VoDebugFeature* features, int maxFeatures)
{
    std::lock_guard<std::mutex> lock(g_debugMtx);
    if (stats) *stats = g_debugStats;
    int count = std::min(std::max(0, maxFeatures), (int)g_debugFeatures.size());
    if (features && count) std::copy_n(g_debugFeatures.begin(), count, features);
    return count;
}

int voGetMapPointCount(void)
{
    std::lock_guard<std::mutex> lk(g_mapMtx);
    return g_mapCount;
}

int voGetTrackingStatus(void)
{
    return g_status.load();
}

int voGetFrameCount(void)
{
    return g_frameCount.load();
}

void voReset(void)
{
    if (!g_running) return;

    // Join before touching worker-owned tracking state.
    float fx=g_fx, fy=g_fy, cx=g_cx, cy=g_cy;
    voStop();
    voInit(fx, fy, cx, cy);
    printf("[VO] reset\n");
}

void voWaitIdle(void){
    std::unique_lock<std::mutex> lock(g_queueMtx);
    g_idle.wait(lock, []{ return g_jobQueue.empty() && !g_busy; });
}

int voSaveMap(const char* filename){
    std::lock_guard<std::mutex> lock(g_mapMtx);
    std::ofstream out(filename);
    if(!out) return 0;
    out << "ply\nformat ascii 1.0\ncomment VO point cloud, metres, viewer coordinates\n"
        << "element vertex " << g_mapCount
        << "\nproperty float x\nproperty float y\nproperty float z\n"
        << "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
    for(int i=0;i<g_mapCount;i++){
        const float* p=g_mapBuf+i*6;
        out << p[0] << ' ' << p[1] << ' ' << p[2];
        for(int c=3;c<6;c++) out << ' ' << (int)std::max(0.f,std::min(255.f,p[c]*255));
        out << '\n';
    }
    out.close();
    return !out.fail();
}
