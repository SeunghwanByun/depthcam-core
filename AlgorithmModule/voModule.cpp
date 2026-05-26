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

// ─── Camera intrinsics ───────────────────────────────────────────────────────

static float g_fx = 570.3f, g_fy = 570.3f;
static float g_cx = 320.0f, g_cy = 240.0f;

// ─── Map storage ─────────────────────────────────────────────────────────────
// Layout per point: [x, y, z, r, g, b] (coords pre-negated for viewer)

static float*      g_mapBuf   = nullptr;
static int         g_mapCount = 0;
static std::mutex  g_mapMtx;

// ─── Worker thread state ─────────────────────────────────────────────────────

struct FrameJob {
    cv::Mat color;    // CV_8UC3 RGB
    cv::Mat depth16;  // CV_16SC1 (or CV_16UC1), millimetres
    uint32_t ts_ms;
};

static std::queue<FrameJob> g_jobQueue;
static std::mutex           g_queueMtx;
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

// ─── Helpers ─────────────────────────────────────────────────────────────────

static void addFrameToMap(const cv::Mat& colorRGB, const cv::Mat& depthF)
{
    std::lock_guard<std::mutex> lk(g_mapMtx);

    const double* Tw = g_T_w_c.ptr<double>(0); // row-major 4×4

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
        }

        int frameIdx = ++g_frameCount;

        // ── Preprocess ───────────────────────────────────────────────────────
        cv::Mat depthF;
        job.depth16.convertTo(depthF, CV_32F, 1.0 / 1000.0);  // mm → m

        cv::Mat gray;
        cv::cvtColor(job.color, gray, cv::COLOR_RGB2GRAY);

        // ── Feature detection ─────────────────────────────────────────────────
        std::vector<cv::KeyPoint> kps;
        cv::Mat desc;
        orb->detectAndCompute(gray, cv::noArray(), kps, desc);

        if (kps.empty() || desc.empty()) {
            g_prevGray   = gray;
            g_prevDepthF = depthF;
            g_prevKps    = {};
            g_prevDesc   = cv::Mat();
            g_status     = 2;
            continue;
        }

        // ── First frame — initialise ──────────────────────────────────────────
        if (g_prevGray.empty()) {
            addFrameToMap(job.color, depthF);
            g_prevGray   = gray;
            g_prevDepthF = depthF;
            g_prevKps    = kps;
            g_prevDesc   = desc;
            g_status     = 1;
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
        pts3D.reserve(good.size());
        pts2D.reserve(good.size());

        for (auto& m : good) {
            const cv::KeyPoint& kp = g_prevKps[m.queryIdx];
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
        }

        // ── PnP pose estimation ───────────────────────────────────────────────
        bool poseOk = false;
        if ((int)pts3D.size() >= MIN_INLIERS) {
            cv::Mat rvec, tvec, inliers;
            bool ok = cv::solvePnPRansac(
                pts3D, pts2D, K, cv::Mat(),
                rvec, tvec,
                false,            // useExtrinsicGuess
                RANSAC_ITER,
                RANSAC_THRESH_PX,
                0.999,
                inliers,
                cv::SOLVEPNP_ITERATIVE);

            if (ok && inliers.rows >= MIN_INLIERS) {
                // T_c1_c0: transforms prev-cam coords → curr-cam coords
                cv::Mat T_c1_c0 = makeTFromRvecTvec(rvec, tvec);

                // T_w_c_new = T_w_c_old * T_c0_c1 = T_w_c_old * inv(T_c1_c0)
                g_T_w_c = g_T_w_c * T_c1_c0.inv();

                addFrameToMap(job.color, depthF);
                poseOk = true;
                g_status = 1;

                printf("[VO] frame %d: %d inliers / %d matches, map=%d pts\n",
                       frameIdx, inliers.rows, (int)good.size(), g_mapCount);
            }
        }

        if (!poseOk) {
            g_status = 2;
            printf("[VO] frame %d: tracking lost (%d pts3D, %d matches)\n",
                   frameIdx, (int)pts3D.size(), (int)good.size());
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

    g_mapBuf = new float[(size_t)MAX_MAP_POINTS * 6]();
    g_mapCount = 0;

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

    delete[] g_mapBuf;
    g_mapBuf   = nullptr;
    g_mapCount = 0;
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
    if (n > 0 && g_mapBuf)
        std::memcpy(dst, g_mapBuf, (size_t)n * 6 * sizeof(float));
    return n;
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

    // Clear the queue
    {
        std::lock_guard<std::mutex> lk(g_queueMtx);
        while (!g_jobQueue.empty()) g_jobQueue.pop();
    }

    // Reset VO state (safe: worker checks g_running and idles between jobs)
    {
        std::lock_guard<std::mutex> lk(g_mapMtx);
        g_mapCount = 0;
        if (g_mapBuf) std::memset(g_mapBuf, 0, (size_t)MAX_MAP_POINTS * 6 * sizeof(float));
    }

    g_prevGray   = cv::Mat();
    g_prevDepthF = cv::Mat();
    g_prevKps.clear();
    g_prevDesc   = cv::Mat();
    g_T_w_c      = cv::Mat::eye(4, 4, CV_64F);
    g_frameCount = 0;
    g_status     = 0;

    printf("[VO] reset\n");
}
