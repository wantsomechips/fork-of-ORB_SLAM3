#ifndef VISUALIZATION_MODEL_H
#define VISUALIZATION_MODEL_H

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>

#include <atomic>
#include <mutex>

namespace ORB_SLAM3 {

class FrameDrawer;
class MapDrawer;

class VisualizationModel {
public:
    VisualizationModel(FrameDrawer *frameDrawer, MapDrawer *mapDrawer,
                       int sensorType, float imageScale, bool isStereo,
                       std::atomic<bool> *resettingFlag);

    // Frame images (Pangolin-free, returns cv::Mat)
    cv::Mat DrawFrame(float scale = 1.f);
    cv::Mat DrawRightFrame(float scale = 1.f);

    // Camera pose (Eigen type, not Pangolin)
    Sophus::SE3f GetCameraPose() const;

    // Metadata
    int GetSensorType() const { return mSensorType; }
    float GetImageScale() const { return mImageScale; }
    bool IsStereo() const { return mbStereo; }
    bool IsImuInitialized() const;
    bool IsResetting() const;

    // MapDrawer accessor (for Viewer's 3D rendering, both in visualizationUi layer)
    MapDrawer *GetMapDrawer() const { return mpMapDrawer; }

private:
    FrameDrawer *mpFrameDrawer;
    MapDrawer *mpMapDrawer;
    int mSensorType;
    float mImageScale;
    bool mbStereo;
    std::atomic<bool> *mpResettingFlag;
};

} // namespace ORB_SLAM3

#endif // VISUALIZATION_MODEL_H
