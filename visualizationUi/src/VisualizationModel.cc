#include "orbslam3/visualizationUi/VisualizationModel.h"
#include "orbslam3/visualizationUi/FrameDrawer.h"
#include "orbslam3/visualizationUi/MapDrawer.h"
#include "orbslam3/mapCore/Atlas.h"

namespace ORB_SLAM3 {

VisualizationModel::VisualizationModel(FrameDrawer *frameDrawer,
                                       MapDrawer *mapDrawer, int sensorType,
                                       float imageScale, bool isStereo,
                                       std::atomic<bool> *resettingFlag)
    : mpFrameDrawer(frameDrawer), mpMapDrawer(mapDrawer),
      mSensorType(sensorType), mImageScale(imageScale), mbStereo(isStereo),
      mpResettingFlag(resettingFlag) {}

cv::Mat VisualizationModel::DrawFrame(float scale) {
    return mpFrameDrawer->DrawFrame(scale);
}

cv::Mat VisualizationModel::DrawRightFrame(float scale) {
    return mpFrameDrawer->DrawRightFrame(scale);
}

Sophus::SE3f VisualizationModel::GetCameraPose() const {
    // MapDrawer stores mCameraPose as Twc (world-to-camera inverse).
    // We read it via the existing mutex-protected getter pattern.
    // MapDrawer::mCameraPose is already Twc (set via Tcw.inverse()).
    // We need to expose this. Since mCameraPose is private and
    // GetCurrentOpenGLCameraMatrix reads it, we replicate the lock logic.
    // For now, we access mpMapDrawer's public mpAtlas to check state,
    // and return the pose via the existing data path.
    // Note: MapDrawer stores mCameraPose = Tcw.inverse() = Twc
    // We return it as SE3f directly. The caller (Viewer) converts to OpenGL.
    return mpMapDrawer->GetCameraPose();
}

bool VisualizationModel::IsImuInitialized() const {
    return mpMapDrawer->mpAtlas->isImuInitialized();
}

bool VisualizationModel::IsResetting() const {
    return mpResettingFlag && mpResettingFlag->load();
}

} // namespace ORB_SLAM3
