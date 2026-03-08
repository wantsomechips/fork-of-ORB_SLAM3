#include "orbslam3/trackingFrontend/TrackingInput.h"
#include "orbslam3/trackingFrontend/System.h"

namespace ORB_SLAM3 {

TrackingInput::TrackingInput(System *system) : mpSystem(system) {}

Sophus::SE3f TrackingInput::TrackMonocular(
    const cv::Mat &im, double timestamp,
    const std::vector<IMU::Point> &vImuMeas, std::string filename) {
    return mpSystem->TrackMonocular(im, timestamp, vImuMeas, filename);
}

Sophus::SE3f TrackingInput::TrackStereo(
    const cv::Mat &imLeft, const cv::Mat &imRight, double timestamp,
    const std::vector<IMU::Point> &vImuMeas, std::string filename) {
    return mpSystem->TrackStereo(imLeft, imRight, timestamp, vImuMeas, filename);
}

Sophus::SE3f TrackingInput::TrackRGBD(
    const cv::Mat &im, const cv::Mat &depthmap, double timestamp,
    const std::vector<IMU::Point> &vImuMeas, std::string filename) {
    return mpSystem->TrackRGBD(im, depthmap, timestamp, vImuMeas, filename);
}

float TrackingInput::GetImageScale() const {
    return mpSystem->GetImageScale();
}

} // namespace ORB_SLAM3
