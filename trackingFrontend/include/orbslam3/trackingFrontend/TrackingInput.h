#ifndef TRACKING_INPUT_H
#define TRACKING_INPUT_H

#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>
#include "orbslam3/commonCore/ImuTypes.h"
#include <vector>
#include <string>

namespace ORB_SLAM3 {

class System;

class TrackingInput {
public:
    explicit TrackingInput(System *system);

    Sophus::SE3f TrackMonocular(
        const cv::Mat &im, double timestamp,
        const std::vector<IMU::Point> &vImuMeas = {},
        std::string filename = "");

    Sophus::SE3f TrackStereo(
        const cv::Mat &imLeft, const cv::Mat &imRight, double timestamp,
        const std::vector<IMU::Point> &vImuMeas = {},
        std::string filename = "");

    Sophus::SE3f TrackRGBD(
        const cv::Mat &im, const cv::Mat &depthmap, double timestamp,
        const std::vector<IMU::Point> &vImuMeas = {},
        std::string filename = "");

    float GetImageScale() const;

private:
    System *mpSystem;
};

} // namespace ORB_SLAM3

#endif // TRACKING_INPUT_H
