#ifndef SYSTEM_H
#define SYSTEM_H

#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <unistd.h>

#include "orbslam3/commonCore/SensorType.h"
#include "orbslam3/commonCore/Verbose.h"
#include "orbslam3/mapCore/Atlas.h"
#include "orbslam3/visualizationUi/FrameDrawer.h"
#include "orbslam3/commonCore/ImuTypes.h"
#include "orbslam3/mapCore/KeyFrameDatabase.h"
#include "orbslam3/mappingBackend/LocalMapping.h"
#include "orbslam3/mappingBackend/LoopClosing.h"
#include "orbslam3/visualizationUi/MapDrawer.h"
#include "orbslam3/featureCore/ORBVocabulary.h"
#include "orbslam3/configCore/Settings.h"
#include "orbslam3/trackingFrontend/Tracking.h"

namespace ORB_SLAM3 {

class FrameDrawer;
class MapDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;

class System {
public:
  // Input sensor
  using eSensor = SensorType;
  static constexpr eSensor MONOCULAR = SensorType::MONOCULAR;
  static constexpr eSensor STEREO = SensorType::STEREO;
  static constexpr eSensor RGBD = SensorType::RGBD;
  static constexpr eSensor IMU_MONOCULAR = SensorType::IMU_MONOCULAR;
  static constexpr eSensor IMU_STEREO = SensorType::IMU_STEREO;
  static constexpr eSensor IMU_RGBD = SensorType::IMU_RGBD;

  // File type
  enum FileType {
    TEXT_FILE = 0,
    BINARY_FILE = 1,
  };

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Initialize the SLAM system. It launches the Local Mapping and Loop Closing
  // threads.
  System(const string &strVocFile, const string &strSettingsFile,
         const eSensor sensor,
         const int initFr = 0, const string &strSequence = std::string());

  // Proccess the given stereo frame. Images must be synchronized and rectified.
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Returns the camera pose (empty if tracking fails).
  Sophus::SE3f
  TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight,
              const double &timestamp,
              const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
              string filename = "");

  // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
  // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Input depthmap: Float (CV_32F). Returns the camera pose (empty
  // if tracking fails).
  Sophus::SE3f
  TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp,
            const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
            string filename = "");

  // Proccess the given monocular frame and optionally imu data
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Returns the camera pose (empty if tracking fails).
  Sophus::SE3f
  TrackMonocular(const cv::Mat &im, const double &timestamp,
                 const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
                 string filename = "");

  // This stops local mapping thread (map building) and performs only camera
  // tracking.
  void ActivateLocalizationMode();
  // This resumes local mapping thread and performs SLAM again.
  void DeactivateLocalizationMode();

  // Returns true if there have been a big map change (loop closure, global BA)
  // since last call to this function
  bool MapChanged();

  // Reset the system (clear Atlas or the active map)
  void Reset();
  void ResetActiveMap();

  // All threads will be requested to finish.
  // It waits until all threads have finished.
  // This function must be called before saving the trajectory.
  void Shutdown();
  bool isShutDown();

  // Save camera trajectory in the TUM RGB-D dataset format.
  void SaveTrajectoryTUM(const string &filename);
  void SaveKeyFrameTrajectoryTUM(const string &filename);

  void SaveTrajectoryEuRoC(const string &filename);
  void SaveKeyFrameTrajectoryEuRoC(const string &filename);

  void SaveTrajectoryEuRoC(const string &filename, Map *pMap);
  void SaveKeyFrameTrajectoryEuRoC(const string &filename, Map *pMap);

  // Save data used for initialization debug
  void SaveDebugData(const int &iniIdx);

  void SaveTrajectoryKITTI(const string &filename);

  // Information from most recent processed frame
  int GetTrackingState();
  std::vector<MapPoint *> GetTrackedMapPoints();
  std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

  // For debugging
  double GetTimeFromIMUInit();
  bool isLost();
  bool isFinished();

  void ChangeDataset();

  float GetImageScale();
  int GetSensorType() const { return static_cast<int>(mSensor); }

  // Getters for SlamSession / NavigationEngine
  Tracking *GetTracker() const { return mpTracker; }
  FrameDrawer *GetFrameDrawer() const { return mpFrameDrawer; }
  MapDrawer *GetMapDrawer() const { return mpMapDrawer; }
  Settings *GetSettings() const { return settings_; }
  Atlas *GetAtlas() const { return mpAtlas; }
  LocalMapping *GetLocalMapper() const { return mpLocalMapper; }
  LoopClosing *GetLoopCloser() const { return mpLoopCloser; }

#ifdef REGISTER_TIMES
  void InsertRectTime(double &time);
  void InsertResizeTime(double &time);
  void InsertTrackTime(double &time);
#endif

private:
  void SaveAtlas(int type);
  bool LoadAtlas(int type);

  string CalculateCheckSum(string filename, int type);

  // Input sensor
  eSensor mSensor;

  // ORB vocabulary used for place recognition and feature matching.
  ORBVocabulary *mpVocabulary;

  // KeyFrame database for place recognition (relocalization and loop
  // detection).
  KeyFrameDatabase *mpKeyFrameDatabase;

  // Map structure that stores the pointers to all KeyFrames and MapPoints.
  Atlas *mpAtlas;

  // Tracker. It receives a frame and computes the associated camera pose.
  Tracking *mpTracker;

  // Local Mapper. It manages the local map and performs local bundle
  // adjustment.
  LocalMapping *mpLocalMapper;

  // Loop Closer.
  LoopClosing *mpLoopCloser;

  FrameDrawer *mpFrameDrawer;
  MapDrawer *mpMapDrawer;

  // System threads: Local Mapping, Loop Closing.
  std::thread *mptLocalMapping;
  std::thread *mptLoopClosing;

  // Reset flag
  std::mutex mMutexReset;
  bool mbReset;
  bool mbResetActiveMap;

  // Change mode flags
  std::mutex mMutexMode;
  bool mbActivateLocalizationMode;
  bool mbDeactivateLocalizationMode;

  // Shutdown flag
  bool mbShutDown;

  // Tracking state
  int mTrackingState;
  std::vector<MapPoint *> mTrackedMapPoints;
  std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
  std::mutex mMutexState;

  //
  string mStrLoadAtlasFromFile;
  string mStrSaveAtlasToFile;

  string mStrVocabularyFilePath;

  Settings *settings_;
};

} // namespace ORB_SLAM3

#endif // SYSTEM_H
