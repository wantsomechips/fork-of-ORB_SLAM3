#ifndef VIEWER_H
#define VIEWER_H

#include "orbslam3/visualizationUi/VisualizationModel.h"
#include "orbslam3/configCore/Settings.h"

#include <functional>
#include <mutex>

namespace ORB_SLAM3 {

class Settings;

enum class UiEvent {
    StopRequested,
    ResetRequested,
    LocalizationOn,
    LocalizationOff,
};

class Viewer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Viewer(VisualizationModel *visModel, const std::string &settingPath,
         Settings *settings);

  void Run();

  void RequestExit();
  bool IsExited() const;

  void SetEventCallback(std::function<void(UiEvent)> cb);

private:
  void newParameterLoader(Settings *settings);
  bool ParseViewerParamFile(cv::FileStorage &fSettings);

  VisualizationModel *mpVisModel;
  std::function<void(UiEvent)> mEventCallback;

  double mT;
  float mImageWidth, mImageHeight;
  float mImageViewerScale;

  float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

  bool mbExitRequested = false;
  bool mbExited = false;
  mutable std::mutex mMutexExit;
};

} // namespace ORB_SLAM3

#endif // VIEWER_H
