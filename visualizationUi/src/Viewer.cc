#include "orbslam3/visualizationUi/Viewer.h"
#include "orbslam3/visualizationUi/MapDrawer.h"
#include "orbslam3/commonCore/SensorType.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM3 {

static void SE3fToOpenGlMatrix(const Sophus::SE3f &pose,
                                pangolin::OpenGlMatrix &M,
                                pangolin::OpenGlMatrix &MOw) {
    Eigen::Matrix4f Twc = pose.matrix();
    for (int i = 0; i < 4; i++) {
        M.m[4 * i]     = Twc(0, i);
        M.m[4 * i + 1] = Twc(1, i);
        M.m[4 * i + 2] = Twc(2, i);
        M.m[4 * i + 3] = Twc(3, i);
    }
    MOw.SetIdentity();
    MOw.m[12] = Twc(0, 3);
    MOw.m[13] = Twc(1, 3);
    MOw.m[14] = Twc(2, 3);
}

Viewer::Viewer(VisualizationModel *visModel, const std::string &settingPath,
               Settings *settings)
    : mpVisModel(visModel) {
  if (settings) {
    newParameterLoader(settings);
  } else {
    cv::FileStorage fSettings(settingPath, cv::FileStorage::READ);
    bool is_correct = ParseViewerParamFile(fSettings);
    if (!is_correct) {
      std::cerr << "**ERROR in the config file, the format is not correct**"
                << std::endl;
      try {
        throw -1;
      } catch (std::exception &e) {
      }
    }
  }
}

void Viewer::newParameterLoader(Settings *settings) {
  mImageViewerScale = 1.f;

  float fps = settings->fps();
  if (fps < 1)
    fps = 30;
  mT = 1e3 / fps;

  cv::Size imSize = settings->newImSize();
  mImageHeight = imSize.height;
  mImageWidth = imSize.width;

  mImageViewerScale = settings->imageViewerScale();
  mViewpointX = settings->viewPointX();
  mViewpointY = settings->viewPointY();
  mViewpointZ = settings->viewPointZ();
  mViewpointF = settings->viewPointF();
}

bool Viewer::ParseViewerParamFile(cv::FileStorage &fSettings) {
  bool b_miss_params = false;
  mImageViewerScale = 1.f;

  float fps = fSettings["Camera.fps"];
  if (fps < 1)
    fps = 30;
  mT = 1e3 / fps;

  cv::FileNode node = fSettings["Camera.width"];
  if (!node.empty()) {
    mImageWidth = node.real();
  } else {
    std::cerr
        << "*Camera.width parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Camera.height"];
  if (!node.empty()) {
    mImageHeight = node.real();
  } else {
    std::cerr
        << "*Camera.height parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Viewer.imageViewScale"];
  if (!node.empty()) {
    mImageViewerScale = node.real();
  }

  node = fSettings["Viewer.ViewpointX"];
  if (!node.empty()) {
    mViewpointX = node.real();
  } else {
    std::cerr
        << "*Viewer.ViewpointX parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Viewer.ViewpointY"];
  if (!node.empty()) {
    mViewpointY = node.real();
  } else {
    std::cerr
        << "*Viewer.ViewpointY parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Viewer.ViewpointZ"];
  if (!node.empty()) {
    mViewpointZ = node.real();
  } else {
    std::cerr
        << "*Viewer.ViewpointZ parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  node = fSettings["Viewer.ViewpointF"];
  if (!node.empty()) {
    mViewpointF = node.real();
  } else {
    std::cerr
        << "*Viewer.ViewpointF parameter doesn't exist or is not a real number*"
        << std::endl;
    b_miss_params = true;
  }

  return !b_miss_params;
}

void Viewer::Run() {
  pangolin::CreateWindowAndBind("ORB-SLAM3: Map Viewer", 1024, 768);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(175));
  pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", false, true);
  pangolin::Var<bool> menuCamView("menu.Camera View", false, false);
  pangolin::Var<bool> menuTopView("menu.Top View", false, false);
  pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
  pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
  pangolin::Var<bool> menuShowGraph("menu.Show Graph", false, true);
  pangolin::Var<bool> menuShowInertialGraph("menu.Show Inertial Graph", true,
                                            true);
  pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false,
                                           true);
  pangolin::Var<bool> menuReset("menu.Reset", false, false);
  pangolin::Var<bool> menuStop("menu.Stop", false, false);
  pangolin::Var<bool> menuShowOptLba("menu.Show LBA opt", false, true);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389,
                                 0.1, 1000),
      pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0,
                                0.0, -1.0, 0.0));

  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175),
                                         1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::OpenGlMatrix Twc, Twr;
  Twc.SetIdentity();
  pangolin::OpenGlMatrix Ow;
  Ow.SetIdentity();
  cv::namedWindow("ORB-SLAM3: Current Frame");

  bool bFollow = true;
  bool bLocalizationMode = false;
  bool bCameraView = true;

  int sensorType = mpVisModel->GetSensorType();
  if (sensorType == static_cast<int>(SensorType::MONOCULAR) ||
      sensorType == static_cast<int>(SensorType::STEREO) ||
      sensorType == static_cast<int>(SensorType::RGBD)) {
    menuShowGraph = true;
  }

  float trackedImageScale = mpVisModel->GetImageScale();
  MapDrawer *pMapDrawer = mpVisModel->GetMapDrawer();

  std::cout << "Starting the Viewer" << std::endl;
  while (1) {
    // Check exit request
    {
      std::unique_lock<std::mutex> lock(mMutexExit);
      if (mbExitRequested)
        break;
    }

    // Skip rendering during reset
    if (mpVisModel->IsResetting()) {
      usleep(3000);
      continue;
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    SE3fToOpenGlMatrix(mpVisModel->GetCameraPose(), Twc, Ow);

    if (menuFollowCamera && bFollow) {
      if (bCameraView)
        s_cam.Follow(Twc);
      else
        s_cam.Follow(Ow);
    } else if (menuFollowCamera && !bFollow) {
      if (bCameraView) {
        s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(
            1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000));
        s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(
            mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
        s_cam.Follow(Twc);
      } else {
        s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(
            1024, 768, 3000, 3000, 512, 389, 0.1, 1000));
        s_cam.SetModelViewMatrix(
            pangolin::ModelViewLookAt(0, 0.01, 10, 0, 0, 0, 0.0, 0.0, 1.0));
        s_cam.Follow(Ow);
      }
      bFollow = true;
    } else if (!menuFollowCamera && bFollow) {
      bFollow = false;
    }

    if (menuCamView) {
      menuCamView = false;
      bCameraView = true;
      s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(
          1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 10000));
      s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(
          mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
      s_cam.Follow(Twc);
    }

    if (menuTopView && mpVisModel->IsImuInitialized()) {
      menuTopView = false;
      bCameraView = false;
      s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(
          1024, 768, 3000, 3000, 512, 389, 0.1, 10000));
      s_cam.SetModelViewMatrix(
          pangolin::ModelViewLookAt(0, 0.01, 50, 0, 0, 0, 0.0, 0.0, 1.0));
      s_cam.Follow(Ow);
    }

    if (menuLocalizationMode && !bLocalizationMode) {
      if (mEventCallback)
        mEventCallback(UiEvent::LocalizationOn);
      bLocalizationMode = true;
    } else if (!menuLocalizationMode && bLocalizationMode) {
      if (mEventCallback)
        mEventCallback(UiEvent::LocalizationOff);
      bLocalizationMode = false;
    }

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    pMapDrawer->DrawCurrentCamera(Twc);
    if (menuShowKeyFrames || menuShowGraph || menuShowInertialGraph ||
        menuShowOptLba)
      pMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph,
                                menuShowInertialGraph, menuShowOptLba);
    if (menuShowPoints)
      pMapDrawer->DrawMapPoints();

    pangolin::FinishFrame();

    cv::Mat toShow;
    cv::Mat im = mpVisModel->DrawFrame(trackedImageScale);

    if (mpVisModel->IsStereo()) {
      cv::Mat imRight = mpVisModel->DrawRightFrame(trackedImageScale);
      cv::hconcat(im, imRight, toShow);
    } else {
      toShow = im;
    }

    if (mImageViewerScale != 1.f) {
      int width = toShow.cols * mImageViewerScale;
      int height = toShow.rows * mImageViewerScale;
      cv::resize(toShow, toShow, cv::Size(width, height));
    }

    cv::imshow("ORB-SLAM3: Current Frame", toShow);
    cv::waitKey(mT);

    if (menuReset) {
      menuShowGraph = true;
      menuShowInertialGraph = true;
      menuShowKeyFrames = true;
      menuShowPoints = true;
      menuLocalizationMode = false;
      if (bLocalizationMode) {
        if (mEventCallback)
          mEventCallback(UiEvent::LocalizationOff);
      }
      bLocalizationMode = false;
      bFollow = true;
      menuFollowCamera = true;
      if (mEventCallback)
        mEventCallback(UiEvent::ResetRequested);
      menuReset = false;
    }

    if (menuStop) {
      if (bLocalizationMode) {
        if (mEventCallback)
          mEventCallback(UiEvent::LocalizationOff);
      }
      if (mEventCallback)
        mEventCallback(UiEvent::StopRequested);
      menuStop = false;
    }

    // Check if Pangolin window was closed
    if (pangolin::ShouldQuit())
      break;
  }

  cv::destroyWindow("ORB-SLAM3: Current Frame");
  pangolin::DestroyWindow("ORB-SLAM3: Map Viewer");

  std::unique_lock<std::mutex> lock(mMutexExit);
  mbExited = true;
}

void Viewer::RequestExit() {
  std::unique_lock<std::mutex> lock(mMutexExit);
  mbExitRequested = true;
}

bool Viewer::IsExited() const {
  std::unique_lock<std::mutex> lock(mMutexExit);
  return mbExited;
}

void Viewer::SetEventCallback(std::function<void(UiEvent)> cb) {
  mEventCallback = std::move(cb);
}

} // namespace ORB_SLAM3
