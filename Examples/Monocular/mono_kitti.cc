#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <opencv2/core/core.hpp>

#include "orbslam3/trackingFrontend/SlamSession.h"
#include "orbslam3/trackingFrontend/TrackingInput.h"

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv) {
  if (argc != 4) {
    cerr << endl
         << "Usage: ./mono_kitti path_to_vocabulary path_to_settings "
            "path_to_sequence"
         << endl;
    return 1;
  }

  // Retrieve paths to images
  vector<string> vstrImageFilenames;
  vector<double> vTimestamps;
  LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

  int nImages = vstrImageFilenames.size();

  ORB_SLAM3::SlamSession::Config config;
  config.vocFile = argv[1];
  config.settingsFile = argv[2];
  config.sensor = ORB_SLAM3::System::MONOCULAR;
  config.useViewer = true;

  ORB_SLAM3::SlamSession session(config);

  session.StartTracking([&](ORB_SLAM3::TrackingInput *input,
                            std::function<bool()> shouldStop) {
    float imageScale = input->GetImageScale();

    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    cv::Mat im;
    for (int ni = 0; ni < nImages && !shouldStop(); ni++) {
      im = cv::imread(vstrImageFilenames[ni], cv::IMREAD_UNCHANGED);
      double tframe = vTimestamps[ni];

      if (im.empty()) {
        cerr << endl
             << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
        break;
      }

      if (imageScale != 1.f) {
        int width = im.cols * imageScale;
        int height = im.rows * imageScale;
        cv::resize(im, im, cv::Size(width, height));
      }

      std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();

      input->TrackMonocular(im, tframe, vector<ORB_SLAM3::IMU::Point>(),
                            vstrImageFilenames[ni]);

      std::chrono::steady_clock::time_point t2 =
          std::chrono::steady_clock::now();

      double ttrack =
          std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
              .count();

      vTimesTrack[ni] = ttrack;

      double T = 0;
      if (ni < nImages - 1)
        T = vTimestamps[ni + 1] - tframe;
      else if (ni > 0)
        T = tframe - vTimestamps[ni - 1];

      if (ttrack < T)
        usleep((T - ttrack) * 1e6);
    }

    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++) {
      totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;
  });

  session.RunMainLoop();

  session.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  return 0;
}

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps) {
  ifstream fTimes;
  string strPathTimeFile = strPathToSequence + "/times.txt";
  fTimes.open(strPathTimeFile.c_str());
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimestamps.push_back(t);
    }
  }

  string strPrefixLeft = strPathToSequence + "/image_0/";

  const int nTimes = vTimestamps.size();
  vstrImageFilenames.resize(nTimes);

  for (int i = 0; i < nTimes; i++) {
    stringstream ss;
    ss << setfill('0') << setw(6) << i;
    vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
  }
}
