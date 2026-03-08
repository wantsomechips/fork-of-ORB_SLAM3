#ifndef SLAM_SESSION_H
#define SLAM_SESSION_H

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <pthread.h>
#include <string>
#include "orbslam3/trackingFrontend/System.h"

namespace ORB_SLAM3 {

enum class UiEvent;
class Viewer;
class VisualizationModel;
class TrackingInput;

class SlamSession {
public:
    enum class State { Initializing, Running, Stopping, Stopped, Failed };

    enum class StopReason {
        InputExhausted,
        UserRequestedStop,
        WindowClosed,
        Error,
    };

    struct Config {
        std::string vocFile;
        std::string settingsFile;
        System::eSensor sensor;
        bool useViewer = true;
        int initFr = 0;
        std::string sequence;
    };

    explicit SlamSession(const Config &config);
    ~SlamSession();

    SlamSession(const SlamSession &) = delete;
    SlamSession &operator=(const SlamSession &) = delete;

    // trackingFn(TrackingInput*, shouldStop)
    void StartTracking(
        std::function<void(TrackingInput *, std::function<bool()>)> trackingFn);

    void RunMainLoop();

    void RequestStop(StopReason reason);

    // Session export (call after RunMainLoop returns)
    void SaveTrajectoryTUM(const std::string &filename);
    void SaveTrajectoryEuRoC(const std::string &filename);
    void SaveKeyFrameTrajectoryTUM(const std::string &filename);
    void SaveKeyFrameTrajectoryEuRoC(const std::string &filename);
    void SaveTrajectoryKITTI(const std::string &filename);

    // For multi-sequence support
    void ChangeDataset();

    State GetState() const;
    StopReason GetStopReason() const;

private:
    void DoShutdown();
    void HandleUiEvent(UiEvent event);

    Config mConfig;
    std::atomic<State> mState{State::Initializing};
    StopReason mStopReason{StopReason::InputExhausted};
    std::once_flag mShutdownOnce;

    std::unique_ptr<System> mpSystem;
    std::unique_ptr<VisualizationModel> mpVisModel;
    std::unique_ptr<Viewer> mpViewer;
    std::unique_ptr<TrackingInput> mpTrackingInput;
    pthread_t mTrackingThread{};
    bool mTrackingThreadStarted{false};
};

} // namespace ORB_SLAM3

#endif // SLAM_SESSION_H
