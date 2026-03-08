#include "orbslam3/trackingFrontend/SlamSession.h"
#include "orbslam3/trackingFrontend/TrackingInput.h"
#include "orbslam3/visualizationUi/VisualizationModel.h"
#include "orbslam3/visualizationUi/Viewer.h"
#include "orbslam3/trackingFrontend/Tracking.h"
#include "orbslam3/visualizationUi/FrameDrawer.h"

#include <iostream>

namespace ORB_SLAM3 {

SlamSession::SlamSession(const Config &config) : mConfig(config) {
    // 1. Create Core
    mpSystem = std::make_unique<System>(
        config.vocFile, config.settingsFile,
        config.sensor, config.initFr, config.sequence);

    // 2. Create TrackingInput (narrow interface)
    mpTrackingInput = std::make_unique<TrackingInput>(mpSystem.get());

    // 3. Create UI if needed
    if (config.useViewer) {
        auto *fd = mpSystem->GetFrameDrawer();
        auto *md = mpSystem->GetMapDrawer();
        auto *tracker = mpSystem->GetTracker();

        mpVisModel = std::make_unique<VisualizationModel>(
            fd, md,
            static_cast<int>(config.sensor),
            tracker->GetImageScale(),
            fd->both,
            &tracker->mResetting);

        mpViewer = std::make_unique<Viewer>(
            mpVisModel.get(), config.settingsFile,
            mpSystem->GetSettings());

        mpViewer->SetEventCallback(
            [this](UiEvent e) { HandleUiEvent(e); });
    }

    mState = State::Running;
}

void SlamSession::StartTracking(
    std::function<void(TrackingInput *, std::function<bool()>)> fn) {

    // Heap-allocate the closure so pthread can use it via void*
    auto *closure = new std::function<void()>(
        [this, fn = std::move(fn)]() {
            fn(mpTrackingInput.get(), [this]() {
                return mState.load() != State::Running;
            });
            RequestStop(StopReason::InputExhausted);
        });

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    // 8 MB stack — matches macOS main-thread default
    pthread_attr_setstacksize(&attr, 8 * 1024 * 1024);

    pthread_create(&mTrackingThread, &attr,
        [](void *arg) -> void * {
            auto *f = static_cast<std::function<void()> *>(arg);
            (*f)();
            delete f;
            return nullptr;
        },
        closure);

    pthread_attr_destroy(&attr);
    mTrackingThreadStarted = true;
}

void SlamSession::RunMainLoop() {
    if (mpViewer) {
        mpViewer->Run();  // Main thread Pangolin loop
        RequestStop(StopReason::WindowClosed);
    }
    if (mTrackingThreadStarted) {
        pthread_join(mTrackingThread, nullptr);
        mTrackingThreadStarted = false;
    }
    DoShutdown();
}

void SlamSession::RequestStop(StopReason reason) {
    State expected = State::Running;
    if (mState.compare_exchange_strong(expected, State::Stopping)) {
        mStopReason = reason;
        if (mpViewer)
            mpViewer->RequestExit();
    }
}

void SlamSession::HandleUiEvent(UiEvent event) {
    switch (event) {
    case UiEvent::StopRequested:
        RequestStop(StopReason::UserRequestedStop);
        break;
    case UiEvent::ResetRequested:
        mpSystem->ResetActiveMap();
        break;
    case UiEvent::LocalizationOn:
        mpSystem->ActivateLocalizationMode();
        break;
    case UiEvent::LocalizationOff:
        mpSystem->DeactivateLocalizationMode();
        break;
    }
}

void SlamSession::DoShutdown() {
    std::call_once(mShutdownOnce, [this]() {
        mpSystem->Shutdown();
        mState = State::Stopped;
    });
}

void SlamSession::SaveTrajectoryTUM(const std::string &f) {
    mpSystem->SaveTrajectoryTUM(f);
}

void SlamSession::SaveTrajectoryEuRoC(const std::string &f) {
    mpSystem->SaveTrajectoryEuRoC(f);
}

void SlamSession::SaveKeyFrameTrajectoryTUM(const std::string &f) {
    mpSystem->SaveKeyFrameTrajectoryTUM(f);
}

void SlamSession::SaveKeyFrameTrajectoryEuRoC(const std::string &f) {
    mpSystem->SaveKeyFrameTrajectoryEuRoC(f);
}

void SlamSession::SaveTrajectoryKITTI(const std::string &f) {
    mpSystem->SaveTrajectoryKITTI(f);
}

void SlamSession::ChangeDataset() {
    mpSystem->ChangeDataset();
}

SlamSession::State SlamSession::GetState() const {
    return mState.load();
}

SlamSession::StopReason SlamSession::GetStopReason() const {
    return mStopReason;
}

SlamSession::~SlamSession() {
    if (mTrackingThreadStarted) {
        pthread_join(mTrackingThread, nullptr);
        mTrackingThreadStarted = false;
    }
    DoShutdown();
}

} // namespace ORB_SLAM3
