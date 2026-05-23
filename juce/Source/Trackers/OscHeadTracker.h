#pragma once

#include <atomic>

#include <juce_osc/juce_osc.h>

#include "../HeadTracker.h"

// Receives /headpose w x y z from buds_daemon over UDP localhost. The daemon
// is what actually talks to the buds via RFCOMM — the AU plugin can't, since
// it inherits its host's (DAW's) Bluetooth permission, which is normally
// absent. See juce/tools/buds_daemon/.
// Process-shared: hosts like SoundSource may instantiate the AU more than
// once inside the same process, and only one of those instances would
// successfully bind the OSC port — the others would silently receive
// nothing. shared() returns the single backing tracker so every plugin
// instance reads the same pose.
class OscHeadTracker : public HeadTracker,
                      private juce::OSCReceiver::ListenerWithOSCAddress<
                          juce::OSCReceiver::MessageLoopCallback>
{
public:
    static OscHeadTracker& shared();

    void   start() override;     // idempotent across instances
    void   stop()  override;     // no-op once started — kept for interface
    Status status() const noexcept override { return status_.load(); }
    Quat   latestPose(uint64_t* frameId = nullptr) const noexcept override;

private:
    OscHeadTracker(int port = 9000);
    ~OscHeadTracker() override;

    void oscMessageReceived(const juce::OSCMessage&) override;
    void notify(Status s);
    void watchdogTick();

    int                  port_;
    juce::OSCReceiver    receiver_;
    std::atomic<bool>    started_ { false };
    std::atomic<Status>  status_ { Status::Disconnected };
    std::atomic<uint64_t> frameId_ { 0 };
    std::atomic<float> qw_ { 1.0f }, qx_ { 0.0f }, qy_ { 0.0f }, qz_ { 0.0f };
    std::atomic<int64_t> lastRxMs_ { 0 };

    struct Watchdog : juce::Timer {
        OscHeadTracker* owner;
        void timerCallback() override { owner->watchdogTick(); }
    } watchdog_;
};
