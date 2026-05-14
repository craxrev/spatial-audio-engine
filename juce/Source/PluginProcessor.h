#pragma once

#include <atomic>
#include <vector>

#include <juce_audio_processors/juce_audio_processors.h>

extern "C" {
#include "engine.h"
}

class SpatialAudioProcessor : public juce::AudioProcessor
{
public:
    SpatialAudioProcessor();
    ~SpatialAudioProcessor() override;

    void prepareToPlay(double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;
    void processBlock(juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override { return true; }

    const juce::String getName() const override { return JucePlugin_Name; }

    bool acceptsMidi() const override { return false; }
    bool producesMidi() const override { return false; }
    bool isMidiEffect() const override { return false; }
    double getTailLengthSeconds() const override { return 0.0; }

    int getNumPrograms() override { return 1; }
    int getCurrentProgram() override { return 0; }
    void setCurrentProgram(int) override {}
    const juce::String getProgramName(int) override { return {}; }
    void changeProgramName(int, const juce::String&) override {}

    void getStateInformation(juce::MemoryBlock&) override;
    void setStateInformation(const void*, int) override;

    bool isBusesLayoutSupported(const BusesLayout& layouts) const override;

    juce::AudioProcessorValueTreeState apvts;

private:
    static juce::AudioProcessorValueTreeState::ParameterLayout makeParameterLayout();

    void applyParametersToEngine();
    void processOneEngineBlock();

    Engine* engine_ = nullptr;
    bool hrtfLoaded_ = false;

    // Fixed-128-sample chunker between host's variable block size
    // and the engine's quantum. ENGINE_BLOCK is hardcoded; engine
    // reports it via engine_block_size().
    static constexpr int ENGINE_BLOCK = 128;
    static constexpr int RING_CAP = 8192;
    std::vector<float> inMonoRing_;
    std::vector<float> outLRing_;
    std::vector<float> outRRing_;
    int inWrite_ = 0, inRead_ = 0;
    int outWrite_ = 0, outRead_ = 0;

    // Cached atomic param pointers (filled in ctor from apvts).
    std::atomic<float>* pDist_      = nullptr;
    std::atomic<float>* pAzim_      = nullptr;
    std::atomic<float>* pElev_      = nullptr;
    std::atomic<float>* pGainDb_    = nullptr;
    std::atomic<float>* pListenerX_ = nullptr;
    std::atomic<float>* pListenerY_ = nullptr;
    std::atomic<float>* pListenerZ_ = nullptr;
    std::atomic<float>* pYaw_       = nullptr;
    std::atomic<float>* pPitch_     = nullptr;
    std::atomic<float>* pRoll_      = nullptr;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(SpatialAudioProcessor)
};
