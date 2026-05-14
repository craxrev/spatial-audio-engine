#pragma once

#include <array>

#include <juce_audio_processors/juce_audio_processors.h>

#include "PluginProcessor.h"

class SpatialCompass;
class ElevationStrip;

class SpatialAudioEditor : public juce::AudioProcessorEditor
{
public:
    explicit SpatialAudioEditor(SpatialAudioProcessor&);
    ~SpatialAudioEditor() override;

    void paint(juce::Graphics&) override;
    void resized() override;

private:
    void resetAllParams();

    SpatialAudioProcessor& proc_;

    std::unique_ptr<SpatialCompass> compass_;
    std::unique_ptr<ElevationStrip> elevation_;

    juce::Slider     gainSlider_;
    juce::Label      gainLabel_;
    juce::TextButton resetButton_ { "Reset" };
    using SliderAttachment = juce::AudioProcessorValueTreeState::SliderAttachment;
    std::unique_ptr<SliderAttachment> gainAttachment_;

    // M6: directivity/occlusion/source-orientation/direct-path row.
    static constexpr int kM6Count = 9;
    std::array<juce::Slider, kM6Count> m6Sliders_;
    std::array<juce::Label,  kM6Count> m6Labels_;
    std::array<std::unique_ptr<SliderAttachment>, kM6Count> m6Attach_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(SpatialAudioEditor)
};
