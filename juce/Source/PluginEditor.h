#pragma once

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
    std::unique_ptr<SpatialCompass> compass_;
    std::unique_ptr<ElevationStrip> elevation_;

    juce::Slider gainSlider_;
    juce::Label  gainLabel_;
    using SliderAttachment = juce::AudioProcessorValueTreeState::SliderAttachment;
    std::unique_ptr<SliderAttachment> gainAttachment_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(SpatialAudioEditor)
};
