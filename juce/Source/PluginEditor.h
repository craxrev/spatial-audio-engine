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
    void resetAllParams();

    SpatialAudioProcessor& proc_;

    juce::TooltipWindow tooltipWindow_ { this, 600 };

    std::unique_ptr<SpatialCompass> compass_;
    std::unique_ptr<ElevationStrip> elevation_;

    juce::Slider     gainSlider_;
    juce::Label      gainLabel_;
    juce::Slider     occlusionSlider_;
    juce::Label      occlusionLabel_;
    juce::Slider     spreadSlider_;
    juce::Label      spreadLabel_;
    juce::Slider     focusSlider_;
    juce::Label      focusLabel_;
    juce::Slider     offGainSlider_;
    juce::Label      offGainLabel_;
    juce::Slider     offLpSlider_;
    juce::Label      offLpLabel_;
    juce::Slider     directPathSlider_;
    juce::Label      directPathLabel_;
    juce::Slider     reverbSendSlider_;
    juce::Label      reverbSendLabel_;
    juce::Slider     reverbAmountSlider_;
    juce::Label      reverbAmountLabel_;
    juce::TextButton resetButton_ { "Reset" };
    juce::ToggleButton aimAtListenerButton_ { "Aim at listener" };

    using SliderAttachment = juce::AudioProcessorValueTreeState::SliderAttachment;
    using ButtonAttachment = juce::AudioProcessorValueTreeState::ButtonAttachment;
    std::unique_ptr<SliderAttachment> gainAttachment_;
    std::unique_ptr<SliderAttachment> occlusionAttachment_;
    std::unique_ptr<SliderAttachment> spreadAttachment_;
    std::unique_ptr<SliderAttachment> focusAttachment_;
    std::unique_ptr<SliderAttachment> offGainAttachment_;
    std::unique_ptr<SliderAttachment> offLpAttachment_;
    std::unique_ptr<SliderAttachment> directPathAttachment_;
    std::unique_ptr<SliderAttachment> reverbSendAttachment_;
    std::unique_ptr<SliderAttachment> reverbAmountAttachment_;
    std::unique_ptr<ButtonAttachment> aimAttachment_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(SpatialAudioEditor)
};
