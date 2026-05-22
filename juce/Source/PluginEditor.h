#pragma once

#include <juce_audio_processors/juce_audio_processors.h>

#include "PluginProcessor.h"

class SpatialCompass;
class ElevationStrip;
class DistanceCurveEditor;

class SpatialAudioEditor : public juce::AudioProcessorEditor,
                            private juce::AudioProcessorValueTreeState::Listener
{
public:
    explicit SpatialAudioEditor(SpatialAudioProcessor&);
    ~SpatialAudioEditor() override;

    void paint(juce::Graphics&) override;
    void resized() override;

private:
    void resetAllParams();
    void toggleAdvanced();
    void refreshPresetSelection();
    void parameterChanged(const juce::String& parameterID, float newValue) override;
    void layoutSliderRow(juce::Rectangle<int>& area, int rowH,
                         juce::Label& label, juce::Slider& slider);
    void layoutPairedRow(juce::Rectangle<int>& area, int rowH,
                         juce::Label& l1, juce::Slider& s1,
                         juce::Label& l2, juce::Slider& s2);

    SpatialAudioProcessor& proc_;

    juce::TooltipWindow tooltipWindow_ { this, 600 };

    std::unique_ptr<SpatialCompass>      compass_;
    std::unique_ptr<ElevationStrip>      elevation_;
    std::unique_ptr<DistanceCurveEditor> curveEditor_;

    // Section header labels (small caps, painted between sections).
    juce::Label shapeHeader_       { "shape",       "SHAPE" };
    juce::Label environmentHeader_ { "environment", "ENVIRONMENT" };
    juce::Label outputHeader_      { "output",      "OUTPUT" };
    juce::Label advancedHeader_    { "advanced",    "ADVANCED" };

    juce::Slider     gainSlider_;
    juce::Label      gainLabel_;
    juce::Slider     widthSlider_;
    juce::Label      widthLabel_;
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
    juce::Slider     reverbSendSlider_;
    juce::Label      reverbSendLabel_;
    juce::Slider     reverbAmountSlider_;
    juce::Label      reverbAmountLabel_;
    juce::Slider     extAmountSlider_;
    juce::Label      extAmountLabel_;
    juce::ComboBox   distPresetBox_;
    juce::Label      distPresetLabel_;

    // Advanced (collapsible) controls.
    juce::Slider     directPathSlider_;
    juce::Label      directPathLabel_;
    juce::Slider     extCharSlider_;
    juce::Label      extCharLabel_;

    juce::ToggleButton stereoBypassButton_  { "Stereo bypass" };
    juce::ToggleButton aimAtListenerButton_ { "Aim at listener" };
    juce::TextButton   resetButton_         { "Reset" };
    juce::TextButton   advancedButton_;

    bool advancedOpen_ = false;

    using SliderAttachment   = juce::AudioProcessorValueTreeState::SliderAttachment;
    using ButtonAttachment   = juce::AudioProcessorValueTreeState::ButtonAttachment;
    using ComboBoxAttachment = juce::AudioProcessorValueTreeState::ComboBoxAttachment;
    std::unique_ptr<SliderAttachment> gainAttachment_;
    std::unique_ptr<SliderAttachment> widthAttachment_;
    std::unique_ptr<SliderAttachment> occlusionAttachment_;
    std::unique_ptr<SliderAttachment> spreadAttachment_;
    std::unique_ptr<SliderAttachment> focusAttachment_;
    std::unique_ptr<SliderAttachment> offGainAttachment_;
    std::unique_ptr<SliderAttachment> offLpAttachment_;
    std::unique_ptr<SliderAttachment> directPathAttachment_;
    std::unique_ptr<SliderAttachment> reverbSendAttachment_;
    std::unique_ptr<SliderAttachment> reverbAmountAttachment_;
    std::unique_ptr<SliderAttachment> extAmountAttachment_;
    std::unique_ptr<SliderAttachment> extCharAttachment_;
    std::unique_ptr<ButtonAttachment> stereoBypassAttachment_;
    std::unique_ptr<ButtonAttachment> aimAttachment_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(SpatialAudioEditor)
};
