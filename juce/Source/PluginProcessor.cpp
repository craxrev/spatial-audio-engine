#include "PluginProcessor.h"

SpatialAudioProcessor::SpatialAudioProcessor()
    : AudioProcessor(BusesProperties()
        .withInput("Input",  juce::AudioChannelSet::stereo(), true)
        .withOutput("Output", juce::AudioChannelSet::stereo(), true))
{
}

SpatialAudioProcessor::~SpatialAudioProcessor() = default;

void SpatialAudioProcessor::prepareToPlay(double /*sampleRate*/, int /*samplesPerBlock*/) {}
void SpatialAudioProcessor::releaseResources() {}

bool SpatialAudioProcessor::isBusesLayoutSupported(const BusesLayout& layouts) const
{
    return layouts.getMainOutputChannels() == 2
        && layouts.getMainInputChannels()  <= 2;
}

void SpatialAudioProcessor::processBlock(juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;
    // M5 step 4: bypass (passthrough). Engine wiring lands in step 6.
}

juce::AudioProcessorEditor* SpatialAudioProcessor::createEditor()
{
    return new juce::GenericAudioProcessorEditor(*this);
}

// AU/VST3/standalone entry point.
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new SpatialAudioProcessor();
}
