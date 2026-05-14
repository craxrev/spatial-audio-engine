#include "PluginProcessor.h"
#include "PluginEditor.h"

#include <cmath>

#include "BinaryData.h"

namespace
{
// Spherical (distance/azimuth/elevation) → native-Cartesian
// (+X forward, +Y left, +Z up). Azimuth: 0 = front, +90 = left.
// Elevation: 0 = horizontal, +90 = up.
inline void sphericalToNative(float dist, float azDeg, float elDeg,
                              float& x, float& y, float& z)
{
    const float az = juce::degreesToRadians(azDeg);
    const float el = juce::degreesToRadians(elDeg);
    const float ce = std::cos(el);
    x = dist * ce * std::cos(az);
    y = dist * ce * std::sin(az);
    z = dist * std::sin(el);
}

// Tait-Bryan ZYX (yaw-pitch-roll) → unit quaternion (w, x, y, z).
// Yaw rotates around native +Z (up), pitch around +Y (left),
// roll around +X (forward).
inline void eulerToQuat(float yawDeg, float pitchDeg, float rollDeg,
                        float& w, float& x, float& y, float& z)
{
    const float y_ = juce::degreesToRadians(yawDeg)   * 0.5f;
    const float p_ = juce::degreesToRadians(pitchDeg) * 0.5f;
    const float r_ = juce::degreesToRadians(rollDeg)  * 0.5f;
    const float cy = std::cos(y_), sy = std::sin(y_);
    const float cp = std::cos(p_), sp = std::sin(p_);
    const float cr = std::cos(r_), sr = std::sin(r_);
    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;
}
// Short cardinal-direction hint for an azimuth in degrees.
inline const char* azimuthCardinal(float deg)
{
    if (deg >= -22.5f  && deg <=  22.5f)  return "front";
    if (deg >   22.5f  && deg <   67.5f)  return "front-left";
    if (deg >=  67.5f  && deg <= 112.5f)  return "left";
    if (deg >  112.5f  && deg <  157.5f)  return "back-left";
    if (deg >= 157.5f  || deg <= -157.5f) return "back";
    if (deg >  -157.5f && deg <  -112.5f) return "back-right";
    if (deg >= -112.5f && deg <=  -67.5f) return "right";
    return "front-right"; // -67.5 .. -22.5
}

inline const char* elevationCardinal(float deg)
{
    if (deg >=  10.0f) return "up";
    if (deg <= -10.0f) return "down";
    return "horizon";
}
} // namespace

juce::AudioProcessorValueTreeState::ParameterLayout
SpatialAudioProcessor::makeParameterLayout()
{
    using P     = juce::AudioParameterFloat;
    using Attrs = juce::AudioParameterFloatAttributes;
    using R     = juce::NormalisableRange<float>;
    juce::AudioProcessorValueTreeState::ParameterLayout layout;

    auto fmtMeters = [](float v, int) { return juce::String(v, 2) + " m"; };
    auto fmtDb     = [](float v, int) {
        return v <= -79.9f ? juce::String("-inf dB") : juce::String(v, 1) + " dB";
    };
    auto fmtAzim = [](float v, int) {
        return juce::String(v, 1) + juce::String::fromUTF8("\xc2\xb0 (")
             + azimuthCardinal(v) + ")";
    };
    auto fmtElev = [](float v, int) {
        return juce::String(v, 1) + juce::String::fromUTF8("\xc2\xb0 (")
             + elevationCardinal(v) + ")";
    };
    auto fmtDeg = [](float v, int) {
        return juce::String(v, 1) + juce::String::fromUTF8("\xc2\xb0");
    };

    layout.add(std::make_unique<P>(juce::ParameterID{"distance",   1}, "Distance",
                                    R{0.0f, 50.0f, 0.001f},  5.0f,
                                    Attrs().withStringFromValueFunction(fmtMeters)));
    layout.add(std::make_unique<P>(juce::ParameterID{"azimuth",    1}, "Azimuth",
                                    R{-180.0f, 180.0f, 0.1f}, 0.0f,
                                    Attrs().withStringFromValueFunction(fmtAzim)));
    layout.add(std::make_unique<P>(juce::ParameterID{"elevation",  1}, "Elevation",
                                    R{-90.0f, 90.0f, 0.1f},   0.0f,
                                    Attrs().withStringFromValueFunction(fmtElev)));
    layout.add(std::make_unique<P>(juce::ParameterID{"gain_db",    1}, "Gain",
                                    R{-80.0f, 12.0f, 0.1f},   0.0f,
                                    Attrs().withStringFromValueFunction(fmtDb)));
    layout.add(std::make_unique<P>(juce::ParameterID{"listener_x", 1}, "Listener X",
                                    R{-50.0f, 50.0f, 0.01f},  0.0f,
                                    Attrs().withStringFromValueFunction(fmtMeters)));
    layout.add(std::make_unique<P>(juce::ParameterID{"listener_y", 1}, "Listener Y",
                                    R{-50.0f, 50.0f, 0.01f},  0.0f,
                                    Attrs().withStringFromValueFunction(fmtMeters)));
    layout.add(std::make_unique<P>(juce::ParameterID{"listener_z", 1}, "Listener Z",
                                    R{-50.0f, 50.0f, 0.01f},  0.0f,
                                    Attrs().withStringFromValueFunction(fmtMeters)));
    layout.add(std::make_unique<P>(juce::ParameterID{"yaw",        1}, "Yaw",
                                    R{-180.0f, 180.0f, 0.1f}, 0.0f,
                                    Attrs().withStringFromValueFunction(fmtDeg)));
    layout.add(std::make_unique<P>(juce::ParameterID{"pitch",      1}, "Pitch",
                                    R{-90.0f, 90.0f, 0.1f},   0.0f,
                                    Attrs().withStringFromValueFunction(fmtDeg)));
    layout.add(std::make_unique<P>(juce::ParameterID{"roll",       1}, "Roll",
                                    R{-180.0f, 180.0f, 0.1f}, 0.0f,
                                    Attrs().withStringFromValueFunction(fmtDeg)));

    auto fmtUnit  = [](float v, int) { return juce::String(v, 2); };
    auto fmtGain  = [](float v, int) { return juce::String(v, 2) + "x"; };

    layout.add(std::make_unique<P>(juce::ParameterID{"source_yaw",   1}, "Src Yaw",
                                    R{-180.0f, 180.0f, 0.1f}, 0.0f,
                                    Attrs().withStringFromValueFunction(fmtDeg)));
    layout.add(std::make_unique<P>(juce::ParameterID{"source_pitch", 1}, "Src Pitch",
                                    R{-90.0f, 90.0f, 0.1f},   0.0f,
                                    Attrs().withStringFromValueFunction(fmtDeg)));
    layout.add(std::make_unique<P>(juce::ParameterID{"source_roll",  1}, "Src Roll",
                                    R{-180.0f, 180.0f, 0.1f}, 0.0f,
                                    Attrs().withStringFromValueFunction(fmtDeg)));
    layout.add(std::make_unique<P>(juce::ParameterID{"occlusion",    1}, "Occlusion",
                                    R{0.0f, 1.0f, 0.001f},    0.0f,
                                    Attrs().withStringFromValueFunction(fmtUnit)));
    // Cone defaults: inner=0°, outer=360°, outerGain=1, outerLP=0 (cone off).
    layout.add(std::make_unique<P>(juce::ParameterID{"dir_inner_deg",1}, "Dir Inner",
                                    R{0.0f, 360.0f, 0.1f},    0.0f,
                                    Attrs().withStringFromValueFunction(fmtDeg)));
    layout.add(std::make_unique<P>(juce::ParameterID{"dir_outer_deg",1}, "Dir Outer",
                                    R{0.0f, 360.0f, 0.1f},  360.0f,
                                    Attrs().withStringFromValueFunction(fmtDeg)));
    layout.add(std::make_unique<P>(juce::ParameterID{"dir_outer_gain",1}, "Dir Outer Gain",
                                    R{0.0f, 1.0f, 0.001f},    1.0f,
                                    Attrs().withStringFromValueFunction(fmtUnit)));
    layout.add(std::make_unique<P>(juce::ParameterID{"dir_outer_lp", 1}, "Dir Outer LP",
                                    R{0.0f, 1.0f, 0.001f},    0.0f,
                                    Attrs().withStringFromValueFunction(fmtUnit)));
    layout.add(std::make_unique<P>(juce::ParameterID{"direct_path_gain",1}, "Direct Path",
                                    R{0.0f, 2.0f, 0.001f},    1.0f,
                                    Attrs().withStringFromValueFunction(fmtGain)));

    return layout;
}

SpatialAudioProcessor::SpatialAudioProcessor()
    : AudioProcessor(BusesProperties()
        .withInput("Input",  juce::AudioChannelSet::stereo(), true)
        .withOutput("Output", juce::AudioChannelSet::stereo(), true)),
      apvts(*this, nullptr, "params", makeParameterLayout()),
      inMonoRing_(RING_CAP, 0.0f),
      outLRing_(RING_CAP, 0.0f),
      outRRing_(RING_CAP, 0.0f)
{
    pDist_      = apvts.getRawParameterValue("distance");
    pAzim_      = apvts.getRawParameterValue("azimuth");
    pElev_      = apvts.getRawParameterValue("elevation");
    pGainDb_    = apvts.getRawParameterValue("gain_db");
    pListenerX_ = apvts.getRawParameterValue("listener_x");
    pListenerY_ = apvts.getRawParameterValue("listener_y");
    pListenerZ_ = apvts.getRawParameterValue("listener_z");
    pYaw_       = apvts.getRawParameterValue("yaw");
    pPitch_     = apvts.getRawParameterValue("pitch");
    pRoll_      = apvts.getRawParameterValue("roll");
    pSrcYaw_    = apvts.getRawParameterValue("source_yaw");
    pSrcPitch_  = apvts.getRawParameterValue("source_pitch");
    pSrcRoll_   = apvts.getRawParameterValue("source_roll");
    pOcclusion_ = apvts.getRawParameterValue("occlusion");
    pDirInner_  = apvts.getRawParameterValue("dir_inner_deg");
    pDirOuter_  = apvts.getRawParameterValue("dir_outer_deg");
    pDirGain_   = apvts.getRawParameterValue("dir_outer_gain");
    pDirLp_     = apvts.getRawParameterValue("dir_outer_lp");
    pDpGain_    = apvts.getRawParameterValue("direct_path_gain");

    setLatencySamples(ENGINE_BLOCK);
}

SpatialAudioProcessor::~SpatialAudioProcessor()
{
    if (engine_ != nullptr)
    {
        engine_destroy(engine_);
        engine_ = nullptr;
    }
}

void SpatialAudioProcessor::prepareToPlay(double sampleRate, int /*samplesPerBlock*/)
{
    if (engine_ != nullptr)
    {
        engine_destroy(engine_);
        engine_ = nullptr;
    }
    engine_ = engine_new(static_cast<uint32_t>(sampleRate), 1);
    if (engine_ == nullptr) return;

    hrtfLoaded_ = engine_load_main_hrtf(
        engine_,
        reinterpret_cast<const uint8_t*>(SpatialAudioBinary::hrtf_decoder_native_bin),
        static_cast<size_t>(SpatialAudioBinary::hrtf_decoder_native_binSize));

    engine_set_source_active(engine_, 0, true);

    // Reset rings; prime output with one engine-block of zeros to
    // cover the chunker's 128-sample latency.
    std::fill(inMonoRing_.begin(),  inMonoRing_.end(),  0.0f);
    std::fill(outLRing_.begin(),    outLRing_.end(),    0.0f);
    std::fill(outRRing_.begin(),    outRRing_.end(),    0.0f);
    inWrite_ = inRead_ = 0;
    outRead_ = 0;
    outWrite_ = ENGINE_BLOCK;
}

void SpatialAudioProcessor::releaseResources()
{
    if (engine_ != nullptr)
    {
        engine_destroy(engine_);
        engine_ = nullptr;
    }
}

bool SpatialAudioProcessor::isBusesLayoutSupported(const BusesLayout& layouts) const
{
    return layouts.getMainOutputChannels() == 2
        && layouts.getMainInputChannels()  <= 2;
}

void SpatialAudioProcessor::applyParametersToEngine()
{
    if (engine_ == nullptr) return;

    float sx, sy, sz;
    sphericalToNative(pDist_->load(), pAzim_->load(), pElev_->load(), sx, sy, sz);
    engine_set_source_position(engine_, 0, sx, sy, sz);

    const float gainLin = std::pow(10.0f, pGainDb_->load() * 0.05f);
    engine_set_source_gain(engine_, 0, gainLin);

    engine_set_listener_position(engine_,
        pListenerX_->load(), pListenerY_->load(), pListenerZ_->load());

    float qw, qx, qy, qz;
    eulerToQuat(pYaw_->load(), pPitch_->load(), pRoll_->load(), qw, qx, qy, qz);
    engine_set_listener_rotation(engine_, qw, qx, qy, qz);

    float sqw, sqx, sqy, sqz;
    eulerToQuat(pSrcYaw_->load(), pSrcPitch_->load(), pSrcRoll_->load(),
                sqw, sqx, sqy, sqz);
    engine_set_source_rotation(engine_, 0, sqw, sqx, sqy, sqz);

    engine_set_source_direct_path_gain(engine_, 0, pDpGain_->load());
    engine_set_source_occlusion(engine_, 0, pOcclusion_->load());
    engine_set_source_directivity(
        engine_, 0,
        juce::degreesToRadians(pDirInner_->load()),
        juce::degreesToRadians(pDirOuter_->load()),
        pDirGain_->load(),
        pDirLp_->load());
}

void SpatialAudioProcessor::processOneEngineBlock()
{
    float block[ENGINE_BLOCK];
    for (int i = 0; i < ENGINE_BLOCK; ++i)
    {
        block[i] = inMonoRing_[(size_t) inRead_];
        inRead_ = (inRead_ + 1) % RING_CAP;
    }

    float outL[ENGINE_BLOCK], outR[ENGINE_BLOCK];
    engine_process_block(engine_, block, 1, outL, outR);

    for (int i = 0; i < ENGINE_BLOCK; ++i)
    {
        outLRing_[(size_t) outWrite_] = outL[i];
        outRRing_[(size_t) outWrite_] = outR[i];
        outWrite_ = (outWrite_ + 1) % RING_CAP;
    }
}

void SpatialAudioProcessor::processBlock(juce::AudioBuffer<float>& buffer, juce::MidiBuffer&)
{
    juce::ScopedNoDenormals nd;

    if (engine_ == nullptr || !hrtfLoaded_)
    {
        buffer.clear();
        return;
    }

    applyParametersToEngine();

    const int n = buffer.getNumSamples();
    const int numIn = juce::jmin(buffer.getNumChannels(), 2);
    const float* inL = buffer.getReadPointer(0);
    const float* inR = numIn >= 2 ? buffer.getReadPointer(1) : nullptr;

    // Fold to mono and push into the input ring.
    for (int i = 0; i < n; ++i)
    {
        const float mono = inR != nullptr ? 0.5f * (inL[i] + inR[i]) : inL[i];
        inMonoRing_[(size_t) inWrite_] = mono;
        inWrite_ = (inWrite_ + 1) % RING_CAP;
    }

    // Drain whole engine blocks from the input ring into the output ring.
    auto inFill = [this]() {
        return (inWrite_ - inRead_ + RING_CAP) % RING_CAP;
    };
    while (inFill() >= ENGINE_BLOCK)
        processOneEngineBlock();

    // Pop n samples from the output ring into the host buffer.
    float* outL = buffer.getWritePointer(0);
    float* outR = buffer.getNumChannels() > 1 ? buffer.getWritePointer(1) : nullptr;
    for (int i = 0; i < n; ++i)
    {
        outL[i] = outLRing_[(size_t) outRead_];
        if (outR != nullptr) outR[i] = outRRing_[(size_t) outRead_];
        outRead_ = (outRead_ + 1) % RING_CAP;
    }
}

void SpatialAudioProcessor::getStateInformation(juce::MemoryBlock& dest)
{
    if (auto state = apvts.copyState(); state.isValid())
    {
        if (auto xml = state.createXml())
            copyXmlToBinary(*xml, dest);
    }
}

void SpatialAudioProcessor::setStateInformation(const void* data, int sizeInBytes)
{
    if (auto xml = getXmlFromBinary(data, sizeInBytes))
    {
        apvts.replaceState(juce::ValueTree::fromXml(*xml));
    }
}

juce::AudioProcessorEditor* SpatialAudioProcessor::createEditor()
{
    return new SpatialAudioEditor(*this);
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new SpatialAudioProcessor();
}
