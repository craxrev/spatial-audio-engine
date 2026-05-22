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
    // Angular spread between the linked L and R virtual sources.
    // 0° = collapsed to centre (mono-ish); 60° = ITU stereo;
    // 180° = hard left/right ear positions.
    layout.add(std::make_unique<P>(juce::ParameterID{"width",      1}, "Width",
                                    R{0.0f, 180.0f, 0.1f},   60.0f,
                                    Attrs().withStringFromValueFunction(fmtDeg)));
    layout.add(std::make_unique<P>(juce::ParameterID{"gain_db",    1}, "Gain",
                                    R{-80.0f, 12.0f, 0.1f},   0.0f,
                                    Attrs().withStringFromValueFunction(fmtDb)));
    // Look-target (world-locked Cartesian point both sources aim at when
    // Aim-at-listener is OFF). Default (0,0,0) = listener origin, so
    // unlocked mode initially behaves identically to locked until the
    // user drags the arrow.
    layout.add(std::make_unique<P>(juce::ParameterID{"target_x", 1}, "Target X",
                                    R{-50.0f, 50.0f, 0.01f},  0.0f,
                                    Attrs().withStringFromValueFunction(fmtMeters)));
    layout.add(std::make_unique<P>(juce::ParameterID{"target_y", 1}, "Target Y",
                                    R{-50.0f, 50.0f, 0.01f},  0.0f,
                                    Attrs().withStringFromValueFunction(fmtMeters)));
    layout.add(std::make_unique<P>(juce::ParameterID{"target_z", 1}, "Target Z",
                                    R{-50.0f, 50.0f, 0.01f},  0.0f,
                                    Attrs().withStringFromValueFunction(fmtMeters)));
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
    layout.add(std::make_unique<P>(juce::ParameterID{"occlusion",    1}, "Occlusion",
                                    R{0.0f, 1.0f, 0.001f},    0.0f,
                                    Attrs().withStringFromValueFunction(fmtUnit)));
    // Cone defaults engaged so every directivity control is audible
    // out of the box. inner=30°, outer=120°, outer_gain=0.5, outer_lp=0.3.
    // Both angles are measured from the source's forward axis (§6.2):
    // anything ≥ 180° is equivalent to "fully open", so cap there.
    layout.add(std::make_unique<P>(juce::ParameterID{"dir_inner_deg",1}, "Dir Inner",
                                    R{0.0f, 180.0f, 0.1f},   30.0f,
                                    Attrs().withStringFromValueFunction(fmtDeg)));
    layout.add(std::make_unique<P>(juce::ParameterID{"dir_outer_deg",1}, "Dir Outer",
                                    R{0.0f, 180.0f, 0.1f},  120.0f,
                                    Attrs().withStringFromValueFunction(fmtDeg)));
    layout.add(std::make_unique<P>(juce::ParameterID{"dir_outer_gain",1}, "Dir Outer Gain",
                                    R{0.0f, 1.0f, 0.001f},    0.5f,
                                    Attrs().withStringFromValueFunction(fmtUnit)));
    layout.add(std::make_unique<P>(juce::ParameterID{"dir_outer_lp", 1}, "Dir Outer LP",
                                    R{0.0f, 1.0f, 0.001f},    0.3f,
                                    Attrs().withStringFromValueFunction(fmtUnit)));
    layout.add(std::make_unique<P>(juce::ParameterID{"direct_path_gain",1}, "Direct Path",
                                    R{0.0f, 2.0f, 0.001f},    1.0f,
                                    Attrs().withStringFromValueFunction(fmtGain)));

    layout.add(std::make_unique<P>(juce::ParameterID{"reverb_send",  1}, "Reverb Send",
                                    R{0.0f, 1.0f, 0.001f},    0.3f,
                                    Attrs().withStringFromValueFunction(fmtUnit)));
    layout.add(std::make_unique<P>(juce::ParameterID{"reverb_amount", 1}, "Reverb Amount",
                                    R{0.0f, 2.0f, 0.001f},    0.0f,
                                    Attrs().withStringFromValueFunction(fmtGain)));

    layout.add(std::make_unique<P>(juce::ParameterID{"externalizer_amount", 1}, "Externalizer Amount",
                                    R{0.0f, 100.0f, 0.1f},    0.0f,
                                    Attrs().withStringFromValueFunction(fmtUnit)));
    layout.add(std::make_unique<P>(juce::ParameterID{"externalizer_character", 1}, "Externalizer Character",
                                    R{0.0f, 100.0f, 0.1f},   50.0f,
                                    Attrs().withStringFromValueFunction(fmtUnit)));

    // §3 4-knot distance curve. Defaults = bit-verified default values
    // (1 m, 0 dB) (12 m, −20 dB) (60 m, −60 dB) (100 m → 0).
    layout.add(std::make_unique<P>(juce::ParameterID{"dist_a",   1}, "Curve A dist",
                                    R{0.0f, 100.0f, 0.01f},    1.0f,
                                    Attrs().withStringFromValueFunction(fmtMeters)));
    layout.add(std::make_unique<P>(juce::ParameterID{"dist_a_db",1}, "Curve A dB",
                                    R{-80.0f, 12.0f, 0.1f},    0.0f,
                                    Attrs().withStringFromValueFunction(fmtDb)));
    layout.add(std::make_unique<P>(juce::ParameterID{"dist_b",   1}, "Curve B dist",
                                    R{0.0f, 200.0f, 0.01f},   12.0f,
                                    Attrs().withStringFromValueFunction(fmtMeters)));
    layout.add(std::make_unique<P>(juce::ParameterID{"dist_b_db",1}, "Curve B dB",
                                    R{-80.0f, 12.0f, 0.1f},  -20.0f,
                                    Attrs().withStringFromValueFunction(fmtDb)));
    layout.add(std::make_unique<P>(juce::ParameterID{"dist_c",   1}, "Curve C dist",
                                    R{0.0f, 200.0f, 0.01f},   60.0f,
                                    Attrs().withStringFromValueFunction(fmtMeters)));
    layout.add(std::make_unique<P>(juce::ParameterID{"dist_c_db",1}, "Curve C dB",
                                    R{-80.0f, 12.0f, 0.1f},  -60.0f,
                                    Attrs().withStringFromValueFunction(fmtDb)));
    layout.add(std::make_unique<P>(juce::ParameterID{"dist_d",   1}, "Curve D dist",
                                    R{0.0f, 300.0f, 0.01f},  100.0f,
                                    Attrs().withStringFromValueFunction(fmtMeters)));

    // §2.4 / §2.5: source position + rendering modes. position_mode
    // stays as a choice so the future 3D view (Phase 5) can repurpose
    // it; rendering_mode is binary so a checkbox is plenty.
    layout.add(std::make_unique<juce::AudioParameterChoice>(
        juce::ParameterID{"position_mode", 1}, "Position mode",
        juce::StringArray{"World", "Relative (head-locked)"}, 0));
    layout.add(std::make_unique<juce::AudioParameterBool>(
        juce::ParameterID{"rendering_mode", 1}, "Stereo bypass", false));

    layout.add(std::make_unique<juce::AudioParameterBool>(
        juce::ParameterID{"aim_at_listener", 1}, "Aim at listener", true));

    return layout;
}

SpatialAudioProcessor::SpatialAudioProcessor()
    : AudioProcessor(BusesProperties()
        .withInput("Input",  juce::AudioChannelSet::stereo(), true)
        .withOutput("Output", juce::AudioChannelSet::stereo(), true)),
      apvts(*this, nullptr, "params", makeParameterLayout()),
      inLRing_(RING_CAP, 0.0f),
      inRRing_(RING_CAP, 0.0f),
      outLRing_(RING_CAP, 0.0f),
      outRRing_(RING_CAP, 0.0f)
{
    pDist_      = apvts.getRawParameterValue("distance");
    pAzim_      = apvts.getRawParameterValue("azimuth");
    pElev_      = apvts.getRawParameterValue("elevation");
    pWidth_     = apvts.getRawParameterValue("width");
    pGainDb_    = apvts.getRawParameterValue("gain_db");
    pTargetX_   = apvts.getRawParameterValue("target_x");
    pTargetY_   = apvts.getRawParameterValue("target_y");
    pTargetZ_   = apvts.getRawParameterValue("target_z");
    pListenerX_ = apvts.getRawParameterValue("listener_x");
    pListenerY_ = apvts.getRawParameterValue("listener_y");
    pListenerZ_ = apvts.getRawParameterValue("listener_z");
    pYaw_       = apvts.getRawParameterValue("yaw");
    pPitch_     = apvts.getRawParameterValue("pitch");
    pRoll_      = apvts.getRawParameterValue("roll");
    pSrcYaw_    = apvts.getRawParameterValue("source_yaw");
    pSrcPitch_  = apvts.getRawParameterValue("source_pitch");
    pOcclusion_ = apvts.getRawParameterValue("occlusion");
    pDirInner_  = apvts.getRawParameterValue("dir_inner_deg");
    pDirOuter_  = apvts.getRawParameterValue("dir_outer_deg");
    pDirGain_   = apvts.getRawParameterValue("dir_outer_gain");
    pDirLp_     = apvts.getRawParameterValue("dir_outer_lp");
    pDpGain_    = apvts.getRawParameterValue("direct_path_gain");
    pRevSend_   = apvts.getRawParameterValue("reverb_send");
    pRevAmount_ = apvts.getRawParameterValue("reverb_amount");
    pExtAmount_ = apvts.getRawParameterValue("externalizer_amount");
    pExtChar_   = apvts.getRawParameterValue("externalizer_character");
    pDistA_     = apvts.getRawParameterValue("dist_a");
    pDistAdB_   = apvts.getRawParameterValue("dist_a_db");
    pDistB_     = apvts.getRawParameterValue("dist_b");
    pDistBdB_   = apvts.getRawParameterValue("dist_b_db");
    pDistC_     = apvts.getRawParameterValue("dist_c");
    pDistCdB_   = apvts.getRawParameterValue("dist_c_db");
    pDistD_     = apvts.getRawParameterValue("dist_d");
    pPosMode_   = apvts.getRawParameterValue("position_mode");
    pRenderMode_= apvts.getRawParameterValue("rendering_mode");

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
    // Two sources: a linked stereo pair. Source 0 = L virtual speaker,
    // source 1 = R virtual speaker. Their positions are computed each
    // block from (azimuth, elevation, distance, width).
    engine_ = engine_new(static_cast<uint32_t>(sampleRate), 2);
    if (engine_ == nullptr) return;

    hrtfLoaded_ = engine_load_main_hrtf(
        engine_,
        reinterpret_cast<const uint8_t*>(SpatialAudioBinary::hrtf_decoder_native_bin),
        static_cast<size_t>(SpatialAudioBinary::hrtf_decoder_native_binSize));

    // §13 W-channel binauralizer (decoder_post): adds a diffuse-field
    // envelopment layer derived from the W (omni) ambisonic channel.
    engine_load_w_binauralizer(
        engine_,
        reinterpret_cast<const uint8_t*>(SpatialAudioBinary::hrtf_post_filter_a_bin),
        static_cast<size_t>(SpatialAudioBinary::hrtf_post_filter_a_binSize),
        reinterpret_cast<const uint8_t*>(SpatialAudioBinary::hrtf_post_filter_b_bin),
        static_cast<size_t>(SpatialAudioBinary::hrtf_post_filter_b_binSize));

    engine_set_source_active(engine_, 0, true);
    engine_set_source_active(engine_, 1, true);

    // Reset rings; prime output with one engine-block of zeros to
    // cover the chunker's 128-sample latency.
    std::fill(inLRing_.begin(),     inLRing_.end(),     0.0f);
    std::fill(inRRing_.begin(),     inRRing_.end(),     0.0f);
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

    // Linked stereo pair: both sources share elevation + distance and
    // mirror the centre azimuth by ±width/2. In native convention
    // +Y is left, so positive azimuth = left → source 0 (L) takes
    // (azim + width/2) and source 1 (R) takes (azim − width/2).
    const float azim   = pAzim_->load();
    const float elev   = pElev_->load();
    const float dist   = pDist_->load();
    const float half_w = pWidth_->load() * 0.5f;
    const float azimL  = azim + half_w;
    const float azimR  = azim - half_w;

    float lx, ly, lz, rx, ry, rz;
    sphericalToNative(dist, azimL, elev, lx, ly, lz);
    sphericalToNative(dist, azimR, elev, rx, ry, rz);
    engine_set_source_position(engine_, 0, lx, ly, lz);
    engine_set_source_position(engine_, 1, rx, ry, rz);

    const float gainLin = std::pow(10.0f, pGainDb_->load() * 0.05f);
    engine_set_source_gain(engine_, 0, gainLin);
    engine_set_source_gain(engine_, 1, gainLin);

    engine_set_listener_position(engine_,
        pListenerX_->load(), pListenerY_->load(), pListenerZ_->load());

    float qw, qx, qy, qz;
    eulerToQuat(pYaw_->load(), pPitch_->load(), pRoll_->load(), qw, qx, qy, qz);
    engine_set_listener_rotation(engine_, qw, qx, qy, qz);

    // Both sources aim at a single world-locked target point.
    // Aim ON: target = listener position. Aim OFF: target = user-set
    // (target_x, target_y, target_z). Per-source rotation is the
    // shortest-arc quaternion that rotates +X (the source's default
    // forward) to the source→target unit vector.
    const bool  aim = apvts.getRawParameterValue("aim_at_listener")->load() > 0.5f;
    const float tx  = aim ? pListenerX_->load() : pTargetX_->load();
    const float ty  = aim ? pListenerY_->load() : pTargetY_->load();
    const float tz  = aim ? pListenerZ_->load() : pTargetZ_->load();

    auto aimSourceAt = [&](uint32_t idx, float sx, float sy, float sz) {
        const float dx = tx - sx, dy = ty - sy, dz = tz - sz;
        const float len = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 1e-6f)
        {
            engine_set_source_rotation(engine_, idx, 1.0f, 0.0f, 0.0f, 0.0f);
            return;
        }
        const float fx = dx / len, fy = dy / len, fz = dz / len;
        // cos(angle) between (1,0,0) and forward.
        const float cosA = fx;
        if (cosA < -0.99999f)
        {
            // Antiparallel: 180° around +Z (compass-up axis).
            engine_set_source_rotation(engine_, idx, 0.0f, 0.0f, 0.0f, 1.0f);
            return;
        }
        // Shortest-arc quat: (1+cosA, axis=cross((1,0,0), forward)).
        float qw = 1.0f + cosA;
        float qx = 0.0f;
        float qy = -fz;
        float qz = fy;
        const float n = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (n < 1e-9f)
        {
            engine_set_source_rotation(engine_, idx, 1.0f, 0.0f, 0.0f, 0.0f);
            return;
        }
        qw /= n; qx /= n; qy /= n; qz /= n;
        engine_set_source_rotation(engine_, idx, qw, qx, qy, qz);
    };
    aimSourceAt(0, lx, ly, lz);
    aimSourceAt(1, rx, ry, rz);

    const float dp  = pDpGain_->load();
    const float occ = pOcclusion_->load();
    const float dInner = juce::degreesToRadians(pDirInner_->load());
    const float dOuter = juce::degreesToRadians(pDirOuter_->load());
    const float dGain  = pDirGain_->load();
    const float dLp    = pDirLp_->load();
    const float revSend = pRevSend_->load();

    for (uint32_t i = 0; i < 2; ++i)
    {
        engine_set_source_direct_path_gain(engine_, i, dp);
        engine_set_source_occlusion(engine_, i, occ);
        engine_set_source_directivity(engine_, i, dInner, dOuter, dGain, dLp);
        engine_set_source_reverb_send(engine_, i, revSend);
    }

    engine_set_reverb_amount(engine_, pRevAmount_->load());
    engine_set_externalizer_amount(engine_, pExtAmount_->load());
    engine_set_externalizer_character(engine_, pExtChar_->load());

    const float aLin = std::pow(10.0f, pDistAdB_->load() * 0.05f);
    const float bLin = std::pow(10.0f, pDistBdB_->load() * 0.05f);
    const float cLin = std::pow(10.0f, pDistCdB_->load() * 0.05f);
    for (uint32_t i = 0; i < 2; ++i)
    {
        engine_set_source_distance_curve(
            engine_, i,
            pDistA_->load(), aLin,
            pDistB_->load(), bLin,
            pDistC_->load(), cLin,
            pDistD_->load());
    }

    const uint8_t posMode = (uint8_t) juce::roundToInt(pPosMode_->load());
    const uint8_t renMode = (uint8_t) juce::roundToInt(pRenderMode_->load());
    for (uint32_t i = 0; i < 2; ++i)
    {
        engine_set_source_position_mode(engine_, i, posMode);
        engine_set_source_rendering_mode(engine_, i, renMode);
    }
}

void SpatialAudioProcessor::processOneEngineBlock()
{
    // Linked stereo pair: 2 sources × 2 chans × 128 samples, source-major.
    // Source 0 (L virtual speaker) eats the host's L input;
    // source 1 (R virtual speaker) eats the host's R input.
    // Both sources are mono (input_channel_count = 1); ch1 stays zero.
    constexpr int slabPerSource = ENGINE_BLOCK * 2;
    float inputs[slabPerSource * 2] = {};
    for (int i = 0; i < ENGINE_BLOCK; ++i)
    {
        inputs[i]                  = inLRing_[(size_t) inRead_];  // src 0, ch0
        inputs[slabPerSource + i]  = inRRing_[(size_t) inRead_];  // src 1, ch0
        inRead_ = (inRead_ + 1) % RING_CAP;
    }

    float outL[ENGINE_BLOCK], outR[ENGINE_BLOCK];
    engine_process_block(engine_, inputs, 2, nullptr, 0, outL, outR);

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
    // Both linked sources are mono: source 0 reads host L, source 1
    // reads host R. With a mono host input, both sources get the same
    // signal (collapses the pair to a single phantom point).
    if (engine_ != nullptr)
    {
        engine_set_source_input_channel_count(engine_, 0, 1);
        engine_set_source_input_channel_count(engine_, 1, 1);
    }

    for (int i = 0; i < n; ++i)
    {
        inLRing_[(size_t) inWrite_] = inL[i];
        inRRing_[(size_t) inWrite_] = inR != nullptr ? inR[i] : inL[i];
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
