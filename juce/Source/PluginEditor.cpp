#include "PluginEditor.h"

#include <cmath>
#include <limits>

namespace
{
constexpr float kCompassMaxMeters = 25.0f;
constexpr float kTwoPi = 6.283185307f;

// Centralised UTF-8 glyph constants. JUCE's String can't be constexpr
// (heap-backed), so these live as namespace-scope `const` instead.
const juce::String kGlyphDeg  = juce::String::fromUTF8("\xC2\xB0");     // °
const juce::String kGlyphMid  = juce::String::fromUTF8("\xC2\xB7");     // ·
const juce::String kGlyphDash = juce::String::fromUTF8("\xE2\x80\x94"); // —
// Larger BLACK DOWN/UP-POINTING TRIANGLE (U+25BC / U+25B2) — better
// JUCE-font coverage than the small variants (U+25BE / U+25B4).
const juce::String kGlyphDown = juce::String::fromUTF8("\xE2\x96\xBC"); // ▼
const juce::String kGlyphUp   = juce::String::fromUTF8("\xE2\x96\xB2"); // ▲

// Cached fonts. Avoid heap-allocating a new juce::Font every paint() call
// — at 30–60 Hz across three custom components this dominates GUI CPU.
const juce::Font& font9()  { static juce::Font f = juce::Font(juce::FontOptions(9.0f));  return f; }
const juce::Font& font10() { static juce::Font f = juce::Font(juce::FontOptions(10.0f)); return f; }
const juce::Font& font11() { static juce::Font f = juce::Font(juce::FontOptions(11.0f)); return f; }

// Native azimuth convention: 0 = front, +90 = left, ±180 = back.
// Screen y-axis points down, so "front" maps to "up" on screen.
inline juce::Point<float> azimDistToScreen(juce::Point<float> centre, float radius,
                                            float azimuthDeg, float distance,
                                            float maxDist)
{
    const float r   = radius * juce::jmin(distance / maxDist, 1.0f);
    const float a   = juce::degreesToRadians(azimuthDeg);
    return centre + juce::Point<float>(-std::sin(a) * r, -std::cos(a) * r);
}

inline void screenToAzimDist(juce::Point<float> centre, float radius, float maxDist,
                             juce::Point<float> screen,
                             float& azimuthDeg, float& distance)
{
    const float dx       = screen.x - centre.x;
    const float dy       = screen.y - centre.y;
    const float pxR      = std::sqrt(dx * dx + dy * dy);
    azimuthDeg           = juce::radiansToDegrees(std::atan2(-dx, -dy));
    distance             = juce::jmin(pxR / radius, 1.0f) * maxDist;
}

// Unit screen vector for a compass-frame angle in degrees (0 = front/up,
// +90 = left, ±180 = back, -90 = right).
inline juce::Point<float> compassDir(float deg)
{
    const float a = juce::degreesToRadians(deg);
    return { -std::sin(a), -std::cos(a) };
}

// Inverse: screen vector (dx, dy) → compass-frame angle in degrees.
inline float compassAngleDeg(juce::Point<float> v)
{
    return juce::radiansToDegrees(std::atan2(-v.x, -v.y));
}

inline float wrap180(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

// Shortest-path lerp for an angle in degrees.
inline float angleLerp(float a, float b, float t)
{
    const float d = wrap180(b - a);
    return a + d * t;
}

// Cmd-held snap helpers. Always snap (no tolerance gate).
inline float snapTo(float v, std::initializer_list<float> targets)
{
    float best = v;
    float dmin = std::numeric_limits<float>::infinity();
    for (float t : targets)
    {
        const float d = std::abs(v - t);
        if (d < dmin) { dmin = d; best = t; }
    }
    return best;
}

// Snap by absolute magnitude, preserve sign.
inline float snapSymmetric(float v, std::initializer_list<float> mags)
{
    const float sign = v >= 0.0f ? 1.0f : -1.0f;
    const float a    = std::abs(v);
    float best = a;
    float dmin = std::numeric_limits<float>::infinity();
    for (float m : mags)
    {
        const float d = std::abs(a - m);
        if (d < dmin) { dmin = d; best = m; }
    }
    return sign * best;
}

// Short cardinal-direction hint.
inline const char* azimuthCardinal(float deg)
{
    if (deg >= -22.5f  && deg <=  22.5f)  return "front";
    if (deg >   22.5f  && deg <   67.5f)  return "front-left";
    if (deg >=  67.5f  && deg <= 112.5f)  return "left";
    if (deg >  112.5f  && deg <  157.5f)  return "back-left";
    if (deg >= 157.5f  || deg <= -157.5f) return "back";
    if (deg >  -157.5f && deg <  -112.5f) return "back-right";
    if (deg >= -112.5f && deg <=  -67.5f) return "right";
    return "front-right";
}
} // namespace

// ---------------------------------------------------------------------------
// SpatialCompass — top-down view with direct manipulation of source position,
// orientation (yaw), directivity cone, and an occlusion fog overlay.
// ---------------------------------------------------------------------------

class SpatialCompass : public juce::Component,
                       public juce::SettableTooltipClient,
                       private juce::Timer
{
public:
    explicit SpatialCompass(juce::AudioProcessorValueTreeState& s)
        : state_(s)
    {
        setMouseCursor(juce::MouseCursor::PointingHandCursor);
        setBufferedToImage(true);
        startTimerHz(60);
    }

    ~SpatialCompass() override { stopTimer(); }

    void paint(juce::Graphics& g) override
    {
        const auto bounds = getLocalBounds().toFloat();
        const auto centre = bounds.getCentre();
        const float outerR =
            juce::jmin(bounds.getWidth(), bounds.getHeight()) * 0.45f;

        // Cached static background — rings, cardinals, listener head/nose/YOU.
        // Re-rendered only when the component size changes.
        if (backgroundImage_.isNull()
            || backgroundImage_.getWidth()  != juce::jmax(1, getWidth())
            || backgroundImage_.getHeight() != juce::jmax(1, getHeight()))
        {
            backgroundImage_ = juce::Image(juce::Image::ARGB,
                                           juce::jmax(1, getWidth()),
                                           juce::jmax(1, getHeight()),
                                           true);
            juce::Graphics bg(backgroundImage_);
            paintStaticBackground(bg, centre, outerR);
        }
        g.drawImageAt(backgroundImage_, 0, 0);

        // Source.
        const float dist = currentDistance();
        const float az   = currentAzimuth();
        const auto  src  = azimDistToScreen(centre, outerR, az, dist, kCompassMaxMeters);

        // Occlusion fog along listener-source line.
        const float occl = displayedOcclusion_;
        if (occl > 0.001f)
        {
            const auto v = src - centre;
            const float len = v.getDistanceFromOrigin();
            if (len > 1.0f)
            {
                const auto u = v / len; // unit vector along path
                const int kPuffs = 8;
                for (int i = 1; i < kPuffs; ++i)
                {
                    const float t = (float)i / (float)kPuffs;
                    const auto p  = centre + u * (t * len);
                    // Puffs grow with occlusion, fade at endpoints.
                    const float endFade = std::sin(t * juce::MathConstants<float>::pi);
                    const float r = 6.0f + 14.0f * occl * endFade;
                    const float a = 0.06f + 0.32f * occl * endFade;
                    g.setColour(juce::Colour::fromFloatRGBA(0.78f, 0.78f, 0.85f, a));
                    g.fillEllipse(p.x - r, p.y - r, 2.0f * r, 2.0f * r);
                }
            }
        }

        // Audibility contour — real top-down map of where the source is
        // audible, in compass meters. Two nested thresholds: a "loud" zone
        // (−6 dB) and an "audible" outer envelope (−24 dB). Combines
        // gain × direct × directivity_gain(θ) × 1/r.
        const float yaw     = displayedYaw_;
        const float outerLp = state_.getRawParameterValue("dir_outer_lp")->load();
        const juce::Colour warm  (0xffff8c42);
        const juce::Colour slate (0xff5a6478);
        const auto tint = warm.interpolatedWith(slate, juce::jlimit(0.0f, 1.0f, outerLp));

        constexpr float kAudibleThresh = 0.0631f; // −24 dB
        constexpr float kLoudThresh    = 0.5f;    // −6 dB

        g.setColour(tint.withAlpha(0.18f));
        g.fillPath(buildAudibilityContour(src, yaw, kAudibleThresh));
        g.setColour(tint.withAlpha(0.36f));
        g.fillPath(buildAudibilityContour(src, yaw, kLoudThresh));

        // Heading arrow + tip handle.
        const float arrowLen = 28.0f;
        const auto arrowTip = src + compassDir(yaw) * arrowLen;
        g.setColour(juce::Colour(0x9bff8c42));
        g.drawLine(src.x, src.y, arrowTip.x, arrowTip.y, 2.0f);
        // Arrowhead.
        {
            const auto dir = compassDir(yaw);
            const juce::Point<float> perp { -dir.y, dir.x };
            juce::Path head;
            head.addTriangle(arrowTip.x + dir.x * 6.0f,  arrowTip.y + dir.y * 6.0f,
                             arrowTip.x - dir.x * 3.0f + perp.x * 5.0f,
                             arrowTip.y - dir.y * 3.0f + perp.y * 5.0f,
                             arrowTip.x - dir.x * 3.0f - perp.x * 5.0f,
                             arrowTip.y - dir.y * 3.0f - perp.y * 5.0f);
            g.setColour(juce::Colour(0xffff8c42));
            g.fillPath(head);
        }

        // Source dot.
        g.setColour(juce::Colour(0xffff8c42));
        g.fillEllipse(src.x - 8.0f, src.y - 8.0f, 16.0f, 16.0f);
        g.setColour(juce::Colour(0xff5a2410));
        g.drawEllipse(src.x - 8.0f, src.y - 8.0f, 16.0f, 16.0f, 1.5f);

        g.setColour(juce::Colour(0xffffb88a));
        g.setFont(font10());
        const bool above = src.y > bounds.getBottom() - 22.0f;
        const float labelY = above ? src.y - 22.0f : src.y + 10.0f;
        g.drawText("SOURCE",
                   juce::Rectangle<float>(src.x - 30.0f, labelY, 60.0f, 12.0f),
                   juce::Justification::centred);

        // Top-left readouts.
        g.setColour(juce::Colour(0xff9a9a9a));
        g.setFont(font11());
        const auto info = juce::String("pos: ") + juce::String(dist, 2) + " m  "
                        + juce::String(az, 1) + kGlyphDeg + " (" + azimuthCardinal(az) + ")   yaw: "
                        + juce::String(yaw, 1) + kGlyphDeg;
        g.drawText(info, juce::Rectangle<int>(8, 8, getWidth() - 16, 14),
                   juce::Justification::topLeft);
        g.setColour(juce::Colour(0xff666666));
        g.setFont(font10());
        // Use ASCII "Cmd" — the U+2318 ⌘ glyph is missing in many fonts.
        const auto hint = juce::String("drag dot to move ") + kGlyphMid + " arrow to aim  "
                        + kGlyphDash + "  hold Cmd to snap";
        g.drawText(hint,
                   juce::Rectangle<int>(8, 22, getWidth() - 16, 12),
                   juce::Justification::topLeft);
    }

    void mouseDown(const juce::MouseEvent& e) override
    {
        activeDrag_ = hitTest(e.position);
        // Manual yaw drag turns off auto-aim so the user isn't fighting the lock.
        if (activeDrag_ == DragTarget::Heading)
            setBoolParam("aim_at_listener", false);
        beginGesture(activeDrag_);
        applyDrag(e.position, e.mods.isCommandDown());
    }

    void mouseDrag(const juce::MouseEvent& e) override
    {
        applyDrag(e.position, e.mods.isCommandDown());
    }

    void mouseUp(const juce::MouseEvent&) override
    {
        endGesture(activeDrag_);
        activeDrag_ = DragTarget::None;
    }

    void mouseMove(const juce::MouseEvent& e) override
    {
        const auto t = hitTest(e.position);
        switch (t)
        {
            case DragTarget::Heading: setTooltip("Source yaw - drag to aim"); break;
            case DragTarget::Source:  setTooltip("Source position - drag to move"); break;
            default:                  setTooltip({}); break;
        }
    }

private:
    enum class DragTarget { None, Source, Heading };

    void timerCallback() override
    {
        // Aim-at-listener: lock source_yaw to face the listener (compass
        // centre). Yaw = azim + 180° (source faces back toward origin).
        const bool aim = state_.getRawParameterValue("aim_at_listener")->load() > 0.5f;
        if (aim && activeDrag_ != DragTarget::Heading)
        {
            const float target = wrap180(currentAzimuth() + 180.0f);
            const float curr   = state_.getRawParameterValue("source_yaw")->load();
            if (std::abs(wrap180(target - curr)) > 0.05f)
                setParam("source_yaw", target);
        }

        const float pyaw  = state_.getRawParameterValue("source_yaw")->load();
        const float poccl = state_.getRawParameterValue("occlusion")->load();
        const float prevDispYaw  = displayedYaw_;
        const float prevDispOccl = displayedOcclusion_;
        if (firstTick_)
        {
            displayedYaw_       = pyaw;
            displayedOcclusion_ = poccl;
            firstTick_          = false;
        }
        else
        {
            displayedYaw_       = wrap180(angleLerp(displayedYaw_, pyaw, 0.25f));
            displayedOcclusion_ += (poccl - displayedOcclusion_) * 0.18f;
        }

        // Conditional repaint: skip when nothing has changed since last
        // frame. Tracks the 8 contour-shape inputs + 2 smoothed values
        // + drag state. JUCE's setBufferedToImage cache then reuses the
        // last rendered buffer for free.
        const float snap[8] = {
            currentDistance(),
            currentAzimuth(),
            state_.getRawParameterValue("dir_inner_deg") ->load(),
            state_.getRawParameterValue("dir_outer_deg") ->load(),
            state_.getRawParameterValue("dir_outer_gain")->load(),
            state_.getRawParameterValue("dir_outer_lp")  ->load(),
            state_.getRawParameterValue("gain_db")       ->load(),
            state_.getRawParameterValue("direct_path_gain")->load(),
        };
        bool changed = activeDrag_ != DragTarget::None
                    || std::abs(displayedYaw_  - prevDispYaw)  > 0.02f
                    || std::abs(displayedOcclusion_ - prevDispOccl) > 0.001f;
        for (int i = 0; i < 8; ++i)
            if (snap[i] != prevSnap_[i]) { prevSnap_[i] = snap[i]; changed = true; }
        if (changed) repaint();
    }

    juce::Point<float> centre() const
    {
        return getLocalBounds().toFloat().getCentre();
    }

    float outerRadius() const
    {
        const auto b = getLocalBounds().toFloat();
        return juce::jmin(b.getWidth(), b.getHeight()) * 0.45f;
    }

    juce::Point<float> sourceScreen() const
    {
        return azimDistToScreen(centre(), outerRadius(),
                                currentAzimuth(), currentDistance(), kCompassMaxMeters);
    }

    float currentDistance() const { return state_.getRawParameterValue("distance")->load(); }
    float currentAzimuth()  const { return state_.getRawParameterValue("azimuth")->load(); }

    float metersToPixels(float meters) const
    {
        return meters * (outerRadius() / kCompassMaxMeters);
    }

    // Inverse §3 curve: given a target linear gain, return the
    // distance at which the per-source distance model produces that
    // gain. Model is monotonically non-increasing, so we walk the
    // segments from A→D.
    float distanceForGain(float gTarget) const
    {
        const float aD  = state_.getRawParameterValue("dist_a")->load();
        const float bD  = state_.getRawParameterValue("dist_b")->load();
        const float cD  = state_.getRawParameterValue("dist_c")->load();
        const float dD  = state_.getRawParameterValue("dist_d")->load();
        const float aG  = std::pow(10.0f, state_.getRawParameterValue("dist_a_db")->load() * 0.05f);
        const float bG  = std::pow(10.0f, state_.getRawParameterValue("dist_b_db")->load() * 0.05f);
        const float cG  = std::pow(10.0f, state_.getRawParameterValue("dist_c_db")->load() * 0.05f);

        if (gTarget >= aG) return 0.0f;          // source louder than target even at A.
        if (gTarget <= 0.0f) return dD;          // never silent inside the curve.
        if (gTarget >= bG)
            return aD + (aG - gTarget) / std::max(1e-9f, aG - bG) * (bD - aD);
        if (gTarget >= cG)
            return bD + (bG - gTarget) / std::max(1e-9f, bG - cG) * (cD - bD);
        return cD + (cG - gTarget) / std::max(1e-9f, cG) * (dD - cD);
    }

    // Radius at which the source level falls below `thresholdLin`, in
    // metres, for a listener at angle `angleFromForwardRad` off the
    // source's forward axis. Combines gain × direct × directivity_gain
    // and then inverts the §3 distance curve.
    float audibilityRadiusMeters(float angleFromForwardRad, float thresholdLin) const
    {
        const float inner = juce::degreesToRadians(state_.getRawParameterValue("dir_inner_deg")->load());
        const float outer = juce::degreesToRadians(state_.getRawParameterValue("dir_outer_deg")->load());
        const float ogain = state_.getRawParameterValue("dir_outer_gain")->load();

        const float a = std::abs(angleFromForwardRad);
        float t;
        if (a <= inner)       t = 0.0f;
        else if (a >= outer)  t = 1.0f;
        else                  t = (a - inner) / std::max(1e-6f, outer - inner);
        const float dirGain = 1.0f + t * (ogain - 1.0f);

        const float dB      = state_.getRawParameterValue("gain_db")->load();
        const float gainLin = std::pow(10.0f, dB / 20.0f);
        const float direct  = state_.getRawParameterValue("direct_path_gain")->load();
        const float strength = gainLin * direct * dirGain;
        if (strength < 1e-6f || thresholdLin < 1e-6f) return 0.0f;
        return distanceForGain(thresholdLin / strength);
    }

    juce::Path buildAudibilityContour(juce::Point<float> src, float yawDeg, float thresholdLin) const
    {
        juce::Path p;
        constexpr int N = 64;
        for (int i = 0; i <= N; ++i)
        {
            const float compassAng = (float) i * (360.0f / (float) N);
            const float devDeg     = wrap180(compassAng - yawDeg);
            const float angRad     = juce::degreesToRadians(std::abs(devDeg));
            const float rPx        = metersToPixels(audibilityRadiusMeters(angRad, thresholdLin));
            const auto pt          = src + compassDir(compassAng) * rPx;
            if (i == 0) p.startNewSubPath(pt.x, pt.y);
            else        p.lineTo(pt.x, pt.y);
        }
        p.closeSubPath();
        return p;
    }

    juce::Path buildWedge(juce::Point<float> src, float yawDeg, float angRad, float radius) const
    {
        juce::Path p;
        const float half = juce::radiansToDegrees(angRad) * 0.5f;
        p.startNewSubPath(src.x, src.y);
        const int kSeg = 64;
        for (int i = 0; i <= kSeg; ++i)
        {
            const float t = (float)i / (float)kSeg;
            const float a = yawDeg - half + t * juce::radiansToDegrees(angRad);
            const auto dir = compassDir(a);
            p.lineTo(src.x + dir.x * radius, src.y + dir.y * radius);
        }
        p.closeSubPath();
        return p;
    }

    DragTarget hitTest(juce::Point<float> p) const
    {
        const auto src = sourceScreen();

        // Heading handle.
        const auto headTip = src + compassDir(displayedYaw_) * 28.0f;
        if (p.getDistanceFrom(headTip) <= 9.0f) return DragTarget::Heading;

        // Source dot fallback (large hit box).
        if (p.getDistanceFrom(src) <= 14.0f) return DragTarget::Source;

        // Anywhere else also moves source — matches existing M5 behaviour.
        return DragTarget::Source;
    }

    void applyDrag(juce::Point<float> p, bool snap)
    {
        switch (activeDrag_)
        {
            case DragTarget::Heading: applyHeading(p, snap);    break;
            case DragTarget::Source:  applySourceMove(p, snap); break;
            default: break;
        }
    }

    void applySourceMove(juce::Point<float> p, bool snap)
    {
        float az, dist;
        screenToAzimDist(centre(), outerRadius(), kCompassMaxMeters, p, az, dist);
        if (snap)
        {
            dist = snapTo(dist, {0.0f, 1.0f, 2.0f, 5.0f, 10.0f, 15.0f, 20.0f, 25.0f});
            az   = snapSymmetric(az, {0.0f, 30.0f, 45.0f, 60.0f, 90.0f, 120.0f, 135.0f, 150.0f, 180.0f});
        }
        setParam("distance", dist);
        setParam("azimuth",  az);
    }

    void applyHeading(juce::Point<float> p, bool snap)
    {
        const auto src = sourceScreen();
        const auto v   = p - src;
        if (v.getDistanceFromOrigin() < 1.0f) return;
        float yawDeg = compassAngleDeg(v);
        if (snap)
            yawDeg = snapSymmetric(yawDeg, {0.0f, 30.0f, 45.0f, 60.0f, 90.0f, 120.0f, 135.0f, 150.0f, 180.0f});
        setParam("source_yaw", yawDeg);
        // Drive displayed_ directly during drag so the wedge tracks the mouse.
        displayedYaw_ = yawDeg;
    }

    void setBoolParam(const char* id, bool v)
    {
        if (auto* p = state_.getParameter(id))
            p->setValueNotifyingHost(v ? 1.0f : 0.0f);
    }

    void setParam(const char* id, float v)
    {
        if (auto* p = state_.getParameter(id))
            p->setValueNotifyingHost(p->convertTo0to1(v));
    }

    void beginGesture(DragTarget t)
    {
        for (const char* id : idsForTarget(t))
            if (auto* p = state_.getParameter(id))
                p->beginChangeGesture();
    }

    void endGesture(DragTarget t)
    {
        for (const char* id : idsForTarget(t))
            if (auto* p = state_.getParameter(id))
                p->endChangeGesture();
    }

    static std::vector<const char*> idsForTarget(DragTarget t)
    {
        switch (t)
        {
            case DragTarget::Source:  return { "distance", "azimuth" };
            case DragTarget::Heading: return { "source_yaw" };
            default:                  return {};
        }
    }

    void resized() override { backgroundImage_ = {}; }

    void paintStaticBackground(juce::Graphics& g,
                                juce::Point<float> centre,
                                float outerR) const
    {
        g.fillAll(juce::Colour(0xff141414));

        g.setColour(juce::Colour(0xff2a2a2a));
        for (float d = 5.0f; d <= kCompassMaxMeters; d += 5.0f)
        {
            const float ringR = outerR * (d / kCompassMaxMeters);
            g.drawEllipse(centre.x - ringR, centre.y - ringR,
                          2.0f * ringR, 2.0f * ringR, 1.0f);
        }
        g.setColour(juce::Colour(0xff444444));
        g.drawEllipse(centre.x - outerR, centre.y - outerR,
                      2.0f * outerR, 2.0f * outerR, 1.5f);

        g.setColour(juce::Colour(0xff262626));
        g.drawLine(centre.x, centre.y - outerR, centre.x, centre.y + outerR);
        g.drawLine(centre.x - outerR, centre.y, centre.x + outerR, centre.y);

        g.setColour(juce::Colour(0xff7a7a7a));
        g.setFont(font11());
        g.drawText("FRONT", juce::Rectangle<float>(centre.x - 40, centre.y - outerR - 16, 80, 12),
                   juce::Justification::centred);
        g.drawText("BACK",  juce::Rectangle<float>(centre.x - 40, centre.y + outerR + 4,  80, 12),
                   juce::Justification::centred);
        g.drawText("LEFT",  juce::Rectangle<float>(centre.x - outerR + 6, centre.y - 6, 40, 12),
                   juce::Justification::centredLeft);
        g.drawText("RIGHT", juce::Rectangle<float>(centre.x + outerR - 46, centre.y - 6, 40, 12),
                   juce::Justification::centredRight);

        g.setColour(juce::Colour(0xff5a82ff));
        g.fillEllipse(centre.x - 13.0f, centre.y - 13.0f, 26.0f, 26.0f);
        juce::Path nose;
        nose.addTriangle(centre.x - 6.0f, centre.y - 9.0f,
                         centre.x + 6.0f, centre.y - 9.0f,
                         centre.x,        centre.y - 18.0f);
        g.fillPath(nose);
        g.setColour(juce::Colour(0xff1a2a55));
        g.drawEllipse(centre.x - 13.0f, centre.y - 13.0f, 26.0f, 26.0f, 1.5f);
        g.strokePath(nose, juce::PathStrokeType(1.5f));
        g.setColour(juce::Colour(0xff8aa8ff));
        g.setFont(font10());
        g.drawText("YOU",
                   juce::Rectangle<float>(centre.x - 30.0f, centre.y + 18.0f, 60.0f, 12.0f),
                   juce::Justification::centred);
    }

    juce::AudioProcessorValueTreeState& state_;
    DragTarget activeDrag_ = DragTarget::None;
    float displayedYaw_       = 0.0f;
    float displayedOcclusion_ = 0.0f;
    bool  firstTick_          = true;
    juce::Image backgroundImage_;
    float prevSnap_[8] = { std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN() };
};

// ---------------------------------------------------------------------------
// ElevationStrip — two columns: position elevation + source pitch.
// ---------------------------------------------------------------------------

class ElevationStrip : public juce::Component,
                       public juce::SettableTooltipClient,
                       private juce::Timer
{
public:
    explicit ElevationStrip(juce::AudioProcessorValueTreeState& s) : state_(s)
    {
        setTooltip("Elevation (position) on the left " + kGlyphMid + " pitch (orientation) on the right");
        setBufferedToImage(true);
        startTimerHz(15);
    }

    ~ElevationStrip() override { stopTimer(); }

    void paint(juce::Graphics& g) override
    {
        g.fillAll(juce::Colour(0xff141414));

        const auto b = getLocalBounds().toFloat();
        const float colW = b.getWidth() * 0.5f;
        const auto leftCol  = juce::Rectangle<float>(b.getX(),         b.getY(), colW, b.getHeight());
        const auto rightCol = juce::Rectangle<float>(b.getX() + colW,  b.getY(), colW, b.getHeight());

        drawPositionColumn(g, leftCol);
        drawPitchColumn   (g, rightCol);
    }

    void mouseDown(const juce::MouseEvent& e) override
    {
        activeDrag_ = dragTarget(e.position);
        if (activeDrag_ == Drag::Pos)
        {
            if (auto* p = state_.getParameter("elevation")) p->beginChangeGesture();
        }
        else if (activeDrag_ == Drag::Pitch)
        {
            if (auto* p = state_.getParameter("source_pitch")) p->beginChangeGesture();
        }
        applyDrag(e.position, e.mods.isCommandDown());
    }

    void mouseDrag(const juce::MouseEvent& e) override
    {
        applyDrag(e.position, e.mods.isCommandDown());
    }

    void mouseUp(const juce::MouseEvent&) override
    {
        if (activeDrag_ == Drag::Pos)
        {
            if (auto* p = state_.getParameter("elevation")) p->endChangeGesture();
        }
        else if (activeDrag_ == Drag::Pitch)
        {
            if (auto* p = state_.getParameter("source_pitch")) p->endChangeGesture();
        }
        activeDrag_ = Drag::None;
    }

private:
    enum class Drag { None, Pos, Pitch };

    void timerCallback() override
    {
        const float el  = state_.getRawParameterValue("elevation")->load();
        const float pt  = state_.getRawParameterValue("source_pitch")->load();
        if (el != prevEl_ || pt != prevPitch_)
        {
            prevEl_    = el;
            prevPitch_ = pt;
            repaint();
        }
    }

    Drag dragTarget(juce::Point<float> p) const
    {
        const auto b = getLocalBounds().toFloat();
        return p.x < b.getCentreX() ? Drag::Pos : Drag::Pitch;
    }

    // Top reserves 16px for the caption; bottom reserves 28px (caption + value).
    static void columnTopBot(juce::Rectangle<float> col, float& topY, float& botY)
    {
        topY = col.getY() + 22.0f;
        botY = col.getBottom() - 30.0f;
    }

    void drawPositionColumn(juce::Graphics& g, juce::Rectangle<float> col)
    {
        const float cx = col.getCentreX();
        float topY, botY;
        columnTopBot(col, topY, botY);
        const float trackH = botY - topY;

        g.setColour(juce::Colour(0xff2c2c2c));
        g.fillRoundedRectangle(cx - 2.0f, topY, 4.0f, trackH, 2.0f);

        const float midY = topY + trackH * 0.5f;
        g.setColour(juce::Colour(0xff4a4a4a));
        g.drawLine(cx - 10.0f, midY, cx + 10.0f, midY, 1.0f);

        g.setColour(juce::Colour(0xff7a7a7a));
        g.setFont(font10());
        g.drawText("UP",
                   juce::Rectangle<float>(col.getX(), col.getY() + 4.0f, col.getWidth(), 12.0f),
                   juce::Justification::centred);
        g.drawText("ELEV",
                   juce::Rectangle<float>(col.getX(), botY + 4.0f, col.getWidth(), 12.0f),
                   juce::Justification::centred);

        const float el  = state_.getRawParameterValue("elevation")->load();
        const float t   = juce::jlimit(0.0f, 1.0f, (90.0f - el) / 180.0f);
        const float h_y = topY + t * trackH;

        g.setColour(juce::Colour(0xffff8c42));
        g.fillEllipse(cx - 9.0f, h_y - 9.0f, 18.0f, 18.0f);
        g.setColour(juce::Colour(0xff5a2410));
        g.drawEllipse(cx - 9.0f, h_y - 9.0f, 18.0f, 18.0f, 1.5f);

        g.setColour(juce::Colour(0xff9a9a9a));
        g.setFont(font10());
        g.drawText(juce::String((int) std::round(el)) + kGlyphDeg,
                   juce::Rectangle<float>(col.getX(), botY + 16.0f, col.getWidth(), 12.0f),
                   juce::Justification::centred);
    }

    void drawPitchColumn(juce::Graphics& g, juce::Rectangle<float> col)
    {
        const float cx = col.getCentreX();
        float topY, botY;
        columnTopBot(col, topY, botY);
        const float trackH = botY - topY;
        const float midY   = topY + trackH * 0.5f;

        g.setColour(juce::Colour(0xff222222));
        g.fillRoundedRectangle(juce::Rectangle<float>(col.getX() + 6, topY, col.getWidth() - 12, trackH), 4.0f);

        g.setColour(juce::Colour(0xff4a4a4a));
        g.drawLine(cx - 14.0f, midY, cx + 14.0f, midY, 1.0f);

        g.setColour(juce::Colour(0xff7a7a7a));
        g.setFont(font10());
        g.drawText("TILT",
                   juce::Rectangle<float>(col.getX(), col.getY() + 4.0f, col.getWidth(), 12.0f),
                   juce::Justification::centred);
        g.drawText("PITCH",
                   juce::Rectangle<float>(col.getX(), botY + 4.0f, col.getWidth(), 12.0f),
                   juce::Justification::centred);

        const float pitch = state_.getRawParameterValue("source_pitch")->load();
        // Handle position mirrors the position column: top → +90, bot → −90.
        const float t   = juce::jlimit(0.0f, 1.0f, (90.0f - pitch) / 180.0f);
        const float h_y = topY + t * trackH;

        g.setColour(juce::Colour(0xffff8c42));
        g.fillEllipse(cx - 9.0f, h_y - 9.0f, 18.0f, 18.0f);
        g.setColour(juce::Colour(0xff5a2410));
        g.drawEllipse(cx - 9.0f, h_y - 9.0f, 18.0f, 18.0f, 1.5f);

        // Tilt indicator: a short line through the handle, tilted by pitch.
        // Pitch +90 → vertical (pointing up); 0 → horizontal; −90 → vertical down.
        const float ang     = juce::degreesToRadians(pitch);
        const float lineLen = 11.0f;
        const float dxL = std::cos(ang), dyL = -std::sin(ang);
        g.setColour(juce::Colour(0xff5a2410));
        g.drawLine(cx - dxL * lineLen, h_y - dyL * lineLen,
                   cx + dxL * lineLen, h_y + dyL * lineLen, 2.0f);
        // Arrowhead at the +ang end.
        {
            const float ahx = cx + dxL * lineLen;
            const float ahy = h_y + dyL * lineLen;
            const float px = -dyL, py = dxL;
            juce::Path head;
            head.addTriangle(ahx + dxL * 4.0f, ahy + dyL * 4.0f,
                             ahx - dxL * 2.0f + px * 3.0f, ahy - dyL * 2.0f + py * 3.0f,
                             ahx - dxL * 2.0f - px * 3.0f, ahy - dyL * 2.0f - py * 3.0f);
            g.fillPath(head);
        }

        g.setColour(juce::Colour(0xff9a9a9a));
        g.setFont(font10());
        g.drawText(juce::String((int) std::round(pitch)) + kGlyphDeg,
                   juce::Rectangle<float>(col.getX(), botY + 16.0f, col.getWidth(), 12.0f),
                   juce::Justification::centred);
    }

    void applyDrag(juce::Point<float> p, bool snap)
    {
        const auto b = getLocalBounds().toFloat();
        const float colW = b.getWidth() * 0.5f;
        const auto col = (activeDrag_ == Drag::Pos)
            ? juce::Rectangle<float>(b.getX(),        b.getY(), colW, b.getHeight())
            : juce::Rectangle<float>(b.getX() + colW, b.getY(), colW, b.getHeight());
        float topY, botY;
        columnTopBot(col, topY, botY);
        const float trackH = botY - topY;
        const float t  = juce::jlimit(0.0f, 1.0f, (p.y - topY) / trackH);
        float v = 90.0f - t * 180.0f;
        if (snap)
            v = snapSymmetric(v, {0.0f, 15.0f, 30.0f, 45.0f, 60.0f, 90.0f});
        const char* id = activeDrag_ == Drag::Pos ? "elevation" : "source_pitch";
        if (auto* param = state_.getParameter(id))
            param->setValueNotifyingHost(param->convertTo0to1(v));
    }

    juce::AudioProcessorValueTreeState& state_;
    Drag activeDrag_ = Drag::None;
    float prevEl_    = std::numeric_limits<float>::quiet_NaN();
    float prevPitch_ = std::numeric_limits<float>::quiet_NaN();
};

// ---------------------------------------------------------------------------
// DistanceCurveEditor — interactive 4-knot piecewise gain graph.
// Replaces the 7 distance-curve sliders with a draggable visualisation.
// ---------------------------------------------------------------------------

class DistanceCurveEditor : public juce::Component,
                            public juce::SettableTooltipClient,
                            private juce::Timer
{
public:
    explicit DistanceCurveEditor(juce::AudioProcessorValueTreeState& s)
        : state_(s)
    {
        setTooltip("Distance vs gain curve. Drag A/B/C/D nodes to edit. A is the near-field anchor; D is the silence anchor (distance only).");
        setBufferedToImage(true);
        startTimerHz(15);
    }

    ~DistanceCurveEditor() override { stopTimer(); }

    void paint(juce::Graphics& g) override
    {
        g.fillAll(juce::Colour(0xff141414));
        const auto plot = plotArea();

        // Grid lines.
        g.setColour(juce::Colour(0xff232323));
        for (int d = 0; d <= (int) DIST_MAX; d += 10)
        {
            const float x = graphX((float) d);
            g.drawLine(x, plot.getY(), x, plot.getBottom(), 1.0f);
        }
        for (int db = (int) DB_MIN; db <= (int) DB_MAX; db += 20)
        {
            const float y = graphY((float) db);
            g.drawLine(plot.getX(), y, plot.getRight(), y, 1.0f);
        }
        // 0-dB reference line emphasised.
        g.setColour(juce::Colour(0xff353535));
        const float y0 = graphY(0.0f);
        g.drawLine(plot.getX(), y0, plot.getRight(), y0, 1.0f);

        // Axis labels.
        g.setColour(juce::Colour(0xff7a7a7a));
        g.setFont(font9());
        for (int d : {1, 10, 50, 100, 150})
        {
            if ((float) d > DIST_MAX) continue;
            g.drawText(juce::String(d) + " m",
                       juce::Rectangle<float>(graphX((float) d) - 18.0f,
                                              plot.getBottom() + 1.0f, 36.0f, 10.0f),
                       juce::Justification::centred);
        }
        for (int db : {0, -20, -40, -60})
        {
            g.drawText(juce::String(db) + " dB",
                       juce::Rectangle<float>(0.0f, graphY((float) db) - 5.0f,
                                              plot.getX() - 2.0f, 10.0f),
                       juce::Justification::centredRight);
        }

        // The curve path.
        const auto a = nodePos(Node::A);
        const auto b = nodePos(Node::B);
        const auto c = nodePos(Node::C);
        const auto d = nodePos(Node::D); // D's "gain" is silence (DB_MIN)

        // Fill the area under the curve up to 0 dB.
        juce::Path fill;
        fill.startNewSubPath(graphX(0.0f), y0);
        fill.lineTo(graphX(0.0f), a.y);
        fill.lineTo(a.x, a.y);
        fill.lineTo(b.x, b.y);
        fill.lineTo(c.x, c.y);
        fill.lineTo(d.x, d.y);
        fill.lineTo(d.x, y0);
        fill.closeSubPath();
        g.setColour(juce::Colour(0x33ff8c42));
        g.fillPath(fill);

        // The curve itself.
        juce::Path line;
        line.startNewSubPath(graphX(0.0f), a.y); // flat below a_dist
        line.lineTo(a.x, a.y);
        line.lineTo(b.x, b.y);
        line.lineTo(c.x, c.y);
        line.lineTo(d.x, d.y);
        g.setColour(juce::Colour(0xffff8c42));
        g.strokePath(line, juce::PathStrokeType(1.5f));

        // Draggable nodes.
        drawNode(g, a, "A", Node::A);
        drawNode(g, b, "B", Node::B);
        drawNode(g, c, "C", Node::C);
        drawNode(g, d, "D", Node::D);
    }

    void mouseDown(const juce::MouseEvent& e) override
    {
        activeNode_ = hitTestNode(e.position);
        beginGesture(activeNode_);
        applyDrag(e.position);
    }

    void mouseDrag(const juce::MouseEvent& e) override { applyDrag(e.position); }

    void mouseUp(const juce::MouseEvent&) override
    {
        endGesture(activeNode_);
        activeNode_ = Node::None;
    }

private:
    enum class Node { None, A, B, C, D };
    static constexpr float DIST_MAX = 150.0f;
    static constexpr float DB_MIN   = -80.0f;
    static constexpr float DB_MAX   = 6.0f;

    void timerCallback() override
    {
        const float vals[7] = {
            state_.getRawParameterValue("dist_a")   ->load(),
            state_.getRawParameterValue("dist_a_db")->load(),
            state_.getRawParameterValue("dist_b")   ->load(),
            state_.getRawParameterValue("dist_b_db")->load(),
            state_.getRawParameterValue("dist_c")   ->load(),
            state_.getRawParameterValue("dist_c_db")->load(),
            state_.getRawParameterValue("dist_d")   ->load(),
        };
        for (int i = 0; i < 7; ++i)
            if (vals[i] != prevVals_[i]) { prevVals_[i] = vals[i]; dirty_ = true; }
        if (dirty_) { dirty_ = false; repaint(); }
    }

    juce::Rectangle<float> plotArea() const
    {
        // Reserve left edge for dB labels, bottom for distance labels.
        const auto b = getLocalBounds().toFloat();
        return b.withTrimmedLeft(34.0f).withTrimmedBottom(12.0f).withTrimmedTop(4.0f).withTrimmedRight(6.0f);
    }

    float graphX(float dist) const
    {
        const auto p = plotArea();
        const float t = juce::jlimit(0.0f, 1.0f, dist / DIST_MAX);
        return p.getX() + t * p.getWidth();
    }
    float graphY(float db) const
    {
        const auto p = plotArea();
        const float t = juce::jlimit(0.0f, 1.0f, (DB_MAX - db) / (DB_MAX - DB_MIN));
        return p.getY() + t * p.getHeight();
    }

    float pxToDist(float x) const
    {
        const auto p = plotArea();
        const float t = juce::jlimit(0.0f, 1.0f, (x - p.getX()) / p.getWidth());
        return t * DIST_MAX;
    }
    float pxToDb(float y) const
    {
        const auto p = plotArea();
        const float t = juce::jlimit(0.0f, 1.0f, (y - p.getY()) / p.getHeight());
        return DB_MAX - t * (DB_MAX - DB_MIN);
    }

    juce::Point<float> nodePos(Node n) const
    {
        const auto v = [&](const char* id) { return state_.getRawParameterValue(id)->load(); };
        switch (n)
        {
            case Node::A: return { graphX(v("dist_a")), graphY(v("dist_a_db")) };
            case Node::B: return { graphX(v("dist_b")), graphY(v("dist_b_db")) };
            case Node::C: return { graphX(v("dist_c")), graphY(v("dist_c_db")) };
            case Node::D: return { graphX(v("dist_d")), graphY(DB_MIN) };
            default: return {};
        }
    }

    void drawNode(juce::Graphics& g, juce::Point<float> p, const char* label, Node n)
    {
        const bool active = activeNode_ == n;
        const float r = active ? 7.0f : 5.0f;
        g.setColour(juce::Colour(0xffff8c42));
        g.fillEllipse(p.x - r, p.y - r, 2 * r, 2 * r);
        g.setColour(juce::Colour(0xff5a2410));
        g.drawEllipse(p.x - r, p.y - r, 2 * r, 2 * r, 1.5f);
        g.setColour(juce::Colour(0xffe8c5a8));
        g.setFont(font10());
        g.drawText(label,
                   juce::Rectangle<float>(p.x + r + 1, p.y - 6, 14, 12),
                   juce::Justification::centredLeft);
    }

    Node hitTestNode(juce::Point<float> p) const
    {
        for (auto n : { Node::A, Node::B, Node::C, Node::D })
        {
            if (p.getDistanceFrom(nodePos(n)) <= 12.0f)
                return n;
        }
        return Node::None;
    }

    void applyDrag(juce::Point<float> p)
    {
        if (activeNode_ == Node::None) return;
        const float dist = pxToDist(p.x);
        const float db   = pxToDb(p.y);
        // Enforce ordering a < b < c < d (small epsilon to avoid identical values).
        const auto v = [&](const char* id) { return state_.getRawParameterValue(id)->load(); };
        const float a_d = v("dist_a"), b_d = v("dist_b"), c_d = v("dist_c"), d_d = v("dist_d");
        switch (activeNode_)
        {
            case Node::A:
                set("dist_a",    juce::jlimit(0.0f, b_d - 0.01f, dist));
                set("dist_a_db", juce::jlimit(-80.0f, 12.0f, db));
                break;
            case Node::B:
                set("dist_b",    juce::jlimit(a_d + 0.01f, c_d - 0.01f, dist));
                set("dist_b_db", juce::jlimit(-80.0f, 12.0f, db));
                break;
            case Node::C:
                set("dist_c",    juce::jlimit(b_d + 0.01f, d_d - 0.01f, dist));
                set("dist_c_db", juce::jlimit(-80.0f, 12.0f, db));
                break;
            case Node::D:
                set("dist_d", juce::jlimit(c_d + 0.01f, 300.0f, dist));
                break;
            default: break;
        }
    }

    void set(const char* id, float v)
    {
        if (auto* p = state_.getParameter(id))
            p->setValueNotifyingHost(p->convertTo0to1(v));
    }
    void beginGesture(Node n) { forEachId(n, [](auto* p) { p->beginChangeGesture(); }); }
    void endGesture(Node n)   { forEachId(n, [](auto* p) { p->endChangeGesture();   }); }
    template <class F> void forEachId(Node n, F f)
    {
        auto ids = idsFor(n);
        for (auto* id : ids)
            if (auto* p = state_.getParameter(id))
                f(p);
    }
    static std::vector<const char*> idsFor(Node n)
    {
        switch (n)
        {
            case Node::A: return { "dist_a", "dist_a_db" };
            case Node::B: return { "dist_b", "dist_b_db" };
            case Node::C: return { "dist_c", "dist_c_db" };
            case Node::D: return { "dist_d" };
            default:      return {};
        }
    }

    juce::AudioProcessorValueTreeState& state_;
    Node activeNode_ = Node::None;
    float prevVals_[7] = { std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN() };
    bool dirty_ = true;
};

// ---------------------------------------------------------------------------
// SpatialAudioEditor.
// ---------------------------------------------------------------------------

namespace {

void styleHeaderLabel(juce::Label& l)
{
    l.setColour(juce::Label::textColourId, juce::Colour(0xff8a8a8a));
    l.setFont(juce::Font(juce::FontOptions(10.0f)).withExtraKerningFactor(0.18f));
    l.setJustificationType(juce::Justification::centredLeft);
}

} // namespace

SpatialAudioEditor::SpatialAudioEditor(SpatialAudioProcessor& p)
    : AudioProcessorEditor(p), proc_(p)
{
    compass_     = std::make_unique<SpatialCompass>(p.apvts);
    elevation_   = std::make_unique<ElevationStrip>(p.apvts);
    curveEditor_ = std::make_unique<DistanceCurveEditor>(p.apvts);
    addAndMakeVisible(*compass_);
    addAndMakeVisible(*elevation_);
    addAndMakeVisible(*curveEditor_);

    styleHeaderLabel(shapeHeader_);
    styleHeaderLabel(environmentHeader_);
    styleHeaderLabel(outputHeader_);
    styleHeaderLabel(advancedHeader_);
    addAndMakeVisible(shapeHeader_);
    addAndMakeVisible(environmentHeader_);
    addAndMakeVisible(outputHeader_);
    addAndMakeVisible(advancedHeader_);

    auto initSlider = [this](juce::Slider& s)
    {
        s.setSliderStyle(juce::Slider::LinearHorizontal);
        s.setTextBoxStyle(juce::Slider::TextBoxRight, false, 60, 18);
        addAndMakeVisible(s);
    };
    auto initLabel = [this](juce::Label& l, const char* text)
    {
        l.setText(text, juce::dontSendNotification);
        l.setColour(juce::Label::textColourId, juce::Colour(0xffbbbbbb));
        l.setJustificationType(juce::Justification::centredRight);
        l.setFont(juce::Font(juce::FontOptions(11.0f)));
        addAndMakeVisible(l);
    };

    // Top row (gain + aim).
    initLabel(gainLabel_, "Gain");
    initSlider(gainSlider_);
    gainSlider_.setTooltip("Source gain (dB).");
    gainAttachment_ = std::make_unique<SliderAttachment>(p.apvts, "gain_db", gainSlider_);

    aimAtListenerButton_.setTooltip("Lock source orientation to face the listener.");
    aimAtListenerButton_.setColour(juce::ToggleButton::textColourId, juce::Colour(0xffbbbbbb));
    addAndMakeVisible(aimAtListenerButton_);
    aimAttachment_ = std::make_unique<ButtonAttachment>(
        p.apvts, "aim_at_listener", aimAtListenerButton_);

    // SHAPE section.
    initLabel(spreadLabel_, "Spread");
    initSlider(spreadSlider_);
    spreadSlider_.setTooltip(juce::String::fromUTF8("Cone outer angle: angle from forward at which the source becomes fully off-axis. 0\xC2\xB0 = pencil beam; 180\xC2\xB0 = omnidirectional."));
    spreadAttachment_ = std::make_unique<SliderAttachment>(p.apvts, "dir_outer_deg", spreadSlider_);

    initLabel(focusLabel_, "Focus");
    initSlider(focusSlider_);
    focusSlider_.setTooltip(juce::String::fromUTF8("Cone inner angle: width of the full-volume zone in front of the source."));
    focusAttachment_ = std::make_unique<SliderAttachment>(p.apvts, "dir_inner_deg", focusSlider_);

    initLabel(offGainLabel_, "Off-Gain");
    initSlider(offGainSlider_);
    offGainSlider_.setTooltip("Off-axis gain at the cone edge (1 = no attenuation, 0 = silent).");
    offGainAttachment_ = std::make_unique<SliderAttachment>(p.apvts, "dir_outer_gain", offGainSlider_);

    initLabel(offLpLabel_, "Off-LP");
    initSlider(offLpSlider_);
    offLpSlider_.setTooltip("Off-axis low-pass at the cone edge (0 = bright, 1 = dark).");
    offLpAttachment_ = std::make_unique<SliderAttachment>(p.apvts, "dir_outer_lp", offLpSlider_);

    // Distance-curve preset combobox (sits above the curve editor).
    struct Preset { const char* name;
                    float aD, aDb, bD, bDb, cD, cDb, dD; };
    static const Preset kPresets[] = {
        {"Default",      1.00f,  0.00f, 12.00f,-20.00f, 60.00f,-60.00f,100.00f},
        {"Direct Sources",   0.00f, -2.47f,  2.06f, -2.75f, 19.31f,-38.93f, 25.36f},
        {"Water Sources",    2.50f,-15.79f,  4.66f,-13.27f,  8.98f,-23.16f, 41.16f},
        {"Birds",            5.13f,-10.53f,  7.53f,-19.58f, 19.95f,-37.46f, 25.63f},
        {"Near Field",       3.19f, -8.64f,  5.07f,-29.89f,  7.80f,-56.19f,  8.88f},
        {"Meeting Room",     1.52f, -0.01f,  2.34f, -3.17f,  4.32f, -6.96f, 17.95f},
        {"Big Hall",         1.20f, -0.34f,  4.60f,  3.44f,  8.82f, -1.44f, 17.95f},
        {"Users (Voice)",    1.52f, -0.01f, 21.62f,-42.25f, 38.81f,-44.25f, 41.00f},
        {"Main (Exp.)",      1.34f,  0.00f, 30.00f,-40.00f, 60.00f,-57.00f,150.00f},
        {"Secondary (Exp.)", 1.34f,  0.00f, 10.00f,-12.00f, 55.00f,-50.00f,150.00f},
    };
    initLabel(distPresetLabel_, "Preset");
    int presetId = 1;
    for (const auto& pr : kPresets)
        distPresetBox_.addItem(pr.name, presetId++);
    addAndMakeVisible(distPresetBox_);
    distPresetBox_.onChange = [this] {
        const int sel = distPresetBox_.getSelectedId() - 1;
        if (sel < 0 || sel >= (int) (sizeof(kPresets) / sizeof(Preset))) return;
        const auto& pr = kPresets[sel];
        auto setp = [this](const char* id, float v) {
            if (auto* prm = proc_.apvts.getParameter(id)) {
                prm->beginChangeGesture();
                prm->setValueNotifyingHost(prm->convertTo0to1(v));
                prm->endChangeGesture();
            }
        };
        setp("dist_a", pr.aD);   setp("dist_a_db", pr.aDb);
        setp("dist_b", pr.bD);   setp("dist_b_db", pr.bDb);
        setp("dist_c", pr.cD);   setp("dist_c_db", pr.cDb);
        setp("dist_d", pr.dD);
    };

    // ENVIRONMENT section.
    initLabel(occlusionLabel_, "Occlusion");
    initSlider(occlusionSlider_);
    occlusionSlider_.setTooltip("Wall thickness between source and listener. Drives a per-source low-pass.");
    occlusionAttachment_ = std::make_unique<SliderAttachment>(p.apvts, "occlusion", occlusionSlider_);

    initLabel(reverbSendLabel_, "Rev. Send");
    initSlider(reverbSendSlider_);
    reverbSendSlider_.setTooltip("Per-source send into the reverb bus. 0 = dry source.");
    reverbSendAttachment_ = std::make_unique<SliderAttachment>(p.apvts, "reverb_send", reverbSendSlider_);

    initLabel(reverbAmountLabel_, "Rev. Amount");
    initSlider(reverbAmountSlider_);
    reverbAmountSlider_.setTooltip("Master reverb mix multiplier. 0 = no reverb.");
    reverbAmountAttachment_ = std::make_unique<SliderAttachment>(p.apvts, "reverb_amount", reverbAmountSlider_);

    // OUTPUT section.
    initLabel(extAmountLabel_, "Ext. Amount");
    initSlider(extAmountSlider_);
    extAmountSlider_.setTooltip("Externalizer amount (0..100). 0 = off; higher = stronger out-of-head effect.");
    extAmountAttachment_ = std::make_unique<SliderAttachment>(p.apvts, "externalizer_amount", extAmountSlider_);

    stereoBypassButton_.setColour(juce::ToggleButton::textColourId, juce::Colour(0xffbbbbbb));
    stereoBypassButton_.setTooltip("Skip all spatial DSP - pass host stereo straight to output. Only Gain still applies.");
    addAndMakeVisible(stereoBypassButton_);
    stereoBypassAttachment_ =
        std::make_unique<ButtonAttachment>(p.apvts, "rendering_mode", stereoBypassButton_);

    // ADVANCED section (hidden by default).
    initLabel(directPathLabel_, "Direct");
    initSlider(directPathSlider_);
    directPathSlider_.setTooltip("Direct-path gain - multiplies the non-reverb signal only.");
    directPathAttachment_ = std::make_unique<SliderAttachment>(p.apvts, "direct_path_gain", directPathSlider_);

    initLabel(extCharLabel_, "Ext. Char.");
    initSlider(extCharSlider_);
    extCharSlider_.setTooltip("Externalizer tilt EQ (0..100, 50 = neutral). Below 50 = brighter; above 50 = darker.");
    extCharAttachment_ = std::make_unique<SliderAttachment>(p.apvts, "externalizer_character", extCharSlider_);

    // Advanced disclosure starts collapsed.
    auto setAdvVisible = [this](bool v) {
        directPathLabel_.setVisible(v);
        directPathSlider_.setVisible(v);
        extCharLabel_.setVisible(v);
        extCharSlider_.setVisible(v);
        advancedHeader_.setVisible(v);
    };
    setAdvVisible(false);

    advancedButton_.setTooltip("Show / hide additional source parameters (Direct, Ext. Character).");
    advancedButton_.setButtonText("Advanced " + kGlyphDown);
    advancedButton_.onClick = [this, setAdvVisible] {
        advancedOpen_ = !advancedOpen_;
        advancedButton_.setButtonText("Advanced " + (advancedOpen_ ? kGlyphUp : kGlyphDown));
        setAdvVisible(advancedOpen_);
        setSize(580, advancedOpen_ ? 784 : 760);
    };
    addAndMakeVisible(advancedButton_);

    resetButton_.setTooltip("Reset all parameters to defaults.");
    resetButton_.onClick = [this] { resetAllParams(); };
    addAndMakeVisible(resetButton_);

    setSize(580, 760);
}

SpatialAudioEditor::~SpatialAudioEditor() = default;

void SpatialAudioEditor::resetAllParams()
{
    constexpr const char* ids[] = {
        "distance", "azimuth", "elevation", "gain_db",
        "listener_x", "listener_y", "listener_z",
        "yaw", "pitch", "roll",
        "source_yaw", "source_pitch",
        "occlusion",
        "dir_inner_deg", "dir_outer_deg",
        "dir_outer_gain", "dir_outer_lp",
        "direct_path_gain",
        "reverb_send", "reverb_amount",
        "externalizer_amount", "externalizer_character",
        "dist_a", "dist_a_db", "dist_b", "dist_b_db",
        "dist_c", "dist_c_db", "dist_d",
        "position_mode", "rendering_mode",
        "aim_at_listener",
    };
    for (auto* id : ids)
    {
        if (auto* p = proc_.apvts.getParameter(id))
        {
            p->beginChangeGesture();
            p->setValueNotifyingHost(p->getDefaultValue());
            p->endChangeGesture();
        }
    }
}

void SpatialAudioEditor::paint(juce::Graphics& g)
{
    g.fillAll(juce::Colour(0xff0c0c0c));
}

void SpatialAudioEditor::layoutSliderRow(juce::Rectangle<int>& area, int rowH,
                                          juce::Label& label, juce::Slider& slider)
{
    auto r = area.removeFromTop(rowH);
    area.removeFromTop(2);
    label.setBounds(r.removeFromLeft(80));
    slider.setBounds(r);
}

void SpatialAudioEditor::layoutPairedRow(juce::Rectangle<int>& area, int rowH,
                                          juce::Label& l1, juce::Slider& s1,
                                          juce::Label& l2, juce::Slider& s2)
{
    auto r = area.removeFromTop(rowH);
    area.removeFromTop(2);
    const int halfW = r.getWidth() / 2;
    auto left  = r.removeFromLeft(halfW - 4);
    r.removeFromLeft(8);
    auto right = r;
    l1.setBounds(left.removeFromLeft(60));
    s1.setBounds(left);
    l2.setBounds(right.removeFromLeft(60));
    s2.setBounds(right);
}

void SpatialAudioEditor::resized()
{
    auto area = getLocalBounds().reduced(8);
    auto headerRect = [&area]() { area.removeFromTop(4); return area.removeFromTop(14); };

    // Reserve bottom row up-front so it can never be clipped by the
    // ADVANCED section overflowing downward.
    auto bottom = area.removeFromBottom(26);
    area.removeFromBottom(6);
    resetButton_.setBounds(bottom.removeFromRight(72));
    bottom.removeFromRight(6);
    advancedButton_.setBounds(bottom.removeFromRight(110));

    // ---- Top: compass + elev strip (square-ish canvas) ----
    auto canvas = area.removeFromTop(340);
    auto strip  = canvas.removeFromRight(90);
    compass_->setBounds(canvas);
    elevation_->setBounds(strip);
    area.removeFromTop(6);

    // ---- Gain + Aim toggle ----
    {
        auto r = area.removeFromTop(24);
        area.removeFromTop(2);
        gainLabel_.setBounds(r.removeFromLeft(60));
        aimAtListenerButton_.setBounds(r.removeFromRight(124));
        r.removeFromRight(4);
        gainSlider_.setBounds(r);
    }

    // ---- SHAPE ----
    shapeHeader_.setBounds(headerRect());
    {
        auto graph = area.removeFromTop(140);
        area.removeFromTop(2);
        auto presetRow = graph.removeFromBottom(22);
        graph.removeFromBottom(2);
        distPresetLabel_.setBounds(presetRow.removeFromLeft(60));
        distPresetBox_.setBounds(presetRow);
        curveEditor_->setBounds(graph);
    }
    layoutPairedRow(area, 22, spreadLabel_, spreadSlider_, focusLabel_, focusSlider_);
    layoutPairedRow(area, 22, offGainLabel_, offGainSlider_, offLpLabel_, offLpSlider_);

    // ---- ENVIRONMENT ----
    environmentHeader_.setBounds(headerRect());
    layoutSliderRow(area, 22, occlusionLabel_, occlusionSlider_);
    layoutPairedRow(area, 22, reverbSendLabel_, reverbSendSlider_,
                              reverbAmountLabel_, reverbAmountSlider_);

    // ---- OUTPUT ----
    outputHeader_.setBounds(headerRect());
    {
        auto r = area.removeFromTop(22);
        area.removeFromTop(2);
        extAmountLabel_.setBounds(r.removeFromLeft(60));
        stereoBypassButton_.setBounds(r.removeFromRight(124));
        r.removeFromRight(4);
        extAmountSlider_.setBounds(r);
    }

    // ---- ADVANCED (when open) ----
    if (advancedOpen_)
    {
        advancedHeader_.setBounds(headerRect());
        layoutPairedRow(area, 22, directPathLabel_, directPathSlider_,
                                  extCharLabel_, extCharSlider_);
    }

}
