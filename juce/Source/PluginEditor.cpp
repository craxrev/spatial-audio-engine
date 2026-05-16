#include "PluginEditor.h"

#include <cmath>

namespace
{
constexpr float kCompassMaxMeters = 25.0f;
constexpr float kTwoPi = 6.283185307f;

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
        startTimerHz(60);
    }

    ~SpatialCompass() override { stopTimer(); }

    void paint(juce::Graphics& g) override
    {
        const auto bounds = getLocalBounds().toFloat();
        const auto centre = bounds.getCentre();
        const float outerR =
            juce::jmin(bounds.getWidth(), bounds.getHeight()) * 0.45f;

        g.fillAll(juce::Colour(0xff141414));

        // Distance rings every 5 m.
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

        // Cardinal axes + labels.
        g.setColour(juce::Colour(0xff262626));
        g.drawLine(centre.x, centre.y - outerR, centre.x, centre.y + outerR);
        g.drawLine(centre.x - outerR, centre.y, centre.x + outerR, centre.y);

        g.setColour(juce::Colour(0xff7a7a7a));
        g.setFont(juce::Font(juce::FontOptions(11.0f)));
        g.drawText("FRONT", juce::Rectangle<float>(centre.x - 40, centre.y - outerR - 16, 80, 12),
                   juce::Justification::centred);
        g.drawText("BACK",  juce::Rectangle<float>(centre.x - 40, centre.y + outerR + 4,  80, 12),
                   juce::Justification::centred);
        // LEFT / RIGHT live just *inside* the outer ring — the ring touches
        // the compass edges, so there's no margin outside it.
        g.drawText("LEFT",  juce::Rectangle<float>(centre.x - outerR + 6, centre.y - 6, 40, 12),
                   juce::Justification::centredLeft);
        g.drawText("RIGHT", juce::Rectangle<float>(centre.x + outerR - 46, centre.y - 6, 40, 12),
                   juce::Justification::centredRight);

        // Listener head.
        const juce::Colour headFill (0xff5a82ff);
        g.setColour(headFill);
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
        g.setFont(juce::Font(juce::FontOptions(10.0f)));
        g.drawText("YOU",
                   juce::Rectangle<float>(centre.x - 30.0f, centre.y + 18.0f, 60.0f, 12.0f),
                   juce::Justification::centred);

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
        g.setFont(juce::Font(juce::FontOptions(10.0f)));
        const bool above = src.y > bounds.getBottom() - 22.0f;
        const float labelY = above ? src.y - 22.0f : src.y + 10.0f;
        g.drawText("SOURCE",
                   juce::Rectangle<float>(src.x - 30.0f, labelY, 60.0f, 12.0f),
                   juce::Justification::centred);

        // Top-left readouts.
        static const juce::String deg = juce::String::fromUTF8("\xC2\xB0");
        static const juce::String mid = juce::String::fromUTF8("\xC2\xB7");
        static const juce::String dash = juce::String::fromUTF8("\xE2\x80\x94");
        static const juce::String cmd  = juce::String::fromUTF8("\xE2\x8C\x98");
        g.setColour(juce::Colour(0xff9a9a9a));
        g.setFont(juce::Font(juce::FontOptions(11.0f)));
        const auto info = juce::String("pos: ") + juce::String(dist, 2) + " m  "
                        + juce::String(az, 1) + deg + " (" + azimuthCardinal(az) + ")   yaw: "
                        + juce::String(yaw, 1) + deg;
        g.drawText(info, juce::Rectangle<int>(8, 8, getWidth() - 16, 14),
                   juce::Justification::topLeft);
        g.setColour(juce::Colour(0xff666666));
        g.setFont(juce::Font(juce::FontOptions(10.0f)));
        const auto hint = juce::String("drag dot to move ") + mid + " arrow to aim  "
                        + dash + "  hold " + cmd + " to snap";
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
            case DragTarget::Heading: setTooltip("Source yaw — drag to aim"); break;
            case DragTarget::Source:  setTooltip("Source position — drag to move"); break;
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
        repaint();
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

    // Radius at which the source level falls below `thresholdLin`, in
    // metres, for a listener at angle `angleFromForwardRad` off the
    // source's forward axis. Combines gain × direct × directivity_gain.
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
        return strength / thresholdLin;
    }

    juce::Path buildAudibilityContour(juce::Point<float> src, float yawDeg, float thresholdLin) const
    {
        juce::Path p;
        constexpr int N = 128;
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

    juce::AudioProcessorValueTreeState& state_;
    DragTarget activeDrag_ = DragTarget::None;
    float displayedYaw_       = 0.0f;
    float displayedOcclusion_ = 0.0f;
    bool  firstTick_          = true;
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
        setTooltip(juce::String::fromUTF8("Elevation (position) on the left \xC2\xB7 pitch (orientation) on the right"));
        startTimerHz(30);
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

    void timerCallback() override { repaint(); }

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
        static const juce::String deg = juce::String::fromUTF8("\xC2\xB0");

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
        g.setFont(juce::Font(juce::FontOptions(10.0f)));
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
        g.setFont(juce::Font(juce::FontOptions(10.0f)));
        g.drawText(juce::String((int) std::round(el)) + deg,
                   juce::Rectangle<float>(col.getX(), botY + 16.0f, col.getWidth(), 12.0f),
                   juce::Justification::centred);
    }

    void drawPitchColumn(juce::Graphics& g, juce::Rectangle<float> col)
    {
        static const juce::String deg = juce::String::fromUTF8("\xC2\xB0");

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
        g.setFont(juce::Font(juce::FontOptions(10.0f)));
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
        g.setFont(juce::Font(juce::FontOptions(10.0f)));
        g.drawText(juce::String((int) std::round(pitch)) + deg,
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
};

// ---------------------------------------------------------------------------
// SpatialAudioEditor.
// ---------------------------------------------------------------------------

SpatialAudioEditor::SpatialAudioEditor(SpatialAudioProcessor& p)
    : AudioProcessorEditor(p), proc_(p)
{
    compass_   = std::make_unique<SpatialCompass>(p.apvts);
    elevation_ = std::make_unique<ElevationStrip>(p.apvts);
    addAndMakeVisible(*compass_);
    addAndMakeVisible(*elevation_);

    auto initLinearSlider = [this](juce::Slider& s)
    {
        s.setSliderStyle(juce::Slider::LinearHorizontal);
        s.setTextBoxStyle(juce::Slider::TextBoxRight, false, 64, 18);
        addAndMakeVisible(s);
    };
    auto initLeftLabel = [this](juce::Label& l, const char* text)
    {
        l.setText(text, juce::dontSendNotification);
        l.setColour(juce::Label::textColourId, juce::Colour(0xffbbbbbb));
        l.setJustificationType(juce::Justification::centredRight);
        addAndMakeVisible(l);
    };

    initLeftLabel(gainLabel_, "Gain");
    initLinearSlider(gainSlider_);
    gainSlider_.setTooltip("Source gain (dB).");
    gainAttachment_ = std::make_unique<SliderAttachment>(p.apvts, "gain_db", gainSlider_);

    initLeftLabel(occlusionLabel_, "Occlusion");
    initLinearSlider(occlusionSlider_);
    occlusionSlider_.setTooltip("Occlusion (0..1): wall thickness between source and listener. Smoothed; drives a per-source low-pass.");
    occlusionAttachment_ =
        std::make_unique<SliderAttachment>(p.apvts, "occlusion", occlusionSlider_);

    initLeftLabel(spreadLabel_, "Spread");
    initLinearSlider(spreadSlider_);
    spreadSlider_.setTooltip(juce::String::fromUTF8("Cone outer angle: angle from forward at which the source becomes fully off-axis. 0\xC2\xB0 = pencil beam; 180\xC2\xB0 = omnidirectional."));
    spreadAttachment_ =
        std::make_unique<SliderAttachment>(p.apvts, "dir_outer_deg", spreadSlider_);

    initLeftLabel(focusLabel_, "Focus");
    initLinearSlider(focusSlider_);
    focusSlider_.setTooltip(juce::String::fromUTF8("Cone inner angle: width of the full-volume zone in front of the source. 0\xC2\xB0 = no sweet spot; matches Spread = hard edge."));
    focusAttachment_ =
        std::make_unique<SliderAttachment>(p.apvts, "dir_inner_deg", focusSlider_);

    initLeftLabel(offGainLabel_, "Off-Gain");
    initLinearSlider(offGainSlider_);
    offGainSlider_.setTooltip("Off-axis gain: how loud the source is at the cone edge (1 = no attenuation, 0 = silent).");
    offGainAttachment_ =
        std::make_unique<SliderAttachment>(p.apvts, "dir_outer_gain", offGainSlider_);

    initLeftLabel(offLpLabel_, "Off-LP");
    initLinearSlider(offLpSlider_);
    offLpSlider_.setTooltip("Off-axis low-pass: how muffled the source is at the cone edge (0 = bright, 1 = dark).");
    offLpAttachment_ =
        std::make_unique<SliderAttachment>(p.apvts, "dir_outer_lp", offLpSlider_);

    initLeftLabel(directPathLabel_, "Direct");
    initLinearSlider(directPathSlider_);
    directPathSlider_.setTooltip("Direct path gain — multiplies the direct (non-reverb) signal only.");
    directPathAttachment_ =
        std::make_unique<SliderAttachment>(p.apvts, "direct_path_gain", directPathSlider_);

    initLeftLabel(reverbSendLabel_, "Reverb Send");
    initLinearSlider(reverbSendSlider_);
    reverbSendSlider_.setTooltip("Per-source send into the reverb bus. 0 = dry source.");
    reverbSendAttachment_ =
        std::make_unique<SliderAttachment>(p.apvts, "reverb_send", reverbSendSlider_);

    initLeftLabel(reverbAmountLabel_, "Reverb Amount");
    initLinearSlider(reverbAmountSlider_);
    reverbAmountSlider_.setTooltip("Master reverb mix multiplier. 0 = no reverb, 1 = unity.");
    reverbAmountAttachment_ =
        std::make_unique<SliderAttachment>(p.apvts, "reverb_amount", reverbAmountSlider_);

    initLeftLabel(extAmountLabel_, "Ext. Amount");
    initLinearSlider(extAmountSlider_);
    extAmountSlider_.setTooltip("Externalizer amount (0..100). 0 = off; higher = stronger out-of-head effect (and more signal attenuation).");
    extAmountAttachment_ =
        std::make_unique<SliderAttachment>(p.apvts, "externalizer_amount", extAmountSlider_);

    initLeftLabel(extCharLabel_, "Ext. Character");
    initLinearSlider(extCharSlider_);
    extCharSlider_.setTooltip("Externalizer tilt EQ (0..100, 50 = neutral). Below 50 = brighter; above 50 = darker.");
    extCharAttachment_ =
        std::make_unique<SliderAttachment>(p.apvts, "externalizer_character", extCharSlider_);

    resetButton_.setTooltip("Reset all parameters to defaults.");
    resetButton_.onClick = [this] { resetAllParams(); };
    addAndMakeVisible(resetButton_);

    aimAtListenerButton_.setTooltip("Lock source orientation to face the listener.");
    aimAtListenerButton_.setColour(juce::ToggleButton::textColourId, juce::Colour(0xffbbbbbb));
    addAndMakeVisible(aimAtListenerButton_);
    aimAttachment_ = std::make_unique<ButtonAttachment>(
        p.apvts, "aim_at_listener", aimAtListenerButton_);

    setSize(520, 800);
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

void SpatialAudioEditor::resized()
{
    auto area = getLocalBounds().reduced(10);

    auto row = [&area](int h) { auto r = area.removeFromBottom(h); area.removeFromBottom(4); return r; };
    auto layoutSlider = [](juce::Rectangle<int> r, juce::Label& l, juce::Slider& s)
    {
        l.setBounds(r.removeFromLeft(80));
        s.setBounds(r);
    };

    auto bottom = row(28);
    gainLabel_.setBounds(bottom.removeFromLeft(80));
    resetButton_.setBounds(bottom.removeFromRight(72));
    bottom.removeFromRight(6);
    aimAtListenerButton_.setBounds(bottom.removeFromRight(124));
    bottom.removeFromRight(6);
    gainSlider_.setBounds(bottom);

    layoutSlider(row(24), extCharLabel_,      extCharSlider_);
    layoutSlider(row(24), extAmountLabel_,    extAmountSlider_);
    layoutSlider(row(24), reverbAmountLabel_, reverbAmountSlider_);
    layoutSlider(row(24), reverbSendLabel_,   reverbSendSlider_);
    layoutSlider(row(24), directPathLabel_,   directPathSlider_);
    layoutSlider(row(24), offLpLabel_,        offLpSlider_);
    layoutSlider(row(24), offGainLabel_,      offGainSlider_);
    layoutSlider(row(24), focusLabel_,        focusSlider_);
    layoutSlider(row(24), spreadLabel_,       spreadSlider_);
    layoutSlider(row(24), occlusionLabel_,    occlusionSlider_);

    area.removeFromBottom(6);

    auto strip = area.removeFromRight(100);
    compass_->setBounds(area);
    elevation_->setBounds(strip);
}
