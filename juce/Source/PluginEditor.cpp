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

// Cached monospace fonts. The CRT-phosphor aesthetic demands a
// terminal/instrument feel — Menlo is bundled on every macOS install
// and renders cleanly at small sizes. Fallback to system monospace
// metric on other platforms.
const juce::Font& mono9()  { static juce::Font f = juce::Font(juce::FontOptions("Menlo", 9.5f,  juce::Font::plain)); return f; }
const juce::Font& mono10() { static juce::Font f = juce::Font(juce::FontOptions("Menlo", 10.5f, juce::Font::plain)); return f; }
const juce::Font& mono11() { static juce::Font f = juce::Font(juce::FontOptions("Menlo", 11.5f, juce::Font::plain)); return f; }
// Keep the old names as aliases so call-sites don't all change.
inline const juce::Font& font9()  { return mono9();  }
inline const juce::Font& font10() { return mono10(); }
inline const juce::Font& font11() { return mono11(); }

// =============================================================================
// Theme — "CRT Phosphor" palette.
// Pure monochrome phosphor green on near-black, vector wireframes,
// glow halos. Aesthetic reference: 1970s radar / sonar consoles,
// PDP-11 oscilloscope displays, terminal CRTs. Everything is a thin
// stroke; nothing is a filled gradient. The single accent dimension
// is *intensity* — same hue, different brightness for hierarchy.
// =============================================================================
namespace theme
{
    // -- Surfaces ------------------------------------------------------
    constexpr juce::uint32 bg0        = 0xff1a0d00; // near-black with amber-tube tint
    constexpr juce::uint32 bg1        = 0xff0e0700; // panel underlay (curve / elev strip)
    constexpr juce::uint32 bg2        = 0xff2a1700; // raised surface (elev strip tracks)

    // -- Phosphor intensity ramp ---------------------------------------
    // The whole UI is shades of one hue. Intensity = hierarchy.
    // Amber P3 phosphor — warm sunset orange, classic 1980s terminal.
    constexpr juce::uint32 phosphor0  = 0xff3a1f00; // deepest — barely visible grid
    constexpr juce::uint32 phosphor1  = 0xff7a4400; // dim — distance rings, faint structure
    constexpr juce::uint32 phosphor2  = 0xffc97a08; // mid — text, axis lines
    constexpr juce::uint32 phosphor3  = 0xffffa429; // bright — emphasis, labels
    constexpr juce::uint32 phosphor4  = 0xffffcc66; // peak — source, active drag, headline
    constexpr juce::uint32 glow       = 0x66ffa429; // halo (alpha-blended)
    constexpr juce::uint32 glowFaint  = 0x33ffa429; // faint halo

    // -- Semantic aliases (so the rest of the file reads cleanly) ------
    constexpr juce::uint32 gridFaint  = phosphor0;
    constexpr juce::uint32 gridMid    = phosphor1;
    constexpr juce::uint32 gridStrong = phosphor2;
    constexpr juce::uint32 text       = phosphor2;
    constexpr juce::uint32 textBright = phosphor4;
    constexpr juce::uint32 textDim    = phosphor1;
    constexpr juce::uint32 textDimmer = phosphor0;

    // -- Source (peak phosphor) ----------------------------------------
    constexpr juce::uint32 src        = phosphor4;
    constexpr juce::uint32 srcLight   = 0xffffe6b8;
    constexpr juce::uint32 srcDeep    = phosphor2;
    constexpr juce::uint32 srcGlow    = glow;
    constexpr juce::uint32 srcWedge   = 0x22ffa429;
    constexpr juce::uint32 srcArrowTr = 0xccffcc66;

    // -- Listener (dim phosphor — observer is structural, not bright) --
    constexpr juce::uint32 lst        = phosphor1;
    constexpr juce::uint32 lstLight   = phosphor2;
    constexpr juce::uint32 lstDeep    = phosphor0;

    // -- Effects -------------------------------------------------------
    constexpr juce::uint32 audibleWarm = phosphor3; // contour stronger phosphor
    constexpr juce::uint32 audibleCool = phosphor1; // contour off-axis dim
    constexpr juce::uint32 occlusion   = phosphor2; // fog tinted phosphor
    constexpr juce::uint32 curveFill   = 0x22ffa429;
    constexpr juce::uint32 curveLine   = phosphor4;
    constexpr juce::uint32 nodeActive  = 0xffffe6b8;
    constexpr juce::uint32 scanline    = 0x0a000000; // subtle dark overlay for scanlines
}

// CRT scanline overlay — every-other horizontal line painted with a
// low-alpha dark stripe. Cheap (just N drawLine calls) and adds the
// signature "this is a phosphor display, not a flat-panel" cue.
inline void paintScanlines(juce::Graphics& g, juce::Rectangle<int> bounds)
{
    g.setColour(juce::Colour(theme::scanline));
    for (int y = bounds.getY(); y < bounds.getBottom(); y += 2)
        g.drawHorizontalLine(y, (float)bounds.getX(), (float)bounds.getRight());
}

// Corner vignette — fade to black at the four corners to suggest CRT
// screen curvature / instrument bezel. Adds depth and frames the
// content. Drawn last (over the scanlines on the static bg).
inline void paintVignette(juce::Graphics& g, juce::Rectangle<int> bounds)
{
    const auto centre = bounds.getCentre().toFloat();
    const float r = juce::jmin(bounds.getWidth(), bounds.getHeight()) * 0.5f;
    juce::ColourGradient grad(
        juce::Colour(0x00000000), centre.x, centre.y,
        juce::Colour(0xb0000000), centre.x + r * 1.15f, centre.y, true);
    grad.addColour(0.65, juce::Colour(0x00000000));
    grad.addColour(0.92, juce::Colour(0x55000000));
    g.setGradientFill(grad);
    g.fillRect(bounds);
}


// Draw an ellipse with a soft phosphor glow halo. `r` is the dot
// radius; the glow extends to ~2.2r and fades out.
inline void drawPhosphorDot(juce::Graphics& g, juce::Point<float> p, float r,
                             juce::uint32 fill, juce::uint32 halo)
{
    const float h = r * 2.2f;
    g.setColour(juce::Colour(halo));
    g.fillEllipse(p.x - h, p.y - h, 2 * h, 2 * h);
    g.setColour(juce::Colour(halo).withMultipliedAlpha(0.55f));
    g.fillEllipse(p.x - h * 1.45f, p.y - h * 1.45f, 2 * h * 1.45f, 2 * h * 1.45f);
    g.setColour(juce::Colour(fill));
    g.fillEllipse(p.x - r, p.y - r, 2 * r, 2 * r);
}

// Draw a line with a phosphor glow underlay. Used for the heading
// arrow and other emphasis strokes.
inline void drawPhosphorLine(juce::Graphics& g,
                              juce::Point<float> a, juce::Point<float> b,
                              float stroke, juce::uint32 fill, juce::uint32 halo)
{
    g.setColour(juce::Colour(halo));
    g.drawLine(a.x, a.y, b.x, b.y, stroke * 3.0f);
    g.setColour(juce::Colour(halo).withMultipliedAlpha(0.6f));
    g.drawLine(a.x, a.y, b.x, b.y, stroke * 1.8f);
    g.setColour(juce::Colour(fill));
    g.drawLine(a.x, a.y, b.x, b.y, stroke);
}

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
    SpatialCompass(juce::AudioProcessorValueTreeState& s,
                   std::function<float()>               headYawDegProvider,
                   std::function<int()>                 headStatusProvider,
                   std::function<juce::uint64()>        headFrameIdProvider,
                   std::function<Quat()>                headQuatProvider)
        : state_(s),
          headYawDeg_(std::move(headYawDegProvider)),
          headStatus_(std::move(headStatusProvider)),
          headFrameId_(std::move(headFrameIdProvider)),
          headQuat_(std::move(headQuatProvider))
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

        // Corner vignette — painted UNDER the dynamic content so it
        // darkens only the background corners, not the text/source.
        paintVignette(g, getLocalBounds());

        // Linked stereo pair. `src` is the midpoint at (azimuth, distance);
        // L and R fan out by ±width/2 around it on the same distance ring.
        const float dist  = currentDistance();
        const float az    = currentAzimuth();
        const float wHalf = currentWidth() * 0.5f;

        // World-frame rotation: rotate everything painted in world coordinates
        // by -headYaw around the (fixed) listener at centre. Sources, target,
        // contours, arrow → all stay world-locked while the listener arrow,
        // rings, ticks and cardinal labels stay head-relative.
        const float headYaw = headYawDeg_ ? headYawDeg_() : 0.0f;
        const bool  rotateWorld = std::abs(headYaw) > 1e-3f;
        {
        juce::Graphics::ScopedSaveState worldFrame(g);
        if (rotateWorld)
            // +headYaw CW: head-left turn rotates world CW relative to the
            // fixed listener arrow, so a source at world-front (compass-up)
            // visually slides to compass-right.
            g.addTransform(juce::AffineTransform::rotation(
                juce::degreesToRadians(headYaw), centre.x, centre.y));
        const auto  src   = azimDistToScreen(centre, outerR, az,         dist, kCompassMaxMeters);
        const auto  srcL  = azimDistToScreen(centre, outerR, az + wHalf, dist, kCompassMaxMeters);
        const auto  srcR  = azimDistToScreen(centre, outerR, az - wHalf, dist, kCompassMaxMeters);

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
                    g.setColour(juce::Colour(theme::occlusion).withAlpha(a));
                    g.fillEllipse(p.x - r, p.y - r, 2.0f * r, 2.0f * r);
                }
            }
        }

        // World-locked target the pair aims at. Aim ON → listener
        // position (locked mode); Aim OFF → user-set target_(x,y,z).
        const bool  aimOn = state_.getRawParameterValue("aim_at_listener")->load() > 0.5f;
        const float tgtX  = aimOn ? state_.getRawParameterValue("listener_x")->load()
                                  : state_.getRawParameterValue("target_x")->load();
        const float tgtY  = aimOn ? state_.getRawParameterValue("listener_y")->load()
                                  : state_.getRawParameterValue("target_y")->load();
        // Native → compass screen pixel mapping (matches azimDistToScreen):
        //   screen = centre + (-y_native, -x_native) · (outerR / kCompassMaxMeters)
        const float pxPerM = outerR / kCompassMaxMeters;
        const juce::Point<float> tgtScreen {
            centre.x - tgtY * pxPerM,
            centre.y - tgtX * pxPerM
        };

        // Native position of each L/R virtual speaker (z = elevation,
        // ignored for compass; elevation has its own strip).
        const float elRad = juce::degreesToRadians(state_.getRawParameterValue("elevation")->load());
        const float ce    = std::cos(elRad);
        const float aLRad = juce::degreesToRadians(az + wHalf);
        const float aRRad = juce::degreesToRadians(az - wHalf);
        const float lAtmX = dist * ce * std::cos(aLRad);
        const float lAtmY = dist * ce * std::sin(aLRad);
        const float rAtmX = dist * ce * std::cos(aRRad);
        const float rAtmY = dist * ce * std::sin(aRRad);

        // Per-source contour yaw (compass-frame degrees, matching
        // compassDir convention) = atan2(delta.native.y, delta.native.x)
        // of the source→target vector, in degrees.
        auto yawDegFromNativeDelta = [](float dx, float dy) {
            return juce::radiansToDegrees(std::atan2(dy, dx));
        };
        const float yawL = yawDegFromNativeDelta(tgtX - lAtmX, tgtY - lAtmY);
        const float yawR = yawDegFromNativeDelta(tgtX - rAtmX, tgtY - rAtmY);

        const float outerLp = state_.getRawParameterValue("dir_outer_lp")->load();
        const juce::Colour warm  (theme::audibleWarm);
        const juce::Colour slate (theme::audibleCool);
        const auto tint = warm.interpolatedWith(slate, juce::jlimit(0.0f, 1.0f, outerLp));

        constexpr float kAudibleThresh = 0.0631f; // −24 dB
        constexpr float kLoudThresh    = 0.5f;    // −6 dB

        auto drawSourceContour = [&](juce::Point<float> pos, float yawDeg) {
            g.setColour(tint.withAlpha(0.22f));
            g.strokePath(buildAudibilityContour(pos, yawDeg, kAudibleThresh),
                         juce::PathStrokeType(0.8f));
            g.setColour(tint.withAlpha(0.55f));
            g.strokePath(buildAudibilityContour(pos, yawDeg, kLoudThresh),
                         juce::PathStrokeType(1.2f));
        };
        drawSourceContour(srcL, yawL);
        drawSourceContour(srcR, yawR);

        // Arrow from the midpoint to the target tip. Drag-target when
        // Aim at listener is off; auto-points at the listener when on.
        const auto arrowTip = tgtScreen;
        const auto arrowVec = arrowTip - src;
        const float arrowLen = arrowVec.getDistanceFromOrigin();
        if (arrowLen > 1.0f)
        {
            drawPhosphorLine(g, src, arrowTip, 1.8f, theme::src, theme::glow);
            const auto dir = arrowVec / arrowLen;
            const juce::Point<float> perp { -dir.y, dir.x };
            juce::Path head;
            head.addTriangle(arrowTip.x + dir.x * 6.0f,  arrowTip.y + dir.y * 6.0f,
                             arrowTip.x - dir.x * 3.0f + perp.x * 5.0f,
                             arrowTip.y - dir.y * 3.0f + perp.y * 5.0f,
                             arrowTip.x - dir.x * 3.0f - perp.x * 5.0f,
                             arrowTip.y - dir.y * 3.0f - perp.y * 5.0f);
            g.setColour(juce::Colour(theme::glow));
            g.fillPath(head, juce::AffineTransform::scale(1.4f, 1.4f, arrowTip.x, arrowTip.y));
            g.setColour(juce::Colour(theme::src));
            g.fillPath(head);
        }

        // Connecting arc along the constant-distance ring between L and R.
        // Drawn on the same circle the dots ride; visually anchors them
        // as one linked pair rather than two independent sources.
        {
            const float ringR = metersToPixels(dist);
            if (ringR > 1.0f && currentWidth() > 0.01f)
            {
                // Compass convention (matches azimDistToScreen):
                //   pt = centre + (-sin(a), -cos(a)) · r
                // so 0° = up (front), +90° = screen-left, -90° = right.
                const float aL = juce::degreesToRadians(az + wHalf);
                const float aR = juce::degreesToRadians(az - wHalf);
                juce::Path arc;
                const int steps = juce::jmax(8, (int) (currentWidth() * 0.5f));
                for (int i = 0; i <= steps; ++i)
                {
                    const float t = (float) i / (float) steps;
                    const float a = aR + (aL - aR) * t;
                    const juce::Point<float> p {
                        centre.x - std::sin(a) * ringR,
                        centre.y - std::cos(a) * ringR
                    };
                    if (i == 0) arc.startNewSubPath(p); else arc.lineTo(p);
                }
                g.setColour(juce::Colour(theme::src).withAlpha(0.45f));
                g.strokePath(arc, juce::PathStrokeType(1.2f));
            }
        }

        // L and R virtual-speaker blips with glow halos. Same phosphor
        // styling as the original single-source dot.
        drawPhosphorDot(g, srcL, 6.0f, theme::src, theme::glow);
        drawPhosphorDot(g, srcR, 6.0f, theme::src, theme::glow);
        g.setColour(juce::Colour(theme::phosphor4));
        g.drawEllipse(srcL.x - 10.0f, srcL.y - 10.0f, 20.0f, 20.0f, 1.2f);
        g.drawEllipse(srcR.x - 10.0f, srcR.y - 10.0f, 20.0f, 20.0f, 1.2f);

        g.setColour(juce::Colour(theme::srcLight));
        g.setFont(font10());
        auto labelFor = [&](const juce::Point<float>& p, const char* tag) {
            const bool above = p.y > bounds.getBottom() - 22.0f;
            const float ly   = above ? p.y - 22.0f : p.y + 12.0f;
            g.drawText(tag,
                       juce::Rectangle<float>(p.x - 30.0f, ly, 60.0f, 12.0f),
                       juce::Justification::centred);
        };
        labelFor(srcL, "L");
        labelFor(srcR, "R");
        }  // end world-frame block

        // Top-left readouts — monospace technical readout.
        g.setColour(juce::Colour(theme::phosphor3));
        g.setFont(font11());
        const int stat = headStatus_ ? headStatus_() : 0;
        const char* statStr = "OFF";
        switch (stat) {
            case 0: statStr = "DISC"; break;
            case 1: statStr = "CONN"; break;
            case 2: statStr = "STR";  break;
            case 3: statStr = "FAIL"; break;
        }
        const juce::uint64 fid = headFrameId_ ? headFrameId_() : 0;
        const Quat hq = headQuat_ ? headQuat_() : Quat::identity();
        const auto info = juce::String("R ") + juce::String(dist, 2) + "M  AZ "
                        + juce::String(az, 1) + kGlyphDeg
                        + "  HEAD " + juce::String(headYaw, 1) + kGlyphDeg
                        + "  " + statStr
                        + "  FID " + juce::String((juce::int64)fid);
        g.drawText(info, juce::Rectangle<int>(8, 8, getWidth() - 16, 14),
                   juce::Justification::topLeft);
        const auto qline = juce::String("Q ")
                        + juce::String(hq.w, 3) + " "
                        + juce::String(hq.x, 3) + " "
                        + juce::String(hq.y, 3) + " "
                        + juce::String(hq.z, 3);
        g.setColour(juce::Colour(theme::phosphor2));
        g.setFont(font10());
        g.drawText(qline, juce::Rectangle<int>(8, getHeight() - 22, getWidth() - 16, 12),
                   juce::Justification::bottomLeft);
        g.setColour(juce::Colour(theme::phosphor2));
        g.setFont(font10());
        // Use ASCII "Cmd" — the U+2318 ⌘ glyph is missing in many fonts.
        const auto hint = juce::String("DRAG DOT TO MOVE ") + kGlyphMid + " ARROW TO AIM  "
                        + kGlyphDash + "  HOLD CMD TO SNAP";
        g.drawText(hint,
                   juce::Rectangle<int>(8, 22, getWidth() - 16, 12),
                   juce::Justification::topLeft);

        // Scanline overlay sits on top so all phosphor elements (text,
        // source, contour) get the CRT line pattern.
        paintScanlines(g, getLocalBounds());
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
        // Smoothed "displayed yaw" = the arrow's screen direction (from
        // midpoint to target tip), used only for the top-left info text.
        // Engine aim is driven directly by target_(x,y,z) via the
        // processor, not by this value.
        const auto srcPx = sourceScreen();
        const auto tgtPx = targetScreen();
        const auto vArr  = tgtPx - srcPx;
        const float arrowYaw = vArr.getDistanceFromOrigin() > 1.0f
            ? compassAngleDeg(vArr)
            : displayedYaw_;

        const float poccl = state_.getRawParameterValue("occlusion")->load();
        const float prevDispYaw  = displayedYaw_;
        const float prevDispOccl = displayedOcclusion_;
        if (firstTick_)
        {
            displayedYaw_       = arrowYaw;
            displayedOcclusion_ = poccl;
            firstTick_          = false;
        }
        else
        {
            displayedYaw_       = wrap180(angleLerp(displayedYaw_, arrowYaw, 0.25f));
            displayedOcclusion_ += (poccl - displayedOcclusion_) * 0.18f;
        }

        // Conditional repaint: skip when nothing has changed since last
        // frame. Tracks contour-shape inputs (source pos, directivity,
        // gain, distance curve, target tip) + smoothed values + drag state.
        const float snap[20] = {
            currentDistance(),
            currentAzimuth(),
            currentWidth(),
            state_.getRawParameterValue("elevation")     ->load(),
            state_.getRawParameterValue("target_x")      ->load(),
            state_.getRawParameterValue("target_y")      ->load(),
            state_.getRawParameterValue("listener_x")    ->load(),
            state_.getRawParameterValue("listener_y")    ->load(),
            state_.getRawParameterValue("aim_at_listener")->load(),
            state_.getRawParameterValue("dir_inner_deg") ->load(),
            state_.getRawParameterValue("dir_outer_deg") ->load(),
            state_.getRawParameterValue("dir_outer_gain")->load(),
            state_.getRawParameterValue("dir_outer_lp")  ->load(),
            state_.getRawParameterValue("gain_db")       ->load(),
            state_.getRawParameterValue("direct_path_gain")->load(),
            state_.getRawParameterValue("dist_a")        ->load(),
            state_.getRawParameterValue("dist_a_db")     ->load(),
            state_.getRawParameterValue("dist_b")        ->load(),
            state_.getRawParameterValue("dist_b_db")     ->load(),
            state_.getRawParameterValue("dist_c")        ->load(),
        };
        // Always repaint while debugging head-tracking — FID/STAT/HEAD readout
        // should refresh live regardless of conditional gates.
        const float headYawNow = headYawDeg_ ? headYawDeg_() : 0.0f;
        lastHeadYaw_ = headYawNow;
        (void)prevDispYaw; (void)prevDispOccl;
        for (int i = 0; i < 20; ++i) prevSnap_[i] = snap[i];
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

    juce::Point<float> targetScreen() const
    {
        const bool aimOn = state_.getRawParameterValue("aim_at_listener")->load() > 0.5f;
        const float tx = aimOn ? state_.getRawParameterValue("listener_x")->load()
                               : state_.getRawParameterValue("target_x")->load();
        const float ty = aimOn ? state_.getRawParameterValue("listener_y")->load()
                               : state_.getRawParameterValue("target_y")->load();
        const float pxPerM = outerRadius() / kCompassMaxMeters;
        const auto c = centre();
        return juce::Point<float>(c.x - ty * pxPerM, c.y - tx * pxPerM);
    }

    float currentDistance() const { return state_.getRawParameterValue("distance")->load(); }
    float currentAzimuth()  const { return state_.getRawParameterValue("azimuth")->load(); }
    float currentWidth()    const { return state_.getRawParameterValue("width")->load(); }

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
        // sourceScreen()/targetScreen() return world-frame coords. Painting
        // applies +headYaw rotation around centre; apply the same here so
        // the click-target matches what the user sees.
        const float yawRad = juce::degreesToRadians(headYawDeg_ ? headYawDeg_() : 0.0f);
        const auto c = centre();
        const float cy = std::cos(yawRad), sy = std::sin(yawRad);
        auto rotate = [&](juce::Point<float> q) {
            const float dx = q.x - c.x, dy = q.y - c.y;
            return juce::Point<float>(c.x + cy * dx - sy * dy,
                                      c.y + sy * dx + cy * dy);
        };
        const auto src     = rotate(sourceScreen());
        const auto headTip = rotate(targetScreen());

        if (p.getDistanceFrom(headTip) <= 9.0f) return DragTarget::Heading;
        if (p.getDistanceFrom(src)     <= 14.0f) return DragTarget::Source;
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
        // Screen click is in head-relative frame; world azimuth = head + yaw.
        az = wrap180(az + (headYawDeg_ ? headYawDeg_() : 0.0f));
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
        // Drag writes target_(x,y) in world-locked native Cartesian. The
        // click position is in head-relative screen frame, so we rotate the
        // screen-frame offset by +headYaw before mapping to native.
        const auto c = centre();
        const float pxPerM = outerRadius() / kCompassMaxMeters;
        if (pxPerM < 1e-3f) return;
        // Screen-frame native offset (matches the no-rotation mapping below).
        float ty_screen = -(p.x - c.x) / pxPerM;
        float tx_screen = -(p.y - c.y) / pxPerM;
        // Rotate by +headYaw to get the world native vector.
        const float yawRad = juce::degreesToRadians(headYawDeg_ ? headYawDeg_() : 0.0f);
        const float cy = std::cos(yawRad), sy = std::sin(yawRad);
        float tx =  cy * tx_screen - sy * ty_screen;
        float ty =  sy * tx_screen + cy * ty_screen;
        if (snap)
        {
            // Snap to integer-metre grid for tidy positioning.
            tx = std::round(tx);
            ty = std::round(ty);
        }
        setBoolParam("aim_at_listener", false);
        setParam("target_x", tx);
        setParam("target_y", ty);
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
        // Flat near-black field — phosphor displays don't have
        // gradients. Vignette comes from low-alpha corner darkening.
        g.fillAll(juce::Colour(theme::bg0));

        // Distance rings — dim phosphor, hierarchical: outer ring full
        // strength, inner rings progressively dimmer.
        for (int i = 1; i <= 5; ++i)
        {
            const float ringR = outerR * (i / 5.0f);
            const auto col = (i == 5) ? juce::Colour(theme::phosphor2)
                                       : juce::Colour(theme::phosphor1).withMultipliedAlpha(0.4f + 0.12f * i);
            g.setColour(col);
            g.drawEllipse(centre.x - ringR, centre.y - ringR,
                          2.0f * ringR, 2.0f * ringR, i == 5 ? 1.0f : 0.6f);
        }

        // Sweep tick marks every 6° (60 ticks total). Cardinals get a
        // double-length tick. Mid-distance grid points get a tiny dot.
        // Authentic radar-scope tick density.
        for (int a = 0; a < 360; a += 6)
        {
            const bool cardinal  = (a % 90 == 0);
            const bool secondary = (a % 30 == 0);
            const float tickLen = cardinal ? 10.0f : secondary ? 6.0f : 3.0f;
            const float ang = juce::degreesToRadians((float)a);
            const float ux = -std::sin(ang), uy = -std::cos(ang);
            const float x0 = centre.x + ux * outerR;
            const float y0 = centre.y + uy * outerR;
            const float x1 = centre.x + ux * (outerR - tickLen);
            const float y1 = centre.y + uy * (outerR - tickLen);
            g.setColour(juce::Colour(cardinal ? theme::phosphor3
                                              : secondary ? theme::phosphor2
                                                          : theme::phosphor1));
            g.drawLine(x0, y0, x1, y1, cardinal ? 1.2f : 0.8f);
        }

        // Cardinal axis lines — phosphor wireframe crosshair.
        g.setColour(juce::Colour(theme::phosphor0));
        g.drawLine(centre.x, centre.y - outerR, centre.x, centre.y + outerR, 0.6f);
        g.drawLine(centre.x - outerR, centre.y, centre.x + outerR, centre.y, 0.6f);

        // Distance numerals along the forward-right diagonal.
        g.setColour(juce::Colour(theme::phosphor1));
        g.setFont(font9());
        for (int d = 5; d <= (int)kCompassMaxMeters; d += 5)
        {
            const float ringR = outerR * ((float)d / kCompassMaxMeters);
            const float ang = juce::degreesToRadians(-45.0f);
            const float lx = centre.x - std::sin(ang) * ringR;
            const float ly = centre.y - std::cos(ang) * ringR;
            g.drawText(juce::String(d) + "M",
                       juce::Rectangle<float>(lx - 16.0f, ly - 6.0f, 32.0f, 12.0f),
                       juce::Justification::centred);
        }

        // Cardinal labels — bright phosphor, monospace caps.
        g.setColour(juce::Colour(theme::phosphor3));
        g.setFont(font11());
        g.drawText("FRONT", juce::Rectangle<float>(centre.x - 40, centre.y - outerR - 16, 80, 12),
                   juce::Justification::centred);
        g.drawText("BACK",  juce::Rectangle<float>(centre.x - 40, centre.y + outerR + 4,  80, 12),
                   juce::Justification::centred);
        g.drawText("LEFT",  juce::Rectangle<float>(centre.x - outerR + 6, centre.y - 6, 40, 12),
                   juce::Justification::centredLeft);
        g.drawText("RIGHT", juce::Rectangle<float>(centre.x + outerR - 46, centre.y - 6, 40, 12),
                   juce::Justification::centredRight);

        // Listener — vector wireframe head. Outer ring + forward-facing
        // chevron. No fills; pure stroke art. The "you are here" idea
        // expressed in radar terminology rather than figurative drawing.
        g.setColour(juce::Colour(theme::phosphor2));
        g.drawEllipse(centre.x - 13.0f, centre.y - 13.0f, 26.0f, 26.0f, 1.2f);
        // Centre crosshair (radar own-position).
        g.drawLine(centre.x - 5.0f, centre.y, centre.x + 5.0f, centre.y, 1.0f);
        g.drawLine(centre.x, centre.y - 5.0f, centre.x, centre.y + 5.0f, 1.0f);
        // Forward chevron at +Y screen (FRONT direction).
        juce::Path chev;
        chev.startNewSubPath(centre.x - 5.0f, centre.y - 10.0f);
        chev.lineTo(centre.x,        centre.y - 16.0f);
        chev.lineTo(centre.x + 5.0f, centre.y - 10.0f);
        g.strokePath(chev, juce::PathStrokeType(1.2f));
        g.setColour(juce::Colour(theme::phosphor3));
        g.setFont(font10());
        g.drawText("YOU",
                   juce::Rectangle<float>(centre.x - 30.0f, centre.y + 18.0f, 60.0f, 12.0f),
                   juce::Justification::centred);
    }

    juce::AudioProcessorValueTreeState& state_;
    std::function<float()>              headYawDeg_;
    std::function<int()>                headStatus_;
    std::function<juce::uint64()>       headFrameId_;
    std::function<Quat()>               headQuat_;
    DragTarget activeDrag_ = DragTarget::None;
    float displayedYaw_       = 0.0f;
    float displayedOcclusion_ = 0.0f;
    float lastHeadYaw_        = 0.0f;
    bool  firstTick_          = true;
    juce::Image backgroundImage_;
    float prevSnap_[20] = {
        std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
    };
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
        g.fillAll(juce::Colour(theme::bg0));

        const auto b = getLocalBounds().toFloat();
        const float colW = b.getWidth() * 0.5f;
        const auto leftCol  = juce::Rectangle<float>(b.getX(),         b.getY(), colW, b.getHeight());
        const auto rightCol = juce::Rectangle<float>(b.getX() + colW,  b.getY(), colW, b.getHeight());

        drawPositionColumn(g, leftCol);
        drawPitchColumn   (g, rightCol);
        paintScanlines(g, getLocalBounds());
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

        g.setColour(juce::Colour(theme::bg2));
        g.fillRoundedRectangle(cx - 2.0f, topY, 4.0f, trackH, 2.0f);

        const float midY = topY + trackH * 0.5f;
        g.setColour(juce::Colour(theme::gridStrong));
        g.drawLine(cx - 10.0f, midY, cx + 10.0f, midY, 1.0f);

        g.setColour(juce::Colour(theme::text));
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

        g.setColour(juce::Colour(theme::src));
        g.fillEllipse(cx - 9.0f, h_y - 9.0f, 18.0f, 18.0f);
        g.setColour(juce::Colour(theme::srcDeep));
        g.drawEllipse(cx - 9.0f, h_y - 9.0f, 18.0f, 18.0f, 1.5f);

        g.setColour(juce::Colour(theme::textBright));
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

        g.setColour(juce::Colour(theme::bg2));
        g.fillRoundedRectangle(juce::Rectangle<float>(col.getX() + 6, topY, col.getWidth() - 12, trackH), 4.0f);

        g.setColour(juce::Colour(theme::gridStrong));
        g.drawLine(cx - 14.0f, midY, cx + 14.0f, midY, 1.0f);

        g.setColour(juce::Colour(theme::text));
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

        g.setColour(juce::Colour(theme::src));
        g.fillEllipse(cx - 9.0f, h_y - 9.0f, 18.0f, 18.0f);
        g.setColour(juce::Colour(theme::srcDeep));
        g.drawEllipse(cx - 9.0f, h_y - 9.0f, 18.0f, 18.0f, 1.5f);

        // Tilt indicator: a short line through the handle, tilted by pitch.
        // Pitch +90 → vertical (pointing up); 0 → horizontal; −90 → vertical down.
        const float ang     = juce::degreesToRadians(pitch);
        const float lineLen = 11.0f;
        const float dxL = std::cos(ang), dyL = -std::sin(ang);
        g.setColour(juce::Colour(theme::srcDeep));
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

        g.setColour(juce::Colour(theme::textBright));
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
        g.fillAll(juce::Colour(theme::bg1));
        const auto plot = plotArea();

        // Grid lines.
        g.setColour(juce::Colour(theme::gridFaint));
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
        g.setColour(juce::Colour(theme::gridMid));
        const float y0 = graphY(0.0f);
        g.drawLine(plot.getX(), y0, plot.getRight(), y0, 1.0f);

        // Axis labels.
        g.setColour(juce::Colour(theme::textDim));
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
        g.setColour(juce::Colour(theme::curveFill));
        g.fillPath(fill);

        // The curve itself.
        juce::Path line;
        line.startNewSubPath(graphX(0.0f), a.y); // flat below a_dist
        line.lineTo(a.x, a.y);
        line.lineTo(b.x, b.y);
        line.lineTo(c.x, c.y);
        line.lineTo(d.x, d.y);
        g.setColour(juce::Colour(theme::curveLine));
        g.strokePath(line, juce::PathStrokeType(1.5f));

        // Draggable nodes.
        drawNode(g, a, "A", Node::A);
        drawNode(g, b, "B", Node::B);
        drawNode(g, c, "C", Node::C);
        drawNode(g, d, "D", Node::D);
        paintScanlines(g, getLocalBounds());
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
        g.setColour(juce::Colour(active ? theme::nodeActive : theme::src));
        g.fillEllipse(p.x - r, p.y - r, 2 * r, 2 * r);
        g.setColour(juce::Colour(theme::srcDeep));
        g.drawEllipse(p.x - r, p.y - r, 2 * r, 2 * r, 1.5f);
        g.setColour(juce::Colour(theme::srcLight));
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
    l.setColour(juce::Label::textColourId, juce::Colour(theme::text));
    l.setFont(juce::Font(juce::FontOptions(10.0f)).withExtraKerningFactor(0.22f));
    l.setJustificationType(juce::Justification::centredLeft);
}

// Distance-curve preset bank. Shared between the combobox setup in
// the constructor and the parameter-listener match logic.
struct Preset {
    const char* name;
    float aD, aDb, bD, bDb, cD, cDb, dD;
};
const Preset kDistPresets[] = {
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
constexpr int kDistPresetCount = (int)(sizeof(kDistPresets) / sizeof(Preset));

const char* const kDistParamIds[7] = {
    "dist_a", "dist_a_db", "dist_b", "dist_b_db",
    "dist_c", "dist_c_db", "dist_d",
};

} // namespace

SpatialAudioEditor::SpatialAudioEditor(SpatialAudioProcessor& p)
    : AudioProcessorEditor(p), proc_(p)
{
    compass_     = std::make_unique<SpatialCompass>(p.apvts,
        [&p] { return p.getEffectiveYawDeg(); },
        [&p] { return (int)p.getHeadTrackerStatus(); },
        [&p] { return p.getHeadTrackerFrameId(); },
        [&p] { return p.getEffectiveQuat(); });
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
        s.setColour(juce::Slider::backgroundColourId,       juce::Colour(theme::bg2));
        s.setColour(juce::Slider::trackColourId,            juce::Colour(theme::src));
        s.setColour(juce::Slider::thumbColourId,            juce::Colour(theme::srcLight));
        s.setColour(juce::Slider::textBoxBackgroundColourId, juce::Colour(0));
        s.setColour(juce::Slider::textBoxOutlineColourId,   juce::Colour(0));
        s.setColour(juce::Slider::textBoxTextColourId,      juce::Colour(theme::textBright));
        addAndMakeVisible(s);
    };
    auto initLabel = [this](juce::Label& l, const char* text)
    {
        l.setText(text, juce::dontSendNotification);
        l.setColour(juce::Label::textColourId, juce::Colour(theme::textBright));
        l.setJustificationType(juce::Justification::centredRight);
        l.setFont(juce::Font(juce::FontOptions(11.0f)));
        addAndMakeVisible(l);
    };

    // Top row (gain + aim).
    initLabel(gainLabel_, "Gain");
    initSlider(gainSlider_);
    gainSlider_.setTooltip("Source gain (dB).");
    gainAttachment_ = std::make_unique<SliderAttachment>(p.apvts, "gain_db", gainSlider_);

    // Width: angular spread between L and R virtual sources.
    initLabel(widthLabel_, "Width");
    initSlider(widthSlider_);
    widthSlider_.setTooltip(
        "Angular spread between L and R virtual sources (degrees). "
        "0" + kGlyphDeg + " = mono. 60" + kGlyphDeg
        + " = standard stereo. 180" + kGlyphDeg + " = hard L/R.");
    widthAttachment_ = std::make_unique<SliderAttachment>(p.apvts, "width", widthSlider_);

    aimAtListenerButton_.setTooltip("Lock source orientation to face the listener.");
    aimAtListenerButton_.setColour(juce::ToggleButton::textColourId, juce::Colour(theme::textBright));
    aimAtListenerButton_.setColour(juce::ToggleButton::tickColourId, juce::Colour(theme::src));
    aimAtListenerButton_.setColour(juce::ToggleButton::tickDisabledColourId, juce::Colour(theme::gridStrong));
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
    // Preset bank lives in the anonymous namespace above.
    initLabel(distPresetLabel_, "Preset");
    int presetId = 1;
    for (const auto& pr : kDistPresets)
        distPresetBox_.addItem(pr.name, presetId++);
    distPresetBox_.setColour(juce::ComboBox::backgroundColourId, juce::Colour(theme::bg2));
    distPresetBox_.setColour(juce::ComboBox::textColourId,       juce::Colour(theme::textBright));
    distPresetBox_.setColour(juce::ComboBox::outlineColourId,    juce::Colour(theme::gridStrong));
    distPresetBox_.setColour(juce::ComboBox::buttonColourId,     juce::Colour(theme::bg2));
    distPresetBox_.setColour(juce::ComboBox::arrowColourId,      juce::Colour(theme::text));
    addAndMakeVisible(distPresetBox_);
    distPresetBox_.onChange = [this] {
        const int sel = distPresetBox_.getSelectedId() - 1;
        if (sel < 0 || sel >= kDistPresetCount) return;
        const auto& pr = kDistPresets[sel];
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

    // Initial preset selection (matches first-launch defaults or any
    // DAW-restored state). Listener below keeps it in sync afterwards.
    refreshPresetSelection();
    for (auto* id : kDistParamIds)
        p.apvts.addParameterListener(id, this);

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

    stereoBypassButton_.setColour(juce::ToggleButton::textColourId, juce::Colour(theme::textBright));
    stereoBypassButton_.setColour(juce::ToggleButton::tickColourId, juce::Colour(theme::src));
    stereoBypassButton_.setColour(juce::ToggleButton::tickDisabledColourId, juce::Colour(theme::gridStrong));
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
    advancedButton_.setColour(juce::TextButton::buttonColourId,   juce::Colour(theme::bg2));
    advancedButton_.setColour(juce::TextButton::buttonOnColourId, juce::Colour(theme::gridStrong));
    advancedButton_.setColour(juce::TextButton::textColourOffId,  juce::Colour(theme::textBright));
    advancedButton_.setColour(juce::TextButton::textColourOnId,   juce::Colour(theme::textBright));
    advancedButton_.onClick = [this, setAdvVisible] {
        advancedOpen_ = !advancedOpen_;
        advancedButton_.setButtonText("Advanced " + (advancedOpen_ ? kGlyphUp : kGlyphDown));
        setAdvVisible(advancedOpen_);
        setSize(580, advancedOpen_ ? 808 : 784);
    };
    addAndMakeVisible(advancedButton_);

    resetButton_.setTooltip("Reset all parameters to defaults.");
    resetButton_.setColour(juce::TextButton::buttonColourId,   juce::Colour(theme::bg2));
    resetButton_.setColour(juce::TextButton::buttonOnColourId, juce::Colour(theme::gridStrong));
    resetButton_.setColour(juce::TextButton::textColourOffId,  juce::Colour(theme::textBright));
    resetButton_.setColour(juce::TextButton::textColourOnId,   juce::Colour(theme::textBright));
    resetButton_.onClick = [this] { resetAllParams(); };
    addAndMakeVisible(resetButton_);

    // Head tracking row (bottom-left).
    headTrackButton_.setTooltip("Drive listener rotation from Galaxy Buds 2 Pro head motion.");
    headTrackButton_.setColour(juce::ToggleButton::textColourId,        juce::Colour(theme::textBright));
    headTrackButton_.setColour(juce::ToggleButton::tickColourId,        juce::Colour(theme::src));
    headTrackButton_.setColour(juce::ToggleButton::tickDisabledColourId,juce::Colour(theme::gridStrong));
    addAndMakeVisible(headTrackButton_);
    headTrackAttachment_ = std::make_unique<ButtonAttachment>(
        p.apvts, "head_tracking_enabled", headTrackButton_);

    recentreButton_.setTooltip("Capture current head pose as 'looking forward'.");
    recentreButton_.setColour(juce::TextButton::buttonColourId,   juce::Colour(theme::bg2));
    recentreButton_.setColour(juce::TextButton::buttonOnColourId, juce::Colour(theme::gridStrong));
    recentreButton_.setColour(juce::TextButton::textColourOffId,  juce::Colour(theme::textBright));
    recentreButton_.setColour(juce::TextButton::textColourOnId,   juce::Colour(theme::textBright));
    recentreButton_.onClick = [this] { proc_.recentreHeadTracker(); };
    addAndMakeVisible(recentreButton_);

    headStatusDot_.setText(juce::String::charToString(0x25CF), juce::dontSendNotification);  // ●
    headStatusDot_.setJustificationType(juce::Justification::centred);
    headStatusDot_.setColour(juce::Label::textColourId, juce::Colour(theme::phosphor0));
    headStatusDot_.setTooltip("Off");
    addAndMakeVisible(headStatusDot_);

    startTimerHz(5);

    setSize(580, 784);
}

SpatialAudioEditor::~SpatialAudioEditor()
{
    stopTimer();
    for (auto* id : kDistParamIds)
        proc_.apvts.removeParameterListener(id, this);
}

void SpatialAudioEditor::timerCallback()
{
    using S = HeadTracker::Status;
    const auto status = proc_.getHeadTrackerStatus();
    const bool enabled = headTrackButton_.getToggleState();
    const bool referenced = proc_.isHeadPoseReferenced();

    juce::uint32 col = theme::phosphor0;
    const char*  tip = "Off";
    if (enabled)
    {
        switch (status)
        {
            case S::Disconnected: col = theme::phosphor1; tip = "Searching for buds…"; break;
            case S::Connecting:   col = theme::phosphor2; tip = "Connecting…"; break;
            case S::Failed:       col = theme::phosphor1; tip = "Could not connect — buds paired & powered?"; break;
            case S::Streaming:
                col = referenced ? theme::phosphor4 : theme::phosphor3;
                tip = referenced ? "Streaming (calibrated)"
                                 : "Streaming — press Re-centre to set forward";
                break;
        }
    }
    headStatusDot_.setColour(juce::Label::textColourId, juce::Colour(col));
    headStatusDot_.setTooltip(tip);

    // Disable Re-centre when there's no live pose to capture.
    recentreButton_.setEnabled(enabled && status == S::Streaming);
}

void SpatialAudioEditor::refreshPresetSelection()
{
    // dB params have step interval 0.1, so values like -0.34 get
    // snapped to -0.3 — eps must exceed half that step to find a
    // match. Distance params are finer (interval 0.01) so they match
    // exactly under the same eps.
    constexpr float kEps = 0.06f;
    const auto& s = proc_.apvts;
    const float a  = s.getRawParameterValue("dist_a")    ->load();
    const float ad = s.getRawParameterValue("dist_a_db") ->load();
    const float b  = s.getRawParameterValue("dist_b")    ->load();
    const float bd = s.getRawParameterValue("dist_b_db") ->load();
    const float c  = s.getRawParameterValue("dist_c")    ->load();
    const float cd = s.getRawParameterValue("dist_c_db") ->load();
    const float d  = s.getRawParameterValue("dist_d")    ->load();
    int matched = 0;
    for (int i = 0; i < kDistPresetCount; ++i) {
        const auto& pr = kDistPresets[i];
        if (std::abs(pr.aD  - a)  < kEps && std::abs(pr.aDb - ad) < kEps &&
            std::abs(pr.bD  - b)  < kEps && std::abs(pr.bDb - bd) < kEps &&
            std::abs(pr.cD  - c)  < kEps && std::abs(pr.cDb - cd) < kEps &&
            std::abs(pr.dD  - d)  < kEps) { matched = i + 1; break; }
    }
    distPresetBox_.setSelectedId(matched, juce::dontSendNotification);
}

void SpatialAudioEditor::parameterChanged(const juce::String&, float)
{
    // APVTS listener fires off-thread for automation; defer to the
    // message thread so combobox updates stay safe.
    juce::MessageManager::callAsync([safe = juce::Component::SafePointer<SpatialAudioEditor>(this)] {
        if (safe != nullptr) safe->refreshPresetSelection();
    });
}

void SpatialAudioEditor::resetAllParams()
{
    constexpr const char* ids[] = {
        "distance", "azimuth", "elevation", "width", "gain_db",
        "target_x", "target_y", "target_z",
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
        "head_tracking_enabled",
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
    // Calibration is not an APVTS param — clear it explicitly so a reset
    // restores the "uncalibrated" state.
    proc_.clearHeadTrackerRef();
}

void SpatialAudioEditor::paint(juce::Graphics& g)
{
    g.fillAll(juce::Colour(theme::bg0));
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
    bottom.removeFromRight(12);
    headStatusDot_.setBounds(bottom.removeFromLeft(16));
    headTrackButton_.setBounds(bottom.removeFromLeft(120));
    bottom.removeFromLeft(4);
    recentreButton_.setBounds(bottom.removeFromLeft(80));

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
    // ---- Width (stereo pair angular spread) ----
    {
        auto r = area.removeFromTop(22);
        area.removeFromTop(2);
        widthLabel_.setBounds(r.removeFromLeft(60));
        widthSlider_.setBounds(r);
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
