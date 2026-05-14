#include "PluginEditor.h"

#include <cmath>

namespace
{
constexpr float kCompassMaxMeters = 25.0f;

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

// Inverse: screen offset (px from centre) → (azimuth deg, distance m).
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
} // namespace

// ---------------------------------------------------------------------------
// SpatialCompass — top-down view, draggable source dot.
// ---------------------------------------------------------------------------

class SpatialCompass : public juce::Component, private juce::Timer
{
public:
    explicit SpatialCompass(juce::AudioProcessorValueTreeState& s)
        : state_(s)
    {
        startTimerHz(30);
    }

    ~SpatialCompass() override { stopTimer(); }

    void paint(juce::Graphics& g) override
    {
        auto bounds       = getLocalBounds().toFloat();
        const auto centre = bounds.getCentre();
        const float outerR =
            juce::jmin(bounds.getWidth(), bounds.getHeight()) * 0.45f;

        g.fillAll(juce::Colour(0xff141414));

        // Concentric rings every 5 m.
        g.setColour(juce::Colour(0xff2c2c2c));
        for (float d = 5.0f; d <= kCompassMaxMeters; d += 5.0f)
        {
            const float ringR = outerR * (d / kCompassMaxMeters);
            g.drawEllipse(centre.x - ringR, centre.y - ringR,
                          2.0f * ringR, 2.0f * ringR, 1.0f);
        }
        // Outer ring (max).
        g.setColour(juce::Colour(0xff444444));
        g.drawEllipse(centre.x - outerR, centre.y - outerR,
                      2.0f * outerR, 2.0f * outerR, 1.5f);

        // Cardinal axes.
        g.setColour(juce::Colour(0xff262626));
        g.drawLine(centre.x, centre.y - outerR, centre.x, centre.y + outerR);
        g.drawLine(centre.x - outerR, centre.y, centre.x + outerR, centre.y);

        // Cardinal labels.
        g.setColour(juce::Colour(0xff7a7a7a));
        g.setFont(juce::Font(juce::FontOptions(11.0f)));
        g.drawText("FRONT", juce::Rectangle<float>(centre.x - 40, centre.y - outerR - 16, 80, 12),
                   juce::Justification::centred);
        g.drawText("BACK",  juce::Rectangle<float>(centre.x - 40, centre.y + outerR + 4,  80, 12),
                   juce::Justification::centred);
        g.drawText("LEFT",  juce::Rectangle<float>(centre.x - outerR - 44, centre.y - 6, 40, 12),
                   juce::Justification::centredRight);
        g.drawText("RIGHT", juce::Rectangle<float>(centre.x + outerR + 4, centre.y - 6, 50, 12),
                   juce::Justification::centredLeft);

        // Listener head — circle with forward-pointing nose.
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

        // Source dot + line.
        const float dist = currentDistance();
        const float az   = currentAzimuth();
        const auto  src  = azimDistToScreen(centre, outerR, az, dist, kCompassMaxMeters);

        g.setColour(juce::Colour(0x80667788));
        g.drawLine(centre.x, centre.y, src.x, src.y, 1.0f);

        g.setColour(juce::Colour(0xffff8c42));
        g.fillEllipse(src.x - 8.0f, src.y - 8.0f, 16.0f, 16.0f);
        g.setColour(juce::Colour(0xff5a2410));
        g.drawEllipse(src.x - 8.0f, src.y - 8.0f, 16.0f, 16.0f, 1.5f);
        g.setColour(juce::Colour(0xffffb88a));
        g.setFont(juce::Font(juce::FontOptions(10.0f)));
        // Place the label below the dot, unless it would clip the bottom
        // of the view — then place it above instead.
        const bool above = src.y > bounds.getBottom() - 22.0f;
        const float labelY = above ? src.y - 22.0f : src.y + 10.0f;
        g.drawText("SOURCE",
                   juce::Rectangle<float>(src.x - 30.0f, labelY, 60.0f, 12.0f),
                   juce::Justification::centred);

        // Top-left readout.
        g.setColour(juce::Colour(0xff9a9a9a));
        g.setFont(juce::Font(juce::FontOptions(11.0f)));
        auto info = juce::String::formatted("source: %.2f m  %.1f°", dist, az);
        g.drawText(info, juce::Rectangle<int>(8, 8, 240, 14),
                   juce::Justification::topLeft);
        g.setColour(juce::Colour(0xff666666));
        g.setFont(juce::Font(juce::FontOptions(10.0f)));
        g.drawText("drag the orange dot",
                   juce::Rectangle<int>(8, 22, 200, 12),
                   juce::Justification::topLeft);
    }

    void mouseDown(const juce::MouseEvent& e) override
    {
        beginGesture();
        applyFromMouse(e);
    }

    void mouseDrag(const juce::MouseEvent& e) override { applyFromMouse(e); }

    void mouseUp(const juce::MouseEvent&) override { endGesture(); }

private:
    void timerCallback() override { repaint(); }

    float currentDistance() const
    {
        return state_.getRawParameterValue("distance")->load();
    }

    float currentAzimuth() const
    {
        return state_.getRawParameterValue("azimuth")->load();
    }

    void applyFromMouse(const juce::MouseEvent& e)
    {
        auto bounds = getLocalBounds().toFloat();
        const auto centre = bounds.getCentre();
        const float outerR =
            juce::jmin(bounds.getWidth(), bounds.getHeight()) * 0.45f;

        float az, dist;
        screenToAzimDist(centre, outerR, kCompassMaxMeters, e.position, az, dist);

        if (auto* p = state_.getParameter("distance"))
            p->setValueNotifyingHost(p->convertTo0to1(dist));
        if (auto* p = state_.getParameter("azimuth"))
            p->setValueNotifyingHost(p->convertTo0to1(az));
    }

    void beginGesture()
    {
        if (auto* p = state_.getParameter("distance")) p->beginChangeGesture();
        if (auto* p = state_.getParameter("azimuth"))  p->beginChangeGesture();
    }

    void endGesture()
    {
        if (auto* p = state_.getParameter("distance")) p->endChangeGesture();
        if (auto* p = state_.getParameter("azimuth"))  p->endChangeGesture();
    }

    juce::AudioProcessorValueTreeState& state_;
};

// ---------------------------------------------------------------------------
// ElevationStrip — vertical bar; drag the handle to set elevation.
// ---------------------------------------------------------------------------

class ElevationStrip : public juce::Component, private juce::Timer
{
public:
    explicit ElevationStrip(juce::AudioProcessorValueTreeState& s) : state_(s)
    {
        startTimerHz(30);
    }

    ~ElevationStrip() override { stopTimer(); }

    void paint(juce::Graphics& g) override
    {
        const auto bounds = getLocalBounds().toFloat();
        const float cx    = bounds.getCentreX();
        const float topY  = bounds.getY() + 22.0f;
        const float botY  = bounds.getBottom() - 22.0f;
        const float trackH = botY - topY;

        g.fillAll(juce::Colour(0xff141414));

        // Track.
        g.setColour(juce::Colour(0xff2c2c2c));
        const float trackW = 4.0f;
        g.fillRoundedRectangle(cx - trackW * 0.5f, topY, trackW, trackH, 2.0f);

        // Horizon mark.
        const float midY = topY + trackH * 0.5f;
        g.setColour(juce::Colour(0xff4a4a4a));
        g.drawLine(cx - 10.0f, midY, cx + 10.0f, midY, 1.0f);

        // Labels.
        g.setColour(juce::Colour(0xff7a7a7a));
        g.setFont(juce::Font(juce::FontOptions(10.0f)));
        g.drawText("UP",   juce::Rectangle<float>(cx - 20.0f, topY - 16.0f, 40.0f, 12.0f),
                   juce::Justification::centred);
        g.drawText("DOWN", juce::Rectangle<float>(cx - 20.0f, botY + 4.0f, 40.0f, 12.0f),
                   juce::Justification::centred);

        // Handle: maps elevation +90..-90 to topY..botY.
        const float el  = state_.getRawParameterValue("elevation")->load();
        const float t   = juce::jlimit(0.0f, 1.0f, (90.0f - el) / 180.0f);
        const float h_y = topY + t * trackH;

        g.setColour(juce::Colour(0xffff8c42));
        g.fillEllipse(cx - 9.0f, h_y - 9.0f, 18.0f, 18.0f);
        g.setColour(juce::Colour(0xff5a2410));
        g.drawEllipse(cx - 9.0f, h_y - 9.0f, 18.0f, 18.0f, 1.5f);

        g.setColour(juce::Colour(0xff9a9a9a));
        g.setFont(juce::Font(juce::FontOptions(11.0f)));
        g.drawText(juce::String::formatted("%.0f°", el),
                   juce::Rectangle<int>(0, (int) midY + 14, getWidth(), 12),
                   juce::Justification::centred);
    }

    void mouseDown(const juce::MouseEvent& e) override
    {
        if (auto* p = state_.getParameter("elevation")) p->beginChangeGesture();
        applyFromMouse(e);
    }

    void mouseDrag(const juce::MouseEvent& e) override { applyFromMouse(e); }

    void mouseUp(const juce::MouseEvent&) override
    {
        if (auto* p = state_.getParameter("elevation")) p->endChangeGesture();
    }

private:
    void timerCallback() override { repaint(); }

    void applyFromMouse(const juce::MouseEvent& e)
    {
        const auto bounds = getLocalBounds().toFloat();
        const float topY  = bounds.getY() + 22.0f;
        const float botY  = bounds.getBottom() - 22.0f;
        const float trackH = botY - topY;
        const float t  = juce::jlimit(0.0f, 1.0f, (e.position.y - topY) / trackH);
        const float el = 90.0f - t * 180.0f;
        if (auto* p = state_.getParameter("elevation"))
            p->setValueNotifyingHost(p->convertTo0to1(el));
    }

    juce::AudioProcessorValueTreeState& state_;
};

// ---------------------------------------------------------------------------
// SpatialAudioEditor — main view.
// ---------------------------------------------------------------------------

namespace
{
struct M6Spec { const char* id; const char* label; };
constexpr std::array<M6Spec, 9> kM6Specs = {{
    {"source_yaw",       "Src Yaw"},
    {"source_pitch",     "Src Pitch"},
    {"source_roll",      "Src Roll"},
    {"occlusion",        "Occl"},
    {"dir_inner_deg",    "Inner"},
    {"dir_outer_deg",    "Outer"},
    {"dir_outer_gain",   "Dir Gain"},
    {"dir_outer_lp",     "Dir LP"},
    {"direct_path_gain", "Direct"},
}};
} // namespace

SpatialAudioEditor::SpatialAudioEditor(SpatialAudioProcessor& p)
    : AudioProcessorEditor(p), proc_(p)
{
    compass_   = std::make_unique<SpatialCompass>(p.apvts);
    elevation_ = std::make_unique<ElevationStrip>(p.apvts);
    addAndMakeVisible(*compass_);
    addAndMakeVisible(*elevation_);

    gainLabel_.setText("Gain", juce::dontSendNotification);
    gainLabel_.setColour(juce::Label::textColourId, juce::Colour(0xffbbbbbb));
    gainLabel_.setJustificationType(juce::Justification::centredRight);
    addAndMakeVisible(gainLabel_);

    gainSlider_.setSliderStyle(juce::Slider::LinearHorizontal);
    gainSlider_.setTextBoxStyle(juce::Slider::TextBoxRight, false, 80, 18);
    addAndMakeVisible(gainSlider_);
    gainAttachment_ = std::make_unique<SliderAttachment>(p.apvts, "gain_db", gainSlider_);

    for (int i = 0; i < kM6Count; ++i)
    {
        auto& s = m6Sliders_[(size_t) i];
        s.setSliderStyle(juce::Slider::RotaryHorizontalVerticalDrag);
        s.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 60, 14);
        s.setColour(juce::Slider::textBoxOutlineColourId, juce::Colour(0x00000000));
        addAndMakeVisible(s);

        auto& lbl = m6Labels_[(size_t) i];
        lbl.setText(kM6Specs[(size_t) i].label, juce::dontSendNotification);
        lbl.setJustificationType(juce::Justification::centred);
        lbl.setColour(juce::Label::textColourId, juce::Colour(0xff9a9a9a));
        lbl.setFont(juce::Font(juce::FontOptions(10.0f)));
        addAndMakeVisible(lbl);

        m6Attach_[(size_t) i] =
            std::make_unique<SliderAttachment>(p.apvts, kM6Specs[(size_t) i].id, s);
    }

    resetButton_.setTooltip("Reset all parameters to defaults");
    resetButton_.onClick = [this] { resetAllParams(); };
    addAndMakeVisible(resetButton_);

    setSize(540, 540);
}

void SpatialAudioEditor::resetAllParams()
{
    constexpr const char* ids[] = {
        "distance", "azimuth", "elevation", "gain_db",
        "listener_x", "listener_y", "listener_z",
        "yaw", "pitch", "roll",
        "source_yaw", "source_pitch", "source_roll",
        "occlusion",
        "dir_inner_deg", "dir_outer_deg",
        "dir_outer_gain", "dir_outer_lp",
        "direct_path_gain",
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

SpatialAudioEditor::~SpatialAudioEditor() = default;

void SpatialAudioEditor::paint(juce::Graphics& g)
{
    g.fillAll(juce::Colour(0xff0c0c0c));
}

void SpatialAudioEditor::resized()
{
    auto area = getLocalBounds().reduced(10);

    auto bottom = area.removeFromBottom(28);
    gainLabel_.setBounds(bottom.removeFromLeft(56));
    resetButton_.setBounds(bottom.removeFromRight(72));
    bottom.removeFromRight(6);
    gainSlider_.setBounds(bottom);

    area.removeFromBottom(6);

    // M6 row: 9 compact rotaries with labels below.
    auto m6Row = area.removeFromBottom(86);
    const int cellW = m6Row.getWidth() / kM6Count;
    for (int i = 0; i < kM6Count; ++i)
    {
        auto cell = m6Row.removeFromLeft(cellW).reduced(2);
        auto lbl  = cell.removeFromTop(14);
        m6Labels_[(size_t) i].setBounds(lbl);
        m6Sliders_[(size_t) i].setBounds(cell);
    }

    area.removeFromBottom(6);

    auto strip = area.removeFromRight(60);
    compass_->setBounds(area);
    elevation_->setBounds(strip);
}
