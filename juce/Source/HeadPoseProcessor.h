#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>

#include "HeadTracker.h"

// Pipeline:
//   q_raw   = tracker output, in bud frame
//   q_delta = q_raw * q_ref^-1                    // re-centre
//   q_atm   = q_M * q_delta * q_M^-1              // bud frame -> native frame
//   q_out   = slerp(q_prev, q_target, alpha)      // one-pole smoother
// where q_target is q_atm while streaming, identity when disconnected.
class HeadPoseProcessor
{
public:
    // Smoother time constant, seconds. Audio-thread param (no smoothing of
    // the smoother itself; just used to compute alpha per block).
    float tauSeconds = 0.040f;

    // ---- producer thread API ----
    // Called whenever the tracker has a fresh raw pose.
    void setRaw(const Quat& q) noexcept
    {
        // 4× int32 store under spinlock-free assumption: write to a slot we
        // own, then bump the publish counter. Single producer.
        const uint32_t next = (publishCount_.load(std::memory_order_relaxed) + 1) & slotMask_;
        slots_[next] = q;
        publishCount_.store(publishCount_.load(std::memory_order_relaxed) + 1,
                            std::memory_order_release);
    }

    // ---- UI thread API ----
    // Capture the latest raw pose as the new reference. No-op if no frame.
    void recentreFromLatestRaw() noexcept
    {
        Quat q = loadLatestRaw();
        // Shared across all plugin instances in the same process — hosts
        // (e.g. SoundSource) can instantiate the AU multiple times, and
        // we need the Re-centre to apply to whichever instance owns the
        // audio path, not just the one whose editor you clicked.
        sQRefConj_[0].store(q.w,  std::memory_order_relaxed);
        sQRefConj_[1].store(-q.x, std::memory_order_relaxed);
        sQRefConj_[2].store(-q.y, std::memory_order_relaxed);
        sQRefConj_[3].store(-q.z, std::memory_order_release);
        sHaveRef_.store(true, std::memory_order_release);
    }

    void clearRef() noexcept
    {
        sHaveRef_.store(false, std::memory_order_release);
    }

    bool hasRef() const noexcept { return sHaveRef_.load(std::memory_order_acquire); }

    // ---- audio thread API ----
    // Returns the smoothed pose in native frame. dt = block duration in
    // seconds. Pass tracker status so we can fade to identity on dropout.
    Quat process(double dtSec, HeadTracker::Status /*status*/) noexcept
    {
        // Gate purely on having a reference. Status flapping inside the AU
        // host (transient UDP jitter) used to wipe the rotation; the raw
        // pose just freezes at its last value during gaps, which is a
        // better UX than snapping to identity every few hundred ms.
        Quat target = Quat::identity();
        if (sHaveRef_.load(std::memory_order_acquire))
        {
            // q_delta = q_raw * q_ref^-1
            const Quat raw = loadLatestRaw();
            const Quat ref{
                sQRefConj_[0].load(std::memory_order_relaxed),
                sQRefConj_[1].load(std::memory_order_relaxed),
                sQRefConj_[2].load(std::memory_order_relaxed),
                sQRefConj_[3].load(std::memory_order_relaxed)};
            const Quat delta = mul(raw, ref);
            // q_atm = q_M * delta * q_M^-1, q_M = (cos45, 0, 0, sin45)
            target = transformBudToNative(delta);
        }

        const float tau = tauSeconds > 1e-4f ? tauSeconds : 1e-4f;
        const float alpha = 1.0f - std::exp(-(float)dtSec / tau);
        smoothed_ = slerp(smoothed_, target, alpha);
        normalize(smoothed_);
        return smoothed_;
    }

    void resetSmoother() noexcept { smoothed_ = Quat::identity(); }

private:
    // Two-slot publish: producer increments counter, consumer reads slot
    // [counter & 1]. Last-write-wins; the data we lose is older than the
    // tail anyway.
    static constexpr int slotMask_ = 1;
    Quat slots_[2] { Quat::identity(), Quat::identity() };
    std::atomic<uint32_t> publishCount_ { 0 };

    // Process-shared so all plugin instances see the same Re-centre state.
    inline static std::atomic<bool>  sHaveRef_ { false };
    inline static std::atomic<float> sQRefConj_[4] { {1.0f}, {0.0f}, {0.0f}, {0.0f} };

    Quat smoothed_ = Quat::identity();

    Quat loadLatestRaw() const noexcept
    {
        const uint32_t c = publishCount_.load(std::memory_order_acquire);
        return slots_[c & slotMask_];
    }

    static Quat mul(const Quat& a, const Quat& b) noexcept
    {
        return {
            a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
            a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
            a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
            a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
        };
    }

    // q_M = (cos45, 0, 0, sin45) = (c, 0, 0, c), c = sqrt(0.5).
    // q_atm = q_M * q_in * conj(q_M). Expanded; q_M has only w and z.
    static Quat transformBudToNative(const Quat& q) noexcept
    {
        constexpr float c = 0.70710678118f;
        const Quat qM { c, 0.0f, 0.0f, c };
        const Quat qMc{ c, 0.0f, 0.0f, -c };
        return mul(mul(qM, q), qMc);
    }

    static Quat slerp(const Quat& a, const Quat& b, float t) noexcept
    {
        // Pick the shorter arc.
        float bw = b.w, bx = b.x, by = b.y, bz = b.z;
        float dot = a.w*bw + a.x*bx + a.y*by + a.z*bz;
        if (dot < 0.0f) { bw = -bw; bx = -bx; by = -by; bz = -bz; dot = -dot; }
        if (dot > 0.9995f)
        {
            // Quaternions nearly identical → lerp.
            return {
                a.w + t * (bw - a.w),
                a.x + t * (bx - a.x),
                a.y + t * (by - a.y),
                a.z + t * (bz - a.z)
            };
        }
        const float theta0 = std::acos(dot);
        const float sin0   = std::sin(theta0);
        const float s1     = std::sin((1.0f - t) * theta0) / sin0;
        const float s2     = std::sin(t * theta0) / sin0;
        return {
            s1*a.w + s2*bw,
            s1*a.x + s2*bx,
            s1*a.y + s2*by,
            s1*a.z + s2*bz
        };
    }

    static void normalize(Quat& q) noexcept
    {
        const float m2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
        if (m2 < 1e-12f) { q = Quat::identity(); return; }
        const float inv = 1.0f / std::sqrt(m2);
        q.w *= inv; q.x *= inv; q.y *= inv; q.z *= inv;
    }
};
