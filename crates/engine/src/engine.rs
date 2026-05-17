//! Top-level engine. M3 covered listener + sources + ambisonic bus +
//! per-block process loop. M4 added the HRTF binaural decode. M6
//! adds source orientation, the directivity cone (§6.2), occlusion +
//! directivity low-pass (§6.3), and `direct_path_gain` (§6.4).
//! Reverb (M7), externalizer (M8), and the rest of §12 land later.

use core::f32::consts::PI;

use crate::consts::{
    BLOCK_SIZE, FDN_SIZE, NUM_AMBI, OCCLUSION_FC_MAX_FACTOR, OCCLUSION_FREQ_BASE,
    OCCLUSION_FREQ_SCALE, OUTPUT_CHANNELS, REV_DECODE_GAINS, REVERB_OUTPUT_DIRS,
    SH_W_NORM,
};
use crate::audio_bed::{AudioBed, BedFormat};
use crate::decoder::HrtfDecoder;
use crate::distance::DistanceModel;
use crate::externalizer::Externalizer;
use crate::hrtf::Hrtf;
use crate::math::{Quat, Vec3};
use crate::reverb::ReverbCore;
use crate::sh::sh_basis_n3d_into;
use crate::source::Source;
use crate::w_binauralizer::WBinauralizer;

#[derive(Clone, Debug)]
pub struct Listener {
    pub pos: Vec3,
    pub quat: Quat,
}

impl Default for Listener {
    fn default() -> Self {
        Self { pos: Vec3::ZERO, quat: Quat::IDENTITY }
    }
}

pub struct Engine {
    pub sample_rate: u32,
    pub listener: Listener,
    pub sources: Vec<Source>,
    /// 16-channel ambisonic accumulator, written by `process_block`.
    pub ambi_bus: Vec<[f32; BLOCK_SIZE]>,
    /// Binaural stereo output of the main HRTF decoder (§8).
    /// Stays zero when no HRTF is loaded.
    pub stereo_out: [[f32; BLOCK_SIZE]; OUTPUT_CHANNELS],
    /// Mono reverb input bus, summed across sources per block.
    pub reverb_in_bus: [f32; BLOCK_SIZE],
    /// Master reverb mix (0 = dry, 1 = unity wet, can go above).
    pub reverb_amount: f32,
    /// Per-direction SH bases for the 8 reverb output directions, scaled
    /// by `REV_DECODE_GAINS[k]`. Precomputed once at construction.
    rev_sh_bases: [[f32; NUM_AMBI]; FDN_SIZE],
    reverb: ReverbCore,
    externalizer: Externalizer,
    decoder: Option<HrtfDecoder>,
    w_binauralizer: Option<WBinauralizer>,
    pub audio_bed: Option<AudioBed>,
}

impl Engine {
    pub fn new(sample_rate: u32, num_sources: usize) -> Self {
        let mut rev_sh_bases = [[0.0_f32; NUM_AMBI]; FDN_SIZE];
        for k in 0..FDN_SIZE {
            let dir = Vec3::new(
                REVERB_OUTPUT_DIRS[k][0],
                REVERB_OUTPUT_DIRS[k][1],
                REVERB_OUTPUT_DIRS[k][2],
            );
            sh_basis_n3d_into(dir, &mut rev_sh_bases[k]);
            let scale = SH_W_NORM * REV_DECODE_GAINS[k];
            for v in rev_sh_bases[k].iter_mut() {
                *v *= scale;
            }
        }

        Self {
            sample_rate,
            listener: Listener::default(),
            sources: (0..num_sources).map(|_| Source::default()).collect(),
            ambi_bus: vec![[0.0; BLOCK_SIZE]; NUM_AMBI],
            stereo_out: [[0.0; BLOCK_SIZE]; OUTPUT_CHANNELS],
            reverb_in_bus: [0.0; BLOCK_SIZE],
            reverb_amount: 1.0,
            rev_sh_bases,
            reverb: ReverbCore::new(sample_rate),
            externalizer: Externalizer::new(sample_rate),
            decoder: None,
            w_binauralizer: None,
            audio_bed: None,
        }
    }

    /// §2.6 audio bed. `format = NoInput` removes the bed; any other
    /// value (re)allocates with the standard speaker layout for that
    /// format. Bed input is passed alongside object inputs in
    /// `process_block`.
    pub fn set_audio_bed_format(&mut self, format: BedFormat) {
        self.audio_bed = match format {
            BedFormat::NoInput => None,
            _ => Some(AudioBed::new(format)),
        };
    }

    /// Bed master gain (linear). Applied to every bed channel before
    /// SH encoding.
    pub fn set_audio_bed_gain(&mut self, gain: f32) {
        if let Some(bed) = self.audio_bed.as_mut() {
            bed.gain = gain.max(0.0);
        }
    }

    /// Bed orientation lock. `true` = headlocked (bed rotates with
    /// the listener); `false` (default) = world-locked.
    pub fn set_audio_bed_headlocked(&mut self, headlocked: bool) {
        if let Some(bed) = self.audio_bed.as_mut() {
            bed.headlocked = headlocked;
        }
    }

    /// §13 / §12 step 10: install the W-channel binauralizer
    /// (decoder_post). `filter_a` and `filter_b` are the raw bundled
    /// blobs (2865 f32 each, little-endian). Returns `true` on success.
    /// IRs are resampled from their authored rate to the engine's
    /// `sample_rate` at load time (§11).
    pub fn load_w_binauralizer(&mut self, filter_a: &[u8], filter_b: &[u8]) -> bool {
        match WBinauralizer::from_bytes_at(filter_a, filter_b, self.sample_rate) {
            Some(wb) => { self.w_binauralizer = Some(wb); true }
            None => false,
        }
    }

    /// Install the main HRTF decoder (§8). Until called, `stereo_out`
    /// stays zero. Replaces any previously installed HRTF.
    pub fn load_main_hrtf(&mut self, hrtf: &Hrtf) {
        self.decoder = Some(HrtfDecoder::new(hrtf));
    }

    pub fn set_listener_position(&mut self, x: f32, y: f32, z: f32) {
        let p = Vec3::new(x, y, z);
        if p != self.listener.pos {
            self.listener.pos = p;
            for s in &mut self.sources {
                s.pos_dirty = true;
            }
        }
    }

    pub fn set_listener_rotation(&mut self, w: f32, x: f32, y: f32, z: f32) {
        let q = Quat::new(w, x, y, z);
        if q != self.listener.quat {
            self.listener.quat = q;
            for s in &mut self.sources {
                s.pos_dirty = true;
            }
        }
    }

    pub fn set_source_position(&mut self, idx: usize, x: f32, y: f32, z: f32) {
        if let Some(s) = self.sources.get_mut(idx) {
            let p = Vec3::new(x, y, z);
            if p != s.pos {
                s.pos = p;
                s.pos_dirty = true;
            }
        }
    }

    /// Source orientation quaternion `(w, x, y, z)` in the engine's
    /// active coord frame (engine-native by default). Used by the
    /// §6.2 directivity cone to compute the source's forward vector.
    pub fn set_source_rotation(&mut self, idx: usize, w: f32, x: f32, y: f32, z: f32) {
        if let Some(s) = self.sources.get_mut(idx) {
            s.rotation = Quat::new(w, x, y, z);
        }
    }

    pub fn set_source_gain(&mut self, idx: usize, gain: f32) {
        if let Some(s) = self.sources.get_mut(idx) {
            s.gain.set_target(gain);
        }
    }

    pub fn set_source_active(&mut self, idx: usize, active: bool) {
        if let Some(s) = self.sources.get_mut(idx) {
            s.active = active;
        }
    }

    /// §6.4 multiplicative gain on the direct path only.
    pub fn set_source_direct_path_gain(&mut self, idx: usize, gain: f32) {
        if let Some(s) = self.sources.get_mut(idx) {
            s.direct_path_gain = gain;
        }
    }

    /// §6.3 occlusion target ∈ [0, 1]. Smoothed via the source's
    /// occlusion ramp; drives the per-source low-pass cutoff.
    pub fn set_source_occlusion(&mut self, idx: usize, occlusion: f32) {
        if let Some(s) = self.sources.get_mut(idx) {
            s.occlusion.set_target(occlusion.clamp(0.0, 1.0));
        }
    }

    /// §6.6 per-source reverb send (linear, ramped per sample).
    pub fn set_source_reverb_send(&mut self, idx: usize, send: f32) {
        if let Some(s) = self.sources.get_mut(idx) {
            s.rev_send.set_target(send.max(0.0));
        }
    }

    /// Master reverb mix multiplier (linear; 0 = dry, 1 = unity).
    pub fn set_reverb_amount(&mut self, amount: f32) {
        self.reverb_amount = amount.max(0.0);
    }

    /// §9.1 externalizer amount, `value ∈ [0, 100]`. 0 disables.
    pub fn set_externalizer_amount(&mut self, value: f32) {
        self.externalizer.set_amount(value);
    }

    /// §9.1 externalizer character (tilt EQ asymmetry), `value ∈ [0, 100]`.
    /// 50 = neutral; below 50 brightens (cut lows, boost highs); above
    /// 50 darkens (boost lows, cut highs). ×4 asymmetric on the "cut"
    /// branch of each shelf — bit-verified against the v0.5 wasm.
    pub fn set_externalizer_character(&mut self, value: f32) {
        self.externalizer.set_character(value);
    }

    /// §2.4 `positionMode` (0 = world, 1 = relative/head-locked).
    pub fn set_source_position_mode(&mut self, idx: usize, mode: u8) {
        if let Some(s) = self.sources.get_mut(idx)
            && s.position_mode != mode
        {
            s.position_mode = mode;
            s.pos_dirty = true;
        }
    }

    /// §2.5 `renderingMode` (0 = spatial, 1 = stereo bypass).
    pub fn set_source_rendering_mode(&mut self, idx: usize, mode: u8) {
        if let Some(s) = self.sources.get_mut(idx) {
            s.rendering_mode = mode;
        }
    }

    /// §6.7 input channel count (1 = mono, 2 = stereo).
    pub fn set_source_input_channel_count(&mut self, idx: usize, count: u8) {
        if let Some(s) = self.sources.get_mut(idx) {
            s.input_channel_count = count.clamp(1, 2);
        }
    }

    /// §3 distance curve. Knot gains are linear (caller does dB → linear).
    /// Default curve is `(1 m, 0 dB) (12 m, −20 dB) (60 m, −60 dB) (100 m → 0)`.
    /// Setting a new curve marks the source as `pos_dirty` so the SH
    /// gains get re-encoded with the new distance attenuation next block.
    #[allow(clippy::too_many_arguments)]
    pub fn set_source_distance_curve(
        &mut self,
        idx: usize,
        a_dist: f32,
        a_gain: f32,
        b_dist: f32,
        b_gain: f32,
        c_dist: f32,
        c_gain: f32,
        d_dist: f32,
    ) {
        if let Some(s) = self.sources.get_mut(idx) {
            let new_model = DistanceModel {
                a_dist, a_gain, b_dist, b_gain, c_dist, c_gain, d_dist,
            };
            // Only invalidate if the curve actually changed (avoid
            // perpetual re-encode + crossfade when the host re-sends
            // the same values every block).
            let m = &s.distance_model;
            let changed = m.a_dist != new_model.a_dist
                || m.a_gain != new_model.a_gain
                || m.b_dist != new_model.b_dist
                || m.b_gain != new_model.b_gain
                || m.c_dist != new_model.c_dist
                || m.c_gain != new_model.c_gain
                || m.d_dist != new_model.d_dist;
            if changed {
                s.distance_model = new_model;
                s.pos_dirty = true;
            }
        }
    }

    /// §6.2 directivity-cone parameters. Angles in radians. Defaults
    /// `{0, 2π, 1, 0}` disable the cone.
    pub fn set_source_directivity(
        &mut self,
        idx: usize,
        inner_ang: f32,
        outer_ang: f32,
        outer_gain: f32,
        outer_lp: f32,
    ) {
        if let Some(s) = self.sources.get_mut(idx) {
            s.inner_ang = inner_ang;
            s.outer_ang = outer_ang;
            s.outer_gain = outer_gain;
            s.outer_lp = outer_lp;
        }
    }

    /// `inputs[i]` is the 2-channel input slab for source `i` for this
    /// block (`[ch0, ch1]`; mono sources only read `ch0`). Inactive
    /// sources and any indices past `inputs.len()` are skipped.
    /// `bed_inputs` is one mono buffer per audio-bed channel (§12
    /// step 6). Pass `&[]` for no bed.
    pub fn process_block(
        &mut self,
        inputs: &[[[f32; BLOCK_SIZE]; 2]],
        bed_inputs: &[[f32; BLOCK_SIZE]],
    ) {
        for ch in &mut self.ambi_bus {
            ch.fill(0.0);
        }
        self.reverb_in_bus.fill(0.0);

        for (i, src) in self.sources.iter_mut().enumerate() {
            if !src.active || i >= inputs.len() {
                continue;
            }
            if src.rendering_mode != 0 {
                // §6.9 stereo bypass: mixed into stereo_out below,
                // after the spatial pipeline. Skip here.
                continue;
            }
            process_source(
                src,
                &self.listener,
                self.sample_rate,
                &inputs[i],
                &mut self.ambi_bus,
                &mut self.reverb_in_bus,
            );
        }

        // §12 step 6: audio bed → ambi_bus. Bed does NOT feed the
        // reverb input bus (spec note in §12 step 6).
        if let Some(bed) = self.audio_bed.as_ref() {
            bed.encode(bed_inputs, self.listener.quat, &mut self.ambi_bus);
        }

        if self.reverb_amount > 0.0 {
            let mut rev_outs = [[0.0_f32; BLOCK_SIZE]; FDN_SIZE];
            self.reverb.process(&self.reverb_in_bus, &mut rev_outs);
            let amt = self.reverb_amount;
            for (k, out_k) in rev_outs.iter().enumerate() {
                let basis = &self.rev_sh_bases[k];
                for (ch_idx, ch) in self.ambi_bus.iter_mut().enumerate() {
                    let g = amt * basis[ch_idx];
                    if g == 0.0 {
                        continue;
                    }
                    for i in 0..BLOCK_SIZE {
                        ch[i] += g * out_k[i];
                    }
                }
            }
        }

        if let Some(decoder) = self.decoder.as_mut() {
            decoder.process(&self.ambi_bus, &mut self.stereo_out);
        } else {
            for ch in &mut self.stereo_out {
                ch.fill(0.0);
            }
        }

        // §9 externalizer: in-place transform of stereo_out. Skipped
        // internally when disabled and ramped to zero.
        self.externalizer.process(&mut self.stereo_out);

        // §13 / §12 step 10: W-channel binauralizer adds a diffuse
        // envelopment layer derived from the W (omni) ambisonic channel.
        if let Some(wb) = self.w_binauralizer.as_mut() {
            wb.process_add(&self.ambi_bus[0], &mut self.stereo_out);
        }

        // §6.9 stereo-bypass sources: mix straight into stereo_out
        // after externalizer (bypass skips ALL spatial DSP, including
        // the externalizer per spec §6.9).
        for (i, src) in self.sources.iter_mut().enumerate() {
            if !src.active || i >= inputs.len() || src.rendering_mode == 0 {
                continue;
            }
            let slab = &inputs[i];
            #[allow(clippy::needless_range_loop)]
            for n in 0..BLOCK_SIZE {
                let g = src.gain.tick();
                if src.input_channel_count >= 2 {
                    self.stereo_out[0][n] += g * slab[0][n];
                    self.stereo_out[1][n] += g * slab[1][n];
                } else {
                    let v = g * slab[0][n];
                    self.stereo_out[0][n] += v;
                    self.stereo_out[1][n] += v;
                }
            }
        }
    }
}

fn process_source(
    src: &mut Source,
    listener: &Listener,
    sample_rate: u32,
    input: &[[f32; BLOCK_SIZE]; 2],
    ambi_bus: &mut [[f32; BLOCK_SIZE]],
    reverb_in: &mut [f32; BLOCK_SIZE],
) {
    // §6.1 / §2.4 position_mode. World (0): compute listener-relative
    // via inverse listener transform. Relative (1): use src.pos as-is,
    // it's already in listener frame (head-locked / HUD source).
    let new_rel = if src.position_mode == 0 {
        let delta = src.pos - listener.pos;
        listener.quat.conjugate().rotate(delta)
    } else {
        src.pos
    };
    if new_rel != src.rel_pos {
        src.rel_pos = new_rel;
        src.pos_dirty = true;
    }

    // §6.2 directivity cone (uses world-frame source/listener).
    let (dir_gain_target, dir_lp_offset) = compute_directivity(src, listener);
    if (dir_gain_target - src.directivity_gain.target).abs() > 0.0 {
        src.directivity_gain.set_target(dir_gain_target);
    }
    src.directivity_lp_offset = dir_lp_offset;

    // §6.3 LP coefficient update (once per block; gate per spec).
    let total = (src.occlusion.current + dir_lp_offset).max(0.0);
    if total != src.lp_total_last {
        update_lp_coeffs(src, total, sample_rate);
        src.lp_total_last = total;
    }

    // §6.6 reverb distance attenuation: use the source's distance
    // curve (§3). M10+ may split direct/reverb curves into separate
    // models; for M9 we share one.
    let r = new_rel.length();
    let reverb_dist_atten = src.distance_model.gain_at(r);

    // §6.4 SH re-encode + crossfade flag.
    let was_dirty = src.pos_dirty;
    if was_dirty {
        src.sh_gains_old = src.sh_gains_new;
        compute_sh_gains(new_rel, &src.distance_model, &mut src.sh_gains_new);
        src.pos_dirty = false;
    }

    let n = BLOCK_SIZE as f32;
    let b0 = src.lp_b0;
    let a1 = src.lp_a1;
    let stereo = src.input_channel_count >= 2;
    for i in 0..BLOCK_SIZE {
        // §6.3 1-pole LP per channel: y = b0·x + state ; state = y·a1 + b0·x
        let x0 = input[0][i];
        let y0 = b0 * x0 + src.lp_state[0];
        src.lp_state[0] = y0 * a1 + b0 * x0;
        let (y1, used_stereo) = if stereo {
            let x1 = input[1][i];
            let y = b0 * x1 + src.lp_state[1];
            src.lp_state[1] = y * a1 + b0 * x1;
            (y, true)
        } else {
            (0.0_f32, false)
        };

        let g = src.gain.tick();
        let dg = src.directivity_gain.tick();
        let rs = src.rev_send.tick();
        src.occlusion.tick();

        let g_direct = g * src.direct_path_gain * dg;

        if was_dirty {
            let t = (i as f32) / n;
            for ((ch, &old), &new_) in ambi_bus
                .iter_mut()
                .zip(src.sh_gains_old.iter())
                .zip(src.sh_gains_new.iter())
            {
                let sh = old + t * (new_ - old);
                ch[i] += g_direct * sh * y0;
                if used_stereo {
                    ch[i] += g_direct * sh * y1;
                }
            }
        } else {
            for (ch, &new_) in ambi_bus.iter_mut().zip(src.sh_gains_new.iter()) {
                ch[i] += g_direct * new_ * y0;
                if used_stereo {
                    ch[i] += g_direct * new_ * y1;
                }
            }
        }

        // §6.6 reverb send (note: direct_path_gain & directivity are
        // NOT applied here — reverb path is independent of the cone).
        // Stereo objects mono-sum into the reverb bus.
        if rs > 0.0 {
            let mut mix = y0;
            if used_stereo {
                mix += y1;
            }
            reverb_in[i] += g * rs * reverb_dist_atten * mix;
        }
    }

    // §6.3 denormal flush for both LP states.
    for s in &mut src.lp_state {
        if s.abs() < 1.175e-38 {
            *s = 0.0;
        }
    }
}

/// §6.2 directivity cone. Forward vector uses engine-native +X
/// (verified against the baseline behavior — see development notes). The
/// `r == 0` fallback also uses `(1, 0, 0)`.
fn compute_directivity(src: &Source, listener: &Listener) -> (f32, f32) {
    let forward = src.rotation.rotate(Vec3::new(1.0, 0.0, 0.0));
    let delta = listener.pos - src.pos;
    let r = delta.length();
    let to_listener = if r > 0.0 {
        delta * (1.0 / r)
    } else {
        Vec3::new(1.0, 0.0, 0.0)
    };
    let d = forward.dot(to_listener).clamp(-1.0, 1.0);
    let angle = d.acos();

    let t = if angle <= src.inner_ang {
        0.0
    } else if angle >= src.outer_ang {
        1.0
    } else {
        let span = src.outer_ang - src.inner_ang;
        if span > 0.0 {
            (angle - src.inner_ang) / span
        } else {
            1.0
        }
    };

    let dir_gain = 1.0 + t * (src.outer_gain - 1.0);
    let dir_lp_offset = t * src.outer_lp;
    (dir_gain, dir_lp_offset)
}

fn update_lp_coeffs(src: &mut Source, total: f32, sample_rate: u32) {
    let sr = sample_rate as f32;
    let fc_raw = OCCLUSION_FREQ_BASE.powf(1.0 - total) * OCCLUSION_FREQ_SCALE;
    let fc = fc_raw.min(sr * OCCLUSION_FC_MAX_FACTOR);
    let w = (PI * fc / sr).tan();
    let inv = 1.0 / (1.0 + w);
    src.lp_b0 = w * inv;
    src.lp_a1 = (1.0 - w) * inv;
}

/// §5 SH-encoder wrapper. M9: distance attenuation uses the source's
/// 4-knot curve (§3) instead of the M3 `1/max(r, 0.1)` placeholder.
///
/// Near-field omni blend: across `r ∈ [0, NEAR_OMNI_M]` the
/// directional SH channels (everything but W) fade linearly to 0,
/// so a source coincident with the listener doesn't amplify any HRTF
/// L/R asymmetry into a stereo bias. W (omni) energy stays
/// continuous through the blend.
fn compute_sh_gains(rel: Vec3, model: &DistanceModel, out: &mut [f32; NUM_AMBI]) {
    const NEAR_OMNI_M: f32 = 0.1;

    let r = rel.length();
    let unit = if r > 0.0 {
        rel * (1.0 / r)
    } else {
        Vec3::new(1.0, 0.0, 0.0)
    };
    sh_basis_n3d_into(unit, out);
    let scale = model.gain_at(r) * SH_W_NORM;
    let omni_blend = (r / NEAR_OMNI_M).clamp(0.0, 1.0);
    for (i, v) in out.iter_mut().enumerate() {
        *v *= scale;
        if i != 0 {
            *v *= omni_blend;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

    fn dc_inputs(n: usize, value: f32) -> Vec<[[f32; BLOCK_SIZE]; 2]> {
        vec![[[value; BLOCK_SIZE]; 2]; n]
    }

    fn settle(e: &mut Engine, inputs: &[[[f32; BLOCK_SIZE]; 2]]) {
        // Two blocks settles the BLOCK_SIZE-length gain ramp.
        for _ in 0..2 {
            e.process_block(inputs, &[]);
        }
    }

    fn channel_energy(ch: &[f32; BLOCK_SIZE]) -> f32 {
        ch.iter().map(|v| v * v).sum()
    }

    fn settle_occlusion(e: &mut Engine, inputs: &[[[f32; BLOCK_SIZE]; 2]]) {
        // 1000-sample ramp / 128-sample block → ~8 blocks to fully settle.
        for _ in 0..16 {
            e.process_block(inputs, &[]);
        }
    }

    #[test]
    fn empty_inputs_leave_bus_zero() {
        let mut e = Engine::new(48000, 1);
        e.set_source_active(0, true);
        e.set_source_position(0, 5.0, 0.0, 0.0);
        e.set_source_gain(0, 1.0);
        let inputs: [[[f32; BLOCK_SIZE]; 2]; 0] = [];
        e.process_block(&inputs, &[]);
        for ch in &e.ambi_bus {
            for &v in ch.iter() {
                assert_eq!(v, 0.0);
            }
        }
    }

    #[test]
    fn inactive_source_contributes_nothing() {
        let mut e = Engine::new(48000, 1);
        // Source 0 is inactive by default.
        e.set_source_position(0, 5.0, 0.0, 0.0);
        e.set_source_gain(0, 1.0);
        let input = dc_inputs(1, 1.0);
        e.process_block(&input, &[]);
        for ch in &e.ambi_bus {
            for &v in ch.iter() {
                assert_eq!(v, 0.0);
            }
        }
    }

    #[test]
    fn forward_source_lights_x_channel() {
        let mut e = Engine::new(48000, 1);
        e.set_source_active(0, true);
        e.set_source_position(0, 5.0, 0.0, 0.0);
        e.set_source_gain(0, 1.0);
        let input = dc_inputs(1, 1.0);
        settle(&mut e, &input);

        let w = channel_energy(&e.ambi_bus[0]);
        let x = channel_energy(&e.ambi_bus[3]);
        let y = channel_energy(&e.ambi_bus[1]);
        let z = channel_energy(&e.ambi_bus[2]);

        assert!(w > 0.0);
        assert!(x > 0.0);
        assert!(y < x * 1e-6);
        assert!(z < x * 1e-6);
    }

    #[test]
    fn left_source_lights_y_channel() {
        let mut e = Engine::new(48000, 1);
        e.set_source_active(0, true);
        e.set_source_position(0, 0.0, 5.0, 0.0);
        e.set_source_gain(0, 1.0);
        let input = dc_inputs(1, 1.0);
        settle(&mut e, &input);

        let y = channel_energy(&e.ambi_bus[1]);
        let x = channel_energy(&e.ambi_bus[3]);
        assert!(y > 0.0);
        assert!(x < y * 1e-6);
    }

    #[test]
    fn up_source_lights_z_channel() {
        let mut e = Engine::new(48000, 1);
        e.set_source_active(0, true);
        e.set_source_position(0, 0.0, 0.0, 5.0);
        e.set_source_gain(0, 1.0);
        let input = dc_inputs(1, 1.0);
        settle(&mut e, &input);

        let z = channel_energy(&e.ambi_bus[2]);
        let x = channel_energy(&e.ambi_bus[3]);
        let y = channel_energy(&e.ambi_bus[1]);
        assert!(z > 0.0);
        assert!(x < z * 1e-6);
        assert!(y < z * 1e-6);
    }

    #[test]
    fn closer_source_is_louder() {
        // Default §3 curve: r=1 m → 0 dB (gain 1.0); r=60 m → −60 dB
        // (gain 0.001). Energy ratio ≈ 1e6 — well above the threshold.
        let mut e_near = Engine::new(48000, 1);
        e_near.set_source_active(0, true);
        e_near.set_source_position(0, 1.0, 0.0, 0.0);
        e_near.set_source_gain(0, 1.0);

        let mut e_far = Engine::new(48000, 1);
        e_far.set_source_active(0, true);
        e_far.set_source_position(0, 60.0, 0.0, 0.0);
        e_far.set_source_gain(0, 1.0);

        let input = dc_inputs(1, 1.0);
        settle(&mut e_near, &input);
        settle(&mut e_far, &input);

        let near = channel_energy(&e_near.ambi_bus[0]);
        let far = channel_energy(&e_far.ambi_bus[0]);
        assert!(near > far * 1000.0, "near={near}, far={far}");
    }

    #[test]
    fn position_change_crossfades_within_block() {
        let mut e = Engine::new(48000, 1);
        e.set_source_active(0, true);
        e.set_source_position(0, 5.0, 0.0, 0.0); // forward
        e.set_source_gain(0, 1.0);
        let input = dc_inputs(1, 1.0);
        settle(&mut e, &input);

        // Steady state: X has full energy, Y is silent.
        let y_before = e.ambi_bus[1][BLOCK_SIZE - 1].abs();
        assert!(y_before < 1e-4);

        // Move the source to +Y.
        e.set_source_position(0, 0.0, 5.0, 0.0);
        e.process_block(&input, &[]);

        // During this block Y must ramp up from ~0 toward its target.
        assert!(e.ambi_bus[1][0].abs() < 0.01, "start of crossfade should be ~0");
        assert!(
            e.ambi_bus[1][BLOCK_SIZE - 1].abs() > 0.1,
            "end of crossfade should be near the new SH gain"
        );
    }

    #[test]
    fn listener_rotation_changes_apparent_direction() {
        let mut e = Engine::new(48000, 1);
        e.set_source_active(0, true);
        e.set_source_position(0, 5.0, 0.0, 0.0); // world +X (forward)
        e.set_source_gain(0, 1.0);
        let input = dc_inputs(1, 1.0);
        settle(&mut e, &input);

        // Now rotate listener +90° about Z. Source should appear on
        // the listener's right (native −Y).
        let s = FRAC_PI_4.sin();
        let c = FRAC_PI_4.cos();
        e.set_listener_rotation(c, 0.0, 0.0, s);
        settle(&mut e, &input);

        // ACN[1] (Y) should now be negative (source on the −Y side).
        let y_sample = e.ambi_bus[1][BLOCK_SIZE - 1];
        let x_sample = e.ambi_bus[3][BLOCK_SIZE - 1];
        assert!(y_sample < -0.01, "Y should reflect source on listener's right: got {y_sample}");
        assert!(x_sample.abs() < 0.05, "X should be near zero post-rotation: got {x_sample}");
    }

    #[test]
    fn multiple_sources_sum_into_bus() {
        let mut e = Engine::new(48000, 2);
        e.set_source_active(0, true);
        e.set_source_active(1, true);
        e.set_source_position(0, 5.0, 0.0, 0.0); // forward
        e.set_source_position(1, 0.0, 5.0, 0.0); // left
        e.set_source_gain(0, 1.0);
        e.set_source_gain(1, 1.0);
        let input = dc_inputs(2, 1.0);
        settle(&mut e, &input);

        // Both X and Y channels active.
        let x = channel_energy(&e.ambi_bus[3]);
        let y = channel_energy(&e.ambi_bus[1]);
        assert!(x > 0.0);
        assert!(y > 0.0);
    }

    // ----- M6 -----

    #[test]
    fn direct_path_gain_scales_ambi_energy() {
        let mut e = Engine::new(48000, 1);
        e.set_source_active(0, true);
        e.set_source_position(0, 1.0, 0.0, 0.0);
        e.set_source_gain(0, 1.0);
        let input = dc_inputs(1, 1.0);
        settle(&mut e, &input);
        let base = channel_energy(&e.ambi_bus[0]);

        e.set_source_direct_path_gain(0, 0.5);
        // No ramp on direct_path_gain itself; engine takes effect
        // immediately. Run an extra block for the gain-ramp tail to
        // settle around the new product.
        settle(&mut e, &input);
        let half = channel_energy(&e.ambi_bus[0]);

        // 0.5² = 0.25.
        let ratio = half / base;
        assert!((ratio - 0.25).abs() < 0.02, "ratio={ratio}");
    }

    #[test]
    fn occlusion_zero_passes_dc_unchanged() {
        // At total=0 the LP cutoff is clipped to 0.425·fs, which still
        // passes DC essentially unattenuated. Same energy as no occlusion.
        let mut e = Engine::new(48000, 1);
        e.set_source_active(0, true);
        e.set_source_position(0, 1.0, 0.0, 0.0);
        e.set_source_gain(0, 1.0);
        e.set_source_occlusion(0, 0.0);
        let input = dc_inputs(1, 1.0);
        settle_occlusion(&mut e, &input);
        let energy = channel_energy(&e.ambi_bus[0]);
        assert!(energy > 0.0);
    }

    #[test]
    fn occlusion_one_attenuates_hf() {
        // At total=1 the LP cutoff is ~20 Hz. A 4 kHz tone should be
        // attenuated by orders of magnitude relative to no occlusion.
        let sr = 48000.0_f32;
        let freq = 4000.0_f32;
        let blocks = 32; // covers occlusion ramp + filter settling

        let mut sine_inputs: Vec<[[f32; BLOCK_SIZE]; 2]> = Vec::with_capacity(blocks);
        let mut phase = 0.0_f32;
        let step = 2.0 * PI * freq / sr;
        for _ in 0..blocks {
            let mut b = [[0.0_f32; BLOCK_SIZE]; 2];
            #[allow(clippy::needless_range_loop)]
            for i in 0..BLOCK_SIZE {
                let s = phase.sin();
                b[0][i] = s;
                b[1][i] = s;
                phase += step;
                if phase > TAU {
                    phase -= TAU;
                }
            }
            sine_inputs.push(b);
        }

        let mut e_open = Engine::new(48000, 1);
        e_open.set_source_active(0, true);
        e_open.set_source_position(0, 1.0, 0.0, 0.0);
        e_open.set_source_gain(0, 1.0);
        e_open.set_source_occlusion(0, 0.0);
        for b in &sine_inputs {
            e_open.process_block(std::slice::from_ref(b), &[]);
        }
        let open_energy: f32 = e_open.ambi_bus[0].iter().map(|v| v * v).sum();

        let mut e_occl = Engine::new(48000, 1);
        e_occl.set_source_active(0, true);
        e_occl.set_source_position(0, 1.0, 0.0, 0.0);
        e_occl.set_source_gain(0, 1.0);
        e_occl.set_source_occlusion(0, 1.0);
        for b in &sine_inputs {
            e_occl.process_block(std::slice::from_ref(b), &[]);
        }
        let occl_energy: f32 = e_occl.ambi_bus[0].iter().map(|v| v * v).sum();

        assert!(open_energy > 0.0);
        assert!(
            occl_energy * 1000.0 < open_energy,
            "occluded HF should be << open: open={open_energy}, occl={occl_energy}"
        );
    }

    #[test]
    fn directivity_off_axis_uses_outer_gain() {
        // Source at +X, oriented facing +X (identity quat). Listener
        // at origin → to_listener = (−1, 0, 0). forward = (+1, 0, 0).
        // Angle = π (180° off-axis).
        let mut e = Engine::new(48000, 1);
        e.set_source_active(0, true);
        e.set_source_position(0, 1.0, 0.0, 0.0);
        e.set_source_gain(0, 1.0);
        // Narrow cone: inner=10°, outer=20°, outerGain=0.1, outerLP=0.
        e.set_source_directivity(0, 10.0_f32.to_radians(), 20.0_f32.to_radians(), 0.1, 0.0);
        let input = dc_inputs(1, 1.0);
        // Two blocks settle gain ramp and directivity-gain ramp.
        settle(&mut e, &input);

        let off_axis = channel_energy(&e.ambi_bus[0]);

        // Now flip the source to face the listener (rotate 180° about Z).
        let s = FRAC_PI_2.sin();
        let c = FRAC_PI_2.cos();
        e.set_source_rotation(0, c, 0.0, 0.0, s);
        // Two more blocks settle the directivity-gain ramp back to 1.
        settle(&mut e, &input);

        let on_axis = channel_energy(&e.ambi_bus[0]);

        // off_axis should be ~outerGain² × on_axis = 0.01·on_axis.
        let ratio = off_axis / on_axis;
        assert!(
            ratio < 0.04 && ratio > 0.005,
            "off/on ratio outside expected outerGain² band: {ratio}"
        );
    }

    // ----- M7 -----

    #[test]
    fn reverb_send_zero_yields_no_reverb_tail() {
        // Source with rev_send=0 must not energise the reverb bus, so
        // long after the impulse the bus should be silent.
        let mut e = Engine::new(48000, 1);
        e.set_source_active(0, true);
        e.set_source_position(0, 5.0, 0.0, 0.0);
        e.set_source_gain(0, 1.0);
        e.set_source_reverb_send(0, 0.0);
        e.set_reverb_amount(1.0);

        let mut input = [[[0.0_f32; BLOCK_SIZE]; 2]; 1];
        input[0][0][0] = 1.0;
        e.process_block(&input, &[]);

        input[0][0].fill(0.0); input[0][1].fill(0.0);
        for _ in 0..200 {
            e.process_block(&input, &[]);
        }
        let bus_energy: f32 = e.reverb_in_bus.iter().map(|v| v * v).sum();
        assert_eq!(bus_energy, 0.0);
    }

    #[test]
    fn reverb_send_lights_bus_and_ambi() {
        // With rev_send>0 and reverb_amount>0 the reverb tail should
        // spread across multiple ambi channels (not just W) after the
        // FDN settles — because the 8 outputs SH-encode at distinct
        // §13 directions.
        let mut e = Engine::new(48000, 1);
        e.set_source_active(0, true);
        e.set_source_position(0, 3.0, 0.0, 0.0);
        e.set_source_gain(0, 1.0);
        e.set_source_reverb_send(0, 1.0);
        e.set_reverb_amount(1.0);

        let mut input = [[[0.0_f32; BLOCK_SIZE]; 2]; 1];
        input[0][0][0] = 1.0;
        e.process_block(&input, &[]);

        input[0][0].fill(0.0); input[0][1].fill(0.0);
        // Settle through the FDN.
        for _ in 0..150 {
            e.process_block(&input, &[]);
        }

        // Energy in the directional ambi channels (Y/Z/X = 1/2/3)
        // should be non-zero after the reverb spreads.
        let mut total = 0.0_f32;
        for k in 1..NUM_AMBI {
            for &v in e.ambi_bus[k].iter() {
                total += v * v;
            }
        }
        assert!(total > 0.0, "reverb didn't spread to directional ambi channels");
    }

    #[test]
    fn reverb_amount_zero_disables_reverb_in_ambi() {
        let mut e = Engine::new(48000, 1);
        e.set_source_active(0, true);
        e.set_source_position(0, 3.0, 0.0, 0.0);
        e.set_source_gain(0, 1.0);
        e.set_source_reverb_send(0, 1.0);
        e.set_reverb_amount(0.0);

        let mut input = [[[0.0_f32; BLOCK_SIZE]; 2]; 1];
        input[0][0][0] = 1.0;
        e.process_block(&input, &[]);

        // No direct positional energy in Y (source is on +X), and with
        // reverb_amount=0 no reverb energy should reach Y either, so Y
        // stays silent after the gain ramp settles.
        input[0][0].fill(0.0); input[0][1].fill(0.0);
        for _ in 0..150 {
            e.process_block(&input, &[]);
        }
        let y_energy: f32 = e.ambi_bus[1].iter().map(|v| v * v).sum();
        assert!(y_energy < 1e-10, "reverb_amount=0 should not feed ambi: {y_energy}");
    }

    // ----- M9 -----

    #[test]
    fn distance_curve_changes_falloff() {
        // Default curve at r=12 → −20 dB linear 0.1. Replace with a
        // curve that's −60 dB at the same distance → ambi energy drops
        // by ~1000× squared.
        let inputs = dc_inputs(1, 1.0);

        let mut e_default = Engine::new(48000, 1);
        e_default.set_source_active(0, true);
        e_default.set_source_position(0, 12.0, 0.0, 0.0);
        e_default.set_source_gain(0, 1.0);
        settle(&mut e_default, &inputs);
        let default_energy = channel_energy(&e_default.ambi_bus[0]);

        let mut e_steep = Engine::new(48000, 1);
        e_steep.set_source_active(0, true);
        e_steep.set_source_position(0, 12.0, 0.0, 0.0);
        e_steep.set_source_gain(0, 1.0);
        // Steeper curve: knot B at 12 m → 0.001 (−60 dB).
        e_steep.set_source_distance_curve(0, 1.0, 1.0, 12.0, 0.001, 60.0, 0.0001, 100.0);
        settle(&mut e_steep, &inputs);
        let steep_energy = channel_energy(&e_steep.ambi_bus[0]);

        // 0.1² vs 0.001² ≈ 10,000× energy ratio.
        assert!(
            default_energy > steep_energy * 1000.0,
            "default={default_energy}, steep={steep_energy}"
        );
    }

    #[test]
    fn directivity_default_is_no_op() {
        // With defaults {0, 2π, 1, 0} the cone is disabled — moving
        // the source orientation has no effect on energy.
        let mut e = Engine::new(48000, 1);
        e.set_source_active(0, true);
        e.set_source_position(0, 1.0, 0.0, 0.0);
        e.set_source_gain(0, 1.0);
        let input = dc_inputs(1, 1.0);
        settle(&mut e, &input);
        let a = channel_energy(&e.ambi_bus[0]);

        let s = FRAC_PI_4.sin();
        let c = FRAC_PI_4.cos();
        e.set_source_rotation(0, c, 0.0, 0.0, s);
        settle(&mut e, &input);
        let b = channel_energy(&e.ambi_bus[0]);

        assert!((a - b).abs() / a < 1e-3, "default cone should not change energy: a={a}, b={b}");
    }
}
