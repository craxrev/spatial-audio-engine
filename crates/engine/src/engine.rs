//! Top-level engine. M3 covered listener + sources + ambisonic bus +
//! per-block process loop. M4 added the HRTF binaural decode. M6
//! adds source orientation, the directivity cone (§6.2), occlusion +
//! directivity low-pass (§6.3), and `direct_path_gain` (§6.4).
//! Reverb (M7), externalizer (M8), and the rest of §12 land later.

use core::f32::consts::PI;

use crate::consts::{
    BLOCK_SIZE, NUM_AMBI, OCCLUSION_FC_MAX_FACTOR, OCCLUSION_FREQ_BASE, OCCLUSION_FREQ_SCALE,
    OUTPUT_CHANNELS, SH_W_NORM,
};
use crate::decoder::HrtfDecoder;
use crate::hrtf::Hrtf;
use crate::math::{Quat, Vec3};
use crate::sh::sh_basis_n3d_into;
use crate::source::Source;

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
    decoder: Option<HrtfDecoder>,
}

impl Engine {
    pub fn new(sample_rate: u32, num_sources: usize) -> Self {
        Self {
            sample_rate,
            listener: Listener::default(),
            sources: (0..num_sources).map(|_| Source::default()).collect(),
            ambi_bus: vec![[0.0; BLOCK_SIZE]; NUM_AMBI],
            stereo_out: [[0.0; BLOCK_SIZE]; OUTPUT_CHANNELS],
            decoder: None,
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

    /// `inputs[i]` is the mono input for source `i` for this block.
    /// Inactive sources and any indices past `inputs.len()` are
    /// skipped. The ambi bus is zeroed at the start of each call.
    /// If a main HRTF is loaded, `stereo_out` carries the binaural
    /// decode at end of block; otherwise it is cleared.
    pub fn process_block(&mut self, inputs: &[[f32; BLOCK_SIZE]]) {
        for ch in &mut self.ambi_bus {
            ch.fill(0.0);
        }
        for (i, src) in self.sources.iter_mut().enumerate() {
            if !src.active || i >= inputs.len() {
                continue;
            }
            process_source(
                src,
                &self.listener,
                self.sample_rate,
                &inputs[i],
                &mut self.ambi_bus,
            );
        }
        if let Some(decoder) = self.decoder.as_mut() {
            decoder.process(&self.ambi_bus, &mut self.stereo_out);
        } else {
            for ch in &mut self.stereo_out {
                ch.fill(0.0);
            }
        }
    }
}

fn process_source(
    src: &mut Source,
    listener: &Listener,
    sample_rate: u32,
    input: &[f32; BLOCK_SIZE],
    ambi_bus: &mut [[f32; BLOCK_SIZE]],
) {
    // §6.1 listener-relative position.
    let delta = src.pos - listener.pos;
    let new_rel = listener.quat.conjugate().rotate(delta);
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

    // §6.4 SH re-encode + crossfade flag.
    let was_dirty = src.pos_dirty;
    if was_dirty {
        src.sh_gains_old = src.sh_gains_new;
        compute_sh_gains(new_rel, &mut src.sh_gains_new);
        src.pos_dirty = false;
    }

    let n = BLOCK_SIZE as f32;
    let b0 = src.lp_b0;
    let a1 = src.lp_a1;
    for i in 0..BLOCK_SIZE {
        // §6.3 1-pole LP: y = b0·x + state ; state = y·a1 + b0·x
        let x = input[i];
        let y = b0 * x + src.lp_state;
        src.lp_state = y * a1 + b0 * x;

        // Per-sample ramps.
        let g = src.gain.tick();
        let dg = src.directivity_gain.tick();
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
                ch[i] += g_direct * sh * y;
            }
        } else {
            for (ch, &new_) in ambi_bus.iter_mut().zip(src.sh_gains_new.iter()) {
                ch[i] += g_direct * new_ * y;
            }
        }
    }

    // §6.3 denormal flush.
    if src.lp_state.abs() < 1.175e-38 {
        src.lp_state = 0.0;
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

/// §5 SH-encoder wrapper without the curve override (M3 uses the
/// `1/max(r, 0.1)` fallback). `rel` is in the listener-local frame.
fn compute_sh_gains(rel: Vec3, out: &mut [f32; NUM_AMBI]) {
    let r = rel.length();
    let (unit, dist_atten) = if r > 0.0 {
        (rel * (1.0 / r), 1.0 / r.max(0.1))
    } else {
        (Vec3::new(1.0, 0.0, 0.0), 10.0)
    };
    sh_basis_n3d_into(unit, out);
    let scale = dist_atten * SH_W_NORM;
    for v in out.iter_mut() {
        *v *= scale;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

    fn dc_inputs(n: usize, value: f32) -> Vec<[f32; BLOCK_SIZE]> {
        vec![[value; BLOCK_SIZE]; n]
    }

    fn settle(e: &mut Engine, inputs: &[[f32; BLOCK_SIZE]]) {
        // Two blocks settles the BLOCK_SIZE-length gain ramp.
        for _ in 0..2 {
            e.process_block(inputs);
        }
    }

    fn channel_energy(ch: &[f32; BLOCK_SIZE]) -> f32 {
        ch.iter().map(|v| v * v).sum()
    }

    fn settle_occlusion(e: &mut Engine, inputs: &[[f32; BLOCK_SIZE]]) {
        // 1000-sample ramp / 128-sample block → ~8 blocks to fully settle.
        for _ in 0..16 {
            e.process_block(inputs);
        }
    }

    #[test]
    fn empty_inputs_leave_bus_zero() {
        let mut e = Engine::new(48000, 1);
        e.set_source_active(0, true);
        e.set_source_position(0, 5.0, 0.0, 0.0);
        e.set_source_gain(0, 1.0);
        let inputs: [[f32; BLOCK_SIZE]; 0] = [];
        e.process_block(&inputs);
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
        e.process_block(&input);
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
        // 1/max(r, 0.1) attenuation: r=1 → 1.0, r=10 → 0.1.
        let mut e_near = Engine::new(48000, 1);
        e_near.set_source_active(0, true);
        e_near.set_source_position(0, 1.0, 0.0, 0.0);
        e_near.set_source_gain(0, 1.0);

        let mut e_far = Engine::new(48000, 1);
        e_far.set_source_active(0, true);
        e_far.set_source_position(0, 10.0, 0.0, 0.0);
        e_far.set_source_gain(0, 1.0);

        let input = dc_inputs(1, 1.0);
        settle(&mut e_near, &input);
        settle(&mut e_far, &input);

        let near = channel_energy(&e_near.ambi_bus[0]);
        let far = channel_energy(&e_far.ambi_bus[0]);
        // Energy ratio should be ~100 (gain ratio squared).
        assert!(near > far * 50.0, "near={near}, far={far}");
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
        e.process_block(&input);

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

        let mut sine_inputs: Vec<[f32; BLOCK_SIZE]> = Vec::with_capacity(blocks);
        let mut phase = 0.0_f32;
        let step = 2.0 * PI * freq / sr;
        for _ in 0..blocks {
            let mut b = [0.0_f32; BLOCK_SIZE];
            for v in &mut b {
                *v = phase.sin();
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
            e_open.process_block(std::slice::from_ref(b));
        }
        let open_energy: f32 = e_open.ambi_bus[0].iter().map(|v| v * v).sum();

        let mut e_occl = Engine::new(48000, 1);
        e_occl.set_source_active(0, true);
        e_occl.set_source_position(0, 1.0, 0.0, 0.0);
        e_occl.set_source_gain(0, 1.0);
        e_occl.set_source_occlusion(0, 1.0);
        for b in &sine_inputs {
            e_occl.process_block(std::slice::from_ref(b));
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
