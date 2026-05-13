//! Top-level engine. For M3: listener, N sources, ambisonic bus,
//! and a per-block process loop. HRTF decode (M4), reverb (M7),
//! externalizer (M8), and the rest of the §12 pipeline land in
//! later milestones; M3 stops at the encoded ambi bus.

use crate::consts::{BLOCK_SIZE, NUM_AMBI, OUTPUT_CHANNELS, SH_W_NORM};
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
            process_source(src, &self.listener, &inputs[i], &mut self.ambi_bus);
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
    input: &[f32; BLOCK_SIZE],
    ambi_bus: &mut [[f32; BLOCK_SIZE]],
) {
    let delta = src.pos - listener.pos;
    let new_rel = listener.quat.conjugate().rotate(delta);
    if new_rel != src.rel_pos {
        src.rel_pos = new_rel;
        src.pos_dirty = true;
    }

    let was_dirty = src.pos_dirty;
    if was_dirty {
        src.sh_gains_old = src.sh_gains_new;
        compute_sh_gains(new_rel, &mut src.sh_gains_new);
        src.pos_dirty = false;
    }

    let n = BLOCK_SIZE as f32;
    for i in 0..BLOCK_SIZE {
        let g = src.gain.tick();
        let s = input[i];
        if was_dirty {
            let t = (i as f32) / n;
            for ((ch, &old), &new_) in ambi_bus
                .iter_mut()
                .zip(src.sh_gains_old.iter())
                .zip(src.sh_gains_new.iter())
            {
                let sh = old + t * (new_ - old);
                ch[i] += g * sh * s;
            }
        } else {
            for (ch, &new_) in ambi_bus.iter_mut().zip(src.sh_gains_new.iter()) {
                ch[i] += g * new_ * s;
            }
        }
    }
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

    fn dc_inputs(n: usize, value: f32) -> Vec<[f32; BLOCK_SIZE]> {
        vec![[value; BLOCK_SIZE]; n]
    }

    fn settle(e: &mut Engine, inputs: &[[f32; BLOCK_SIZE]]) {
        // 2 blocks is enough for the BLOCK_SIZE-length gain ramp.
        for _ in 0..2 {
            e.process_block(inputs);
        }
    }

    fn channel_energy(ch: &[f32; BLOCK_SIZE]) -> f32 {
        ch.iter().map(|v| v * v).sum()
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
        let s = std::f32::consts::FRAC_PI_4.sin();
        let c = std::f32::consts::FRAC_PI_4.cos();
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
}
