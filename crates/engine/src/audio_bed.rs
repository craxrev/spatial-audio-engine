//! §12 step 6: non-ambisonic audio bed. Multichannel input (mono /
//! stereo / 5.1 / 5.1.2 / 5.1.4 / 7.1 / 7.1.2 / 7.1.4) statically
//! SH-encoded into the ambisonic bus.
//!
//! Each channel has a fixed speaker direction (ITU surround positions
//! in engine-native frame: +X forward, +Y left, +Z up). LFE channels
//! are encoded as omni (W only). When the bed is *world-locked* the
//! engine rotates each non-LFE direction by the listener-inverse
//! quaternion before SH-encoding; when *headlocked* the speakers
//! follow the listener's head.
//!
//! Ambisonic-input beds (1st/2nd/3rd order) are M14 territory and
//! use Wigner-D rotation instead of this static encoding path.

use crate::consts::{BLOCK_SIZE, NUM_AMBI, SH_W_NORM};
use crate::math::{Quat, Vec3};
use crate::sh::sh_basis_n3d_into;
use crate::sh_rotation::ShRotation;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum BedFormat {
    NoInput        = 0,
    Mono           = 1,
    Stereo         = 2,
    Surround5_1    = 3,
    Surround5_1_2  = 4,
    Surround5_1_4  = 5,
    Surround7_1    = 6,
    Surround7_1_2  = 7,
    Surround7_1_4  = 8,
    /// 1st-order ambisonic input (W, Y, Z, X) — 4 channels.
    Ambisonics1st  = 9,
    /// 2nd-order ambisonic input — 9 channels.
    Ambisonics2nd  = 10,
    /// 3rd-order ambisonic input — 16 channels.
    Ambisonics3rd  = 11,
}

impl BedFormat {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(BedFormat::NoInput),
            1 => Some(BedFormat::Mono),
            2 => Some(BedFormat::Stereo),
            3 => Some(BedFormat::Surround5_1),
            4 => Some(BedFormat::Surround5_1_2),
            5 => Some(BedFormat::Surround5_1_4),
            6 => Some(BedFormat::Surround7_1),
            7 => Some(BedFormat::Surround7_1_2),
            8 => Some(BedFormat::Surround7_1_4),
            9 => Some(BedFormat::Ambisonics1st),
            10 => Some(BedFormat::Ambisonics2nd),
            11 => Some(BedFormat::Ambisonics3rd),
            _ => None,
        }
    }

    pub fn channel_count(self) -> usize {
        match self {
            BedFormat::NoInput        => 0,
            BedFormat::Mono           => 1,
            BedFormat::Stereo         => 2,
            BedFormat::Surround5_1    => 6,
            BedFormat::Surround5_1_2  => 8,
            BedFormat::Surround5_1_4  => 10,
            BedFormat::Surround7_1    => 8,
            BedFormat::Surround7_1_2  => 10,
            BedFormat::Surround7_1_4  => 12,
            BedFormat::Ambisonics1st  => 4,
            BedFormat::Ambisonics2nd  => 9,
            BedFormat::Ambisonics3rd  => 16,
        }
    }

    pub fn is_ambisonic(self) -> bool {
        matches!(
            self,
            BedFormat::Ambisonics1st | BedFormat::Ambisonics2nd | BedFormat::Ambisonics3rd
        )
    }
}

#[derive(Clone, Copy, Debug)]
struct Speaker {
    direction: Vec3,
    is_lfe: bool,
}

fn dir(az_deg: f32, el_deg: f32) -> Vec3 {
    let az = az_deg.to_radians();
    let el = el_deg.to_radians();
    let ce = el.cos();
    Vec3::new(ce * az.cos(), ce * az.sin(), el.sin())
}

fn spk(az: f32, el: f32) -> Speaker {
    Speaker { direction: dir(az, el), is_lfe: false }
}

fn lfe() -> Speaker {
    Speaker { direction: Vec3::ZERO, is_lfe: true }
}

/// Standard ITU + Atmos channel directions for each format.
/// Azimuth in engine-native: 0° = front (+X), +90° = left (+Y),
/// ±180° = back. Elevation: 0° = horizon, +30° for height layers.
fn speakers_for(format: BedFormat) -> Vec<Speaker> {
    match format {
        BedFormat::NoInput => vec![],

        BedFormat::Mono => vec![spk(0.0, 0.0)],

        BedFormat::Stereo => vec![
            spk( 30.0, 0.0),  // L
            spk(-30.0, 0.0),  // R
        ],

        BedFormat::Surround5_1 => vec![
            spk( 30.0,  0.0),   // L
            spk(-30.0,  0.0),   // R
            spk(  0.0,  0.0),   // C
            lfe(),              // LFE
            spk(110.0,  0.0),   // Ls
            spk(-110.0, 0.0),   // Rs
        ],

        BedFormat::Surround5_1_2 => {
            let mut v = speakers_for(BedFormat::Surround5_1);
            v.push(spk( 30.0, 30.0));  // Tl
            v.push(spk(-30.0, 30.0));  // Tr
            v
        }

        BedFormat::Surround5_1_4 => {
            let mut v = speakers_for(BedFormat::Surround5_1);
            v.push(spk( 30.0,  30.0));  // Tlf
            v.push(spk(-30.0,  30.0));  // Trf
            v.push(spk( 150.0, 30.0));  // Tlb
            v.push(spk(-150.0, 30.0));  // Trb
            v
        }

        BedFormat::Surround7_1 => vec![
            spk( 30.0,  0.0),   // L
            spk(-30.0,  0.0),   // R
            spk(  0.0,  0.0),   // C
            lfe(),              // LFE
            spk( 90.0,  0.0),   // Ls
            spk(-90.0,  0.0),   // Rs
            spk( 150.0, 0.0),   // Lsb
            spk(-150.0, 0.0),   // Rsb
        ],

        BedFormat::Surround7_1_2 => {
            let mut v = speakers_for(BedFormat::Surround7_1);
            v.push(spk( 30.0, 30.0));
            v.push(spk(-30.0, 30.0));
            v
        }

        BedFormat::Surround7_1_4 => {
            let mut v = speakers_for(BedFormat::Surround7_1);
            v.push(spk( 30.0,  30.0));
            v.push(spk(-30.0,  30.0));
            v.push(spk( 150.0, 30.0));
            v.push(spk(-150.0, 30.0));
            v
        }

        // Ambisonic formats don't use speaker directions; the
        // encode path branches via `BedFormat::is_ambisonic()`.
        BedFormat::Ambisonics1st
        | BedFormat::Ambisonics2nd
        | BedFormat::Ambisonics3rd => vec![],
    }
}

pub struct AudioBed {
    format: BedFormat,
    speakers: Vec<Speaker>,
    pub headlocked: bool,
    /// Master linear gain. Unramped for M12 — the per-source gain
    /// ramps in §6.5 don't apply to the bed; if smoothing is needed
    /// later add a Ramp here.
    pub gain: f32,

    /// Cached prev rotation matrix for the §12 ambisonic-bed
    /// crossfade. Starts at identity, updated each block from the
    /// previous target.
    prev_rot: ShRotation,
}

impl AudioBed {
    pub fn new(format: BedFormat) -> Self {
        Self {
            format,
            speakers: speakers_for(format),
            headlocked: false,
            gain: 1.0,
            prev_rot: ShRotation::identity(),
        }
    }

    pub fn format(&self) -> BedFormat {
        self.format
    }

    pub fn channel_count(&self) -> usize {
        self.format.channel_count()
    }

    /// Encode bed channels into the ambisonic bus. `inputs.len()`
    /// must match the configured channel count or the call is a no-op
    /// (matches spec §12 step 6 `format_match` guard).
    pub fn encode(
        &mut self,
        inputs: &[[f32; BLOCK_SIZE]],
        listener_quat: Quat,
        ambi_bus: &mut [[f32; BLOCK_SIZE]],
    ) {
        if inputs.len() != self.channel_count() || self.gain == 0.0 {
            return;
        }
        if self.format.is_ambisonic() {
            self.encode_ambisonic(inputs, listener_quat, ambi_bus);
            return;
        }
        let inv = listener_quat.conjugate();
        let mut sh = [0.0_f32; NUM_AMBI];
        for (ch, spec) in self.speakers.iter().enumerate() {
            // Build the SH coefficients for this channel's effective
            // direction, then accumulate the channel's audio scaled
            // by each coefficient into the matching ambi bus channel.
            if spec.is_lfe {
                // LFE → omni: W only.
                sh.fill(0.0);
                sh[0] = SH_W_NORM;
            } else {
                let d = if self.headlocked {
                    spec.direction
                } else {
                    inv.rotate(spec.direction)
                };
                sh_basis_n3d_into(d, &mut sh);
                for v in sh.iter_mut() {
                    *v *= SH_W_NORM;
                }
            }
            let g = self.gain;
            let src = &inputs[ch];
            for (k, bus_ch) in ambi_bus.iter_mut().enumerate() {
                let coef = g * sh[k];
                if coef == 0.0 {
                    continue;
                }
                #[allow(clippy::needless_range_loop)]
                for i in 0..BLOCK_SIZE {
                    bus_ch[i] += coef * src[i];
                }
            }
        }
    }

    /// §12 step 6 ambisonic branch. Each input channel is already an
    /// SH coefficient (ACN ordering). We apply the listener rotation
    /// (passive: `quat⁻¹`) to keep the bed world-locked, crossfading
    /// from the previous block's rotation to this block's rotation
    /// across the 128 samples to hide zipper noise.
    fn encode_ambisonic(
        &mut self,
        inputs: &[[f32; BLOCK_SIZE]],
        listener_quat: Quat,
        ambi_bus: &mut [[f32; BLOCK_SIZE]],
    ) {
        let target_rot = if self.headlocked {
            ShRotation::identity()
        } else {
            ShRotation::from_quat(listener_quat.conjugate())
        };

        let n_ch = inputs.len();
        let g = self.gain;
        let mut in_buf = [0.0_f32; NUM_AMBI];
        let mut out_prev = [0.0_f32; NUM_AMBI];
        let mut out_target = [0.0_f32; NUM_AMBI];

        #[allow(clippy::needless_range_loop)]
        for i in 0..BLOCK_SIZE {
            for (ch, src) in inputs.iter().enumerate().take(n_ch) {
                in_buf[ch] = src[i];
            }
            // Unused higher-order channels (e.g. for 1st-order input
            // with NUM_AMBI = 16) stay zero from a previous iteration.
            for ch in n_ch..NUM_AMBI {
                in_buf[ch] = 0.0;
            }

            self.prev_rot.apply(&in_buf, &mut out_prev);
            target_rot.apply(&in_buf, &mut out_target);

            let t = (i as f32) / (BLOCK_SIZE as f32);
            let one_m_t = 1.0 - t;
            for (k, bus_ch) in ambi_bus.iter_mut().enumerate() {
                bus_ch[i] += g * (one_m_t * out_prev[k] + t * out_target[k]);
            }
        }

        self.prev_rot = target_rot;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn channel_counts_match_spec() {
        assert_eq!(BedFormat::NoInput.channel_count(), 0);
        assert_eq!(BedFormat::Mono.channel_count(), 1);
        assert_eq!(BedFormat::Stereo.channel_count(), 2);
        assert_eq!(BedFormat::Surround5_1.channel_count(), 6);
        assert_eq!(BedFormat::Surround5_1_2.channel_count(), 8);
        assert_eq!(BedFormat::Surround5_1_4.channel_count(), 10);
        assert_eq!(BedFormat::Surround7_1.channel_count(), 8);
        assert_eq!(BedFormat::Surround7_1_2.channel_count(), 10);
        assert_eq!(BedFormat::Surround7_1_4.channel_count(), 12);
    }

    fn ambi_zero() -> Vec<[f32; BLOCK_SIZE]> {
        vec![[0.0; BLOCK_SIZE]; NUM_AMBI]
    }

    fn dc_channels(n: usize, v: f32) -> Vec<[f32; BLOCK_SIZE]> {
        vec![[v; BLOCK_SIZE]; n]
    }

    fn channel_energy(ch: &[f32; BLOCK_SIZE]) -> f32 {
        ch.iter().map(|x| x * x).sum()
    }

    #[test]
    fn mono_bed_lights_w_and_x() {
        // Mono bed has one speaker at front (+X) → W and X lit, Y and Z zero.
        let mut bed = AudioBed::new(BedFormat::Mono);
        let mut bus = ambi_zero();
        bed.encode(&dc_channels(1, 1.0), Quat::IDENTITY, &mut bus);
        let w = channel_energy(&bus[0]);
        let y = channel_energy(&bus[1]);
        let z = channel_energy(&bus[2]);
        let x = channel_energy(&bus[3]);
        assert!(w > 0.0 && x > 0.0);
        assert!(y < w * 1e-6 && z < w * 1e-6);
    }

    #[test]
    fn stereo_bed_lights_y_channel() {
        // Stereo bed: L at +30°, R at −30°. The +Y/−Y projection
        // is mirrored, but two channels with the same DC input
        // accumulate non-zero Y if their signals differ. Use opposite
        // signs to exercise the L/R asymmetry.
        let mut bed = AudioBed::new(BedFormat::Stereo);
        let mut bus = ambi_zero();
        let mut inputs: Vec<[f32; BLOCK_SIZE]> = vec![[0.0; BLOCK_SIZE]; 2];
        #[allow(clippy::needless_range_loop)]
        for i in 0..BLOCK_SIZE {
            inputs[0][i] =  1.0;
            inputs[1][i] = -1.0;
        }
        bed.encode(&inputs, Quat::IDENTITY, &mut bus);
        let y = channel_energy(&bus[1]);
        let x = channel_energy(&bus[3]);
        assert!(y > 0.0, "Y should be non-zero with anti-phase L/R");
        assert!(x < y * 1e-3, "X should cancel: X={x}, Y={y}");
    }

    #[test]
    fn lfe_only_lights_w() {
        // 5.1 with the LFE channel only (index 3): should only light
        // the W channel of the ambi bus.
        let mut bed = AudioBed::new(BedFormat::Surround5_1);
        let mut bus = ambi_zero();
        let mut inputs: Vec<[f32; BLOCK_SIZE]> = vec![[0.0; BLOCK_SIZE]; 6];
        inputs[3] = [1.0; BLOCK_SIZE]; // LFE only
        bed.encode(&inputs, Quat::IDENTITY, &mut bus);
        let w = channel_energy(&bus[0]);
        assert!(w > 0.0);
        #[allow(clippy::needless_range_loop)]
        for k in 1..NUM_AMBI {
            assert!(
                channel_energy(&bus[k]) < w * 1e-6,
                "LFE leaked into ambi[{k}]"
            );
        }
    }

    #[test]
    fn channel_count_mismatch_is_noop() {
        let mut bed = AudioBed::new(BedFormat::Surround5_1);
        let mut bus = ambi_zero();
        bed.encode(&dc_channels(2, 1.0), Quat::IDENTITY, &mut bus); // wrong count
        for ch in &bus {
            assert!(ch.iter().all(|v| *v == 0.0));
        }
    }

    // ----- M14 ambisonic-bed tests -----

    #[test]
    fn ambisonic_channel_counts() {
        assert_eq!(BedFormat::Ambisonics1st.channel_count(), 4);
        assert_eq!(BedFormat::Ambisonics2nd.channel_count(), 9);
        assert_eq!(BedFormat::Ambisonics3rd.channel_count(), 16);
        assert!(BedFormat::Ambisonics1st.is_ambisonic());
        assert!(!BedFormat::Stereo.is_ambisonic());
    }

    #[test]
    fn ambisonic_identity_listener_passes_through() {
        // 1st-order input with identity listener: bed should appear
        // directly in ambi_bus[0..4], unchanged after rotation.
        let mut bed = AudioBed::new(BedFormat::Ambisonics1st);
        let mut bus = ambi_zero();
        let mut inputs: Vec<[f32; BLOCK_SIZE]> = vec![[0.0; BLOCK_SIZE]; 4];
        // Put W=1, Y=0.5, Z=-0.25, X=0.75 across the block.
        #[allow(clippy::needless_range_loop)]
        for i in 0..BLOCK_SIZE {
            inputs[0][i] =  1.0;   // W
            inputs[1][i] =  0.5;   // Y
            inputs[2][i] = -0.25;  // Z
            inputs[3][i] =  0.75;  // X
        }
        bed.encode(&inputs, Quat::IDENTITY, &mut bus);
        // After the crossfade settles (last few samples), output ≈ input
        // for the 4 input channels and ≈ 0 for the higher-order channels.
        let i = BLOCK_SIZE - 1;
        assert!((bus[0][i] - 1.0).abs()   < 1e-3);
        assert!((bus[1][i] - 0.5).abs()   < 1e-3);
        assert!((bus[2][i] - (-0.25)).abs()< 1e-3);
        assert!((bus[3][i] - 0.75).abs()  < 1e-3);
        #[allow(clippy::needless_range_loop)]
        for k in 4..NUM_AMBI {
            assert!(bus[k][i].abs() < 1e-3, "non-input channel {k} = {}", bus[k][i]);
        }
    }

    #[test]
    fn ambisonic_world_locked_rotates_under_listener() {
        // 1st-order input: pure Y (m=-1, ACN[1]). With listener
        // rotated +90° about Z, the listener-frame encoding moves
        // a world-Y source into listener-+X (i.e. directly in front
        // of the now-left-facing listener). So bus[1] (Y) ≈ 0 and
        // bus[3] (X) ≈ +1.
        let mut bed = AudioBed::new(BedFormat::Ambisonics1st);
        let mut inputs: Vec<[f32; BLOCK_SIZE]> = vec![[0.0; BLOCK_SIZE]; 4];
        #[allow(clippy::needless_range_loop)]
        for i in 0..BLOCK_SIZE {
            inputs[1][i] = 1.0; // Y channel only
        }
        let s = core::f32::consts::FRAC_PI_4.sin();
        let c = core::f32::consts::FRAC_PI_4.cos();
        let q = Quat::new(c, 0.0, 0.0, s);

        let mut bus = ambi_zero();
        // Run twice to settle the prev/target crossfade.
        bed.encode(&inputs, q, &mut bus);
        let mut bus = ambi_zero();
        bed.encode(&inputs, q, &mut bus);

        let i = BLOCK_SIZE - 1;
        // After settling: input Y should map to encoding-frame -X.
        assert!(bus[1][i].abs() < 0.05, "Y leaks through: {}", bus[1][i]);
        assert!(bus[3][i] > 0.5, "X expected positive-large: {}", bus[3][i]);
    }

    #[test]
    fn world_locked_rotates_with_listener() {
        // Stereo bed, listener rotated +90° about Z: L (+30° world)
        // becomes (-60° relative). With anti-phase L/R, the +Y
        // component of the encoded bus should flip sign vs the
        // unrotated case.
        let mut bed = AudioBed::new(BedFormat::Stereo);
        let mut inputs: Vec<[f32; BLOCK_SIZE]> = vec![[0.0; BLOCK_SIZE]; 2];
        #[allow(clippy::needless_range_loop)]
        for i in 0..BLOCK_SIZE {
            inputs[0][i] =  1.0;
            inputs[1][i] = -1.0;
        }

        let mut bus_a = ambi_zero();
        bed.encode(&inputs, Quat::IDENTITY, &mut bus_a);

        // Listener rotated +90° about Z.
        let s = (std::f32::consts::FRAC_PI_4).sin();
        let c = (std::f32::consts::FRAC_PI_4).cos();
        let q = Quat::new(c, 0.0, 0.0, s);
        let mut bus_b = ambi_zero();
        bed.encode(&inputs, q, &mut bus_b);

        let y_a = bus_a[1][BLOCK_SIZE - 1];
        let y_b = bus_b[1][BLOCK_SIZE - 1];
        // Different listener orientations must produce different
        // Y-channel content (not strictly opposite sign, depending on
        // which speaker has which sign, but definitely different).
        assert!(
            (y_a - y_b).abs() > 0.1 * y_a.abs().max(y_b.abs()),
            "rotation should change bed encoding: y_a={y_a}, y_b={y_b}"
        );
    }
}
