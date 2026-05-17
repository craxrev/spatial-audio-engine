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
        }
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
}

impl AudioBed {
    pub fn new(format: BedFormat) -> Self {
        Self {
            format,
            speakers: speakers_for(format),
            headlocked: false,
            gain: 1.0,
        }
    }

    pub fn format(&self) -> BedFormat {
        self.format
    }

    pub fn channel_count(&self) -> usize {
        self.speakers.len()
    }

    /// Encode bed channels into the ambisonic bus. `inputs.len()`
    /// must match the configured channel count or the call is a no-op
    /// (matches spec §12 step 6 `format_match` guard).
    pub fn encode(
        &self,
        inputs: &[[f32; BLOCK_SIZE]],
        listener_quat: Quat,
        ambi_bus: &mut [[f32; BLOCK_SIZE]],
    ) {
        if inputs.len() != self.speakers.len() || self.gain == 0.0 {
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
        let bed = AudioBed::new(BedFormat::Mono);
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
        let bed = AudioBed::new(BedFormat::Stereo);
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
        let bed = AudioBed::new(BedFormat::Surround5_1);
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
        let bed = AudioBed::new(BedFormat::Surround5_1);
        let mut bus = ambi_zero();
        bed.encode(&dc_channels(2, 1.0), Quat::IDENTITY, &mut bus); // wrong count
        for ch in &bus {
            assert!(ch.iter().all(|v| *v == 0.0));
        }
    }

    #[test]
    fn world_locked_rotates_with_listener() {
        // Stereo bed, listener rotated +90° about Z: L (+30° world)
        // becomes (-60° relative). With anti-phase L/R, the +Y
        // component of the encoded bus should flip sign vs the
        // unrotated case.
        let bed = AudioBed::new(BedFormat::Stereo);
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
