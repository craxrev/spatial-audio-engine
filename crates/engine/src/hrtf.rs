//! Bundled HRTF coefficient loader. Reads the 16,384-byte
//! `hrtf_decoder_native.bin` per §13: 32 cells of 128 time-domain
//! float32 taps, ear-major layout: slots 0..15 = left-ear filters,
//! slots 16..31 = right-ear filters. Each cell is multiplied by
//! `HRTF_LOAD_GAIN` (1.585) on load, matching the reference's
//! per-cell calibration scalar.
//!
//! When the engine's runtime rate differs from the authored rate
//! (`HRTF_SOURCE_RATE` = 48 kHz), the IRs are resampled at load time
//! via the §11 Kaiser-windowed sinc resampler.

use crate::consts::{HRTF_LOAD_GAIN, NUM_AMBI, OUTPUT_CHANNELS};
use crate::resampler::resample;

/// Authored sample rate of the bundled HRTF data.
pub const HRTF_SOURCE_RATE: u32 = 48_000;
/// Number of taps per cell in the bundled file (before any resampling).
pub const HRTF_SOURCE_TAPS: usize = 128;
const FILTER_BYTES: usize = HRTF_SOURCE_TAPS * 4;
const N_CELLS: usize = NUM_AMBI * OUTPUT_CHANNELS;
const TOTAL_BYTES: usize = N_CELLS * FILTER_BYTES;

#[derive(Debug)]
pub enum HrtfLoadError {
    WrongLength { got: usize, want: usize },
}

#[derive(Clone, Debug)]
pub struct Hrtf {
    /// 32 time-domain IRs, all the same length (`ir_len`), stored in
    /// file order (left-ear block first, then right-ear block). Look
    /// them up via `ir(ambi, ear)` with the L=0/R=1 convention.
    irs: Vec<Vec<f32>>,
    ir_len: usize,
}

impl Hrtf {
    /// Backward-compatible loader: assumes the runtime rate equals
    /// `HRTF_SOURCE_RATE` (no resampling). Kept for old call sites.
    pub fn load_from_bytes(bytes: &[u8]) -> Result<Self, HrtfLoadError> {
        Self::load_from_bytes_at(bytes, HRTF_SOURCE_RATE)
    }

    /// Load + (optionally) resample to `target_rate`.
    pub fn load_from_bytes_at(
        bytes: &[u8],
        target_rate: u32,
    ) -> Result<Self, HrtfLoadError> {
        if bytes.len() != TOTAL_BYTES {
            return Err(HrtfLoadError::WrongLength {
                got: bytes.len(),
                want: TOTAL_BYTES,
            });
        }
        let mut irs: Vec<Vec<f32>> = Vec::with_capacity(N_CELLS);
        for cell in 0..N_CELLS {
            let start = cell * FILTER_BYTES;
            let mut raw = Vec::with_capacity(HRTF_SOURCE_TAPS);
            for chunk in bytes[start..start + FILTER_BYTES].chunks_exact(4) {
                let v = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                raw.push(v * HRTF_LOAD_GAIN);
            }
            let cell = if target_rate == HRTF_SOURCE_RATE {
                raw
            } else {
                resample(&raw, HRTF_SOURCE_RATE, target_rate)
            };
            irs.push(cell);
        }
        let ir_len = irs[0].len();
        Ok(Self { irs, ir_len })
    }

    pub fn ir_len(&self) -> usize {
        self.ir_len
    }

    /// Look up the IR for `(ambi, ear)` with caller-facing
    /// `ear ∈ {0=left, 1=right}`. The file layout is ear-major:
    /// slots 0..15 are the left-ear filters, 16..31 the right-ear
    /// filters.
    pub fn ir(&self, ambi: usize, ear: usize) -> &[f32] {
        &self.irs[ear * NUM_AMBI + ambi]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BUNDLED: &[u8] =
        include_bytes!("../../../data/hrtf_decoder_native.bin");

    #[test]
    fn bundled_file_is_expected_size() {
        assert_eq!(BUNDLED.len(), TOTAL_BYTES);
    }

    #[test]
    fn loads_32_cells() {
        let h = Hrtf::load_from_bytes(BUNDLED).unwrap();
        assert_eq!(h.irs.len(), N_CELLS);
        assert_eq!(h.ir_len(), HRTF_SOURCE_TAPS);
    }

    #[test]
    fn irs_have_plausible_magnitudes() {
        let h = Hrtf::load_from_bytes(BUNDLED).unwrap();
        for (i, ir) in h.irs.iter().enumerate() {
            let peak = ir.iter().fold(0.0_f32, |m, &v| m.max(v.abs()));
            assert!(peak.is_finite(), "cell {i} non-finite");
            assert!(peak < 10.0, "cell {i} peak {peak} too large");
            assert!(peak > 0.0, "cell {i} silent");
        }
    }

    #[test]
    fn w_filter_is_near_symmetric() {
        let h = Hrtf::load_from_bytes(BUNDLED).unwrap();
        let l_e: f32 = h.ir(0, 0).iter().map(|v| v * v).sum();
        let r_e: f32 = h.ir(0, 1).iter().map(|v| v * v).sum();
        let ratio = (l_e / r_e).max(r_e / l_e);
        assert!(
            ratio < 1.5,
            "W filter L vs R energy: L={l_e}, R={r_e}, ratio={ratio}"
        );
    }

    #[test]
    fn resamples_to_44_1k() {
        let h = Hrtf::load_from_bytes_at(BUNDLED, 44_100).unwrap();
        // 128 * 44100/48000 = 117.6 → 118
        assert_eq!(h.ir_len(), 118);
        // Energy should still be non-zero in every cell.
        for ir in &h.irs {
            let e: f32 = ir.iter().map(|v| v * v).sum();
            assert!(e > 0.0);
        }
    }

    #[test]
    fn wrong_length_rejected() {
        let short = vec![0_u8; 100];
        assert!(matches!(
            Hrtf::load_from_bytes(&short),
            Err(HrtfLoadError::WrongLength { .. })
        ));
    }
}
