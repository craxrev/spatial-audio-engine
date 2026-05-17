//! §17 v0.4 legacy post-decoder. A 2-in × 2-out cross-channel
//! coloration filter applied to the binaural intermediate as a
//! *replacing* transform (unlike v0.5's W-binauralizer which adds).
//!
//! Bundled filter: `data/hrtf_post_legacy_v04.bin` — 8 192 bytes =
//! 4 cells × 512-tap time-domain f32 IRs. Cells ordered:
//! ```text
//! cell_index = in_ear · 2 + out_ear      (in/out ∈ {0=L, 1=R})
//! ```
//!
//! Spec discrepancy: §17.2 describes the file as "16 frequency-domain
//! partitions in halfcomplex packing" but inspection shows it is in
//! fact a flat time-domain IR set — same pattern as the §8.1 / §13
//! findings for v0.5. We load it directly into a `TimeDomainConvEngine`.

use crate::consts::{BLOCK_SIZE, OUTPUT_CHANNELS};
use crate::conv::{ConvolutionEngine, TimeDomainConvEngine};

pub const LEGACY_TAPS_PER_CELL: usize = 512;
pub const LEGACY_FILE_BYTES: usize = LEGACY_TAPS_PER_CELL * 4 * 4; // 4 cells × 512 × 4

pub struct LegacyPostDecoder {
    conv: TimeDomainConvEngine,
}

impl LegacyPostDecoder {
    pub fn from_bytes(filter: &[u8]) -> Option<Self> {
        if filter.len() != LEGACY_FILE_BYTES {
            return None;
        }
        // Decode into 2048 f32, then split into 4 cells of 512.
        let mut floats = Vec::with_capacity(LEGACY_TAPS_PER_CELL * 4);
        for c in filter.chunks_exact(4) {
            floats.push(f32::from_le_bytes([c[0], c[1], c[2], c[3]]));
        }
        let mut conv = TimeDomainConvEngine::new(
            OUTPUT_CHANNELS, // 2 inputs (L, R)
            OUTPUT_CHANNELS, // 2 outputs (L, R)
            LEGACY_TAPS_PER_CELL,
        );
        for in_ear in 0..OUTPUT_CHANNELS {
            for out_ear in 0..OUTPUT_CHANNELS {
                let cell_index = in_ear * OUTPUT_CHANNELS + out_ear;
                let start = cell_index * LEGACY_TAPS_PER_CELL;
                conv.set_ir(in_ear, out_ear, &floats[start..start + LEGACY_TAPS_PER_CELL]);
            }
        }
        Some(Self { conv })
    }

    /// Process the stereo binaural intermediate in place. The conv
    /// engine overwrites `stereo` (replacing transform per §17).
    pub fn process_replace(
        &mut self,
        stereo: &mut [[f32; BLOCK_SIZE]; OUTPUT_CHANNELS],
    ) {
        let inputs = *stereo;
        self.conv.process(&inputs, &mut stereo[..]);
    }

    pub fn reset(&mut self) {
        self.conv.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn impulse_blob() -> Vec<u8> {
        // Cells laid out as 4 × 512 floats. Identity: cell 0 (L→L) δ[0]=1,
        // cell 3 (R→R) δ[0]=1, cells 1 and 2 (cross) = 0.
        let mut floats = vec![0.0_f32; 4 * LEGACY_TAPS_PER_CELL];
        floats[0] = 1.0; // L→L δ[0] = 1
        floats[3 * LEGACY_TAPS_PER_CELL] = 1.0; // R→R
        floats.iter().flat_map(|f| f.to_le_bytes()).collect()
    }

    #[test]
    fn rejects_wrong_size() {
        assert!(LegacyPostDecoder::from_bytes(&[0; 100]).is_none());
        assert!(LegacyPostDecoder::from_bytes(&[0; LEGACY_FILE_BYTES - 1]).is_none());
    }

    #[test]
    fn identity_filter_passes_stereo_through() {
        let mut d = LegacyPostDecoder::from_bytes(&impulse_blob()).unwrap();
        let mut stereo = [[0.0_f32; BLOCK_SIZE]; OUTPUT_CHANNELS];
        stereo[0][10] = 0.5;
        stereo[1][20] = -0.25;
        d.process_replace(&mut stereo);
        // δ[0] · 1.0 means output equals input (no delay, no scale).
        assert!((stereo[0][10] - 0.5).abs() < 1e-6);
        assert!((stereo[1][20] + 0.25).abs() < 1e-6);
        // No cross-feed: L input shouldn't appear in R output and vice versa.
        assert!(stereo[1][10].abs() < 1e-6);
        assert!(stereo[0][20].abs() < 1e-6);
    }

    #[test]
    fn cross_feed_filter_mixes_channels() {
        // Build a filter with cross-feed: L→R = δ[0] · 0.3, R→L = δ[0] · 0.3,
        // identity on diagonal. R output should pick up 30% of L input.
        let mut floats = vec![0.0_f32; 4 * LEGACY_TAPS_PER_CELL];
        floats[0] = 1.0;                       // L→L δ[0] = 1
        floats[LEGACY_TAPS_PER_CELL] = 0.3;     // L→R
        floats[2 * LEGACY_TAPS_PER_CELL] = 0.3; // R→L
        floats[3 * LEGACY_TAPS_PER_CELL] = 1.0; // R→R
        let blob: Vec<u8> = floats.iter().flat_map(|f| f.to_le_bytes()).collect();
        let mut d = LegacyPostDecoder::from_bytes(&blob).unwrap();

        let mut stereo = [[0.0_f32; BLOCK_SIZE]; OUTPUT_CHANNELS];
        stereo[0][5] = 1.0;
        d.process_replace(&mut stereo);
        assert!((stereo[0][5] - 1.0).abs() < 1e-6);   // L pass-through
        assert!((stereo[1][5] - 0.3).abs() < 1e-6);   // L → R at 0.3
    }
}
