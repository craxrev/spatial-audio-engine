//! §13 W-channel binauralizer (`decoder_post`). Reads the W (omni)
//! channel of the ambisonic bus, convolves it with two ear filters
//! (`hrtf_post_filter_a.bin` → left, `hrtf_post_filter_b.bin` → right),
//! and **adds** the result to the stereo output as a diffuse-field
//! envelopment layer.
//!
//! Research-agent verified (v0.5 meet wasm `func[64]`,
//! `decoder_prepare`): the bundled blobs are **flat time-domain IRs**
//! (2865 floats per ear, ~60 ms at 48 kHz). The spec's "NonUniform
//! partition layout" is built at runtime from these flat IRs in the
//! reference; for our impl we feed them directly into a
//! `TimeDomainConvEngine`. See development notes for the deviation note.

use crate::consts::{BLOCK_SIZE, OUTPUT_CHANNELS};
use crate::conv::{ConvolutionEngine, TimeDomainConvEngine};

pub const W_BINAURALIZER_TAPS: usize = 2865;

pub struct WBinauralizer {
    conv: TimeDomainConvEngine,
}

impl WBinauralizer {
    /// Build a W-binauralizer from two raw byte blobs.
    /// Each blob is exactly `W_BINAURALIZER_TAPS * 4` bytes
    /// (little-endian f32) — directly from `hrtf_post_filter_a.bin` /
    /// `hrtf_post_filter_b.bin`. Returns `None` on size mismatch.
    pub fn from_bytes(filter_a: &[u8], filter_b: &[u8]) -> Option<Self> {
        let expected = W_BINAURALIZER_TAPS * 4;
        if filter_a.len() != expected || filter_b.len() != expected {
            return None;
        }
        let ir_a = decode_le_f32(filter_a);
        let ir_b = decode_le_f32(filter_b);
        let mut conv = TimeDomainConvEngine::new(1, OUTPUT_CHANNELS, W_BINAURALIZER_TAPS);
        conv.set_ir(0, 0, &ir_a);
        conv.set_ir(0, 1, &ir_b);
        Some(Self { conv })
    }

    /// Process one block: mono W-channel input → stereo additive output.
    pub fn process_add(
        &mut self,
        w_in: &[f32; BLOCK_SIZE],
        stereo_out: &mut [[f32; BLOCK_SIZE]; OUTPUT_CHANNELS],
    ) {
        let inputs = [*w_in];
        let mut outs = [[0.0_f32; BLOCK_SIZE]; OUTPUT_CHANNELS];
        self.conv.process(&inputs, &mut outs[..]);
        for ear in 0..OUTPUT_CHANNELS {
            for i in 0..BLOCK_SIZE {
                stereo_out[ear][i] += outs[ear][i];
            }
        }
    }

    pub fn reset(&mut self) {
        self.conv.reset();
    }
}

fn decode_le_f32(bytes: &[u8]) -> Vec<f32> {
    bytes
        .chunks_exact(4)
        .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fake_filter_blob(impulse_at: usize) -> Vec<u8> {
        let mut ir = vec![0.0_f32; W_BINAURALIZER_TAPS];
        ir[impulse_at] = 1.0;
        ir.iter().flat_map(|f| f.to_le_bytes()).collect()
    }

    #[test]
    fn rejects_wrong_size_blobs() {
        assert!(WBinauralizer::from_bytes(&[0; 10], &[0; 10]).is_none());
        assert!(
            WBinauralizer::from_bytes(&[0; W_BINAURALIZER_TAPS * 4], &[0; 10]).is_none()
        );
    }

    #[test]
    fn delta_impulses_pass_w_through() {
        // Filter A = δ[0], Filter B = δ[0] → output is just W
        // for both ears (added to stereo_out).
        let a = fake_filter_blob(0);
        let b = fake_filter_blob(0);
        let mut wb = WBinauralizer::from_bytes(&a, &b).unwrap();
        let mut w = [0.0_f32; BLOCK_SIZE];
        w[10] = 0.5;
        let mut out = [[0.0_f32; BLOCK_SIZE]; OUTPUT_CHANNELS];
        wb.process_add(&w, &mut out);
        assert!((out[0][10] - 0.5).abs() < 1e-6);
        assert!((out[1][10] - 0.5).abs() < 1e-6);
    }

    #[test]
    fn additive_mix() {
        // The output is ADDED to stereo_out, not overwritten.
        let a = fake_filter_blob(0);
        let b = fake_filter_blob(0);
        let mut wb = WBinauralizer::from_bytes(&a, &b).unwrap();
        let mut w = [0.0_f32; BLOCK_SIZE];
        w[3] = 0.25;
        let mut out = [[1.0_f32; BLOCK_SIZE]; OUTPUT_CHANNELS]; // pre-filled
        wb.process_add(&w, &mut out);
        assert!((out[0][3] - 1.25).abs() < 1e-6);
        assert!((out[1][3] - 1.25).abs() < 1e-6);
        // Other samples untouched (still 1.0).
        assert!((out[0][0] - 1.0).abs() < 1e-6);
        assert!((out[1][5] - 1.0).abs() < 1e-6);
    }
}
