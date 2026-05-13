//! Bundled HRTF coefficient loader (§13). Reads the 16,384-byte
//! `hrtf_decoder_native.bin` (32 filters × 128 floats halfcomplex)
//! and recovers each cell's 128-tap time-domain IR via inverse FFT.
//!
//! Cell layout (§13): `i = ambi_channel · OUTPUT_CHANNELS + ear`,
//! `ear ∈ {0=left, 1=right}` (per §13 note, ear ordering must be
//! confirmed empirically — see the integration tests).
//!
//! Halfcomplex packing (§8.1): 1 real DC, 1 real Nyquist, then 63
//! complex bins as `(re, im)` pairs → 128 floats per cell.

use realfft::RealFftPlanner;
use realfft::num_complex::Complex;

use crate::consts::{NUM_AMBI, OUTPUT_CHANNELS};

const FFT_SIZE: usize = 128;
const FILTER_FLOATS: usize = 128;
const FILTER_BYTES: usize = FILTER_FLOATS * 4;
const N_CELLS: usize = NUM_AMBI * OUTPUT_CHANNELS;
const TOTAL_BYTES: usize = N_CELLS * FILTER_BYTES;
const N_BINS: usize = FFT_SIZE / 2 + 1; // 65 unique bins

#[derive(Debug)]
pub enum HrtfLoadError {
    WrongLength { got: usize, want: usize },
}

#[derive(Clone, Debug)]
pub struct Hrtf {
    /// 32 time-domain IRs of `FFT_SIZE` taps each. Indexed
    /// `ambi · OUTPUT_CHANNELS + ear`.
    pub irs: Vec<[f32; FFT_SIZE]>,
}

impl Hrtf {
    pub fn load_from_bytes(bytes: &[u8]) -> Result<Self, HrtfLoadError> {
        if bytes.len() != TOTAL_BYTES {
            return Err(HrtfLoadError::WrongLength {
                got: bytes.len(),
                want: TOTAL_BYTES,
            });
        }
        let mut planner = RealFftPlanner::<f32>::new();
        let c2r = planner.plan_fft_inverse(FFT_SIZE);
        let mut irs = Vec::with_capacity(N_CELLS);
        for cell in 0..N_CELLS {
            let start = cell * FILTER_BYTES;
            let floats = read_le_f32_block(&bytes[start..start + FILTER_BYTES]);
            let mut spectrum = halfcomplex_to_bins(&floats);
            let mut time = vec![0.0_f32; FFT_SIZE];
            // realfft's c2r is unnormalized; per §10.4 the bundled
            // file already has 1/N folded into the spectrum, so no
            // additional scaling is needed here.
            c2r.process(&mut spectrum, &mut time)
                .expect("c2r length");
            let mut arr = [0.0_f32; FFT_SIZE];
            arr.copy_from_slice(&time);
            irs.push(arr);
        }
        Ok(Self { irs })
    }

    pub fn ir(&self, ambi: usize, ear: usize) -> &[f32; FFT_SIZE] {
        &self.irs[ambi * OUTPUT_CHANNELS + ear]
    }
}

fn read_le_f32_block(bytes: &[u8]) -> [f32; FILTER_FLOATS] {
    let mut out = [0.0_f32; FILTER_FLOATS];
    for (i, chunk) in bytes.chunks_exact(4).enumerate() {
        out[i] = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
    }
    out
}

fn halfcomplex_to_bins(floats: &[f32; FILTER_FLOATS]) -> Vec<Complex<f32>> {
    let mut bins = vec![Complex::new(0.0, 0.0); N_BINS];
    bins[0] = Complex::new(floats[0], 0.0); // DC
    bins[FFT_SIZE / 2] = Complex::new(floats[1], 0.0); // Nyquist (bin 64)
    for bin in 1..(FFT_SIZE / 2) {
        let re = floats[2 + (bin - 1) * 2];
        let im = floats[2 + (bin - 1) * 2 + 1];
        bins[bin] = Complex::new(re, im);
    }
    bins
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
    }

    #[test]
    fn irs_have_plausible_magnitudes() {
        // HRTF impulse responses should have absolute values well below
        // 1.0 — typically a few tenths at peak. This sanity-checks our
        // halfcomplex parse + IFFT normalization.
        let h = Hrtf::load_from_bytes(BUNDLED).unwrap();
        for (i, ir) in h.irs.iter().enumerate() {
            let peak = ir.iter().fold(0.0_f32, |m, &v| m.max(v.abs()));
            assert!(peak.is_finite(), "cell {i} non-finite");
            assert!(peak < 10.0, "cell {i} peak {peak} too large");
            assert!(peak > 0.0, "cell {i} silent");
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
