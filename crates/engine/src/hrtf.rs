//! Bundled HRTF coefficient loader. Reads the 16,384-byte
//! `hrtf_decoder_native.bin` per §13: 32 cells of 128 time-domain
//! float32 taps, ear-major layout with `ear ∈ {0=right, 1=left}`
//! on disk. Each cell is multiplied by `HRTF_LOAD_GAIN` (1.585) on
//! load, matching the reference's per-cell calibration scalar.

use crate::consts::{HRTF_LOAD_GAIN, NUM_AMBI, OUTPUT_CHANNELS};

const IR_LEN: usize = 128;
const FILTER_BYTES: usize = IR_LEN * 4;
const N_CELLS: usize = NUM_AMBI * OUTPUT_CHANNELS;
const TOTAL_BYTES: usize = N_CELLS * FILTER_BYTES;

#[derive(Debug)]
pub enum HrtfLoadError {
    WrongLength { got: usize, want: usize },
}

#[derive(Clone, Debug)]
pub struct Hrtf {
    /// 32 time-domain IRs of `IR_LEN` taps each, stored in file
    /// order (right-ear block first, then left-ear block). Use
    /// `ir()` to look up by `(ambi, ear)` in the L=0/R=1 convention.
    pub irs: Vec<[f32; IR_LEN]>,
}

impl Hrtf {
    pub fn load_from_bytes(bytes: &[u8]) -> Result<Self, HrtfLoadError> {
        if bytes.len() != TOTAL_BYTES {
            return Err(HrtfLoadError::WrongLength {
                got: bytes.len(),
                want: TOTAL_BYTES,
            });
        }
        let mut irs = Vec::with_capacity(N_CELLS);
        for cell in 0..N_CELLS {
            let start = cell * FILTER_BYTES;
            let mut arr = [0.0_f32; IR_LEN];
            for (i, chunk) in bytes[start..start + FILTER_BYTES]
                .chunks_exact(4)
                .enumerate()
            {
                let v = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                arr[i] = v * HRTF_LOAD_GAIN;
            }
            irs.push(arr);
        }
        Ok(Self { irs })
    }

    /// Look up the IR for `(ambi, ear)` with caller-facing
    /// `ear ∈ {0=left, 1=right}`. The file's right-ear block is
    /// first, so the indices are flipped here.
    pub fn ir(&self, ambi: usize, ear: usize) -> &[f32; IR_LEN] {
        let file_ear = 1 - ear;
        &self.irs[file_ear * NUM_AMBI + ambi]
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
        // W (ACN[0]) is omni; left and right ear filters should be
        // very similar in energy and peak position.
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
    fn wrong_length_rejected() {
        let short = vec![0_u8; 100];
        assert!(matches!(
            Hrtf::load_from_bytes(&short),
            Err(HrtfLoadError::WrongLength { .. })
        ));
    }
}
