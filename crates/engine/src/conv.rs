//! Convolution engines (§10). The trait abstracts an N-input × M-output
//! per-block convolution. For M4 only a direct-form time-domain FIR
//! implementation exists; an FFT-partitioned implementation (Gardner
//! non-uniform) lands at M11 behind the same trait.

use crate::consts::BLOCK_SIZE;

pub trait ConvolutionEngine {
    fn num_inputs(&self) -> usize;
    fn num_outputs(&self) -> usize;

    /// Read one block from each input, write one block to each output.
    /// Outputs are overwritten (not accumulated).
    fn process(
        &mut self,
        inputs: &[[f32; BLOCK_SIZE]],
        outputs: &mut [[f32; BLOCK_SIZE]],
    );

    /// Clear all internal state (delay lines, accumulators).
    fn reset(&mut self);
}

/// Direct-form FIR per cross-cell. Supports arbitrary IR length —
/// for `ir_len ≤ BLOCK_SIZE+1` the history fits inside one block; for
/// longer IRs (e.g. the M11 W-binauralizer's 2865-tap filters) the
/// scratch buffer grows accordingly.
///
/// IRs are stored **reversed** so the per-sample dot product is a
/// forward `zip` over two contiguous slices — the compiler then
/// elides bounds checks and emits a tight SIMD multiply-accumulate
/// inner loop. Without this the inner-loop bounds checks dominate
/// CPU (~33% of plugin time, confirmed by Instruments).
pub struct TimeDomainConvEngine {
    num_inputs: usize,
    num_outputs: usize,
    ir_len: usize,
    history_len: usize,
    /// Per cross-cell IR (REVERSED), indexed `in_idx * num_outputs + out_idx`.
    irs: Vec<Vec<f32>>,
    /// Per-input scratch: `[history(history_len), curr(BLOCK_SIZE)]`,
    /// total `history_len + BLOCK_SIZE` floats. After each block we
    /// shift left by `BLOCK_SIZE` so the tail becomes the new history.
    scratch: Vec<Vec<f32>>,
}

impl TimeDomainConvEngine {
    pub fn new(num_inputs: usize, num_outputs: usize, ir_len: usize) -> Self {
        let history_len = ir_len.saturating_sub(1);
        let scratch_len = history_len + BLOCK_SIZE;
        Self {
            num_inputs,
            num_outputs,
            ir_len,
            history_len,
            irs: vec![vec![0.0; ir_len]; num_inputs * num_outputs],
            scratch: vec![vec![0.0; scratch_len]; num_inputs],
        }
    }

    pub fn set_ir(&mut self, in_idx: usize, out_idx: usize, ir: &[f32]) {
        let idx = in_idx * self.num_outputs + out_idx;
        let cell = &mut self.irs[idx];
        for v in cell.iter_mut() {
            *v = 0.0;
        }
        // Store reversed and tail-aligned: source `ir[0..n]` lands in
        // `cell[ir_len - n .. ir_len]` in reverse order, leaving any
        // short-IR zero-padding at the *front* of the cell. This is
        // what makes the inner dot product a forward `zip`.
        let n = ir.len().min(self.ir_len);
        let dst = &mut cell[self.ir_len - n..];
        for (i, v) in ir.iter().take(n).enumerate() {
            dst[n - 1 - i] = *v;
        }
    }
}

impl ConvolutionEngine for TimeDomainConvEngine {
    fn num_inputs(&self) -> usize {
        self.num_inputs
    }

    fn num_outputs(&self) -> usize {
        self.num_outputs
    }

    fn process(
        &mut self,
        inputs: &[[f32; BLOCK_SIZE]],
        outputs: &mut [[f32; BLOCK_SIZE]],
    ) {
        assert_eq!(inputs.len(), self.num_inputs);
        assert_eq!(outputs.len(), self.num_outputs);

        // Stage current inputs into the tail of each scratch buffer.
        // Head of scratch (`..history_len`) already holds the last
        // `history_len` samples from prior blocks.
        for (s, curr) in self.scratch.iter_mut().zip(inputs.iter()) {
            s[self.history_len..].copy_from_slice(curr);
        }

        for out in outputs.iter_mut() {
            out.fill(0.0);
        }

        // Per output sample `n`, accumulate `sum_k(scratch[n + k - 0] *
        // ir_reversed[k])` over all (in_idx, k). The IR is stored
        // reversed (see `set_ir`), so the dot product is a forward
        // `zip` over two contiguous slices — compiler-vectorisable.
        //
        // Loop order: (out_idx, in_idx, n, k). Hoisting the per-cell
        // slice borrows out of the BLOCK_SIZE loop lets the compiler
        // keep them in registers for the inner kernel.
        let ir_len = self.ir_len;
        let history_len = self.history_len;
        for (out_idx, out) in outputs.iter_mut().enumerate() {
            for in_idx in 0..self.num_inputs {
                let ir_rev = self.irs[in_idx * self.num_outputs + out_idx].as_slice();
                let scratch = self.scratch[in_idx].as_slice();
                for (n, out_n) in out.iter_mut().enumerate() {
                    // Window covers scratch[(history_len + n + 1 - ir_len)..=history_len + n].
                    // Length is exactly `ir_len` by construction.
                    let start = history_len + n + 1 - ir_len;
                    let window = &scratch[start..start + ir_len];
                    let acc: f32 = window
                        .iter()
                        .zip(ir_rev.iter())
                        .map(|(a, b)| a * b)
                        .sum();
                    *out_n += acc;
                }
            }
        }

        // Shift scratch left by BLOCK_SIZE: the new history is the
        // last `history_len` samples of `[old_history + curr]`.
        for s in self.scratch.iter_mut() {
            let total = s.len();
            s.copy_within(BLOCK_SIZE..total, 0);
        }
    }

    fn reset(&mut self) {
        for s in &mut self.scratch {
            for v in s.iter_mut() {
                *v = 0.0;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn impulse_passthrough() {
        // 1-input, 1-output engine with δ[n] as the IR. Output equals input.
        let mut e = TimeDomainConvEngine::new(1, 1, 4);
        let ir = [1.0_f32, 0.0, 0.0, 0.0];
        e.set_ir(0, 0, &ir);

        let mut input = [[0.0_f32; BLOCK_SIZE]; 1];
        for (i, v) in input[0].iter_mut().enumerate() {
            *v = (i as f32) * 0.01;
        }
        let mut out = [[0.0_f32; BLOCK_SIZE]; 1];
        e.process(&input, &mut out);

        for i in 0..BLOCK_SIZE {
            assert!((out[0][i] - input[0][i]).abs() < 1e-6);
        }
    }

    #[test]
    fn delay_ir_delays_signal() {
        // IR = [0, 0, 1, 0] → 2-sample delay.
        let mut e = TimeDomainConvEngine::new(1, 1, 4);
        let ir = [0.0_f32, 0.0, 1.0, 0.0];
        e.set_ir(0, 0, &ir);

        let mut input = [[0.0_f32; BLOCK_SIZE]; 1];
        input[0][0] = 1.0;
        let mut out = [[0.0_f32; BLOCK_SIZE]; 1];
        e.process(&input, &mut out);

        assert_eq!(out[0][0], 0.0);
        assert_eq!(out[0][1], 0.0);
        assert_eq!(out[0][2], 1.0);
        assert_eq!(out[0][3], 0.0);
    }

    #[test]
    fn history_carries_across_blocks() {
        // Put an impulse at sample 126 of block 1, with a 4-tap IR.
        // Expect non-zero outputs in block 2 at samples 0 and 1
        // (tail of the convolution).
        let mut e = TimeDomainConvEngine::new(1, 1, 4);
        let ir = [1.0_f32, 0.5, 0.25, 0.125];
        e.set_ir(0, 0, &ir);

        let mut block1 = [[0.0_f32; BLOCK_SIZE]; 1];
        block1[0][BLOCK_SIZE - 2] = 1.0;
        let mut out1 = [[0.0_f32; BLOCK_SIZE]; 1];
        e.process(&block1, &mut out1);

        // Block 1 outputs at sample 126: h[0]*1 = 1.0; sample 127: h[1]*1 = 0.5.
        assert!((out1[0][BLOCK_SIZE - 2] - 1.0).abs() < 1e-6);
        assert!((out1[0][BLOCK_SIZE - 1] - 0.5).abs() < 1e-6);

        // Block 2 (zeros). The impulse's tail should appear at samples 0 and 1.
        let block2 = [[0.0_f32; BLOCK_SIZE]; 1];
        let mut out2 = [[0.0_f32; BLOCK_SIZE]; 1];
        e.process(&block2, &mut out2);
        assert!((out2[0][0] - 0.25).abs() < 1e-6);
        assert!((out2[0][1] - 0.125).abs() < 1e-6);
        assert!(out2[0][2].abs() < 1e-6);
    }

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn multi_io_routes_correctly() {
        // 2-in × 2-out engine, identity diagonal, zero off-diagonal.
        // out[0] should equal in[0], out[1] should equal in[1].
        let mut e = TimeDomainConvEngine::new(2, 2, 1);
        e.set_ir(0, 0, &[1.0]);
        e.set_ir(1, 1, &[1.0]);

        let mut input = [[0.0_f32; BLOCK_SIZE]; 2];
        for i in 0..BLOCK_SIZE {
            input[0][i] = (i as f32) * 0.1;
            input[1][i] = -(i as f32) * 0.1;
        }
        let mut out = [[0.0_f32; BLOCK_SIZE]; 2];
        e.process(&input, &mut out);

        for i in 0..BLOCK_SIZE {
            assert!((out[0][i] - input[0][i]).abs() < 1e-6);
            assert!((out[1][i] - input[1][i]).abs() < 1e-6);
        }
    }

    #[test]
    fn long_ir_history_spans_multiple_blocks() {
        // IR of length 300 (> BLOCK_SIZE = 128) with a non-zero tap
        // at index 200. An impulse at sample 0 of block 1 should
        // produce non-zero output at sample (200 - 128) = 72 of
        // block 2 — i.e. the history must reach across two blocks.
        let ir_len = 300;
        let mut e = TimeDomainConvEngine::new(1, 1, ir_len);
        let mut ir = vec![0.0_f32; ir_len];
        ir[200] = 1.0;
        e.set_ir(0, 0, &ir);

        let mut block1 = [[0.0_f32; BLOCK_SIZE]; 1];
        block1[0][0] = 1.0;
        let mut out1 = [[0.0_f32; BLOCK_SIZE]; 1];
        e.process(&block1, &mut out1);
        // Within block 1 the impulse only reaches up to tap 127.
        assert!(out1[0].iter().all(|v| v.abs() < 1e-6));

        let block2 = [[0.0_f32; BLOCK_SIZE]; 1];
        let mut out2 = [[0.0_f32; BLOCK_SIZE]; 1];
        e.process(&block2, &mut out2);
        // Impulse arrives at sample (200 - 128) = 72 of block 2.
        assert!((out2[0][72] - 1.0).abs() < 1e-6, "got {}", out2[0][72]);
        // Surrounding samples should be silent.
        assert!(out2[0][71].abs() < 1e-6);
        assert!(out2[0][73].abs() < 1e-6);
    }

    #[test]
    fn reset_clears_history() {
        let mut e = TimeDomainConvEngine::new(1, 1, 4);
        e.set_ir(0, 0, &[1.0, 1.0, 1.0, 1.0]);

        let mut input = [[1.0_f32; BLOCK_SIZE]; 1];
        let mut out = [[0.0_f32; BLOCK_SIZE]; 1];
        e.process(&input, &mut out);
        e.reset();

        // After reset, processing zeros should yield zeros.
        input[0].fill(0.0);
        e.process(&input, &mut out);
        for v in out[0] {
            assert_eq!(v, 0.0);
        }
    }
}
