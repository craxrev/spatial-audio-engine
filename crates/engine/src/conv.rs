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

/// Direct-form FIR per cross-cell. IRs are stored time-domain at a
/// fixed length. Per-input delay line carries the previous block so
/// taps reaching past the current block read from there.
pub struct TimeDomainConvEngine {
    num_inputs: usize,
    num_outputs: usize,
    ir_len: usize,
    /// Per cross-cell IR, indexed `in_idx * num_outputs + out_idx`.
    irs: Vec<Vec<f32>>,
    /// Per-input previous block, used as the "history" half of a
    /// virtual 2×BLOCK_SIZE window.
    prev_inputs: Vec<[f32; BLOCK_SIZE]>,
}

impl TimeDomainConvEngine {
    pub fn new(num_inputs: usize, num_outputs: usize, ir_len: usize) -> Self {
        assert!(ir_len <= BLOCK_SIZE, "ir_len must fit in the prev-block window");
        Self {
            num_inputs,
            num_outputs,
            ir_len,
            irs: vec![vec![0.0; ir_len]; num_inputs * num_outputs],
            prev_inputs: vec![[0.0; BLOCK_SIZE]; num_inputs],
        }
    }

    pub fn set_ir(&mut self, in_idx: usize, out_idx: usize, ir: &[f32]) {
        let idx = in_idx * self.num_outputs + out_idx;
        let cell = &mut self.irs[idx];
        let n = ir.len().min(self.ir_len);
        cell[..n].copy_from_slice(&ir[..n]);
        for v in &mut cell[n..] {
            *v = 0.0;
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
        for out in outputs.iter_mut() {
            out.fill(0.0);
        }

        // For each output sample n, accumulate over all (in_idx, k):
        //   y[n] += x[in_idx, n - k] * h[in_idx, out_idx, k]
        // where x at negative offsets comes from prev_inputs.
        #[allow(clippy::needless_range_loop)]
        for n in 0..BLOCK_SIZE {
            for (out_idx, out) in outputs.iter_mut().enumerate() {
                let mut acc = 0.0_f32;
                for in_idx in 0..self.num_inputs {
                    let ir = &self.irs[in_idx * self.num_outputs + out_idx];
                    let curr = &inputs[in_idx];
                    let prev = &self.prev_inputs[in_idx];
                    for k in 0..self.ir_len {
                        let x = if n >= k {
                            curr[n - k]
                        } else {
                            prev[BLOCK_SIZE + n - k]
                        };
                        acc += x * ir[k];
                    }
                }
                out[n] = acc;
            }
        }

        // Save current inputs as next block's history.
        for (prev, curr) in self.prev_inputs.iter_mut().zip(inputs.iter()) {
            prev.copy_from_slice(curr);
        }
    }

    fn reset(&mut self) {
        for p in &mut self.prev_inputs {
            p.fill(0.0);
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
