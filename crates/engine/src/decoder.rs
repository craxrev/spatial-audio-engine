//! Main HRTF binaural decoder (§8): 16-ambi → 2-ear. Wraps a
//! `ConvolutionEngine` with the bundled-or-supplied HRTF coefficient
//! set.

use crate::consts::{BLOCK_SIZE, NUM_AMBI, OUTPUT_CHANNELS};
use crate::conv::{ConvolutionEngine, TimeDomainConvEngine};
use crate::hrtf::Hrtf;

pub struct HrtfDecoder {
    conv: TimeDomainConvEngine,
}

impl HrtfDecoder {
    pub fn new(hrtf: &Hrtf) -> Self {
        let mut conv = TimeDomainConvEngine::new(NUM_AMBI, OUTPUT_CHANNELS, hrtf.ir_len());
        for ambi in 0..NUM_AMBI {
            for ear in 0..OUTPUT_CHANNELS {
                conv.set_ir(ambi, ear, hrtf.ir(ambi, ear));
            }
        }
        Self { conv }
    }

    pub fn process(
        &mut self,
        ambi_bus: &[[f32; BLOCK_SIZE]],
        stereo_out: &mut [[f32; BLOCK_SIZE]; OUTPUT_CHANNELS],
    ) {
        self.conv.process(ambi_bus, &mut stereo_out[..]);
    }

    pub fn reset(&mut self) {
        self.conv.reset();
    }
}
