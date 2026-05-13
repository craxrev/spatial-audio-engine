//! End-to-end tests of the bundled HRTF decoder. Loads the real
//! `data/hrtf_decoder_native.bin` and confirms the §13 ear-ordering
//! convention by routing a known source direction through the full
//! M3+M4 pipeline.

use engine::consts::BLOCK_SIZE;
use engine::{Engine, Hrtf};

const HRTF_BYTES: &[u8] =
    include_bytes!("../../../data/hrtf_decoder_native.bin");

fn energy(buf: &[f32]) -> f32 {
    buf.iter().map(|v| v * v).sum()
}

fn settle(e: &mut Engine, input: &[[f32; BLOCK_SIZE]], n: usize) {
    for _ in 0..n {
        e.process_block(input);
    }
}

fn dc_input() -> Vec<[f32; BLOCK_SIZE]> {
    vec![[1.0_f32; BLOCK_SIZE]]
}

fn build_engine_with_source_at(x: f32, y: f32, z: f32) -> Engine {
    let hrtf = Hrtf::load_from_bytes(HRTF_BYTES).expect("hrtf load");
    let mut e = Engine::new(48000, 1);
    e.load_main_hrtf(&hrtf);
    e.set_source_active(0, true);
    e.set_source_position(0, x, y, z);
    e.set_source_gain(0, 1.0);
    e
}

#[test]
fn stereo_output_is_nonzero_after_decode() {
    let mut e = build_engine_with_source_at(0.0, 0.0, 1.0); // overhead omni-ish
    settle(&mut e, &dc_input(), 8);
    assert!(energy(&e.stereo_out[0]) > 0.0);
    assert!(energy(&e.stereo_out[1]) > 0.0);
}

#[test]
fn left_native_source_lights_left_ear() {
    // native +Y = listener's left → ear 0 (left) louder.
    let mut e = build_engine_with_source_at(0.0, 5.0, 0.0);
    settle(&mut e, &dc_input(), 8);
    let l = energy(&e.stereo_out[0]);
    let r = energy(&e.stereo_out[1]);
    assert!(l > r, "+Y source should be louder in left ear: L={l}, R={r}");
}

#[test]
fn right_native_source_lights_right_ear() {
    let mut e = build_engine_with_source_at(0.0, -5.0, 0.0);
    settle(&mut e, &dc_input(), 8);
    let l = energy(&e.stereo_out[0]);
    let r = energy(&e.stereo_out[1]);
    assert!(r > l, "−Y source should be louder in right ear: L={l}, R={r}");
}

#[test]
fn front_source_is_roughly_symmetric() {
    // Median-plane source: L and R should be within ~3 dB.
    let mut e = build_engine_with_source_at(5.0, 0.0, 0.0);
    settle(&mut e, &dc_input(), 8);
    let l = energy(&e.stereo_out[0]);
    let r = energy(&e.stereo_out[1]);
    let ratio = (l / r).max(r / l);
    assert!(
        ratio < 2.0,
        "frontal source should be near-symmetric: L={l}, R={r}, ratio={ratio}"
    );
}

/// Diagnostic: print the first taps and peak of the W→L and W→R IRs.
/// For a typical HRTF, W-channel IRs should be near-identical (both
/// ears see roughly the same omni response).
#[test]
#[ignore]
fn w_filter_diagnostic() {
    let hrtf = Hrtf::load_from_bytes(HRTF_BYTES).unwrap();
    for ear in 0..2 {
        let ir = hrtf.ir(0, ear);
        let peak_idx = ir
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        println!(
            "\nW → ear {ear} IR: peak at tap {peak_idx} = {:+.4}",
            ir[peak_idx]
        );
        for (label, range) in [("0..32", 0..32usize), ("96..128", 96..128)] {
            print!("  taps {label}:");
            for (i, v) in ir[range.clone()].iter().enumerate() {
                if i % 8 == 0 {
                    println!();
                    print!("   ");
                }
                print!(" {v:+8.4}");
            }
            println!();
        }
    }
}

/// Diagnostic: dump per-filter DC gain and energy for each
/// (ambi, ear) cell. Lets us see whether the loaded IRs have a
/// reasonable L/R mirror structure or look broken.
#[test]
#[ignore]
fn filter_diagnostic() {
    let hrtf = Hrtf::load_from_bytes(HRTF_BYTES).unwrap();
    println!("\nFilter DC gain (sum of taps) and energy (sum of squared taps):");
    println!("ambi |     ear0_DC     ear1_DC |       ear0_E       ear1_E");
    for ambi in 0..16 {
        let l_dc: f32 = hrtf.ir(ambi, 0).iter().sum();
        let r_dc: f32 = hrtf.ir(ambi, 1).iter().sum();
        let l_e: f32 = hrtf.ir(ambi, 0).iter().map(|v| v * v).sum();
        let r_e: f32 = hrtf.ir(ambi, 1).iter().map(|v| v * v).sum();
        println!(
            "  {ambi:2} | {l_dc:+11.4} {r_dc:+11.4} | {l_e:11.4} {r_e:11.4}"
        );
    }
}

/// Diagnostic: dump ACN bus values and L/R outputs for a small set
/// of canonical source positions. Run with `cargo test -- --nocapture
/// --ignored bundle_diagnostic` to inspect.
#[test]
#[ignore]
fn bundle_diagnostic() {
    let hrtf = Hrtf::load_from_bytes(HRTF_BYTES).unwrap();
    let cases = [
        ("front (+X, native-forward)", (5.0, 0.0, 0.0)),
        ("left  (+Y, native-left)", (0.0, 5.0, 0.0)),
        ("right (−Y, native-right)", (0.0, -5.0, 0.0)),
        ("above (+Z, native-up)", (0.0, 0.0, 5.0)),
        ("back  (−X)", (-5.0, 0.0, 0.0)),
    ];
    for (label, (x, y, z)) in cases {
        let mut e = Engine::new(48000, 1);
        e.load_main_hrtf(&hrtf);
        e.set_source_active(0, true);
        e.set_source_position(0, x, y, z);
        e.set_source_gain(0, 1.0);
        settle(&mut e, &dc_input(), 8);
        let l = energy(&e.stereo_out[0]);
        let r = energy(&e.stereo_out[1]);
        println!("\n=== {label} ===");
        println!("  ACN steady-state (sample 127):");
        for k in 0..16 {
            println!("    ACN[{k:2}] = {:+.4}", e.ambi_bus[k][127]);
        }
        println!("  L energy = {l:8.4}    R energy = {r:8.4}    L/R = {:.3}", l / r.max(1e-12));
    }
}

#[test]
fn no_hrtf_means_silent_stereo() {
    let mut e = Engine::new(48000, 1);
    e.set_source_active(0, true);
    e.set_source_position(0, 0.0, 5.0, 0.0);
    e.set_source_gain(0, 1.0);
    settle(&mut e, &dc_input(), 4);
    // ambi_bus has signal, but stereo_out is zero.
    let l = energy(&e.stereo_out[0]);
    let r = energy(&e.stereo_out[1]);
    assert_eq!(l, 0.0);
    assert_eq!(r, 0.0);
}
