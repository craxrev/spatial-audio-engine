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

fn settle(e: &mut Engine, input: &[[[f32; BLOCK_SIZE]; 2]], n: usize) {
    for _ in 0..n {
        e.process_block(input, &[]);
    }
}

fn dc_input() -> Vec<[[f32; BLOCK_SIZE]; 2]> {
    vec![[[1.0_f32; BLOCK_SIZE]; 2]]
}

/// Drive a stereo impulse into source 0, then accumulate stereo-out
/// energy across `tail_blocks` silent blocks. Broadband-equivalent
/// way to probe HRTF L/R response without DC-bias artefacts.
fn impulse_tail_energy(e: &mut Engine, tail_blocks: usize) -> (f32, f32) {
    let mut input = [[[0.0_f32; BLOCK_SIZE]; 2]; 1];
    input[0][0][0] = 1.0;
    input[0][1][0] = 1.0;
    e.process_block(&input, &[]);
    input[0][0].fill(0.0);
    input[0][1].fill(0.0);
    let (mut l, mut r) = (0.0_f32, 0.0_f32);
    for _ in 0..tail_blocks {
        e.process_block(&input, &[]);
        l += energy(&e.stereo_out[0]);
        r += energy(&e.stereo_out[1]);
    }
    (l, r)
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
    let mut e = build_engine_with_source_at(0.0, 5.0, 0.0);
    let (l, r) = impulse_tail_energy(&mut e, 16);
    assert!(l > r, "+Y source should be louder in left ear: L={l}, R={r}");
}

#[test]
fn right_native_source_lights_right_ear() {
    let mut e = build_engine_with_source_at(0.0, -5.0, 0.0);
    let (l, r) = impulse_tail_energy(&mut e, 16);
    assert!(r > l, "−Y source should be louder in right ear: L={l}, R={r}");
}

#[test]
fn front_source_is_roughly_symmetric() {
    // Median-plane source: L and R should be within ~3 dB.
    let mut e = build_engine_with_source_at(5.0, 0.0, 0.0);
    let (l, r) = impulse_tail_energy(&mut e, 16);
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

#[test]
fn reverb_tail_reaches_stereo_out() {
    // With reverb engaged, after an impulse + silence, the binaural
    // output should still carry tail energy long after the source has
    // stopped — the reverb FDN must reach stereo_out.
    let mut e = build_engine_with_source_at(3.0, 0.0, 0.0);
    e.set_source_reverb_send(0, 1.0);
    e.set_reverb_amount(1.0);

    let mut input = [[[0.0_f32; BLOCK_SIZE]; 2]; 1];
    input[0][0][0] = 1.0;
    e.process_block(&input, &[]);

    // Run 200 silent blocks (~533ms at 48k) so the impulse propagates
    // into and back out of the FDN delays and HRTF convolution.
    input[0][0].fill(0.0); input[0][1].fill(0.0);
    let mut tail_energy = 0.0_f32;
    for _ in 0..200 {
        e.process_block(&input, &[]);
        tail_energy += energy(&e.stereo_out[0]) + energy(&e.stereo_out[1]);
    }
    assert!(tail_energy > 0.0, "reverb tail never reached stereo_out");
}

#[test]
fn reverb_amount_zero_silences_tail() {
    // Same scenario but reverb_amount = 0: the binaural output's tail
    // (long after impulse) should be silent.
    let mut e = build_engine_with_source_at(3.0, 0.0, 0.0);
    e.set_source_reverb_send(0, 1.0);
    e.set_reverb_amount(0.0);

    let mut input = [[[0.0_f32; BLOCK_SIZE]; 2]; 1];
    input[0][0][0] = 1.0;
    e.process_block(&input, &[]);

    input[0][0].fill(0.0); input[0][1].fill(0.0);
    let mut tail_energy = 0.0_f32;
    // Skip 50 blocks past the direct-path HRTF settle, then measure.
    for _ in 0..50 {
        e.process_block(&input, &[]);
    }
    for _ in 0..200 {
        e.process_block(&input, &[]);
        tail_energy += energy(&e.stereo_out[0]) + energy(&e.stereo_out[1]);
    }
    assert!(tail_energy < 1e-6, "tail should be silent with reverb_amount=0: {tail_energy}");
}

const W_BIN_A_BYTES: &[u8] = include_bytes!("../../../data/hrtf_post_filter_a.bin");
const W_BIN_B_BYTES: &[u8] = include_bytes!("../../../data/hrtf_post_filter_b.bin");

#[test]
fn w_binauralizer_adds_envelopment_to_stereo_out() {
    // Compare stereo output energy with and without the W-binauralizer
    // loaded. With it, the diffuse-field layer adds energy on top.
    let hrtf = Hrtf::load_from_bytes(HRTF_BYTES).unwrap();

    let make_engine = |with_wbin: bool| {
        let mut e = Engine::new(48000, 1);
        e.load_main_hrtf(&hrtf);
        if with_wbin {
            assert!(e.load_w_binauralizer(W_BIN_A_BYTES, W_BIN_B_BYTES));
        }
        e.set_source_active(0, true);
        e.set_source_position(0, 5.0, 0.0, 0.0);
        e.set_source_gain(0, 1.0);
        e
    };

    let input = dc_input();
    let mut e_off = make_engine(false);
    let mut e_on  = make_engine(true);

    settle(&mut e_off, &input, 32);
    settle(&mut e_on,  &input, 32);

    let off_energy = energy(&e_off.stereo_out[0]) + energy(&e_off.stereo_out[1]);
    let on_energy  = energy(&e_on.stereo_out[0])  + energy(&e_on.stereo_out[1]);

    assert!(on_energy > off_energy,
            "W-binauralizer should add stereo energy: off={off_energy}, on={on_energy}");
}

#[test]
fn engine_at_44_1k_resamples_hrtf_and_w_bin() {
    // Run the full pipeline at 44.1 kHz: the HRTF and W-binauralizer
    // IRs must auto-resample at load. Output should still be non-zero.
    let mut e = Engine::new(44_100, 1);
    let hrtf = Hrtf::load_from_bytes_at(HRTF_BYTES, 44_100).expect("hrtf load");
    e.load_main_hrtf(&hrtf);
    assert!(e.load_w_binauralizer(W_BIN_A_BYTES, W_BIN_B_BYTES));
    e.set_source_active(0, true);
    e.set_source_position(0, 5.0, 0.0, 0.0);
    e.set_source_gain(0, 1.0);
    settle(&mut e, &dc_input(), 16);
    let l = energy(&e.stereo_out[0]);
    let r = energy(&e.stereo_out[1]);
    assert!(l > 0.0 && r > 0.0, "44.1k pipeline silent: L={l}, R={r}");
}

/// Equivalence test: the new "linked stereo pair at width = 0" config —
/// two sources stacked at the same position, each fed one host channel —
/// should produce audibly identical output to the old "single stereo
/// object" config (one source, `input_channel_count = 2`, both host
/// channels collapsed at the same position per spec §6.7).
///
/// Not bit-identical because the float accumulation order differs and
/// ramps are duplicated, but within a few ULPs after the ramps settle.
#[test]
fn pair_at_zero_width_matches_legacy_mono_stereo_object() {
    fn make_mono(x: f32, y: f32, z: f32) -> Engine {
        let hrtf = Hrtf::load_from_bytes(HRTF_BYTES).expect("hrtf load");
        let mut e = Engine::new(48000, 1);
        e.load_main_hrtf(&hrtf);
        e.set_source_active(0, true);
        e.set_source_position(0, x, y, z);
        e.set_source_gain(0, 1.0);
        e.set_source_input_channel_count(0, 2);
        e
    }
    fn make_pair(x: f32, y: f32, z: f32) -> Engine {
        let hrtf = Hrtf::load_from_bytes(HRTF_BYTES).expect("hrtf load");
        let mut e = Engine::new(48000, 2);
        e.load_main_hrtf(&hrtf);
        for i in 0..2 {
            e.set_source_active(i, true);
            e.set_source_position(i, x, y, z); // identical position = width 0
            e.set_source_gain(i, 1.0);
            e.set_source_input_channel_count(i, 1);
        }
        e
    }

    // Deterministic broadband stereo input: one impulse per channel.
    let mut stereo_l = [0.0_f32; BLOCK_SIZE];
    let mut stereo_r = [0.0_f32; BLOCK_SIZE];
    stereo_l[0] = 1.0;
    stereo_r[0] = 0.7;

    let mono_slab: Vec<[[f32; BLOCK_SIZE]; 2]> = vec![[stereo_l, stereo_r]];
    let zeros = [0.0_f32; BLOCK_SIZE];
    let pair_slab: Vec<[[f32; BLOCK_SIZE]; 2]> = vec![
        [stereo_l, zeros], // src 0 reads ch0 only (mono); L → ch0
        [stereo_r, zeros], // src 1 reads ch0 only (mono); R → ch0
    ];
    let silence_mono: Vec<[[f32; BLOCK_SIZE]; 2]> = vec![[zeros, zeros]];
    let silence_pair: Vec<[[f32; BLOCK_SIZE]; 2]> = vec![[zeros, zeros], [zeros, zeros]];

    // Test several HRTF directions to make sure the equivalence isn't
    // a happy coincidence at one position.
    let positions = [
        ( 5.0_f32,  0.0,  0.0), // front
        ( 0.0,      5.0,  0.0), // left
        ( 0.0,     -5.0,  0.0), // right
        ( 0.0,      0.0,  5.0), // above
        ( 3.0,      4.0,  0.0), // off-axis horizontal
    ];

    for (x, y, z) in positions {
        let mut e_mono = make_mono(x, y, z);
        let mut e_pair = make_pair(x, y, z);

        // Settle: a couple blocks of silence so gain / directivity /
        // SH-crossfade ramps reach their steady state before measuring.
        for _ in 0..3 {
            e_mono.process_block(&silence_mono, &[]);
            e_pair.process_block(&silence_pair, &[]);
        }

        // Now drive an impulse + observe one block.
        e_mono.process_block(&mono_slab, &[]);
        e_pair.process_block(&pair_slab, &[]);

        // Then capture several tail blocks (HRTF response runs out
        // ~128 taps long).
        let mut max_diff = 0.0_f32;
        for _ in 0..4 {
            e_mono.process_block(&silence_mono, &[]);
            e_pair.process_block(&silence_pair, &[]);
            for ch in 0..2 {
                for i in 0..BLOCK_SIZE {
                    let d = (e_mono.stereo_out[ch][i] - e_pair.stereo_out[ch][i]).abs();
                    if d > max_diff {
                        max_diff = d;
                    }
                }
            }
        }

        // Tolerance: sub-LSB float drift accumulates across SH-encode +
        // 16-channel HRTF convolution + externalizer + W-binauralizer.
        // 5e-5 is comfortably below any audible threshold (>−85 dB FS).
        assert!(
            max_diff < 5e-5,
            "pair@width=0 should match mono stereo-object at ({x},{y},{z}); max_diff = {max_diff}"
        );
    }
}
