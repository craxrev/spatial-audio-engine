//! Spatial audio engine — clean-room implementation of the spec in
//! `design notes`. See `development notes` for the phased build plan.

pub mod consts;
pub mod conv;
pub mod coord;
pub mod decoder;
pub mod distance;
pub mod engine;
pub mod hrtf;
pub mod math;
pub mod ramp;
pub mod sh;
pub mod source;

pub use engine::{Engine, Listener};
pub use hrtf::Hrtf;
pub use math::{Quat, Vec3};
pub use source::Source;
