//! Spatial audio engine — clean-room implementation of the spec in
//! `design notes`. See `development notes` for the phased build plan.

pub mod consts;
pub mod coord;
pub mod distance;
pub mod engine;
pub mod math;
pub mod ramp;
pub mod sh;
pub mod source;

pub use engine::{Engine, Listener};
pub use math::{Quat, Vec3};
pub use source::Source;
