//! Linear ramp toward a target value over a fixed sample count
//! (§6.5). Used for gain, send, occlusion, and other smoothed
//! parameters.

#[derive(Clone, Copy, Debug)]
pub struct Ramp {
    pub current: f32,
    pub target: f32,
    pub step: f32,
    pub remaining: u32,
    pub total: u32,
}

impl Ramp {
    pub fn new(initial: f32, total: u32) -> Self {
        Self {
            current: initial,
            target: initial,
            step: 0.0,
            remaining: 0,
            total,
        }
    }

    pub fn set_target(&mut self, target: f32) {
        self.target = target;
        if self.total == 0 {
            self.current = target;
            self.step = 0.0;
            self.remaining = 0;
        } else {
            self.step = (target - self.current) / self.total as f32;
            self.remaining = self.total;
        }
    }

    pub fn set_total(&mut self, total: u32) {
        self.total = total;
    }

    pub fn is_active(&self) -> bool {
        self.remaining > 0
    }

    pub fn tick(&mut self) -> f32 {
        if self.remaining > 0 {
            self.current += self.step;
            self.remaining -= 1;
            if self.remaining == 0 {
                self.current = self.target;
            }
        }
        self.current
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_starts_at_initial_and_idle() {
        let r = Ramp::new(0.5, 100);
        assert_eq!(r.current, 0.5);
        assert_eq!(r.target, 0.5);
        assert!(!r.is_active());
    }

    #[test]
    fn ramps_linearly_to_target_and_snaps() {
        let mut r = Ramp::new(0.0, 4);
        r.set_target(1.0);
        let expected = [0.25_f32, 0.5, 0.75, 1.0];
        for &want in &expected {
            let got = r.tick();
            assert!((got - want).abs() < 1e-6, "got {got}, want {want}");
        }
        assert!(!r.is_active());
        assert_eq!(r.current, 1.0);
    }

    #[test]
    fn tick_is_noop_when_idle() {
        let mut r = Ramp::new(0.3, 10);
        let before = r.current;
        for _ in 0..50 {
            r.tick();
        }
        assert_eq!(r.current, before);
    }

    #[test]
    fn zero_total_snaps_immediately() {
        let mut r = Ramp::new(0.0, 0);
        r.set_target(0.7);
        assert_eq!(r.current, 0.7);
        assert!(!r.is_active());
    }

    #[test]
    fn retarget_midway_resets_remaining() {
        let mut r = Ramp::new(0.0, 4);
        r.set_target(1.0);
        r.tick();
        r.tick();
        assert!(r.is_active());
        r.set_target(0.0);
        assert_eq!(r.remaining, 4);
        for _ in 0..4 {
            r.tick();
        }
        assert_eq!(r.current, 0.0);
    }
}
