//! Lock-free, UI-agnostic background simulation progress tracking (`chip-sim`).

use std::sync::{
    Arc,
    atomic::{AtomicU8, AtomicU32, Ordering},
};

/// Coarse execution stage reported while a background stock simulation is running.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum SimProgressStage {
    #[default]
    PreparingStock = 0,
    Sweeping = 1,
    CheckingFixtures = 2,
    ComputingMetrics = 3,
    Complete = 4,
}

impl SimProgressStage {
    #[inline]
    pub const fn from_u8(value: u8) -> Self {
        match value {
            0 => Self::PreparingStock,
            1 => Self::Sweeping,
            2 => Self::CheckingFixtures,
            3 => Self::ComputingMetrics,
            _ => Self::Complete,
        }
    }

    #[inline]
    pub const fn label(self) -> &'static str {
        match self {
            Self::PreparingStock => "Preparing stock & target",
            Self::Sweeping => "Simulating toolpaths",
            Self::CheckingFixtures => "Checking fixtures",
            Self::ComputingMetrics => "Computing error metrics",
            Self::Complete => "Complete",
        }
    }
}

/// Snapshot of live background simulation progress (`0.0..=1.0`, `0.0..=100.0%`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimProgressSnapshot {
    pub fraction: f32,
    pub percent: f32,
    pub stage: SimProgressStage,
}

impl SimProgressSnapshot {
    /// Returns the rounded integer percent complete in `0..=100`.
    #[inline]
    pub fn percent_rounded(self) -> u8 {
        self.percent.round().clamp(0.0, 100.0) as u8
    }
}

/// Lock-free, UI-agnostic progress tracker shared between a background simulation worker
/// and a UI or CLI observer.
///
/// Tracks completion monotonically in permille (`0..=1000`, i.e. `0.0%`–`100.0%`) and invokes
/// an optional wakeup callback whenever the displayed integer percent or active stage changes.
pub struct SimProgressTracker {
    permille: AtomicU32,
    stage: AtomicU8,
    on_update: Option<Arc<dyn Fn() + Send + Sync>>,
}

impl std::fmt::Debug for SimProgressTracker {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let snap = self.snapshot();
        f.debug_struct("SimProgressTracker")
            .field("percent", &snap.percent)
            .field("stage", &snap.stage)
            .finish()
    }
}

impl Default for SimProgressTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl SimProgressTracker {
    pub const MAX_PERMILLE: u32 = 1000;

    /// Creates a new tracker initialized at `0%` (`SimProgressStage::PreparingStock`).
    pub fn new() -> Self {
        Self {
            permille: AtomicU32::new(0),
            stage: AtomicU8::new(SimProgressStage::PreparingStock as u8),
            on_update: None,
        }
    }

    /// Creates a new tracker that invokes `on_update` whenever the integer percentage
    /// (`0..=100`) or execution stage changes.
    pub fn with_callback(on_update: Arc<dyn Fn() + Send + Sync>) -> Self {
        Self {
            permille: AtomicU32::new(0),
            stage: AtomicU8::new(SimProgressStage::PreparingStock as u8),
            on_update: Some(on_update),
        }
    }

    /// Returns a [`SimProgressScope`] spanning the full `[0.0, 1.0]` progress range.
    #[inline]
    pub fn full_scope(&self) -> SimProgressScope<'_> {
        SimProgressScope {
            tracker: self,
            start_permille: 0,
            span_permille: Self::MAX_PERMILLE,
        }
    }

    /// Reads the current progress snapshot without locking.
    #[inline]
    pub fn snapshot(&self) -> SimProgressSnapshot {
        let p = self
            .permille
            .load(Ordering::Relaxed)
            .min(Self::MAX_PERMILLE);
        let stage = SimProgressStage::from_u8(self.stage.load(Ordering::Relaxed));
        let fraction = (p as f32) / (Self::MAX_PERMILLE as f32);
        SimProgressSnapshot {
            fraction,
            percent: fraction * 100.0,
            stage,
        }
    }

    /// Monotonically advances the recorded progress permille (`0..=1000`) and updates the active stage.
    pub fn set_permille_and_stage(&self, permille: u32, stage: SimProgressStage) {
        let clamped = permille.min(Self::MAX_PERMILLE);
        let prev_permille = self.permille.fetch_max(clamped, Ordering::Relaxed);
        let stage_u8 = stage as u8;
        let prev_stage = self.stage.swap(stage_u8, Ordering::Relaxed);
        let new_permille = prev_permille.max(clamped);
        if (new_permille / 10 > prev_permille / 10 || prev_stage != stage_u8)
            && let Some(cb) = &self.on_update
        {
            cb();
        }
    }
}

/// Sub-range projection onto a parent [`SimProgressTracker`], mapping local `[0.0, 1.0]`
/// progress linearly into `[start_permille, start_permille + span_permille]`.
#[derive(Clone, Copy, Debug)]
pub struct SimProgressScope<'a> {
    tracker: &'a SimProgressTracker,
    start_permille: u32,
    span_permille: u32,
}

impl<'a> SimProgressScope<'a> {
    /// Creates a child scope spanning `[start_frac, end_frac]` within this scope.
    #[inline]
    pub fn sub_scope(&self, start_frac: f64, end_frac: f64) -> Self {
        let s = if start_frac.is_finite() {
            start_frac.clamp(0.0, 1.0)
        } else {
            0.0
        };
        let e = if end_frac.is_finite() {
            end_frac.clamp(s, 1.0)
        } else {
            s
        };
        let span_f = f64::from(self.span_permille);
        let rel_start = (s * span_f).round() as u32;
        let rel_end = (e * span_f).round() as u32;
        let start = (self.start_permille + rel_start).min(SimProgressTracker::MAX_PERMILLE);
        let end = (self.start_permille + rel_end)
            .max(start)
            .min(SimProgressTracker::MAX_PERMILLE);
        Self {
            tracker: self.tracker,
            start_permille: start,
            span_permille: end - start,
        }
    }

    /// Reports a local completion fraction in `[0.0, 1.0]` for the given execution stage.
    #[inline]
    pub fn report(&self, local_frac: f64, stage: SimProgressStage) {
        let frac = if local_frac.is_finite() {
            local_frac.clamp(0.0, 1.0)
        } else {
            0.0
        };
        let offset =
            ((frac * f64::from(self.span_permille)).round() as u32).min(self.span_permille);
        self.tracker
            .set_permille_and_stage(self.start_permille + offset, stage);
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    };

    use super::*;

    #[test]
    fn tracker_is_monotonic_clamped_and_throttles_callbacks() {
        let calls = Arc::new(AtomicUsize::new(0));
        let cb_calls = Arc::clone(&calls);
        let tracker = SimProgressTracker::with_callback(Arc::new(move || {
            cb_calls.fetch_add(1, Ordering::Relaxed);
        }));

        let scope = tracker.full_scope();
        assert_eq!(tracker.snapshot().percent_rounded(), 0);
        assert_eq!(tracker.snapshot().stage, SimProgressStage::PreparingStock);

        // Sub-1% permille increments in the same stage should not spam callbacks.
        scope.report(0.002, SimProgressStage::PreparingStock);
        scope.report(0.004, SimProgressStage::PreparingStock);
        scope.report(0.009, SimProgressStage::PreparingStock);
        assert_eq!(calls.load(Ordering::Relaxed), 0);

        // Crossing 1% triggers a single wakeup.
        scope.report(0.010, SimProgressStage::PreparingStock);
        assert_eq!(calls.load(Ordering::Relaxed), 1);
        assert_eq!(tracker.snapshot().percent_rounded(), 1);

        // Stage transitions trigger a wakeup even without crossing another percent boundary.
        scope.report(0.011, SimProgressStage::Sweeping);
        assert_eq!(calls.load(Ordering::Relaxed), 2);
        assert_eq!(tracker.snapshot().stage, SimProgressStage::Sweeping);

        // Regressions and out-of-range / NaN values never move progress backward or exceed 100%.
        scope.report(0.50, SimProgressStage::Sweeping);
        assert_eq!(tracker.snapshot().percent_rounded(), 50);
        scope.report(0.25, SimProgressStage::Sweeping);
        assert_eq!(tracker.snapshot().percent_rounded(), 50);
        scope.report(f64::NAN, SimProgressStage::Sweeping);
        assert_eq!(tracker.snapshot().percent_rounded(), 50);
        scope.report(1.75, SimProgressStage::Complete);
        assert_eq!(tracker.snapshot().percent_rounded(), 100);
        assert!((tracker.snapshot().fraction - 1.0).abs() < 1e-6);
    }

    #[test]
    fn sub_scopes_map_carryover_chains_monotonically() {
        let tracker = SimProgressTracker::new();
        let full = tracker.full_scope();
        let first = full.sub_scope(0.0, 0.4);
        let second = full.sub_scope(0.4, 1.0);

        first.report(0.5, SimProgressStage::Sweeping);
        assert_eq!(tracker.snapshot().percent_rounded(), 20);
        first.report(1.0, SimProgressStage::Complete);
        assert_eq!(tracker.snapshot().percent_rounded(), 40);

        second.report(0.0, SimProgressStage::PreparingStock);
        assert_eq!(tracker.snapshot().percent_rounded(), 40);
        second.report(0.5, SimProgressStage::Sweeping);
        assert_eq!(tracker.snapshot().percent_rounded(), 70);
        second.report(1.0, SimProgressStage::Complete);
        assert_eq!(tracker.snapshot().percent_rounded(), 100);
    }
}
