//! When to call the slot grid locked, and when to go get a new one.
//!
//! Split out of `m5stack-cores3-app`'s decode pipeline so it can be
//! tested on the host. Everything here is a pure function of the
//! per-slot decode count — no clock, no audio, no hardware — and the
//! two bugs it exists to prevent were both reachable from a short
//! sequence of counts, which is exactly what a unit test is for. They
//! cost several reflash-and-watch cycles to find on the board instead.
//!
//! The model, in one line: **the grid is set once and held; a run of
//! slots that are not decoding properly is what sends us back for a
//! new one** (#356).

/// Decodes a slot must produce before the grid counts as locked.
///
/// Three, because one is demonstrably reachable on a grid that is a
/// full second wrong: at a 1.2 s offset `MFSK_CORES3_SIM` decoded
/// exactly one station for 13 slots straight, against 8 for the same
/// audio aligned. Two would still be inside the noise of that; three
/// is the smallest count that says the window is genuinely on the band
/// rather than clipping one strong signal's edge.
pub const LOCK_MIN_DECODES: usize = 3;

/// Consecutive under-par slots before a cold acquisition, from a
/// standing start (nothing has ever locked).
pub const ACQUIRE_TRIGGER_SLOTS: u32 = 3;

/// The same count once a lock has produced decodes. Higher, because
/// after a lock a run of empty slots is usually a quiet band rather
/// than a lost grid, and re-acquiring costs 25 s of capture plus
/// whatever the grid would have decoded meanwhile. Six slots is 90 s
/// of genuinely nothing heard.
pub const REACQUIRE_TRIGGER_SLOTS: u32 = 6;

/// What the caller should do with the grid after a slot's decodes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GridAction {
    /// Nothing to do.
    Hold,
    /// This slot cleared [`LOCK_MIN_DECODES`] and the grid was not
    /// locked before — raise `GridLock::Air`.
    Lock { n_dec: usize },
    /// Long enough without decoding properly. Arm the capture ring and
    /// run an acquisition. `relock` is true when a working grid is
    /// being given up, which the caller reports differently and which
    /// also drops the lock state.
    Acquire { slots: u32, relock: bool },
}

/// Lock state and the under-par run that leads back to acquisition.
#[derive(Debug, Default, Clone, Copy)]
pub struct GridState {
    /// Highest decode count seen since the current lock. `0` means
    /// nothing has locked yet.
    best_n: usize,
    /// Consecutive slots that were not decoding properly.
    lost_slots: u32,
    /// An acquisition is already in flight; the trigger stays quiet
    /// until the caller reports its outcome.
    acquiring: bool,
}

impl GridState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn is_locked(&self) -> bool {
        self.best_n > 0
    }

    pub fn best_n(&self) -> usize {
        self.best_n
    }

    pub fn lost_slots(&self) -> u32 {
        self.lost_slots
    }

    /// Fold in one slot's decode count.
    pub fn observe(&mut self, n_dec: usize) -> GridAction {
        let was_locked = self.is_locked();

        // **One decode is not a lock.** A grid a full second out still
        // decodes the odd station, and the first cut of lock-and-hold
        // treated that as good enough: it set `best_n`, and the trigger
        // — `n_dec == 0` at the time — could then never fire again, so
        // the receiver sat at 1-of-8 indefinitely with nothing able to
        // correct it.
        let locking = n_dec >= LOCK_MIN_DECODES && !was_locked;
        if n_dec >= LOCK_MIN_DECODES {
            self.best_n = self.best_n.max(n_dec);
        }

        // Before a lock, "not decoding properly" — not "not decoding at
        // all". After one, the test stays at zero: a few empty slots
        // read as a quiet band sooner than as a grid that was
        // demonstrably working going bad.
        let below_par = if was_locked {
            n_dec == 0
        } else {
            n_dec < LOCK_MIN_DECODES
        };
        self.lost_slots = if below_par { self.lost_slots + 1 } else { 0 };

        if locking {
            return GridAction::Lock { n_dec };
        }

        let trigger = if was_locked {
            REACQUIRE_TRIGGER_SLOTS
        } else {
            ACQUIRE_TRIGGER_SLOTS
        };
        if self.lost_slots >= trigger && !self.acquiring {
            self.acquiring = true;
            let slots = self.lost_slots;
            if was_locked {
                // Give up the lock while the capture runs, so the state
                // matches reality and the panel stops claiming a grid
                // the decoder can no longer demonstrate.
                self.best_n = 0;
            }
            return GridAction::Acquire {
                slots,
                relock: was_locked,
            };
        }
        GridAction::Hold
    }

    /// Report that the in-flight acquisition finished. `applied` is
    /// true when a phase cleared the confidence gate and was used, in
    /// which case the under-par run starts over — the grid just moved,
    /// so the slots that led here say nothing about the new one.
    pub fn acquisition_done(&mut self, applied: bool) {
        self.acquiring = false;
        if applied {
            self.lost_slots = 0;
        }
    }

    pub fn is_acquiring(&self) -> bool {
        self.acquiring
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The bug this module was extracted for. A grid ~1 s out decodes
    /// one station per slot forever; the receiver has to notice.
    #[test]
    fn one_decode_a_slot_is_not_a_lock_and_still_reaches_acquisition() {
        let mut g = GridState::new();
        assert_eq!(g.observe(1), GridAction::Hold);
        assert_eq!(g.observe(1), GridAction::Hold);
        assert!(!g.is_locked(), "one decode a slot must not lock the grid");
        assert_eq!(
            g.observe(1),
            GridAction::Acquire {
                slots: 3,
                relock: false
            }
        );
    }

    /// The same sequence under the original rule (`n_dec == 0`) never
    /// triggered, which is what pinned the receiver at 1-of-8. Guard
    /// the property directly: no run of under-par slots may be silent.
    #[test]
    fn an_under_par_run_always_terminates_in_acquisition() {
        for n in 0..LOCK_MIN_DECODES {
            let mut g = GridState::new();
            let mut fired = false;
            for _ in 0..ACQUIRE_TRIGGER_SLOTS * 4 {
                if matches!(g.observe(n), GridAction::Acquire { .. }) {
                    fired = true;
                    break;
                }
            }
            assert!(fired, "n_dec={n} never reached acquisition");
        }
    }

    #[test]
    fn a_real_lock_holds_and_does_not_re_acquire_on_a_quiet_slot() {
        let mut g = GridState::new();
        assert_eq!(g.observe(8), GridAction::Lock { n_dec: 8 });
        assert!(g.is_locked());
        // Well short of the lock threshold, but a locked grid rides it
        // out rather than throwing away something that works.
        for _ in 0..20 {
            assert_eq!(g.observe(1), GridAction::Hold);
        }
        assert!(g.is_locked());
    }

    #[test]
    fn a_locked_grid_that_goes_silent_re_acquires_and_drops_the_lock() {
        let mut g = GridState::new();
        g.observe(8);
        for _ in 0..REACQUIRE_TRIGGER_SLOTS - 1 {
            assert_eq!(g.observe(0), GridAction::Hold);
        }
        assert_eq!(
            g.observe(0),
            GridAction::Acquire {
                slots: REACQUIRE_TRIGGER_SLOTS,
                relock: true
            }
        );
        assert!(!g.is_locked(), "the lock is given up while re-acquiring");
    }

    /// A locked grid tolerates more silence than a cold start, so a
    /// quiet band does not cost a 25 s capture every three slots.
    #[test]
    fn a_lock_buys_a_longer_leash() {
        let mut cold = GridState::new();
        let mut warm = GridState::new();
        warm.observe(8);
        let mut cold_at = None;
        let mut warm_at = None;
        for i in 1..=REACQUIRE_TRIGGER_SLOTS {
            if matches!(cold.observe(0), GridAction::Acquire { .. }) && cold_at.is_none() {
                cold_at = Some(i);
            }
            if matches!(warm.observe(0), GridAction::Acquire { .. }) && warm_at.is_none() {
                warm_at = Some(i);
            }
        }
        assert_eq!(cold_at, Some(ACQUIRE_TRIGGER_SLOTS));
        assert_eq!(warm_at, Some(REACQUIRE_TRIGGER_SLOTS));
        assert!(warm_at > cold_at);
    }

    /// While a capture is in flight the trigger stays quiet — one
    /// acquisition at a time, however long the run gets.
    #[test]
    fn no_second_acquisition_while_one_is_in_flight() {
        let mut g = GridState::new();
        for _ in 0..ACQUIRE_TRIGGER_SLOTS {
            g.observe(0);
        }
        assert!(g.is_acquiring());
        for _ in 0..10 {
            assert_eq!(g.observe(0), GridAction::Hold);
        }
        // An inconclusive attempt leaves the run standing, so the next
        // slot can trigger the retry immediately.
        g.acquisition_done(false);
        assert_eq!(
            g.observe(0),
            GridAction::Acquire {
                slots: ACQUIRE_TRIGGER_SLOTS + 11,
                relock: false
            }
        );
    }

    /// A phase that was applied moved the grid, so the slots that led
    /// there say nothing about the new one — the run starts over.
    #[test]
    fn an_applied_acquisition_resets_the_run() {
        let mut g = GridState::new();
        for _ in 0..ACQUIRE_TRIGGER_SLOTS {
            g.observe(0);
        }
        g.acquisition_done(true);
        assert_eq!(g.lost_slots(), 0);
        assert_eq!(g.observe(0), GridAction::Hold);
    }

    /// The measured recovery, as a sequence: 1-of-8 for a few slots,
    /// acquisition, a bad phase that decodes nothing, a second
    /// acquisition, then a grid that works.
    #[test]
    fn the_hardware_recovery_sequence() {
        let mut g = GridState::new();
        assert_eq!(g.observe(1), GridAction::Hold);
        assert_eq!(g.observe(1), GridAction::Hold);
        assert!(matches!(g.observe(1), GridAction::Acquire { .. }));
        g.acquisition_done(true); // -5.75 s, applied, and wrong
        for _ in 0..ACQUIRE_TRIGGER_SLOTS - 1 {
            assert_eq!(g.observe(0), GridAction::Hold);
        }
        assert!(matches!(g.observe(0), GridAction::Acquire { .. }));
        g.acquisition_done(true); // +6.60 s, applied, close enough
        assert_eq!(g.observe(5), GridAction::Lock { n_dec: 5 });
        assert!(g.is_locked());
    }
}
