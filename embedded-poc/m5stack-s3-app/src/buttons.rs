//! KEY1 / KEY2 polling with debounce + long-press detection.
//!
//! Phase 0.7c: the only consumer right now is the boot-mode flip
//! (KEY2 held 2 s → `boot_mode::flip_and_restart`). The polling runs
//! inside the display loop (100 ms cadence), so this module just
//! tracks edge / hold state and emits events the caller can drain.
//!
//! Active-low with internal pull-up; `pressed = level == 0`.

use esp_idf_svc::sys::{
    gpio_get_level, gpio_pullup_en, gpio_set_direction, GPIO_MODE_DEF_INPUT,
};

use crate::board::{BTN_A_PIN, BTN_B_PIN};

const LONG_PRESS_MS: u32 = 2000;
/// Display loop pulses every 100 ms, so 20 ticks ≈ 2000 ms.
const LONG_PRESS_TICKS: u32 = LONG_PRESS_MS / 100;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Key {
    Key1,
    Key2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// Released within `LONG_PRESS_MS`. Fired on release.
    ShortPress(Key),
    /// Crossed the `LONG_PRESS_MS` threshold while still held. Fired
    /// once per hold (not on release) so a single sustained press
    /// can't trigger both a long and short event.
    LongPress(Key),
}

#[derive(Default)]
struct KeyState {
    held_ticks: u32,
    long_fired: bool,
    was_pressed: bool,
}

pub struct Buttons {
    key1: KeyState,
    key2: KeyState,
    /// Single-slot event queue. The display loop polls + drains every
    /// tick, so we never have more than one pending event.
    pending: Option<Event>,
}

impl Buttons {
    /// Configure both pins as input + pull-up. Call once during boot.
    pub fn init() -> Self {
        for pin in [BTN_A_PIN, BTN_B_PIN] {
            unsafe {
                gpio_set_direction(pin, GPIO_MODE_DEF_INPUT);
                gpio_pullup_en(pin);
            }
        }
        Self {
            key1: KeyState::default(),
            key2: KeyState::default(),
            pending: None,
        }
    }

    /// Read both pins and advance state machines. Call every 100 ms
    /// from the display loop.
    pub fn poll(&mut self) {
        let key1_pressed = unsafe { gpio_get_level(BTN_A_PIN) } == 0;
        let key2_pressed = unsafe { gpio_get_level(BTN_B_PIN) } == 0;
        if let Some(ev) = update(&mut self.key1, key1_pressed, Key::Key1) {
            self.pending = Some(ev);
        }
        if let Some(ev) = update(&mut self.key2, key2_pressed, Key::Key2) {
            self.pending = Some(ev);
        }
    }

    pub fn take_event(&mut self) -> Option<Event> {
        self.pending.take()
    }
}

fn update(state: &mut KeyState, pressed: bool, key: Key) -> Option<Event> {
    match (state.was_pressed, pressed) {
        (false, true) => {
            state.held_ticks = 1;
            state.long_fired = false;
            state.was_pressed = true;
            None
        }
        (true, true) => {
            state.held_ticks = state.held_ticks.saturating_add(1);
            if !state.long_fired && state.held_ticks >= LONG_PRESS_TICKS {
                state.long_fired = true;
                Some(Event::LongPress(key))
            } else {
                None
            }
        }
        (true, false) => {
            let fired_long = state.long_fired;
            state.was_pressed = false;
            state.held_ticks = 0;
            state.long_fired = false;
            if fired_long {
                None
            } else {
                Some(Event::ShortPress(key))
            }
        }
        (false, false) => None,
    }
}
