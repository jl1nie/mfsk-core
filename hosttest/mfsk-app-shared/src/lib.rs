// SPDX-License-Identifier: GPL-3.0-or-later
//! Host harness for the parts of `embedded-poc/mfsk-app-shared` that
//! do not touch the ESP-IDF.
//!
//! That crate depends on `esp-idf-svc`, so it only builds for Xtensa
//! and is excluded from this workspace — which means unit tests written
//! inside it never run anywhere. This repository has been bitten by
//! exactly that before (five protocols' golden tests silently skipping
//! in CI, see `docs/notes` and `MFSK_REQUIRE_CORPUS`), so a module whose
//! logic is worth testing gets compiled here instead of being trusted.
//!
//! Modules are pulled in by `#[path]` rather than copied: there is one
//! source file, and it is the one that ships.
//!
//! Only add a module here if it is genuinely target-independent. If it
//! needs `esp_idf_svc`, it does not belong.

/// wsprnet.org spot encoding — pure string building, no I/O.
#[path = "../../../embedded-poc/mfsk-app-shared/src/wsprnet.rs"]
pub mod wsprnet;

/// Unix-epoch → UTC calendar time — pure integer arithmetic, no I/O.
#[path = "../../../embedded-poc/mfsk-app-shared/src/civil_time.rs"]
pub mod civil_time;

/// The capture window a receiver opens on a UTC slot boundary — pure
/// sample arithmetic over a phase reading passed in, so the batch
/// splitting and the gap between windows are testable without a radio.
#[path = "../../../embedded-poc/mfsk-app-shared/src/capture_window.rs"]
pub mod capture_window;

/// WSPR band/dial-frequency table — pure data, no I/O.
#[path = "../../../embedded-poc/mfsk-app-shared/src/wspr_bands.rs"]
pub mod wspr_bands;

/// WSPR UI row formatting — pure string building, no I/O, no
/// `embedded-graphics` draw calls (those live in `wspr_list.rs`,
/// which does need this crate's `esp_idf_svc`-adjacent draw stack and
/// so is not pulled in here).
#[path = "../../../embedded-poc/mfsk-app-shared/src/ui/wspr_row.rs"]
pub mod wspr_row;

/// The spot-list container shared by the FST4 and WSPR screens —
/// `heapless` plus one atomic, no draw stack, so the truncation and
/// history-rolling rules can be tested where tests actually run.
#[path = "../../../embedded-poc/mfsk-app-shared/src/ui/spot_state.rs"]
pub mod spot_state;

/// Slot parity — pure arithmetic; `time_sync` needs it.
#[path = "../../../embedded-poc/mfsk-app-shared/src/parity.rs"]
pub mod parity;

/// The FT4 capture window's place on the slot grid — from
/// `embedded-shared`, not `mfsk-app-shared`, and here rather than in a
/// hosttest crate of its own because this one is what CI runs. It is
/// integer arithmetic over sample counts with no dependency at all;
/// the module it was split out of pulls in `esp_idf_svc` and so cannot
/// be compiled off-target, which left a live run against a radio as
/// the only thing checking it.
#[path = "../../../embedded-poc/embedded-shared/src/apps/ft4_grid.rs"]
pub mod ft4_grid;

/// Slot-boundary time sync. Pulled in for `ClockSource`: the rule that
/// only an NTP-disciplined clock may be written back to the RTC is the
/// kind of thing that was wrong for months without anything failing
/// (#354), so it gets a test where tests run.
#[path = "../../../embedded-poc/mfsk-app-shared/src/time_sync.rs"]
pub mod time_sync;

/// Persisted grid-phase fix — the NVS I/O is `#[cfg(espidf)]`, but the
/// `GridFix::correction_for` re-wrap and staleness logic is pure and
/// the thing most worth a test (a wrong sign here mis-aligns FT4).
#[path = "../../../embedded-poc/mfsk-app-shared/src/grid_fix.rs"]
pub mod grid_fix;

#[path = "../../../embedded-poc/mfsk-app-shared/src/grid_state.rs"]
pub mod grid_state;
