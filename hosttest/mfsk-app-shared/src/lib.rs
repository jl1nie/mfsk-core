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

/// WSPR band/dial-frequency table — pure data, no I/O.
#[path = "../../../embedded-poc/mfsk-app-shared/src/wspr_bands.rs"]
pub mod wspr_bands;

/// WSPR UI row formatting — pure string building, no I/O, no
/// `embedded-graphics` draw calls (those live in `wspr_list.rs`,
/// which does need this crate's `esp_idf_svc`-adjacent draw stack and
/// so is not pulled in here).
#[path = "../../../embedded-poc/mfsk-app-shared/src/ui/wspr_row.rs"]
pub mod wspr_row;
