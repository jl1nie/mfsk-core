// SPDX-License-Identifier: GPL-3.0-only
//! The board UI modules that need a `crate::ui::...` path to resolve.
//!
//! A real directory rather than an inline `mod ui { .. }` in `lib.rs`:
//! `#[path]` inside an inline module resolves against `src/ui/`, and
//! the `..` components in it cannot traverse a directory that does not
//! exist on disk.

#[path = "../../../../embedded-poc/mfsk-app-shared/src/ui/state.rs"]
pub mod state;

#[path = "../../../../embedded-poc/mfsk-app-shared/src/ui/decoded_list.rs"]
pub mod decoded_list;
