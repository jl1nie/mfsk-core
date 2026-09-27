// SPDX-License-Identifier: GPL-3.0-or-later
//! Working memory made once and lent to one user at a time (#499).
//!
//! An embedded receiver builds its tables and buffers while allocations prefer internal DRAM
//! and allocates nothing while it decodes; a [`Slot`] is how a `&self` receiver, shared
//! between threads on a host, lends such a buffer. Whoever finds it taken allocates its own.

use core::cell::UnsafeCell;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicBool, Ordering};

/// A value one borrower at a time may take.
pub(crate) struct Slot<T> {
    busy: AtomicBool,
    cell: UnsafeCell<T>,
}

// SAFETY: the cell is reached only through `take`, which hands it to one holder at a time.
unsafe impl<T: Send> Sync for Slot<T> {}

impl<T> Slot<T> {
    pub(crate) fn new(value: T) -> Self {
        Self {
            busy: AtomicBool::new(false),
            cell: UnsafeCell::new(value),
        }
    }

    /// The value, unless someone else holds it.
    pub(crate) fn take(&self) -> Option<Guard<'_, T>> {
        self.busy
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .ok()
            .map(|_| Guard(self))
    }
}

/// A held [`Slot`]; released on drop.
pub(crate) struct Guard<'a, T>(&'a Slot<T>);

impl<T> Deref for Guard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        // SAFETY: `busy` is held by this guard alone until it drops.
        unsafe { &*self.0.cell.get() }
    }
}

impl<T> DerefMut for Guard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: as above.
        unsafe { &mut *self.0.cell.get() }
    }
}

impl<T> Drop for Guard<'_, T> {
    fn drop(&mut self) {
        self.0.busy.store(false, Ordering::Release);
    }
}
