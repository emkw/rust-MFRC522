/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
//! Log macro wrapperers, allowing building with ndebug.
//!
//! When building with feature `ndebug` or `nlog` those become a no-op.
//!
//! Normal log macros should be used in case of logging that do not require
//! `fmt::Debug` implementation.

#[cfg(not(any(feature= "nlog", feature = "ndebug")))]
macro_rules! ninfo {
    ($($arg:tt)*) => ( info!($($arg)*) );
}

#[cfg(any(feature= "nlog", feature = "ndebug"))]
macro_rules! ninfo {
	($($arg:tt)*) => ( () )
}

#[cfg(not(any(feature= "nlog", feature = "ndebug")))]
macro_rules! ndebug {
	($($arg:tt)*) => ( debug!($($arg)*) )
}

#[cfg(any(feature= "nlog", feature = "ndebug"))]
macro_rules! ndebug {
	($($arg:tt)*) => ( () )
}

#[cfg(not(any(feature= "nlog", feature = "ndebug")))]
macro_rules! ntrace {
    ($($arg:tt)*) => ( trace!($($arg)*) );
}

#[cfg(any(feature= "nlog", feature = "ndebug"))]
macro_rules! ntrace {
	($($arg:tt)*) => ( () )
}
