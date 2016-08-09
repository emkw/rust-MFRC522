/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
//! Helper Macros.

/// Helper macro unwrapping `Ok` value or returning its second parameter from fn.
#[macro_export]
macro_rules! ok_or_return {
	($expr:expr, $err_ret:expr) => (match $expr {
		::core::result::Result::Ok(val) => val,
		::core::result::Result::Err(_) => {
			return $err_ret
		}
	})
}

/// Helper macro for bus operations, unwrapping `Ok` value or returning from fn
/// with `Status::BusError`.
///
/// This is intendef for functions returning -> `Status` or `(Status, T)`.
///
/// For funcrions returning `(Status, T)`, it can be used used ie. like this:
///
/// ```
/// fn x() -> (Status, usize) {
/// 	let mut bytes_read = 0;
/// 	...
/// 	bus!(some_bus_operation(), return bytes_read);
/// 	...
/// }
/// ```
#[macro_export]
macro_rules! bus {
	// Variant for fn() -> Status
	($expr:expr) => ( ok_or_return!($expr, $crate::Status::BusError) );

	// Variant for fn() -> (Status, value)
	($expr:expr, return $value:expr) => ( ok_or_return!($expr, ($crate::Status::BusError, $value)) );
}

/// Shorthand try! macro mapping error to second parameter. _err is used as closure parameter.
#[macro_export]
macro_rules! try_map_err {
	($expr:expr, $map_err:expr)  => ( try!($expr.map_err(|_err| $map_err)) );
}

/// Shorthand try! macro for bus operations, mapping error to Status::BusError.
///
/// This is intended for functions returning -> `Result<T, Status>`.
#[macro_export]
macro_rules! try_bus {
	($expr:expr) => ( try_map_err!($expr, $crate::Status::BusError) )
}
