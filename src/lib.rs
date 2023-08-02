#![doc = include_str!("../README.md")]
#![warn(missing_docs)]
#![no_std]

mod fmt;

mod crc;
mod dfu;

#[doc(hidden)]
pub mod prelude {
    pub use crate::dfu::*;
}
