//! Async, hardware-agnostic driver for the DFRobot Maqueen Plus V3 robot.
//!
//! The crate mirrors the MakeCode extension `DFRobot_MaqueenPlus_V2` (v2.2.29),
//! which DFRobot also uses for the V3 hardware. It contains:
//!
//! - [`MaqueenPlusV3`]: the robot coprocessor at I2C address 0x10 (motors,
//!   headlights, line sensors, built-in line following, PID moves, telemetry).
//! - [`MatrixLidar`]: the 8×8 matrix laser ranging sensor at I2C address 0x30.
//! - [`nec::Decoder`]: a pure NEC infrared decoder fed with edge timings.
//! - [`ws2812`]: colour type and bit-stream encoder for the four RGB LEDs.
//!
//! Everything is generic over `embedded-hal-async` traits, so it runs on any
//! async HAL. The `francor-maqueen-plus-microbit` crate wires it to the BBC micro:bit V2.
#![cfg_attr(not(test), no_std)]
#![warn(missing_docs)]

pub mod lidar;
pub mod nec;
pub mod registers;
pub mod robot;
pub mod ws2812;

pub use lidar::MatrixLidar;
pub use registers::*;
pub use robot::MaqueenPlusV3;

/// Errors returned by the drivers in this crate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// The underlying I2C bus reported an error.
    I2c(E),
    /// The robot coprocessor did not report a version after reset (not powered, not plugged in).
    NotReady,
    /// A bounded wait (PID move, lidar response) ran out of time.
    Timeout,
    /// The lidar answered with a failure status or an unexpected command echo (the byte is included).
    LidarFailed(u8),
    /// An argument was outside the range the hardware accepts.
    InvalidArgument,
    /// The device returned bytes that do not form a valid response (too short, not UTF-8).
    InvalidData,
}

/// Result alias used throughout the crate.
pub type Result<T, E> = core::result::Result<T, Error<E>>;
