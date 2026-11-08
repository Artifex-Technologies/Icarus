//! Icarus's core: ROM images, what their numbers mean (definitions) and the formulas between stored and real values. No
//! communication with a vehicle happens here (that is `icarus-obd`), so everything in this crate is safe to try on a file.

pub mod def;
pub mod expr;
pub mod rom;
pub mod tires;
pub mod xdf;
