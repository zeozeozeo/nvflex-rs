mod buffer;
mod library;
mod mesh;
mod solver;
mod types;

pub use buffer::*;
pub use library::*;
pub use mesh::*;
pub use solver::*;
pub use types::*;

macro_rules! rstr {
    ($cstring:expr) => {{
        #[allow(unused_unsafe)]
        unsafe { std::ffi::CStr::from_ptr($cstring) }
            .to_str()
            .expect("unable to convert C string")
    }};
}

pub(crate) use rstr;

#[inline(always)]
pub fn make_phase_with_channels(group: i32, particle_flags: i32, shape_channels: i32) -> i32 {
    nvflex_sys::NvFlexMakePhaseWithChannels(group, particle_flags, shape_channels)
}

#[inline(always)]
pub fn make_phase(group: i32, particle_flags: i32) -> i32 {
    nvflex_sys::NvFlexMakePhase(group, particle_flags)
}
