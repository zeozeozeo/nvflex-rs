mod buffer;
mod library;
mod mesh;
mod solver;
mod types;
mod vector;

#[cfg(feature = "ext")]
pub mod ext;

pub use buffer::*;
pub use library::*;
pub use mesh::*;
pub use solver::*;
pub use types::*;
pub use vector::*;

#[macro_export]
macro_rules! rstr {
    ($cstring:expr) => {{
        #[allow(unused_unsafe)]
        unsafe { std::ffi::CStr::from_ptr($cstring) }
            .to_str()
            .expect("unable to convert C string")
    }};
}

#[inline(always)]
pub fn make_phase_with_channels(group: i32, particle_flags: i32, shape_channels: i32) -> i32 {
    nvflex_sys::NvFlexMakePhaseWithChannels(group, particle_flags, shape_channels)
}

#[inline(always)]
pub fn make_phase(group: i32, particle_flags: i32) -> i32 {
    nvflex_sys::NvFlexMakePhase(group, particle_flags)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_library() {
        let _ = Library::new(None, None).unwrap();
    }

    #[test]
    fn library_with_solver() {
        let lib = Library::new(None, None).unwrap();
        let _ = Solver::new(lib, &SolverDesc::default());
    }
}
