use crate::{Library, Params, SolverCallback, SolverCallbackStage, SolverDesc};
use nvflex_sys::*;

/// Initialize the solver desc to its default values
///
/// # Parameters
///
/// * `desc` - Mutable reference to a description structure that will be initialized to default values
#[inline]
pub fn set_solver_desc_defaults(desc: &mut SolverDesc) {
    unsafe { NvFlexSetSolverDescDefaults(desc as *mut _ as *mut _) }
}

/// Opaque type representing a collection of particles and constraints
#[derive(Debug)]
#[repr(C)]
pub struct Solver {
    pub(crate) solver: *mut NvFlexSolver,
}

impl Solver {
    /// Create a new particle solver
    ///
    /// # Parameters
    ///
    /// * `lib` - The library instance to use
    /// * `desc` - Reference to a solver description structure used to create the solver
    #[inline]
    pub fn create(lib: &Library, desc: &SolverDesc) -> Self {
        unsafe {
            Self {
                solver: NvFlexCreateSolver(lib.lib, desc as *const _ as *const _),
            }
        }
    }

    /// Return the library associated with a solver
    ///
    /// # Returns
    ///
    /// A library instance
    #[inline]
    pub fn get_solver_library(&self) -> Library {
        unsafe {
            Library {
                lib: NvFlexGetSolverLibrary(self.solver),
            }
        }
    }

    /// Return the solver desc that was used to create a solver
    #[inline]
    pub fn get_solver_desc(&self) -> SolverDesc {
        unsafe {
            let mut desc = std::mem::zeroed::<SolverDesc>();
            NvFlexGetSolverDesc(self.solver, &mut desc as *mut _ as *mut _);
            desc
        }
    }

    /// Registers a callback for a solver stage, the callback will be invoked from the same thread that calls NvFlexUpdateSolver().
    ///
    /// # Parameters
    ///
    /// * `function` - A pointer to a function that will be called during the solver update
    /// * `stage` - The stage of the update at which the callback function will be called
    ///
    /// # Returns
    ///
    /// The previously registered callback for this slot, this allows multiple users to chain callbacks together
    #[inline]
    pub fn register_solver_callback(
        &self,
        function: SolverCallback,
        stage: SolverCallbackStage,
    ) -> SolverCallback {
        unsafe { NvFlexRegisterSolverCallback(self.solver, function, stage as _) }
    }

    // TODO: translate the C code example to Rust
    /// Integrate particle solver forward in time. Below is an example of how to step Flex in the context of a simple game loop:
    ///
    /// ```c
    /// NvFlexLibrary* library = NvFlexInit();
    /// NvFlexSolver* solver = NvFlexCreateSolver(library);
    ///
    /// NvFlexBuffer* particleBuffer = NvFlexAllocBuffer(library, n, sizeof(Vec4), eNvFlexBufferHost);
    /// NvFlexBuffer* velocityBuffer = NvFlexAllocBuffer(library, n, sizeof(Vec4), eNvFlexBufferHost);
    /// NvFlexBuffer* phaseBuffer = NvFlexAllocBuffer(library, n, sizeof(int), eNvFlexBufferHost);
    ///
    /// while(!done)
    /// {
    ///     // map buffers for reading / writing
    ///     float4* particles = (float4*)NvFlexMap(particles, eNvFlexMapWait);
    ///     float3* velocities  = (float3*)NvFlexMap(velocities, eNvFlexMapWait);
    ///     int* phases = (int*)NvFlexMap(phases, eNvFlexMapWait);
    ///
    ///     // spawn (user method)
    ///     SpawnParticles(particles, velocities, phases);
    ///
    ///     // render (user method)
    ///     RenderParticles(particles, velocities, phases);
    ///
    ///     // unmap buffers
    ///     NvFlexUnmap(particleBuffer);
    ///     NvFlexUnmap(velocityBuffer);
    ///     NvFlexUnmap(phaseBuffer);
    ///
    ///     // write to device (async)
    ///     NvFlexSetParticles(particleBuffer, n);
    ///     NvFlexSetVelocities(velocityBuffer, n);
    ///     NvFlexSetPhases(phaseBuffer, n);
    ///
    ///     // tick
    ///     NvFlexUpdateSolver(solver, dt, 1, NULL);
    ///
    ///     // read back (async)
    ///     NvFlexGetParticles(particleBuffer, n);
    ///     NvFlexGetVelocities(velocityBuffer, n);
    ///     NvFlexGetPhases(phaseBuffer, n);
    /// }
    ///
    /// NvFlexFreeBuffer(particleBuffer);
    /// NvFlexFreeBuffer(velocityBuffer);
    /// NvFlexFreeBuffer(phaseBuffer);
    ///
    /// NvFlexDestroySolver(solver);
    /// NvFlexShutdown(library);
    /// ```
    ///
    /// # Parameters
    ///
    /// * `dt` - Time to integrate the solver forward in time by
    /// * `substeps` - The time dt will be divided into the number of sub-steps given by this parameter
    /// * `enableTimers` - Whether to enable per-kernel timers for profiling. Note that profiling can substantially slow down overall performance so this param should only be true in non-release builds
    ///
    #[inline]
    pub fn update_solver(&self, dt: f32, substeps: i32, enable_timers: bool) {
        unsafe { NvFlexUpdateSolver(self.solver, dt, substeps, enable_timers) }
    }

    /// Update solver paramters
    ///
    /// # Parameters
    ///
    /// * `params` - Parameters structure in host memory, see [`Params`]
    #[inline]
    pub fn set_params(&self, params: &Params) {
        unsafe { NvFlexSetParams(self.solver, params as *const _ as *const _) }
    }

    /// Retrieve solver paramters, default values will be set at solver creation time
    ///
    /// # Returns
    ///
    /// Parameters structure
    #[inline]
    pub fn get_params(&self) -> Params {
        unsafe {
            let mut params = std::mem::zeroed::<Params>();
            NvFlexGetParams(self.solver, &mut params as *mut _ as *mut _);
            params
        }
    }

    pub fn set_active(&self) {
        
    }
}

impl Drop for Solver {
    #[inline]
    fn drop(&mut self) {
        unsafe {
            NvFlexDestroySolver(self.solver);
        }
    }
}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Send for Solver {}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Sync for Solver {}
