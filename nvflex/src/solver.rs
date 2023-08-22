use crate::{
    rstr, Buffer, CopyDesc, DetailTimer, Library, Params, SolverCallback, SolverCallbackStage,
    SolverDesc, Timers,
};
use nvflex_sys::*;
use std::any::Any;

/// Initialize the solver desc to its default values
///
/// # Parameters
///
/// * `desc` - Mutable reference to a description structure that will be initialized to default values
#[inline]
pub fn set_solver_desc_defaults(desc: &mut SolverDesc) {
    unsafe { NvFlexSetSolverDescDefaults(desc as *mut _ as *mut _) }
}

/// Easy way of generating `NvFlexGet.*` / `NvFlexSet.*` method wrappers
macro_rules! get_set_method {
    ($name:ident, $solver:expr, $buf:expr, $desc:expr) => {{
        unsafe {
            $name(
                $solver,
                $buf,
                if $desc.is_some() {
                    &$desc.unwrap_unchecked() as *const _ as *const _
                } else {
                    std::ptr::null()
                },
            )
        }
    }};
}

/// Opaque type representing a collection of particles and constraints
#[derive(Debug, Clone)]
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

    /// Set the active particles indices in the solver
    ///
    /// # Parameters
    ///
    /// * `indices` - Holds the indices of particles that have been made active
    /// * `desc` - Describes the copy region, if [`None`] the solver will try to access the entire buffer (`max_particles` length)
    #[inline]
    pub fn set_active<T: Any>(&self, indices: &Buffer<T>, desc: Option<CopyDesc>) {
        get_set_method!(NvFlexSetActive, self.solver, indices.buf, desc);
    }

    /// Set the active particles indices in the solver
    ///
    /// # Parameters
    ///
    /// * `indices` - Holds the indices of particles that have been made active
    /// * `desc` - Describes the copy region, if [`None`] the solver will try to access the entire buffer (`max_particles` length)
    #[inline]
    pub fn get_active<T: Any>(&self, indices: &Buffer<T>, desc: Option<CopyDesc>) {
        get_set_method!(NvFlexGetActive, self.solver, indices.buf, desc);
    }

    /// Set the total number of active particles
    ///
    /// # Parameters
    ///
    /// * `n` - The number of active particles, the first `n` indices in the active particles array will be used as the active count
    #[inline]
    pub fn set_active_count(&self, n: i32) {
        unsafe { NvFlexSetActiveCount(self.solver, n) };
    }

    /// Return the number of active particles in the solver
    ///
    /// # Returns
    ///
    /// The number of active particles in the solver
    #[inline]
    pub fn get_active_count(&self) -> i32 {
        unsafe { NvFlexGetActiveCount(self.solver) }
    }

    /// Set the particles state of the solver, a particle consists of 4 floating point numbers, its x,y,z position followed by its inverse mass (1/m)
    ///
    /// # Parameters
    ///
    /// * `p` - A buffer of particle data, should be 4*n in length
    /// * `desc` - Describes the copy region, if [`None`] the solver will try to access the entire buffer (`max_particles` length)
    #[inline]
    pub fn set_particles<T: Any>(&self, p: &Buffer<T>, desc: Option<CopyDesc>) {
        get_set_method!(NvFlexSetParticles, self.solver, p.buf, desc);
    }

    /// Get the particles state of the solver, a particle consists of 4 floating point numbers, its x,y,z position followed by its inverse mass (1/m)
    ///
    /// # Parameters
    ///
    /// * `p` - A buffer of 4*n floats that will be filled out with the particle data, can be either a host or device pointer
    /// * `desc` - Describes the copy region, if [`None`] the solver will try to access the entire buffer (`max_particles` length)
    #[inline]
    pub fn get_particles<T: Any>(&self, p: &Buffer<T>, desc: Option<CopyDesc>) {
        get_set_method!(NvFlexGetParticles, self.solver, p.buf, desc);
    }

    /// Set the particle positions in their rest state, if eNvFlexPhaseSelfCollideFilter is set on the particle's
    /// phase attribute then particles that overlap in the rest state will not generate collisions with each other
    ///
    /// # Parameters
    ///
    /// * `p` - A buffer of particle data, should be 4*n in length
    /// * `desc` - Describes the copy region, if [`None`] the solver will try to access the entire buffer (`max_particles` length)
    #[inline]
    pub fn set_rest_particles<T: Any>(&self, p: &Buffer<T>, desc: Option<CopyDesc>) {
        get_set_method!(NvFlexSetRestParticles, self.solver, p.buf, desc);
    }

    /// Get the particle positions in their rest state
    ///
    /// # Parameters
    ///
    /// * `p` - A buffer of particle data, should be 4*n in length
    /// * `desc` - Describes the copy region, if [`None`] the solver will try to access the entire buffer (`max_particles` length)
    #[inline]
    pub fn get_rest_particles<T: Any>(&self, p: &Buffer<T>, desc: Option<CopyDesc>) {
        get_set_method!(NvFlexGetRestParticles, self.solver, p.buf, desc);
    }

    /// Get the Laplacian smoothed particle positions for rendering, see [`Params::smoothing`]
    ///
    /// # Parameters
    ///
    /// * `p` - A buffer of 4*n floats that will be filled out with the data, can be either a host or device pointer
    /// * `desc` - Describes the copy region, if [`None`] the solver will try to access the entire buffer (`max_particles` length)
    #[inline]
    pub fn get_smooth_particles<T: Any>(&self, p: &Buffer<T>, desc: Option<CopyDesc>) {
        get_set_method!(NvFlexGetSmoothParticles, self.solver, p.buf, desc);
    }

    /// Set the particle velocities, each velocity is a 3-tuple of x,y,z floating point values
    ///
    /// # Parameters
    ///
    /// * `v` - Pointer to a buffer of 3*n floats
    /// * `desc` - Describes the copy region, if [`None`] the solver will try to access the entire buffer (`max_particles` length)
    #[inline]
    pub fn set_velocities<T: Any>(&self, v: &Buffer<T>, desc: Option<CopyDesc>) {
        get_set_method!(NvFlexSetVelocities, self.solver, v.buf, desc);
    }

    /// Get the particle velocities, each velocity is a 3-tuple of x,y,z floating point values
    ///
    /// # Parameters
    ///
    /// * `v` - Pointer to a buffer of 3*n floats that will be filled out with the data, can be either a host or device pointer
    /// * `desc` - Describes the copy region, if [`None`] the solver will try to access the entire buffer (`max_particles` length)
    #[inline]
    pub fn get_velocities<T: Any>(&self, v: &Buffer<T>, desc: Option<CopyDesc>) {
        get_set_method!(NvFlexGetVelocities, self.solver, v.buf, desc);
    }

    /// Set the particles phase id array, each particle has an associated phase id which
    /// controls how it interacts with other particles. Particles with phase 0 interact with all
    /// other phase types.
    ///
    /// Particles with a non-zero phase id only interact with particles whose phase differs
    /// from theirs. This is useful, for example, to stop particles belonging to a single
    /// rigid shape from interacting with each other.
    ///
    /// Phase 0 is used to indicate fluid particles when [`Params::mFluid`] is set. (?)
    ///
    /// # Parameters
    ///
    /// * `phases` - Pointer to a buffer of n integers containing the phases
    /// * `desc` - Describes the copy region, if [`None`] the solver will try to access the entire buffer (`max_particles` length)
    #[inline]
    pub fn set_phases<T: Any>(&self, phases: &Buffer<T>, desc: Option<CopyDesc>) {
        get_set_method!(NvFlexSetPhases, self.solver, phases.buf, desc);
    }

    /// Set the particles phase id array, each particle has an associated phase id which
    /// controls how it interacts with other particles. Particles with phase 0 interact with all
    /// other phase types.
    ///
    /// Particles with a non-zero phase id only interact with particles whose phase differs
    /// from theirs. This is useful, for example, to stop particles belonging to a single
    /// rigid shape from interacting with each other.
    ///
    /// Phase 0 is used to indicate fluid particles when [`Params::mFluid`] is set. (?)
    ///
    /// # Parameters
    ///
    /// * `phases` - Pointer to a buffer of n integers containing the phases
    /// * `desc` - Describes the copy region, if [`None`] the solver will try to access the entire buffer (`max_particles` length)
    #[inline]
    pub fn get_phases<T: Any>(&self, phases: &Buffer<T>, desc: Option<CopyDesc>) {
        get_set_method!(NvFlexGetPhases, self.solver, phases.buf, desc);
    }

    /// Set per-particle normals to the solver, these will be overwritten after each simulation step, but can be used to initialize the normals to valid values
    ///
    /// # Parameters
    ///
    /// * `normals` - Pointer to a buffer of normals, should be 4*n in length
    /// * `desc` - Describes the copy region, if [`None`] the solver will try to access the entire buffer (`max_particles` length)
    #[inline]
    pub fn set_normals<T: Any>(&self, normals: &Buffer<T>, desc: Option<CopyDesc>) {
        get_set_method!(NvFlexSetNormals, self.solver, normals.buf, desc);
    }

    /// Get per-particle normals from the solver, these are the world-space normals computed during surface tension, cloth, and rigid body calculations
    ///
    /// # Parameters
    ///
    /// * `normals` - Pointer to a buffer of normals, should be 4*n in length
    /// * `desc` - Describes the copy region, if [`None`] the solver will try to access the entire buffer (`max_particles` length)
    #[inline]
    pub fn get_normals<T: Any>(&self, normals: &Buffer<T>, desc: Option<CopyDesc>) {
        get_set_method!(NvFlexGetNormals, self.solver, normals.buf, desc);
    }

    /// Set distance constraints for the solver. Each distance constraint consists of two particle indices
    /// stored consecutively, a rest-length, and a stiffness value. These are not springs in the traditional
    /// sense, but behave somewhat like a traditional spring when lowering the stiffness coefficient.
    ///
    /// # Parameters
    ///
    /// * `indices` - Pointer to the spring indices array, should be 2*num_springs length, 2 indices per-spring
    /// * `rest_lengths` - Pointer to a buffer of rest lengths, should be num_springs length
    /// * `stiffness` - Pointer to the spring stiffness coefficents, should be num_springs in length, a negative stiffness value represents a tether constraint
    /// * `num_springs` - The number of springs to set
    #[inline]
    pub fn set_springs<T: Any>(
        &self,
        indices: &Buffer<T>,
        rest_lengths: &Buffer<T>,
        stiffness: &Buffer<T>,
        num_springs: i32,
    ) {
        unsafe {
            NvFlexSetSprings(
                self.solver,
                indices.buf,
                rest_lengths.buf,
                stiffness.buf,
                num_springs,
            )
        };
    }

    /// Get the distance constraints from the solver
    ///
    /// # Parameters
    ///
    /// * `indices` - Pointer to the spring indices array, should be 2*num_springs length, 2 indices per-spring
    /// * `rest_lengths` - Pointer to a buffer of rest lengths, should be num_springs length
    /// * `stiffness` - Pointer to the spring stiffness coefficents, should be num_springs in length, a negative stiffness value represents a unilateral tether constraint (only resists stretching, not compression), valid range [-1, 1]
    /// * `num_springs` - The number of springs to get
    #[inline]
    pub fn get_springs<T: Any>(
        &self,
        indices: &Buffer<T>,
        rest_lengths: &Buffer<T>,
        stiffness: &Buffer<T>,
        num_springs: i32,
    ) {
        unsafe {
            NvFlexGetSprings(
                self.solver,
                indices.buf,
                rest_lengths.buf,
                stiffness.buf,
                num_springs,
            )
        };
    }

    /// Set rigid body constraints for the solver.
    ///
    /// ### NOTE: A particle should not belong to more than one rigid body at a time.
    ///
    /// # Parameters
    ///
    /// * `offsets` - Pointer to a buffer of start offsets for a rigid in the indices array, should be `num_rigids`+1 in length, the first entry must be 0
    /// * `indices` - Pointer to a buffer of indices for the rigid bodies, the indices for the jth rigid body start at `indices[offsets[j]]` and run to `indices[offsets[j+1]]` exclusive
    /// * `rest_positions` - Pointer to a buffer of local space positions relative to the rigid's center of mass (average position), this should be at least 3*`num_indices` in length in the format x,y,z
    /// * `rest_normals` - Pointer to a buffer of local space normals, this should be at least 4*`num_indices` in length in the format x,y,z,w where w is the (negative) signed distance of the particle inside its shape
    /// * `stiffness` - Pointer to a buffer of rigid stiffness coefficents, should be `num_rigids` in length, valid values in range `[0, 1]`
    /// * `thresholds` - Pointer to a buffer of plastic deformation threshold coefficients, should be `num_rigids` in length
    /// * `creeps` - Pointer to a buffer of plastic deformation creep coefficients, should be `num_rigids` in length, valid values in range `[0, 1]`
    /// * `rotations` - Pointer to a buffer of quaternions (4*`num_rigids` in length)
    /// * `translations` - Pointer to a buffer of translations of the center of mass (3*`num_rigids` in length)
    /// * `num_rigids` - The number of rigid bodies to set
    /// * `num_indices` - The number of indices in the indices array
    #[inline]
    pub fn set_rigids<T: Any>(
        &self,
        offsets: &Buffer<T>,
        indices: &Buffer<T>,
        rest_positions: &Buffer<T>,
        rest_normals: &Buffer<T>,
        stiffness: &Buffer<T>,
        thresholds: &Buffer<T>,
        creeps: &Buffer<T>,
        rotations: &Buffer<T>,
        translations: &Buffer<T>,
        num_rigids: i32,
        num_indices: i32,
    ) {
        unsafe {
            NvFlexSetRigids(
                self.solver,
                offsets.buf,
                indices.buf,
                rest_positions.buf,
                rest_normals.buf,
                stiffness.buf,
                thresholds.buf,
                creeps.buf,
                rotations.buf,
                translations.buf,
                num_rigids,
                num_indices,
            )
        };
    }

    /// Retrive the rigid body shape matching constraints and transforms, if any buffer pointers are NULL then they will be ignored
    /// This method supersedes the previous NvFlexGetRigidTransforms method and can be used to retrieve modified rest positions from plastic deformation.
    ///
    /// # Parameters
    ///
    /// * `offsets` - Pointer to a buffer of start offsets for a rigid in the indices array, should be `num_rigids`+1 in length, the first entry must be 0
    /// * `indices` - Pointer to a buffer of indices for the rigid bodies, the indices for the jth rigid body start at `indices[offsets[j]]` and run to `indices[offsets[j+1]]` exclusive
    /// * `rest_positions` - Pointer to a buffer of local space positions relative to the rigid's center of mass (average position), this should be at least 3*`numIndices` in length in the format x,y,z
    /// * `rest_normals` - Pointer to a buffer of local space normals, this should be at least 4*`numIndices` in length in the format x,y,z,w where w is the (negative) signed distance of the particle inside its shape
    /// * `stiffness` - Pointer to a buffer of rigid stiffness coefficents, should be `num_rigids` in length, valid values in range `[0, 1]`
    /// * `thresholds` - Pointer to a buffer of plastic deformation threshold coefficients, should be `num_rigids` in length
    /// * `creeps` - Pointer to a buffer of plastic deformation creep coefficients, should be `num_rigids` in length, valid values in range `[0, 1]`
    /// * `rotations` - Pointer to a buffer of quaternions (4*`num_rigids` in length with the imaginary elements in the x,y,z components)
    /// * `translations` - Pointer to a buffer of translations of the center of mass (3*`num_rigids` in length)
    #[inline]
    pub fn get_rigids<T: Any>(
        &self,
        offsets: &Buffer<T>,
        indices: &Buffer<T>,
        rest_positions: &Buffer<T>,
        rest_normals: &Buffer<T>,
        stiffness: &Buffer<T>,
        thresholds: &Buffer<T>,
        creeps: &Buffer<T>,
        rotations: &Buffer<T>,
        translations: &Buffer<T>,
    ) {
        unsafe {
            NvFlexGetRigids(
                self.solver,
                offsets.buf,
                indices.buf,
                rest_positions.buf,
                rest_normals.buf,
                stiffness.buf,
                thresholds.buf,
                creeps.buf,
                rotations.buf,
                translations.buf,
            )
        };
    }

    /// Set the collision shapes for the solver
    ///
    /// # Parameters
    ///
    /// * `geometry` - Pointer to a buffer of [`crate::CollisionGeometry`] entries, the type of each shape determines how many entries it has in the array
    /// * `shape_positions` - Pointer to a buffer of translations for each shape in world space, should be 4*num_shapes in length
    /// * `shape_rotations` - Pointer to an a buffer of rotations for each shape stored as quaternion, should be 4*num_shapes in length
    /// * `shape_prev_positions` - Pointer to a buffer of translations for each shape at the start of the time step, should be 4*num_shapes in length
    /// * `shape_prev_rotations` - Pointer to an a buffer of rotations for each shape stored as a quaternion at the start of the time step, should be 4*num_shapes in length
    /// * `shape_flags` - The type and behavior of the shape, [`crate::CollisionShapeFlags`] for more detail
    /// * `num_shapes` - The number of shapes
    #[inline]
    pub fn set_shapes<T: Any>(
        &self,
        geometry: &Buffer<T>,
        shape_positions: &Buffer<T>,
        shape_rotations: &Buffer<T>,
        shape_prev_positions: &Buffer<T>,
        shape_prev_rotations: &Buffer<T>,
        shape_flags: &Buffer<T>,
        num_shapes: i32,
    ) {
        unsafe {
            NvFlexSetShapes(
                self.solver,
                geometry.buf,
                shape_positions.buf,
                shape_rotations.buf,
                shape_prev_positions.buf,
                shape_prev_rotations.buf,
                shape_flags.buf,
                num_shapes,
            )
        }
    }

    /// Set dynamic triangles mesh indices, typically used for cloth. Flex will calculate normals and
    /// apply wind and drag effects to connected particles. See [`Params::drag`], [`Params::wind`].
    ///
    /// # Parameters
    ///
    /// * `indices` - Pointer to a buffer of triangle indices into the particles array, should be 3*num_tris in length
    /// * `normals` - Pointer to a buffer of triangle normals, should be 3*num_tris in length, can be [`None`]
    /// * `num_tris` - The number of dynamic triangles
    #[inline]
    pub fn set_dynamic_triangles<T: Any>(
        &self,
        indices: &Buffer<T>,
        normals: Option<&Buffer<T>>,
        num_tris: i32,
    ) {
        unsafe {
            NvFlexSetDynamicTriangles(
                self.solver,
                indices.buf,
                if normals.is_some() {
                    normals.unwrap_unchecked().buf
                } else {
                    std::ptr::null_mut()
                },
                num_tris,
            )
        };
    }

    /// Get the dynamic triangle indices and normals.
    ///
    /// # Parameters
    ///
    /// * `indices` - Pointer to a buffer of triangle indices into the particles array, should be 3*num_tris in length, if [`None`] indices will not be returned
    /// * `normals` - Pointer to a buffer of triangle normals, should be 3*num_tris in length, if [`None`] normals will be not be returned
    /// * `num_tris` - The number of dynamic triangles
    #[inline]
    pub fn get_dynamic_triangles<T: Any>(
        &self,
        indices: Option<&Buffer<T>>,
        normals: Option<&Buffer<T>>,
        num_tris: i32,
    ) {
        unsafe {
            NvFlexGetDynamicTriangles(
                self.solver,
                if indices.is_some() {
                    indices.unwrap_unchecked().buf
                } else {
                    std::ptr::null_mut()
                },
                if normals.is_some() {
                    normals.unwrap_unchecked().buf
                } else {
                    std::ptr::null_mut()
                },
                num_tris,
            )
        };
    }

    /// Set inflatable shapes, an inflatable is a range of dynamic triangles (wound CCW) that represent a closed mesh.
    /// Each inflatable has a given rest volume, constraint scale (roughly equivalent to stiffness), and "over pressure"
    /// that controls how much the shape is inflated.
    ///
    /// # Parameters
    ///
    /// * `start_tris` - Pointer to a buffer of offsets into the solver's dynamic triangles for each inflatable, should be num_inflatables in length
    /// * `num_tris` - Pointer to a buffer of triangle counts for each inflatable, should be num_inflatables in length
    /// * `rest_volumes` - Pointer to a buffer of rest volumes for the inflatables, should be num_inflatables in length
    /// * `over_pressures` - Pointer to a buffer of floats specifying the pressures for each inflatable, a value of 1.0 means the rest volume, > 1.0 means over-inflated, and < 1.0 means under-inflated, should be num_inflatables in length
    /// * `constraint_scales` - Pointer to a buffer of scaling factors for the constraint, this is roughly equivalent to stiffness but includes a constraint scaling factor from position-based dynamics, see helper code for details, should be num_inflatables in length
    /// * `num_inflatables` - Number of inflatables to set
    #[inline]
    pub fn set_inflatables<T: Any>(
        &self,
        start_tris: &Buffer<T>,
        num_tris: &Buffer<T>,
        rest_volumes: &Buffer<T>,
        over_pressures: &Buffer<T>,
        constraint_scales: &Buffer<T>,
        num_inflatables: i32,
    ) {
        unsafe {
            NvFlexSetInflatables(
                self.solver,
                start_tris.buf,
                num_tris.buf,
                rest_volumes.buf,
                over_pressures.buf,
                constraint_scales.buf,
                num_inflatables,
            )
        };
    }

    /// Get the density values for fluid particles
    ///
    /// # Parameters
    ///
    /// * `densities` - Pointer to a buffer of floats, should be `max_particles` in length, density values are normalized between `[0, 1]` where 1 represents the rest density
    /// * `desc` - A descriptor specifying the contents to read back, can be [`None`]
    #[inline]
    pub fn get_densities<T: Any>(&self, densities: &Buffer<T>, desc: Option<CopyDesc>) {
        unsafe {
            NvFlexGetDensities(
                self.solver,
                densities.buf,
                if desc.is_some() {
                    &desc.unwrap_unchecked() as *const _ as *const _
                } else {
                    std::ptr::null()
                },
            )
        };
    }

    /// Get the anisotropy of fluid particles, the particle distribution for a particle is represented
    /// by 3 orthogonal vectors. Each 3-vector has unit length with the variance along that axis
    /// packed into the w component, i.e.: x,y,z,lambda.
    /// The anisotropy defines an oriented ellipsoid in worldspace that can be used for rendering
    /// or surface extraction.
    ///
    /// # Parameters
    ///
    /// * `q1` - Pointer to a buffer of floats that receive the first basis vector and scale, should be 4*`max_particles` in length
    /// * `q2` - Pointer to a buffer of floats that receive the second basis vector and scale, should be 4*`max_particles` in length
    /// * `q3` - Pointer to a buffer of floats that receive the third basis vector and scale, should be 4*`max_particles` in length
    /// * `desc` - A descriptor specifying the contents to read back, can be [`None`]
    #[inline]
    pub fn get_anisotropy<T: Any>(
        &self,
        q1: &Buffer<T>,
        q2: &Buffer<T>,
        q3: &Buffer<T>,
        desc: Option<CopyDesc>,
    ) {
        unsafe {
            NvFlexGetAnisotropy(
                self.solver,
                q1.buf,
                q2.buf,
                q3.buf,
                if desc.is_some() {
                    &desc.unwrap_unchecked() as *const _ as *const _
                } else {
                    std::ptr::null()
                },
            )
        };
    }

    /// Get the state of the diffuse particles. Diffuse particles are passively advected by the fluid
    /// velocity field.
    ///
    /// # Parameters
    ///
    /// * `p` - Pointer to a buffer of floats, should be 4*`max_particles` in length, the w component represents the particles lifetime with 1 representing a new particle, and 0 representing an inactive particle
    /// * `v` - Pointer to a buffer of floats, should be 4*`max_particles` in length, the w component is not used
    /// * `count` - Pointer to a buffer of a single int that holds the current particle count (this may be updated by the GPU which is why it is passed back in a buffer)
    #[inline]
    pub fn get_diffuse_particles<T: Any>(&self, p: &Buffer<T>, v: &Buffer<T>, count: &Buffer<T>) {
        unsafe { NvFlexGetDiffuseParticles(self.solver, p.buf, v.buf, count.buf) };
    }

    /// Set the state of the diffuse particles. Diffuse particles are passively advected by the fluid
    /// velocity field.
    ///
    /// # Parameters
    ///
    /// * `p` - Pointer to a buffer of floats, should be 4*n in length, the w component represents the particles lifetime with 1 representing a new particle, and 0 representing an inactive particle
    /// * `v` - Pointer to a buffer of floats, should be 4*n in length, the w component is not used
    /// * `n` - The number of active diffuse particles
    #[inline]
    pub fn set_diffuse_particles<T: Any>(&self, p: &Buffer<T>, v: &Buffer<T>, n: i32) {
        unsafe { NvFlexSetDiffuseParticles(self.solver, p.buf, v.buf, n) };
    }

    /// Get the particle contact planes. Note this will only include contacts that were active on the last substep of an update, and will include all contact planes generated within [`Params::shape_collision_margin`].
    ///
    /// # Parameters
    ///
    /// * `planes` - Pointer to a destination buffer containing the contact planes for the particle, each particle can have up to `max_contacts_per_particle` contact planes (see [`SolverDesc`]) so this buffer should be 4*`max_contacts_per_particle`*`max_particles` floats in length
    /// * `velocities` - Pointer to a destination buffer containing the velocity of the contact point on the shape in world space, the index of the shape (corresponding to the shape in `set_shapes()` is stored in the w component), each particle can have up to `max_contacts_per_particle` contact planes so this buffer should be 4*`max_contacts_per_particle`*`max_particles` floats in length
    /// * `indices` - Pointer to a buffer of indices into the contacts buffer, the first contact plane for the i'th particle is given by `planes[indices[i]*sizeof(float)*4*max_contacts_per_particle]` and subsequent contact planes for that particle are stored sequentially, this array should be `max_particles` in length
    /// * `counts` - Pointer to a buffer of contact counts for each particle (will be <= `max_contacts_per_particle`), this buffer should be `max_particles` in length
    #[inline]
    pub fn get_contacts<T: Any>(
        &self,
        planes: &Buffer<T>,
        velocities: &Buffer<T>,
        indices: &Buffer<T>,
        counts: &Buffer<T>,
    ) {
        unsafe {
            NvFlexGetContacts(
                self.solver,
                planes.buf,
                velocities.buf,
                indices.buf,
                counts.buf,
            )
        };
    }

    // TODO: translate the C code example to Rust
    /// Get the particle neighbor lists, these are stored in a strided format, and can be iterated in the following manner:
    ///
    /// ```c
    /// NvFlexGetNeighbors(solver, neighborsBuffer, countsBuffer, apiToInternalBuffer, internalToApiBuffer);
    /// int* neighbors = (int*)NvFlexMap(neighborsBuffer, 0);
    /// int* counts = (int*)NvFlexMap(countsBuffer, 0);
    /// int* apiToInternal = (int*)NvFlexMap(apiToInternalBuffer, 0);
    /// int* internalToApi = (int*)NvFlexMap(internalToApiBuffer, 0);
    /// // neighbors are stored in a strided format so that the first neighbor
    /// // of each particle is stored sequentially, then the second, and so on
    ///     
    /// int stride = maxParticles;
    /// for (int i=0; i < maxParticles; ++i)
    /// {
    ///     // find offset in the neighbors buffer
    ///     int offset = apiToInternal[i];
    ///     int count = counts[offset];
    ///     for (int c=0; c < count; ++c)
    ///     {
    ///         int neighbor = internalToApi[neighbors[c*stride + offset]];
    ///         printf("Particle %d's neighbor %d is particle %d\n", i, c, neighbor);
    ///     }
    /// }
    /// NvFlexUnmap(neighborsBuffer);
    /// NvFlexUnmap(countsBuffer);
    /// NvFlexUnmap(apiToInternalBuffer);
    /// NvFlexUnmap(internalToApiBuffer);
    /// ```
    ///
    /// # Parameters
    ///
    /// * `neighbors` - Pointer to a destination buffer containing the the neighbors for all particles, this should be `max_particles`*`max_particle_neighbors` ints (passed to NvFlexInit() in length)
    /// * `counts` - Pointer to a buffer of neighbor counts per-particle, should be `max_particles` ints in length
    /// * `api_to_internal` - Pointer to a buffer of indices, because Flex internally re-orders particles these are used to map from an API particle index to it internal index
    /// * `internal_to_api` - Pointer to a buffer of indices, because Flex internally re-orders particles these are used to map from an internal index to an API index
    /// ### NOTE: Neighbors are only valid after a call to NvFlexUpdateSolver() has completed, the returned neighbors correspond to the last substep of the last update
    #[inline]
    pub fn get_neighbors<T: Any>(
        &self,
        neighbors: &Buffer<T>,
        counts: &Buffer<T>,
        api_to_internal: &Buffer<T>,
        internal_to_api: &Buffer<T>,
    ) {
        unsafe {
            NvFlexGetNeighbors(
                self.solver,
                neighbors.buf,
                counts.buf,
                api_to_internal.buf,
                internal_to_api.buf,
            )
        };
    }

    /// Get the world space AABB of all particles in the solver, note that the bounds are calculated during the update (see NvFlexUpdateSolver()) so only become valid after an update has been performed.
    /// The returned bounds represent bounds of the particles in their predicted positions *before* the constraint solve.
    ///
    /// # Parameters
    ///
    /// * `lower` - Buffer of 3 floats to receive the lower bounds
    /// * `upper` - Buffer of 3 floats to receive the upper bounds
    #[inline]
    pub fn get_bounds<T: Any>(&self, lower: &Buffer<T>, upper: &Buffer<T>) {
        unsafe { NvFlexGetBounds(self.solver, lower.buf, upper.buf) };
    }

    /// # Parameters
    ///
    /// * `begin` - Optional reference to a 64 bit unsigned to receive the value of the GPU clock when Flex update began (in cycles)
    /// * `end` - Optional reference to a 64 bit unsigned to receive the value of the GPU clock when Flex update ended (in cycles)
    /// * `frequency` - Optional reference to a 64 bit unsigned to receive the frequency of the clock used to measure begin and end
    ///
    /// # Returns
    ///
    /// The time in seconds between the first and last GPU operations executed by the last NvFlexUpdateSolver.
    ///
    /// ### NOTE: This method causes the CPU to wait until the GPU has finished any outstanding work. To avoid blocking the calling thread it should be called after work has completed, e.g.: directly after a NvFlexMap().
    #[inline]
    pub fn get_device_latency(
        &self,
        begin: Option<&mut u64>,
        end: Option<&mut u64>,
        frequency: Option<&mut u64>,
    ) -> f32 {
        unsafe {
            NvFlexGetDeviceLatency(
                self.solver,
                if begin.is_some() {
                    begin.unwrap_unchecked() as *mut _
                } else {
                    std::ptr::null_mut()
                },
                if end.is_some() {
                    end.unwrap_unchecked() as *mut _
                } else {
                    std::ptr::null_mut()
                },
                if frequency.is_some() {
                    frequency.unwrap_unchecked() as *mut _
                } else {
                    std::ptr::null_mut()
                },
            )
        }
    }

    /// Fetch high-level GPU timers.
    ///
    /// # Parameters
    ///
    /// * `solver` - The solver instance to use
    /// * `timers` - A struct containing the GPU latency of each stage in the physics pipeline.
    /// ### NOTE: This method causes the CPU to wait until the GPU has finished any outstanding work. To avoid blocking the calling thread it should be called after work has completed, e.g.: directly after a NvFlexMap(). To capture there timers you must pass true for `enable_timers` in NvFlexUpdateSolver()
    #[inline]
    pub fn get_timers(&self) -> Timers {
        unsafe {
            let mut timers = std::mem::zeroed::<Timers>();
            NvFlexGetTimers(self.solver, &mut timers as *mut _ as *mut _);
            timers
        }
    }

    /// Fetch per-shader GPU timers.
    ///
    /// # Parameters
    ///
    /// * `solver` - The solver instance to use
    /// * `timers` - An array of NvFlexDetailTimer structures, each representing a unique shader.
    ///
    /// # Returns
    ///
    /// The timers array and the number of detail timers in the timers array
    ///
    /// ### NOTE: This method causes the CPU to wait until the GPU has finished any outstanding work. To avoid blocking the calling thread it should be called after work has completed, e.g.: directly after a NvFlexMap(). To capture there timers you must pass true for `enable_timers` in NvFlexUpdateSolver(). Timers are valid until the next call to NvFlexGetDetailTimers
    pub fn get_detail_timers(&self, timers: &mut [DetailTimer]) {
        unsafe {
            let mut detail_timers: Vec<*mut NvFlexDetailTimer> =
                vec![std::ptr::null_mut(); timers.len()];
            NvFlexGetDetailTimers(self.solver, detail_timers.as_mut_ptr());

            let mut i = 0;
            for timer in detail_timers {
                timers[i] = DetailTimer {
                    name: rstr!((*timer).name).to_string(),
                    time: (*timer).time,
                };
                i += 1;
            }
        }
    }

    /// Debug method (unsupported)
    #[inline]
    pub fn set_debug(&self, enable: bool) {
        unsafe { NvFlexSetDebug(self.solver, enable) };
    }

    // TODO: add other debug methods
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
