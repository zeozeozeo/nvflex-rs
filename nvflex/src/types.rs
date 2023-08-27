use std::ffi::c_void;

use bitflags::bitflags;
use nvflex_sys::NvFlexSetSolverDescDefaults;

/// least 2 significant digits define minor version, eg: 10 -> version 0.10
pub const VERSION: u32 = nvflex_sys::NV_FLEX_VERSION;

/// Controls behavior of NvFlexMap()
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(C)]
pub enum MapFlags {
    /// Calling thread will be blocked until buffer is ready for access, default
    Wait = 0,
    /// Calling thread will check if buffer is ready for access, if not ready then the method will return NULL immediately
    DoNotWait = 1,
}

/// Controls memory space of a NvFlexBuffer, see NvFlexAllocBuffer()
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(C)]
pub enum BufferType {
    /// A host mappable buffer, pinned memory on CUDA, staging buffer on DX
    Host = 0,
    /// A device memory buffer, mapping this on CUDA will return a device memory pointer, and will return a buffer pointer on DX
    Device = 1,
}

/// Controls the relaxation method used by the solver to ensure convergence
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(C)]
pub enum RelaxationMode {
    /// The relaxation factor is a fixed multiplier on each constraint's position delta
    Global = 0,
    /// The relaxation factor is a fixed multiplier on each constraint's delta divided by the particle's constraint count, convergence will be slower but more reliable
    Local = 1,
}

/// Simulation parameters for a solver
#[derive(Copy, Clone, PartialEq, Debug)]
#[repr(C)]
pub struct Params {
    /// Number of solver iterations to perform per-substep
    pub num_iterations: i32,
    /// Constant acceleration applied to all particles
    pub gravity: [f32; 3usize],
    /// The maximum interaction radius for particles
    pub radius: f32,
    /// The distance non-fluid particles attempt to maintain from each other, must be in the range (0, radius]
    pub solid_rest_distance: f32,
    /// The distance fluid particles are spaced at the rest density, must be in the range (0, radius], for fluids this should generally be 50-70% of mRadius, for rigids this can simply be the same as the particle radius
    pub fluid_rest_distance: f32,
    /// Coefficient of friction used when colliding against shapes
    pub dynamic_friction: f32,
    /// Coefficient of static friction used when colliding against shapes
    pub static_friction: f32,
    /// Coefficient of friction used when colliding particles
    pub particle_friction: f32,
    /// Coefficient of restitution used when colliding against shapes, particle collisions are always inelastic
    pub restitution: f32,
    /// Controls how strongly particles stick to surfaces they hit, default 0.0, range [0.0, +inf]
    pub adhesion: f32,
    /// Particles with a velocity magnitude < this threshold will be considered fixed
    pub sleep_threshold: f32,
    /// The magnitude of particle velocity will be clamped to this value at the end of each step
    pub max_speed: f32,
    /// The magnitude of particle acceleration will be clamped to this value at the end of each step (limits max velocity change per-second), useful to avoid popping due to large interpenetrations
    pub max_acceleration: f32,
    /// Artificially decrease the mass of particles based on height from a fixed reference point, this makes stacks and piles converge faster
    pub shock_propagation: f32,
    /// Damps particle velocity based on how many particle contacts it has
    pub dissipation: f32,
    /// Viscous drag force, applies a force proportional, and opposite to the particle velocity
    pub damping: f32,
    /// Constant acceleration applied to particles that belong to dynamic triangles, drag needs to be > 0 for wind to affect triangles
    pub wind: [f32; 3usize],
    /// Drag force applied to particles belonging to dynamic triangles, proportional to velocity^2*area in the negative velocity direction
    pub drag: f32,
    /// Lift force applied to particles belonging to dynamic triangles, proportional to velocity^2*area in the direction perpendicular to velocity and (if possible), parallel to the plane normal
    pub lift: f32,
    /// Control how strongly particles hold each other together, default: 0.025, range [0.0, +inf]
    pub cohesion: f32,
    /// Controls how strongly particles attempt to minimize surface area, default: 0.0, range: [0.0, +inf]    
    pub surface_tension: f32,
    /// Smoothes particle velocities using XSPH viscosity
    pub viscosity: f32,
    /// Increases vorticity by applying rotational forces to particles
    pub vorticity_confinement: f32,
    /// Control how much anisotropy is present in resulting ellipsoids for rendering, if zero then anisotropy will not be calculated, see NvFlexGetAnisotropy()
    pub anisotropy_scale: f32,
    /// Clamp the anisotropy scale to this fraction of the radius
    pub anisotropy_min: f32,
    /// Clamp the anisotropy scale to this fraction of the radius
    pub anisotropy_max: f32,
    /// Control the strength of Laplacian smoothing in particles for rendering, if zero then smoothed positions will not be calculated, see NvFlexGetSmoothParticles()
    pub smoothing: f32,
    /// Add pressure from solid surfaces to particles
    pub solid_pressure: f32,
    /// Drag force applied to boundary fluid particles
    pub free_surface_drag: f32,
    /// Gravity is scaled by this value for fluid particles
    pub buoyancy: f32,
    /// Particles with kinetic energy + divergence above this threshold will spawn new diffuse particles
    pub diffuse_threshold: f32,
    /// Scales force opposing gravity that diffuse particles receive
    pub diffuse_buoyancy: f32,
    /// Scales force diffuse particles receive in direction of neighbor fluid particles
    pub diffuse_drag: f32,
    /// The number of neighbors below which a diffuse particle is considered ballistic
    pub diffuse_ballistic: i32,
    /// Time in seconds that a diffuse particle will live for after being spawned, particles will be spawned with a random lifetime in the range [0, diffuseLifetime]
    pub diffuse_lifetime: f32,
    /// Distance particles maintain against shapes, note that for robust collision against triangle meshes this distance should be greater than zero
    pub collision_distance: f32,
    /// Increases the radius used during neighbor finding, this is useful if particles are expected to move significantly during a single step to ensure contacts aren't missed on subsequent iterations
    pub particle_collision_margin: f32,
    /// Increases the radius used during contact finding against kinematic shapes
    pub shape_collision_margin: f32,
    /// Collision planes in the form ax + by + cz + d = 0
    pub planes: [[f32; 4usize]; 8usize],
    /// Num collision planes
    pub num_planes: i32,
    /// How the relaxation is applied inside the solver
    pub relaxation_mode: RelaxationMode,
    /// Control the convergence rate of the parallel solver, default: 1, values greater than 1 may lead to instability
    pub relaxation_factor: f32,
}

bitflags! {
    /// Flags that control a particle's behavior and grouping, use NvFlexMakePhase() to construct a valid 32bit phase identifier
    #[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
    #[repr(C)]
    pub struct Phase: i32 {
        /// Bits [ 0, 19] represent the particle group for controlling collisions
        const GroupMask = 0x000fffff;
        /// Bits [20, 23] hold flags about how the particle behave
        const FlagsMask = 0x00f00000;
        /// Bits [24, 30] hold flags representing what shape collision channels particles will collide with, see NvFlexMakeShapeFlags() (highest bit reserved for now)
        const ShapeChannelMask	= 0x7f000000;

        /// If set this particle will interact with particles of the same group
        const SelfCollide = 1 << 20;
        /// If set this particle will ignore collisions with particles closer than the radius in the rest pose, this flag should not be specified unless valid rest positions have been specified using NvFlexSetRestParticles()
        const SelfCollideFilter	= 1 << 21;
        /// If set this particle will generate fluid density constraints for its overlapping neighbors
        const Fluid = 1 << 22;
        /// Reserved
        const Unused= 1 << 23;

        /// Particle will collide with shapes with channel 0 set (see NvFlexMakeShapeFlags())
        const ShapeChannel0 = 1 << 24;
        /// Particle will collide with shapes with channel 1 set (see NvFlexMakeShapeFlags())
        const ShapeChannel1 = 1 << 25;
        /// Particle will collide with shapes with channel 2 set (see NvFlexMakeShapeFlags())
        const ShapeChannel2 = 1 << 26;
        /// Particle will collide with shapes with channel 3 set (see NvFlexMakeShapeFlags())
        const ShapeChannel3 = 1 << 27;
        /// Particle will collide with shapes with channel 4 set (see NvFlexMakeShapeFlags())
        const ShapeChannel4 = 1 << 28;
        /// Particle will collide with shapes with channel 5 set (see NvFlexMakeShapeFlags())
        const ShapeChannel5 = 1 << 29;
        /// Particle will collide with shapes with channel 6 set (see NvFlexMakeShapeFlags())
        const ShapeChannel6 = 1 << 30;
    }
}

/// Time spent in each section of the solver update, times in GPU seconds, see NvFlexUpdateSolver()
#[derive(Copy, Clone, PartialEq, Debug, Default)]
#[repr(C)]
pub struct Timers {
    /// Time spent in prediction
    pub predict: f32,
    /// Time spent creating grid indices
    pub create_cell_indices: f32,
    /// Time spent sorting grid indices
    pub sort_cell_indices: f32,
    /// Time spent creating grid
    pub create_grid: f32,
    /// Time spent reordering particles
    pub reorder: f32,
    /// Time spent finding particle neighbors
    pub collide_particles: f32,
    /// Time spent colliding convex shapes
    pub collide_shapes: f32,
    /// Time spent colliding triangle shapes
    pub collide_triangles: f32,
    /// Time spent colliding signed distance field shapes
    pub collide_fields: f32,
    /// Time spent calculating fluid density
    pub calculate_density: f32,
    /// Time spent solving density constraints
    pub solve_densities: f32,
    /// Time spent solving velocity constraints
    pub solve_velocities: f32,
    /// Time spent solving rigid body constraints
    pub solve_shapes: f32,
    /// Time spent solving distance constraints
    pub solve_springs: f32,
    /// Time spent solving contact constraints
    pub solve_contacts: f32,
    /// Time spent solving pressure constraints
    pub solve_inflatables: f32,
    /// Time spent adding position deltas to particles
    pub apply_deltas: f32,
    /// Time spent calculating particle anisotropy for fluid
    pub calculate_anisotropy: f32,
    /// Time spent updating diffuse particles
    pub update_diffuse: f32,
    /// Time spent updating dynamic triangles
    pub update_triangles: f32,
    /// Time spent updating vertex normals
    pub update_normals: f32,
    /// Time spent finalizing state
    pub finalize: f32,
    /// Time spent updating particle bounds
    pub update_bounds: f32,
    /// Sum of all timers above
    pub total: f32,
}

/// Flex error return codes
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(C)]
pub enum ErrorSeverity {
    /// Error messages
    Error = 0,
    /// Information messages
    Info = 1,
    /// Warning messages
    Warning = 2,
    /// Used only in debug version of dll
    Debug = 4,
    /// All log types
    All = -1,
}

/// Defines the set of stages at which callbacks may be registered
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(C)]
pub enum SolverCallbackStage {
    /// Called at the beginning of each constraint iteration
    IterationStart,
    /// Called at the end of each constraint iteration
    IterationEnd,
    /// Called at the beginning of each substep after the prediction step has been completed
    SubstepBegin,
    /// Called at the end of each substep after the velocity has been updated by the constraints
    SubstepEnd,
    /// Called at the end of solver update after the final substep has completed
    UpdateEnd,
    /// Number of stages
    Count,
}

// TODO
/*
///Structure containing pointers to the internal solver data that is passed to each registered solver callback
///
/// # Remarks
///
/// * Pointers to internal data are only valid for the lifetime of the callback and should not be stored.
/// However, it is safe to launch kernels and memory transfers using the device pointers.
///
/// * Because Flex re-orders particle data internally for performance, the particle data in the callback is not
/// in the same order as it was provided to the API. The callback provides arrays which map original particle indices
/// to sorted positions and vice-versa.
///
/// * Particle positions may be modified during any callback, but velocity modifications should only occur during
/// the eNvFlexStageUpdateEnd stage, otherwise any velocity changes will be discarded.

pub struct SolverCallbackParams {
    NvFlexSolver* solver;				//!< Pointer to the solver that the callback is registered to
    void* userData;						//!< Pointer to the user data provided to NvFlexRegisterSolverCallback()

    float* particles;					//!< Device pointer to the active particle basic data in the form x,y,z,1/m
    float* velocities;					//!< Device pointer to the active particle velocity data in the form x,y,z,w (last component is not used)
    int* phases;						//!< Device pointer to the active particle phase data

    int numActive;						//!< The number of active particles returned, the callback data only return pointers to active particle data, this is the same as NvFlexGetActiveCount()

    float dt;							//!< The per-update time-step, this is the value passed to NvFlexUpdateSolver()

    const int* originalToSortedMap;		//!< Device pointer that maps the sorted callback data to the original position given by SetParticles()
    const int* sortedToOriginalMap;		//!< Device pointer that maps the original particle index to the index in the callback data structure
}
*/

/// Function pointer type for error reporting callbacks
pub type ErrorCallback = nvflex_sys::NvFlexErrorCallback;

/// Solver callback definition, see NvFlexRegisterSolverCallback()
pub type SolverCallback = nvflex_sys::NvFlexSolverCallback;

/// Defines the different compute backends that Flex can use
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(C)]
pub enum ComputeType {
    /// Use CUDA compute for Flex, the application must link against the CUDA libraries
    CUDA,
    /// Use DirectX 11 compute for Flex, the application must link against the D3D libraries
    D3D11,
    /// Use DirectX 12 compute for Flex, the application must link against the D3D libraries
    D3D12,
}

/// Descriptor used to initialize Flex
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(C)]
pub struct InitDesc {
    /// The GPU device index that should be used, if there is already a CUDA context on the calling thread then this parameter will be ignored and the active CUDA context used. Otherwise a new context will be created using the suggested device ordinal.
    pub device_index: i32,
    /// Enable or disable NVIDIA/AMD extensions in DirectX, can lead to improved performance.
    pub enable_extensions: bool,
    /// Direct3D device to use for simulation, if none is specified a new device and context will be created.
    pub render_device: *mut c_void,
    /// Direct3D context that the app is using for rendering. In DirectX 12 this should be a ID3D12CommandQueue pointer.
    pub render_context: *mut c_void,
    /// Direct3D context to use for simulation, if none is specified a new context will be created, in DirectX 12 this should be a pointer to the ID3D12CommandQueue where compute operations will take place.
    pub compute_context: *mut c_void,
    /// If true, run Flex on D3D11 render context, or D3D12 direct queue. If false, run on a D3D12 compute queue, or vendor specific D3D11 compute queue, allowing compute and graphics to run in parallel on some GPUs.
    pub run_on_render_context: bool,

    /// Set to [`ComputeType::D3D11`] if DirectX 11 should be used, [`ComputeType::D3D12`] for DirectX 12, this must match the libraries used to link the application
    pub compute_type: ComputeType,
}

/// Controls which features are enabled, choosing a simple option will disable features and can lead to better performance and reduced memory usage
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(C)]
pub enum FeatureMode {
    /// All features enabled
    Default = 0,
    /// Simple per-particle collision (no per-particle SDF normals, no fluids)
    SimpleSolids = 1,
    /// Simple single phase fluid-only particles (no solids)
    SimpleFluids = 2,
}

/// Describes the creation time parameters for the solver
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(C)]
pub struct SolverDesc {
    /// Control which features are enabled
    pub feature_mode: FeatureMode,

    /// Maximum number of regular particles in the solver
    pub max_particles: i32,
    /// Maximum number of diffuse particles in the solver
    pub max_diffuse_particles: i32,
    /// Maximum number of neighbors per-particle, for solids this can be around 32, for fluids up to 128 may be necessary depending on smoothing radius
    pub max_neighbors_per_particle: i32,
    /// Maximum number of collision contacts per-particle
    pub max_contacts_per_particle: i32,
}

impl Default for SolverDesc {
    /// Default [`SolverDesc`] values (set by [`NvFlexSetSolverDescDefaults()`])
    #[inline]
    fn default() -> Self {
        unsafe {
            let mut solver_desc = std::mem::zeroed::<SolverDesc>();
            NvFlexSetSolverDescDefaults(&mut solver_desc as *mut _ as _);
            solver_desc
        }
    }
}

/// Describes a source and destination buffer region for performing a copy operation.
#[derive(Copy, Clone, PartialEq, Eq, Debug, Default)]
#[repr(C)]
pub struct CopyDesc {
    /// Offset in elements from the start of the source buffer to begin reading from
    pub src_offset: i32,
    /// Offset in elements from the start of the destination buffer to being writing to
    pub dst_offset: i32,
    /// Number of elements to copy
    pub element_count: i32,
}

/// An opaque type representing a static triangle mesh in the solver
pub type TriangleMeshId = u32;

/// An opaque type representing a signed distance field collision shape in the solver.
pub type DistanceFieldId = u32;

/// An opaque type representing a convex mesh collision shape in the solver.
///
/// Convex mesh shapes may consist of up to 64 planes of the form a*x + b*y + c*z + d = 0,
/// particles will be constrained to the outside of the shape.
pub type ConvexMeshId = u32;

/// A basic sphere shape with origin at the center of the sphere and radius
#[derive(Copy, Clone, PartialEq, Debug, Default)]
#[repr(C)]
pub struct SphereGeometry {
    pub radius: f32,
}

/// A collision capsule extends along the x-axis with its local origin at the center of the capsule
#[derive(Copy, Clone, PartialEq, Debug, Default)]
#[repr(C)]
pub struct CapsuleGeometry {
    pub radius: f32,
    pub half_height: f32,
}

/// A simple box with interior [-halfHeight, +halfHeight] along each dimension
#[derive(Copy, Clone, PartialEq, Debug, Default)]
#[repr(C)]
pub struct BoxGeometry {
    pub half_extents: [f32; 3usize],
}

/// A convex mesh instance with non-uniform scale
#[derive(Copy, Clone, PartialEq, Debug, Default)]
#[repr(C)]
pub struct ConvexMeshGeometry {
    pub scale: [f32; 3usize],
    pub mesh: ConvexMeshId,
}

/// A scaled triangle mesh instance with non-uniform scale
#[derive(Copy, Clone, PartialEq, Debug, Default)]
#[repr(C)]
pub struct TriangleMeshGeometry {
    /// The scale of the object from local space to world space
    pub scale: [f32; 3usize],
    /// A triangle mesh pointer created by NvFlexCreateTriangleMesh()
    pub mesh: TriangleMeshId,
}

/// A scaled signed distance field instance, the local origin of the SDF is at corner of the field corresponding to the first voxel.
///
/// The field is mapped to the local space volume [0, 1] in each dimension.
#[derive(Copy, Clone, PartialEq, Debug, Default)]
#[repr(C)]
pub struct SDFGeometry {
    /// Uniform scale of SDF, this corresponds to the world space width of the shape
    pub scale: f32,
    /// A signed distance field pointer created by NvFlexCreateDistanceField()
    pub field: DistanceFieldId,
}

/// This union allows collision geometry to be sent to Flex as a flat array of 16-byte data structures,
/// the shape flags array specifies the type for each shape, see NvFlexSetShapes().
#[derive(Copy, Clone)]
#[repr(C)]
pub union CollisionGeometry {
    pub sphere: SphereGeometry,
    pub capsule: CapsuleGeometry,
    pub r#box: BoxGeometry,
    pub convex_mesh: ConvexMeshGeometry,
    pub tri_mesh: TriangleMeshGeometry,
    pub sdf: SDFGeometry,
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(C)]
pub enum CollisionShapeType {
    /// A sphere shape, see [`SphereGeometry`]
    Sphere = 0,
    /// A capsule shape, see [`CapsuleGeometry`]
    Capsule = 1,
    /// A box shape, see [`BoxGeometry`]
    Box = 2,
    /// A convex mesh shape, see [`ConvexMeshGeometry`]
    ConvexMesh = 3,
    /// A triangle mesh shape, see [`TriangleMeshGeometry`]
    TriangleMesh = 4,
    /// A signed distance field shape, see [`SDFGeometry`]
    SDF = 5,
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[repr(C)]
pub enum CollisionShapeFlags {
    /// Lower 3 bits holds the type of the collision shape given by the NvFlexCollisionShapeType enum
    TypeMask = 0x7,
    /// Indicates the shape is dynamic and should have lower priority over static collision shapes
    Dynamic = 0x8,
    /// Indicates that the shape is a trigger volume, this means it will not perform any collision response, but will be reported in the contacts array (see NvFlexGetContacts())
    Trigger = 0x10,

    Reserved = 0xffffff00,
}

/// Holds the execution time for a specfic shader
#[derive(Clone, PartialEq, Debug, Default)]
#[repr(C)]
pub struct DetailTimer {
    pub name: String,
    pub time: f32,
}

#[cfg(test)]
mod tests {
    use super::*;
    use nvflex_sys::*;
    use std::mem::size_of;

    #[test]
    fn types_have_equal_size() {
        assert!(VERSION == NV_FLEX_VERSION);
        assert!(size_of::<MapFlags>() == size_of::<NvFlexMapFlags>());
        assert!(size_of::<BufferType>() == size_of::<NvFlexBufferType>());
        assert!(size_of::<RelaxationMode>() == size_of::<NvFlexRelaxationMode>());
        assert!(size_of::<ErrorSeverity>() == size_of::<NvFlexErrorSeverity>());
        assert!(size_of::<SolverCallbackStage>() == size_of::<NvFlexSolverCallbackStage>());
        assert!(size_of::<ComputeType>() == size_of::<NvFlexComputeType>());
        assert!(size_of::<FeatureMode>() == size_of::<NvFlexFeatureMode>());
        assert!(size_of::<CollisionShapeType>() == size_of::<NvFlexCollisionShapeType>());
        assert!(size_of::<CollisionShapeFlags>() == size_of::<NvFlexCollisionShapeFlags>());
        assert!(size_of::<Params>() == size_of::<NvFlexParams>());
        assert!(size_of::<Phase>() == size_of::<NvFlexPhase>());
        assert!(size_of::<Timers>() == size_of::<NvFlexTimers>());
        // TODO
        // assert!(size_of::<SolverCallbackParams>() == size_of::<NvFlexSolverCallbackParams>());
        // assert!(size_of::<SolverCallback>() == size_of::<NvFlexSolverCallback>());
        assert!(size_of::<InitDesc>() == size_of::<NvFlexInitDesc>());
        assert!(size_of::<SolverDesc>() == size_of::<NvFlexSolverDesc>());
        assert!(size_of::<CopyDesc>() == size_of::<NvFlexCopyDesc>());
        assert!(size_of::<SphereGeometry>() == size_of::<NvFlexSphereGeometry>());
        assert!(size_of::<CapsuleGeometry>() == size_of::<NvFlexCapsuleGeometry>());
        assert!(size_of::<BoxGeometry>() == size_of::<NvFlexBoxGeometry>());
        assert!(size_of::<ConvexMeshGeometry>() == size_of::<NvFlexConvexMeshGeometry>());
        assert!(size_of::<TriangleMeshGeometry>() == size_of::<NvFlexTriangleMeshGeometry>());
        assert!(size_of::<SDFGeometry>() == size_of::<NvFlexSDFGeometry>());
        // DetailTimer uses a String instead of a &str, so the size will be different
        // assert!(size_of::<DetailTimer>() == size_of::<NvFlexDetailTimer>());
        assert!(size_of::<CollisionGeometry>() == size_of::<NvFlexCollisionGeometry>());
    }
}
