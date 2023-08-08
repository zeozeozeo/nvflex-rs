use nvflex_sys::*;

/// Controls the relaxation method used by the solver to ensure convergence.
#[derive(Debug, Copy, Clone)]
pub enum RelaxationMode {
    /// The relaxation factor is a fixed multiplier on each constraint's position delta.
    Global = 0,
    /// The relaxation factor is a fixed multiplier on each constraint's delta divided by the particle's constraint count, convergence will be slower but more reliable.
    Local = 1,
}

impl Into<NvFlexRelaxationMode> for RelaxationMode {
    #[inline]
    fn into(self) -> NvFlexRelaxationMode {
        match self {
            RelaxationMode::Global => eNvFlexRelaxationGlobal,
            RelaxationMode::Local => eNvFlexRelaxationLocal,
        }
    }
}

/// Simulation parameters for a solver.
#[derive(Debug, Copy, Clone)]
pub struct SolverParams {
    /// Number of solver iterations to perform per-substep.
    pub num_iterations: i32,
    /// Constant acceleration applied to all particles.
    pub gravity: [f32; 3usize],
    /// The maximum interaction radius for particles.
    pub radius: f32,
    /// The distance non-fluid particles attempt to maintain from each other, must be in the range (0, radius].
    pub solid_rest_distance: f32,
    /// The distance fluid particles are spaced at the rest density, must be in the range (0, radius], for fluids this should generally be 50-70% of mRadius, for rigids this can simply be the same as the particle radius.
    pub fluid_rest_distance: f32,
    /// Coefficient of friction used when colliding against shapes.
    pub dynamic_friction: f32,
    /// Coefficient of static friction used when colliding against shapes.
    pub static_friction: f32,
    /// Coefficient of friction used when colliding particles.
    pub particle_friction: f32,
    /// Coefficient of restitution used when colliding against shapes, particle collisions are always inelastic.
    pub restitution: f32,
    /// Controls how strongly particles stick to surfaces they hit, default 0.0, range [0.0, +inf].
    pub adhesion: f32,
    /// Particles with a velocity magnitude < this threshold will be considered fixed.
    pub sleep_threshold: f32,
    /// The magnitude of particle velocity will be clamped to this value at the end of each step.
    pub max_speed: f32,
    /// The magnitude of particle acceleration will be clamped to this value at the end of each step (limits max velocity change per-second), useful to avoid popping due to large interpenetrations.
    pub max_acceleration: f32,
    /// Artificially decrease the mass of particles based on height from a fixed reference point, this makes stacks and piles converge faster.
    pub shock_propagation: f32,
    /// Damps particle velocity based on how many particle contacts it has.
    pub dissipation: f32,
    /// Viscous drag force, applies a force proportional, and opposite to the particle velocity.
    pub damping: f32,
    /// Constant acceleration applied to particles that belong to dynamic triangles, drag needs to be > 0 for wind to affect triangles.
    pub wind: [f32; 3usize],
    /// Drag force applied to particles belonging to dynamic triangles, proportional to velocity^2*area in the negative velocity direction.
    pub drag: f32,
    /// Lift force applied to particles belonging to dynamic triangles, proportional to velocity^2*area in the direction perpendicular to velocity and (if possible), parallel to the plane normal.
    pub lift: f32,
    /// Control how strongly particles hold each other together, default: 0.025, range [0.0, +inf].
    pub cohesion: f32,
    /// Controls how strongly particles attempt to minimize surface area, default: 0.0, range: [0.0, +inf].
    pub surface_tension: f32,
    /// Smoothes particle velocities using XSPH viscosity.
    pub viscosity: f32,
    /// Increases vorticity by applying rotational forces to particles.
    pub vorticity_confinement: f32,
    /// Control how much anisotropy is present in resulting ellipsoids for rendering, if zero then anisotropy will not be calculated, see NvFlexGetAnisotropy().
    pub anisotropy_scale: f32,
    /// Clamp the anisotropy scale to this fraction of the radius.
    pub anisotropy_min: f32,
    /// Clamp the anisotropy scale to this fraction of the radius.
    pub anisotropy_max: f32,
    /// Control the strength of Laplacian smoothing in particles for rendering, if zero then smoothed positions will not be calculated, see NvFlexGetSmoothParticles().
    pub smoothing: f32,
    /// Add pressure from solid surfaces to particles.
    pub solid_pressure: f32,
    /// Drag force applied to boundary fluid particles.
    pub free_surface_drag: f32,
    /// Gravity is scaled by this value for fluid particles.
    pub buoyancy: f32,
    /// Particles with kinetic energy + divergence above this threshold will spawn new diffuse particles.
    pub diffuse_threshold: f32,
    /// Scales force opposing gravity that diffuse particles receive.
    pub diffuse_buoyancy: f32,
    /// Scales force diffuse particles receive in direction of neighbor fluid particles.
    pub diffuse_drag: f32,
    /// The number of neighbors below which a diffuse particle is considered ballistic.
    pub diffuse_ballistic: i32,
    /// Time in seconds that a diffuse particle will live for after being spawned, particles will be spawned with a random lifetime in the range [0, diffuseLifetime].
    pub diffuse_lifetime: f32,
    /// Distance particles maintain against shapes, note that for robust collision against triangle meshes this distance should be greater than zero.
    pub collision_distance: f32,
    /// Increases the radius used during neighbor finding, this is useful if particles are expected to move significantly during a single step to ensure contacts aren't missed on subsequent iterations.
    pub particle_collision_margin: f32,
    /// Increases the radius used during contact finding against kinematic shapes.
    pub shape_collision_margin: f32,
    /// Collision planes in the form ax + by + cz + d = 0.
    pub planes: [[f32; 4usize]; 8usize],
    /// Num collision planes.
    pub num_planes: i32,
    /// How the relaxation is applied inside the solver.
    pub relaxation_mode: RelaxationMode,
    /// Control the convergence rate of the parallel solver, default: 1, values greater than 1 may lead to instability.
    pub relaxation_factor: f32,
}

impl SolverParams {
    /// Default solver params taken from the FleX demo
    pub const DEFAULT_PARAMS: Self = Self {
        gravity: [0.0, -9.81, 0.0],
        wind: [0.0f32; 3],
        radius: 0.15, // The maximum interaction radius for particles
        viscosity: 0.0,
        dynamic_friction: 0.0,
        static_friction: 0.0,
        particle_friction: 0.0, // scale friction between particles by default
        free_surface_drag: 0.0,
        drag: 0.0,
        lift: 0.0,
        num_iterations: 3,
        fluid_rest_distance: 0.0,
        solid_rest_distance: 0.0,
        anisotropy_scale: 1.0, // If this parameter is set to larger than 0.0 the Flex simulation generates per particle information that can be used to improve the fluid surface rendering. The anisotropy information is used to render oriented ellipsoids instead of spheres which makes the surface look much smoother. Setting this higher allow the ellipsoids to stretch more. A value around 1.0 works usually well
        anisotropy_min: 0.1, // Defines how thin a particle can become. It makes sense to limit the thickness for example to avoid shadowing artefact which can appear for very thin objects. A value around 0.1 works usually well
        anisotropy_max: 2.0, // Defines how thick a particle can become. It can look weird if ellipsoids stretch too much as the geometric shape can be become very obvious. A value around 1.0 works usually well
        smoothing: 1.0, // The look of the surface can be improved by enabling smoothing of the particle positions. Values larger than 0.0 enable the particle smoothing
        dissipation: 0.0,
        damping: 0.0,
        particle_collision_margin: 0.0,
        shape_collision_margin: 0.0,
        collision_distance: 0.0, // Distance particles maintain against shapes, note that for robust collision against triangle meshes this distance should be greater than zero
        sleep_threshold: 0.0,
        shock_propagation: 0.0,
        restitution: 0.0,
        max_speed: f32::MAX,
        max_acceleration: 100.0, // approximately 10x gravity
        relaxation_mode: RelaxationMode::Local,
        relaxation_factor: 1.0,
        solid_pressure: 1.0,
        adhesion: 0.0,
        cohesion: 0.025,
        surface_tension: 0.0,
        vorticity_confinement: 0.0,
        buoyancy: 1.0,
        diffuse_threshold: 100.0,
        diffuse_buoyancy: 1.0,
        diffuse_drag: 0.8,
        diffuse_ballistic: 16,
        diffuse_lifetime: 2.0,
        planes: [
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
        ],
        num_planes: 0,
    };

    /// Tweaks solver parameters to be more or less sane.
    ///
    /// This can change `solid_rest_distance`, `collision_distance`, `particle_friction` and `shape_collision_margin` depending on their values.
    pub fn fixed(self) -> Self {
        let mut fixed = self;

        if fixed.solid_rest_distance == 0.0 {
            fixed.solid_rest_distance = fixed.radius;
        }

        // if fluid present then we assume solid particles have the same radius
        if fixed.fluid_rest_distance > 0.0 {
            fixed.solid_rest_distance = fixed.fluid_rest_distance;
        }

        // set collision distance automatically based on rest distance if not alraedy set
        if fixed.collision_distance == 0.0 {
            fixed.collision_distance =
                fixed.solid_rest_distance.max(fixed.fluid_rest_distance) * 0.5;
        }

        // default particle friction to 10% of shape friction
        if fixed.particle_friction == 0.0 {
            fixed.particle_friction = fixed.dynamic_friction * 0.1;
        }

        // add a margin for detecting contacts between particles and shapes
        if fixed.shape_collision_margin == 0.0 {
            fixed.shape_collision_margin = fixed.collision_distance * 0.5;
        }

        fixed
    }
}

impl Into<NvFlexParams> for SolverParams {
    fn into(self) -> NvFlexParams {
        NvFlexParams {
            numIterations: self.num_iterations,
            gravity: self.gravity,
            radius: self.radius,
            solidRestDistance: self.solid_rest_distance,
            fluidRestDistance: self.fluid_rest_distance,
            dynamicFriction: self.dynamic_friction,
            staticFriction: self.static_friction,
            particleFriction: self.particle_friction,
            restitution: self.restitution,
            adhesion: self.adhesion,
            sleepThreshold: self.sleep_threshold,
            maxSpeed: self.max_speed,
            maxAcceleration: self.max_acceleration,
            shockPropagation: self.shock_propagation,
            dissipation: self.dissipation,
            damping: self.damping,
            wind: self.wind,
            drag: self.drag,
            lift: self.lift,
            cohesion: self.cohesion,
            surfaceTension: self.surface_tension,
            viscosity: self.viscosity,
            vorticityConfinement: self.vorticity_confinement,
            anisotropyScale: self.anisotropy_scale,
            anisotropyMin: self.anisotropy_min,
            anisotropyMax: self.anisotropy_max,
            smoothing: self.smoothing,
            solidPressure: self.solid_pressure,
            freeSurfaceDrag: self.free_surface_drag,
            buoyancy: self.buoyancy,
            diffuseThreshold: self.diffuse_threshold,
            diffuseBuoyancy: self.diffuse_buoyancy,
            diffuseDrag: self.diffuse_drag,
            diffuseBallistic: self.diffuse_ballistic,
            diffuseLifetime: self.diffuse_lifetime,
            collisionDistance: self.collision_distance,
            particleCollisionMargin: self.particle_collision_margin,
            shapeCollisionMargin: self.shape_collision_margin,
            planes: self.planes,
            numPlanes: self.num_planes,
            relaxationMode: self.relaxation_mode.into(),
            relaxationFactor: self.relaxation_factor,
        }
    }
}

/// Controls which features are enabled, choosing a simple option will disable features and can lead to better performance and reduced memory usage.
#[derive(Debug, Copy, Clone)]
pub enum FeatureMode {
    /// All features enabled.
    Default = 0,
    /// Simple per-particle collision (no per-particle SDF normals, no fluids).
    SimpleSolids = 1,
    /// Simple single phase fluid-only particles (no solids).
    SimpleFluids = 2,
}

impl Into<NvFlexFeatureMode> for FeatureMode {
    #[inline]
    fn into(self) -> NvFlexFeatureMode {
        match self {
            FeatureMode::Default => eNvFlexFeatureModeDefault,
            FeatureMode::SimpleSolids => eNvFlexFeatureModeSimpleSolids,
            FeatureMode::SimpleFluids => eNvFlexFeatureModeSimpleFluids,
        }
    }
}

/// Describes the creation time parameters for the solver.
#[derive(Debug, Copy, Clone)]
pub struct SolverDesc {
    /// Control which features are enabled.
    pub feature_mode: FeatureMode,
    /// Maximum number of regular particles in the solver.
    pub max_particles: i32,
    /// Maximum number of diffuse particles in the solver.
    pub max_diffuse_particles: i32,
    /// Maximum number of neighbors per-particle, for solids this can be around 32, for fluids up to 128 may be necessary depending on smoothing radius.
    pub max_neighbors_per_particle: i32,
    /// Maximum number of collision contacts per-particle.
    pub max_contacts_per_particle: i32,
}

impl Into<NvFlexSolverDesc> for SolverDesc {
    #[inline]
    fn into(self) -> NvFlexSolverDesc {
        NvFlexSolverDesc {
            featureMode: self.feature_mode.into(),
            maxParticles: self.max_particles,
            maxDiffuseParticles: self.max_diffuse_particles,
            maxNeighborsPerParticle: self.max_neighbors_per_particle,
            maxContactsPerParticle: self.max_contacts_per_particle,
        }
    }
}

/// Defines the different compute backends that Flex can use.
#[derive(Debug, Copy, Clone)]
pub enum ComputeType {
    /// Use CUDA compute for Flex, the application must link against the CUDA libraries.
    CUDA,
    /// Use DirectX 11 compute for Flex, the application must link against the D3D libraries.
    D3D11,
    /// Use DirectX 12 compute for Flex, the application must link against the D3D libraries.
    D3D12,
}

impl Into<NvFlexComputeType> for ComputeType {
    #[inline]
    fn into(self) -> NvFlexComputeType {
        match self {
            ComputeType::CUDA => eNvFlexCUDA,
            ComputeType::D3D11 => eNvFlexD3D11,
            ComputeType::D3D12 => eNvFlexD3D12,
        }
    }
}

/// Descriptor used to initialize Flex.
#[derive(Debug, Copy, Clone)]
pub struct InitDesc {
    /// The GPU device index that should be used, if there is already a CUDA context on the calling thread then this parameter will be ignored and the active CUDA context used. Otherwise a new context will be created using the suggested device ordinal.
    device_index: i32,
    /// Enable or disable NVIDIA/AMD extensions in DirectX, can lead to improved performance.
    enable_extensions: bool,
    /// Direct3D device to use for simulation, if none is specified a new device and context will be created.
    render_device: *mut std::os::raw::c_void,
    /// Direct3D context that the app is using for rendering. In DirectX 12 this should be a ID3D12CommandQueue pointer.
    render_context: *mut std::os::raw::c_void,
    /// Direct3D context to use for simulation, if none is specified a new context will be created, in DirectX 12 this should be a pointer to the ID3D12CommandQueue where compute operations will take place.
    compute_context: *mut std::os::raw::c_void,
    /// If true, run Flex on D3D11 render context, or D3D12 direct queue. If false, run on a D3D12 compute queue, or vendor specific D3D11 compute queue, allowing compute and graphics to run in parallel on some GPUs.
    run_on_render_context: bool,
    /// Set to `ComputeType::D3D11` if DirectX 11 should be used, `ComputeType::D3D12` for DirectX 12, this must match the libraries used to link the application.
    compute_type: ComputeType,
}

impl Into<NvFlexInitDesc> for InitDesc {
    #[inline]
    fn into(self) -> NvFlexInitDesc {
        NvFlexInitDesc {
            deviceIndex: self.device_index,
            enableExtensions: self.enable_extensions,
            renderDevice: self.render_device,
            renderContext: self.render_context,
            computeContext: self.compute_context,
            runOnRenderContext: self.run_on_render_context,
            computeType: self.compute_type.into(),
        }
    }
}
