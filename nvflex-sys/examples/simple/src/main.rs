#![allow(unused_variables, dead_code, unused_imports)]
use nvflex_sys::*;
use std::ffi::CString;

macro_rules! rstr {
    ($cstring:expr) => {{
        #[allow(unused_unsafe)]
        unsafe { std::ffi::CStr::from_ptr($cstring) }
            .to_str()
            .expect("unable to convert C string")
    }};
}

fn main() {
    unsafe { run() };
}

// severity = NvFlexErrorSeverity
unsafe extern "C" fn flex_error_callback(
    severity: NvFlexErrorSeverity,
    msg: *const i8,
    file: *const i8,
    line: i32,
) {
    eprintln!(
        "FleX error at {}:{} (severity {}): {}",
        rstr!(file),
        line,
        severity,
        rstr!(msg)
    );
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Default)]
pub struct Vec4 {
    x: f32,
    y: f32,
    z: f32,
    w: f32,
}

impl Vec4 {
    const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 0.0,
    };

    #[inline(always)]
    pub fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, Default)]
pub struct Vec3 {
    x: f32,
    y: f32,
    z: f32,
}

impl Vec3 {
    const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    #[inline(always)]
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}

unsafe fn run() {
    // initialize FleX
    let library = NvFlexInit(
        NV_FLEX_VERSION as _,
        Some(flex_error_callback),
        std::ptr::null_mut(),
    );
    if library.is_null() {
        panic!("unable to initialize FleX");
    }

    // create new solver
    const MAX_PARTICLES: i32 = 10_000;
    let mut solver_desc = std::mem::zeroed::<NvFlexSolverDesc>();
    NvFlexSetSolverDescDefaults(&mut solver_desc);

    solver_desc.maxParticles = MAX_PARTICLES;
    solver_desc.maxDiffuseParticles = 0;

    let solver = NvFlexCreateSolver(library, &solver_desc);
    let particle_buffer = NvFlexAllocBuffer(
        library,
        MAX_PARTICLES,
        std::mem::size_of::<Vec4>() as _,
        eNvFlexBufferHost,
    );
    // the velocity buffer requires a 12 byte stride, so we use a vec3 here
    let velocity_buffer = NvFlexAllocBuffer(
        library,
        MAX_PARTICLES,
        std::mem::size_of::<Vec3>() as _,
        eNvFlexBufferHost,
    );
    let phase_buffer = NvFlexAllocBuffer(
        library,
        MAX_PARTICLES,
        std::mem::size_of::<i32>() as _,
        eNvFlexBufferHost,
    );

    // set params
    let mut params = std::mem::zeroed::<NvFlexParams>();
    params.gravity[0] = 0.0;
    params.gravity[1] = -10.0;
    params.gravity[2] = 0.0; // Constant acceleration applied to all particles
    params.radius = 0.01; // The maximum interaction radius for particles
    params.solidRestDistance = 0.005; // The distance non-fluid particles attempt to maintain from each other, must be in the range (0, radius]
    params.fluidRestDistance = 0.001; // The distance fluid particles are spaced at the rest density, must be in the range (0, radius], for fluids this should generally be 50-70% of mRadius, for rigids this can simply be the same as the particle radius

    // common params
    params.dynamicFriction = 1.0; // Coefficient of friction used when colliding against shapes
    params.staticFriction = 1.0; // Coefficient of static friction used when colliding against shapes
    params.particleFriction = 1.0; // Coefficient of friction used when colliding particles
    params.restitution = 1.0; // Coefficient of restitution used when colliding against shapes, particle collisions are always inelastic
    params.adhesion = 1.0; // Controls how strongly particles stick to surfaces they hit, default 0.0, range [0.0, +inf]
    params.sleepThreshold = 1.0; // Particles with a velocity magnitude < this threshold will be considered fixed

    params.maxSpeed = f32::MAX; // The magnitude of particle velocity will be clamped to this value at the end of each step
    params.maxAcceleration = 1000.0; // The magnitude of particle acceleration will be clamped to this value at the end of each step (limits max velocity change per-second), useful to avoid popping due to large interpenetrations

    params.shockPropagation = 0.0; // Artificially decrease the mass of particles based on height from a fixed reference point, this makes stacks and piles converge faster
    params.dissipation = 1.0; // Damps particle velocity based on how many particle contacts it has
    params.damping = 10.0; // Viscous drag force, applies a force proportional, and opposite to the particle velocity

    // cloth params
    params.wind = [0.0; 3]; // Constant acceleration applied to particles that belong to dynamic triangles, drag needs to be > 0 for wind to affect triangles
    params.drag = 1.0; // Drag force applied to particles belonging to dynamic triangles, proportional to velocity^2*area in the negative velocity direction
    params.lift = 1.0; // Lift force applied to particles belonging to dynamic triangles, proportional to velocity^2*area in the direction perpendicular to velocity and (if possible), parallel to the plane normal

    // fluid params
    params.cohesion = 0.025; // Control how strongly particles hold each other together, default: 0.025, range [0.0, +inf]
    params.surfaceTension = 0.0; // Controls how strongly particles attempt to minimize surface area, default: 0.0, range: [0.0, +inf]
    params.viscosity = 1.0; // Smoothes particle velocities using XSPH viscosity
    params.vorticityConfinement = 0.0; // Increases vorticity by applying rotational forces to particles
    params.anisotropyScale = 0.0; // Control how much anisotropy is present in resulting ellipsoids for rendering, if zero then anisotropy will not be calculated, see NvFlexGetAnisotropy()
    params.anisotropyMin = 0.0; // Clamp the anisotropy scale to this fraction of the radius
    params.anisotropyMax = 0.0; // Clamp the anisotropy scale to this fraction of the radius
    params.smoothing = 1.0; // Control the strength of Laplacian smoothing in particles for rendering, if zero then smoothed positions will not be calculated, see NvFlexGetSmoothParticles()
    params.solidPressure = 0.0; // Add pressure from solid surfaces to particles
    params.freeSurfaceDrag = 0.0; // Drag force applied to boundary fluid particles
    params.buoyancy = 0.0; // Gravity is scaled by this value for fluid particles

    // diffuse params
    params.diffuseThreshold = 0.0; // Particles with kinetic energy + divergence above this threshold will spawn new diffuse particles
    params.diffuseBuoyancy = 0.0; // Scales force opposing gravity that diffuse particles receive
    params.diffuseDrag = 0.0; // Scales force diffuse particles receive in direction of neighbor fluid particles
    params.diffuseBallistic = 0; // The number of neighbors below which a diffuse particle is considered ballistic
    params.diffuseLifetime = 0.0; // Time in seconds that a diffuse particle will live for after being spawned, particles will be spawned with a random lifetime in the range [0, diffuseLifetime]

    // collision params
    params.collisionDistance = 0.01; // Distance particles maintain against shapes, note that for robust collision against triangle meshes this distance should be greater than zero
    params.particleCollisionMargin = 1.0; // Increases the radius used during neighbor finding, this is useful if particles are expected to move significantly during a single step to ensure contacts aren't missed on subsequent iterations
    params.shapeCollisionMargin = 0.0; // Increases the radius used during contact finding against kinematic shapes

    // Collision planes in the form ax + by + cz + d = 0
    for i in 0..8 {
        for j in 0..4 {
            params.planes[i][j] = 0.0;
        }
    }
    params.numPlanes = 0; // Num collision planes

    params.relaxationMode = eNvFlexRelaxationGlobal; // How the relaxation is applied inside the solver
    params.relaxationFactor = 1.0;

    // apply params
    NvFlexSetParams(solver, &params);

    const PARTICLE_COUNT: i32 = 6;
    const PARTICLE_MASS: f32 = 100.0;

    // map buffers
    let particles: *mut Vec4 = NvFlexMap(particle_buffer, eNvFlexMapWait) as _;
    let velocities: *mut Vec3 = NvFlexMap(velocity_buffer, eNvFlexMapWait) as _;
    let phases: *mut i32 = NvFlexMap(phase_buffer, eNvFlexMapWait) as _;

    for i in 0..PARTICLE_COUNT {
        *particles.add(i as _) = Vec4::new(i as f32 * 5.0 + 10.0, 0.0, 0.0, 1.0 / PARTICLE_MASS);
        *velocities.add(i as _) = Vec3::new(0.0, 0.0, 0.0);
        *phases.add(i as _) = NvFlexMakePhase(0, 1);
    }

    // unmap buffers
    NvFlexUnmap(particle_buffer);
    NvFlexUnmap(velocity_buffer);
    NvFlexUnmap(phase_buffer);

    NvFlexSetParticles(solver, particle_buffer, std::ptr::null());
    NvFlexSetVelocities(solver, velocity_buffer, std::ptr::null());
    NvFlexSetPhases(solver, phase_buffer, std::ptr::null());

    const DELTA_TIME: f32 = 1.0 / 60.0;

    // used for drawing the physics scene in terminal (obviously this not 3d, but it gets the point across)
    // indexed with [y][x], true means that there is a particle, false means that it is empty
    let mut screen: Vec<Vec<bool>> = vec![vec![false; 50]; 25];

    for i in 0..5_000 {
        // map buffers
        let particles: *mut Vec4 = NvFlexMap(particle_buffer, eNvFlexMapWait) as _;
        let velocities: *mut Vec3 = NvFlexMap(velocity_buffer, eNvFlexMapWait) as _;
        let phases: *mut i32 = NvFlexMap(phase_buffer, eNvFlexMapWait) as _;

        // draw the scene in the terminal
        for i in 0..PARTICLE_COUNT as usize {
            let p = *particles.add(i) as Vec4;
            let x = (p.x) as usize;
            let y = (-p.y) as usize;
            if y > screen.len() - 1 || x > screen[y].len() - 1 {
                continue;
            }
            screen[y][x] = true;
        }

        use crossterm::{cursor::MoveTo, ExecutableCommand};
        use std::io::stdout;
        let _ = stdout().execute(MoveTo(0, 0));

        println!("step {}", i);
        for y in 0..screen.len() {
            let rows = screen[y].len();
            let mut row = String::with_capacity(rows);

            for x in 0..rows {
                if screen[y][x] || x == rows - 1 || y == screen.len() - 1 || x == 0 || y == 0 {
                    row.push('*');
                    screen[y][x] = false; // clear the pixel for the next iteration
                } else {
                    row.push(' ');
                }
            }

            println!("{}", row);
        }

        // unmap buffers
        NvFlexUnmap(particle_buffer);
        NvFlexUnmap(velocity_buffer);
        NvFlexUnmap(phase_buffer);

        // set active count
        NvFlexSetActiveCount(solver, PARTICLE_COUNT);

        // tick
        NvFlexUpdateSolver(solver, DELTA_TIME, 1, false);

        // read back (async)
        NvFlexGetParticles(solver, particle_buffer, std::ptr::null());
        NvFlexGetVelocities(solver, velocity_buffer, std::ptr::null());
        NvFlexGetPhases(solver, phase_buffer, std::ptr::null());
    }

    // destroy
    NvFlexFreeBuffer(particle_buffer);
    NvFlexFreeBuffer(velocity_buffer);
    NvFlexFreeBuffer(phase_buffer);

    NvFlexDestroySolver(solver);
    NvFlexShutdown(library);
}
