use mint::{Vector3, Vector4};
use nvflex_sys::*;

use bitflags::bitflags;

bitflags! {
    /// Flags that control a particle's behavior and grouping, use `make_phase()` or `make_phase_with_channels()` to construct a valid 32bit phase identifier.
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    pub struct PhaseFlags: i32 {
        /// Bits `[ 0, 19]` represent the particle group for controlling collisions.
        const GroupMask = 0x000fffff;
        /// Bits `[20, 23]` hold flags about how the particle behave.
        const FlagsMask = 0x00f00000;
        /// Bits `[24, 30]` hold flags representing what shape collision channels particles will collide with, see `make_shape_flags()` (highest bit reserved for now).
        const ShapeChannelMask = 0x7f000000;

        /// If set this particle will interact with particles of the same group.
        const SelfCollide = 1 << 20;
        // TODO: change the comment below
        /// If set this particle will ignore collisions with particles closer than the radius in the rest pose, this flag should not be specified unless valid rest positions have been specified using NvFlexSetRestParticles().
        const SelfCollideFilter = 1 << 21;
        /// If set this particle will generate fluid density constraints for its overlapping neighborsr = 1 << 21,.
        const Fluid = 1 << 22;
        /// Reserved.
        const Unused = 1 << 23;

        /// Particle will collide with shapes with channel 0 set (see `make_shape_flags()`).
        const ShapeChannel0 = 1 << 24;
        /// Particle will collide with shapes with channel 1 set (see `make_shape_flags()`).
        const ShapeChannel1 = 1 << 25;
        /// Particle will collide with shapes with channel 2 set (see `make_shape_flags()`).
        const ShapeChannel2 = 1 << 26;
        /// Particle will collide with shapes with channel 3 set (see `make_shape_flags()`).
        const ShapeChannel3 = 1 << 27;
        /// Particle will collide with shapes with channel 4 set (see `make_shape_flags()`).
        const ShapeChannel4 = 1 << 28;
        /// Particle will collide with shapes with channel 5 set (see `make_shape_flags()`).
        const ShapeChannel5 = 1 << 29;
        /// Particle will collide with shapes with channel 6 set (see `make_shape_flags()`).
        const ShapeChannel6 = 1 << 30;
    }
}

/// Helper method to generates a phase with all shape channels set.
///
/// # Example
///
///
#[inline]
pub fn make_phase(group: PhaseFlags, particle_flags: PhaseFlags) -> i32 {
    // this is not an FFI call, this is an inline function is implemented in Rust by the `nvflex_sys` crate
    NvFlexMakePhase(group.bits(), particle_flags.bits())
}

/// Helper method to generates a phase with all shape channels set.
#[inline]
pub fn make_phase_with_channels(
    group: PhaseFlags,
    particle_flags: PhaseFlags,
    shape_channels: PhaseFlags,
) -> i32 {
    // this is not an FFI call, this is an inline function is implemented in Rust by the `nvflex_sys` crate
    NvFlexMakePhaseWithChannels(group.bits(), particle_flags.bits(), shape_channels.bits())
}

/// Represets a FleX particle.
#[derive(Debug, Copy, Clone)]
pub struct Particle {
    /// Particle position, `pos.x` = x postition, `pos.y` = y position, `pos.z` = z position, `pos.w` = mass.
    pub pos: Vector4<f32>,
    /// Particle velocity, `vel.x` = x velocity, `vel.y` = y velocity, `vel.z` = z velocity.
    pub vel: Vector3<f32>,
    /// Flags that control a particle's behavior and grouping, use `make_phase()` or `make_phase_with_channels()` to construct a valid 32bit phase identifier.
    pub phase: i32,
    /// Whether the particle is active or not.
    pub active: bool,
}

impl Particle {
    #[inline]
    pub fn new(
        pos: impl Into<Vector4<f32>>,
        vel: impl Into<Vector3<f32>>,
        phase: i32,
        active: bool,
    ) -> Self {
        Self {
            pos: pos.into(),
            vel: vel.into(),
            phase,
            active,
        }
    }
}

/// Holds FleX buffers, provides methods to spawn particles and upload them to the solver.
#[derive(Debug, Clone)]
pub struct ParticleSpawner {
    /// Particles pending to be sent to FleX by `flush()`.
    pub pending_particles: Vec<Particle>,
    /// Maximum amount of particles that can be spawned.
    max_particles: usize,
    /// Amount of spawned particles. This includes particles that are not active.
    num_particles: usize,
    /// Amount of active particles.
    num_active_particles: usize,
    /// Holds the particle positions. x = x position, y = y position, z = z position, w = mass.
    pub buffer: *mut NvFlexBuffer,
    /// Holds the particle velocities.
    pub velocities: *mut NvFlexBuffer,
    /// Each particle has an associated phase id which controls how it interacts with other particles.
    ///
    /// Use `make_phase()` or `make_phase_with_channels()` to construct a valid 32bit phase identifier.
    pub phases: *mut NvFlexBuffer,
    /// Holds the indices of particles that have been made active.
    pub active_indices: *mut NvFlexBuffer,
}

impl ParticleSpawner {
    pub fn new(lib: *mut NvFlexLibrary, max_particles: i32) -> Self {
        // max_particles cannot be negative
        let max_particles = if max_particles < 0 { 0 } else { max_particles };

        unsafe {
            Self {
                pending_particles: vec![],
                max_particles: max_particles as _,
                num_particles: 0,
                num_active_particles: 0,
                buffer: NvFlexAllocBuffer(
                    lib,
                    max_particles,
                    std::mem::size_of::<Vector4<f32>>() as _,
                    eNvFlexBufferHost,
                ),
                velocities: NvFlexAllocBuffer(
                    lib,
                    max_particles,
                    std::mem::size_of::<Vector3<f32>>() as _,
                    eNvFlexBufferHost,
                ),
                phases: NvFlexAllocBuffer(
                    lib,
                    max_particles,
                    std::mem::size_of::<i32>() as _,
                    eNvFlexBufferHost,
                ),
                active_indices: NvFlexAllocBuffer(
                    lib,
                    max_particles,
                    std::mem::size_of::<i32>() as _,
                    eNvFlexBufferHost,
                ),
            }
        }
    }

    /// Amount of spawned particles. This includes particles that are not active.
    #[inline]
    pub fn num_particles(&self) -> usize {
        self.num_particles
    }

    /// Amount of active particles.
    #[inline]
    pub fn num_active_particles(&self) -> usize {
        self.num_active_particles
    }

    /// Maximum amount of particles that can be spawned.
    #[inline]
    pub fn max_particles(&self) -> usize {
        self.max_particles
    }

    /// Amount of particles pending to be sent to FleX by `flush()`.
    #[inline]
    pub fn num_pending(&self) -> usize {
        self.pending_particles.len()
    }

    /// Spaws a new particle, but doesn't apply the changes immediately. Calling `self.flush()` will process all pending changes.
    ///
    /// # Arguments
    ///
    /// * `pos` - Position of the particle, `pos.x` = x postition, `pos.y` = y position, `pos.z` = z position, `pos.w` = mass.
    /// * `vel` - Velocity of the particle, `vel.x` = x velocity, `vel.y` = y velocity, `vel.z` = z velocity.
    /// * `phase` - Flags that control a particle's behavior and grouping, use `make_phase()` or `make_phase_with_channels()` to construct a valid 32bit phase identifier.
    /// * `active` - Whether the particle is active or not. Usually you want to set this to `true` for new particles.
    pub fn spawn(
        &mut self,
        pos: impl Into<Vector4<f32>>,
        vel: impl Into<Vector3<f32>>,
        phase: i32,
        active: bool,
    ) {
        if self.pending_particles.len() < self.max_particles {
            self.pending_particles
                .push(Particle::new(pos, vel, phase, active));
        }
    }

    /// Flushes all pending particles to the FleX solver. Returns whether any changes were applied or not.
    pub fn flush(&mut self, solver: *mut NvFlexSolver) -> bool {
        if self.pending_particles.is_empty() || solver.is_null() {
            return false;
        }

        unsafe {
            let buffer = NvFlexMap(self.buffer, eNvFlexMapWait) as *mut Vector4<f32>;
            let velocities = NvFlexMap(self.velocities, eNvFlexMapWait) as *mut Vector3<f32>;
            let phases = NvFlexMap(self.phases, eNvFlexMapWait) as *mut i32;
            // none of the particles could be active, so we don't need to map the active indices buffer yet
            let mut active_indices: Option<*mut i32> = None;

            for (i, p) in self.pending_particles.iter().enumerate() {
                let offset = self.num_particles + i; // num_particles is incremented at the end
                buffer.add(offset).write(p.pos);
                velocities.add(offset).write(p.vel);
                phases.add(offset).write(p.phase);

                if p.active {
                    // the particle is active, we need to map the active indices buffer if it's not mapped yet
                    if active_indices.is_none() {
                        active_indices =
                            Some(NvFlexMap(self.active_indices, eNvFlexMapWait) as *mut i32);
                    }

                    active_indices
                        .unwrap_unchecked()
                        .add(self.num_active_particles) // num_active_particles is incremented here, so we don't need to add an offset
                        .write(offset as _);
                    self.num_active_particles += 1;
                }
            }

            // all particles are flushed to FleX, unmap the buffers now
            NvFlexUnmap(self.buffer);
            NvFlexUnmap(self.velocities);
            NvFlexUnmap(self.phases);
            if active_indices.is_some() {
                NvFlexUnmap(self.active_indices);
            }
        }

        // update the amount of spawned particles and clear the pending particle list
        self.num_particles += self.pending_particles.len();
        self.pending_particles.clear();

        // upload updated buffers to the solver
        unsafe {
            NvFlexSetParticles(solver, self.buffer, std::ptr::null_mut());
            NvFlexSetVelocities(solver, self.velocities, std::ptr::null_mut());
            NvFlexSetPhases(solver, self.phases, std::ptr::null_mut());
            NvFlexSetActive(solver, self.active_indices, std::ptr::null_mut());
            NvFlexSetActiveCount(solver, self.num_active_particles as _);
        }

        true
    }
}

impl Drop for ParticleSpawner {
    fn drop(&mut self) {
        unsafe {
            NvFlexFreeBuffer(self.buffer);
            NvFlexFreeBuffer(self.velocities);
            NvFlexFreeBuffer(self.phases);
            NvFlexFreeBuffer(self.active_indices);
        }
    }
}
