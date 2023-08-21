use bitflags::bitflags;
use cgmath::{Vector3, Vector4};
use nvflex_sys::*;
use std::{ops::RangeBounds, sync::atomic::AtomicPtr};

bitflags! {
    /// Flags that control a particle's behavior and grouping, use [`make_phase()`] or [`make_phase_with_channels()`] to construct a valid 32bit phase identifier.
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    pub struct PhaseFlags: i32 {
        /// Bits `[ 0, 19]` represent the particle group for controlling collisions.
        const GroupMask = 0x000fffff;
        /// Bits `[20, 23]` hold flags about how the particle behave.
        const FlagsMask = 0x00f00000;
        /// Bits `[24, 30]` hold flags representing what shape collision channels particles will collide with, see [`make_shape_flags()`] (highest bit reserved for now).
        const ShapeChannelMask = 0x7f000000;

        /// If set this particle will interact with particles of the same group.
        const SelfCollide = 1 << 20;
        // TODO: change the comment below
        /// If set this particle will ignore collisions with particles closer than the radius in the rest pose, this flag should not be specified unless valid rest positions have been specified using [`NvFlexSetRestParticles()`].
        const SelfCollideFilter = 1 << 21;
        /// If set this particle will generate fluid density constraints for its overlapping neighborsr = 1 << 21,.
        const Fluid = 1 << 22;
        /// Reserved.
        const Unused = 1 << 23;

        /// Particle will collide with shapes with channel 0 set (see [`make_shape_flags()`]).
        const ShapeChannel0 = 1 << 24;
        /// Particle will collide with shapes with channel 1 set (see [`make_shape_flags()`]).
        const ShapeChannel1 = 1 << 25;
        /// Particle will collide with shapes with channel 2 set (see [`make_shape_flags()`]).
        const ShapeChannel2 = 1 << 26;
        /// Particle will collide with shapes with channel 3 set (see [`make_shape_flags()`]).
        const ShapeChannel3 = 1 << 27;
        /// Particle will collide with shapes with channel 4 set (see [`make_shape_flags()`]).
        const ShapeChannel4 = 1 << 28;
        /// Particle will collide with shapes with channel 5 set (see [`make_shape_flags()`]).
        const ShapeChannel5 = 1 << 29;
        /// Particle will collide with shapes with channel 6 set (see [`make_shape_flags()`]).
        const ShapeChannel6 = 1 << 30;

        /// Equal to zero.
        const Zero = 0;
    }
}

/// Helper method to generates a phase with all shape channels set.
///
/// # Arguments
///
/// * `group` - The index of the group for this particle, should be an integer < 2^20.
/// * `particle_flags` - A combination of the phase flags which should be a combination of [`PhaseFlags::SelfCollide`], [`PhaseFlags::SelfCollideFilter`], and [`PhaseFlags::Fluid`].
///
/// # Example
///
/// ```
/// let phase = make_phase(
///     PhaseFlags::Zero, // particle group
///     PhaseFlags::SelfCollide | PhaseFlags::Fluid, // phase flags
/// );
/// ```
#[inline]
pub fn make_phase(group: PhaseFlags, particle_flags: PhaseFlags) -> i32 {
    // this is not an FFI call, this is an inline function is implemented in Rust by the `nvflex_sys` crate
    NvFlexMakePhase(group.bits(), particle_flags.bits())
}

/// Generate a bit set for the particle phase, this is a helper method to simply combine the group id and bit flags into a single integer.
///
/// # Arguments
/// * `group` - The index of the group for this particle, should be an integer < 2^20.
/// * `particle_flags` - A combination of the phase flags which should be a combination of [`PhaseFlags::SelfCollide`], [`PhaseFlags::SelfCollideFilter`], and [`PhaseFlags::Fluid`].
/// * `shape_channels` - A combination of [`PhaseFlags::ShapeChannel0`] - [`PhaseFlags::ShapeChannel6`] flags that control which shapes will be collided against, particles will only collide against shapes that share at least one set channel, see `make_shape_flags_with_channels()`.
///
/// # Example
///
/// ```
/// let phase = make_phase_with_channels(
///     PhaseFlags::Zero, // particle group
///     PhaseFlags::SelfCollide | PhaseFlags::Fluid, // phase flags
///     PhaseFlags::ShapeChannel0 | PhaseFlags::ShapeChannel2, // shape channels
/// );
/// ```
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
    /// Flags that control a particle's behavior and grouping, use [`make_phase()`] or [`make_phase_with_channels()`] to construct a valid 32bit phase identifier.
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
#[derive(Debug)]
pub struct World {
    /// Particles pending to be sent to FleX by `flush()`.
    pub pending_particles: Vec<Particle>,
    /// Maximum amount of particles that can be spawned.
    max_particles: usize,
    /// Amount of spawned particles. This includes particles that are not active and doesn't include pending particles.
    num_particles: usize,
    /// Amount of active particles.
    num_active_particles: usize,
    /// Holds the particle positions. x = x position, y = y position, z = z position, w = mass.
    pub buffer: AtomicPtr<NvFlexBuffer>,
    /// Holds the particle velocities.
    pub velocities: AtomicPtr<NvFlexBuffer>,
    /// Each particle has an associated phase id which controls how it interacts with other particles.
    ///
    /// Use [`make_phase()`] or [`make_phase_with_channels()`] to construct a valid 32bit phase identifier.
    pub phases: AtomicPtr<NvFlexBuffer>,
    /// Holds the indices of particles that have been made active.
    pub active_indices: AtomicPtr<NvFlexBuffer>,
    pub solver: AtomicPtr<NvFlexSolver>,
}

impl World {
    pub fn new(lib: *mut NvFlexLibrary, solver: *mut NvFlexSolver, max_particles: i32) -> Self {
        // max_particles cannot be negative
        let max_particles = if max_particles < 0 { 0 } else { max_particles };

        unsafe {
            Self {
                pending_particles: vec![],
                max_particles: max_particles as _,
                num_particles: 0,
                num_active_particles: 0,
                buffer: AtomicPtr::new(NvFlexAllocBuffer(
                    lib,
                    max_particles,
                    std::mem::size_of::<Vector4<f32>>() as _,
                    eNvFlexBufferHost,
                )),
                velocities: AtomicPtr::new(NvFlexAllocBuffer(
                    lib,
                    max_particles,
                    std::mem::size_of::<Vector3<f32>>() as _,
                    eNvFlexBufferHost,
                )),
                phases: AtomicPtr::new(NvFlexAllocBuffer(
                    lib,
                    max_particles,
                    std::mem::size_of::<i32>() as _,
                    eNvFlexBufferHost,
                )),
                active_indices: AtomicPtr::new(NvFlexAllocBuffer(
                    lib,
                    max_particles,
                    std::mem::size_of::<i32>() as _,
                    eNvFlexBufferHost,
                )),
                solver: AtomicPtr::new(solver), // TODO: is this safe?
            }
        }
    }

    /// Amount of spawned particles. This includes particles that are not active and doesn't include pending particles.
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

    #[inline]
    pub unsafe fn solver_ptr(&self) -> *mut NvFlexSolver {
        *self.solver.as_ptr()
    }

    #[inline]
    pub unsafe fn buffer_ptr(&self) -> *mut NvFlexBuffer {
        *self.buffer.as_ptr()
    }

    #[inline]
    pub unsafe fn velocities_ptr(&self) -> *mut NvFlexBuffer {
        *self.velocities.as_ptr()
    }

    #[inline]
    pub unsafe fn phases_ptr(&self) -> *mut NvFlexBuffer {
        *self.phases.as_ptr()
    }

    #[inline]
    pub unsafe fn active_indices_ptr(&self) -> *mut NvFlexBuffer {
        *self.active_indices.as_ptr()
    }

    /// Spaws a new particle, but doesn't apply the changes immediately. Calling `self.flush()` will process all pending changes.
    ///
    /// Returns the index of the particle, or [`None`] if the particle limit is reached.
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
    ) -> Option<usize> {
        let num_pending = self.num_pending();
        if self.num_particles() + num_pending < self.max_particles {
            self.pending_particles
                .push(Particle::new(pos, vel, phase, active));
            return Some(self.num_particles + num_pending);
        }
        None
    }

    /// Flushes all pending particles to the FleX solver. Returns whether any changes were applied or not.
    pub fn flush(&mut self) -> bool {
        unsafe {
            if self.pending_particles.is_empty() || self.solver_ptr().is_null() {
                return false;
            }

            let buffer = NvFlexMap(self.buffer_ptr(), eNvFlexMapWait) as *mut Vector4<f32>;
            let velocities = NvFlexMap(self.velocities_ptr(), eNvFlexMapWait) as *mut Vector3<f32>;
            let phases = NvFlexMap(self.phases_ptr(), eNvFlexMapWait) as *mut i32;
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
                            Some(NvFlexMap(self.active_indices_ptr(), eNvFlexMapWait) as *mut i32);
                    }

                    active_indices
                        .unwrap_unchecked()
                        .add(self.num_active_particles) // num_active_particles is incremented here, so we don't need to add an offset
                        .write(offset as _);
                    self.num_active_particles += 1;
                }
            }

            // all particles are copied into the buffers, unmap them now
            NvFlexUnmap(self.buffer_ptr());
            NvFlexUnmap(self.velocities_ptr());
            NvFlexUnmap(self.phases_ptr());
            if active_indices.is_some() {
                NvFlexUnmap(self.active_indices_ptr());
            }
        }

        // update the amount of spawned particles and clear the pending particle list
        self.num_particles += self.pending_particles.len();
        self.pending_particles.clear();

        // upload updated buffers to the solver
        unsafe {
            NvFlexSetParticles(self.solver_ptr(), self.buffer_ptr(), std::ptr::null_mut());
            NvFlexSetVelocities(
                self.solver_ptr(),
                self.velocities_ptr(),
                std::ptr::null_mut(),
            );
            NvFlexSetPhases(self.solver_ptr(), self.phases_ptr(), std::ptr::null_mut());
            NvFlexSetActive(
                self.solver_ptr(),
                self.active_indices_ptr(),
                std::ptr::null_mut(),
            );
            NvFlexSetActiveCount(self.solver_ptr(), self.num_active_particles as _);
        }

        true
    }

    /// Returns a slice of all particles in a specified range. This will map and unmap particle buffers.
    ///
    /// # Arguments
    ///
    /// * `range` - Range of particles to get.
    ///
    /// # Example
    ///
    /// ```
    /// let flex = FlexContext::new(None, None).unwrap();
    /// let first_3_particles = flex.spawner().get_particles_range(0..3);
    /// ```
    pub fn get_particles_range<R>(&self, range: R) -> Vec<Particle>
    where
        R: RangeBounds<usize>,
    {
        use std::ops::Bound::*;
        let num_particles = self.num_particles();

        // TODO: is this correct?
        let start = match range.start_bound() {
            Unbounded => 0,
            Included(idx) => (*idx).clamp(0, num_particles),
            Excluded(idx) => (*idx).clamp(1, num_particles),
        };
        let end = match range.end_bound() {
            Unbounded => num_particles,
            Included(idx) => (*idx).clamp(0, num_particles - 1),
            Excluded(idx) => (*idx).clamp(0, num_particles),
        };
        if end - start == 0 {
            return vec![];
        }

        unsafe {
            // map buffers
            let buffer = NvFlexMap(self.buffer_ptr(), eNvFlexMapWait) as *mut Vector4<f32>;
            let velocities = NvFlexMap(self.velocities_ptr(), eNvFlexMapWait) as *mut Vector3<f32>;
            let phases = NvFlexMap(self.phases_ptr(), eNvFlexMapWait) as *mut i32;
            // let active_indices = NvFlexMap(self.active_indices_ptr(), eNvFlexMapWait) as *mut i32;

            let mut particles: Vec<Particle> = Vec::with_capacity(end - start);
            for i in start..end {
                let offset = i;
                particles.push(Particle {
                    pos: *buffer.add(offset),
                    vel: *velocities.add(offset),
                    phase: *phases.add(offset),
                    active: true, // TODO: how do we get this?
                });
            }

            NvFlexUnmap(self.buffer_ptr());
            NvFlexUnmap(self.velocities_ptr());
            NvFlexUnmap(self.phases_ptr());
            // NvFlexUnmap(self.active_indices_ptr());

            particles
        }
    }

    /// Returns a slice of all particles. This will map and unmap particle buffers.
    ///
    /// # Example
    ///
    /// ```
    /// let flex = FlexContext::new(None, None).unwrap();
    /// let all_particles = flex.spawner().get_particles();
    /// ```
    #[inline]
    pub fn get_particles(&self) -> Vec<Particle> {
        self.get_particles_range(..)
    }

    pub fn read_buffers(&self) {
        unsafe {
            NvFlexGetParticles(self.solver_ptr(), self.buffer_ptr(), std::ptr::null());
            NvFlexGetVelocities(self.solver_ptr(), self.velocities_ptr(), std::ptr::null());
            NvFlexGetPhases(self.solver_ptr(), self.phases_ptr(), std::ptr::null());
            /* NvFlexGetActive(
                self.solver_ptr(),
                self.active_indices_ptr(),
                std::ptr::null(),
            ); */
        }
    }
}

impl Drop for World {
    fn drop(&mut self) {
        unsafe {
            NvFlexFreeBuffer(self.buffer_ptr());
            NvFlexFreeBuffer(self.velocities_ptr());
            NvFlexFreeBuffer(self.phases_ptr());
            NvFlexFreeBuffer(self.active_indices_ptr());
        }
    }
}
