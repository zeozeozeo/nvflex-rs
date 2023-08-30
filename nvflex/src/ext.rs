//! This file is only is only included if the `ext` feature is enabled.

use crate::{Library, Solver};
use nvflex_sys::*;

/// Helper struct for storing the state of a moving frame, see NvFlexExtMovingFrameInit()
#[derive(Debug, Clone, Copy, Default)]
#[repr(C)]
pub struct MovingFrame {
    pub position: [f32; 3],
    pub rotation: [f32; 4],

    pub velocity: [f32; 3],
    pub omega: [f32; 3],

    pub acceleration: [f32; 3],
    pub tau: [f32; 3],

    pub delta: [[f32; 4]; 4],
}

impl MovingFrame {
    // TODO: translate the C example to Rust
    /// Creates a new moving frame struct. This helper method is used to calculate inertial forces for particles
    /// inside an attached parent frame. For example, when simulating cloth attached to the character, we would like to perform
    /// a local space simulation of the cloth to avoid excessive stretching and collision issues during fast animations.
    /// However, we would still like the cloth to respond to character movements in at least a limited, or controlled fashion.
    ///
    /// The [`MovingFrame`] provides a way to include or remove these inertial forces. The basic usage is as follows:
    ///
    /// ```c
    /// NvFlexExtMovingFrame frame;
    /// NvFlexExtMovingFrameInit(&frame, initialTranslation, initialRotation);
    /// const linearInertiaScale = 0.25f;
    /// const angularInertiaScale 0.5;
    /// while(simulating)
    /// {
    ///     float3 newPosition;
    ///     float4 newRotation;
    ///     
    ///     // move parent frame (character / emitter) according to application's animation system
    ///     Animate(newPosition, newRotation);
    ///     
    ///     // update the frame
    ///     NvFlexExtMovingFrameUpdate(frame, newPosition, newRotation, dt);
    ///     // apply inertial forces and update particles
    ///     NvFlexExtMovingFrameApply(frame, particlePositions, particleVelocities, numParticles, linearInertiaScale, angularInertiaScale, dt);
    /// }
    /// ```
    ///
    /// # Parameters
    ///
    /// * `world_translation` - A vec3 storing the frame's initial translation in world space
    /// * `world_rotation` - A quaternion storing the frame's initial rotation in world space
    ///
    /// # Returns
    ///
    /// The [`MovingFrame`] struct
    #[inline]
    pub fn new(world_translation: [f32; 3], world_rotation: [f32; 4]) -> Self {
        unsafe {
            let mut frame = MovingFrame::default();
            NvFlexExtMovingFrameInit(
                &mut frame as *mut _ as *mut _,
                world_translation.as_ptr(),
                world_rotation.as_ptr(),
            );
            frame
        }
    }

    /// Update a frame to a new position, this will automatically update the velocity and acceleration of
    /// the frame, which can then be used to calculate inertial forces. This should be called once per-frame
    /// with the new position and time-step used when moving the frame.
    ///
    /// # Parameters
    ///
    /// * `world_translation` - A vec3 storing the frame's initial translation in world space
    /// * `world_rotation` - A quaternion storing the frame's initial rotation in world space
    /// * `dt` - The time that elapsed since the last call to the frame update
    #[inline]
    pub fn update(&mut self, world_translation: [f32; 3], world_rotation: [f32; 4], dt: f32) {
        unsafe {
            NvFlexExtMovingFrameUpdate(
                self as *mut _ as *mut _,
                world_translation.as_ptr(),
                world_rotation.as_ptr(),
                dt,
            )
        }
    }

    /// Teleport particles to the frame's new position and apply the inertial forces
    ///
    /// # Parameters
    ///
    /// * `positions` - A reference to an array of particle positions in (x, y, z, 1/m) format
    /// * `velocities` - A reference to an array of particle velocities in (vx, vy, vz) format
    /// * `num_particles` - The number of particles to update
    /// * `linear_scale` - How strongly the translational inertial forces should be applied, 0.0 corresponds to a purely local space simulation removing all inertial forces, 1.0 corresponds to no inertial damping and has no benefit over regular world space simulation
    /// * `angular_scale` - How strongly the angular inertial forces should be applied, 0.0 corresponds to a purely local space simulation, 1.0 corresponds to no inertial damping
    /// * `dt` - The time that elapsed since the last call to the frame update, should match the value passed to `update()`
    #[inline]
    pub fn apply(
        &mut self,
        positions: &mut [f32; 4],
        velocities: &mut [f32; 3],
        num_particles: i32,
        linear_scale: f32,
        angular_scale: f32,
        dt: f32,
    ) {
        unsafe {
            NvFlexExtMovingFrameApply(
                self as *mut _ as *mut _,
                positions.as_mut_ptr(),
                velocities.as_mut_ptr(),
                num_particles,
                linear_scale,
                angular_scale,
                dt,
            )
        }
    }
}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Send for MovingFrame {}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Sync for MovingFrame {}

/// Create an index buffer of unique vertices in the mesh (collapses vertices in the same position even if they have different normals / texcoords).
/// This can be used to create simulation meshes from render meshes, and is typically done as a pre-pass before calling NvFlexExtCreateClothFromMesh().
///
/// # Parameters
///
/// * `vertices` - An array of float3 positions
/// * `unique_verts` - A list of unique mesh vertex indices, should be `num_vertices` in length (worst case all verts are unique)
/// * `original_to_unique_map` - Mapping from the original vertex index to the unique vertex index, should be `num_vertices` in length
/// * `threshold` - The distance below which two vertices are considered duplicates
///
/// # Returns
///
/// The number of unique vertices in the mesh
#[inline]
pub fn create_welded_mesh_indices(
    vertices: &[f32],
    unique_verts: &mut [i32],
    original_to_unique_map: &mut [i32],
    threshold: f32,
) -> i32 {
    assert!(unique_verts.len() >= vertices.len() && original_to_unique_map.len() >= vertices.len());
    unsafe {
        NvFlexExtCreateWeldedMeshIndices(
            vertices.as_ptr(),
            vertices.len() as _,
            unique_verts.as_mut_ptr(),
            original_to_unique_map.as_mut_ptr(),
            threshold,
        )
    }
}

/// Represents a group of particles and constraints, each asset
/// can be instanced into a container using NvFlexExtCreateInstance()
#[derive(Debug)]
pub struct Asset {
    pub(crate) asset: *mut NvFlexExtAsset,
    pub(crate) is_tearing_cloth: bool,
}

/// Particles and vertices may need to be copied during tearing. Because the user may maintain particle data
/// outside of Flex, this structure describes how to update the particle data. The application should copy each
/// existing particle given by `src_index` (in the asset's particle array) to the `dest_index` (also in the asset's array).
#[derive(Debug, Clone, Copy, Default)]
#[repr(C)]
pub struct TearingParticleClone {
    pub src_index: i32,
    pub dest_index: i32,
}

/// The mesh topology may need to be updated to reference new particles during tearing. Because the user may maintain
/// mesh topology outside of Flex, this structure describes the necessary updates that should be performed on the mesh.
/// The `tri_index` member is the index of the index to be updated, e.g.:
/// a `tri_index` value of 4 refers to the index 1 vertex (4%3) of the index 1 triangle (4/3). This entry in the indices
/// array should be updated to point to the `new_particle_index`.
#[derive(Debug, Clone, Copy, Default)]
#[repr(C)]
pub struct TearingMeshEdit {
    /// Index into the triangle indices array to update
    pub tri_index: i32,
    /// New value for the index
    pub new_particle_index: i32,
}

impl Asset {
    /// Create a cloth asset consisting of stretch and bend distance constraints given an indexed triangle mesh. Stretch constraints will be placed along
    /// triangle edges, while bending constraints are placed over two edges.
    ///
    /// # Parameters
    ///
    /// * `particles` - Positions and masses of the particles in the format `[x, y, z, 1/m]`
    /// * `indices` - The triangle indices, these should be 'welded' using [`create_welded_mesh_indices()`] first
    /// * `stretch_stiffness` - The stiffness coefficient for stretch constraints
    /// * `bend_stiffness` - The stiffness coefficient used for bending constraints
    /// * `tether_stiffness` - If > 0.0f then the function will create tethers attached to particles with zero inverse mass. These are unilateral, long-range attachments, which can greatly reduce stretching even at low iteration counts.
    /// * `tether_give` - Because tether constraints are so effective at reducing stiffness, it can be useful to allow a small amount of extension before the constraint activates.
    /// * `pressure` - If > 0.0f then a volume (pressure) constraint will also be added to the asset, the rest volume and stiffness will be automatically computed by this function
    ///
    /// # Returns
    ///
    /// A pointer to an asset structure holding the particles and constraints
    #[inline]
    pub fn create_cloth_from_mesh(
        particles: &[f32],
        indices: &[i32],
        stretch_stiffness: f32,
        bend_stiffness: f32,
        tether_stiffness: f32,
        tether_give: f32,
        pressure: f32,
    ) -> Self {
        unsafe {
            Self {
                asset: NvFlexExtCreateClothFromMesh(
                    particles.as_ptr(),
                    particles.len() as _,
                    indices.as_ptr(),
                    indices.len() as _,
                    stretch_stiffness,
                    bend_stiffness,
                    tether_stiffness,
                    tether_give,
                    pressure,
                ),
                is_tearing_cloth: false,
            }
        }
    }

    /// Create a cloth asset consisting of stretch and bend distance constraints given an indexed triangle mesh. This creates an asset with the same
    /// structure as NvFlexExtCreateClothFromMesh(), however tether constraints are not supported, and additional information regarding mesh topology
    /// will be stored with the asset to allow tearing.
    /// @note: Typically each instance of a tearable cloth mesh will have it's own asset. This is because the asset holds the topology of the mesh which is
    /// unique for each instance.
    ///
    /// # Parameters
    ///
    /// * `particles` - Positions and masses of the particles in the format `[x, y, z, 1/m]`
    /// * `max_particles` - The maximum number of particles for this asset, this will limit the amount of tearing that can be performed.
    /// * `indices` - The triangle indices, these should be 'welded' using [`create_welded_mesh_indices()`] first
    /// * `num_triangles` - The number of triangles
    /// * `stretch_stiffness` - The stiffness coefficient for stretch constraints
    /// * `bend_stiffness` - The stiffness coefficient used for bending constraints
    /// * `pressure` - If > 0.0f then a volume (pressure) constraint will also be added to the asset, the rest volume and stiffness will be automatically computed by this function
    ///
    /// # Returns
    ///
    /// A pointer to an asset structure holding the particles and constraints
    #[inline]
    pub fn create_tearing_cloth_from_mesh(
        particles: &[f32],
        max_particles: i32,
        indices: &[i32],
        num_triangles: i32,
        stretch_stiffness: f32,
        bend_stiffness: f32,
        pressure: f32,
    ) -> Self {
        unsafe {
            Self {
                asset: NvFlexExtCreateTearingClothFromMesh(
                    particles.as_ptr(),
                    particles.len() as _,
                    max_particles,
                    indices.as_ptr(),
                    num_triangles,
                    stretch_stiffness,
                    bend_stiffness,
                    pressure,
                ),
                is_tearing_cloth: true,
            }
        }
    }

    /// Perform cloth mesh tearing, this function will calculate the strain on each distance constraint and perform splits if it is
    /// above a certain strain threshold (i.e.: length/rest_length > max_strain).
    ///
    /// # Parameters
    ///
    /// * `max_strain` - The maximum allowable strain on each edge
    /// * `max_splits` - The maximum number of constraint breaks that will be performed, this controls the 'rate' of mesh tearing
    /// * `particle_copies` - Reference to an array of [`TearingParticleClone`] structures that describe the particle copies that need to be performed
    /// * `triangle_edits` - Reference to an array of [`TearingMeshEdit`] structures that describe the topology updates that need to be performed
    ///
    /// # Returns
    ///
    /// * `.0` - The number of particle copies performed written to the `particle_copies` array (`num_particle_copies`).
    /// * `.1` - The number of topology updates written to the `triangle_edits` array (`num_triangle_edits`).
    #[inline]
    pub fn tear_cloth_mesh(
        &self,
        max_strain: f32,
        max_splits: i32,
        particle_copies: &mut [TearingParticleClone],
        triangle_edits: &mut [TearingMeshEdit],
    ) -> (i32, i32) {
        unsafe {
            let mut num_particle_copies: i32 = 0;
            let mut num_triangle_edits: i32 = 0;
            NvFlexExtTearClothMesh(
                self.asset,
                max_strain,
                max_splits,
                particle_copies.as_mut_ptr() as *mut _,
                &mut num_particle_copies,
                particle_copies.len() as _,
                triangle_edits.as_mut_ptr() as *mut _,
                &mut num_triangle_edits,
                triangle_edits.len() as _,
            );
            (num_particle_copies, num_triangle_edits)
        }
    }

    /// Create a shape body asset from a closed triangle mesh. The mesh is first voxelized at a spacing specified by the radius, and particles are placed at occupied voxels.
    ///
    /// # Parameters
    ///
    /// * `vertices` - Vertices of the triangle mesh
    /// * `indices` - The triangle indices
    /// * `radius` - The spacing used for voxelization, note that the number of voxels grows proportional to the inverse cube of radius, currently this method limits construction to resolutions < 64^3
    /// * `expand` - Particles will be moved inwards (if negative) or outwards (if positive) from the surface of the mesh according to this factor
    ///
    /// # Returns
    ///
    /// An asset structure holding the particles and constraints
    #[inline]
    pub fn create_rigid_from_mesh(
        vertices: &[f32],
        indices: &[i32],
        radius: f32,
        expand: f32,
    ) -> Self {
        unsafe {
            Self {
                asset: NvFlexExtCreateRigidFromMesh(
                    vertices.as_ptr(),
                    vertices.len() as _,
                    indices.as_ptr(),
                    indices.len() as _,
                    radius,
                    expand,
                ),
                is_tearing_cloth: false,
            }
        }
    }

    /// Create a shape body asset from a closed triangle mesh. The mesh is first voxelized at a spacing specified by the radius, and particles are placed at occupied voxels.
    ///
    /// # Parameters
    ///
    /// * `vertices` - Vertices of the triangle mesh
    /// * `indices` - The triangle indices
    /// * `particle_spacing` - The spacing to use when creating particles
    /// * `volume_sampling` - Control the resolution the mesh is voxelized at in order to generate interior sampling, if the mesh is not closed then this should be set to zero and surface sampling should be used instead
    /// * `surface_sampling` - Controls how many samples are taken of the mesh surface, this is useful to ensure fine features of the mesh are represented by particles, or if the mesh is not closed
    /// * `cluster_spacing` - The spacing for shape-matching clusters, should be at least the particle spacing
    /// * `cluster_radius` - Controls the overall size of the clusters, this controls how much overlap the clusters have which affects how smooth the final deformation is, if parts of the body are detaching then it means the clusters are not overlapping sufficiently to form a fully connected set of clusters
    /// * `cluster_stiffness` - Controls the stiffness of the resulting clusters
    /// * `link_radius` - Any particles below this distance will have additional distance constraints created between them
    /// * `link_stiffness` - The stiffness of distance links
    /// * `global_stiffness` - If this parameter is > 0.0f, adds an additional global cluster that consists of all particles in the shape. The stiffness of this cluster is the `global_stiffness`.
    /// * `cluster_plastic_threshold` - Particles belonging to rigid shapes that move with a position delta magnitude > threshold will be permanently deformed in the rest pose, if `cluster_plastic_creep` > 0.0f
    /// * `cluster_plastic_creep` - Controls the rate at which particles in the rest pose are deformed for particles passing the deformation threshold
    ///
    /// # Returns
    ///
    /// A pointer to an asset structure holding the particles and constraints
    #[inline]
    pub fn create_soft_from_mesh(
        vertices: &[f32],
        indices: &[i32],
        particle_spacing: f32,
        volume_sampling: f32,
        surface_sampling: f32,
        cluster_spacing: f32,
        cluster_radius: f32,
        cluster_stiffness: f32,
        link_radius: f32,
        link_stiffness: f32,
        global_stiffness: f32,
        cluster_plastic_threshold: f32,
        cluster_plastic_creep: f32,
    ) -> Self {
        unsafe {
            Self {
                asset: NvFlexExtCreateSoftFromMesh(
                    vertices.as_ptr(),
                    vertices.len() as _,
                    indices.as_ptr(),
                    indices.len() as _,
                    particle_spacing,
                    volume_sampling,
                    surface_sampling,
                    cluster_spacing,
                    cluster_radius,
                    cluster_stiffness,
                    link_radius,
                    link_stiffness,
                    global_stiffness,
                    cluster_plastic_threshold,
                    cluster_plastic_creep,
                ),
                is_tearing_cloth: false,
            }
        }
    }
    // particles
    /// Local space particle positions, x,y,z,1/mass
    #[inline]
    pub fn particles(&self) -> *mut f32 {
        unsafe { (*self.asset).particles }
    }
    /// Number of particles
    #[inline]
    pub fn num_particles(&self) -> i32 {
        unsafe { (*self.asset).numParticles }
    }
    /// Maximum number of particles, allows extra space for tearable assets which duplicate particles
    #[inline]
    pub fn max_particles(&self) -> i32 {
        unsafe { (*self.asset).maxParticles }
    }

    // springs

    /// Spring indices
    #[inline]
    pub fn spring_indices(&self) -> *mut i32 {
        unsafe { (*self.asset).springIndices }
    }
    /// Spring coefficients
    #[inline]
    pub fn spring_coefficients(&self) -> *mut f32 {
        unsafe { (*self.asset).springCoefficients }
    }
    /// Spring rest-lengths
    #[inline]
    pub fn spring_rest_lengths(&self) -> *mut f32 {
        unsafe { (*self.asset).springRestLengths }
    }
    /// Number of springs
    #[inline]
    pub fn num_springs(&self) -> i32 {
        unsafe { (*self.asset).numSprings }
    }
    // shapes

    /// The indices of the shape matching constraints
    #[inline]
    pub fn shape_indices(&self) -> *mut i32 {
        unsafe { (*self.asset).shapeIndices }
    }
    /// Total number of indices for shape constraints
    #[inline]
    pub fn num_shape_indices(&self) -> i32 {
        unsafe { (*self.asset).numShapeIndices }
    }
    /// Each entry stores the end of the shape's indices in the indices array (exclusive prefix sum of shape lengths)
    #[inline]
    pub fn shape_offsets(&self) -> *mut i32 {
        unsafe { (*self.asset).shapeOffsets }
    }
    /// The stiffness coefficient for each shape
    #[inline]
    pub fn shape_coefficients(&self) -> *mut f32 {
        unsafe { (*self.asset).shapeCoefficients }
    }
    /// The position of the center of mass of each shape, an array of vec3s mNumShapes in length
    #[inline]
    pub fn shape_centers(&self) -> *mut f32 {
        unsafe { (*self.asset).shapeCenters }
    }
    /// The number of shape matching constraints
    #[inline]
    pub fn num_shapes(&self) -> i32 {
        unsafe { (*self.asset).numShapes }
    }

    // plastic deformation

    /// The plastic threshold coefficient for each shape
    #[inline]
    pub fn shape_plastic_thresholds(&self) -> *mut f32 {
        unsafe { (*self.asset).shapePlasticThresholds }
    }
    /// The plastic creep coefficient for each shape
    #[inline]
    pub fn shape_plastic_creeps(&self) -> *mut f32 {
        unsafe { (*self.asset).shapePlasticCreeps }
    }

    // faces for cloth

    /// Indexed triangle mesh indices for clothing
    #[inline]
    pub fn triangle_indices(&self) -> *mut i32 {
        unsafe { (*self.asset).triangleIndices }
    }
    /// Number of triangles
    #[inline]
    pub fn num_triangles(&self) -> i32 {
        unsafe { (*self.asset).numTriangles }
    }

    // inflatable params

    /// Whether an inflatable constraint should be added
    #[inline]
    pub fn inflatable(&self) -> bool {
        unsafe { (*self.asset).inflatable }
    }
    /// The rest volume for the inflatable constraint
    #[inline]
    pub fn inflatable_volume(&self) -> f32 {
        unsafe { (*self.asset).inflatableVolume }
    }
    /// How much over the rest volume the inflatable should attempt to maintain
    #[inline]
    pub fn inflatable_pressure(&self) -> f32 {
        unsafe { (*self.asset).inflatablePressure }
    }
    /// How stiff the inflatable is
    #[inline]
    pub fn inflatable_stiffness(&self) -> f32 {
        unsafe { (*self.asset).inflatableStiffness }
    }
}

impl Drop for Asset {
    fn drop(&mut self) {
        unsafe {
            // TODO: is it necessary that we call different functions for destroying the asset?
            if self.is_tearing_cloth {
                NvFlexExtDestroyTearingCloth(self.asset);
            } else {
                NvFlexExtDestroyAsset(self.asset);
            }
        }
    }
}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Send for Asset {}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Sync for Asset {}

/// Creates information for linear blend skining a graphics mesh to a set of transforms (bones)
///
/// # Parameters
///
/// * `vertices` - Vertices of the triangle mesh
/// * `bones` - Pointer to an array of vec3 positions representing the bone positions
/// * `falloff` - The speed at which the bone's influence on a vertex falls off with distance
/// * `max_distance` - The maximum distance a bone can be from a vertex before it will not influence it any more
/// * `skinning_weights` - The normalized weights for each bone, there are up to 4 weights per-vertex so this should be `num_vertices`*4 in length
/// * `skinning_indices` - The indices of each bone corresponding to the skinning weight, will be -1 if this weight is not used
#[inline]
pub fn create_soft_mesh_skinning(
    vertices: &[f32],
    bones: &[f32],
    falloff: f32,
    max_distance: f32,
    skinning_weights: &mut [f32],
    skinning_indices: &mut [i32],
) {
    let num_skinning = vertices.len() * 4;
    assert!(skinning_weights.len() >= num_skinning && skinning_indices.len() >= num_skinning);

    unsafe {
        NvFlexExtCreateSoftMeshSkinning(
            vertices.as_ptr(),
            vertices.len() as _,
            bones.as_ptr(),
            bones.len() as _,
            falloff,
            max_distance,
            skinning_weights.as_mut_ptr(),
            skinning_indices.as_mut_ptr(),
        )
    }
}

#[derive(Debug)]
#[repr(C)]
pub struct ShapeData {
    /// Receives a pointer to the array quaternion rotation data in `[x, y, z, w]` format
    pub rotations: *mut f32,
    /// Receives a pointer to an array of shape body translations in `[x, y, z]` format
    pub positions: *mut f32,
    /// Number of valid tranforms
    pub n: i32,
}

#[derive(Debug)]
#[repr(C)]
pub struct ParticleData {
    /// Receives a pointer to the particle position / mass data
    pub particles: *mut f32,
    /// Receives a pointer to the particle's rest position (used for self collision culling)
    pub rest_particles: *mut f32,
    /// Receives a pointer to the particle velocity data
    pub velocities: *mut f32,
    /// Receives a pointer to the particle phase data
    pub phases: *mut i32,
    /// Receives a pointer to the particle normal data with 16 byte stride in format `[nx, ny, nz, nw]`
    pub normals: *mut f32,

    /// Receive a pointer to the particle lower bounds `[x, y, z]`
    pub lower: *const f32,
    /// Receive a pointer to the particle upper bounds `[x, y, z]`
    pub upper: *const f32,
}

/// Opaque type representing a simulation
#[derive(Debug)]
pub struct Container {
    pub(crate) container: *mut NvFlexExtContainer,
}

impl Container {
    /// Creates a wrapper object around a Flex solver that can hold assets / instances, the container manages sending and retrieving partical data from the solver
    ///
    /// # Parameters
    ///
    /// * `lib` - The library instance to use
    /// * `solver` - The solver to wrap
    /// * `max_particles` - The maximum number of particles to manage
    ///
    /// # Returns
    ///
    /// A new container
    #[inline]
    pub fn new(lib: Library, solver: &Solver, max_particles: i32) -> Self {
        unsafe {
            Self {
                container: NvFlexExtCreateContainer(lib.lib, solver.solver, max_particles),
            }
        }
    }

    /// Allocates particles in the container.
    ///
    /// # Parameters
    ///
    /// * `n` - The number of particles to allocate
    /// * `indices` - An n-length array of ints that will store the indices to the allocated particles
    #[inline]
    pub fn alloc_particles(&self, n: i32, indices: &mut [i32]) -> i32 {
        assert!(indices.len() >= n as usize);
        unsafe { NvFlexExtAllocParticles(self.container, n, indices.as_mut_ptr()) }
    }

    /// Free allocated particles
    ///
    /// # Parameters
    ///
    /// * `n` - The number of particles to free
    /// * `indices` - The indices of the particles to free
    #[inline]
    pub fn free_particles(&self, n: i32, indices: &[i32]) {
        unsafe { NvFlexExtFreeParticles(self.container, n, indices.as_ptr()) }
    }

    /// Retrives the indices of all active particles
    ///
    /// # Parameters
    ///
    /// * `indices` - Returns the number of active particles (should be at least the maximum number of particles)
    ///
    /// # Returns
    ///
    /// The number of active particles
    #[inline]
    pub fn get_active_list(&self, indices: &mut [i32]) -> i32 {
        unsafe { NvFlexExtGetActiveList(self.container, indices.as_mut_ptr()) }
    }

    /// Returns pointers to the internal data stored by the container. These are host-memory pointers, and will
    /// remain valid `unmap_particle_data()` is called.
    #[inline]
    pub fn map_particle_data(&self) -> ParticleData {
        unsafe { std::mem::transmute(NvFlexExtMapParticleData(self.container)) }
    }

    #[inline]
    pub fn unmap_particle_data(&self) {
        unsafe { NvFlexExtUnmapParticleData(self.container) }
    }

    /// Access shape body constraint data, see NvFlexExtGetParticleData() for notes on ownership.
    #[inline]
    pub fn map_shape_data(&self) -> ShapeData {
        unsafe { std::mem::transmute(NvFlexExtMapShapeData(self.container)) }
    }

    /// Unmap shape transform data, see `map_shape_data()`
    #[inline]
    pub fn unmap_shape_data(&self) {
        unsafe { NvFlexExtUnmapParticleData(self.container) }
    }

    /// Notifies the container that asset data has changed and needs to be sent to the GPU
    /// this should be called if the constraints for an existing asset are modified by the user
    ///
    /// # Parameters
    ///
    /// * `asset` - The asset which was modified (can be [`None`])
    #[inline]
    pub fn notify_asset_changed(&self, asset: Option<&Asset>) {
        unsafe {
            NvFlexExtNotifyAssetChanged(
                self.container,
                if let Some(asset) = asset {
                    asset.asset
                } else {
                    std::ptr::null()
                },
            )
        }
    }

    // TODO: rewrite the C code example in Rust
    /// Updates the container, applies force fields, steps the solver forward in time, updates the host with the results synchronously.
    /// This is a helper function which performs a synchronous update using the following flow.
    ///
    /// ```c
    /// // async update GPU data
    /// NvFlexExtPushToDevice(container);
    /// // update solver
    /// NvFlexUpdateSolver(container, dt, iterations);
    /// // async read data back to CPU
    /// NvFlexExtPullFromDevice(container);
    /// // read / write particle data on CPU
    /// NvFlexExtParticleData data = NvFlexExtMapParticleData();
    /// // CPU particle processing
    /// ProcessParticles(data);
    /// // unmap data
    /// NvFlexExtUnmapParticleData();
    /// ```
    ///
    /// # Parameters
    ///
    /// * `dt` - The time-step in seconds
    /// * `num_substeps` - The number of substeps to perform
    /// * `enable_timers` - Whether to record detailed timers, see NvFlexUpdateSolver()
    #[inline]
    pub fn tick(&self, dt: f32, num_substeps: i32, enable_timers: bool) {
        unsafe { NvFlexExtTickContainer(self.container, dt, num_substeps, enable_timers) }
    }

    /// Updates the device asynchronously, transfers any particle and constraint changes to the flex solver,
    /// expected to be called in the following sequence: NvFlexExtPushToDevice, NvFlexUpdateSolver, NvFlexExtPullFromDevice, flexSynchronize
    #[inline]
    pub fn push_to_device(&self) {
        unsafe { NvFlexExtPushToDevice(self.container) }
    }

    /// Updates the host asynchronously, transfers particle and constraint data back to he host,
    /// expected to be called in the following sequence: NvFlexExtPushToDevice, NvFlexUpdateSolver, NvFlexExtPullFromDevice
    #[inline]
    pub fn pull_from_device(&self) {
        unsafe { NvFlexExtPullFromDevice(self.container) }
    }

    /// Synchronizes the per-instance data with the container's data, should be called after the synchronization with the solver read backs are complete
    ///
    /// The instances belonging to this container will be updated
    #[inline]
    pub fn update_instances(&self) {
        unsafe { NvFlexExtUpdateInstances(self.container) }
    }
}

impl Drop for Container {
    #[inline]
    fn drop(&mut self) {
        unsafe { NvFlexExtDestroyContainer(self.container) }
    }
}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Send for Container {}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Sync for Container {}

/// Represents an instance of a [`Asset`] in a container
#[derive(Debug)]
pub struct Instance {
    pub(crate) container: *mut NvFlexExtContainer,
    pub(crate) instance: *mut NvFlexExtInstance,
}

impl Instance {
    /// Creates an instance of an asset, the container will internally store a reference to the asset so it should remain valid for the instance lifetime. This
    /// method will allocate particles for the asset, assign their initial positions, velocity and phase.
    ///
    /// # Parameters
    ///
    /// * `container` - The container to spawn into
    /// * `particle_data` - Reference to a mapped particle data struct, returned from [`Container::map_particle_data()`]
    /// * `asset` - The asset to be spawned
    /// * `transform` - A 4x4 column major, column vector transform that specifies the initial world space configuration of the particles
    /// * `vx` - The velocity of the particles along the x axis
    /// * `vy` - The velocity of the particles along the y axis
    /// * `vz` - The velocity of the particles along the z axis
    /// * `phase` - The phase used for the particles
    /// * `inv_mass_scale` - A factor applied to the per particle inverse mass
    ///
    /// # Returns
    ///
    /// The instance of the asset
    #[inline]
    pub fn new(
        container: &Container,
        particle_data: &mut ParticleData,
        asset: &Asset,
        transform: &[f32; 4 * 4],
        vx: f32,
        vy: f32,
        vz: f32,
        phase: i32,
        inv_mass_scale: f32,
    ) -> Self {
        unsafe {
            Self {
                container: container.container,
                instance: NvFlexExtCreateInstance(
                    container.container,
                    particle_data as *mut _ as _,
                    asset.asset,
                    transform.as_ptr(),
                    vx,
                    vy,
                    vz,
                    phase,
                    inv_mass_scale,
                ),
            }
        }
    }
}

impl Drop for Instance {
    #[inline]
    fn drop(&mut self) {
        unsafe { NvFlexExtDestroyInstance(self.container, self.instance) }
    }
}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Send for Instance {}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Sync for Instance {}

/// Controls the way that force fields affect particles
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub enum ForceMode {
    /// Apply field value as a force.
    Force = 0,
    /// Apply field value as an impulse.
    Impulse = 1,
    /// Apply field value as a velocity change.
    VelocityChange = 2,
}

/// Force field data, currently just supports radial fields
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct ForceField {
    /// Center of force field
    position: [f32; 3],
    /// Radius of the force field
    radius: f32,
    /// Strength of the force field
    strength: f32,
    /// Mode of field application
    mode: ForceMode,
    /// Linear or no falloff
    linear_falloff: bool,
}

/// Opaque type representing a force field callback structure that ecapsulates
/// the force field kernels and associated data applied as a callback during the Flex update
#[derive(Debug)]
pub struct ForceFieldCallback {
    callback: *mut NvFlexExtForceFieldCallback,
}

impl ForceFieldCallback {
    /// Create a [`ForceFieldCallback`] structure, each callback is associated with the
    /// passed in solver once the NvFlexExtSetForceFields() is called.
    ///
    /// # Parameters
    ///
    /// * `solver` - A valid solver created with NvFlexCreateSolver()
    ///
    /// # Returns
    ///
    /// A callback structure
    #[inline]
    pub fn new(&self, solver: &Solver) -> Self {
        unsafe {
            Self {
                callback: NvFlexExtCreateForceFieldCallback(solver.solver),
            }
        }
    }

    /// Set force fields on the container, these will be applied during the Flex update
    ///
    /// # Parameters
    ///
    /// * `force_fields` - An array of force field data, may be host or GPU memory
    #[inline]
    pub fn set_force_fields(&self, force_fields: &[ForceField]) {
        unsafe {
            NvFlexExtSetForceFields(
                self.callback,
                force_fields.as_ptr() as _,
                force_fields.len() as _,
            )
        }
    }
}

impl Drop for ForceFieldCallback {
    #[inline]
    fn drop(&mut self) {
        unsafe { NvFlexExtDestroyForceFieldCallback(self.callback) }
    }
}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Send for ForceFieldCallback {}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Sync for ForceFieldCallback {}

/// Represents a soft joint with a radius overlapping different flex objects
///
/// Each soft joint can be spawned into a container using [`SoftJoint::new()`]
#[derive(Debug)]
pub struct SoftJoint {
    pub(crate) container: *mut NvFlexExtContainer,
    pub(crate) joint: *mut NvFlexExtSoftJoint,
}

impl SoftJoint {
    /// Create a soft joint, the container will internally store a reference to the joint array
    ///
    /// # Parameters
    ///
    /// * `container` - The container to spawn into
    /// * `particle_indices` - A pointer to an array of particle indices
    /// * `particle_local_positions` - A pointer to an array of particle local positions
    /// * `num_joint_particles` - The number of particles in the joint
    /// * `stiffness` - The stiffness of the joint
    ///
    /// # Returns
    ///
    /// A soft joint instance
    #[inline]
    pub fn new(
        container: &Container,
        particle_indices: &[i32],
        particle_local_positions: &[f32],
        num_joint_particles: i32,
        stiffness: f32,
    ) -> Self {
        assert!(
            particle_indices.len() >= num_joint_particles as _
                && particle_local_positions.len() >= num_joint_particles as _
        );
        unsafe {
            Self {
                container: container.container,
                joint: NvFlexExtCreateSoftJoint(
                    container.container,
                    particle_indices.as_ptr(),
                    particle_local_positions.as_ptr(),
                    num_joint_particles,
                    stiffness,
                ),
            }
        }
    }

    /// Transform all the local particles of the soft joint
    ///
    /// # Parameters
    ///
    /// * `position` - A vec3 storing the soft joint new position
    /// * `rotation` - A quaternion storing the soft joint new rotation
    #[inline]
    pub fn set_transform(&self, position: &[f32; 3], rotation: &[f32; 4]) {
        unsafe {
            NvFlexExtSoftJointSetTransform(
                self.container,
                self.joint,
                position.as_ptr(),
                rotation.as_ptr(),
            )
        }
    }

    /// Global indices
    #[inline]
    pub fn particle_indices(&self) -> *mut i32 {
        unsafe { (*self.joint).particleIndices }
    }
    /// Relative offsets from the particles of the joint to the center
    #[inline]
    pub fn particle_local_positions(&self) -> *mut f32 {
        unsafe { (*self.joint).particleLocalPositions }
    }
    /// Index in the container's shape body constraints array
    #[inline]
    pub fn shape_index(&self) -> i32 {
        unsafe { (*self.joint).shapeIndex }
    }
    /// Number of particles in the joint
    #[inline]
    pub fn num_particles(&self) -> i32 {
        unsafe { (*self.joint).numParticles }
    }

    /// Joint shape matching group translations (vec3s)
    #[inline]
    pub fn shape_translations(&self) -> [f32; 3] {
        unsafe { (*self.joint).shapeTranslations }
    }
    /// Joint shape matching group rotations (quaternions)
    #[inline]
    pub fn shape_rotations(&self) -> [f32; 4] {
        unsafe { (*self.joint).shapeRotations }
    }

    /// Joint stiffness
    #[inline]
    pub fn stiffness(&self) -> f32 {
        unsafe { (*self.joint).stiffness }
    }

    /// Joint status flag
    #[inline]
    pub fn initialized(&self) -> bool {
        unsafe { (*self.joint).initialized }
    }
}

impl Drop for SoftJoint {
    #[inline]
    fn drop(&mut self) {
        unsafe { NvFlexExtDestroySoftJoint(self.container, self.joint) }
    }
}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Send for SoftJoint {}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Sync for SoftJoint {}
