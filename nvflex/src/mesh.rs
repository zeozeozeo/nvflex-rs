use std::any::Any;

use crate::{Buffer, ConvexMeshId, DistanceFieldId, Library, TriangleMeshId};
use nvflex_sys::*;

/// An opaque type representing a static triangle mesh in the solver
#[derive(Debug, Clone)]
pub struct TriangleMesh {
    pub(crate) id: TriangleMeshId,
    pub(crate) lib: *mut NvFlexLibrary,
}

impl TriangleMesh {
    /// Create triangle mesh geometry, note that meshes may be used by multiple solvers if desired
    ///
    /// # Parameters
    ///
    /// * `lib` - The library instance to use
    ///
    /// # Returns
    ///
    /// A triangle mesh object
    #[inline]
    pub fn new(lib: Library) -> Self {
        unsafe {
            Self {
                id: NvFlexCreateTriangleMesh(lib.lib),
                lib: lib.lib,
            }
        }
    }

    /// Creates a [`TriangleMesh`] from a [`Library`] and a [`TriangleMeshId`]
    #[inline]
    pub fn from_id(lib: Library, id: TriangleMeshId) -> Self {
        Self { id, lib: lib.lib }
    }

    #[inline]
    pub fn id(self) -> TriangleMeshId {
        self.id
    }

    #[inline]
    pub fn lib(self) -> *mut NvFlexLibrary {
        self.lib
    }

    /// Specifies the triangle mesh geometry (vertices and indices), this method will cause any internal
    /// data structures (e.g.: bounding volume hierarchies) to be rebuilt.
    ///
    /// # Parameters
    ///
    /// * `vertices` - Pointer to a buffer of float4 vertex positions
    /// * `indices` - Pointer to a buffer of triangle indices, should be length numTriangles*3
    /// * `num_vertices` - The number of vertices in the vertices array
    /// * `num_triangles` - The number of triangles in the mesh
    /// * `lower` - A pointer to a float3 vector holding the lower spatial bounds of the mesh
    /// * `upper` - A pointer to a float3 vector holding the upper spatial bounds of the mesh
    #[inline]
    pub fn update<T: Any>(
        &self,
        vertices: &Buffer<T>,
        indices: &Buffer<T>,
        num_vertices: i32,
        num_triangles: i32,
        lower: [f32; 3],
        upper: [f32; 3],
    ) {
        unsafe {
            NvFlexUpdateTriangleMesh(
                self.lib,
                self.id,
                vertices.buf,
                indices.buf,
                num_vertices,
                num_triangles,
                &lower as *const _,
                &upper as *const _,
            )
        }
    }

    /// Return the local space bounds of the mesh, these are the same values specified to `update()`
    ///
    /// # Returns
    ///
    /// * .0 - Lower mesh bounds
    /// * .1 - Upper mesh bounds
    #[inline]
    pub fn get_bounds(&self) -> ([f32; 3], [f32; 3]) {
        unsafe {
            let mut lower = [0.0f32; 3];
            let mut upper = [0.0f32; 3];
            NvFlexGetTriangleMeshBounds(self.lib, self.id, lower.as_mut_ptr(), upper.as_mut_ptr());
            (lower, upper)
        }
    }
}

impl Drop for TriangleMesh {
    #[inline]
    fn drop(&mut self) {
        unsafe { NvFlexDestroyTriangleMesh(self.lib, self.id) };
    }
}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Send for TriangleMesh {}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Sync for TriangleMesh {}

/// An opaque type representing a signed distance field collision shape in the solver.
#[derive(Debug, Clone)]
pub struct DistanceField {
    pub(crate) id: DistanceFieldId,
    pub(crate) lib: *mut NvFlexLibrary,
}

impl DistanceField {
    /// Create a signed distance field collision shape, see [`DistanceFieldId`] for details.
    ///
    /// # Returns
    ///
    /// A signed distance field object
    #[inline]
    pub fn new(lib: Library) -> Self {
        unsafe {
            Self {
                id: NvFlexCreateDistanceField(lib.lib),
                lib: lib.lib,
            }
        }
    }

    /// Creates a [`DistanceField`] from a [`Library`] and a [`DistanceFieldId`]
    #[inline]
    pub fn from_id(lib: Library, id: DistanceFieldId) -> Self {
        Self { id, lib: lib.lib }
    }

    #[inline]
    pub fn id(&self) -> DistanceFieldId {
        self.id
    }

    #[inline]
    pub fn lib(&self) -> *mut NvFlexLibrary {
        self.lib
    }

    /// Update the signed distance field volume data, this method will upload
    /// the field data to a 3D texture on the GPU
    ///
    /// # Parameters
    ///
    /// * `sdf` - A signed distance field created with [`DistanceField::new()`]
    /// * `dimx` - The x-dimension of the volume data in voxels
    /// * `dimy` - The y-dimension of the volume data in voxels
    /// * `dimz` - The z-dimension of the volume data in voxels
    /// * `field` - The volume data stored such that the voxel at the x,y,z coordinate is addressed as `field[z*dimx*dimy + y*dimx + x]`
    #[inline]
    pub fn update<T: Any>(&self, dimx: i32, dimy: i32, dimz: i32, field: &Buffer<T>) {
        unsafe { NvFlexUpdateDistanceField(self.lib, self.id, dimx, dimy, dimz, field.buf) };
    }
}

impl Drop for DistanceField {
    #[inline]
    fn drop(&mut self) {
        unsafe { NvFlexDestroyDistanceField(self.lib, self.id) };
    }
}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Send for DistanceField {}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Sync for DistanceField {}

/// An opaque type representing a convex mesh collision shape in the solver.
///
/// Convex mesh shapes may consist of up to 64 planes of the form ax + by + c*z + d = 0, particles will be constrained to the outside of the shape.
pub struct ConvexMesh {
    pub(crate) id: ConvexMeshId,
    pub(crate) lib: *mut NvFlexLibrary,
}

impl ConvexMesh {
    /// Create a convex mesh collision shape, see [`ConvexMeshId`] for details.
    ///
    /// # Returns
    ///
    /// A convex mesh object
    #[inline]
    pub fn new(lib: Library) -> Self {
        unsafe {
            Self {
                id: NvFlexCreateConvexMesh(lib.lib),
                lib: lib.lib,
            }
        }
    }

    /// Creates a [`ConvexMesh`] from a [`Library`] and a [`ConvexMeshId`]
    #[inline]
    pub fn from_id(lib: Library, id: ConvexMeshId) -> Self {
        Self { id, lib: lib.lib }
    }

    #[inline]
    pub fn id(&self) -> ConvexMeshId {
        self.id
    }

    #[inline]
    pub fn lib(&self) -> *mut NvFlexLibrary {
        self.lib
    }

    /// Update the convex mesh geometry
    ///
    /// # Parameters
    ///
    /// * `convex` - A valid convex mesh shape created from [`ConvexMesh::new()`]
    /// * `planes` - An array of planes, each plane consists of 4 floats in the form a*x + b*y + c*z + d = 0
    /// * `num_planes` - The number of planes in the convex, must be less than 64 planes per-convex
    /// * `lower` - The local space lower bound of the convex shape
    /// * `upper` - The local space upper bound of the convex shape
    #[inline]
    pub fn update<T: Any>(
        &self,
        planes: &Buffer<T>,
        num_planes: i32,
        lower: [f32; 3],
        upper: [f32; 3],
    ) {
        unsafe {
            let mut lower = lower;
            let mut upper = upper;
            NvFlexUpdateConvexMesh(
                self.lib,
                self.id,
                planes.buf,
                num_planes,
                lower.as_mut_ptr(),
                upper.as_mut_ptr(),
            )
        };
    }

    /// Returns the local space bounds of the mesh, these are the same values specified to [`ConvexMesh::update`]
    ///
    /// # Returns
    ///
    /// * `.0` - Lower mesh bounds
    /// * `.1` - Upper mesh bounds
    #[inline]
    pub fn get_bounds(&self) -> ([f32; 3], [f32; 3]) {
        unsafe {
            let mut lower = [0.0f32; 3];
            let mut upper = [0.0f32; 3];
            NvFlexGetConvexMeshBounds(self.lib, self.id, lower.as_mut_ptr(), upper.as_mut_ptr());
            (lower, upper)
        }
    }
}

impl Drop for ConvexMesh {
    #[inline]
    fn drop(&mut self) {
        unsafe { NvFlexDestroyConvexMesh(self.lib, self.id) };
    }
}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Send for ConvexMesh {}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Sync for ConvexMesh {}
