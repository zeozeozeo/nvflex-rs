use cgmath::{vec3, Vector3};
use genmesh::generators::{
    Circle, Cone, Cube, Cylinder, IcoSphere, IndexedPolygon, Plane, SharedVertex, SphereUv,
};
use genmesh::Triangulate;

/// Represents a trianglular mesh.
#[derive(Debug, Clone, Default)]
pub struct Shape {
    /// The vertices of the shape.
    pub vertices: Vec<Vector3<f32>>,
    /// The indices of the shape.
    pub indices: Vec<u32>,
}

impl Shape {
    pub fn new() -> Self {
        Self {
            vertices: vec![],
            indices: vec![],
        }
    }

    /// Create a shape from vertices and indices.
    ///
    /// # Arguments
    ///
    /// * `vertices` - The vertices of the shape.
    /// * `indices` - The indices of the shape.
    #[inline]
    pub fn trimesh(vertices: Vec<Vector3<f32>>, indices: Vec<u32>) -> Self {
        Self { vertices, indices }
    }

    /// Circle shape in the XY plane with radius of 1, centered at (0, 0, 0).
    ///
    /// # Arguments
    ///
    /// * `points` - The number of points around the circle, must be > 3.
    #[inline]
    pub fn circle(points: usize) -> Self {
        Self::trimesh(
            Circle::new(points)
                .shared_vertex_iter()
                .map(|v| vec3(v.pos.x, v.pos.y, v.pos.z))
                .collect(),
            Circle::new(points)
                .indexed_polygon_iter()
                .triangulate()
                .flat_map(|v| [v.x as u32, v.y as u32, v.z as u32])
                .collect(),
        )
    }

    /// Create a mesh that is from 1 to -1.
    /// The bottom will be a circle around (0, 0, -1) with a radius of 1,
    /// all coords on the bottom will follow the plan equation -z-1=0.
    /// The tip of the cone will always be at coord (0, 0, 1).
    ///
    /// # Arguments
    ///
    /// * `points` - The number of subdivisions around the radius of the cone, must be > 1.
    #[inline]
    pub fn cone(points: usize) -> Self {
        Self::trimesh(
            Cone::new(points)
                .shared_vertex_iter()
                .map(|v| vec3(v.pos.x, v.pos.y, v.pos.z))
                .collect(),
            Cone::new(points)
                .indexed_polygon_iter()
                .triangulate()
                .flat_map(|v| [v.x as u32, v.y as u32, v.z as u32])
                .collect(),
        )
    }

    #[inline]
    pub fn cube() -> Self {
        Self::trimesh(
            Cube::new()
                .shared_vertex_iter()
                .map(|v| vec3(v.pos.x, v.pos.y, v.pos.z))
                .collect(),
            Cube::new()
                .indexed_polygon_iter()
                .triangulate()
                .flat_map(|v| [v.x as u32, v.y as u32, v.z as u32])
                .collect(),
        )
    }

    /// Cylinder with radius of 1, height of 2, and centered at (0, 0, 0) pointing up (to 0, 0, 1).
    ///
    /// # Arguments
    ///
    /// * `points` - the number of points across the radius.
    #[inline]
    pub fn cylinder(points: usize) -> Self {
        Self::trimesh(
            Cylinder::new(points)
                .shared_vertex_iter()
                .map(|v| vec3(v.pos.x, v.pos.y, v.pos.z))
                .collect(),
            Cylinder::new(points)
                .indexed_polygon_iter()
                .triangulate()
                .flat_map(|v| [v.x as u32, v.y as u32, v.z as u32])
                .collect(),
        )
    }

    /// Icosahedral sphere with radius 1, centered at (0, 0, 0).
    #[inline]
    pub fn icosphere() -> Self {
        Self::trimesh(
            IcoSphere::new()
                .shared_vertex_iter()
                .map(|v| vec3(v.pos.x, v.pos.y, v.pos.z))
                .collect(),
            IcoSphere::new()
                .indexed_polygon_iter()
                .triangulate()
                .flat_map(|v| [v.x as u32, v.y as u32, v.z as u32])
                .collect(),
        )
    }

    /// 2D plane with origin of (0, 0), from 1 to -1.
    #[inline]
    pub fn plane() -> Self {
        Self::trimesh(
            Plane::new()
                .shared_vertex_iter()
                .map(|v| vec3(v.pos.x, v.pos.y, v.pos.z))
                .collect(),
            Plane::new()
                .indexed_polygon_iter()
                .triangulate()
                .flat_map(|v| [v.x as u32, v.y as u32, v.z as u32])
                .collect(),
        )
    }

    /// Sphere with radius of 1, centered at (0, 0, 0).
    ///
    /// # Arguments
    ///
    /// * `u` - The number of points across the equator of the sphere.
    /// * `v` - The number of points from pole to pole.
    #[inline]
    pub fn sphere_uv(u: usize, v: usize) -> Self {
        Self::trimesh(
            SphereUv::new(u, v)
                .shared_vertex_iter()
                .map(|v| vec3(v.pos.x, v.pos.y, v.pos.z))
                .collect(),
            SphereUv::new(u, v)
                .indexed_polygon_iter()
                .triangulate()
                .flat_map(|v| [v.x as u32, v.y as u32, v.z as u32])
                .collect(),
        )
    }
}
