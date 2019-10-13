use crate::material::Material;
use crate::ray::Ray;
use crate::transform::AABB;
use nalgebra::{Point2, Point3, Transform3, Vector3};
use std::f64;

#[derive(Copy, Clone)]
pub struct Sphere {
    index: usize,

    // Shape
    object_to_world: Transform3<f64>,
    world_to_object: Transform3<f64>,
    reverse_orientation: bool,

    // Sphere
    radius: f64,
    z_min: f64,
    z_max: f64,
    theta_min: f64,
    theta_max: f64,
    phi_max: f64,
    material: Material,
}

impl PartialEq for Sphere {
    fn eq(&self, other: &Self) -> bool {
        self.index == other.index
    }
}

pub struct SurfaceInteraction<'a> {
    uv: Point2<f64>,
    dpdu: Vector3<f64>,
    dpdv: Vector3<f64>,
    sphere: &'a Sphere,
    p: Point3<f64>,
    time: f64,
    wo: Vector3<f64>,
    n: Vector3<f64>,
}

impl<'a> SurfaceInteraction<'a> {
    fn new(
        p: Point3<f64>,
        uv: Point2<f64>,
        wo: Vector3<f64>,
        dpdu: Vector3<f64>,
        dpdv: Vector3<f64>,
        time: f64,
        sphere: &'a Sphere,
    ) -> Self {
        Self {
            uv,
            dpdu,
            dpdv,
            sphere,
            p,
            time,
            wo,
            n: dpdu.cross(&dpdv).normalize(),
        }
    }

    fn transformed(&self, transform: &Transform3<f64>) -> SurfaceInteraction<'a> {
        Self{
            uv: self.uv,
            dpdu: transform * self.dpdu,
            dpdv: transform * self.dpdv,
            sphere: self.sphere,
            p: transform * self.p,
            time: self.time,
            wo: transform * self.wo,
            // May be able to use inverse if can mark transforms as affine or projective
            n: transform
                .try_inverse()
                .unwrap()
                .to_homogeneous()
                .transpose() * &self.n,
        }
    }
}

impl Sphere {
    pub fn new(
        index: usize,
        object_to_world: Transform3<f64>,
        world_to_object: Transform3<f64>,
        reverse_orientation: bool,
        radius: f64,
        z_min: f64,
        z_max: f64,
        phi_max: f64,
        material: Material,
    ) -> Self {
        Sphere {
            index,
            object_to_world,
            world_to_object,
            reverse_orientation,
            radius,
            z_min: z_min.min(z_max).clamp(-radius, radius),
            z_max: z_min.max(z_max).clamp(-radius, radius),
            theta_min: (z_min / radius).clamp(-1.0, 1.0).acos(),
            theta_max: (z_max / radius).clamp(-1.0, 1.0).acos(),
            phi_max: phi_max.clamp(0.0, 360.0).radians(),
            material,
        }
    }

    pub fn object_bounds(&self) -> AABB {
        AABB {
            min: Point3::new(-self.radius, -self.radius, self.z_min),
            max: Point3::new(self.radius, self.radius, self.z_max),
        }
    }

    // this only really needs to be exposed for bounding sphers
    pub fn radius(&self) -> f64 {
        self.radius
    }

    pub fn material(&self) -> &Material {
        &self.material
    }

    pub fn intersect<'a>(&'a self, ray_in: Ray) -> SurfaceInteraction<'a> {
        let ray = self.world_to_object * ray_in.direction;

        SurfaceInteraction{
            
        }
    }
}

trait AnglesExt {
    fn radians(&self) -> f64;
    fn degrees(&self) -> f64;
}

impl AnglesExt for f64 {
    fn radians(&self) -> f64 {
        (f64::consts::PI / 180.0) * self
    }

    fn degrees(&self) -> f64 {
        (180.0 / f64::consts::PI) * self
    }
}
