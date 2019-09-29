use crate::material::{SurfaceInteraction, SurfacePoint};
use crate::ray::Ray;
use crate::scene::Scene;
use crate::sensor::{Sensor, SensorDimensions};
pub use nalgebra::Vector3;

pub trait Screen {
    fn write(&mut self, i: usize, r: u8, g: u8, b: u8);
}

pub struct SingleThreadedTracer {
    bounces: u32,
    dimensions: SensorDimensions,
    sensor: Sensor,
    sample_count: usize,
}

impl SingleThreadedTracer {
    pub fn new(bounces: u32, gamma: f64, width: usize, height: usize) -> Self {
        let dimensions = SensorDimensions { width, height };
        Self {
            bounces,
            dimensions,
            sensor: Sensor::new(dimensions, 1.0 / gamma),
            sample_count: 0,
        }
    }

    pub fn update(&mut self, scene: &Scene, screen: &mut impl Screen) {
        let pixel = self.dimensions.pixel_for_index(self.sample_count);

        let ray = scene.camera.ray(
            pixel.x,
            pixel.y,
            self.dimensions.width,
            self.dimensions.height,
        );

        let sample = StratisfiedImageSampler::new(scene, ray, 1, self.bounces as usize)
            .next()
            .unwrap();

        let index = self.dimensions.index_for_pixel(pixel);
        self.sensor.add_sample(index, sample);
        let color = self.sensor.color_at(index);
        screen.write(index, color.x, color.y, color.z);

        self.sample_count += 1;
    }
}

pub struct StratisfiedImageSampler<'a> {
    scene: &'a Scene,
    ray: Ray,
    samples: u32,
    bounces: usize,
}

impl<'a> StratisfiedImageSampler<'a> {
    pub fn new(scene: &'a Scene, ray: Ray, samples: u32, bounces: usize) -> Self {
        Self {
            scene,
            ray,
            samples,
            bounces,
        }
    }
}

impl<'a> Iterator for StratisfiedImageSampler<'a> {
    type Item = Vector3<f64>;

    fn next(&mut self) -> Option<Vector3<f64>> {
        let mut total_energy = Vector3::new(0.0, 0.0, 0.0);
        let n = f64::from(self.samples).sqrt() as u32;
        for u in 0..n {
            for v in 0..n {
                let fu = (f64::from(u) + rand::random::<f64>()) / f64::from(n);
                let fv = (f64::from(v) + rand::random::<f64>()) / f64::from(n);
                total_energy += LightPath::new(&self.scene, self.ray, (fu, fv))
                    .take(self.bounces)
                    .sum::<Vector3<f64>>();
            }
        }

        Some(total_energy / f64::from(n * n))
    }
}

struct LightPath<'a> {
    scene: &'a Scene,
    ray: Ray,
    signal: Vector3<f64>,
    uv: (f64, f64),
}

impl<'a> LightPath<'a> {
    fn new(scene: &'a Scene, ray: Ray, first_uv: (f64, f64)) -> Self {
        Self {
            scene,
            ray,
            signal: Vector3::new(1.0, 1.0, 1.0),
            uv: first_uv,
        }
    }
}

impl<'a> Iterator for LightPath<'a> {
    type Item = Vector3<f64>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.signal == Vector3::zeros() {
            return None;
        }

        if let Some(intersect) = self.scene.intersect(&self.ray) {
            let interaction = SurfaceInteraction {
                wo: -self.ray.direction,
                surface: SurfacePoint {
                    p: intersect.hit,
                    n: intersect.normal,
                },
            };

            let (direction, signal) = intersect.material.bsdf(
                &self.scene,
                interaction,
                intersect.distance,
                self.uv.0,
                self.uv.1,
            );

            self.uv = (rand::random(), rand::random());

            let contribution = intersect.material.emit().component_mul(&self.signal);
            self.ray = Ray {
                origin: intersect.hit,
                direction: direction,
            };
            self.signal = self.signal.component_mul(&signal);
            Some(contribution)
        } else {
            let contribution = self.scene.bg(&self.ray).component_mul(&self.signal);
            self.signal = Vector3::zeros();
            Some(contribution)
        }
    }
}
