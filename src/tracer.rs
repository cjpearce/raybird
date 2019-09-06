use crate::ray::{Ray, DirectionExt};
use crate::scene::{Scene, Intersection};
use nalgebra::Point2;
pub use nalgebra::Vector3;

pub trait Screen {
    fn write(&mut self, i: usize, r: u8, g: u8, b: u8);
}

#[derive(Clone)]
struct PixelInfo {
    color: Vector3<f64>,
    exposures: u32,
}

impl PixelInfo {
    fn color(&self, reciprocal_gamma: f64) -> Vector3<u8> {
        let color = self.color * (1.0 / f64::from(self.exposures));
        let color = (color / 255.0)
            .apply_into(|v| v.powf(reciprocal_gamma).min(1.0))
            * 255.0;
        Vector3::new(color.x as u8, color.y as u8, color.z as u8)
    }
}

pub struct Exposures(Vec<PixelInfo>, f64);

impl Exposures {
    pub fn new(length: usize, reciprocal_gamma: f64) -> Self {
        let default = PixelInfo {
            color: Vector3::new(0.0, 0.0, 0.0),
            exposures: 0
        };
        Self(vec![default; length], reciprocal_gamma)
    }

    pub fn add_sample(&mut self, position: usize, sample: Vector3<f64>) {
        self.0[position].color += sample;
        self.0[position].exposures += 1;
    }

    pub fn color_at(&self, position: usize) -> Vector3<u8> {
        self.0[position].color(self.1)
    }
}

pub struct Tracer {
    bounces: u32,
    width: usize,
    height: usize,
    exposures: Exposures,
    index: usize
}

impl Tracer {
    pub fn new(bounces: u32, gamma: f64, width: usize, height: usize) -> Tracer {
        Tracer {
            bounces,
            width,
            height,
            exposures: Exposures::new(width*height, 1.0/gamma),
            index: 0
        }
    }

    pub fn update(&mut self, scene: &Scene, screen: &mut impl Screen) {
        let pixel = self.pixel_for_index(self.index);
        
        let sample = self.expose(scene, pixel);

        let index = pixel.x + pixel.y * self.width;
        self.exposures.add_sample(index, sample);

        let color = self.exposures.color_at(pixel.x + pixel.y * self.width);
        
        screen.write(index, color.x, color.y, color.z);

        self.index += 1;
    }

    pub fn pixel_for_index(&self, index: usize) -> Point2<usize> {
        let wrapped = index % (self.width * self.height);
        Point2::new(wrapped % self.width, wrapped / self.width)
    }

    pub fn expose(&self, scene: &Scene, pixel: Point2<usize>) -> Vector3<f64> {
        let ray = scene
            .camera
            .ray(pixel.x, pixel.y, self.width, self.height);
        self.trace(scene, ray, 16)
    }

    fn trace(&self, scene: &Scene, ray: Ray, samples: u32) -> Vector3<f64> {
        let mut total_energy = Vector3::new(0.0, 0.0, 0.0);
        let n = f64::from(samples).sqrt() as u32;
        for u in 0..n {
            for v in 0..n {
                let mut ray = ray;
                let mut energy = Vector3::new(0.0, 0.0, 0.0);
                let mut signal = Vector3::new(1.0, 1.0, 1.0);

                for bounce in 0..self.bounces {
                    let stratisfied_count = if bounce == 0 { n } else { 1 };
                    let fu = (f64::from(u) + rand::random::<f64>()) / f64::from(stratisfied_count);
                    let fv = (f64::from(v) + rand::random::<f64>()) / f64::from(stratisfied_count);
            
                    if let Some(intersect) = scene.intersect(&ray) {
                        let sample = intersect
                            .material
                            .bsdf(
                                &intersect.normal,
                                &ray.direction,
                                intersect.distance,
                                fu,
                                fv,
                                &scene,
                                &intersect
                            );

                        ray.origin = intersect.hit;
                        ray.direction = sample.direction;
                        energy += intersect.material.emit().component_mul(&signal);
                        signal = signal.component_mul(&sample.signal);
                    } else {
                        energy += scene.bg(&ray).component_mul(&signal);
                        break;
                    }
                }

                total_energy += energy;
            }
        }

        total_energy / f64::from(n*n)
    }
}