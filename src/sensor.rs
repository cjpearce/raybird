pub use nalgebra::{Point2, Vector3};

#[derive(Copy, Clone)]
pub struct SensorDimensions {
    pub width: usize,
    pub height: usize,
}

impl SensorDimensions {
    pub fn pixel_for_index(&self, index: usize) -> Point2<usize> {
        let wrapped = index % (self.width * self.height);
        Point2::new(wrapped % self.width, wrapped / self.width)
    }

    pub fn index_for_pixel(&self, pixel: Point2<usize>) -> usize {
        pixel.x + pixel.y * self.width
    }
}

#[derive(Clone)]
struct PixelInfo {
    color: Vector3<f64>,
    sensor: u32,
}

impl PixelInfo {
    fn color(&self, reciprocal_gamma: f64) -> Vector3<u8> {
        let color = self.color * (1.0 / f64::from(self.sensor));
        let color = (color / 255.0).apply_into(|v| v.powf(reciprocal_gamma).min(1.0)) * 255.0;
        Vector3::new(color.x as u8, color.y as u8, color.z as u8)
    }
}

pub struct Sensor {
    pixels: Vec<PixelInfo>,
    reciprocal_gamma: f64,
}

impl Sensor {
    pub fn new(dimensions: SensorDimensions, reciprocal_gamma: f64) -> Self {
        let default = PixelInfo {
            color: Vector3::new(0.0, 0.0, 0.0),
            sensor: 0,
        };
        Self {
            pixels: vec![default; dimensions.width * dimensions.height],
            reciprocal_gamma,
        }
    }

    pub fn add_sample(&mut self, position: usize, sample: Vector3<f64>) {
        self.pixels[position].color += sample;
        self.pixels[position].sensor += 1;
    }

    pub fn color_at(&self, position: usize) -> Vector3<u8> {
        self.pixels[position].color(self.reciprocal_gamma)
    }
}
