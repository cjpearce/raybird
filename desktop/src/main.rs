use minifb::{Key, Window, WindowOptions};
use raybird::{
    scene_loader,
    tracer::{Screen, StratisfiedImageSampler, Vector3},
    sensor::{Sensor, SensorDimensions}
};
use crossbeam::{unbounded, scope};

const WIDTH: usize = 1920;
const HEIGHT: usize = 1080;
const NUM_THREADS: usize = 10;
const NUM_SAMPLES: usize = 10;
const GAMMA: f64 = 2.2;

fn main() {
    let scene = scene_loader::load_scene("spheres").unwrap();
    let pixels = vec![0u32; WIDTH * HEIGHT];
    let mut screen = MiniFbScreen{pixels};
    let mut window = Window::new(
        "Test - ESC to exit",
        WIDTH,
        HEIGHT,
        WindowOptions::default(),
    )
    .unwrap_or_else(|e| {
        panic!("{}", e);
    });

    scope(|scoped| {
        let dimensions = SensorDimensions{width: WIDTH, height: HEIGHT};
        let (sender, receiver) = unbounded::<[(usize, Vector3<f64>); NUM_SAMPLES]>();
        for a in 0..NUM_THREADS {
            let se = sender.clone();
            let scene_ref = &scene;
            scoped.spawn(move |_| {
                let mut i = a*(HEIGHT/NUM_THREADS) * WIDTH;
                loop {
                    let mut samples = [(0, Vector3::new(0.0, 0.0, 0.0)); NUM_SAMPLES];
                    for sample in samples.iter_mut() {
                        let pixel = dimensions.pixel_for_index(i);
                        let ray = scene_ref.camera.ray(pixel.x, pixel.y, dimensions.width, dimensions.height);
                        let index = dimensions.index_for_pixel(pixel);
                        *sample = (index, StratisfiedImageSampler::new(scene_ref, ray, 1, 6).next().unwrap());
                        i += 1;
                    }

                    if se.send(samples).is_err() {
                        break;
                    }
                }
            });
        }

        let mut sensor = Sensor::new(dimensions, 1.0/GAMMA);
        while window.is_open() && !window.is_key_down(Key::Escape) {
            while let Ok(samples) = receiver.try_recv() {
                for (i, sample) in samples.iter() {
                    sensor.add_sample(*i, *sample);
                    let color = sensor.color_at(*i);
                    screen.write(*i, color.x, color.y, color.z);
                }
            }
            
            window.update_with_buffer(&screen.pixels).unwrap();
        }
    }).unwrap();
}

struct MiniFbScreen {
    pixels: Vec<u32>,
}

impl Screen for MiniFbScreen {
    fn write(&mut self, i: usize, r: u8, g: u8, b: u8) {
        self.pixels[i] = 0;
        self.pixels[i] = (256 as u32) << 24| (r as u32) << 16 | (g as u32) << 8 | (b as u32)
    }
}