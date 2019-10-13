use nalgebra::{geometry::Reflection, Unit, Vector3, Point3};
use crate::ray::{DirectionExt, Ray};
use crate::scene::{Scene, Intersection};
use crate::onb::{OrthonormalBasis};
use crate::sphere::{Sphere};
use rand;
use rand::seq::SliceRandom;
use std::f64;

#[derive(Copy, Clone)]
pub struct SurfacePoint {
    pub n: Vector3<f64>,
    pub p: Point3<f64>,
}

#[derive(Copy, Clone)]
pub struct SurfaceInteraction {
    pub wo: Vector3<f64>,
    pub surface: SurfacePoint
}

pub struct BSDF {
    pub direction: Vector3<f64>,
    pub signal: Vector3<f64>
}

#[derive(Copy, Clone)]
pub struct Material {
    color: Vector3<f64>,
    refraction: f64,
    transparency: f64,
    light: Vector3<f64>,
    frensel: Vector3<f64>,
    metal: f64,
    gloss: f64,
}

impl Material {
    pub fn new(
        color: Vector3<f64>,
        refraction: f64,
        transparency: f64,
        light: Vector3<f64>,
        frensel: Vector3<f64>,
        metal: f64,
        gloss: f64,
    ) -> Self {
        Self {
            color,
            refraction,
            transparency,
            light,
            frensel,
            metal,
            gloss,
        }
    }

    pub fn can_emit(&self) -> bool {
        self.light.norm() > 0.0
    }

    pub fn emit(&self) -> Vector3<f64> {
        self.light
    }

    pub fn bsdf(
        &self,
        scene: &Scene,
        interaction: SurfaceInteraction,
        length: f64,
        u: f64,
        v: f64
    ) -> BSDF {
        if interaction.wo.dot(&interaction.surface.n) > 0f64 {
            // brdf
            let mut test = FilteredProbabilityTest::new();
            if test.or(self.schilck(&interaction).component_average()) {
                self.reflected(&interaction, u, v)
            } else if test.or(self.transparency) {
                self.refracted_entry(&interaction)
            } else if test.or(self.metal) {
                self.dead()
            } else {
                self.diffused(scene, &interaction, u, v)
            }
        } else if let Some(exited) = (-interaction.wo).refraction(
                &-interaction.surface.n,
                self.refraction,
                1.0
        ) {
            // btdf
            self.refracted_exit(exited, length)
        } else {
            self.dead()
        }
    }

    fn dead(&self) -> BSDF {
        BSDF {
            direction: Vector3::new(0.0, 0.0, 0.0),
            signal: Vector3::new(0.0, 0.0, 0.0)
        }
    }

    fn schilck(&self, interaction: &SurfaceInteraction) -> Vector3<f64> {
        let cos_incident = interaction.wo.dot(&interaction.surface.n);
        self.frensel + ((Vector3::new(1.0, 1.0, 1.0) - self.frensel) * (1.0 - cos_incident).powf(5.0))
    }

    fn diffused(&self, scene: &Scene, interaction: &SurfaceInteraction, u: f64, v: f64, ) -> BSDF {
        let cos_component = CosWeightedDiffuse::new(interaction.surface.n, u, v);
        let mut components: Vec<&dyn Pdf> = vec![&cos_component];
        let lights = scene.lights();
        let light_components = lights.iter().map(|light| {
            LightWeightedDiffuse::new(
                interaction.surface.p,
                interaction.surface.n,
                &light
            )
        }).collect::<Vec<_>>();

        light_components.iter().fold(&mut components, |acc, light| {
            acc.push(light);
            acc
        });

        let mix = MixturePdf::new(components);
        let direction = mix.gen();
        BSDF {
            direction,
            signal: self.color * mix.p(direction)
        }
    }

    fn reflected(&self, interaction: &SurfaceInteraction, u: f64, v: f64) -> BSDF {
        let mut reflected = -interaction.wo;
        Reflection::new(Unit::new_normalize(interaction.surface.n), 0.0)
            .reflect(&mut reflected);

        BSDF{
            direction: Vector3::random_in_cone(
                &reflected,
                1.0 - self.gloss,
                u,
                v
            ),
            signal: Vector3::new(1.0, 1.0, 1.0).lerp(&self.frensel, self.metal)
        }
    }

    fn refracted_entry(&self, interaction: &SurfaceInteraction) -> BSDF {
        BSDF{
            direction: (-interaction.wo).refraction(
                &interaction.surface.n,
                1.0,
                self.refraction
            ).unwrap(),
            signal: Vector3::new(1.0, 1.0, 1.0)
        }
    }

    fn refracted_exit(&self, exited: Vector3<f64>, length: f64) -> BSDF {
        let opacity = 1.0 - self.transparency;
        let volume = f64::min(opacity * length * length, 1.0);
        let tint = Vector3::new(1.0, 1.0, 1.0).lerp(&self.color, volume);
        BSDF {
            direction: exited,
            signal: tint
        }
    }
}

trait Pdf {
    fn p(&self, v: Vector3<f64>) -> f64;
    fn gen(&self) -> Vector3<f64>;
}

struct MixturePdf<'a> {
    components: Vec<&'a dyn Pdf>,
}

impl <'a> MixturePdf<'a> {
    fn new(components: Vec<&'a dyn Pdf>) -> Self {
        Self{components}
    }
}

impl <'a> Pdf for MixturePdf<'a> {
    fn p(&self, v: Vector3<f64>) -> f64 {
        let mut total = 0.0;
        for component in self.components.iter() {
            total += component.p(v);
        }
        total / self.components.len() as f64
    }

    fn gen(&self) -> Vector3<f64> {
        self.components.choose(&mut rand::thread_rng()).unwrap().gen()
    }
}


struct CosWeightedDiffuse {
    u: f64,
    v: f64,
    normal: Vector3<f64>
}

impl CosWeightedDiffuse {
    fn new(normal: Vector3<f64>, u: f64, v: f64) -> Self {
        Self{u, v, normal}
    }
}

impl Pdf for CosWeightedDiffuse {
    fn p(&self, v: Vector3<f64>) -> f64 {
        let cosine = v.dot(&self.normal);
        if cosine > 0.0 { cosine / std::f64::consts::PI } else { 0.0 }
    }

    fn gen(&self) -> Vector3<f64> {
        let vec = Vector3::random_in_cos_hemisphere(self.u, self.v);
        let onb = OrthonormalBasis::from_normal(self.normal);
        onb.local(vec)
    }
}

struct LightWeightedDiffuse<'a> {
    point: Point3<f64>,
    normal: Vector3<f64>,
    light: &'a Sphere
}

impl <'a> LightWeightedDiffuse<'a> {
    fn new(point: Point3<f64>, normal: Vector3<f64>, light: &'a Sphere) -> Self {
        Self{point, normal, light}
    }
}

impl <'a> Pdf for LightWeightedDiffuse<'a> {
    fn p(&self, v: Vector3<f64>) -> f64 {
        let light = self.light;

        // get bounding sphere center and radius
        let center = light.center();
        let radius = light.radius();

        let cos_angle = v.dot(&self.normal);
        if cos_angle <= 0.0 {
            return 0.0;
        }

        // compute solid angle (hemisphere coverage)
        let hyp = (center - self.point).norm();
        let opp = radius;
        let theta = (opp / hyp).asin();
        let adj = opp / theta.tan();
        let d = theta.cos() * adj;
        let r = theta.sin() * adj;

        let coverage = if hyp < opp {
            1.0
        } else {
            f64::min((r * r) / (d * d), 1.0)
        };

        let a = (center - self.point).angle(&v);

        if a < radius {
            coverage / std::f64::consts::PI
        } else {
            0.0
        }
    }

    fn gen(&self) -> Vector3<f64> {
        let light = self.light;

        // get bounding sphere center and radius
        let center = light.center();
        let radius = light.radius();

        // get random point in disk
        let point = loop {
            let x = rand::random::<f64>() * 2.0 - 1.0;
            let y = rand::random::<f64>() * 2.0 - 1.0;
            if x*x + y*y <= 1.0 {
                let l = (center - self.point).normalize();
                let u = l.cross(&Vector3::random_in_sphere()).normalize();
                let v = l.cross(&u);

                break center + (u * x * radius) + (v * y * radius);
            }
        };

        // construct ray toward light point
        (point - self.point).normalize()
    }
}

struct FilteredProbabilityTest {
    r: f64,
    p: f64
}

impl FilteredProbabilityTest {
    fn new() -> Self {
        Self{r: rand::random::<f64>(), p: 0.0}
    }

    fn or(&mut self, p: f64) -> bool {
        self.p = (1.0 - self.p) * p;
        self.r <= self.p
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use nalgebra::Vector3;

    #[test]
    fn schilck_is_correct() {

        let incident = Vector3::new(
            0.9999877074290066,
            0.002070457097031252,
            0.004505352182583419,
        );
        let normal = Vector3::new(
            -0.42430229364657923,
            0.17526903761586785,
            -0.8883964925974548,
        );
        
        let material = Material::new(
            Vector3::new(0.1, 0.1, 1.0),
            1.0,
            0.0,
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.04, 0.04, 0.04),
            0.0,
            0.2
        );

        assert_eq!(
            material.schilck(&incident, &normal),
            Vector3::new(
                0.09881546766725074,
                0.09881546766725074,
                0.09881546766725074
            )
        )
    }
}
