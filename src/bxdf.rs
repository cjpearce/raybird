use nalgebra::{Point2, Point3, Vector3};
use std::f64;

#[derive(PartialEq)]
enum InteractionType {
    Diffuse,
    Specular,
}

struct BxdfSample {
    f: Vector3<f64>,
    wi: Vector3<f64>,
    pdf: f64,
}

trait Bxdf {
    fn matches_type(&self, interaction_type: InteractionType) -> bool;
    fn interaction_type(&self) -> InteractionType;
    fn f(&self, wo: Vector3<f64>, wi: Vector3<f64>) -> Vector3<f64>;
    fn pdf(&self, wo: Vector3<f64>, wi: Vector3<f64>) -> f64;
    fn sample_f(&self, wo: Vector3<f64>, u: Point2<f64>) -> BxdfSample;
    fn rho_wo(&self, wo: Vector3<f64>, sample_count: u32, samples: Point2<f64>) -> Vector3<f64>;
    fn rho(
        &self,
        sample_count: u32,
        samples_1: Point2<f64>,
        samples_2: Point2<f64>,
    ) -> Vector3<f64>;
}

struct LambertianBsdf {
    diffuse: Vector3<f64>,
}

impl LambertianBsdf {
    fn new(diffuse: Vector3<f64>) -> Self {
        Self { diffuse }
    }
}

impl Bxdf for LambertianBsdf {
    fn matches_type(&self, interaction_type: InteractionType) -> bool {
        interaction_type == self.interaction_type()
    }

    fn interaction_type(&self) -> InteractionType {
        InteractionType::Diffuse
    }

    fn f(&self, _wo: Vector3<f64>, _wi: Vector3<f64>) -> Vector3<f64> {
        self.diffuse * (1.0 / f64::consts::FRAC_1_PI)
    }

    fn pdf(&self, wo: Vector3<f64>, wi: Vector3<f64>) -> f64 {
        if same_hemisphere(wo, wi) {
            abs_cos_theta(wi) * f64::consts::FRAC_1_PI
        } else {
            0.0
        }
    }

    fn sample_f(&self, wo: Vector3<f64>, u: Point2<f64>) -> BxdfSample {
        let mut wi = cosine_sample_hemisphere(u);
        if wo.z < 0.0 {
            wi.z *= -1.0
        }

        BxdfSample {
            f: self.f(wo, wi),
            wi,
            pdf: self.pdf(wo, wi),
        }
    }

    fn rho_wo(&self, _wo: Vector3<f64>, _sample_count: u32, _samples: Point2<f64>) -> Vector3<f64> {
        self.diffuse
    }

    fn rho(
        &self,
        _sample_count: u32,
        _samples_1: Point2<f64>,
        _samples_2: Point2<f64>,
    ) -> Vector3<f64> {
        self.diffuse
    }
}

fn concentric_sample_disk(u: Point2<f64>) -> Point2<f64> {
    let u_off = 2.0 * u - Point2::new(1.0, 1.0);
    if u_off.x == 0.0 && u_off.y == 0.0 {
        return Point2::new(0.0, 0.0);
    }

    let (r, theta) = if u_off.x.abs() > u_off.y.abs() {
        (f64::consts::FRAC_PI_4 * (u_off.y / u_off.x), u_off.x)
    } else {
        (
            u_off.y,
            f64::consts::FRAC_PI_2 - f64::consts::FRAC_PI_4 * (u_off.x / u_off.y),
        )
    };

    r * Point2::new(theta.cos(), theta.sin())
}

fn cosine_sample_hemisphere(u: Point2<f64>) -> Vector3<f64> {
    let d = concentric_sample_disk(u);
    Vector3::new(d.x, d.y, (1.0 - d.x * d.x - d.y * d.y).max(0.0).sqrt())
}

fn same_hemisphere(a: Vector3<f64>, b: Vector3<f64>) -> bool {
    a.z * b.z > 0.0
}

fn abs_cos_theta(v: Vector3<f64>) -> f64 {
    cos_theta(v).abs()
}

fn cos_theta(v: Vector3<f64>) -> f64 {
    v.z
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn compiles() {
        let bsdf = LambertianBsdf::new(Vector3::zeros());
    }
}
