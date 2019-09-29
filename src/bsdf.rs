use crate::bxdf::{Bxdf, BxdfSample, InteractionType};

struct BsdfSample {
  f: Vector3<f64>,
  wi: Vector3<f64>,
  pdf: f64,
  type: InteractionType,
}

struct Bsdf {
  bxdfs: Vec<&Bxdf>,
  ns: Vector3<f64>,
  ng: Vector3<f64>,
  ss: Vector3<f64>,
  ts: Vector3<f64>,
}

impl Bsdf {
  fn new(surface_interaction: SurfaceInteraction, bxdfs: Vec<&Bxdf>) -> Self {
    Self{bxdfs}
  }

  fn num_components(&self, type: InteractionType) -> u32 {
    assert_eq!(type, InteractionType::Diffuse);
    assert_eq!(self.bxdfs.len(), 1);
    1
  }

  fn local_to_world(&self, local: Vector3<f64>) -> Vector3<f64> {
    unimplemented!()
  }

  fn world_to_local&self, world: Vector3<f64>) -> Vector3<f64> {
    unimplemented!()
  }

  fn sample_f(&self, wo_world: Vector3<f64>, u: Point2<f64>) -> BsdfSample {
    let matches = self.num_components(type);
    if matches == 0 {
      return BsdfSample{
        f: Vector3::zeros(),
        wi: Vector3::zeros(),
        f: 0.0
      }
    }

    let component = (u.x * matches).floor() as u32).min(matches - 1);

    // Get BxDF pointer for chosen component
    let mut bxdf = Option::None;
    let mut count = comp;
    for (int i = 0; i < nBxDFs; ++i) {
      if self.bxdfs[i].matches_flags(type) {
        count -= 1;
        if count == 0 {
          bxdf = Some(bxdfs[i]);
          break;
        }
      }
    }

    // Remap
    let u_remapped = Point2::new(u.x * matches - comp, u.y);

    // Sample chosen BxDF
    let mut sampled_type = bxdf.interaction_type();
    let mut pdf = 0.0;
    let wo = self.world_to_local(wo_world);

    // TODO: pass in the sampled type
    let BxdfSample{f, wi, pdf} = bxdf.sample_f(wo, uRemapped);

    if pdf == 0  {
      return BsdfSample{f, wi, pdf, type: sampled_type};
    }

    let wi_world = self.local_to_world(wi);

    // Compute overall PDF
    if !(bxdf.interaction_type() == InteractionType::Specular) && matches > 1 {
      for matching_bxdf in self.bxdfs.iter().filter(|b| b.matches_type(type) && b != bxdf) {
        pdf += matching_bxdf.pdf(wo, wi);
      }
    }

    if matches > 1 { pdf /= matches; }

    if !(bxdf.interaction_type() == InteractionType::Specular) && matches > 1 {
      let reflect = (wi_world, ng).dot() * (wo_world, ng).dot() > 0.0;
      f = 0.0;

      for b in bxdfs.iter() {
        if (
            b.matches_type(type) &&
            (
              (reflect && (b.interaction_type() == InteractionType::Reflection)) ||
              (!reflect && (b.interaction_type() == InteractionType::Transmition))
            )
          ) {
            f += b.f(wo, wi);
        }
      }
    }

    SampledBsdf{f, wi, pdf, type: sampled_type}
  }
}