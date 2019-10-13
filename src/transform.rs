use nalgebra::{Point3, Transform3};

pub struct AABB {
    min: Point3<f64>,
    max: Point3<f64>,
}

impl AABB {
    fn union(&self, point: Point3<f64>) -> Self {
        Self {
            min: Point3::new(
                self.min.x.min(point.x),
                self.min.y.min(point.y),
                self.min.z.min(point.z),
            ),
            max: Point3::new(
                self.max.x.max(point.x),
                self.max.y.max(point.y),
                self.max.z.max(point.z),
            ),
        }
    }

    fn transformed(&self, transform: &Transform3<f64>) -> Self {
        let mut ret = Self {
            min: transform * self.min,
            max: transform * self.min,
        };
        ret = ret.union(transform * Point3::new(self.max.x, self.min.y, self.min.z));
        ret = ret.union(transform * Point3::new(self.min.x, self.max.y, self.min.z));
        ret = ret.union(transform * Point3::new(self.min.x, self.min.y, self.max.z));
        ret = ret.union(transform * Point3::new(self.min.x, self.max.y, self.max.z));
        ret = ret.union(transform * Point3::new(self.max.x, self.max.y, self.min.z));
        ret = ret.union(transform * Point3::new(self.max.x, self.min.y, self.max.z));
        ret = ret.union(transform * Point3::new(self.max.x, self.max.y, self.max.z));
        ret
    }
}
