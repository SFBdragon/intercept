use super::{Fp, Vec2, BodySweepData};

pub trait Reacter {
    /// Return the velocity to resume after a collision.
    fn react(&mut self, vel: Vec2, self_id: usize, other_id: usize, data: BodySweepData, epsilon: Fp) -> Vec2;
}

/// React by maintaining a coefficient of the previous velocity, phasing through static colliders.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PhaseReacter {
    /// `0.0` = halt, `1.0` = maintain velocity, `-1.0` = reverse velocity, etc.
    pub coefficient: Fp,
}

impl Default for PhaseReacter {
    fn default() -> Self {
        //! Returns a `PhaseReacter` with a coefficient of `1.0`.
        PhaseReacter { coefficient: 1.0 }
    }
}
impl PhaseReacter {
    pub fn new(coeff: Fp) -> PhaseReacter {
        PhaseReacter { coefficient: coeff }
    }
}
impl Reacter for PhaseReacter {
    fn react(&mut self, vel: Vec2, _: usize, _: usize, _: BodySweepData, _: Fp) -> Vec2 {
        vel * self.coefficient
    }
}

/// React by reflecting a coefficient of the velocity across the normal of collision.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeflectReacter {
    pub coefficient: Fp,
}

impl Default for DeflectReacter {
    fn default() -> Self {
        //! Returns a `DeflectReacter` with a coefficient of `1.0`.
        DeflectReacter { coefficient: 1.0 }
    }
}
impl DeflectReacter {
    pub fn new(coeff: Fp) -> DeflectReacter {
        DeflectReacter { coefficient: coeff }
    }
}
impl Reacter for DeflectReacter {
    fn react(&mut self, vel: Vec2, _: usize, _: usize, data: BodySweepData, _: Fp) -> Vec2 {
        // i = r by normal: r=d−2(d⋅n)n
        (vel - data.norm * (vel.dot(data.norm) * 2.0)) * self.coefficient
    }
}

/// React by maintaining previous velocity in the direction perpendicular to the normal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlideReacter {
    pub coefficient: Fp,
    norm_from_constricting: Vec2,
}

impl Default for SlideReacter {
    fn default() -> Self {
        //! Returns a `SlideReacter` with a coefficient of `1.0` and an invalid constricting normal of zero.
        SlideReacter { coefficient: 1.0, norm_from_constricting: Vec2::ZERO }
    }
}
impl SlideReacter {
    pub fn new(coeff: Fp) -> SlideReacter {
        SlideReacter { coefficient: coeff, norm_from_constricting: Vec2::ZERO }
    }
}
impl Reacter for SlideReacter {
    fn react(&mut self, vel: Vec2, _: usize, _: usize, data: BodySweepData, epsilon: Fp) -> Vec2 {
        // check if the collision is instantanious, accounting for epsilon adjustment
        let is_instant = (vel * data.travel).dot(data.norm) > epsilon * -2.0;
        // Update to current norm of constriction
        let noc = self.norm_from_constricting;
        if is_instant {
            self.norm_from_constricting = noc;
        } else {
            self.norm_from_constricting = Vec2::ZERO;
        }
        
        // Stop if both this and the previous collisions constrict movement totally, or if velocity is reverse-parallel to the normal.
        if (is_instant && vel.dot(noc) < 0.0) || vel.dot(data.norm) <= -vel.length() * 0.999 {
            self.norm_from_constricting = data.norm;
            return Vec2::ZERO;
        }
        let perp = Vec2::new(data.norm.y, -data.norm.x);
        perp * (vel.dot(perp) * self.coefficient)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn phase_reacter_test() {
        let mut rea = PhaseReacter::default();
        assert_eq!(rea.react(Vec2::new(1.0, -1.0), 0, 0, BodySweepData::new_invalid(), 0.05), Vec2::new(1.0, -1.0));
        rea.coefficient = 0.0;
        assert_eq!(rea.react(Vec2::new(1.0, -1.0), 0, 0, BodySweepData::new_invalid(), 0.05), Vec2::new(0.0, 0.0));
    }

    #[test]
    fn deflect_reacter_test() {
        let mut rea = DeflectReacter::default();
        let mut bsd = BodySweepData { b1_shape: 0, b2_shape: 0, travel: 0.5, norm: Vec2::Y };

        assert_eq!(rea.react(Vec2::new(1.0, -1.0), 0, 0, bsd, 0.05), Vec2::new(1.0, 1.0));
        rea.coefficient = 0.0;
        assert_eq!(rea.react(Vec2::new(1.0, -1.0), 0, 0, bsd, 0.05), Vec2::new(0.0, 0.0));
        rea.coefficient = 0.25;
        bsd.norm = Vec2::new(-Fp::sqrt(0.5), Fp::sqrt(0.5));
        assert_abs_diff_eq!(rea.react(Vec2::new(1.0, -1.0), 0, 0, bsd, 0.05), Vec2::new(-0.25, 0.25));
    }

    #[test]
    fn slide_reacter_test() {
        let mut rea = SlideReacter::default();
        let mut bsd = BodySweepData { b1_shape: 0, b2_shape: 0, travel: 0.5, norm: Vec2::Y };

        assert_eq!(rea.react(Vec2::new(1.0, -1.0), 0, 0, bsd, 0.05), Vec2::new(1.0, 0.0));
        assert_eq!(rea.norm_from_constricting, Vec2::ZERO);
        rea.coefficient = 0.0;
        assert_eq!(rea.react(Vec2::new(1.0, -1.0), 0, 0, bsd, 0.05), Vec2::new(0.0, 0.0));
        assert_eq!(rea.norm_from_constricting, Vec2::ZERO);

        rea.coefficient = 0.25;
        bsd.travel = 0.001;
        bsd.norm = Vec2::new(-Fp::sqrt(0.5), Fp::sqrt(0.5));
        assert_abs_diff_eq!(rea.react(Vec2::new(1.0, -1.0), 0, 0, bsd, 0.05), Vec2::new(0.0, 0.0));
        assert_eq!(rea.norm_from_constricting, bsd.norm);

        
        bsd.norm = Vec2::new(-Fp::sqrt(0.5), -Fp::sqrt(0.5));
        assert_abs_diff_eq!(rea.react(Vec2::new(1.0, 0.0), 0, 0, bsd, 0.05), Vec2::new(0.0, 0.0));
        assert_eq!(rea.norm_from_constricting, bsd.norm);
    }
}
