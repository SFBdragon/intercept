use super::{Fp, Vec2, BodySweepData};

pub trait Reacter {
    /// Return the velocity to resume after a collision.
    fn react(&mut self, vel: Vec2, self_id: usize, other_id: usize, data: BodySweepData, epsilon: Fp) -> Vec2;
}

/// React by maintaining a coefficient of the previous velocity, phasing through static colliders.
pub struct PhaseReacter {
    /// `0.0` = halt, `1.0` = maintain velocity, `-1.0` = reverse velocity, etc.
    coefficient: Fp,
}
impl Reacter for PhaseReacter {
    fn react(&mut self, vel: Vec2, _: usize, _: usize, _: BodySweepData, _: Fp) -> Vec2 {
        vel * self.coefficient
    }
}

/// React by reflecting a coefficient of the velocity across the normal of collision.
pub struct DeflectReacter {
    coefficient: Fp,
}
impl Reacter for DeflectReacter {
    fn react(&mut self, vel: Vec2, _: usize, _: usize, data: BodySweepData, _: Fp) -> Vec2 {
        // i = r by normal: r=d−2(d⋅n)n
        (vel - data.norm * (vel.dot(data.norm) * 2.0)) * self.coefficient
    }
}

/// React by maintaining previous velocity in the direction perpendicular to the normal.
pub struct SlideReacter {
    norm_from_constricting: Vec2,
    coefficient: Fp,
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
        if (is_instant && vel.dot(noc) > 0.0) || vel.normalize().dot(data.norm) <= 0.99 {
            self.norm_from_constricting = data.norm;
            return Vec2::ZERO;
        }
        let perp = Vec2::new(data.norm.y, -data.norm.x);
        perp * (vel.dot(perp) * self.coefficient)
    }
}
