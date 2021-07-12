use crate::{Fp, Vec2, narrow::{Aabb, Circle, Intersect, Poly, Shape, ShapeUnion}};

#[derive(Debug, Clone)]
pub struct Body {
    /// Posistion
    pub pos: Vec2,
    /// Compositing shapes
    pub shapes: Vec<Shape>,
    /// Bounding box
    pub aabb: Aabb,
    /// Velocity
    pub vel: Vec2,
    /// Whether the `Body` is *relatively* fast moving.
    pub bullet: bool,
}
impl Body {
    pub fn new(shapes: Vec<Shape>, pos: Vec2, vel: Vec2, is_bullet: bool) -> Body {
        //! Note: This function is not zero-cost. Manually initialize `Body`s when its multi-shaped and its bounding box is known. Check implementation for details.

        // Body's full bounding box must be found.
        let mut bbb = shapes[0].get_bounding_box();
        if shapes.len() != 1 {
            for s in shapes.iter().skip(1) {
                let sbb = s.get_bounding_box();
                if sbb.min.x < bbb.min.x { bbb.min.x = sbb.min.x; }
                if sbb.max.x > bbb.max.x { bbb.max.x = sbb.max.x; }
                if sbb.min.y < bbb.min.y { bbb.min.y = sbb.min.y; }
                if sbb.max.y > bbb.max.y { bbb.max.y = sbb.max.y; }
            }
        }

        // Bounding box must be translated and maintained accordingly.
        Body { pos, vel, aabb: bbb.translate(pos), shapes, bullet: is_bullet }
    }

    pub fn get_broad(&self) -> Aabb {
        //! Returns `self`'s Aabb broadened over `self`'s velocity.
        self.aabb.broaden(self.vel)
    }
    pub fn get_shape_broad(&self, shape: usize) -> Aabb {
        //! Returns its `Shape`'s Aabb broadened over `self`'s velocity.
        self.shapes[shape]
            .get_bounding_box()
            .translate(self.pos)
            .broaden(self.vel)
    }

    pub fn translate(&mut self, offset: Vec2) {
        //! Teleports the body.
        self.pos += offset;
        self.aabb = self.aabb.translate(offset);
    }
}
impl Intersect for Body {
    #[inline]
    fn get_bounding_box(&self) -> Aabb {
        self.aabb
    }

    fn point_test(&self, point: Vec2) -> bool {
        for s in self.shapes.iter() {
            if s.point_test(point) {
                return true;
            }
        }
        false
    }
    fn line_test(&self, a: Vec2, b: Vec2) -> bool {
        for s in self.shapes.iter() {
            if s.line_test(a, b) {
                return true;
            }
        }
        false
    }
    fn line_query(&self, a: Vec2, b: Vec2) -> Option<Fp> {
        for s in self.shapes.iter() {
            if let Some(r) = s.line_query(a, b) {
                return Some(r);
            }
        }
        None
    }

    fn circle_test(&self, circle: &Circle) -> bool {
        for s in self.shapes.iter() {
            if s.circle_test(circle) {
                return true;
            }
        }
        false
    }
    fn aabb_test(&self, aabb: &Aabb) -> bool {
        for s in self.shapes.iter() {
            if s.aabb_test(aabb) {
                return true;
            }
        }
        false
    }
    fn poly_test(&self, poly: &Poly) -> bool {
        for s in self.shapes.iter() {
            if s.poly_test(poly) {
                return true;
            }
        }
        false
    }
}

// ---------- Sweep Helper Functions ---------- //

#[inline]
fn circle_aabb_passby(unit_vel_ca: Vec2, aabb: &Aabb, o: Vec2, rad: Fp) -> bool { // ~3.6ns
    let (l, r) = if unit_vel_ca.x >= 0.0 {
        if unit_vel_ca.y >= 0.0 {
            (aabb.minx_maxy(), aabb.maxx_miny())
        } else {
            (aabb.min, aabb.max)
        }
    } else {
        if unit_vel_ca.y >= 0.0 {
            (aabb.max, aabb.min)
        } else {
            (aabb.maxx_miny(), aabb.minx_maxy())
        }
    };
    (l - o).perp_dot(unit_vel_ca) < rad 
    && (r - o).perp_dot(unit_vel_ca) > -rad
}
#[inline]
fn aabb_aabb_passby(vel_a1a2: Vec2, aabb1: &Aabb, aabb2: &Aabb, diff_p1p2: Vec2) -> bool { // ~4.4ns
    let (l1r2, r1l2) = if vel_a1a2.x >= 0.0 {
        if vel_a1a2.y >= 0.0 {
            (aabb2.maxx_miny() - aabb1.minx_maxy(), aabb2.minx_maxy() - aabb1.maxx_miny())
        } else {
            (aabb2.max - aabb1.max, aabb2.min - aabb1.min)
        }
    } else {
        if vel_a1a2.y >= 0.0 {
            (aabb2.min - aabb1.min, aabb2.max - aabb1.max)
        } else {
            (aabb2.minx_maxy() - aabb1.maxx_miny(), aabb2.maxx_miny() - aabb1.minx_maxy())
        }
    };
    (r1l2 + diff_p1p2).perp_dot(vel_a1a2) < 0.0 
    && (l1r2 + diff_p1p2).perp_dot(vel_a1a2) > 0.0
}

// ---------- Sweep ---------- //

// note: perf timings are for my machine specifically, as reference for a low-end, modern processor (Ryzen 3200G, stock clock speed)
fn aabb_aabb_sweep(b1: &Body, a1: &Aabb, b2: &Body, a2: &Aabb, t: Fp) -> Option<(Fp, Vec2)> { // ~12ns
    let rv = (b1.vel - b2.vel) * t;
    if rv.x == 0.0 && rv.y == 0.0 {
        return None;
    }
    let p1p2 = b2.pos - b1.pos;
    let (dx_entry, dx_exit, dy_entry, dy_exit): (Fp, Fp, Fp, Fp);
    if rv.x >= 0.0 {
        dx_entry = p1p2.x + a2.min.x - a1.max.x;
        dx_exit = p1p2.x + a2.max.x - a1.min.x;
    } else {
        dx_entry = a1.min.x - p1p2.x - a2.max.x;
        dx_exit = a1.max.x - p1p2.x - a2.min.x;
    }
    if rv.y >= 0.0 {
        dy_entry = p1p2.y + a2.min.y - a1.max.y;
        dy_exit = p1p2.y + a2.max.y - a1.min.y;
    } else {
        dy_entry = a1.min.y - p1p2.y - a2.max.y;
        dy_exit = a1.max.y - p1p2.y - a2.min.y;
    }

    let (tx_entry, tx_exit, ty_entry, ty_exit): (Fp, Fp, Fp, Fp);
    if rv.x == 0.0 {
        tx_entry = Fp::MIN;
        tx_exit = Fp::MAX;
        ty_entry = dy_entry / rv.y;
        ty_exit = dy_exit / rv.y;
    } else if rv.y == 0.0 {
        tx_entry = dx_entry / rv.x;
        tx_exit = dx_exit / rv.x;
        ty_entry = Fp::MIN;
        ty_exit = Fp::MAX;
    } else {
        // rv.x and rv.y cannot both be zero
        tx_entry = dx_entry / rv.x;
        tx_exit = dx_exit / rv.x;
        ty_entry = dy_entry / rv.y;
        ty_exit = dy_exit / rv.y;
    }

    if (tx_entry < 0.0 && ty_entry < 0.0) || tx_entry > 1.0 || ty_entry > 1.0 {
        return None;
    }

    let entry = Fp::max(tx_entry, ty_entry);
    let exit = Fp::min(tx_exit, ty_exit);
    if entry > exit {
        None
    } else {
        if tx_entry > ty_entry {
            if tx_entry > 0.0 {
                Some((entry, Vec2::new(1.0, 0.0)))
            } else {
                Some((entry, Vec2::new(-1.0, 0.0)))
            }
        } else {
            if ty_entry > 0.0 {
                Some((entry, Vec2::new(0.0, 1.0)))
            } else {
                Some((entry, Vec2::new(0.0, -1.0)))
            }
        }
    }
}
fn circle_circle_sweep(b1: &Body, c1: &Circle, b2: &Body, c2: &Circle, t: Fp) -> Option<(Fp, Vec2)> { // ~12ns(f) - ~19ns(s)
    let rv = (b1.vel - b2.vel) * t;
    if rv.x == 0.0 && rv.y == 0.0 {
        return None;
    }

    let inv_rv_len = 1.0 / rv.length();
    let unit = rv * inv_rv_len;
    let srad = c1.rad + c2.rad;

    let c2c1 = (b1.pos + c1.pos) - (b2.pos + c2.pos);
    let dot = unit.dot(c2c1);
    let discr = srad * srad + dot * dot - c2c1.length_squared();
    if discr > 0.0 {
        let dist = -(dot + discr.sqrt()) * inv_rv_len;
        if dist >= 0.0 && dist <= 1.0 {
            return Some((dist, (c2c1 + rv * dist) / srad));
        }
    }
    None
}
fn poly_poly_sweep(b1: &Body, p1: &Poly, b2: &Body, p2: &Poly, t: Fp) -> Option<(Fp, Vec2)> { // (6-4) ~80ns
    let rv1 = (b1.vel - b2.vel) * t;
    if rv1.x == 0.0 && rv1.y == 0.0 {
        return None;
    }
    let dpos = b2.pos - b1.pos;

    // bullets are swept-checked (extra ~5ns)
    if (b1.bullet || b2.bullet) && !aabb_aabb_passby(rv1, &p1.aabb, &p2.aabb, dpos) {
        return None;
    }

    let rv2 = -rv1;
    let mut immenence = Fp::MAX; // time to collision
    let mut imminent_norm = Vec2::ZERO; // norm of collision from 2

    for p1vi in 0..p1.norms.len() {
        let n = p1.norms[p1vi];
        if n.dot(rv2) >= 0.0 {
            continue;
        }
        let v = -p1.verts[p1vi];

        let diff_verts = p1.verts[p1vi + 1] + v;
        let dot = rv2.perp_dot(diff_verts);
        let dd = dot * dot;
        for p2vi in 0..p2.norms.len() {
            let p2v = p2.verts[p2vi] + dpos + v;
            let desc = rv2.perp_dot(p2v) * dot;
            if desc >= 0.0 && desc <= dd {
                let t = diff_verts.perp_dot(p2v) * dot;
                if t >= 0.0 && t <= dd && t < immenence * dd {
                    immenence = t / dd;
                    imminent_norm = n;
                }
            }
        }
    }
    for p2vi in 0..p2.norms.len() {
        let n = p2.norms[p2vi];
        if n.dot(rv1) >= 0.0 {
            continue;
        }
        let v = -p2.verts[p2vi];

        let diff_verts = p2.verts[p2vi + 1] + v;
        let dot = rv1.perp_dot(diff_verts);
        let dd = dot * dot;
        for p1vi in 0..p1.norms.len() {
            let vc = p1.verts[p1vi] + dpos + v;
            let desc = rv1.perp_dot(vc) * dot;
            if desc >= 0.0 && desc <= dd {
                let t = diff_verts.perp_dot(vc) * dot;
                if t >= 0.0 && t <= dd && t < immenence * dd {
                    immenence = t / dd;
                    imminent_norm = -n;
                }
            }
        }
    }

    if immenence < 1.0 {
        Some((immenence, imminent_norm))
    } else {
        None
    }
}
fn circle_aabb_sweep(b1: &Body, circle: &Circle, b2: &Body, aabb: &Aabb, t: Fp) -> Option<(Fp, Vec2)> { // ~13ns(!c) - ~34ns
    // Checks against expanded AABB, if it's a corner case, circle test to get collision point, if any (bullet checks won't be worth it as the norm of vel isn't precalculated)
    let rv = (b1.vel - b2.vel) * t;
    if rv.x == 0.0 && rv.y == 0.0 {
        return None;
    }
    let seg_origin = circle.pos + b1.pos - b2.pos; // relative to aabb
    let seg_end = seg_origin + rv; // relative to aabb
                                   // good luck reading this
    let aabb_height = aabb.max.y - aabb.min.y;
    if seg_origin.x < aabb.min.x - circle.rad && seg_end.x > aabb.min.x - circle.rad {
        // LEFT
        // this operation is inlined and simplified from line_seg_query(), above is the seg (t) guard
        let dx = (aabb.min.x - circle.rad) - seg_origin.x;
        let differentiator = (rv.x * (aabb.min.y - seg_origin.y) - rv.y * dx) / -rv.x; // n = (0,1) o = min
        if differentiator >= -circle.rad && differentiator < aabb_height + circle.rad {
            // (u) guard
            return if differentiator < 0.0 {
                // whether collision occurs below aabb.min.y
                Circle { rad: circle.rad, pos: aabb.min }.line_query(seg_origin, seg_end).map(|travel| 
                    (travel, aabb.min - (seg_origin - rv * travel) / circle.rad))
            } else if differentiator > aabb_height {
                // whether collision occurs above aabb.max.y
                Circle { rad: circle.rad, pos: aabb.minx_maxy() }.line_query(seg_origin, seg_end).map(|travel| 
                    (travel, aabb.minx_maxy() - (seg_origin - rv * travel) / circle.rad))
            } else {
                // collision occured between aabb.min.y and aabb.max.y
                Some((dx / rv.x, Vec2::new(-1.0, 0.0)))
            };
        }
    } else if seg_origin.x > aabb.max.x + circle.rad && seg_end.x < aabb.max.x + circle.rad {
        // RIGHT
        let dx = aabb.max.x + circle.rad - seg_origin.x;
        let differentiator = (rv.x * (aabb.max.y - seg_origin.y) - rv.y * dx) / rv.x; // n = (0,-1) o = max
        if differentiator >= -circle.rad && differentiator < aabb_height + circle.rad {
            return if differentiator < 0.0 {
                Circle { rad: circle.rad, pos: aabb.max }.line_query(seg_origin, seg_end).map(|travel| 
                    (travel, aabb.max - (seg_origin - rv * travel) / circle.rad))
            } else if differentiator > aabb_height {
                Circle { rad: circle.rad, pos: aabb.maxx_miny() }.line_query(seg_origin, seg_end).map(|travel| 
                    (travel, aabb.maxx_miny() - (seg_origin - rv * travel) / circle.rad))
            } else {
                Some((dx / -rv.x, Vec2::new(1.0, 0.0)))
            };
        }
    }

    let aabb_width = aabb.max.x - aabb.min.x;
    if seg_origin.y < aabb.min.y - circle.rad && seg_end.y > aabb.min.y - circle.rad {
        // BOTTOM
        let dy = aabb.min.y - circle.rad - seg_origin.y;
        let differentiator = (rv.x * dy - rv.y * (aabb.min.x - seg_origin.x)) / rv.y; // n = (1,0) o = min
        if differentiator >= -circle.rad && differentiator < aabb_width + circle.rad {
            return if differentiator < 0.0 {
                Circle { rad: circle.rad, pos: aabb.min }.line_query(seg_origin, seg_end).map(|travel| 
                    (travel, aabb.min - (seg_origin - rv * travel) / circle.rad))
            } else if differentiator > aabb_width {
                Circle { rad: circle.rad, pos: aabb.maxx_miny() }.line_query(seg_origin, seg_end).map(|travel| 
                    (travel, aabb.maxx_miny() - (seg_origin - rv * travel) / circle.rad))
            } else {
                Some((dy / -rv.y, Vec2::new(0.0, -1.0)))
            };
        }
    } else if seg_origin.y > aabb.max.y + circle.rad && seg_end.y < aabb.max.y + circle.rad {
        // TOP
        let dy = (aabb.max.y + circle.rad) - seg_origin.y;
        let differentiator = (rv.x * dy - rv.y * (aabb.max.x - seg_origin.x)) / -rv.y; // n = (-1,0) o = max
        if differentiator >= -circle.rad && differentiator < aabb_width + circle.rad {
            return if differentiator < 0.0 {
                Circle { rad: circle.rad, pos: aabb.max }.line_query(seg_origin, seg_end).map(|travel| 
                    (travel, aabb.max - (seg_origin - rv * travel) / circle.rad))
            } else if differentiator > aabb_width {
                Circle { rad: circle.rad, pos: aabb.minx_maxy() }.line_query(seg_origin, seg_end).map(|travel| 
                    (travel, aabb.minx_maxy() - (seg_origin - rv * travel) / circle.rad))
            } else {
                Some((dy / rv.y, Vec2::new(0.0, 1.0)))
            };
        }
    }
    None
}
fn aabb_poly_sweep(b1: &Body, aabb: &Aabb, b2: &Body, poly: &Poly, t: Fp) -> Option<(Fp, Vec2)> { // 46ns (n=6)
    #[inline(always)]
    fn line_query_mod(rv: Vec2, d_v: Vec2, d_o: Vec2) -> Option<Fp> {
        let dot = rv.x * d_v.y - d_v.x * rv.y;
        let u = (rv.x * d_o.y - rv.y * d_o.x) / dot;
        if u < 0.0 || u > 1.0 {
            return None;
        }
        Some((d_v.x * d_o.y - d_v.y * d_o.x) / dot)
    }

    let rv1 = (b1.vel - b2.vel) * t;
    if rv1.x == 0.0 && rv1.y == 0.0 {
        return None;
    }
    let aabb = aabb.translate(b1.pos - b2.pos);

    // bullets should be swept-checked (extra ~5ns)
    if (b1.bullet || b2.bullet) && !aabb_aabb_passby(rv1, &aabb, &poly.aabb, Vec2::ZERO) {
        return None;
    }

    let mut immenence = Fp::MAX; // time until collision
    let mut imminent_norm = Vec2::ZERO; // norm of collision

    for p2vi in 0..poly.norms.len() {
        let n = poly.norms[p2vi];
        if n.dot(rv1) >= 0.0 {
            continue;
        }
        let v = poly.verts[p2vi];
        let diff = poly.verts[p2vi + 1] - v;
        if n.x >= 0.0 {
            if n.y >= 0.0 {
                if let Some(t) = line_query_mod(rv1, diff, aabb.min - v) {
                    if t >= 0.0 && t < immenence {
                        immenence = t;
                        imminent_norm = n;
                    }
                }
            }
            if n.y <= 0.0 {
                if let Some(t) = line_query_mod(rv1, diff, aabb.minx_maxy() - v) {
                    if t >= 0.0 && t < immenence {
                        immenence = t;
                        imminent_norm = n;
                    }
                }
            }
        }
        if n.x <= 0.0 {
            if n.y >= 0.0 {
                if let Some(t) = line_query_mod(rv1, diff, aabb.maxx_miny() - v) {
                    if t >= 0.0 && t < immenence {
                        immenence = t;
                        imminent_norm = n;
                    }
                }
            }
            if n.y <= 0.0 {
                if let Some(t) = line_query_mod(rv1, diff, aabb.max - v) {
                    if t >= 0.0 && t < immenence {
                        immenence = t;
                        imminent_norm = n;
                    }
                }
            }
        }
    }

    let rv2 = -rv1;
    if rv2.x > 0.0 {
        // LEFT
        for p2vi in 0..poly.norms.len() {
            let vo = poly.verts[p2vi] - aabb.min;
            if vo.x < 0.0 && vo.x + rv2.x > 0.0 {
                let t = -vo.y / rv2.x;
                if t >= 0.0 && t < immenence {
                    immenence = t;
                    imminent_norm = poly.norms[p2vi];
                }
            }
        }
    } else if rv2.x < 0.0 {
        // RIGHT
        for p2vi in 0..poly.norms.len() {
            let vo = poly.verts[p2vi] - aabb.max;
            if vo.x > 0.0 && vo.x + rv2.x < 0.0 {
                let t = -vo.y / rv2.x;
                if t >= 0.0 && t < immenence {
                    immenence = t;
                    imminent_norm = poly.norms[p2vi];
                }
            }
        }
    }
    if rv2.y > 0.0 {
        // BOTTOM
        for p2vi in 0..poly.norms.len() {
            let vo = poly.verts[p2vi] - aabb.min;
            if vo.y < 0.0 && vo.y + rv2.y > 0.0 {
                let t = -vo.x / rv2.y;
                if t >= 0.0 && t < immenence {
                    immenence = t;
                    imminent_norm = poly.norms[p2vi];
                }
            }
        }
    } else if rv2.y < 0.0 {
        // TOP
        for p2vi in 0..poly.norms.len() {
            let vo = poly.verts[p2vi] - aabb.max;
            if vo.y > 0.0 && vo.y + rv2.y < 0.0 {
                let t = -vo.x / rv2.y;
                if t >= 0.0 && t < immenence {
                    immenence = t;
                    imminent_norm = poly.norms[p2vi];
                }
            }
        }
    }

    if immenence < 1.0 {
        Some((immenence, imminent_norm))
    } else {
        None
    }
}
fn circle_poly_sweep(b1: &Body, circle: &Circle, b2: &Body, poly: &Poly, t: Fp) -> Option<(Fp, Vec2)> { // 18(s)-36(s)ns (n=6)
    let rv = (b1.vel - b2.vel) * t;
    if rv.x == 0.0 && rv.y == 0.0 {
        return None;
    }

    let rv_len = rv.length();
    let unit_rv = rv / rv_len;
    let origin = b1.pos - b2.pos + circle.pos;

    // the expensive operations must be performed regardless, thus bullets should be swept-checked (extra ~4ns)
    if (b1.bullet || b2.bullet) && !circle_aabb_passby(unit_rv, &poly.aabb, origin, circle.rad) {
        return None;
    }

    let mut closest_vert = usize::MAX;
    let mut closest_dist = Fp::MAX;
    let end = origin + rv;
    let cr2 = circle.rad * circle.rad;
    for i in 0..poly.norms.len() {
        let n = poly.norms[i];
        let v = poly.verts[i];
        let vo = origin - v;

        let vo_mag2 = vo.length_squared();
        if vo_mag2 < closest_dist {
            closest_dist = vo_mag2;
            closest_vert = i;
        }

        let ndotvo = n.dot(vo);
        if ndotvo >= 0.0 {
            if n.dot(end - v) >= 0.0 {
                // fallshort/tangential
                return None;
            } else {
                // guarantees ray-seg collision, and that n.dot(rv) != 0.0
                let edge = poly.verts[i + 1] - v;
                let t = vo.perp_dot(rv) / edge.perp_dot(rv);
                if t < 0.0 {
                    // modified circle collision algorithm
                    // note: this will never re-test prev v1 due to the circle-fallshort check
                    let udotvo = unit_rv.dot(vo);
                    let rad_discr = udotvo * udotvo + cr2 - vo_mag2;
                    if rad_discr > 0.0 {
                        // tangential and passing lines are not collisions
                        let dist = -udotvo - rad_discr.sqrt();
                        return if dist < rv_len {
                            // impale/fallshort : success/guaranteed failure
                            let norm = (unit_rv * dist - vo) / circle.rad;
                            Some((dist, norm))
                        } else {
                            None
                        };
                    }
                } else if t <= 1.0 {
                    return Some((vo.perp_dot(edge) / edge.perp_dot(rv), n));
                } else {
                    let v1o = origin - poly.verts[i + 1];
                    let udotv1o = unit_rv.dot(v1o);
                    let rad_discr = udotv1o * udotv1o + cr2 - v1o.length_squared();
                    if rad_discr > 0.0 {
                        let dist = -udotv1o - rad_discr.sqrt();
                        return if dist < rv_len {
                            let norm = (unit_rv * dist - v1o) / circle.rad;
                            Some((dist, norm))
                        } else {
                            None
                        };
                    }
                }
            }
        }
    }
    // perform a circle check to ensure against corner deadzones
    let vo = origin - poly.verts[closest_vert];
    let udotvo = (-unit_rv).dot(vo);
    let rad_discr = udotvo * udotvo + cr2 - vo.length_squared();
    if rad_discr > 0.0 {
        // tangential and passing lines are not collisions
        let dist = -udotvo - rad_discr.sqrt();
        if dist < rv_len {
            // impale/fallshort : success/guaranteed failure
            let norm = (unit_rv * -dist - vo) / circle.rad;
            return Some((dist, norm))
        }
    }
    None // circle is (at least partially) inside poly, no collision occurs
}
#[inline]
fn shape_sweep(b1: &Body, s1: usize, b2: &Body, s2: usize, t: Fp) -> Option<(Fp, Vec2)> {
    // manual jump table implementation - decreases performance cost from 18-20ns to 3-5ns

    // Safety: functions in jump table are decided by associated ids, each function is called only when union types are correct
    const SHAPE_JUMP_TABLE: [[fn(&Body, &ShapeUnion, &Body, &ShapeUnion, Fp) -> Option<(Fp, Vec2)>; 3]; 3] = [
        [
            |b1: &Body, su1: &ShapeUnion, b2: &Body, su2: &ShapeUnion, t: Fp| unsafe { 
                circle_circle_sweep(b1, &su1.circle, b2, &su2.circle, t) },
            |b1: &Body, su1: &ShapeUnion, b2: &Body, su2: &ShapeUnion, t: Fp| unsafe { 
                circle_aabb_sweep(b1, &su1.circle, b2, &su2.aabb, t) },
            |b1: &Body, su1: &ShapeUnion, b2: &Body, su2: &ShapeUnion, t: Fp| unsafe { 
                circle_poly_sweep(b1, &su1.circle, b2, &su2.poly, t) },
        ], [
            |b1: &Body, su1: &ShapeUnion, b2: &Body, su2: &ShapeUnion, t: Fp| unsafe { 
                circle_aabb_sweep(b2, &su2.circle, b1, &su1.aabb, t).map(|r| (r.0, -r.1)) },
            |b1: &Body, su1: &ShapeUnion, b2: &Body, su2: &ShapeUnion, t: Fp| unsafe { 
                aabb_aabb_sweep(b1, &su1.aabb, b2, &su2.aabb, t) },
            |b1: &Body, su1: &ShapeUnion, b2: &Body, su2: &ShapeUnion, t: Fp| unsafe { 
                aabb_poly_sweep(b1, &su1.aabb, b2, &su2.poly, t) },
        ], [
            |b1: &Body, su1: &ShapeUnion, b2: &Body, su2: &ShapeUnion, t: Fp| unsafe { 
                circle_poly_sweep(b2, &su2.circle, b1, &su1.poly, t).map(|r| (r.0, -r.1)) },
            |b1: &Body, su1: &ShapeUnion, b2: &Body, su2: &ShapeUnion, t: Fp| unsafe { 
                aabb_poly_sweep(b2, &su2.aabb, b1, &su1.poly, t).map(|r| (r.0, -r.1)) },
            |b1: &Body, su1: &ShapeUnion, b2: &Body, su2: &ShapeUnion, t: Fp| unsafe { 
                poly_poly_sweep(b1, &su1.poly, b2, &su2.poly, t) },
        ]];

    // runtime:
    let s1s = &b1.shapes[s1];
    let s2s = &b2.shapes[s2];
    SHAPE_JUMP_TABLE[s1s.id as usize][s2s.id as usize](b1, &s1s.shape, b2, &s2s.shape, t)
}

#[derive(Debug, Clone, Copy)]
pub struct BodySweepData {
    pub b1_shape: usize,
    pub b2_shape: usize,
    /// Fraction of bodies' velocity until collision occurs.
    pub travel: Fp,
    /// Collision normal from shape of b2.
    pub norm: Vec2,
}
impl BodySweepData {
    pub fn new_invalid() -> BodySweepData {
        //! Returns a BodySweptData struct with `usize::MAX` shape indecies, `Fp::INFINITY` travel and `0.0, 0.0` normal.
        BodySweepData {
            b1_shape: usize::MAX,
            b2_shape: usize::MAX,
            travel: Fp::INFINITY,
            norm: Vec2::ZERO,
        }
    }

    pub fn invert(self) -> BodySweepData {
        //! Utility function for reversing the 'perspective' of the BodySweepData from b1 to b2.
        BodySweepData {
            b1_shape: self.b2_shape,
            b2_shape: self.b1_shape,
            travel: self.travel,
            norm: -self.norm,
        }
    }
}

pub fn body_sweep(b1: &Body, b2: &Body, t: Fp) -> Option<BodySweepData> {
    //! Returns whether a collision happened between two bodies, and the collision data if so. <br>
    //! Does not perform a broadened bounding box collision check.
    if b1.shapes.len() == 1 && b2.shapes.len() == 1 {
        // single shape optimisation
        if let Some((travel, n)) = shape_sweep(b1, 0, b2, 0, t) {
            Some(BodySweepData {
                b1_shape: 0,
                b2_shape: 0,
                travel: travel * t,
                norm: n,
            })
        } else {
            None
        }
    } else {
        let b1_aabb = &b1.get_broad();
        let b2_aabb = &b2.get_broad();
        let mut result: Option<BodySweepData> = None;
        let mut min_travel = Fp::MAX;

        //let f2: Vec<usize> = (0..b2.shapes.len()).filter(|y| b2.get_shape_broad(*y).aabb_test(b1_aabb)).collect(); // slow
        for x in (0..b1.shapes.len()).filter(|x| b2_aabb.aabb_test(&b1.get_shape_broad(*x))) {
            for y in (0..b2.shapes.len()).filter(|y| b1_aabb.aabb_test(&b2.get_shape_broad(*y))) {
                if let Some((travel, n)) = shape_sweep(b1, x, b2, y, t) {
                    if t < min_travel {
                        min_travel = t;
                        result = Some(BodySweepData {
                            b1_shape: x,
                            b2_shape: y,
                            norm: n,
                            travel: travel * t,
                        })
                    }
                }
            }
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aabb_circle_approx_sweep_test() {
        let aabb = Aabb::new(0.0, 0.0, 1.0, 1.0);
        let circle = Circle::new(1.0, 2.0, 2.0);
        assert_eq!(circle_aabb_passby(Vec2::new(0.0, -1.0).normalize(), &aabb, circle.pos, circle.rad), false);
        assert_eq!(circle_aabb_passby(Vec2::new(1.0, -1.0).normalize(), &aabb, circle.pos, circle.rad), false);


        assert_eq!(circle_aabb_passby(Vec2::new(-1.0, -1.0).normalize(), &aabb, circle.pos, circle.rad), true);
        assert_eq!(circle_aabb_passby(Vec2::new(1.0, 1.0).normalize(), &aabb, circle.pos, circle.rad), true); // does not detect backward passes
        assert_eq!(circle_aabb_passby(Vec2::new(1.0, 0.0).normalize(), &aabb, circle.pos - Vec2::new(4.0, 2.1), circle.rad), true);
    }

    #[test]
    fn aabb_aabb_approx_sweep_test() {
        let aabb1 = Aabb::new(0.0, 0.0, 1.0, 1.0);
        let aabb2 = Aabb::new(2.0, 1.0, 3.0, 2.0);

        assert_eq!(aabb_aabb_passby(Vec2::new(2.0, 0.0), &aabb1, &aabb2, Vec2::new(0.0, 0.0)), false);
        assert_eq!(aabb_aabb_passby(Vec2::new(2.0, 0.0), &aabb1, &aabb2.translate(Vec2::new(0.0, 1.0)), Vec2::new(0.0, 0.0)), false);
        assert_eq!(aabb_aabb_passby(Vec2::new(2.0, 1.0), &aabb1, &aabb2, Vec2::new(3.0, 0.0)), false);

        assert_eq!(aabb_aabb_passby(Vec2::new(2.0, 1.0), &aabb1, &aabb2, Vec2::new(0.0, 0.0)), true);
        assert_eq!(aabb_aabb_passby(Vec2::new(-2.0, -1.0), &aabb1, &aabb2, Vec2::new(0.0, 0.0)), true); // does not detect backward passes
        assert_eq!(aabb_aabb_passby(Vec2::new(2.0, 1.1), &aabb1, &aabb2.translate(Vec2::new(0.0, 1.0)), Vec2::new(0.0, 0.0)), true);
    }

    #[test]
    fn aabb_aabb_swept_test() {
        let aabb1 = Aabb::new(0.0, 0.0, 1.0, 1.0);
        let b1 = Body::new(vec![Shape::aabb(aabb1)], Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0), false);
        let aabb2 = Aabb::new(2.0, 1.0, 3.0, 2.0);
        let b2 = Body { aabb: aabb2, shapes: vec![Shape::aabb(aabb2)], pos: Vec2::new(0.2, -0.5), vel: Vec2::new(-1.0, 0.0), bullet: false };

        assert_eq!(aabb_aabb_sweep(&b1, &aabb1, &b2, &aabb2, 1.0).is_some(), true);
        assert_eq!(aabb_aabb_sweep(&b1, &aabb1, &b2, &aabb2, 100.0).is_some(), true);
        assert_eq!(aabb_aabb_sweep(&b1, &aabb1, &b2, &aabb2, -1.0).is_some(), false);
    }

    #[test]
    fn circle_circle_swept_test() {
        let c1 = Circle::new(0.5, 0.0, 0.0);
        let b1 = Body { aabb: c1.get_bounding_box(), shapes: vec![Shape::circle(c1)], pos: Vec2::new(1.0, 1.0), vel: Vec2::new(1.0, 1.0), bullet: false };
        let c2 = Circle::new(0.1, 2.0, 4.0);
        let b2 = Body { aabb: c2.get_bounding_box(), shapes: vec![Shape::circle(c2)], pos: Vec2::new(0.2, -0.5), vel: Vec2::new(0.0, -2.0), bullet: false };

        println!("{:?}", circle_circle_sweep(&b1, &c1, &b2, &c2, 1.0));
        assert_eq!(circle_circle_sweep(&b1, &c1, &b2, &c2, 1.0).is_some(), true);
        println!("{:?}", body_sweep(&b1, &b2, 1.0));
        assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);
        assert_eq!(circle_circle_sweep(&b1, &c1, &b2, &c2, 0.5).is_some(), false);
        assert_eq!(circle_circle_sweep(&b1, &c1, &b2, &c2, -1.0).is_some(), false);

        let c1 = Circle::new(0.5, 0.0, 0.0);
        let b1 = Body { aabb: c1.get_bounding_box(), shapes: vec![Shape::circle(c1)], pos: Vec2::new(2.0, 1.0), vel: Vec2::new(1.0, 1.0), bullet: false };
        let c2 = Circle::new(0.1, 2.0, 4.0);
        let b2 = Body { aabb: c2.get_bounding_box(), shapes: vec![Shape::circle(c2)], pos: Vec2::new(0.2, -0.5), vel: Vec2::new(0.0, -2.0), bullet: false };

        assert_eq!(circle_circle_sweep(&b1, &c1, &b2, &c2, 1.0).is_some(), false);
        assert_eq!(circle_circle_sweep(&b1, &c1, &b2, &c2, 0.5).is_some(), false);
        assert_eq!(circle_circle_sweep(&b1, &c1, &b2, &c2, -1.0).is_some(), false);
    }

    #[test]
    fn aabb_circle_swept() {
        let a1 = Aabb::new(0.0, 0.0, 1.0, 1.0);
        let b1 = Body { aabb: a1, shapes: vec![Shape::aabb(a1)], pos: Vec2::new(0.0, 0.0), vel: Vec2::new(0.0, 0.0), bullet: false };
        let c2 = Circle::new(0.5, 1.0, 1.0);
        let b2 = Body { aabb: c2.get_bounding_box(), shapes: vec![Shape::circle(c2)], pos: Vec2::new(1.0, 1.4), vel: Vec2::new(-2.0, -2.0), bullet: false };

        println!("{:?}", body_sweep(&b1, &b2, 1.0));
        assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);

        let a1 = Aabb::new(-0.5, -0.5, 0.5, 0.5);
        let b1 = Body { aabb: a1, shapes: vec![Shape::aabb(a1)], pos: Vec2::new(1.0, 1.0), vel: Vec2::new(1.0, 2.0), bullet: false };
        let c2 = Circle::new(0.5, 2.0, 4.5);
        let b2 = Body { aabb: c2.get_bounding_box(), shapes: vec![Shape::circle(c2)], pos: Vec2::new(0.2, -0.5), vel: Vec2::new(0.0, -2.0), bullet: false };

        println!("{:?}", body_sweep(&b1, &b2, 1.0));
        assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);
    }

    #[test]
    fn poly_circle_swept_test() {
        // copy of circle tests, both succeed
        let p1 = Poly::new(&[Vec2::new(0.0, 0.5), Vec2::new(0.5, 0.2), Vec2::new(0.5, -0.2), Vec2::new(0.0, -0.5), Vec2::new(-0.5, -0.2), Vec2::new(-0.5, 0.2)]);
        let b1 = Body { aabb: p1.aabb, shapes: vec![Shape::poly(p1.clone())], pos: Vec2::new(1.0, 1.0), vel: Vec2::new(1.0, 1.0), bullet: false };
        let c2 = Circle::new(0.1, 2.0, 4.0);
        let b2 = Body { aabb: c2.get_bounding_box(), shapes: vec![Shape::circle(c2)], pos: Vec2::new(0.2, -0.5), vel: Vec2::new(0.0, -2.0), bullet: false };

        println!("{:?}", body_sweep(&b1, &b2, 1.0));
        assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);

        let p1 = Poly::new(&[Vec2::new(0.0, 0.5), Vec2::new(0.5, 0.2), Vec2::new(0.5, -0.2), Vec2::new(0.0, -0.5), Vec2::new(-0.5, -0.2), Vec2::new(-0.5, 0.2)]).translate(Vec2::new(0.5, 0.5));
        let b1 = Body { aabb: p1.aabb, shapes: vec![Shape::poly(p1.clone())], pos: Vec2::new(2.0, 1.0), vel: Vec2::new(1.0, 1.0), bullet: false };
        let c2 = Circle::new(0.1, 2.0, 4.0);
        let b2 = Body { aabb: c2.get_bounding_box(), shapes: vec![Shape::circle(c2)], pos: Vec2::new(0.2, -0.5), vel: Vec2::new(0.0, -2.0), bullet: false };

        assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), false);
        assert_eq!(body_sweep(&b1, &b2, 0.5).is_some(), false);
        assert_eq!(body_sweep(&b1, &b2, -1.0).is_some(), false);
    }

    #[test]
    fn poly_aabb_swept_test() {
        let poly = Poly::new(&[Vec2::new(0.0, 0.5), Vec2::new(0.5, 0.2), Vec2::new(0.5, -0.2), Vec2::new(0.0, -0.5), Vec2::new(-0.5, -0.2), Vec2::new(-0.5, 0.2)]).translate(Vec2::new(0.5, 0.5));
        let b1 = Body { aabb: poly.aabb, shapes: vec![Shape::poly(poly)], pos: Vec2::new(0.2, 0.2), vel: Vec2::new(1.0, 0.1), bullet: false };
        let aabb = Aabb::new(0.0, 0.0, 1.0, 1.0).translate(Vec2::new(3.0, 0.0));
        let b2 = Body { aabb: aabb, shapes: vec![Shape::aabb(aabb)], pos: Vec2::new(0.1, -0.2), vel: Vec2::new(-4.0, 0.0), bullet: false };

        //dbg!(b2.clone());
        println!("{:?}", body_sweep(&b1, &b2, 0.3));
        println!("{:?}", body_sweep(&b1, &b2, 1.0));
        println!("{:?}", body_sweep(&b1, &b2, 4.0));
        assert_eq!(body_sweep(&b1, &b2, 0.3).is_some(), false);
        assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);
        assert_eq!(body_sweep(&b1, &b2, 5.0).is_some(), true);

        let poly = Poly::new(&[Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)]);
        let b1 = Body { aabb: poly.aabb, shapes: vec![Shape::poly(poly)], pos: Vec2::new(0.0, 0.0), vel: Vec2::new(1.0, 1.0), bullet: false };
        let aabb = Aabb::new(0.0, 0.0, 1.0, 1.0).translate(Vec2::new(2.0, 1.0));
        let b2 = Body { aabb, shapes: vec![Shape::aabb(aabb)], pos: Vec2::new(0.2, -0.5), vel: Vec2::new(-1.0, 0.0), bullet: false };

        assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);
    }

    #[test]
    fn test_poly_sweep() {
        let p1 = Poly::new(&[Vec2::new(0.0, 0.5), Vec2::new(0.5, 0.2), Vec2::new(0.5, -0.2), Vec2::new(0.0, -0.5), Vec2::new(-0.5, -0.2), Vec2::new(-0.5, 0.2)]).translate(Vec2::new(0.5, 0.5));
        let b1 = Body { aabb: p1.aabb, shapes: vec![Shape::poly(p1)], pos: Vec2::new(0.2, 0.2), vel: Vec2::new(1.0, 0.1), bullet: false };
        let p2 = Poly::new(&[Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)]).translate(Vec2::new(3.0, 0.0));
        let b2 = Body { aabb: p2.aabb, shapes: vec![Shape::poly(p2)], pos: Vec2::new(0.1, -0.2), vel: Vec2::new(-4.0, 0.0), bullet: false };

        println!("{:?}", body_sweep(&b1, &b2, 0.3));
        println!("{:?}", body_sweep(&b1, &b2, 1.0));
        println!("{:?}", body_sweep(&b1, &b2, 4.0));
        assert_eq!(body_sweep(&b1, &b2, 0.3).is_some(), false);
        assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);
        assert_eq!(body_sweep(&b1, &b2, 5.0).is_some(), true);

        let p1 = Poly::new(&[Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)]);
        let b1 = Body { aabb: p1.aabb, shapes: vec![Shape::poly(p1)], pos: Vec2::new(0.0, 0.0), vel: Vec2::new(1.0, 1.0), bullet: false };
        let p2 = Poly::new(&[Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)]).translate(Vec2::new(2.0, 1.0));
        let b2 = Body { aabb: p2.aabb, shapes: vec![Shape::poly(p2)], pos: Vec2::new(0.2, -0.5), vel: Vec2::new(-1.0, 0.0), bullet: false };

        assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);

        let p1 = Poly::new(&[Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)]);
        let b1 = Body { aabb: p1.aabb, shapes: vec![Shape::poly(p1)], pos: Vec2::new(0.0, 0.0), vel: Vec2::new(2.0, 1.0), bullet: false };
        let p2 = Poly::new(&[Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0), Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)]).translate(Vec2::new(1.0, 0.0));
        let b2 = Body { aabb: p2.aabb, shapes: vec![Shape::poly(p2)], pos: Vec2::new(1.0, -0.5), vel: Vec2::new(-2.0, 0.0), bullet: false };

        assert_eq!(body_sweep(&b1, &b2, 1.0).is_some(), true);
    }
}
