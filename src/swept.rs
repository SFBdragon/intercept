use cgmath::{InnerSpace, Vector2};
use super::inters::{Shape, ShapeType, AABB};
use super::utils::*;


// ---------- Bodies ---------- //

pub struct StaticBody {
   /// Posistion
   pub pos: Vector2<f64>,
   /// Compositing shapes
   pub shapes: Vec<Shape>,
}

impl StaticBody {
   pub fn new(shapes: Vec<Shape>) -> StaticBody {
      StaticBody { pos: v2f64_zero(), shapes }
   }
}

pub struct Body {
   /// Posistion
   pub pos: Vector2<f64>,
   /// Compositing shapes
   pub shapes: Vec<Shape>,
   /// Bounding box
   pub aabb: AABB,
   /// Velocity
   pub vel: Vector2<f64>,
}

impl Body {
   pub fn new(shapes: Vec<Shape>) -> Body {
      panic!(); // todo aabb create
      Body { pos: v2f64_zero(), vel: v2f64_zero(), aabb, shapes }
   }
}

pub struct SweptDynStatData {
   /// Index of the colliding shape of body.
   pub body_shape: usize,
   /// Index of the colliding shape of stat.
   pub stat_shape: usize,
   /// Collision normal from [] to []. // todo
   pub norm: Vector2<f64>,
   /// Fraction of body's velocity until collision.
   pub travel: f64,
}

pub fn body_stat_swept(body: &Body, stat: &StaticBody) -> Option<SweptDynStatData> {
   let mut result: Option<SweptDynStatData> = None;
   let mut travel2 = f64::MAX;

   let v2 = body.vel.magnitude2() as f64;
   for (x, sd) in body.shapes.iter().enumerate() {
      for (y, ss) in stat.shapes.iter().enumerate() {
         // get data
         let d2: f64;
         let n: Vector2<f64>;

         match &sd.kind {
            &ShapeType::Aabb { .. } => match &ss.kind {
               &ShapeType::Aabb { .. } => aabb_aabb_body_stat_swept(),
               &ShapeType::Circle { } => ,
            },
         }

         if d2 < v2 && d2 < travel2 {
            travel2 = d2;
            result = Some(SweptDynStatData {
               body_shape: x,
               stat_shape: y,
               norm: n,
               travel: d2.sqrt(),
            })
         }
      }
   }

   result
}

fn aabb_aabb_body_stat_swept()