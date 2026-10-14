//! Booleans of bodies with faces in a common plane: a face is pushed out of the plane only where
//! that leaves the result as it is. Every expected volume is what the two bodies define.
#![allow(clippy::unwrap_used, clippy::panic)]
use crate::*;
use solidcraft_geom::Vec3;

/// An l x w x h block with two bottom edges chamfered by 1 mm: no prism along any axis, so
/// its booleans are not done on sections.
fn block(l: f64, w: f64, h: f64) -> Body {
    let b = box_solid(Vec3::ZERO, Vec3::new(l, w, h)).unwrap();
    let b = chamfer(&b, &[Vec3::new(l / 2.0, w, 0.0)], 1.0).unwrap();
    chamfer(&b, &[Vec3::new(l, w / 2.0, 0.0)], 1.0).unwrap()
}

/// A 200 x 100 x 20 block with a rib across it: 20 wide over the block's 100, from the top
/// face z = 20 up to z = 30.
fn ribbed() -> Body {
    let rib = box_solid(Vec3::new(90.0, 0.0, 20.0), Vec3::new(110.0, 100.0, 30.0)).unwrap();
    boolean(&block(200.0, 100.0, 20.0), &rib, BoolOp::Union).unwrap().unwrap()
}

/// A channel along that block, longer than its 200: y 30 to 70 and z 10 to 20, so its top
/// lies in the plane of the face the rib stands on.
fn channel() -> Body {
    box_solid(Vec3::new(-20.0, 30.0, 10.0), Vec3::new(220.0, 70.0, 20.0)).unwrap()
}

/// What the boolean of `a` and `b` adds to a's volume, the result's lumps, and how it was made.
fn change(a: &Body, b: &Body, op: BoolOp) -> (f64, usize, BooleanDiagnostics) {
    let (r, how) = capture_boolean_diagnostics(|| boolean(a, b, op));
    let r = r.unwrap().unwrap();
    (measure(&r).unwrap().volume - measure(a).unwrap().volume, r.lumps().unwrap().len(), how)
}

fn used(how: &BooleanDiagnostics, strategy: BooleanStrategy) -> bool {
    how.events.iter().any(|e| e.strategy == strategy)
}

/// A channel cut along a block, up to the face a rib stands on. The channel's top lies in
/// that face's plane and the rib bridges it: pushed up out of the plane (2.29 % of the
/// body's size, 5.6 mm), the top would cut a notch 20 x 40 into the rib.
#[test]
fn a_cut_up_to_a_face_leaves_what_stands_on_the_face() {
    let (added, lumps, how) = change(&ribbed(), &channel(), BoolOp::Cut);
    // It takes 200 x 40 x 10 of the block and nothing of the rib.
    assert!((added + 80_000.0).abs() < 1e-6 && lumps == 1, "{added}, {lumps} lumps, {how:?}");
    assert!(!used(&how, BooleanStrategy::CoplanarSeparation), "{how:?}");
}

/// The same with a round hole through the block beside the channel (r 4 at 30, 15: y 11 to
/// 19, the channel from 30 on). The body no longer has planar faces only, but the hole lies
/// clear of the tool, and the cut stays what the channel is.
#[test]
fn a_round_hole_beside_the_cut_changes_nothing() {
    let hole = cylinder(Vec3::new(30.0, 15.0, -1.0), Vec3::Z, 4.0, 22.0).unwrap();
    let drilled = boolean(&ribbed(), &hole, BoolOp::Cut).unwrap().unwrap();
    let (added, lumps, how) = change(&drilled, &channel(), BoolOp::Cut);
    // 200 x 40 x 10 again (the hole is measured on its mesh, before and after).
    assert!((added + 80_000.0).abs() < 1e-3 && lumps == 1, "{added}, {lumps} lumps, {how:?}");
    assert!(!used(&how, BooleanStrategy::CoplanarSeparation), "{how:?}");
}

/// With nothing in their way the faces are pushed as before: a pocket's top out of the plane
/// of the block's top face, a pad's foot into the block. (This holds without the check too:
/// it keeps the check from refusing what is harmless.)
#[test]
fn faces_with_room_are_pushed_apart() {
    let block = block(200.0, 100.0, 20.0);
    // A pocket 50 x 60 x 5 from the top face: 15 000 less.
    let pocket = box_solid(Vec3::new(75.0, 20.0, 15.0), Vec3::new(125.0, 80.0, 20.0)).unwrap();
    let (added, _, how) = change(&block, &pocket, BoolOp::Cut);
    assert!((added + 15_000.0).abs() < 1e-6 && used(&how, BooleanStrategy::CoplanarSeparation), "{added}, {how:?}");
    // A pad 50 x 60 x 10 on the top face: 30 000 more.
    let pad = box_solid(Vec3::new(75.0, 20.0, 20.0), Vec3::new(125.0, 80.0, 30.0)).unwrap();
    let (added, lumps, how) = change(&block, &pad, BoolOp::Union);
    assert!((added - 30_000.0).abs() < 1e-6 && lumps == 1 && used(&how, BooleanStrategy::CoplanarSeparation), "{added}, {how:?}");
}

/// A wing: a flat-bottomed airfoil section through fit points, as a smooth spline through
/// them (one cubic per interval, Catmull-Rom tangents), lofted from the root at y = 0 to a
/// smaller tip at y = 600, 52.5 higher (dihedral). Each cubic gives one free-form side.
fn spline_wing() -> Body {
    use solidcraft_geom::{Loop2, Plane, Seg2, Vec2};
    const UPPER: [(f64, f64); 17] = [
        (0.0, 0.0),
        (0.0125, 0.034),
        (0.025, 0.048),
        (0.05, 0.066),
        (0.075, 0.078),
        (0.1, 0.087),
        (0.15, 0.1),
        (0.2, 0.108),
        (0.3, 0.117),
        (0.4, 0.113),
        (0.5, 0.104),
        (0.6, 0.091),
        (0.7, 0.074),
        (0.8, 0.053),
        (0.9, 0.028),
        (0.95, 0.015),
        (1.0, 0.0),
    ];
    const LOWER: [(f64, f64); 12] = [
        (1.0, 0.0),
        (0.7, 0.0),
        (0.5, 0.0),
        (0.4, -0.001),
        (0.3, -0.005),
        (0.2, -0.012),
        (0.15, -0.016),
        (0.1, -0.019),
        (0.05, -0.019),
        (0.025, -0.016),
        (0.0125, -0.012),
        (0.0, 0.0),
    ];
    let cubics = |pts: &[(f64, f64)], c: f64, segs: &mut Vec<Seg2>| {
        let p: Vec<Vec2> = pts.iter().map(|(x, y)| Vec2::new(x * c, y * c)).collect();
        let tangent = |i: usize| (p[(i + 1).min(p.len() - 1)] - p[i.saturating_sub(1)]) * 0.5;
        for i in 0..p.len() - 1 {
            segs.push(Seg2::Cubic { p0: p[i], p1: p[i] + tangent(i) / 3.0, p2: p[i + 1] - tangent(i + 1) / 3.0, p3: p[i + 1] });
        }
    };
    let section = |c: f64| {
        let mut segs = Vec::new();
        cubics(&UPPER, c, &mut segs);
        cubics(&LOWER, c, &mut segs);
        Loop2 { segs }
    };
    // Sketch x runs backwards from the leading edge, sketch y up: the normal is +Y, along the span.
    let plane = |o: Vec3| Plane { origin: o, x: -Vec3::X, y: Vec3::Z };
    loft(&[(plane(Vec3::new(60.0, 0.0, 35.0)), section(200.0)), (plane(Vec3::new(50.0, 600.0, 87.5)), section(160.0))]).unwrap()
}

/// A spline wing joined to a fuselage, then its mirror image joined as well (the mirrored root
/// cap lies back to back on the original's): the join must finish in seconds, not minutes. The
/// partial-contact imprint used to trace every free-form skin of the mirrored wing over the
/// root cap, one global nearest-point search per sample.
#[test]
fn mirrored_spline_wing_joins_its_fuselage_in_bounded_time() {
    let fuselage = box_solid(Vec3::new(-250.0, -32.0, -37.0), Vec3::new(250.0, 32.0, 37.0)).unwrap();
    let wing = spline_wing();
    assert!(wing.face_count() > 20, "{} faces", wing.face_count());
    // Its curved skins are free-form split tools (the flat bottom and the caps are not); the
    // fuselage's faces are not.
    let skins = (0..wing.face_count()).filter(|f| crate::splitface::is_free_form_tool(&wing, *f)).count();
    assert!(skins >= 20, "{skins} free-form faces of {}", wing.face_count());
    assert!((0..fuselage.face_count()).all(|f| !crate::splitface::is_free_form_tool(&fuselage, f)));
    let a = boolean(&fuselage, &wing, BoolOp::Union).unwrap().unwrap();
    let mirrored = transform_matrix(&wing, [[1.0, 0.0, 0.0, 0.0], [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]).unwrap();
    let start = std::time::Instant::now();
    let r = boolean(&a, &mirrored, BoolOp::Union).unwrap().unwrap();
    let took = start.elapsed();
    assert!(took.as_secs() < 120, "the mirrored join took {took:?}");
    let (va, vw, vr) = (measure(&a).unwrap().volume, measure(&wing).unwrap().volume, measure(&r).unwrap().volume);
    // The mirrored wing adds a second wing, less the part of it inside the fuselage.
    assert!(vr > va + 0.5 * vw && vr < va + vw, "volumes: fuselage+wing {va}, wing {vw}, both wings {vr}");
    assert_eq!(r.shell_count(), 1);
    assert!(r.validity().is_empty(), "{:?}", r.validity());
}
