//! C FFI bindings for ALICE-Motion
//!
//! 12 extern "C" functions for Unity / UE5 / any C-compatible host.
//!
//! Author: Moroya Sakamoto

use crate::bezier::CubicBezier;
use crate::nurbs::NurbsCurve;
use crate::planner::MotionPlan;
use crate::vec3::Vec3;

// ---------------------------------------------------------------------------
// Vec3 (2)
// ---------------------------------------------------------------------------

/// Create a Vec3. Returns [x, y, z] packed as 3 consecutive f32.
///
/// # Safety
/// `out` must point to a buffer of at least 3 f32.
#[no_mangle]
pub unsafe extern "C" fn alice_motion_vec3_new(x: f32, y: f32, z: f32, out: *mut f32) {
    let v = Vec3::new(x, y, z);
    *out = v.x;
    *out.add(1) = v.y;
    *out.add(2) = v.z;
}

/// Euclidean distance between two points.
#[no_mangle]
pub extern "C" fn alice_motion_vec3_distance(
    ax: f32,
    ay: f32,
    az: f32,
    bx: f32,
    by: f32,
    bz: f32,
) -> f32 {
    Vec3::new(ax, ay, az).distance(Vec3::new(bx, by, bz))
}

// ---------------------------------------------------------------------------
// CubicBezier (4)
// ---------------------------------------------------------------------------

/// Create a CubicBezier on the heap. Caller must free with `alice_bezier_destroy`.
#[no_mangle]
pub extern "C" fn alice_bezier_new(
    p0x: f32,
    p0y: f32,
    p0z: f32,
    p1x: f32,
    p1y: f32,
    p1z: f32,
    p2x: f32,
    p2y: f32,
    p2z: f32,
    p3x: f32,
    p3y: f32,
    p3z: f32,
) -> *mut CubicBezier {
    let b = CubicBezier::new(
        Vec3::new(p0x, p0y, p0z),
        Vec3::new(p1x, p1y, p1z),
        Vec3::new(p2x, p2y, p2z),
        Vec3::new(p3x, p3y, p3z),
    );
    Box::into_raw(Box::new(b))
}

/// Evaluate position at parameter t. Writes [x,y,z] to `out`.
///
/// # Safety
/// `ptr` must be a valid pointer from `alice_bezier_new`.
/// `out` must point to a buffer of at least 3 f32.
#[no_mangle]
pub unsafe extern "C" fn alice_bezier_position(ptr: *const CubicBezier, t: f32, out: *mut f32) {
    let b = &*ptr;
    let p = b.position(t);
    *out = p.x;
    *out.add(1) = p.y;
    *out.add(2) = p.z;
}

/// Approximate arc length with given subdivisions.
///
/// # Safety
/// `ptr` must be a valid pointer from `alice_bezier_new`.
#[no_mangle]
pub unsafe extern "C" fn alice_bezier_arc_length(
    ptr: *const CubicBezier,
    subdivisions: u32,
) -> f32 {
    let b = &*ptr;
    b.arc_length(subdivisions as usize)
}

/// Free a CubicBezier.
///
/// # Safety
/// `ptr` must be a valid pointer from `alice_bezier_new`, or null.
#[no_mangle]
pub unsafe extern "C" fn alice_bezier_destroy(ptr: *mut CubicBezier) {
    if !ptr.is_null() {
        drop(Box::from_raw(ptr));
    }
}

// ---------------------------------------------------------------------------
// MotionPlan (6)
// ---------------------------------------------------------------------------

/// Create a MotionPlan with Bezier path + trapezoidal profile.
#[no_mangle]
pub extern "C" fn alice_plan_bezier_trapezoidal(
    p0x: f32,
    p0y: f32,
    p0z: f32,
    p3x: f32,
    p3y: f32,
    p3z: f32,
    v_max: f32,
    a_max: f32,
) -> *mut MotionPlan {
    let curve = CubicBezier::from_endpoints(Vec3::new(p0x, p0y, p0z), Vec3::new(p3x, p3y, p3z));
    let plan = MotionPlan::bezier_trapezoidal(curve, v_max, a_max);
    Box::into_raw(Box::new(plan))
}

/// Create a MotionPlan with NURBS path + S-curve profile.
///
/// # Safety
/// `points` must point to `n_points * 3` consecutive f32 values.
#[no_mangle]
pub unsafe extern "C" fn alice_plan_nurbs_scurve(
    points: *const f32,
    n_points: u32,
    v_max: f32,
    a_max: f32,
    j_max: f32,
) -> *mut MotionPlan {
    let n = n_points as usize;
    let mut builder = NurbsCurve::builder(3);
    for i in 0..n {
        let base = i * 3;
        let x = *points.add(base);
        let y = *points.add(base + 1);
        let z = *points.add(base + 2);
        builder = builder.add_point(Vec3::new(x, y, z));
    }
    let curve = builder.build();
    let plan = MotionPlan::nurbs_scurve(curve, v_max, a_max, j_max);
    Box::into_raw(Box::new(plan))
}

/// Evaluate position at time t. Writes [x,y,z] to `out`.
///
/// # Safety
/// `ptr` must be a valid pointer from a plan constructor.
/// `out` must point to a buffer of at least 3 f32.
#[no_mangle]
pub unsafe extern "C" fn alice_plan_position(ptr: *const MotionPlan, t: f32, out: *mut f32) {
    let plan = &*ptr;
    let p = plan.position(t);
    *out = p.x;
    *out.add(1) = p.y;
    *out.add(2) = p.z;
}

/// Speed (scalar) at time t.
///
/// # Safety
/// `ptr` must be a valid pointer from a plan constructor.
#[no_mangle]
pub unsafe extern "C" fn alice_plan_speed(ptr: *const MotionPlan, t: f32) -> f32 {
    let plan = &*ptr;
    plan.speed(t)
}

/// Total duration in seconds.
///
/// # Safety
/// `ptr` must be a valid pointer from a plan constructor.
#[no_mangle]
pub unsafe extern "C" fn alice_plan_duration(ptr: *const MotionPlan) -> f32 {
    let plan = &*ptr;
    plan.duration()
}

/// Free a MotionPlan.
///
/// # Safety
/// `ptr` must be a valid pointer from a plan constructor, or null.
#[no_mangle]
pub unsafe extern "C" fn alice_plan_destroy(ptr: *mut MotionPlan) {
    if !ptr.is_null() {
        drop(Box::from_raw(ptr));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vec3_new_ffi() {
        let mut out = [0.0f32; 3];
        unsafe { alice_motion_vec3_new(1.0, 2.0, 3.0, out.as_mut_ptr()) };
        assert_eq!(out, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_vec3_distance_ffi() {
        let d = alice_motion_vec3_distance(0.0, 0.0, 0.0, 3.0, 4.0, 0.0);
        assert!((d - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_bezier_roundtrip_ffi() {
        let ptr = alice_bezier_new(0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 2.0, 1.0, 0.0, 3.0, 0.0, 0.0);
        assert!(!ptr.is_null());

        let mut out = [0.0f32; 3];
        unsafe { alice_bezier_position(ptr, 0.0, out.as_mut_ptr()) };
        assert!((out[0]).abs() < 0.01);

        let len = unsafe { alice_bezier_arc_length(ptr, 64) };
        assert!(len > 0.0);

        unsafe { alice_bezier_destroy(ptr) };
    }

    #[test]
    fn test_plan_bezier_trapezoidal_ffi() {
        let ptr = alice_plan_bezier_trapezoidal(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 2.0, 4.0);
        assert!(!ptr.is_null());

        let dur = unsafe { alice_plan_duration(ptr) };
        assert!(dur > 0.0);

        let speed = unsafe { alice_plan_speed(ptr, dur / 2.0) };
        assert!(speed > 0.1);

        let mut out = [0.0f32; 3];
        unsafe { alice_plan_position(ptr, 0.0, out.as_mut_ptr()) };
        assert!(out[0].abs() < 0.5);

        unsafe { alice_plan_destroy(ptr) };
    }

    #[test]
    fn test_plan_nurbs_scurve_ffi() {
        let points: [f32; 12] = [0.0, 0.0, 0.0, 3.0, 5.0, 0.0, 7.0, 5.0, 0.0, 10.0, 0.0, 0.0];
        let ptr = unsafe { alice_plan_nurbs_scurve(points.as_ptr(), 4, 2.0, 4.0, 20.0) };
        assert!(!ptr.is_null());

        let dur = unsafe { alice_plan_duration(ptr) };
        assert!(dur > 0.0);

        unsafe { alice_plan_destroy(ptr) };
    }

    #[test]
    fn test_bezier_destroy_null_safe() {
        unsafe { alice_bezier_destroy(core::ptr::null_mut()) };
    }

    #[test]
    fn test_plan_destroy_null_safe() {
        unsafe { alice_plan_destroy(core::ptr::null_mut()) };
    }
}
