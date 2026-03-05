// AliceMotion.h — UE5 C FFI header for alice-motion
// Author: Moroya Sakamoto

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Vec3
void  alice_motion_vec3_new(float x, float y, float z, float* out);
float alice_motion_vec3_distance(float ax, float ay, float az, float bx, float by, float bz);

// CubicBezier
void* alice_bezier_new(float p0x, float p0y, float p0z,
                       float p1x, float p1y, float p1z,
                       float p2x, float p2y, float p2z,
                       float p3x, float p3y, float p3z);
void  alice_bezier_position(const void* ptr, float t, float* out);
float alice_bezier_arc_length(const void* ptr, unsigned int subdivisions);
void  alice_bezier_destroy(void* ptr);

// MotionPlan
void* alice_plan_bezier_trapezoidal(float p0x, float p0y, float p0z,
                                    float p3x, float p3y, float p3z,
                                    float v_max, float a_max);
void* alice_plan_nurbs_scurve(const float* points, unsigned int n_points,
                              float v_max, float a_max, float j_max);
void  alice_plan_position(const void* ptr, float t, float* out);
float alice_plan_speed(const void* ptr, float t);
float alice_plan_duration(const void* ptr);
void  alice_plan_destroy(void* ptr);

#ifdef __cplusplus
}
#endif
