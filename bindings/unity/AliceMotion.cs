// AliceMotion.cs — Unity P/Invoke bindings for alice-motion
// Author: Moroya Sakamoto

using System;
using System.Runtime.InteropServices;

namespace Alice.Motion
{
    public static class AliceMotionNative
    {
#if UNITY_IOS && !UNITY_EDITOR
        private const string Lib = "__Internal";
#else
        private const string Lib = "alice_motion";
#endif

        // Vec3
        [DllImport(Lib)] public static extern void alice_motion_vec3_new(float x, float y, float z, float[] outVec);
        [DllImport(Lib)] public static extern float alice_motion_vec3_distance(float ax, float ay, float az, float bx, float by, float bz);

        // CubicBezier
        [DllImport(Lib)] public static extern IntPtr alice_bezier_new(float p0x, float p0y, float p0z, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z, float p3x, float p3y, float p3z);
        [DllImport(Lib)] public static extern void alice_bezier_position(IntPtr ptr, float t, float[] outVec);
        [DllImport(Lib)] public static extern float alice_bezier_arc_length(IntPtr ptr, uint subdivisions);
        [DllImport(Lib)] public static extern void alice_bezier_destroy(IntPtr ptr);

        // MotionPlan
        [DllImport(Lib)] public static extern IntPtr alice_plan_bezier_trapezoidal(float p0x, float p0y, float p0z, float p3x, float p3y, float p3z, float vMax, float aMax);
        [DllImport(Lib)] public static extern IntPtr alice_plan_nurbs_scurve(float[] points, uint nPoints, float vMax, float aMax, float jMax);
        [DllImport(Lib)] public static extern void alice_plan_position(IntPtr ptr, float t, float[] outVec);
        [DllImport(Lib)] public static extern float alice_plan_speed(IntPtr ptr, float t);
        [DllImport(Lib)] public static extern float alice_plan_duration(IntPtr ptr);
        [DllImport(Lib)] public static extern void alice_plan_destroy(IntPtr ptr);
    }

    public class MotionPlanHandle : IDisposable
    {
        private IntPtr _ptr;

        public MotionPlanHandle(IntPtr ptr) { _ptr = ptr; }

        public float Duration => AliceMotionNative.alice_plan_duration(_ptr);

        public UnityEngine.Vector3 Position(float t)
        {
            float[] buf = new float[3];
            AliceMotionNative.alice_plan_position(_ptr, t, buf);
            return new UnityEngine.Vector3(buf[0], buf[1], buf[2]);
        }

        public float Speed(float t) => AliceMotionNative.alice_plan_speed(_ptr, t);

        public void Dispose()
        {
            if (_ptr != IntPtr.Zero)
            {
                AliceMotionNative.alice_plan_destroy(_ptr);
                _ptr = IntPtr.Zero;
            }
        }
    }
}
