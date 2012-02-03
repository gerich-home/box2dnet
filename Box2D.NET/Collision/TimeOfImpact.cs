// ****************************************************************************
// Copyright (c) 2011, Daniel Murphy
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// ****************************************************************************

using System.Diagnostics;
using Box2D.Common;
using Box2D.Pooling;

namespace Box2D.Collision
{

    /// <summary>
    /// Class used for computing the time of impact.
    /// This class should not be constructed usually, just retrieve from the {@link SingletonPool#getTOI()}.
    /// </summary>
    /// <author>daniel</author>
    public class TimeOfImpact
    {
        public enum TOIOutputState
        {
            Unknown,
            Failed,
            Overlapped,
            Touching,
            Separated
        }

        public const int MAX_ITERATIONS = 1000;

        public static int ToiCalls = 0;
        public static int ToiIters = 0;
        public static int ToiMaxIters = 0;
        public static int ToiRootIters = 0;
        public static int ToiMaxRootIters = 0;

        /// <summary>
        /// Input parameters for TOI
        /// </summary>
        /// <author>Daniel Murphy</author>
        public class TOIInput
        {
            public readonly Distance.DistanceProxy ProxyA = new Distance.DistanceProxy();
            public readonly Distance.DistanceProxy ProxyB = new Distance.DistanceProxy();
            public readonly Sweep SweepA = new Sweep();
            public readonly Sweep SweepB = new Sweep();

            /// <summary>
            /// Defines sweep interval [0, tMax]
            /// </summary>
            public float TMax;
        }

        /// <summary>
        /// Output parameters for TimeOfImpact
        /// </summary>
        /// <author>daniel</author>
        public class TOIOutput
        {
            public TOIOutputState State;
            public float T;
        }


        // djm pooling
        private readonly Distance.SimplexCache cache = new Distance.SimplexCache();
        private readonly DistanceInput distanceInput = new DistanceInput();
        private readonly Transform xfA = new Transform();
        private readonly Transform xfB = new Transform();
        private readonly DistanceOutput distanceOutput = new DistanceOutput();
        private readonly SeparationFunction fcn = new SeparationFunction();
        private readonly int[] indexes = new int[2];
        private readonly Sweep sweepA = new Sweep();
        private readonly Sweep sweepB = new Sweep();


        private readonly IWorldPool pool;

        public TimeOfImpact(IWorldPool argPool)
        {
            pool = argPool;
        }

        /// <summary>
        /// Compute the upper bound on time before two shapes penetrate. Time is represented as a fraction
        /// between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
        /// non-tunneling collision. If you change the time interval, you should call this function again.
        /// Note: use Distance to compute the contact point and normal at the time of impact.
        /// </summary>
        /// <param name="output"></param>
        /// <param name="input"></param>
        public void GetTimeOfImpact(TOIOutput output, TOIInput input)
        {
            // CCD via the local separating axis method. This seeks progression
            // by computing the largest time at which separation is maintained.

            ++ToiCalls;

            output.State = TOIOutputState.Unknown;
            output.T = input.TMax;

            Distance.DistanceProxy proxyA = input.ProxyA;
            Distance.DistanceProxy proxyB = input.ProxyB;

            sweepA.set_Renamed(input.SweepA);
            sweepB.set_Renamed(input.SweepB);

            // Large rotations can make the root finder fail, so we normalize the
            // sweep angles.
            sweepA.normalize();
            sweepB.normalize();

            float tMax = input.TMax;

            float totalRadius = proxyA.Radius + proxyB.Radius;
            // djm: whats with all these constants?
            float target = MathUtils.Max(Settings.linearSlop, totalRadius - 3.0f * Settings.linearSlop);
            const float tolerance = 0.25f * Settings.linearSlop;

            Debug.Assert(target > tolerance);

            float t1 = 0f;
            int iter = 0;

            cache.Count = 0;
            distanceInput.ProxyA = input.ProxyA;
            distanceInput.ProxyB = input.ProxyB;
            distanceInput.UseRadii = false;

            // The outer loop progressively attempts to compute new separating axes.
            // This loop terminates when an axis is repeated (no progress is made).
            for (; ; )
            {
                sweepA.getTransform(xfA, t1);
                sweepB.getTransform(xfB, t1);
                // System.out.printf("sweepA: %f, %f, sweepB: %f, %f\n",
                // sweepA.c.x, sweepA.c.y, sweepB.c.x, sweepB.c.y);
                // Get the distance between shapes. We can also use the results
                // to get a separating axis
                distanceInput.TransformA = xfA;
                distanceInput.TransformB = xfB;
                pool.GetDistance().GetDistance(distanceOutput, cache, distanceInput);

                // System.out.printf("Dist: %f at points %f, %f and %f, %f.  %d iterations\n",
                // distanceOutput.distance, distanceOutput.pointA.x, distanceOutput.pointA.y,
                // distanceOutput.pointB.x, distanceOutput.pointB.y,
                // distanceOutput.iterations);

                // If the shapes are overlapped, we give up on continuous collision.
                if (distanceOutput.Distance <= 0f)
                {
                    // System.out.println("failure, overlapped");
                    // Failure!
                    output.State = TOIOutputState.Overlapped;
                    output.T = 0f;
                    break;
                }

                if (distanceOutput.Distance < target + tolerance)
                {
                    // System.out.println("touching, victory");
                    // Victory!
                    output.State = TOIOutputState.Touching;
                    output.T = t1;
                    break;
                }

                // Initialize the separating axis.
                fcn.Initialize(cache, proxyA, sweepA, proxyB, sweepB, t1);

                // Compute the TOI on the separating axis. We do this by successively
                // resolving the deepest point. This loop is bounded by the number of
                // vertices.
                bool done = false;
                float t2 = tMax;
                int pushBackIter = 0;
                for (; ; )
                {

                    // Find the deepest point at t2. Store the witness point indices.
                    float s2 = fcn.FindMinSeparation(indexes, t2);
                    // System.out.printf("s2: %f\n", s2);
                    // Is the final configuration separated?
                    if (s2 > target + tolerance)
                    {
                        // Victory!
                        // System.out.println("separated");
                        output.State = TOIOutputState.Separated;
                        output.T = tMax;
                        done = true;
                        break;
                    }

                    // Has the separation reached tolerance?
                    if (s2 > target - tolerance)
                    {
                        // System.out.println("advancing");
                        // Advance the sweeps
                        t1 = t2;
                        break;
                    }

                    // Compute the initial separation of the witness points.
                    float s1 = fcn.Evaluate(indexes[0], indexes[1], t1);
                    // Check for initial overlap. This might happen if the root finder
                    // runs out of iterations.
                    // System.out.printf("s1: %f, target: %f, tolerance: %f\n", s1, target,
                    // tolerance);
                    if (s1 < target - tolerance)
                    {
                        // System.out.println("failed?");
                        output.State = TOIOutputState.Failed;
                        output.T = t1;
                        done = true;
                        break;
                    }

                    // Check for touching
                    if (s1 <= target + tolerance)
                    {
                        // System.out.println("touching?");
                        // Victory! t1 should hold the TOI (could be 0.0).
                        output.State = TOIOutputState.Touching;
                        output.T = t1;
                        done = true;
                        break;
                    }

                    // Compute 1D root of: f(x) - target = 0
                    int rootIterCount = 0;
                    float a1 = t1, a2 = t2;
                    for (; ; )
                    {
                        // Use a mix of the secant rule and bisection.
                        float t;
                        if ((rootIterCount & 1) == 1)
                        {
                            // Secant rule to improve convergence.
                            t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
                        }
                        else
                        {
                            // Bisection to guarantee progress.
                            t = 0.5f * (a1 + a2);
                        }

                        float s = fcn.Evaluate(indexes[0], indexes[1], t);

                        if (MathUtils.Abs(s - target) < tolerance)
                        {
                            // t2 holds a tentative value for t1
                            t2 = t;
                            break;
                        }

                        // Ensure we continue to bracket the root.
                        if (s > target)
                        {
                            a1 = t;
                            s1 = s;
                        }
                        else
                        {
                            a2 = t;
                            s2 = s;
                        }

                        ++rootIterCount;
                        ++ToiRootIters;

                        // djm: whats with this? put in settings?
                        if (rootIterCount == 50)
                        {
                            break;
                        }
                    }

                    ToiMaxRootIters = MathUtils.Max(ToiMaxRootIters, rootIterCount);

                    ++pushBackIter;

                    if (pushBackIter == Settings.maxPolygonVertices)
                    {
                        break;
                    }
                }

                ++iter;
                ++ToiIters;

                if (done)
                {
                    // System.out.println("done");
                    break;
                }

                if (iter == MAX_ITERATIONS)
                {
                    // System.out.println("failed, root finder stuck");
                    // Root finder got stuck. Semi-victory.
                    output.State = TOIOutputState.Failed;
                    output.T = t1;
                    break;
                }
            }

            // System.out.printf("final sweeps: %f, %f, %f; %f, %f, %f", input.s)
            ToiMaxIters = MathUtils.Max(ToiMaxIters, iter);
        }
    }


    enum Type
    {
        Points,
        FaceA,
        FaceB
    }


    class SeparationFunction
    {

        public Distance.DistanceProxy ProxyA;
        public Distance.DistanceProxy ProxyB;
        public Type Type;
        public readonly Vec2 LocalPoint = new Vec2();
        public readonly Vec2 Axis = new Vec2();
        public Sweep SweepA;
        public Sweep SweepB;

        // djm pooling
        private readonly Vec2 localPointA = new Vec2();
        private readonly Vec2 localPointB = new Vec2();
        private readonly Vec2 pointA = new Vec2();
        private readonly Vec2 pointB = new Vec2();
        private readonly Vec2 localPointA1 = new Vec2();
        private readonly Vec2 localPointA2 = new Vec2();
        private readonly Vec2 normal = new Vec2();
        private readonly Vec2 localPointB1 = new Vec2();
        private readonly Vec2 localPointB2 = new Vec2();
        private readonly Vec2 temp = new Vec2();
        private readonly Transform xfa = new Transform();
        private readonly Transform xfb = new Transform();

        // TODO_ERIN might not need to return the separation

        public virtual float Initialize(Distance.SimplexCache cache, Distance.DistanceProxy proxyA, Sweep sweepA, Distance.DistanceProxy proxyB, Sweep sweepB, float t1)
        {
            ProxyA = proxyA;
            ProxyB = proxyB;
            int count = cache.Count;
            Debug.Assert(0 < count && count < 3);

            SweepA = sweepA;
            SweepB = sweepB;

            SweepA.getTransform(xfa, t1);
            SweepB.getTransform(xfb, t1);

            // log.debug("initializing separation.\n" +
            // "cache: "+cache.count+"-"+cache.metric+"-"+cache.indexA+"-"+cache.indexB+"\n"
            // "distance: "+proxyA.

            if (count == 1)
            {
                Type = Type.Points;
                /*
                * Vec2 localPointA = m_proxyA.GetVertex(cache.indexA[0]); Vec2 localPointB =
                * m_proxyB.GetVertex(cache.indexB[0]); Vec2 pointA = Mul(transformA, localPointA); Vec2
                * pointB = Mul(transformB, localPointB); m_axis = pointB - pointA; m_axis.Normalize();
                */
                localPointA.Set(ProxyA.GetVertex(cache.IndexA[0]));
                localPointB.Set(ProxyB.GetVertex(cache.IndexB[0]));
                Transform.mulToOutUnsafe(xfa, localPointA, pointA);
                Transform.mulToOutUnsafe(xfb, localPointB, pointB);
                Axis.Set(pointB).SubLocal(pointA);
                float s = Axis.Normalize();
                return s;
            }
            else if (cache.IndexA[0] == cache.IndexA[1])
            {
                // Two points on B and one on A.
                Type = Type.FaceB;

                localPointB1.Set(ProxyB.GetVertex(cache.IndexB[0]));
                localPointB2.Set(ProxyB.GetVertex(cache.IndexB[1]));

                temp.Set(localPointB2).SubLocal(localPointB1);
                Vec2.CrossToOutUnsafe(temp, 1f, Axis);
                Axis.Normalize();

                Rot.MulToOutUnsafe(xfb.q, Axis, normal);

                LocalPoint.Set(localPointB1).AddLocal(localPointB2).MulLocal(.5f);
                Transform.mulToOutUnsafe(xfb, LocalPoint, pointB);

                localPointA.Set(proxyA.GetVertex(cache.IndexA[0]));
                Transform.mulToOutUnsafe(xfa, localPointA, pointA);

                temp.Set(pointA).SubLocal(pointB);
                float s = Vec2.Dot(temp, normal);
                if (s < 0.0f)
                {
                    Axis.NegateLocal();
                    s = -s;
                }
                return s;
            }
            else
            {
                // Two points on A and one or two points on B.
                Type = Type.FaceA;

                localPointA1.Set(ProxyA.GetVertex(cache.IndexA[0]));
                localPointA2.Set(ProxyA.GetVertex(cache.IndexA[1]));

                temp.Set(localPointA2).SubLocal(localPointA1);
                Vec2.CrossToOutUnsafe(temp, 1.0f, Axis);
                Axis.Normalize();

                Rot.MulToOutUnsafe(xfa.q, Axis, normal);

                LocalPoint.Set(localPointA1).AddLocal(localPointA2).MulLocal(.5f);
                Transform.mulToOutUnsafe(xfa, LocalPoint, pointA);

                localPointB.Set(ProxyB.GetVertex(cache.IndexB[0]));
                Transform.mulToOutUnsafe(xfb, localPointB, pointB);

                temp.Set(pointB).SubLocal(pointA);
                float s = Vec2.Dot(temp, normal);
                if (s < 0.0f)
                {
                    Axis.NegateLocal();
                    s = -s;
                }
                return s;
            }
        }

        private readonly Vec2 axisA = new Vec2();
        private readonly Vec2 axisB = new Vec2();

        // float FindMinSeparation(int* indexA, int* indexB, float t) const
        public virtual float FindMinSeparation(int[] indexes, float t)
        {

            SweepA.getTransform(xfa, t);
            SweepB.getTransform(xfb, t);

            switch (Type)
            {

                case Type.Points:
                    {
                        Rot.MulTransUnsafe(xfa.q, Axis, axisA);
                        Rot.MulTransUnsafe(xfb.q, Axis.NegateLocal(), axisB);
                        Axis.NegateLocal();

                        indexes[0] = ProxyA.GetSupport(axisA);
                        indexes[1] = ProxyB.GetSupport(axisB);

                        localPointA.Set(ProxyA.GetVertex(indexes[0]));
                        localPointB.Set(ProxyB.GetVertex(indexes[1]));

                        Transform.mulToOutUnsafe(xfa, localPointA, pointA);
                        Transform.mulToOutUnsafe(xfb, localPointB, pointB);

                        float separation = Vec2.Dot(pointB.SubLocal(pointA), Axis);
                        return separation;
                    }

                case Type.FaceA:
                    {
                        Rot.MulToOutUnsafe(xfa.q, Axis, normal);
                        Transform.mulToOutUnsafe(xfa, LocalPoint, pointA);

                        Rot.MulTransUnsafe(xfb.q, normal.NegateLocal(), axisB);
                        normal.NegateLocal();

                        indexes[0] = -1;
                        indexes[1] = ProxyB.GetSupport(axisB);

                        localPointB.Set(ProxyB.GetVertex(indexes[1]));
                        Transform.mulToOutUnsafe(xfb, localPointB, pointB);

                        float separation = Vec2.Dot(pointB.SubLocal(pointA), normal);
                        return separation;
                    }

                case Type.FaceB:
                    {
                        Rot.MulToOutUnsafe(xfb.q, Axis, normal);
                        Transform.mulToOutUnsafe(xfb, LocalPoint, pointB);

                        Rot.MulTransUnsafe(xfa.q, normal.NegateLocal(), axisA);
                        normal.NegateLocal();

                        indexes[1] = -1;
                        indexes[0] = ProxyA.GetSupport(axisA);

                        localPointA.Set(ProxyA.GetVertex(indexes[0]));
                        Transform.mulToOutUnsafe(xfa, localPointA, pointA);

                        float separation = Vec2.Dot(pointA.SubLocal(pointB), normal);
                        return separation;
                    }

                default:
                    Debug.Assert(false);
                    indexes[0] = -1;
                    indexes[1] = -1;
                    return 0f;

            }
        }

        public virtual float Evaluate(int indexA, int indexB, float t)
        {
            SweepA.getTransform(xfa, t);
            SweepB.getTransform(xfb, t);

            switch (Type)
            {

                case Type.Points:
                    {
                        Rot.MulTransUnsafe(xfa.q, Axis, axisA);
                        Rot.MulTransUnsafe(xfb.q, Axis.NegateLocal(), axisB);
                        Axis.NegateLocal();

                        localPointA.Set(ProxyA.GetVertex(indexA));
                        localPointB.Set(ProxyB.GetVertex(indexB));

                        Transform.mulToOutUnsafe(xfa, localPointA, pointA);
                        Transform.mulToOutUnsafe(xfb, localPointB, pointB);

                        float separation = Vec2.Dot(pointB.SubLocal(pointA), Axis);
                        return separation;
                    }

                case Type.FaceA:
                    {
                        // System.out.printf("We're faceA\n");
                        Rot.MulToOutUnsafe(xfa.q, Axis, normal);
                        Transform.mulToOutUnsafe(xfa, LocalPoint, pointA);

                        Rot.MulTransUnsafe(xfb.q, normal.NegateLocal(), axisB);
                        normal.NegateLocal();

                        localPointB.Set(ProxyB.GetVertex(indexB));
                        Transform.mulToOutUnsafe(xfb, localPointB, pointB);
                        float separation = Vec2.Dot(pointB.SubLocal(pointA), normal);
                        return separation;
                    }

                case Type.FaceB:
                    {
                        // System.out.printf("We're faceB\n");
                        Rot.MulToOutUnsafe(xfb.q, Axis, normal);
                        Transform.mulToOutUnsafe(xfb, LocalPoint, pointB);

                        Rot.MulTransUnsafe(xfa.q, normal.NegateLocal(), axisA);
                        normal.NegateLocal();

                        localPointA.Set(ProxyA.GetVertex(indexA));
                        Transform.mulToOutUnsafe(xfa, localPointA, pointA);

                        float separation = Vec2.Dot(pointA.SubLocal(pointB), normal);
                        return separation;
                    }

                default:
                    Debug.Assert(false);
                    return 0f;

            }
        }
    }
}