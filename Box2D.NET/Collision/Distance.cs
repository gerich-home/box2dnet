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

using System;
using System.Diagnostics;
using Box2D.Collision.Shapes;
using Box2D.Common;

namespace Box2D.Collision
{

    // updated to rev 100
    /// <summary>
    /// This is non-static for faster pooling. To get an instance, use the {@link SingletonPool}, don't
    /// construct a distance object.
    /// </summary>
    /// <author>Daniel Murphy</author>
    public class Distance
    {
        public static int GJK_CALLS = 0;
        public static int GJK_ITERS = 0;
        public static int GJK_MAX_ITERS = 20;

        /// <summary> 
        /// GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
        /// </summary>
        //UPGRADE_NOTE: The access modifier for this class or class field has been changed in order to prevent compilation errors due to the visibility level. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1296'"
        private sealed class SimplexVertex
        {
            public readonly Vec2 WA = new Vec2(); // support point in shapeA
            public readonly Vec2 WB = new Vec2(); // support point in shapeB
            public readonly Vec2 W = new Vec2(); // wB - wA
            public float A; // barycentric coordinate for closest point
            public int IndexA; // wA index
            public int IndexB; // wB index

            public void Set(SimplexVertex sv)
            {
                WA.Set(sv.WA);
                WB.Set(sv.WB);
                W.Set(sv.W);
                A = sv.A;
                IndexA = sv.IndexA;
                IndexB = sv.IndexB;
            }
        }

        /// <summary>
        /// Used to warm start Distance. Set count to zero on first call.
        /// </summary>
        /// <author>daniel</author>
        public sealed class SimplexCache
        {
            /// <summary>
            /// length or area
            /// </summary>
            public float Metric;
            public int Count;

            /// <summary>
            /// vertices on shape A
            /// </summary>
            public readonly int[] IndexA = new int[3];

            /// <summary>
            /// vertices on shape B
            /// </summary>
            public readonly int[] IndexB = new int[3];

            public SimplexCache()
            {
                Metric = 0;
                Count = 0;
                IndexA[0] = Int32.MaxValue;
                IndexA[1] = Int32.MaxValue;
                IndexA[2] = Int32.MaxValue;
                IndexB[0] = Int32.MaxValue;
                IndexB[1] = Int32.MaxValue;
                IndexB[2] = Int32.MaxValue;
            }

            public void Set(SimplexCache sc)
            {
                Array.Copy(sc.IndexA, 0, IndexA, 0, IndexA.Length);
                Array.Copy(sc.IndexB, 0, IndexB, 0, IndexB.Length);
                Metric = sc.Metric;
                Count = sc.Count;
            }
        }

        private sealed class Simplex
        {
            private readonly SimplexVertex m_v1 = new SimplexVertex();
            private readonly SimplexVertex m_v2 = new SimplexVertex();
            private readonly SimplexVertex m_v3 = new SimplexVertex();
            public readonly SimplexVertex[] Vertices;
            public int Count;

            public Simplex()
            {
                Vertices = new[] { m_v1, m_v2, m_v3 };
            }

            private float Metric
            {
                // djm pooled, from above

                get
                {
                    switch (Count)
                    {

                        case 0:
                            Debug.Assert(false);
                            return 0.0f;


                        case 1:
                            return 0.0f;


                        case 2:
                            return MathUtils.Distance(m_v1.W, m_v2.W);


                        case 3:
                            case3.Set(m_v2.W).SubLocal(m_v1.W);
                            case33.Set(m_v3.W).SubLocal(m_v1.W);
                            // return Vec2.cross(m_v2.w - m_v1.w, m_v3.w - m_v1.w);
                            return Vec2.Cross(case3, case33);


                        default:
                            Debug.Assert(false);
                            return 0.0f;

                    }
                }

            }

            public void ReadCache(SimplexCache cache, DistanceProxy proxyA, Transform transformA, DistanceProxy proxyB, Transform transformB)
            {
                Debug.Assert(cache.Count <= 3);

                // Copy data from cache.
                Count = cache.Count;

                for (int i = 0; i < Count; ++i)
                {
                    SimplexVertex v = Vertices[i];
                    v.IndexA = cache.IndexA[i];
                    v.IndexB = cache.IndexB[i];
                    Vec2 wALocal = proxyA.GetVertex(v.IndexA);
                    Vec2 wBLocal = proxyB.GetVertex(v.IndexB);
                    Transform.mulToOutUnsafe(transformA, wALocal, v.WA);
                    Transform.mulToOutUnsafe(transformB, wBLocal, v.WB);
                    v.W.Set(v.WB).SubLocal(v.WA);
                    v.A = 0.0f;
                }

                // Compute the new simplex metric, if it is substantially different than
                // old metric then flush the simplex.
                if (Count > 1)
                {
                    float metric1 = cache.Metric;
                    float metric2 = Metric;
                    if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < Settings.EPSILON)
                    {
                        // Reset the simplex.
                        Count = 0;
                    }
                }

                // If the cache is empty or invalid ...
                if (Count == 0)
                {
                    SimplexVertex v = Vertices[0];
                    v.IndexA = 0;
                    v.IndexB = 0;
                    Vec2 wALocal = proxyA.GetVertex(0);
                    Vec2 wBLocal = proxyB.GetVertex(0);
                    Transform.mulToOutUnsafe(transformA, wALocal, v.WA);
                    Transform.mulToOutUnsafe(transformB, wBLocal, v.WB);
                    v.W.Set(v.WB).SubLocal(v.WA);
                    Count = 1;
                }
            }

            public void WriteCache(SimplexCache cache)
            {
                cache.Metric = Metric;
                cache.Count = Count;

                for (int i = 0; i < Count; ++i)
                {
                    cache.IndexA[i] = (Vertices[i].IndexA);
                    cache.IndexB[i] = (Vertices[i].IndexB);
                }
            }

            private readonly Vec2 e12 = new Vec2();

            public void GetSearchDirection(Vec2 result)
            {
                switch (Count)
                {

                    case 1:
                        result.Set(m_v1.W).NegateLocal();
                        return;

                    case 2:
                        e12.Set(m_v2.W).SubLocal(m_v1.W);
                        // use out for a temp variable real quick
                        result.Set(m_v1.W).NegateLocal();
                        float sgn = Vec2.Cross(e12, result);

                        if (sgn > 0f)
                        {
                            // Origin is left of e12.
                            Vec2.CrossToOutUnsafe(1f, e12, result);
                            return;
                        }
                        else
                        {
                            // Origin is right of e12.
                            Vec2.CrossToOutUnsafe(e12, 1f, result);
                            return;
                        }
                    default:
                        Debug.Assert(false);
                        result.SetZero();
                        return;
                }
            }

            // djm pooled
            private readonly Vec2 case2 = new Vec2();
            private readonly Vec2 case22 = new Vec2();

            /// <summary> 
            /// this returns pooled objects. don't keep or modify them
            /// </summary>
            /// <returns></returns>
            public void GetClosestPoint(Vec2 result)
            {
                switch (Count)
                {

                    case 0:
                        Debug.Assert(false);
                        result.SetZero();
                        return;

                    case 1:
                        result.Set(m_v1.W);
                        return;

                    case 2:
                        case22.Set(m_v2.W).MulLocal(m_v2.A);
                        case2.Set(m_v1.W).MulLocal(m_v1.A).AddLocal(case22);
                        result.Set(case2);
                        return;

                    case 3:
                        result.SetZero();
                        return;

                    default:
                        Debug.Assert(false);
                        result.SetZero();
                        return;

                }
            }

            // djm pooled, and from above
            private readonly Vec2 case3 = new Vec2();
            private readonly Vec2 case33 = new Vec2();

            public void GetWitnessPoints(Vec2 pA, Vec2 pB)
            {
                switch (Count)
                {

                    case 0:
                        Debug.Assert(false);
                        break;


                    case 1:
                        pA.Set(m_v1.WA);
                        pB.Set(m_v1.WB);
                        break;


                    case 2:
                        case2.Set(m_v1.WA).MulLocal(m_v1.A);
                        pA.Set(m_v2.WA).MulLocal(m_v2.A).AddLocal(case2);
                        // m_v1.a * m_v1.wA + m_v2.a * m_v2.wA;
                        // *pB = m_v1.a * m_v1.wB + m_v2.a * m_v2.wB;
                        case2.Set(m_v1.WB).MulLocal(m_v1.A);
                        pB.Set(m_v2.WB).MulLocal(m_v2.A).AddLocal(case2);

                        break;


                    case 3:
                        pA.Set(m_v1.WA).MulLocal(m_v1.A);
                        case3.Set(m_v2.WA).MulLocal(m_v2.A);
                        case33.Set(m_v3.WA).MulLocal(m_v3.A);
                        pA.AddLocal(case3).AddLocal(case33);
                        pB.Set(pA);
                        // *pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA + m_v3.a * m_v3.wA;
                        // *pB = *pA;
                        break;


                    default:
                        Debug.Assert(false);
                        break;

                }
            }

            // djm pooled from above
            /// <summary>
            ///  Solve a line segment using barycentric coordinates.
            /// </summary>
            public void Solve2()
            {
                // Solve a line segment using barycentric coordinates.
                //
                // p = a1 * w1 + a2 * w2
                // a1 + a2 = 1
                //
                // The vector from the origin to the closest point on the line is
                // perpendicular to the line.
                // e12 = w2 - w1
                // dot(p, e) = 0
                // a1 * dot(w1, e) + a2 * dot(w2, e) = 0
                //
                // 2-by-2 linear system
                // [1 1 ][a1] = [1]
                // [w1.e12 w2.e12][a2] = [0]
                //
                // Define
                // d12_1 = dot(w2, e12)
                // d12_2 = -dot(w1, e12)
                // d12 = d12_1 + d12_2
                //
                // Solution
                // a1 = d12_1 / d12
                // a2 = d12_2 / d12
                Vec2 w1 = m_v1.W;
                Vec2 w2 = m_v2.W;
                e12.Set(w2).SubLocal(w1);

                // w1 region
                float d12_2 = -Vec2.Dot(w1, e12);
                if (d12_2 <= 0.0f)
                {
                    // a2 <= 0, so we clamp it to 0
                    m_v1.A = 1.0f;
                    Count = 1;
                    return;
                }

                // w2 region
                float d12_1 = Vec2.Dot(w2, e12);
                if (d12_1 <= 0.0f)
                {
                    // a1 <= 0, so we clamp it to 0
                    m_v2.A = 1.0f;
                    Count = 1;
                    m_v1.Set(m_v2);
                    return;
                }

                // Must be in e12 region.
                float inv_d12 = 1.0f / (d12_1 + d12_2);
                m_v1.A = d12_1 * inv_d12;
                m_v2.A = d12_2 * inv_d12;
                Count = 2;
            }

            // djm pooled, and from above
            private readonly Vec2 e13 = new Vec2();
            private readonly Vec2 e23 = new Vec2();
            private readonly Vec2 w1 = new Vec2();
            private readonly Vec2 w2 = new Vec2();
            private readonly Vec2 w3 = new Vec2();

            /// <summary>
            ///  Solve a line segment using barycentric coordinates.<br/>
            ///  Possible regions:<br/>
            ///  - points[2]<br/>
            ///  - edge points[0]-points[2]<br/>
            ///  - edge points[1]-points[2]<br/>
            ///  - inside the triangle
            /// </summary>
            public void Solve3()
            {
                w1.Set(m_v1.W);
                w2.Set(m_v2.W);
                w3.Set(m_v3.W);

                // Edge12
                // [1 1 ][a1] = [1]
                // [w1.e12 w2.e12][a2] = [0]
                // a3 = 0
                e12.Set(w2).SubLocal(w1);
                float w1e12 = Vec2.Dot(w1, e12);
                float w2e12 = Vec2.Dot(w2, e12);
                float d12_1 = w2e12;
                float d12_2 = -w1e12;

                // Edge13
                // [1 1 ][a1] = [1]
                // [w1.e13 w3.e13][a3] = [0]
                // a2 = 0
                e13.Set(w3).SubLocal(w1);
                float w1e13 = Vec2.Dot(w1, e13);
                float w3e13 = Vec2.Dot(w3, e13);
                float d13_1 = w3e13;
                float d13_2 = -w1e13;

                // Edge23
                // [1 1 ][a2] = [1]
                // [w2.e23 w3.e23][a3] = [0]
                // a1 = 0
                e23.Set(w3).SubLocal(w2);
                float w2e23 = Vec2.Dot(w2, e23);
                float w3e23 = Vec2.Dot(w3, e23);
                float d23_1 = w3e23;
                float d23_2 = -w2e23;

                // Triangle123
                float n123 = Vec2.Cross(e12, e13);

                float d123_1 = n123 * Vec2.Cross(w2, w3);
                float d123_2 = n123 * Vec2.Cross(w3, w1);
                float d123_3 = n123 * Vec2.Cross(w1, w2);

                // w1 region
                if (d12_2 <= 0.0f && d13_2 <= 0.0f)
                {
                    m_v1.A = 1.0f;
                    Count = 1;
                    return;
                }

                // e12
                if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
                {
                    float inv_d12 = 1.0f / (d12_1 + d12_2);
                    m_v1.A = d12_1 * inv_d12;
                    m_v2.A = d12_2 * inv_d12;
                    Count = 2;
                    return;
                }

                // e13
                if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
                {
                    float inv_d13 = 1.0f / (d13_1 + d13_2);
                    m_v1.A = d13_1 * inv_d13;
                    m_v3.A = d13_2 * inv_d13;
                    Count = 2;
                    m_v2.Set(m_v3);
                    return;
                }

                // w2 region
                if (d12_1 <= 0.0f && d23_2 <= 0.0f)
                {
                    m_v2.A = 1.0f;
                    Count = 1;
                    m_v1.Set(m_v2);
                    return;
                }

                // w3 region
                if (d13_1 <= 0.0f && d23_1 <= 0.0f)
                {
                    m_v3.A = 1.0f;
                    Count = 1;
                    m_v1.Set(m_v3);
                    return;
                }

                // e23
                if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
                {
                    float inv_d23 = 1.0f / (d23_1 + d23_2);
                    m_v2.A = d23_1 * inv_d23;
                    m_v3.A = d23_2 * inv_d23;
                    Count = 2;
                    m_v1.Set(m_v3);
                    return;
                }

                // Must be in triangle123
                float inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
                m_v1.A = d123_1 * inv_d123;
                m_v2.A = d123_2 * inv_d123;
                m_v3.A = d123_3 * inv_d123;
                Count = 3;
            }
        }

        /// <summary> 
        /// A distance proxy is used by the GJK algorithm. It encapsulates any shape. TODO: see if we can
        /// just do assignments with m_vertices, instead of copying stuff over
        /// </summary>
        /// <author>daniel</author>
        public sealed class DistanceProxy
        {
            /// <summary>
            ///  Get the vertex count.
            /// </summary>
            /// <returns></returns>
            public int VertexCount { get; set; }

            public readonly Vec2[] Vertices;
            public float Radius;
            public readonly Vec2[] Buffer;

            public DistanceProxy()
            {
                Vertices = new Vec2[Settings.MAX_POLYGON_VERTICES];
                for (int i = 0; i < Vertices.Length; i++)
                {
                    Vertices[i] = new Vec2();
                }
                Buffer = new Vec2[2];
                VertexCount = 0;
                Radius = 0f;
            }

            /// <summary>
            ///  Initialize the proxy using the given shape. The shape must remain in scope while the proxy is in use.
            /// </summary>
            public void Set(Shape shape, int index)
            {
                switch (shape.Type)
                {

                    case ShapeType.Circle:
                        CircleShape circle = (CircleShape)shape;
                        Vertices[0].Set(circle.P);
                        VertexCount = 1;
                        Radius = circle.Radius;

                        break;

                    case ShapeType.Polygon:
                        PolygonShape poly = (PolygonShape)shape;
                        VertexCount = poly.VertexCount;
                        Radius = poly.Radius;
                        for (int i = 0; i < VertexCount; i++)
                        {
                            Vertices[i].Set(poly.Vertices[i]);
                        }
                        break;

                    case ShapeType.Chain:
                        ChainShape chain = (ChainShape)shape;
                        Debug.Assert(0 <= index && index < chain.Count);

                        Buffer[0] = chain.Vertices[index];
                        if (index + 1 < chain.Count)
                        {
                            Buffer[1] = chain.Vertices[index + 1];
                        }
                        else
                        {
                            Buffer[1] = chain.Vertices[0];
                        }

                        Vertices[0].Set(Buffer[0]);
                        Vertices[1].Set(Buffer[1]);
                        VertexCount = 2;
                        Radius = chain.Radius;
                        break;

                    case ShapeType.Edge:
                        EdgeShape edge = (EdgeShape)shape;
                        Vertices[0].Set(edge.Vertex1);
                        Vertices[1].Set(edge.Vertex2);
                        VertexCount = 2;
                        Radius = edge.Radius;
                        break;

                    default:
                        Debug.Assert(false);
                        break;

                }
            }

            /// <summary>
            /// Get the supporting vertex index in the given direction.
            /// </summary>
            /// <param name="d"></param>
            /// <returns></returns>
            public int GetSupport(Vec2 d)
            {
                int bestIndex = 0;
                float bestValue = Vec2.Dot(Vertices[0], d);
                for (int i = 1; i < VertexCount; i++)
                {
                    float value = Vec2.Dot(Vertices[i], d);
                    if (value > bestValue)
                    {
                        bestIndex = i;
                        bestValue = value;
                    }
                }

                return bestIndex;
            }

            /// <summary>
            /// Get the supporting vertex in the given direction.
            /// </summary>
            /// <param name="d"></param>
            /// <returns></returns>
            public Vec2 GetSupportVertex(Vec2 d)
            {
                int bestIndex = 0;
                float bestValue = Vec2.Dot(Vertices[0], d);
                for (int i = 1; i < VertexCount; i++)
                {
                    float value = Vec2.Dot(Vertices[i], d);
                    if (value > bestValue)
                    {
                        bestIndex = i;
                        bestValue = value;
                    }
                }

                return Vertices[bestIndex];
            }

            /// <summary>
            /// Get a vertex by index. Used by Distance.
            /// </summary>
            /// <param name="index"></param>
            /// <returns></returns>
            public Vec2 GetVertex(int index)
            {
                Debug.Assert(0 <= index && index < VertexCount);
                return Vertices[index];
            }
        }

        private readonly Simplex simplex = new Simplex();
        private readonly int[] saveA = new int[3];
        private readonly int[] saveB = new int[3];
        private readonly Vec2 closestPoint = new Vec2();
        private readonly Vec2 d = new Vec2();
        private readonly Vec2 temp = new Vec2();
        private readonly Vec2 normal = new Vec2();

        /// <summary>
        /// Compute the closest points between two shapes. Supports any combination of: CircleShape and
        /// PolygonShape. The simplex cache is input/output. On the first call set SimplexCache.count to
        /// zero.
        /// </summary>
        /// <param name="output"></param>
        /// <param name="cache"></param>
        /// <param name="input"></param>
        public void GetDistance(DistanceOutput output, SimplexCache cache, DistanceInput input)
        {
            GJK_CALLS++;

            DistanceProxy proxyA = input.ProxyA;
            DistanceProxy proxyB = input.ProxyB;

            Transform transformA = input.TransformA;
            Transform transformB = input.TransformB;

            // Initialize the simplex.
            simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);

            // Get simplex vertices as an array.
            SimplexVertex[] vertices = simplex.Vertices;

            // These store the vertices of the last simplex so that we
            // can check for duplicates and prevent cycling.
            // (pooled above)
            int saveCount;

            simplex.GetClosestPoint(closestPoint);
            float distanceSqr1 = closestPoint.LengthSquared();
            float distanceSqr2;

            // Main iteration loop
            int iter = 0;
            while (iter < GJK_MAX_ITERS)
            {

                // Copy simplex so we can identify duplicates.
                saveCount = simplex.Count;
                for (int i = 0; i < saveCount; i++)
                {
                    saveA[i] = vertices[i].IndexA;
                    saveB[i] = vertices[i].IndexB;
                }

                switch (simplex.Count)
                {

                    case 1:
                        break;

                    case 2:
                        simplex.Solve2();
                        break;

                    case 3:
                        simplex.Solve3();
                        break;

                    default:
                        Debug.Assert(false);
                        break;

                }

                // If we have 3 points, then the origin is in the corresponding triangle.
                if (simplex.Count == 3)
                {
                    break;
                }

                // Compute closest point.
                simplex.GetClosestPoint(closestPoint);
                distanceSqr2 = closestPoint.LengthSquared();

                // ensure progress
                if (distanceSqr2 >= distanceSqr1)
                {
                    // break;
                }
                distanceSqr1 = distanceSqr2;

                // get search direction;
                simplex.GetSearchDirection(d);

                // Ensure the search direction is numerically fit.
                if (d.LengthSquared() < Settings.EPSILON * Settings.EPSILON)
                {
                    // The origin is probably contained by a line segment
                    // or triangle. Thus the shapes are overlapped.

                    // We can't return zero here even though there may be overlap.
                    // In case the simplex is a point, segment, or triangle it is difficult
                    // to determine if the origin is contained in the CSO or very close to it.
                    break;
                }
                /*
                * SimplexVertex* vertex = vertices + simplex.m_count; vertex.indexA =
                * proxyA.GetSupport(MulT(transformA.R, -d)); vertex.wA = Mul(transformA,
                * proxyA.GetVertex(vertex.indexA)); Vec2 wBLocal; vertex.indexB =
                * proxyB.GetSupport(MulT(transformB.R, d)); vertex.wB = Mul(transformB,
                * proxyB.GetVertex(vertex.indexB)); vertex.w = vertex.wB - vertex.wA;
                */

                // Compute a tentative new simplex vertex using support points.
                SimplexVertex vertex = vertices[simplex.Count];

                Rot.MulTransUnsafe(transformA.q, d.NegateLocal(), temp);
                vertex.IndexA = proxyA.GetSupport(temp);
                Transform.mulToOutUnsafe(transformA, proxyA.GetVertex(vertex.IndexA), vertex.WA);
                // Vec2 wBLocal;
                Rot.MulTransUnsafe(transformB.q, d.NegateLocal(), temp);
                vertex.IndexB = proxyB.GetSupport(temp);
                Transform.mulToOutUnsafe(transformB, proxyB.GetVertex(vertex.IndexB), vertex.WB);
                vertex.W.Set(vertex.WB).SubLocal(vertex.WA);

                // Iteration count is equated to the number of support point calls.
                ++iter;
                ++GJK_ITERS;

                // Check for duplicate support points. This is the main termination criteria.
                bool duplicate = false;
                for (int i = 0; i < saveCount; ++i)
                {
                    if (vertex.IndexA == saveA[i] && vertex.IndexB == saveB[i])
                    {
                        duplicate = true;
                        break;
                    }
                }

                // If we found a duplicate support point we must exit to avoid cycling.
                if (duplicate)
                {
                    break;
                }

                // New vertex is ok and needed.
                ++simplex.Count;
            }

            GJK_MAX_ITERS = MathUtils.Max(GJK_MAX_ITERS, iter);

            // Prepare output.
            simplex.GetWitnessPoints(output.PointA, output.PointB);
            output.Distance = MathUtils.Distance(output.PointA, output.PointB);
            output.Iterations = iter;

            // Cache the simplex.
            simplex.WriteCache(cache);

            // Apply radii if requested.
            if (input.UseRadii)
            {
                float rA = proxyA.Radius;
                float rB = proxyB.Radius;

                if (output.Distance > rA + rB && output.Distance > Settings.EPSILON)
                {
                    // Shapes are still no overlapped.
                    // Move the witness points to the outer surface.
                    output.Distance -= (rA + rB);
                    normal.Set(output.PointB).SubLocal(output.PointA);
                    normal.Normalize();
                    temp.Set(normal).MulLocal(rA);
                    output.PointA.AddLocal(temp);
                    temp.Set(normal).MulLocal(rB);
                    output.PointB.SubLocal(temp);
                }
                else
                {
                    // Shapes are overlapped when radii are considered.
                    // Move the witness points to the middle.
                    // Vec2 p = 0.5f * (output.pointA + output.pointB);
                    output.PointA.AddLocal(output.PointB).MulLocal(.5f);
                    output.PointB.Set(output.PointA);
                    output.Distance = 0.0f;
                }
            }
        }
    }
}