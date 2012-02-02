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

        /// <summary> GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.</summary>
        //UPGRADE_NOTE: The access modifier for this class or class field has been changed in order to prevent compilation errors due to the visibility level. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1296'"
        public class SimplexVertex
        {
            public readonly Vec2 wA = new Vec2(); // support point in shapeA
            public readonly Vec2 wB = new Vec2(); // support point in shapeB
            public readonly Vec2 w = new Vec2(); // wB - wA
            public float a; // barycentric coordinate for closest point
            public int indexA; // wA index
            public int indexB; // wB index

            public virtual void set_Renamed(SimplexVertex sv)
            {
                wA.set_Renamed(sv.wA);
                wB.set_Renamed(sv.wB);
                w.set_Renamed(sv.w);
                a = sv.a;
                indexA = sv.indexA;
                indexB = sv.indexB;
            }
        }

        /// <summary>
        /// Used to warm start Distance. Set count to zero on first call.
        /// </summary>
        /// <author>daniel</author>
        public class SimplexCache
        {
            /// <summary>length or area</summary>
            public float metric;
            public int count;

            /// <summary>vertices on shape A</summary>
            public readonly int[] indexA = new int[3];

            /// <summary>vertices on shape B</summary>
            public readonly int[] indexB = new int[3];

            public SimplexCache()
            {
                metric = 0;
                count = 0;
                indexA[0] = Int32.MaxValue;
                indexA[1] = Int32.MaxValue;
                indexA[2] = Int32.MaxValue;
                indexB[0] = Int32.MaxValue;
                indexB[1] = Int32.MaxValue;
                indexB[2] = Int32.MaxValue;
            }

            public virtual void set_Renamed(SimplexCache sc)
            {
                Array.Copy(sc.indexA, 0, indexA, 0, indexA.Length);
                Array.Copy(sc.indexB, 0, indexB, 0, indexB.Length);
                metric = sc.metric;
                count = sc.count;
            }
        }

        private class Simplex
        {
            public readonly SimplexVertex m_v1 = new SimplexVertex();
            public readonly SimplexVertex m_v2 = new SimplexVertex();
            public readonly SimplexVertex m_v3 = new SimplexVertex();
            public SimplexVertex[] vertices;
            public int m_count;

            public Simplex()
            {
                vertices = new SimplexVertex[] { m_v1, m_v2, m_v3 };
            }

            virtual public float Metric
            {
                // djm pooled, from above

                get
                {
                    switch (m_count)
                    {

                        case 0:
                            Debug.Assert(false);
                            return 0.0f;


                        case 1:
                            return 0.0f;


                        case 2:
                            return MathUtils.distance(m_v1.w, m_v2.w);


                        case 3:
                            case3.set_Renamed(m_v2.w).subLocal(m_v1.w);
                            case33.set_Renamed(m_v3.w).subLocal(m_v1.w);
                            // return Vec2.cross(m_v2.w - m_v1.w, m_v3.w - m_v1.w);
                            return Vec2.cross(case3, case33);


                        default:
                            Debug.Assert(false);
                            return 0.0f;

                    }
                }

            }

            public virtual void readCache(SimplexCache cache, DistanceProxy proxyA, Transform transformA, DistanceProxy proxyB, Transform transformB)
            {
                Debug.Assert(cache.count <= 3);

                // Copy data from cache.
                m_count = cache.count;

                for (int i = 0; i < m_count; ++i)
                {
                    SimplexVertex v = vertices[i];
                    v.indexA = cache.indexA[i];
                    v.indexB = cache.indexB[i];
                    Vec2 wALocal = proxyA.getVertex(v.indexA);
                    Vec2 wBLocal = proxyB.getVertex(v.indexB);
                    Transform.mulToOutUnsafe(transformA, wALocal, v.wA);
                    Transform.mulToOutUnsafe(transformB, wBLocal, v.wB);
                    v.w.set_Renamed(v.wB).subLocal(v.wA);
                    v.a = 0.0f;
                }

                // Compute the new simplex metric, if it is substantially different than
                // old metric then flush the simplex.
                if (m_count > 1)
                {
                    float metric1 = cache.metric;
                    float metric2 = Metric;
                    if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < Settings.EPSILON)
                    {
                        // Reset the simplex.
                        m_count = 0;
                    }
                }

                // If the cache is empty or invalid ...
                if (m_count == 0)
                {
                    SimplexVertex v = vertices[0];
                    v.indexA = 0;
                    v.indexB = 0;
                    Vec2 wALocal = proxyA.getVertex(0);
                    Vec2 wBLocal = proxyB.getVertex(0);
                    Transform.mulToOutUnsafe(transformA, wALocal, v.wA);
                    Transform.mulToOutUnsafe(transformB, wBLocal, v.wB);
                    v.w.set_Renamed(v.wB).subLocal(v.wA);
                    m_count = 1;
                }
            }

            public virtual void writeCache(SimplexCache cache)
            {
                cache.metric = Metric;
                cache.count = m_count;

                for (int i = 0; i < m_count; ++i)
                {
                    cache.indexA[i] = (vertices[i].indexA);
                    cache.indexB[i] = (vertices[i].indexB);
                }
            }

            private readonly Vec2 e12 = new Vec2();

            public void getSearchDirection(Vec2 out_Renamed)
            {
                switch (m_count)
                {

                    case 1:
                        out_Renamed.set_Renamed(m_v1.w).negateLocal();
                        return;

                    case 2:
                        e12.set_Renamed(m_v2.w).subLocal(m_v1.w);
                        // use out for a temp variable real quick
                        out_Renamed.set_Renamed(m_v1.w).negateLocal();
                        float sgn = Vec2.cross(e12, out_Renamed);

                        if (sgn > 0f)
                        {
                            // Origin is left of e12.
                            Vec2.crossToOutUnsafe(1f, e12, out_Renamed);
                            return;
                        }
                        else
                        {
                            // Origin is right of e12.
                            Vec2.crossToOutUnsafe(e12, 1f, out_Renamed);
                            return;
                        }
                        goto default;

                    default:
                        Debug.Assert(false);
                        out_Renamed.setZero();
                        return;

                }
            }

            // djm pooled
            private readonly Vec2 case2 = new Vec2();
            private readonly Vec2 case22 = new Vec2();

            /// <summary> this returns pooled objects. don't keep or modify them
            /// 
            /// </summary>
            /// <returns>
            /// </returns>
            public virtual void getClosestPoint(Vec2 out_Renamed)
            {
                switch (m_count)
                {

                    case 0:
                        Debug.Assert(false);
                        out_Renamed.setZero();
                        return;

                    case 1:
                        out_Renamed.set_Renamed(m_v1.w);
                        return;

                    case 2:
                        case22.set_Renamed(m_v2.w).mulLocal(m_v2.a);
                        case2.set_Renamed(m_v1.w).mulLocal(m_v1.a).addLocal(case22);
                        out_Renamed.set_Renamed(case2);
                        return;

                    case 3:
                        out_Renamed.setZero();
                        return;

                    default:
                        Debug.Assert(false);
                        out_Renamed.setZero();
                        return;

                }
            }

            // djm pooled, and from above
            private readonly Vec2 case3 = new Vec2();
            private readonly Vec2 case33 = new Vec2();

            public virtual void getWitnessPoints(Vec2 pA, Vec2 pB)
            {
                switch (m_count)
                {

                    case 0:
                        Debug.Assert(false);
                        break;


                    case 1:
                        pA.set_Renamed(m_v1.wA);
                        pB.set_Renamed(m_v1.wB);
                        break;


                    case 2:
                        case2.set_Renamed(m_v1.wA).mulLocal(m_v1.a);
                        pA.set_Renamed(m_v2.wA).mulLocal(m_v2.a).addLocal(case2);
                        // m_v1.a * m_v1.wA + m_v2.a * m_v2.wA;
                        // *pB = m_v1.a * m_v1.wB + m_v2.a * m_v2.wB;
                        case2.set_Renamed(m_v1.wB).mulLocal(m_v1.a);
                        pB.set_Renamed(m_v2.wB).mulLocal(m_v2.a).addLocal(case2);

                        break;


                    case 3:
                        pA.set_Renamed(m_v1.wA).mulLocal(m_v1.a);
                        case3.set_Renamed(m_v2.wA).mulLocal(m_v2.a);
                        case33.set_Renamed(m_v3.wA).mulLocal(m_v3.a);
                        pA.addLocal(case3).addLocal(case33);
                        pB.set_Renamed(pA);
                        // *pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA + m_v3.a * m_v3.wA;
                        // *pB = *pA;
                        break;


                    default:
                        Debug.Assert(false);
                        break;

                }
            }

            // djm pooled from above
            /// <summary> Solve a line segment using barycentric coordinates.</summary>
            public virtual void solve2()
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
                Vec2 w1 = m_v1.w;
                Vec2 w2 = m_v2.w;
                e12.set_Renamed(w2).subLocal(w1);

                // w1 region
                float d12_2 = -Vec2.dot(w1, e12);
                if (d12_2 <= 0.0f)
                {
                    // a2 <= 0, so we clamp it to 0
                    m_v1.a = 1.0f;
                    m_count = 1;
                    return;
                }

                // w2 region
                float d12_1 = Vec2.dot(w2, e12);
                if (d12_1 <= 0.0f)
                {
                    // a1 <= 0, so we clamp it to 0
                    m_v2.a = 1.0f;
                    m_count = 1;
                    m_v1.set_Renamed(m_v2);
                    return;
                }

                // Must be in e12 region.
                float inv_d12 = 1.0f / (d12_1 + d12_2);
                m_v1.a = d12_1 * inv_d12;
                m_v2.a = d12_2 * inv_d12;
                m_count = 2;
            }

            // djm pooled, and from above
            private readonly Vec2 e13 = new Vec2();
            private readonly Vec2 e23 = new Vec2();
            private readonly Vec2 w1 = new Vec2();
            private readonly Vec2 w2 = new Vec2();
            private readonly Vec2 w3 = new Vec2();

            /// <summary> Solve a line segment using barycentric coordinates.<br/>
            /// Possible regions:<br/>
            /// - points[2]<br/>
            /// - edge points[0]-points[2]<br/>
            /// - edge points[1]-points[2]<br/>
            /// - inside the triangle
            /// </summary>
            public virtual void solve3()
            {
                w1.set_Renamed(m_v1.w);
                w2.set_Renamed(m_v2.w);
                w3.set_Renamed(m_v3.w);

                // Edge12
                // [1 1 ][a1] = [1]
                // [w1.e12 w2.e12][a2] = [0]
                // a3 = 0
                e12.set_Renamed(w2).subLocal(w1);
                float w1e12 = Vec2.dot(w1, e12);
                float w2e12 = Vec2.dot(w2, e12);
                float d12_1 = w2e12;
                float d12_2 = -w1e12;

                // Edge13
                // [1 1 ][a1] = [1]
                // [w1.e13 w3.e13][a3] = [0]
                // a2 = 0
                e13.set_Renamed(w3).subLocal(w1);
                float w1e13 = Vec2.dot(w1, e13);
                float w3e13 = Vec2.dot(w3, e13);
                float d13_1 = w3e13;
                float d13_2 = -w1e13;

                // Edge23
                // [1 1 ][a2] = [1]
                // [w2.e23 w3.e23][a3] = [0]
                // a1 = 0
                e23.set_Renamed(w3).subLocal(w2);
                float w2e23 = Vec2.dot(w2, e23);
                float w3e23 = Vec2.dot(w3, e23);
                float d23_1 = w3e23;
                float d23_2 = -w2e23;

                // Triangle123
                float n123 = Vec2.cross(e12, e13);

                float d123_1 = n123 * Vec2.cross(w2, w3);
                float d123_2 = n123 * Vec2.cross(w3, w1);
                float d123_3 = n123 * Vec2.cross(w1, w2);

                // w1 region
                if (d12_2 <= 0.0f && d13_2 <= 0.0f)
                {
                    m_v1.a = 1.0f;
                    m_count = 1;
                    return;
                }

                // e12
                if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
                {
                    float inv_d12 = 1.0f / (d12_1 + d12_2);
                    m_v1.a = d12_1 * inv_d12;
                    m_v2.a = d12_2 * inv_d12;
                    m_count = 2;
                    return;
                }

                // e13
                if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
                {
                    float inv_d13 = 1.0f / (d13_1 + d13_2);
                    m_v1.a = d13_1 * inv_d13;
                    m_v3.a = d13_2 * inv_d13;
                    m_count = 2;
                    m_v2.set_Renamed(m_v3);
                    return;
                }

                // w2 region
                if (d12_1 <= 0.0f && d23_2 <= 0.0f)
                {
                    m_v2.a = 1.0f;
                    m_count = 1;
                    m_v1.set_Renamed(m_v2);
                    return;
                }

                // w3 region
                if (d13_1 <= 0.0f && d23_1 <= 0.0f)
                {
                    m_v3.a = 1.0f;
                    m_count = 1;
                    m_v1.set_Renamed(m_v3);
                    return;
                }

                // e23
                if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
                {
                    float inv_d23 = 1.0f / (d23_1 + d23_2);
                    m_v2.a = d23_1 * inv_d23;
                    m_v3.a = d23_2 * inv_d23;
                    m_count = 2;
                    m_v1.set_Renamed(m_v3);
                    return;
                }

                // Must be in triangle123
                float inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
                m_v1.a = d123_1 * inv_d123;
                m_v2.a = d123_2 * inv_d123;
                m_v3.a = d123_3 * inv_d123;
                m_count = 3;
            }
        }

        /// <summary> A distance proxy is used by the GJK algorithm. It encapsulates any shape. TODO: see if we can
        /// just do assignments with m_vertices, instead of copying stuff over
        /// 
        /// </summary>
        /// <author>  daniel
        /// </author>
        public class DistanceProxy
        {
            /// <summary> Get the vertex count.
            /// 
            /// </summary>
            /// <returns>
            /// </returns>
            virtual public int VertexCount
            {
                get
                {
                    return m_count;
                }

            }
            public readonly Vec2[] m_vertices;
            public int m_count;
            public float m_radius;
            public readonly Vec2[] m_buffer;

            public DistanceProxy()
            {
                m_vertices = new Vec2[Settings.maxPolygonVertices];
                for (int i = 0; i < m_vertices.Length; i++)
                {
                    m_vertices[i] = new Vec2();
                }
                m_buffer = new Vec2[2];
                m_count = 0;
                m_radius = 0f;
            }

            /// <summary> Initialize the proxy using the given shape. The shape must remain in scope while the proxy is
            /// in use.
            /// </summary>
            public void set_Renamed(Shape shape, int index)
            {
                switch (shape.Type)
                {

                    case ShapeType.Circle:
                        CircleShape circle = (CircleShape)shape;
                        m_vertices[0].set_Renamed(circle.m_p);
                        m_count = 1;
                        m_radius = circle.Radius;

                        break;

                    case ShapeType.Polygon:
                        PolygonShape poly = (PolygonShape)shape;
                        m_count = poly.m_count;
                        m_radius = poly.Radius;
                        for (int i = 0; i < m_count; i++)
                        {
                            m_vertices[i].set_Renamed(poly.m_vertices[i]);
                        }
                        break;

                    case ShapeType.Chain:
                        ChainShape chain = (ChainShape)shape;
                        Debug.Assert(0 <= index && index < chain.Count);

                        m_buffer[0] = chain.Vertices[index];
                        if (index + 1 < chain.Count)
                        {
                            m_buffer[1] = chain.Vertices[index + 1];
                        }
                        else
                        {
                            m_buffer[1] = chain.Vertices[0];
                        }

                        m_vertices[0].set_Renamed(m_buffer[0]);
                        m_vertices[1].set_Renamed(m_buffer[1]);
                        m_count = 2;
                        m_radius = chain.Radius;
                        break;

                    case ShapeType.Edge:
                        EdgeShape edge = (EdgeShape)shape;
                        m_vertices[0].set_Renamed(edge.m_vertex1);
                        m_vertices[1].set_Renamed(edge.m_vertex2);
                        m_count = 2;
                        m_radius = edge.Radius;
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
            public int getSupport(Vec2 d)
            {
                int bestIndex = 0;
                float bestValue = Vec2.dot(m_vertices[0], d);
                for (int i = 1; i < m_count; i++)
                {
                    float value_Renamed = Vec2.dot(m_vertices[i], d);
                    if (value_Renamed > bestValue)
                    {
                        bestIndex = i;
                        bestValue = value_Renamed;
                    }
                }

                return bestIndex;
            }

            /// <summary>
            /// Get the supporting vertex in the given direction.
            /// </summary>
            /// <param name="d"></param>
            /// <returns></returns>
            public Vec2 getSupportVertex(Vec2 d)
            {
                int bestIndex = 0;
                float bestValue = Vec2.dot(m_vertices[0], d);
                for (int i = 1; i < m_count; i++)
                {
                    float value_Renamed = Vec2.dot(m_vertices[i], d);
                    if (value_Renamed > bestValue)
                    {
                        bestIndex = i;
                        bestValue = value_Renamed;
                    }
                }

                return m_vertices[bestIndex];
            }

            /// <summary>
            /// Get a vertex by index. Used by Distance.
            /// </summary>
            /// <param name="index"></param>
            /// <returns></returns>
            public Vec2 getVertex(int index)
            {
                Debug.Assert(0 <= index && index < m_count);
                return m_vertices[index];
            }
        }

        private Simplex simplex = new Simplex();
        private int[] saveA = new int[3];
        private int[] saveB = new int[3];
        private Vec2 closestPoint = new Vec2();
        private Vec2 d = new Vec2();
        private Vec2 temp = new Vec2();
        private Vec2 normal = new Vec2();

        /// <summary>
        /// Compute the closest points between two shapes. Supports any combination of: CircleShape and
        /// PolygonShape. The simplex cache is input/output. On the first call set SimplexCache.count to
        /// zero.
        /// </summary>
        /// <param name="output"></param>
        /// <param name="cache"></param>
        /// <param name="input"></param>
        public void distance(DistanceOutput output, SimplexCache cache, DistanceInput input)
        {
            GJK_CALLS++;

            DistanceProxy proxyA = input.proxyA;
            DistanceProxy proxyB = input.proxyB;

            Transform transformA = input.transformA;
            Transform transformB = input.transformB;

            // Initialize the simplex.
            simplex.readCache(cache, proxyA, transformA, proxyB, transformB);

            // Get simplex vertices as an array.
            SimplexVertex[] vertices = simplex.vertices;

            // These store the vertices of the last simplex so that we
            // can check for duplicates and prevent cycling.
            // (pooled above)
            int saveCount = 0;

            simplex.getClosestPoint(closestPoint);
            float distanceSqr1 = closestPoint.lengthSquared();
            float distanceSqr2 = distanceSqr1;

            // Main iteration loop
            int iter = 0;
            while (iter < GJK_MAX_ITERS)
            {

                // Copy simplex so we can identify duplicates.
                saveCount = simplex.m_count;
                for (int i = 0; i < saveCount; i++)
                {
                    saveA[i] = vertices[i].indexA;
                    saveB[i] = vertices[i].indexB;
                }

                switch (simplex.m_count)
                {

                    case 1:
                        break;

                    case 2:
                        simplex.solve2();
                        break;

                    case 3:
                        simplex.solve3();
                        break;

                    default:
                        Debug.Assert(false);
                        break;

                }

                // If we have 3 points, then the origin is in the corresponding triangle.
                if (simplex.m_count == 3)
                {
                    break;
                }

                // Compute closest point.
                simplex.getClosestPoint(closestPoint);
                distanceSqr2 = closestPoint.lengthSquared();

                // ensure progress
                if (distanceSqr2 >= distanceSqr1)
                {
                    // break;
                }
                distanceSqr1 = distanceSqr2;

                // get search direction;
                simplex.getSearchDirection(d);

                // Ensure the search direction is numerically fit.
                if (d.lengthSquared() < Settings.EPSILON * Settings.EPSILON)
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
                SimplexVertex vertex = vertices[simplex.m_count];

                Rot.mulTransUnsafe(transformA.q, d.negateLocal(), temp);
                vertex.indexA = proxyA.getSupport(temp);
                Transform.mulToOutUnsafe(transformA, proxyA.getVertex(vertex.indexA), vertex.wA);
                // Vec2 wBLocal;
                Rot.mulTransUnsafe(transformB.q, d.negateLocal(), temp);
                vertex.indexB = proxyB.getSupport(temp);
                Transform.mulToOutUnsafe(transformB, proxyB.getVertex(vertex.indexB), vertex.wB);
                vertex.w.set_Renamed(vertex.wB).subLocal(vertex.wA);

                // Iteration count is equated to the number of support point calls.
                ++iter;
                ++GJK_ITERS;

                // Check for duplicate support points. This is the main termination criteria.
                bool duplicate = false;
                for (int i = 0; i < saveCount; ++i)
                {
                    if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i])
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
                ++simplex.m_count;
            }

            GJK_MAX_ITERS = MathUtils.max(GJK_MAX_ITERS, iter);

            // Prepare output.
            simplex.getWitnessPoints(output.pointA, output.pointB);
            output.distance = MathUtils.distance(output.pointA, output.pointB);
            output.iterations = iter;

            // Cache the simplex.
            simplex.writeCache(cache);

            // Apply radii if requested.
            if (input.useRadii)
            {
                float rA = proxyA.m_radius;
                float rB = proxyB.m_radius;

                if (output.distance > rA + rB && output.distance > Settings.EPSILON)
                {
                    // Shapes are still no overlapped.
                    // Move the witness points to the outer surface.
                    output.distance -= (rA + rB);
                    normal.set_Renamed(output.pointB).subLocal(output.pointA);
                    normal.normalize();
                    temp.set_Renamed(normal).mulLocal(rA);
                    output.pointA.addLocal(temp);
                    temp.set_Renamed(normal).mulLocal(rB);
                    output.pointB.subLocal(temp);
                }
                else
                {
                    // Shapes are overlapped when radii are considered.
                    // Move the witness points to the middle.
                    // Vec2 p = 0.5f * (output.pointA + output.pointB);
                    output.pointA.addLocal(output.pointB).mulLocal(.5f);
                    output.pointB.set_Renamed(output.pointA);
                    output.distance = 0.0f;
                }
            }
        }
    }
}