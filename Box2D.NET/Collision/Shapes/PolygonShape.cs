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
using Box2D.Common;
using Box2D.Pooling.Arrays;

namespace Box2D.Collision.Shapes
{

    /// <summary>
    /// A convex polygon shape. Polygons have a maximum number of vertices equal to _maxPolygonVertices.
    /// In most cases you should not need many vertices for a convex polygon.
    /// </summary>
    public sealed class PolygonShape : Shape
    {
        /// <summary>
        /// Dump lots of debug information.
        /// </summary>
        private const bool m_debug = false;

        /// <summary>
        /// Local position of the shape centroid in parent body frame.
        /// </summary>
        internal readonly Vec2 Centroid = new Vec2();

        // pooling
        private readonly Vec2 pool1 = new Vec2();
        private readonly Vec2 pool2 = new Vec2();
        private readonly Vec2 pool3 = new Vec2();
        private readonly Vec2 pool4 = new Vec2();
        private readonly Transform poolt1 = new Transform();

        public PolygonShape() :
            base(ShapeType.Polygon)
        {

            VertexCount = 0;
            Vertices = new Vec2[Settings.maxPolygonVertices];
            for (int i = 0; i < Vertices.Length; i++)
            {
                Vertices[i] = new Vec2();
            }
            Normals = new Vec2[Settings.maxPolygonVertices];
            for (int i = 0; i < Normals.Length; i++)
            {
                Normals[i] = new Vec2();
            }
            Radius = Settings.polygonRadius;
            Centroid.SetZero();
        }

        public override Shape Clone()
        {
            PolygonShape shape = new PolygonShape();
            shape.Centroid.Set(Centroid);
            for (int i = 0; i < shape.Normals.Length; i++)
            {
                shape.Normals[i].Set(Normals[i]);
                shape.Vertices[i].Set(Vertices[i]);
            }
            shape.Radius = Radius;
            shape.VertexCount = VertexCount;
            return shape;
        }

        /// <summary>
        /// Set this as a single edge.
        /// </summary>
        /// <param name="v1"></param>
        /// <param name="v2"></param>
        /// <deprecated></deprecated>
        public void SetAsEdge(Vec2 v1, Vec2 v2)
        {
            VertexCount = 2;
            Vertices[0].Set(v1);
            Vertices[1].Set(v2);
            Centroid.Set(v1).AddLocal(v2).MulLocal(0.5f);
            // = 0.5f * (v1 + v2);
            Normals[0].Set(v2).SubLocal(v1);
            Vec2.CrossToOut(Normals[0], 1f, Normals[0]);
            // m_normals[0] = Cross(v2 - v1, 1.0f);
            Normals[0].Normalize();
            Normals[1].Set(Normals[0]).NegateLocal();
        }

        /// <summary>
        /// Create a convex hull from the given array of points.
        /// The count must be in the range [3, Settings.maxPolygonVertices].
        /// </summary>
        /// <warning>the points may be re-ordered, even if they form a convex polygon</warning>
        /// <warning>collinear points are handled but not removed. Collinear points may lead to poor stacking behavior.</warning>
        public void Set(Vec2[] vertices, int count)
        {
            Set(vertices, count, null, null);
        }

        /// <summary>
        /// Create a convex hull from the given array of points.
        /// The count must be in the range [3, Settings.maxPolygonVertices].
        /// This method takes an arraypool for pooling
        /// </summary>
        /// <warning>the points may be re-ordered, even if they form a convex polygon</warning>
        /// <warning>collinear points are handled but not removed. Collinear points may lead to poor stacking behavior.</warning>
        public void Set(Vec2[] verts, int num, Vec2Array vecPool, IntArray intPool)
        {
            Debug.Assert(3 <= num && num <= Settings.maxPolygonVertices);
            if (num < 3)
            {
                SetAsBox(1.0f, 1.0f);
                return;
            }

            int n = MathUtils.Min(num, Settings.maxPolygonVertices);

            // Copy the vertices into a local buffer
            Vec2[] ps = (vecPool != null) ? vecPool.Get(n) : new Vec2[n];
            for (int i = 0; i < n; ++i)
            {
                ps[i] = verts[i];
            }

            // Create the convex hull using the Gift wrapping algorithm
            // http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

            // Find the right most point on the hull
            int i0 = 0;
            float x0 = ps[0].X;
            for (int i = 1; i < num; ++i)
            {
                float x = ps[i].X;
                if (x > x0 || (x == x0 && ps[i].Y < ps[i0].Y))
                {
                    i0 = i;
                    x0 = x;
                }
            }

            int[] hull = (intPool != null) ? intPool.Get(Settings.maxPolygonVertices) : new int[Settings.maxPolygonVertices];
            int m = 0;
            int ih = i0;

            while (true)
            {
                hull[m] = ih;

                int ie = 0;
                for (int j = 1; j < n; ++j)
                {
                    if (ie == ih)
                    {
                        ie = j;
                        continue;
                    }

                    Vec2 r = pool1.Set(ps[ie]).SubLocal(ps[hull[m]]);
                    Vec2 v = pool2.Set(ps[j]).SubLocal(ps[hull[m]]);
                    float c = Vec2.Cross(r, v);
                    if (c < 0.0f)
                    {
                        ie = j;
                    }

                    // Collinearity check
                    if (c == 0.0f && v.LengthSquared() > r.LengthSquared())
                    {
                        ie = j;
                    }
                }

                ++m;
                ih = ie;

                if (ie == i0)
                {
                    break;
                }
            }

            VertexCount = m;

            // Copy vertices.
            for (int i = 0; i < VertexCount; ++i)
            {
                if (Vertices[i] == null)
                {
                    Vertices[i] = new Vec2();
                }
                Vertices[i].Set(ps[hull[i]]);
            }

            Vec2 edge = pool1;

            // Compute normals. Ensure the edges have non-zero length.
            for (int i = 0; i < VertexCount; ++i)
            {
                int i1 = i;
                int i2 = i + 1 < VertexCount ? i + 1 : 0;
                edge.Set(Vertices[i2]).SubLocal(Vertices[i1]);

                Debug.Assert(edge.LengthSquared() > Settings.EPSILON * Settings.EPSILON);
                Vec2.CrossToOutUnsafe(edge, 1f, Normals[i]);
                Normals[i].Normalize();
            }

            // Compute the polygon centroid.
            ComputeCentroidToOut(Vertices, VertexCount, Centroid);
        }

        /// <summary>
        /// Build vertices to represent an axis-aligned box.
        /// </summary>
        /// <param name="hx">the half-width.</param>
        /// <param name="hy">the half-height.</param>
        public void SetAsBox(float hx, float hy)
        {
            VertexCount = 4;
            Vertices[0].Set(-hx, -hy);
            Vertices[1].Set(hx, -hy);
            Vertices[2].Set(hx, hy);
            Vertices[3].Set(-hx, hy);
            Normals[0].Set(0.0f, -1.0f);
            Normals[1].Set(1.0f, 0.0f);
            Normals[2].Set(0.0f, 1.0f);
            Normals[3].Set(-1.0f, 0.0f);
            Centroid.SetZero();
        }

        /// <summary>
        /// Build vertices to represent an oriented box.
        /// </summary>
        /// <param name="hx">the half-width.</param>
        /// <param name="hy">the half-height.</param>
        /// <param name="center">the center of the box in local coordinates.</param>
        /// <param name="angle">the rotation of the box in local coordinates.</param>
        public void SetAsBox(float hx, float hy, Vec2 center, float angle)
        {
            VertexCount = 4;
            Vertices[0].Set(-hx, -hy);
            Vertices[1].Set(hx, -hy);
            Vertices[2].Set(hx, hy);
            Vertices[3].Set(-hx, hy);
            Normals[0].Set(0.0f, -1.0f);
            Normals[1].Set(1.0f, 0.0f);
            Normals[2].Set(0.0f, 1.0f);
            Normals[3].Set(-1.0f, 0.0f);
            Centroid.Set(center);

            Transform xf = poolt1;
            xf.p.Set(center);
            xf.q.Set(angle);

            // Transform vertices and normals.
            for (int i = 0; i < VertexCount; ++i)
            {
                Transform.mulToOut(xf, Vertices[i], Vertices[i]);
                Rot.MulToOut(xf.q, Normals[i], Normals[i]);
            }
        }

        override public int ChildCount
        {
            get
            {
                return 1;
            }
        }

        public override bool TestPoint(Transform xf, Vec2 p)
        {
            Vec2 pLocal = pool1;
            Vec2 temp = pool2;

            pLocal.Set(p).SubLocal(xf.p);
            Rot.MulTransUnsafe(xf.q, pLocal, temp);
            pLocal.Set(temp);

            if (m_debug)
            {
                Console.Out.WriteLine("--testPoint debug--");
                Console.Out.WriteLine("Vertices: ");
                for (int i = 0; i < VertexCount; ++i)
                {
                    Console.Out.WriteLine(Vertices[i]);
                }
                Console.Out.WriteLine("pLocal: " + pLocal);
            }


            for (int i = 0; i < VertexCount; ++i)
            {
                temp.Set(pLocal).SubLocal(Vertices[i]);
                float dot = Vec2.Dot(Normals[i], temp);
                if (dot > 0.0f)
                {
                    return false;
                }
            }

            return true;
        }

        public override void ComputeAABB(AABB aabb, Transform xf, int childIndex)
        {
            Vec2 v = pool1;
            Vec2 lower = aabb.LowerBound;
            Vec2 upper = aabb.UpperBound;
            Vec2 v1 = Vertices[0];
            lower.X = (xf.q.Cos * v1.X - xf.q.Sin * v1.Y) + xf.p.X;
            lower.Y = (xf.q.Sin * v1.X + xf.q.Cos * v1.Y) + xf.p.Y;
            upper.Set(lower);

            for (int i = 1; i < VertexCount; ++i)
            {
                Vec2 v2 = Vertices[i];
                v.X = (xf.q.Cos * v2.X - xf.q.Sin * v2.Y) + xf.p.X;
                v.Y = (xf.q.Sin * v2.X + xf.q.Cos * v2.Y) + xf.p.Y;
                // Vec2 v = Mul(xf, m_vertices[i]);
                Vec2.MinToOut(lower, v, lower);
                Vec2.MaxToOut(upper, v, upper);
            }

            // Vec2 r(m_radius, m_radius);
            // aabb.lowerBound = lower - r;
            // aabb.upperBound = upper + r;

            aabb.LowerBound.X -= Radius;
            aabb.LowerBound.Y -= Radius;
            aabb.UpperBound.X += Radius;
            aabb.UpperBound.Y += Radius;
        }

        // djm pooling, and from above
        /*
        * private static final TLVec2 tlNormalL = new TLVec2(); private static final TLMassData tlMd =
        * new TLMassData(); private static final FloatArray tldepths = new FloatArray(); private static
        * final TLVec2 tlIntoVec = new TLVec2(); private static final TLVec2 tlOutoVec = new TLVec2();
        * private static final TLVec2 tlP2b = new TLVec2(); private static final TLVec2 tlP3 = new
        * TLVec2(); private static final TLVec2 tlcenter = new TLVec2(); /*
        * 
        * @see Shape#computeSubmergedArea(Vec2, float, XForm, Vec2) public float
        * computeSubmergedArea(final Vec2 normal, float offset, Transform xf, Vec2 c) { final Vec2
        * normalL = tlNormalL.get(); final MassData md = tlMd.get(); //Transform plane into shape
        * co-ordinates Mat22.mulTransToOut(xf.R,normal, normalL); float offsetL = offset -
        * Vec2.dot(normal,xf.position); final Float[] depths = tldepths.get(Settings.maxPolygonVertices);
        * int diveCount = 0; int intoIndex = -1; int outoIndex = -1; boolean lastSubmerged = false; int i
        * = 0; for (i = 0; i < m_vertexCount; ++i){ depths[i] = Vec2.dot(normalL,m_vertices[i]) -
        * offsetL; boolean isSubmerged = depths[i]<-Settings.EPSILON; if (i > 0){ if (isSubmerged){ if
        * (!lastSubmerged){ intoIndex = i-1; diveCount++; } } else{ if (lastSubmerged){ outoIndex = i-1;
        * diveCount++; } } } lastSubmerged = isSubmerged; } switch(diveCount){ case 0: if
        * (lastSubmerged){ //Completely submerged computeMass(md, 1.0f); Transform.mulToOut(xf,md.center,
        * c); return md.mass; } else{ return 0; } case 1: if(intoIndex==-1){ intoIndex = m_vertexCount-1;
        * } else{ outoIndex = m_vertexCount-1; } break; } final Vec2 intoVec = tlIntoVec.get(); final
        * Vec2 outoVec = tlOutoVec.get(); final Vec2 e1 = tle1.get(); final Vec2 e2 = tle2.get(); int
        * intoIndex2 = (intoIndex+1) % m_vertexCount; int outoIndex2 = (outoIndex+1) % m_vertexCount;
        * float intoLambda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]); float
        * outoLambda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]);
        * intoVec.set(m_vertices[intoIndex].x*(1-intoLambda)+m_vertices[intoIndex2].x*intoLambda ,
        * m_vertices[intoIndex].y*(1-intoLambda)+m_vertices[intoIndex2].y*intoLambda);
        * outoVec.set(m_vertices[outoIndex].x*(1-outoLambda)+m_vertices[outoIndex2].x*outoLambda ,
        * m_vertices[outoIndex].y*(1-outoLambda)+m_vertices[outoIndex2].y*outoLambda); // Initialize
        * accumulator float area = 0; final Vec2 center = tlcenter.get(); center.setZero(); final Vec2
        * p2b = tlP2b.get().set(m_vertices[intoIndex2]); final Vec2 p3 = tlP3.get(); p3.setZero(); float
        * k_inv3 = 1.0f / 3.0f; // An awkward loop from intoIndex2+1 to outIndex2 i = intoIndex2; while
        * (i != outoIndex2){ i = (i+1) % m_vertexCount; if (i == outoIndex2){ p3.set(outoVec); } else{
        * p3.set(m_vertices[i]); } // Add the triangle formed by intoVec,p2,p3 {
        * e1.set(p2b).subLocal(intoVec); e2.set(p3).subLocal(intoVec); float D = Vec2.cross(e1, e2);
        * float triangleArea = 0.5f * D; area += triangleArea; // Area weighted centroid center.x +=
        * triangleArea * k_inv3 * (intoVec.x + p2b.x + p3.x); center.y += triangleArea * k_inv3 *
        * (intoVec.y + p2b.y + p3.y); } // p2b.set(p3); } // Normalize and transform centroid center.x *=
        * 1.0f / area; center.y *= 1.0f / area; Transform.mulToOut(xf, center, c); return area; }
        */

        /*
        * Get the supporting vertex index in the given direction.
        * 
        * @param d
        * 
        * @return public final int getSupport( final Vec2 d){ int bestIndex = 0; float bestValue =
        * Vec2.dot(m_vertices[0], d); for (int i = 1; i < m_vertexCount; ++i){ final float value =
        * Vec2.dot(m_vertices[i], d); if (value > bestValue){ bestIndex = i; bestValue = value; } }
        * return bestIndex; } /** Get the supporting vertex in the given direction.
        * 
        * @param d
        * 
        * @return public final Vec2 getSupportVertex( final Vec2 d){ int bestIndex = 0; float bestValue =
        * Vec2.dot(m_vertices[0], d); for (int i = 1; i < m_vertexCount; ++i){ final float value =
        * Vec2.dot(m_vertices[i], d); if (value > bestValue){ bestIndex = i; bestValue = value; } }
        * return m_vertices[bestIndex]; }
        */


        /// <summary>
        /// Get the vertex count.
        /// </summary>
        /// <returns></returns>
        public int VertexCount { get; set; }

        /// <summary>
        /// Get a vertex by index.
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public Vec2 GetVertex(int index)
        {
            Debug.Assert(0 <= index && index < VertexCount);
            return Vertices[index];
        }

        public override bool Raycast(RayCastOutput output, RayCastInput input, Transform xf, int childIndex)
        {
            Vec2 p1 = pool1;
            Vec2 p2 = pool2;
            Vec2 d = pool3;
            Vec2 temp = pool4;

            p1.Set(input.P1).SubLocal(xf.p);
            Rot.MulTrans(xf.q, p1, p1);
            p2.Set(input.P2).SubLocal(xf.p);
            Rot.MulTrans(xf.q, p2, p2);
            d.Set(p2).SubLocal(p1);

            // if (count == 2) {

            // } else {

            float lower = 0, upper = input.MaxFraction;

            int index = -1;

            for (int i = 0; i < VertexCount; ++i)
            {
                // p = p1 + a * d
                // dot(normal, p - v) = 0
                // dot(normal, p1 - v) + a * dot(normal, d) = 0
                temp.Set(Vertices[i]).SubLocal(p1);
                float numerator = Vec2.Dot(Normals[i], temp);
                float denominator = Vec2.Dot(Normals[i], d);

                if (denominator == 0.0f)
                {
                    if (numerator < 0.0f)
                    {
                        return false;
                    }
                }
                else
                {
                    // Note: we want this predicate without division:
                    // lower < numerator / denominator, where denominator < 0
                    // Since denominator < 0, we have to flip the inequality:
                    // lower < numerator / denominator <==> denominator * lower >
                    // numerator.
                    if (denominator < 0.0f && numerator < lower * denominator)
                    {
                        // Increase lower.
                        // The segment enters this half-space.
                        lower = numerator / denominator;
                        index = i;
                    }
                    else if (denominator > 0.0f && numerator < upper * denominator)
                    {
                        // Decrease upper.
                        // The segment exits this half-space.
                        upper = numerator / denominator;
                    }
                }

                if (upper < lower)
                {
                    return false;
                }
            }

            Debug.Assert(0.0f <= lower && lower <= input.MaxFraction);

            if (index >= 0)
            {
                output.Fraction = lower;
                Rot.MulToOutUnsafe(xf.q, Normals[index], output.Normal);
                // normal = Mul(xf.R, m_normals[index]);
                return true;
            }
            return false;
        }

        public void ComputeCentroidToOut(Vec2[] vs, int count, Vec2 result)
        {
            Debug.Assert(count >= 3);

            result.Set(0.0f, 0.0f);
            float area = 0.0f;

            // pRef is the reference point for forming triangles.
            // It's location doesn't change the result (except for rounding error).
            Vec2 pRef = pool1;
            pRef.SetZero();

            Vec2 e1 = pool2;
            Vec2 e2 = pool3;

            const float inv3 = 1.0f / 3.0f;

            for (int i = 0; i < count; ++i)
            {
                // Triangle vertices.
                Vec2 p1 = pRef;
                Vec2 p2 = vs[i];
                Vec2 p3 = i + 1 < count ? vs[i + 1] : vs[0];

                e1.Set(p2).SubLocal(p1);
                e2.Set(p3).SubLocal(p1);

                float D = Vec2.Cross(e1, e2);

                float triangleArea = 0.5f * D;
                area += triangleArea;

                // Area weighted centroid
                e1.Set(p1).AddLocal(p2).AddLocal(p3).MulLocal(triangleArea * inv3);
                result.AddLocal(e1);
            }

            // Centroid
            Debug.Assert(area > Settings.EPSILON);
            result.MulLocal(1.0f / area);
        }

        public override void ComputeMass(MassData massData, float density)
        {
            // Polygon mass, centroid, and inertia.
            // Let rho be the polygon density in mass per unit area.
            // Then:
            // mass = rho * int(dA)
            // centroid.x = (1/mass) * rho * int(x * dA)
            // centroid.y = (1/mass) * rho * int(y * dA)
            // I = rho * int((x*x + y*y) * dA)
            //
            // We can compute these integrals by summing all the integrals
            // for each triangle of the polygon. To evaluate the integral
            // for a single triangle, we make a change of variables to
            // the (u,v) coordinates of the triangle:
            // x = x0 + e1x * u + e2x * v
            // y = y0 + e1y * u + e2y * v
            // where 0 <= u && 0 <= v && u + v <= 1.
            //
            // We integrate u from [0,1-v] and then v from [0,1].
            // We also need to use the Jacobian of the transformation:
            // D = cross(e1, e2)
            //
            // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
            //
            // The rest of the derivation is handled by computer algebra.

            Debug.Assert(VertexCount >= 3);

            Vec2 center = pool1;
            center.SetZero();
            float area = 0.0f;
            float I = 0.0f;

            // pRef is the reference point for forming triangles.
            // It's location doesn't change the result (except for rounding error).
            Vec2 s = pool2;
            s.SetZero();
            // This code would put the reference point inside the polygon.
            for (int i = 0; i < VertexCount; ++i)
            {
                s.AddLocal(Vertices[i]);
            }
            s.MulLocal(1.0f / VertexCount);

            const float k_inv3 = 1.0f / 3.0f;

            Vec2 e1 = pool3;
            Vec2 e2 = pool4;

            for (int i = 0; i < VertexCount; ++i)
            {
                // Triangle vertices.
                e1.Set(Vertices[i]).SubLocal(s);
                e2.Set(s).NegateLocal().AddLocal(i + 1 < VertexCount ? Vertices[i + 1] : Vertices[0]);

                float D = Vec2.Cross(e1, e2);

                float triangleArea = 0.5f * D;
                area += triangleArea;

                // Area weighted centroid
                center.X += triangleArea * k_inv3 * (e1.X + e2.X);
                center.Y += triangleArea * k_inv3 * (e1.Y + e2.Y);

                float ex1 = e1.X;
                float ey1 = e1.Y;
                float ex2 = e2.X;
                float ey2 = e2.Y;

                float intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
                float inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

                I += (0.25f * k_inv3 * D) * (intx2 + inty2);
            }

            // Total mass
            massData.Mass = density * area;

            // Center of mass
            Debug.Assert(area > Settings.EPSILON);
            center.MulLocal(1.0f / area);
            massData.Center.Set(center).AddLocal(s);

            // Inertia tensor relative to the local origin (point s)
            massData.I = I * density;

            // Shift to center of mass then to original body origin.
            massData.I += massData.Mass * (Vec2.Dot(massData.Center, massData.Center));
        }

        /// <summary>
        /// Validate convexity. This is a very time consuming operation.
        /// </summary>
        /// <returns></returns>
        public bool Validate()
        {
            for (int i = 0; i < VertexCount; ++i)
            {
                int i1 = i;
                int i2 = i < VertexCount - 1 ? i1 + 1 : 0;
                Vec2 p = Vertices[i1];
                Vec2 e = pool1.Set(Vertices[i2]).SubLocal(p);

                for (int j = 0; j < VertexCount; ++j)
                {
                    if (j == i1 || j == i2)
                    {
                        continue;
                    }

                    Vec2 v = pool2.Set(Vertices[j]).SubLocal(p);
                    float c = Vec2.Cross(e, v);
                    if (c < 0.0f)
                    {
                        return false;
                    }
                }
            }

            return true;
        }

        /// <summary>
        /// Get the vertices in local coordinates.
        /// </summary>
        public Vec2[] Vertices { get; private set; }

        /// <summary>
        /// Get the edge normal vectors. There is one for each vertex.
        /// </summary>
        public Vec2[] Normals { get; private set; }

        /// <summary>
        /// Get the centroid and apply the supplied transform.
        /// </summary>
        public Vec2 GetCentroid(Transform xf)
        {
            return Transform.mul(xf, Centroid);
        }

        /// <summary>
        /// Get the centroid and apply the supplied transform.
        /// </summary>
        public Vec2 CentroidToOut(Transform xf, Vec2 result)
        {
            Transform.mulToOut(xf, Centroid, result);
            return result;
        }
    }
}