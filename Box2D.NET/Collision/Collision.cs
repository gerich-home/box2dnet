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
using Box2D.Pooling;

namespace Box2D.Collision
{

    /// <summary>
    /// Functions used for computing contact points, distance queries, and TOI queries. Collision methods
    /// are non-static for pooling speed, retrieve a collision object from the {@link SingletonPool}.
    /// Should not be constructed.
    /// </summary>
    /// <author>Daniel Murphy</author>
    public class Collision
    {
        public static readonly int NullFeature = Int32.MaxValue;

        private readonly IWorldPool pool;

        public Collision(IWorldPool argPool)
        {
            incidentEdge[0] = new ClipVertex();
            incidentEdge[1] = new ClipVertex();
            clipPoints1[0] = new ClipVertex();
            clipPoints1[1] = new ClipVertex();
            clipPoints2[0] = new ClipVertex();
            clipPoints2[1] = new ClipVertex();
            pool = argPool;
        }

        private readonly DistanceInput input = new DistanceInput();
        private readonly Distance.SimplexCache cache = new Distance.SimplexCache();
        private readonly DistanceOutput output = new DistanceOutput();

        /// <summary>
        /// Determine if two generic shapes overlap.
        /// </summary>
        /// <param name="shapeA"></param>
        /// <param name="indexA"></param>
        /// <param name="shapeB"></param>
        /// <param name="indexB"></param>
        /// <param name="xfA"></param>
        /// <param name="xfB"></param>
        /// <returns></returns>
        public bool TestOverlap(Shape shapeA, int indexA, Shape shapeB, int indexB, Transform xfA, Transform xfB)
        {
            input.proxyA.Set(shapeA, indexA);
            input.proxyB.Set(shapeB, indexB);
            input.transformA.set_Renamed(xfA);
            input.transformB.set_Renamed(xfB);
            input.useRadii = true;

            cache.Count = 0;

            pool.GetDistance().GetDistance(output, cache, input);
            // djm note: anything significant about 10.0f?
            return output.distance < 10.0f * Settings.EPSILON;
        }

        /// <summary>
        /// Compute the point states given two manifolds. The states pertain to the transition from
        /// manifold1 to manifold2. So state1 is either persist or remove while state2 is either add or
        /// persist.
        /// </summary>
        /// <param name="state1"></param>
        /// <param name="state2"></param>
        /// <param name="manifold1"></param>
        /// <param name="manifold2"></param>
        public static void GetPointStates(PointState[] state1, PointState[] state2, Manifold manifold1, Manifold manifold2)
        {

            for (int i = 0; i < Settings.maxManifoldPoints; i++)
            {
                state1[i] = PointState.NullState;
                state2[i] = PointState.NullState;
            }

            // Detect persists and removes.
            for (int i = 0; i < manifold1.pointCount; i++)
            {
                ContactID id = manifold1.points[i].id;

                state1[i] = PointState.RemoveState;

                for (int j = 0; j < manifold2.pointCount; j++)
                {
                    if (manifold2.points[j].id.IsEqual(id))
                    {
                        state1[i] = PointState.PersistState;
                        break;
                    }
                }
            }

            // Detect persists and adds
            for (int i = 0; i < manifold2.pointCount; i++)
            {
                ContactID id = manifold2.points[i].id;

                state2[i] = PointState.AddState;

                for (int j = 0; j < manifold1.pointCount; j++)
                {
                    if (manifold1.points[j].id.IsEqual(id))
                    {
                        state2[i] = PointState.PersistState;
                        break;
                    }
                }
            }
        }

        /// <summary>
        /// Clipping for contact manifolds. Sutherland-Hodgman clipping.
        /// </summary>
        /// <param name="vOut"></param>
        /// <param name="vIn"></param>
        /// <param name="normal"></param>
        /// <param name="offset"></param>
        /// <param name="vertexIndexA"></param>
        /// <returns></returns>
        public static int ClipSegmentToLine(ClipVertex[] vOut, ClipVertex[] vIn, Vec2 normal, float offset, int vertexIndexA)
        {

            // Start with no output points
            int numOut = 0;

            // Calculate the distance of end points to the line
            float distance0 = Vec2.dot(normal, vIn[0].v) - offset;
            float distance1 = Vec2.dot(normal, vIn[1].v) - offset;

            // If the points are behind the plane
            if (distance0 <= 0.0f)
            {
                vOut[numOut++].Set(vIn[0]);
            }
            if (distance1 <= 0.0f)
            {
                vOut[numOut++].Set(vIn[1]);
            }

            // If the points are on different sides of the plane
            if (distance0 * distance1 < 0.0f)
            {
                // Find intersection point of edge and plane
                float interp = distance0 / (distance0 - distance1);
                // vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
                vOut[numOut].v.set_Renamed(vIn[1].v).subLocal(vIn[0].v).mulLocal(interp).addLocal(vIn[0].v);

                // VertexA is hitting edgeB.
                vOut[numOut].id.IndexA = (sbyte)vertexIndexA;
                vOut[numOut].id.IndexB = vIn[0].id.IndexB;
                vOut[numOut].id.TypeA = (sbyte)ContactID.Type.Vertex;
                vOut[numOut].id.TypeB = (sbyte)ContactID.Type.Face;
                ++numOut;
            }

            return numOut;
        }

        // #### COLLISION STUFF (not from collision.h or collision.cpp) ####

        // djm pooling
        private static readonly Vec2 pA = new Vec2();
        private static readonly Vec2 pB = new Vec2();
        private static readonly Vec2 d = new Vec2();

        /// <summary>
        /// Compute the collision manifold between two circles.
        /// </summary>
        /// <param name="manifold"></param>
        /// <param name="circle1"></param>
        /// <param name="xfA"></param>
        /// <param name="circle2"></param>
        /// <param name="xfB"></param>
        public void CollideCircles(Manifold manifold, CircleShape circle1, Transform xfA, CircleShape circle2, Transform xfB)
        {
            manifold.pointCount = 0;

            // before inline:
            Transform.mulToOut(xfA, circle1.P, pA);
            Transform.mulToOut(xfB, circle2.P, pB);
            d.set_Renamed(pB).subLocal(pA);
            float distSqr = d.x * d.x + d.y * d.y;

            // after inline:
            // final Vec2 v = circle1.m_p;
            // final float pAy = xfA.p.y + xfA.q.ex.y * v.x + xfA.q.ey.y * v.y;
            // final float pAx = xfA.p.x + xfA.q.ex.x * v.x + xfA.q.ey.x * v.y;
            //
            // final Vec2 v1 = circle2.m_p;
            // final float pBy = xfB.p.y + xfB.q.ex.y * v1.x + xfB.q.ey.y * v1.y;
            // final float pBx = xfB.p.x + xfB.q.ex.x * v1.x + xfB.q.ey.x * v1.y;
            //
            // final float dx = pBx - pAx;
            // final float dy = pBy - pAy;
            //
            // final float distSqr = dx * dx + dy * dy;
            // end inline

            float radius = circle1.Radius + circle2.Radius;
            if (distSqr > radius * radius)
            {
                return;
            }

            manifold.type = Manifold.ManifoldType.CIRCLES;
            manifold.localPoint.set_Renamed(circle1.P);
            manifold.localNormal.setZero();
            manifold.pointCount = 1;

            manifold.points[0].localPoint.set_Renamed(circle2.P);
            manifold.points[0].id.Zero();
        }

        // djm pooling, and from above
        private static readonly Vec2 c = new Vec2();
        private static readonly Vec2 cLocal = new Vec2();

        /// <summary>
        /// Compute the collision manifold between a polygon and a circle.
        /// </summary>
        /// <param name="manifold"></param>
        /// <param name="polygon"></param>
        /// <param name="xfA"></param>
        /// <param name="circle"></param>
        /// <param name="xfB"></param>
        public void CollidePolygonAndCircle(Manifold manifold, PolygonShape polygon, Transform xfA, CircleShape circle, Transform xfB)
        {
            manifold.pointCount = 0;
            //Vec2 v = circle.m_p;

            // Compute circle position in the frame of the polygon.
            // before inline:
            Transform.mulToOut(xfB, circle.P, c);
            Transform.mulTransToOut(xfA, c, cLocal);

            float cLocalx = cLocal.x;
            float cLocaly = cLocal.y;
            // after inline:
            // final float cy = xfB.p.y + xfB.q.ex.y * v.x + xfB.q.ey.y * v.y;
            // final float cx = xfB.p.x + xfB.q.ex.x * v.x + xfB.q.ey.x * v.y;
            // final float v1x = cx - xfA.p.x;
            // final float v1y = cy - xfA.p.y;
            // final Vec2 b = xfA.q.ex;
            // final Vec2 b1 = xfA.q.ey;
            // final float cLocaly = v1x * b1.x + v1y * b1.y;
            // final float cLocalx = v1x * b.x + v1y * b.y;
            // end inline

            // Find the min separating edge.
            int normalIndex = 0;
            //UPGRADE_TODO: The equivalent in .NET for field 'java.lang.Float.MIN_VALUE' may return a different value. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1043'"
            float separation = Single.Epsilon;
            float radius = polygon.Radius + circle.Radius;
            int vertexCount = polygon.VertexCount;

            Vec2[] vertices = polygon.Vertices;
            Vec2[] normals = polygon.Normals;

            for (int i = 0; i < vertexCount; i++)
            {
                // before inline
                // temp.set(cLocal).subLocal(vertices[i]);
                // float s = Vec2.dot(normals[i], temp);
                // after inline
                Vec2 vertex = vertices[i];
                float tempx = cLocalx - vertex.x;
                float tempy = cLocaly - vertex.y;
                Vec2 normal = normals[i];
                float s = normal.x * tempx + normal.y * tempy;


                if (s > radius)
                {
                    // early out
                    return;
                }

                if (s > separation)
                {
                    separation = s;
                    normalIndex = i;
                }
            }

            // Vertices that subtend the incident face.
            int vertIndex1 = normalIndex;
            int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
            Vec2 v1 = vertices[vertIndex1];
            Vec2 v2 = vertices[vertIndex2];

            // If the center is inside the polygon ...
            if (separation < Settings.EPSILON)
            {
                manifold.pointCount = 1;
                manifold.type = Manifold.ManifoldType.FACE_A;

                // before inline:
                // manifold.localNormal.set(normals[normalIndex]);
                // manifold.localPoint.set(v1).addLocal(v2).mulLocal(.5f);
                // manifold.points[0].localPoint.set(circle.m_p);
                // after inline:
                Vec2 normal = normals[normalIndex];
                manifold.localNormal.x = normal.x;
                manifold.localNormal.y = normal.y;
                manifold.localPoint.x = (v1.x + v2.x) * .5f;
                manifold.localPoint.y = (v1.y + v2.y) * .5f;
                ManifoldPoint mpoint = manifold.points[0];
                mpoint.localPoint.x = circle.P.x;
                mpoint.localPoint.y = circle.P.y;
                mpoint.id.Zero();
                // end inline
                return;
            }

            // Compute barycentric coordinates
            // before inline:
            // temp.set(cLocal).subLocal(v1);
            // temp2.set(v2).subLocal(v1);
            // float u1 = Vec2.dot(temp, temp2);
            // temp.set(cLocal).subLocal(v2);
            // temp2.set(v1).subLocal(v2);
            // float u2 = Vec2.dot(temp, temp2);
            // after inline:
            float tempX = cLocalx - v1.x;
            float tempY = cLocaly - v1.y;
            float temp2X = v2.x - v1.x;
            float temp2Y = v2.y - v1.y;
            float u1 = tempX * temp2X + tempY * temp2Y;

            float temp3X = cLocalx - v2.x;
            float temp3Y = cLocaly - v2.y;
            float temp4X = v1.x - v2.x;
            float temp4Y = v1.y - v2.y;
            float u2 = temp3X * temp4X + temp3Y * temp4Y;
            // end inline

            if (u1 <= 0f)
            {
                // inlined
                float dx = cLocalx - v1.x;
                float dy = cLocaly - v1.y;
                if (dx * dx + dy * dy > radius * radius)
                {
                    return;
                }

                manifold.pointCount = 1;
                manifold.type = Manifold.ManifoldType.FACE_A;
                // before inline:
                // manifold.localNormal.set(cLocal).subLocal(v1);
                // after inline:
                manifold.localNormal.x = cLocalx - v1.x;
                manifold.localNormal.y = cLocaly - v1.y;
                // end inline
                manifold.localNormal.normalize();
                manifold.localPoint.set_Renamed(v1);
                manifold.points[0].localPoint.set_Renamed(circle.P);
                manifold.points[0].id.Zero();
            }
            else if (u2 <= 0.0f)
            {
                // inlined
                float dx = cLocalx - v2.x;
                float dy = cLocaly - v2.y;
                if (dx * dx + dy * dy > radius * radius)
                {
                    return;
                }

                manifold.pointCount = 1;
                manifold.type = Manifold.ManifoldType.FACE_A;
                // before inline:
                // manifold.localNormal.set(cLocal).subLocal(v2);
                // after inline:
                manifold.localNormal.x = cLocalx - v2.x;
                manifold.localNormal.y = cLocaly - v2.y;
                // end inline
                manifold.localNormal.normalize();
                manifold.localPoint.set_Renamed(v2);
                manifold.points[0].localPoint.set_Renamed(circle.P);
                manifold.points[0].id.Zero();
            }
            else
            {
                // Vec2 faceCenter = 0.5f * (v1 + v2);
                // (temp is faceCenter)
                // before inline:
                // temp.set(v1).addLocal(v2).mulLocal(.5f);
                //
                // temp2.set(cLocal).subLocal(temp);
                // separation = Vec2.dot(temp2, normals[vertIndex1]);
                // if (separation > radius) {
                // return;
                // }
                // after inline:
                float fcx = (v1.x + v2.x) * .5f;
                float fcy = (v1.y + v2.y) * .5f;

                float tx = cLocalx - fcx;
                float ty = cLocaly - fcy;
                Vec2 normal = normals[vertIndex1];
                separation = tx * normal.x + ty * normal.y;
                if (separation > radius)
                {
                    return;
                }
                // end inline

                manifold.pointCount = 1;
                manifold.type = Manifold.ManifoldType.FACE_A;
                manifold.localNormal.set_Renamed(normals[vertIndex1]);
                manifold.localPoint.x = fcx; // (faceCenter)
                manifold.localPoint.y = fcy;
                manifold.points[0].localPoint.set_Renamed(circle.P);
                manifold.points[0].id.Zero();
            }
        }

        // djm pooling
        private readonly Vec2 normal1 = new Vec2();
        private readonly Vec2 normal1World = new Vec2();
        private readonly Vec2 v1 = new Vec2();
        private readonly Vec2 v2 = new Vec2();

        /// <summary> Find the separation between poly1 and poly2 for a given edge normal on poly1.
        /// 
        /// </summary>
        /// <param name="poly1">
        /// </param>
        /// <param name="xf1">
        /// </param>
        /// <param name="edge1">
        /// </param>
        /// <param name="poly2">
        /// </param>
        /// <param name="xf2">
        /// </param>
        public float EdgeSeparation(PolygonShape poly1, Transform xf1, int edge1, PolygonShape poly2, Transform xf2)
        {
            int count1;
            Vec2[] vertices1 = poly1.Vertices;
            Vec2[] normals1 = poly1.Normals;

            int count2 = poly2.VertexCount;
            Vec2[] vertices2 = poly2.Vertices;

            Debug.Assert(0 <= edge1 && edge1 < count1);
            // Convert normal from poly1's frame into poly2's frame.
            // before inline:
            // Vec2 normal1World = Mul(xf1.R, normals1[edge1]);
            Rot.mulToOutUnsafe(xf1.q, normals1[edge1], normal1World);
            // Vec2 normal1 = MulT(xf2.R, normal1World);
            Rot.mulTransUnsafe(xf2.q, normal1World, normal1);
            float normal1x = normal1.x;
            float normal1y = normal1.y;
            // after inline:
            // R.mulToOut(v,out);
            // final Mat22 R = xf1.q;
            // final Vec2 v = normals1[edge1];
            // final float normal1Worldy = R.ex.y * v.x + R.ey.y * v.y;
            // final float normal1Worldx = R.ex.x * v.x + R.ey.x * v.y;
            // final Mat22 R1 = xf2.q;
            // final float normal1x = normal1Worldx * R1.ex.x + normal1Worldy * R1.ex.y;
            // final float normal1y = normal1Worldx * R1.ey.x + normal1Worldy * R1.ey.y;
            // end inline

            // Find support vertex on poly2 for -normal.
            int index = 0;
            float minDot = Single.MaxValue;

            for (int i = 0; i < count2; ++i)
            {
                Vec2 a = vertices2[i];
                float dot = a.x * normal1x + a.y * normal1y;
                if (dot < minDot)
                {
                    minDot = dot;
                    index = i;
                }
            }

            // Vec2 v1 = Mul(xf1, vertices1[edge1]);
            // Vec2 v2 = Mul(xf2, vertices2[index]);
            // before inline:
            Transform.mulToOut(xf1, vertices1[edge1], v1);
            Transform.mulToOut(xf2, vertices2[index], v2);

            float separation = Vec2.dot(v2.subLocal(v1), normal1World);
            return separation;

            // after inline:
            // final Vec2 v3 = vertices1[edge1];
            // final float v1y = xf1.p.y + R.ex.y * v3.x + R.ey.y * v3.y;
            // final float v1x = xf1.p.x + R.ex.x * v3.x + R.ey.x * v3.y;
            // final Vec2 v4 = vertices2[index];
            // final float v2y = xf2.p.y + R1.ex.y * v4.x + R1.ey.y * v4.y - v1y;
            // final float v2x = xf2.p.x + R1.ex.x * v4.x + R1.ey.x * v4.y - v1x;
            //
            // return v2x * normal1Worldx + v2y * normal1Worldy;
            // end inline
        }

        // djm pooling, and from above
        private readonly Vec2 temp = new Vec2();
        private readonly Vec2 dLocal1 = new Vec2();

        /// <summary>
        /// Find the max separation between poly1 and poly2 using edge normals from poly1.
        /// </summary>
        /// <param name="results"></param>
        /// <param name="poly1"></param>
        /// <param name="xf1"></param>
        /// <param name="poly2"></param>
        /// <param name="xf2"></param>
        /// <returns></returns>
        public void FindMaxSeparation(EdgeResults results, PolygonShape poly1, Transform xf1, PolygonShape poly2, Transform xf2)
        {
            int count1 = poly1.VertexCount;
            Vec2[] normals1 = poly1.Normals;
            //Vec2 v = poly2.m_centroid;

            // Vector pointing from the centroid of poly1 to the centroid of poly2.
            // before inline:
            Transform.mulToOutUnsafe(xf2, poly2.Centroid, d);
            Transform.mulToOutUnsafe(xf1, poly1.Centroid, temp);
            d.subLocal(temp);

            Rot.mulTransUnsafe(xf1.q, d, dLocal1);
            float dLocal1x = dLocal1.x;
            float dLocal1y = dLocal1.y;
            // after inline:
            // final float predy = xf2.p.y + xf2.q.ex.y * v.x + xf2.q.ey.y * v.y;
            // final float predx = xf2.p.x + xf2.q.ex.x * v.x + xf2.q.ey.x * v.y;
            // final Vec2 v1 = poly1.m_centroid;
            // final float tempy = xf1.p.y + xf1.q.ex.y * v1.x + xf1.q.ey.y * v1.y;
            // final float tempx = xf1.p.x + xf1.q.ex.x * v1.x + xf1.q.ey.x * v1.y;
            // final float dx = predx - tempx;
            // final float dy = predy - tempy;
            //
            // final Mat22 R = xf1.q;
            // final float dLocal1x = dx * R.ex.x + dy * R.ex.y;
            // final float dLocal1y = dx * R.ey.x + dy * R.ey.y;
            // end inline

            // Find edge normal on poly1 that has the largest projection onto d.
            int edge = 0;
            float dot;
            //UPGRADE_TODO: The equivalent in .NET for field 'java.lang.Float.MIN_VALUE' may return a different value. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1043'"
            float maxDot = Single.Epsilon;
            for (int i = 0; i < count1; i++)
            {
                Vec2 normal = normals1[i];
                dot = normal.x * dLocal1x + normal.y * dLocal1y;
                if (dot > maxDot)
                {
                    maxDot = dot;
                    edge = i;
                }
            }

            // Get the separation for the edge normal.
            float s = EdgeSeparation(poly1, xf1, edge, poly2, xf2);

            // Check the separation for the previous edge normal.
            int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
            float sPrev = EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2);

            // Check the separation for the next edge normal.
            int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
            float sNext = EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2);

            // Find the best edge and the search direction.
            int bestEdge;
            float bestSeparation;
            int increment;
            if (sPrev > s && sPrev > sNext)
            {
                increment = -1;
                bestEdge = prevEdge;
                bestSeparation = sPrev;
            }
            else if (sNext > s)
            {
                increment = 1;
                bestEdge = nextEdge;
                bestSeparation = sNext;
            }
            else
            {
                results.EdgeIndex = edge;
                results.Separation = s;
                return;
            }

            // Perform a local search for the best edge normal.
            for (; ; )
            {
                if (increment == -1)
                {
                    edge = bestEdge - 1 >= 0 ? bestEdge - 1 : count1 - 1;
                }
                else
                {
                    edge = bestEdge + 1 < count1 ? bestEdge + 1 : 0;
                }

                s = EdgeSeparation(poly1, xf1, edge, poly2, xf2);

                if (s > bestSeparation)
                {
                    bestEdge = edge;
                    bestSeparation = s;
                }
                else
                {
                    break;
                }
            }

            results.EdgeIndex = bestEdge;
            results.Separation = bestSeparation;
        }

        // djm pooling from above
        public void FindIncidentEdge(ClipVertex[] c, PolygonShape poly1, Transform xf1, int edge1, PolygonShape poly2, Transform xf2)
        {
            int count1;
            Vec2[] normals1 = poly1.Normals;

            int count2 = poly2.VertexCount;
            Vec2[] vertices2 = poly2.Vertices;
            Vec2[] normals2 = poly2.Normals;

            Debug.Assert(0 <= edge1 && edge1 < count1);

            // Get the normal of the reference edge in poly2's frame.
            Rot.mulToOutUnsafe(xf1.q, normals1[edge1], normal1); // temporary
            // Vec2 normal1 = MulT(xf2.R, Mul(xf1.R, normals1[edge1]));
            Rot.mulTrans(xf2.q, normal1, normal1);

            // Find the incident edge on poly2.
            int index = 0;
            float minDot = Single.MaxValue;
            for (int i = 0; i < count2; ++i)
            {
                float dot = Vec2.dot(normal1, normals2[i]);
                if (dot < minDot)
                {
                    minDot = dot;
                    index = i;
                }
            }

            // Build the clip vertices for the incident edge.
            int i1 = index;
            int i2 = i1 + 1 < count2 ? i1 + 1 : 0;

            Transform.mulToOutUnsafe(xf2, vertices2[i1], c[0].v); // = Mul(xf2, vertices2[i1]);
            c[0].id.IndexA = (sbyte)edge1;
            c[0].id.IndexB = (sbyte)i1;
            c[0].id.TypeA = (sbyte)ContactID.Type.Face;
            c[0].id.TypeB = (sbyte)ContactID.Type.Vertex;

            Transform.mulToOutUnsafe(xf2, vertices2[i2], c[1].v); // = Mul(xf2, vertices2[i2]);
            c[1].id.IndexA = (sbyte)edge1;
            c[1].id.IndexB = (sbyte)i2;
            c[1].id.TypeA = (sbyte)ContactID.Type.Face;
            c[1].id.TypeB = (sbyte)ContactID.Type.Vertex;
        }

        private readonly EdgeResults results1 = new EdgeResults();
        private readonly EdgeResults results2 = new EdgeResults();
        private readonly ClipVertex[] incidentEdge = new ClipVertex[2];
        private readonly Vec2 localTangent = new Vec2();
        private readonly Vec2 localNormal = new Vec2();
        private readonly Vec2 planePoint = new Vec2();
        private readonly Vec2 tangent = new Vec2();
        private readonly Vec2 normal = new Vec2();
        private readonly Vec2 v11 = new Vec2();
        private readonly Vec2 v12 = new Vec2();
        private readonly ClipVertex[] clipPoints1 = new ClipVertex[2];
        private readonly ClipVertex[] clipPoints2 = new ClipVertex[2];

        /// <summary>
        /// Compute the collision manifold between two polygons.
        /// </summary>
        /// <param name="manifold"></param>
        /// <param name="polyA"></param>
        /// <param name="xfA"></param>
        /// <param name="polyB"></param>
        /// <param name="xfB"></param>
        public void CollidePolygons(Manifold manifold, PolygonShape polyA, Transform xfA, PolygonShape polyB, Transform xfB)
        {
            // Find edge normal of max separation on A - return if separating axis is found
            // Find edge normal of max separation on B - return if separation axis is found
            // Choose reference edge as min(minA, minB)
            // Find incident edge
            // Clip

            // The normal points from 1 to 2

            manifold.pointCount = 0;
            float totalRadius = polyA.Radius + polyB.Radius;

            FindMaxSeparation(results1, polyA, xfA, polyB, xfB);
            if (results1.Separation > totalRadius)
            {
                return;
            }

            FindMaxSeparation(results2, polyB, xfB, polyA, xfA);
            if (results2.Separation > totalRadius)
            {
                return;
            }

            PolygonShape poly1; // reference polygon
            PolygonShape poly2; // incident polygon
            Transform xf1, xf2;
            int edge1; // reference edge
            bool flip;
            const float k_relativeTol = 0.98f;
            const float k_absoluteTol = 0.001f;

            if (results2.Separation > k_relativeTol * results1.Separation + k_absoluteTol)
            {
                poly1 = polyB;
                poly2 = polyA;
                xf1 = xfB;
                xf2 = xfA;
                edge1 = results2.EdgeIndex;
                manifold.type = Manifold.ManifoldType.FACE_B;
                flip = true;
            }
            else
            {
                poly1 = polyA;
                poly2 = polyB;
                xf1 = xfA;
                xf2 = xfB;
                edge1 = results1.EdgeIndex;
                manifold.type = Manifold.ManifoldType.FACE_A;
                flip = false;
            }

            FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

            int count1 = poly1.VertexCount;
            Vec2[] vertices1 = poly1.Vertices;

            int iv1 = edge1;
            int iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;
            v11.set_Renamed(vertices1[iv1]);
            v12.set_Renamed(vertices1[iv2]);
            localTangent.set_Renamed(v12).subLocal(v11);
            localTangent.normalize();

            Vec2.crossToOutUnsafe(localTangent, 1f, localNormal); // Vec2 localNormal = Vec2.cross(dv,
            // 1.0f);

            planePoint.set_Renamed(v11).addLocal(v12).mulLocal(.5f); // Vec2 planePoint = 0.5f * (v11
            // + v12);

            Rot.mulToOutUnsafe(xf1.q, localTangent, tangent); // Vec2 sideNormal = Mul(xf1.R, v12
            // - v11);
            Vec2.crossToOutUnsafe(tangent, 1f, normal); // Vec2 frontNormal = Vec2.cross(sideNormal,
            // 1.0f);

            Transform.mulToOut(xf1, v11, v11);
            Transform.mulToOut(xf1, v12, v12);
            // v11 = Mul(xf1, v11);
            // v12 = Mul(xf1, v12);

            // Face offset
            float frontOffset = Vec2.dot(normal, v11);

            // Side offsets, extended by polytope skin thickness.
            float sideOffset1 = -Vec2.dot(tangent, v11) + totalRadius;
            float sideOffset2 = Vec2.dot(tangent, v12) + totalRadius;

            // Clip incident edge against extruded edge1 side edges.
            // ClipVertex clipPoints1[2];
            // ClipVertex clipPoints2[2];
            int np;

            // Clip to box side 1
            // np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, sideOffset1);
            tangent.negateLocal();
            np = ClipSegmentToLine(clipPoints1, incidentEdge, tangent, sideOffset1, iv1);
            tangent.negateLocal();

            if (np < 2)
            {
                return;
            }

            // Clip to negative box side 1
            np = ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2);

            if (np < 2)
            {
                return;
            }

            // Now clipPoints2 contains the clipped points.
            manifold.localNormal.set_Renamed(localNormal);
            manifold.localPoint.set_Renamed(planePoint);

            int pointCount = 0;
            for (int i = 0; i < Settings.maxManifoldPoints; ++i)
            {
                float separation = Vec2.dot(normal, clipPoints2[i].v) - frontOffset;

                if (separation <= totalRadius)
                {
                    ManifoldPoint cp = manifold.points[pointCount];
                    Transform.mulTransToOut(xf2, clipPoints2[i].v, cp.localPoint);
                    // cp.m_localPoint = MulT(xf2, clipPoints2[i].v);
                    cp.id.Set(clipPoints2[i].id);
                    if (flip)
                    {
                        // Swap features
                        cp.id.Flip();
                    }
                    ++pointCount;
                }
            }

            manifold.pointCount = pointCount;
        }


        private readonly Vec2 Q = new Vec2();
        private readonly Vec2 e = new Vec2();
        private readonly ContactID cf = new ContactID();
        private readonly Vec2 e1 = new Vec2();
        private readonly Vec2 P = new Vec2();
        private readonly Vec2 n = new Vec2();

        // Compute contact points for edge versus circle.
        // This accounts for edge connectivity.
        public virtual void CollideEdgeAndCircle(Manifold manifold, EdgeShape edgeA, Transform xfA, CircleShape circleB, Transform xfB)
        {
            manifold.pointCount = 0;


            // Compute circle in frame of edge
            // Vec2 Q = MulT(xfA, Mul(xfB, circleB.m_p));
            Transform.mulToOutUnsafe(xfB, circleB.P, temp);
            Transform.mulTransToOutUnsafe(xfA, temp, Q);

            Vec2 A = edgeA.Vertex1;
            Vec2 B = edgeA.Vertex2;
            e.set_Renamed(B).subLocal(A);

            // Barycentric coordinates
            float u = Vec2.dot(e, temp.set_Renamed(B).subLocal(Q));
            float v = Vec2.dot(e, temp.set_Renamed(Q).subLocal(A));

            float radius = edgeA.Radius + circleB.Radius;

            // ContactFeature cf;
            cf.IndexB = 0;
            cf.TypeB = (sbyte)ContactID.Type.Vertex;

            // Region A
            if (v <= 0.0f)
            {
                Vec2 _P = A;
                d.set_Renamed(Q).subLocal(_P);
                float dd = Vec2.dot(d, d);
                if (dd > radius * radius)
                {
                    return;
                }

                // Is there an edge connected to A?
                if (edgeA.HasVertex0)
                {
                    Vec2 A1 = edgeA.Vertex0;
                    Vec2 B1 = A;
                    e1.set_Renamed(B1).subLocal(A1);
                    float u1 = Vec2.dot(e1, temp.set_Renamed(B1).subLocal(Q));

                    // Is the circle in Region AB of the previous edge?
                    if (u1 > 0.0f)
                    {
                        return;
                    }
                }

                cf.IndexA = 0;
                cf.TypeA = (sbyte)ContactID.Type.Vertex;
                manifold.pointCount = 1;
                manifold.type = Manifold.ManifoldType.CIRCLES;
                manifold.localNormal.setZero();
                manifold.localPoint.set_Renamed(_P);
                // manifold.points[0].id.key = 0;
                manifold.points[0].id.Set(cf);
                manifold.points[0].localPoint.set_Renamed(circleB.P);
                return;
            }

            // Region B
            if (u <= 0.0f)
            {
                Vec2 _P = B;
                d.set_Renamed(Q).subLocal(_P);
                float dd = Vec2.dot(d, d);
                if (dd > radius * radius)
                {
                    return;
                }

                // Is there an edge connected to B?
                if (edgeA.HasVertex3)
                {
                    Vec2 B2 = edgeA.Vertex3;
                    Vec2 A2 = B;
                    Vec2 e2 = e1;
                    e2.set_Renamed(B2).subLocal(A2);
                    float v2 = Vec2.dot(e2, temp.set_Renamed(Q).subLocal(A2));

                    // Is the circle in Region AB of the next edge?
                    if (v2 > 0.0f)
                    {
                        return;
                    }
                }

                cf.IndexA = 1;
                cf.TypeA = (sbyte)ContactID.Type.Vertex;
                manifold.pointCount = 1;
                manifold.type = Manifold.ManifoldType.CIRCLES;
                manifold.localNormal.setZero();
                manifold.localPoint.set_Renamed(_P);
                // manifold.points[0].id.key = 0;
                manifold.points[0].id.Set(cf);
                manifold.points[0].localPoint.set_Renamed(circleB.P);
                return;
            }

            // Region AB
            float den = Vec2.dot(e, e);
            Debug.Assert(den > 0.0f);

            // Vec2 P = (1.0f / den) * (u * A + v * B);
            P.set_Renamed(A).mulLocal(u).addLocal(temp.set_Renamed(B).mulLocal(v));
            P.mulLocal(1.0f / den);
            d.set_Renamed(Q).subLocal(P);
            float dd2 = Vec2.dot(d, d);
            if (dd2 > radius * radius)
            {
                return;
            }

            n.x = -e.y;
            n.y = e.x;
            if (Vec2.dot(n, temp.set_Renamed(Q).subLocal(A)) < 0.0f)
            {
                n.set_Renamed(-n.x, -n.y);
            }
            n.normalize();

            cf.IndexA = 0;
            cf.TypeA = (sbyte)ContactID.Type.Face;
            manifold.pointCount = 1;
            manifold.type = Manifold.ManifoldType.FACE_A;
            manifold.localNormal.set_Renamed(n);
            manifold.localPoint.set_Renamed(A);
            // manifold.points[0].id.key = 0;
            manifold.points[0].id.Set(cf);
            manifold.points[0].localPoint.set_Renamed(circleB.P);
        }

        private readonly EPCollider collider = new EPCollider();

        public virtual void CollideEdgeAndPolygon(Manifold manifold, EdgeShape edgeA, Transform xfA, PolygonShape polygonB, Transform xfB)
        {
            collider.Collide(manifold, edgeA, xfA, polygonB, xfB);
        }

        /// <summary>
        ///  Java-specific class for returning edge results
        /// </summary>
        public class EdgeResults
        {
            public float Separation;
            public int EdgeIndex;
        }

        /// <summary>
        ///  Used for computing contact manifolds.
        /// </summary>
        public class ClipVertex
        {
            public readonly Vec2 v;
            public readonly ContactID id;

            public ClipVertex()
            {
                v = new Vec2();
                id = new ContactID();
            }

            public virtual void Set(ClipVertex cv)
            {
                v.set_Renamed(cv.v);
                id.Set(cv.id);
            }
        }

        /// <summary>
        /// This is used for determining the state of contact points.
        /// </summary>
        /// <author>Daniel Murphy</author>
        public enum PointState
        {
            /// <summary> point does not exist</summary>
            NullState,

            /// <summary> point was added in the update</summary>
            AddState,

            /// <summary> point persisted across the update</summary>
            PersistState,

            /// <summary> point was removed in the update</summary>
            RemoveState
        }

        /// <summary>
        ///  This structure is used to keep track of the best separating axis.
        /// </summary>
        //UPGRADE_NOTE: The access modifier for this class or class field has been changed in order to prevent compilation errors due to the visibility level. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1296'"
        public class EPAxis
        {
            internal enum Type
            {
                Unknown,
                EdgeA,
                EdgeB
            }

            internal Type type;
            internal int Index;
            internal float Separation;
        }

        /// <summary>
        /// This holds polygon B expressed in frame A.
        /// </summary>
        internal class TempPolygon
        {
            internal Vec2[] Vertices = new Vec2[Settings.maxPolygonVertices];
            internal Vec2[] Normals = new Vec2[Settings.maxPolygonVertices];
            internal int count;

            public TempPolygon()
            {
                for (int i = 0; i < Vertices.Length; i++)
                {
                    Vertices[i] = new Vec2();
                    Normals[i] = new Vec2();
                }
            }
        }

        /// <summary>
        ///  Reference face used for clipping
        /// </summary>
        internal class ReferenceFace
        {
            internal int I1;
            internal int I2;
            internal readonly Vec2 V1 = new Vec2();
            internal readonly Vec2 V2 = new Vec2();
            internal readonly Vec2 Normal = new Vec2();

            internal readonly Vec2 SideNormal1 = new Vec2();
            internal float SideOffset1;

            internal readonly Vec2 SideNormal2 = new Vec2();
            internal float SideOffset2;
        }

        /// <summary> 
        /// This class collides and edge and a polygon, taking into account edge adjacency.
        /// </summary>
        internal class EPCollider
        {
            internal enum VertexType
            {
                Isolated,
                Concave,
                Convex
            }

            internal readonly TempPolygon PolygonB = new TempPolygon();

            internal readonly Transform Xf = new Transform();
            internal readonly Vec2 CentroidB = new Vec2();
            internal Vec2 V0 = new Vec2();
            internal Vec2 V1 = new Vec2();
            internal Vec2 V2 = new Vec2();
            internal Vec2 V3 = new Vec2();
            internal readonly Vec2 Normal0 = new Vec2();
            internal readonly Vec2 Normal1 = new Vec2();
            internal readonly Vec2 Normal2 = new Vec2();
            internal readonly Vec2 Normal = new Vec2();

            internal VertexType Type1;
            internal VertexType Type2;

            internal readonly Vec2 LowerLimit = new Vec2();
            internal readonly Vec2 UpperLimit = new Vec2();
            internal float Radius;
            internal bool Front;

            public EPCollider()
            {
                for (int i = 0; i < 2; i++)
                {
                    ie[i] = new ClipVertex();
                    clipPoints1[i] = new ClipVertex();
                    clipPoints2[i] = new ClipVertex();
                }
            }

            private readonly Vec2 edge1 = new Vec2();
            private readonly Vec2 temp = new Vec2();
            private readonly Vec2 edge0 = new Vec2();
            private readonly Vec2 edge2 = new Vec2();
            private readonly ClipVertex[] ie = new ClipVertex[2];
            private readonly ClipVertex[] clipPoints1 = new ClipVertex[2];
            private readonly ClipVertex[] clipPoints2 = new ClipVertex[2];
            private readonly ReferenceFace rf = new ReferenceFace();
            private readonly EPAxis edgeAxis = new EPAxis();
            private readonly EPAxis polygonAxis = new EPAxis();

            public virtual void Collide(Manifold manifold, EdgeShape edgeA, Transform xfA, PolygonShape polygonB, Transform xfB)
            {

                Transform.mulTransToOutUnsafe(xfA, xfB, Xf);
                Transform.mulToOutUnsafe(Xf, polygonB.Centroid, CentroidB);

                V0 = edgeA.Vertex0;
                V1 = edgeA.Vertex1;
                V2 = edgeA.Vertex2;
                V3 = edgeA.Vertex3;

                bool hasVertex0 = edgeA.HasVertex0;
                bool hasVertex3 = edgeA.HasVertex3;

                edge1.set_Renamed(V2).subLocal(V1);
                edge1.normalize();
                Normal1.set_Renamed(edge1.y, -edge1.x);
                float offset1 = Vec2.dot(Normal1, temp.set_Renamed(CentroidB).subLocal(V1));
                float offset0 = 0.0f, offset2 = 0.0f;
                bool convex1 = false, convex2 = false;

                // Is there a preceding edge?
                if (hasVertex0)
                {
                    edge0.set_Renamed(V1).subLocal(V0);
                    edge0.normalize();
                    Normal0.set_Renamed(edge0.y, -edge0.x);
                    convex1 = Vec2.cross(edge0, edge1) >= 0.0f;
                    offset0 = Vec2.dot(Normal0, temp.set_Renamed(CentroidB).subLocal(V0));
                }

                // Is there a following edge?
                if (hasVertex3)
                {
                    edge2.set_Renamed(V3).subLocal(V2);
                    edge2.normalize();
                    Normal2.set_Renamed(edge2.y, -edge2.x);
                    convex2 = Vec2.cross(edge1, edge2) > 0.0f;
                    offset2 = Vec2.dot(Normal2, temp.set_Renamed(CentroidB).subLocal(V2));
                }

                // Determine front or back collision. Determine collision normal limits.
                if (hasVertex0 && hasVertex3)
                {
                    if (convex1 && convex2)
                    {
                        Front = offset0 >= 0.0f || offset1 >= 0.0f || offset2 >= 0.0f;
                        if (Front)
                        {
                            Normal.set_Renamed(Normal1);
                            LowerLimit.set_Renamed(Normal0);
                            UpperLimit.set_Renamed(Normal2);
                        }
                        else
                        {
                            Normal.set_Renamed(Normal1).negateLocal();
                            LowerLimit.set_Renamed(Normal1).negateLocal();
                            UpperLimit.set_Renamed(Normal1).negateLocal();
                        }
                    }
                    else if (convex1)
                    {
                        Front = offset0 >= 0.0f || (offset1 >= 0.0f && offset2 >= 0.0f);
                        if (Front)
                        {
                            Normal.set_Renamed(Normal1);
                            LowerLimit.set_Renamed(Normal0);
                            UpperLimit.set_Renamed(Normal1);
                        }
                        else
                        {
                            Normal.set_Renamed(Normal1).negateLocal();
                            LowerLimit.set_Renamed(Normal2).negateLocal();
                            UpperLimit.set_Renamed(Normal1).negateLocal();
                        }
                    }
                    else if (convex2)
                    {
                        Front = offset2 >= 0.0f || (offset0 >= 0.0f && offset1 >= 0.0f);
                        if (Front)
                        {
                            Normal.set_Renamed(Normal1);
                            LowerLimit.set_Renamed(Normal1);
                            UpperLimit.set_Renamed(Normal2);
                        }
                        else
                        {
                            Normal.set_Renamed(Normal1).negateLocal();
                            LowerLimit.set_Renamed(Normal1).negateLocal();
                            UpperLimit.set_Renamed(Normal0).negateLocal();
                        }
                    }
                    else
                    {
                        Front = offset0 >= 0.0f && offset1 >= 0.0f && offset2 >= 0.0f;
                        if (Front)
                        {
                            Normal.set_Renamed(Normal1);
                            LowerLimit.set_Renamed(Normal1);
                            UpperLimit.set_Renamed(Normal1);
                        }
                        else
                        {
                            Normal.set_Renamed(Normal1).negateLocal();
                            LowerLimit.set_Renamed(Normal2).negateLocal();
                            UpperLimit.set_Renamed(Normal0).negateLocal();
                        }
                    }
                }
                else if (hasVertex0)
                {
                    if (convex1)
                    {
                        Front = offset0 >= 0.0f || offset1 >= 0.0f;
                        if (Front)
                        {
                            Normal.set_Renamed(Normal1);
                            LowerLimit.set_Renamed(Normal0);
                            UpperLimit.set_Renamed(Normal1).negateLocal();
                        }
                        else
                        {
                            Normal.set_Renamed(Normal1).negateLocal();
                            LowerLimit.set_Renamed(Normal1);
                            UpperLimit.set_Renamed(Normal1).negateLocal();
                        }
                    }
                    else
                    {
                        Front = offset0 >= 0.0f && offset1 >= 0.0f;
                        if (Front)
                        {
                            Normal.set_Renamed(Normal1);
                            LowerLimit.set_Renamed(Normal1);
                            UpperLimit.set_Renamed(Normal1).negateLocal();
                        }
                        else
                        {
                            Normal.set_Renamed(Normal1).negateLocal();
                            LowerLimit.set_Renamed(Normal1);
                            UpperLimit.set_Renamed(Normal0).negateLocal();
                        }
                    }
                }
                else if (hasVertex3)
                {
                    if (convex2)
                    {
                        Front = offset1 >= 0.0f || offset2 >= 0.0f;
                        if (Front)
                        {
                            Normal.set_Renamed(Normal1);
                            LowerLimit.set_Renamed(Normal1).negateLocal();
                            UpperLimit.set_Renamed(Normal2);
                        }
                        else
                        {
                            Normal.set_Renamed(Normal1).negateLocal();
                            LowerLimit.set_Renamed(Normal1).negateLocal();
                            UpperLimit.set_Renamed(Normal1);
                        }
                    }
                    else
                    {
                        Front = offset1 >= 0.0f && offset2 >= 0.0f;
                        if (Front)
                        {
                            Normal.set_Renamed(Normal1);
                            LowerLimit.set_Renamed(Normal1).negateLocal();
                            UpperLimit.set_Renamed(Normal1);
                        }
                        else
                        {
                            Normal.set_Renamed(Normal1).negateLocal();
                            LowerLimit.set_Renamed(Normal2).negateLocal();
                            UpperLimit.set_Renamed(Normal1);
                        }
                    }
                }
                else
                {
                    Front = offset1 >= 0.0f;
                    if (Front)
                    {
                        Normal.set_Renamed(Normal1);
                        LowerLimit.set_Renamed(Normal1).negateLocal();
                        UpperLimit.set_Renamed(Normal1).negateLocal();
                    }
                    else
                    {
                        Normal.set_Renamed(Normal1).negateLocal();
                        LowerLimit.set_Renamed(Normal1);
                        UpperLimit.set_Renamed(Normal1);
                    }
                }

                // Get polygonB in frameA
                PolygonB.count = polygonB.VertexCount;
                for (int i = 0; i < polygonB.VertexCount; ++i)
                {
                    Transform.mulToOutUnsafe(Xf, polygonB.Vertices[i], PolygonB.Vertices[i]);
                    Rot.mulToOutUnsafe(Xf.q, polygonB.Normals[i], PolygonB.Normals[i]);
                }

                Radius = 2.0f * Settings.polygonRadius;

                manifold.pointCount = 0;

                ComputeEdgeSeparation(edgeAxis);

                // If no valid normal can be found than this edge should not collide.
                if (edgeAxis.type == EPAxis.Type.Unknown)
                {
                    return;
                }

                if (edgeAxis.Separation > Radius)
                {
                    return;
                }

                ComputePolygonSeparation(polygonAxis);
                if (polygonAxis.type != EPAxis.Type.Unknown && polygonAxis.Separation > Radius)
                {
                    return;
                }

                // Use hysteresis for jitter reduction.
                const float k_relativeTol = 0.98f;
                const float k_absoluteTol = 0.001f;

                EPAxis primaryAxis;
                if (polygonAxis.type == EPAxis.Type.Unknown)
                {
                    primaryAxis = edgeAxis;
                }
                else if (polygonAxis.Separation > k_relativeTol * edgeAxis.Separation + k_absoluteTol)
                {
                    primaryAxis = polygonAxis;
                }
                else
                {
                    primaryAxis = edgeAxis;
                }

                // ClipVertex[] ie = new ClipVertex[2];
                if (primaryAxis.type == EPAxis.Type.EdgeA)
                {
                    manifold.type = Manifold.ManifoldType.FACE_A;

                    // Search for the polygon normal that is most anti-parallel to the edge normal.
                    int bestIndex = 0;
                    float bestValue = Vec2.dot(Normal, PolygonB.Normals[0]);
                    for (int i = 1; i < PolygonB.count; ++i)
                    {
                        float value = Vec2.dot(Normal, PolygonB.Normals[i]);
                        if (value < bestValue)
                        {
                            bestValue = value;
                            bestIndex = i;
                        }
                    }

                    int i1 = bestIndex;
                    int i2 = i1 + 1 < PolygonB.count ? i1 + 1 : 0;

                    ie[0].v.set_Renamed(PolygonB.Vertices[i1]);
                    ie[0].id.IndexA = 0;
                    ie[0].id.IndexB = (sbyte)i1;
                    ie[0].id.TypeA = (sbyte)ContactID.Type.Face;
                    ie[0].id.TypeB = (sbyte)ContactID.Type.Vertex;

                    ie[1].v.set_Renamed(PolygonB.Vertices[i2]);
                    ie[1].id.IndexA = 0;
                    ie[1].id.IndexB = (sbyte)i2;
                    ie[1].id.TypeA = (sbyte)ContactID.Type.Face;
                    ie[1].id.TypeB = (sbyte)ContactID.Type.Vertex;

                    if (Front)
                    {
                        rf.I1 = 0;
                        rf.I2 = 1;
                        rf.V1.set_Renamed(V1);
                        rf.V2.set_Renamed(V2);
                        rf.Normal.set_Renamed(Normal1);
                    }
                    else
                    {
                        rf.I1 = 1;
                        rf.I2 = 0;
                        rf.V1.set_Renamed(V2);
                        rf.V2.set_Renamed(V1);
                        rf.Normal.set_Renamed(Normal1).negateLocal();
                    }
                }
                else
                {
                    manifold.type = Manifold.ManifoldType.FACE_B;

                    ie[0].v.set_Renamed(V1);
                    ie[0].id.IndexA = 0;
                    ie[0].id.IndexB = (sbyte)primaryAxis.Index;
                    ie[0].id.TypeA = (sbyte)ContactID.Type.Vertex;
                    ie[0].id.TypeB = (sbyte)ContactID.Type.Face;

                    ie[1].v.set_Renamed(V2);
                    ie[1].id.IndexA = 0;
                    ie[1].id.IndexB = (sbyte)primaryAxis.Index;
                    ie[1].id.TypeA = (sbyte)ContactID.Type.Vertex;
                    ie[1].id.TypeB = (sbyte)ContactID.Type.Face;

                    rf.I1 = primaryAxis.Index;
                    rf.I2 = rf.I1 + 1 < PolygonB.count ? rf.I1 + 1 : 0;
                    rf.V1.set_Renamed(PolygonB.Vertices[rf.I1]);
                    rf.V2.set_Renamed(PolygonB.Vertices[rf.I2]);
                    rf.Normal.set_Renamed(PolygonB.Normals[rf.I1]);
                }

                rf.SideNormal1.set_Renamed(rf.Normal.y, -rf.Normal.x);
                rf.SideNormal2.set_Renamed(rf.SideNormal1).negateLocal();
                rf.SideOffset1 = Vec2.dot(rf.SideNormal1, rf.V1);
                rf.SideOffset2 = Vec2.dot(rf.SideNormal2, rf.V2);

                // Clip incident edge against extruded edge1 side edges.
                int np;

                // Clip to box side 1
                np = ClipSegmentToLine(clipPoints1, ie, rf.SideNormal1, rf.SideOffset1, rf.I1);

                if (np < Settings.maxManifoldPoints)
                {
                    return;
                }

                // Clip to negative box side 1
                np = ClipSegmentToLine(clipPoints2, clipPoints1, rf.SideNormal2, rf.SideOffset2, rf.I2);

                if (np < Settings.maxManifoldPoints)
                {
                    return;
                }

                // Now clipPoints2 contains the clipped points.
                if (primaryAxis.type == EPAxis.Type.EdgeA)
                {
                    manifold.localNormal.set_Renamed(rf.Normal);
                    manifold.localPoint.set_Renamed(rf.V1);
                }
                else
                {
                    manifold.localNormal.set_Renamed(polygonB.Normals[rf.I1]);
                    manifold.localPoint.set_Renamed(polygonB.Vertices[rf.I1]);
                }

                int pointCount = 0;
                for (int i = 0; i < Settings.maxManifoldPoints; ++i)
                {
                    float separation;

                    separation = Vec2.dot(rf.Normal, temp.set_Renamed(clipPoints2[i].v).subLocal(rf.V1));

                    if (separation <= Radius)
                    {
                        ManifoldPoint cp = manifold.points[pointCount];

                        if (primaryAxis.type == EPAxis.Type.EdgeA)
                        {
                            // cp.localPoint = MulT(m_xf, clipPoints2[i].v);
                            Transform.mulTransToOutUnsafe(Xf, clipPoints2[i].v, cp.localPoint);
                            cp.id.Set(clipPoints2[i].id);
                        }
                        else
                        {
                            cp.localPoint.set_Renamed(clipPoints2[i].v);
                            cp.id.TypeA = clipPoints2[i].id.TypeB;
                            cp.id.TypeB = clipPoints2[i].id.TypeA;
                            cp.id.IndexA = clipPoints2[i].id.IndexB;
                            cp.id.IndexB = clipPoints2[i].id.IndexA;
                        }

                        ++pointCount;
                    }
                }

                manifold.pointCount = pointCount;
            }


            public virtual void ComputeEdgeSeparation(EPAxis axis)
            {
                axis.type = EPAxis.Type.EdgeA;
                axis.Index = Front ? 0 : 1;
                axis.Separation = System.Single.MaxValue;

                for (int i = 0; i < PolygonB.count; ++i)
                {
                    float s = Vec2.dot(Normal, temp.set_Renamed(PolygonB.Vertices[i]).subLocal(V1));
                    if (s < axis.Separation)
                    {
                        axis.Separation = s;
                    }
                }
            }

            private readonly Vec2 perp = new Vec2();
            private readonly Vec2 n = new Vec2();

            public virtual void ComputePolygonSeparation(EPAxis axis)
            {
                axis.type = EPAxis.Type.Unknown;
                axis.Index = -1;
                //UPGRADE_TODO: The equivalent in .NET for field 'java.lang.Float.MIN_VALUE' may return a different value. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1043'"
                axis.Separation = Single.Epsilon;

                perp.set_Renamed(-Normal.y, Normal.x);

                for (int i = 0; i < PolygonB.count; ++i)
                {
                    n.set_Renamed(PolygonB.Normals[i]).negateLocal();

                    float s1 = Vec2.dot(n, temp.set_Renamed(PolygonB.Vertices[i]).subLocal(V1));
                    float s2 = Vec2.dot(n, temp.set_Renamed(PolygonB.Vertices[i]).subLocal(V2));
                    float s = MathUtils.min(s1, s2);

                    if (s > Radius)
                    {
                        // No collision
                        axis.type = EPAxis.Type.EdgeB;
                        axis.Index = i;
                        axis.Separation = s;
                        return;
                    }

                    // Adjacency
                    if (Vec2.dot(n, perp) >= 0.0f)
                    {
                        if (Vec2.dot(temp.set_Renamed(n).subLocal(UpperLimit), Normal) < -Settings.angularSlop)
                        {
                            continue;
                        }
                    }
                    else
                    {
                        if (Vec2.dot(temp.set_Renamed(n).subLocal(LowerLimit), Normal) < -Settings.angularSlop)
                        {
                            continue;
                        }
                    }

                    if (s > axis.Separation)
                    {
                        axis.type = EPAxis.Type.EdgeB;
                        axis.Index = i;
                        axis.Separation = s;
                    }
                }
            }
        }
    }
}