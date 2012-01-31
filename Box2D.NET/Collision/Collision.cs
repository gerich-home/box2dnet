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
        public static readonly int NULL_FEATURE = Int32.MaxValue;

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
        /// <param name="shapeB"></param>
        /// <param name="xfA"></param>
        /// <param name="xfB"></param>
        /// <returns></returns>
        public bool testOverlap(Shape shapeA, int indexA, Shape shapeB, int indexB, Transform xfA, Transform xfB)
        {
            input.proxyA.set_Renamed(shapeA, indexA);
            input.proxyB.set_Renamed(shapeB, indexB);
            input.transformA.set_Renamed(xfA);
            input.transformB.set_Renamed(xfB);
            input.useRadii = true;

            cache.count = 0;

            pool.getDistance().distance(output, cache, input);
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
        public static void getPointStates(PointState[] state1, PointState[] state2, Manifold manifold1, Manifold manifold2)
        {

            for (int i = 0; i < Settings.maxManifoldPoints; i++)
            {
                state1[i] = PointState.NULL_STATE;
                state2[i] = PointState.NULL_STATE;
            }

            // Detect persists and removes.
            for (int i = 0; i < manifold1.pointCount; i++)
            {
                ContactID id = manifold1.points[i].id;

                state1[i] = PointState.REMOVE_STATE;

                for (int j = 0; j < manifold2.pointCount; j++)
                {
                    if (manifold2.points[j].id.isEqual(id))
                    {
                        state1[i] = PointState.PERSIST_STATE;
                        break;
                    }
                }
            }

            // Detect persists and adds
            for (int i = 0; i < manifold2.pointCount; i++)
            {
                ContactID id = manifold2.points[i].id;

                state2[i] = PointState.ADD_STATE;

                for (int j = 0; j < manifold1.pointCount; j++)
                {
                    if (manifold1.points[j].id.isEqual(id))
                    {
                        state2[i] = PointState.PERSIST_STATE;
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
        /// <returns></returns>
        public static int clipSegmentToLine(ClipVertex[] vOut, ClipVertex[] vIn, Vec2 normal, float offset, int vertexIndexA)
        {

            // Start with no output points
            int numOut = 0;

            // Calculate the distance of end points to the line
            float distance0 = Vec2.dot(normal, vIn[0].v) - offset;
            float distance1 = Vec2.dot(normal, vIn[1].v) - offset;

            // If the points are behind the plane
            if (distance0 <= 0.0f)
            {
                vOut[numOut++].set_Renamed(vIn[0]);
            }
            if (distance1 <= 0.0f)
            {
                vOut[numOut++].set_Renamed(vIn[1]);
            }

            // If the points are on different sides of the plane
            if (distance0 * distance1 < 0.0f)
            {
                // Find intersection point of edge and plane
                float interp = distance0 / (distance0 - distance1);
                // vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
                vOut[numOut].v.set_Renamed(vIn[1].v).subLocal(vIn[0].v).mulLocal(interp).addLocal(vIn[0].v);

                // VertexA is hitting edgeB.
                vOut[numOut].id.indexA = (sbyte)vertexIndexA;
                vOut[numOut].id.indexB = vIn[0].id.indexB;
                vOut[numOut].id.typeA = (sbyte)ContactID.Type.VERTEX;
                vOut[numOut].id.typeB = (sbyte)ContactID.Type.FACE;
                ++numOut;
            }

            return numOut;
        }

        // #### COLLISION STUFF (not from collision.h or collision.cpp) ####

        // djm pooling
        private static Vec2 pA = new Vec2();
        private static Vec2 pB = new Vec2();
        private static Vec2 d = new Vec2();

        /// <summary>
        /// Compute the collision manifold between two circles.
        /// </summary>
        /// <param name="manifold"></param>
        /// <param name="circle1"></param>
        /// <param name="xfA"></param>
        /// <param name="circle2"></param>
        /// <param name="xfB"></param>
        public void collideCircles(Manifold manifold, CircleShape circle1, Transform xfA, CircleShape circle2, Transform xfB)
        {
            manifold.pointCount = 0;

            // before inline:
            Transform.mulToOut(xfA, circle1.m_p, pA);
            Transform.mulToOut(xfB, circle2.m_p, pB);
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

            float radius = circle1.m_radius + circle2.m_radius;
            if (distSqr > radius * radius)
            {
                return;
            }

            manifold.type = Manifold.ManifoldType.CIRCLES;
            manifold.localPoint.set_Renamed(circle1.m_p);
            manifold.localNormal.setZero();
            manifold.pointCount = 1;

            manifold.points[0].localPoint.set_Renamed(circle2.m_p);
            manifold.points[0].id.zero();
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
        public void collidePolygonAndCircle(Manifold manifold, PolygonShape polygon, Transform xfA, CircleShape circle, Transform xfB)
        {
            manifold.pointCount = 0;
            Vec2 v = circle.m_p;

            // Compute circle position in the frame of the polygon.
            // before inline:
            Transform.mulToOut(xfB, circle.m_p, c);
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
            float radius = polygon.m_radius + circle.m_radius;
            int vertexCount = polygon.m_count;

            Vec2[] vertices = polygon.m_vertices;
            Vec2[] normals = polygon.m_normals;

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
                mpoint.localPoint.x = circle.m_p.x;
                mpoint.localPoint.y = circle.m_p.y;
                mpoint.id.zero();
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
                manifold.points[0].localPoint.set_Renamed(circle.m_p);
                manifold.points[0].id.zero();
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
                manifold.points[0].localPoint.set_Renamed(circle.m_p);
                manifold.points[0].id.zero();
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
                manifold.points[0].localPoint.set_Renamed(circle.m_p);
                manifold.points[0].id.zero();
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
        public float edgeSeparation(PolygonShape poly1, Transform xf1, int edge1, PolygonShape poly2, Transform xf2)
        {
            int count1 = poly1.m_count;
            Vec2[] vertices1 = poly1.m_vertices;
            Vec2[] normals1 = poly1.m_normals;

            int count2 = poly2.m_count;
            Vec2[] vertices2 = poly2.m_vertices;

            Debug.Assert(0 <= edge1 && edge1 < count1);
            // Convert normal from poly1's frame into poly2's frame.
            // before inline:
            // Vec2 normal1World = Mul(xf1.R, normals1[edge1]);
            Rot.mulToOutUnsafe(xf1.q, normals1[edge1], normal1World);
            // Vec2 normal1 = MulT(xf2.R, normal1World);
            Rot.mulTransUnsafe(xf2.q, normal1World, normal1);
            float normal1x = normal1.x;
            float normal1y = normal1.y;
            float normal1Worldx = normal1World.x;
            float normal1Worldy = normal1World.y;
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
        /// <param name="edgeIndex"></param>
        /// <param name="poly1"></param>
        /// <param name="xf1"></param>
        /// <param name="poly2"></param>
        /// <param name="xf2"></param>
        /// <returns></returns>
        public void findMaxSeparation(EdgeResults results, PolygonShape poly1, Transform xf1, PolygonShape poly2, Transform xf2)
        {
            int count1 = poly1.m_count;
            Vec2[] normals1 = poly1.m_normals;
            Vec2 v = poly2.m_centroid;

            // Vector pointing from the centroid of poly1 to the centroid of poly2.
            // before inline:
            Transform.mulToOutUnsafe(xf2, poly2.m_centroid, d);
            Transform.mulToOutUnsafe(xf1, poly1.m_centroid, temp);
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
            float s = edgeSeparation(poly1, xf1, edge, poly2, xf2);

            // Check the separation for the previous edge normal.
            int prevEdge = edge - 1 >= 0 ? edge - 1 : count1 - 1;
            float sPrev = edgeSeparation(poly1, xf1, prevEdge, poly2, xf2);

            // Check the separation for the next edge normal.
            int nextEdge = edge + 1 < count1 ? edge + 1 : 0;
            float sNext = edgeSeparation(poly1, xf1, nextEdge, poly2, xf2);

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
                results.edgeIndex = edge;
                results.separation = s;
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

                s = edgeSeparation(poly1, xf1, edge, poly2, xf2);

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

            results.edgeIndex = bestEdge;
            results.separation = bestSeparation;
        }

        // djm pooling from above
        public void findIncidentEdge(ClipVertex[] c, PolygonShape poly1, Transform xf1, int edge1, PolygonShape poly2, Transform xf2)
        {
            int count1 = poly1.m_count;
            Vec2[] normals1 = poly1.m_normals;

            int count2 = poly2.m_count;
            Vec2[] vertices2 = poly2.m_vertices;
            Vec2[] normals2 = poly2.m_normals;

            Debug.Assert(0 <= edge1 && edge1 < count1);

            // Get the normal of the reference edge in poly2's frame.
            Rot.mulToOutUnsafe(xf1.q, normals1[edge1], normal1); // temporary
            // b2Vec2 normal1 = b2MulT(xf2.R, b2Mul(xf1.R, normals1[edge1]));
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
            c[0].id.indexA = (sbyte)edge1;
            c[0].id.indexB = (sbyte)i1;
            c[0].id.typeA = (sbyte)ContactID.Type.FACE;
            c[0].id.typeB = (sbyte)ContactID.Type.VERTEX;

            Transform.mulToOutUnsafe(xf2, vertices2[i2], c[1].v); // = Mul(xf2, vertices2[i2]);
            c[1].id.indexA = (sbyte)edge1;
            c[1].id.indexB = (sbyte)i2;
            c[1].id.typeA = (sbyte)ContactID.Type.FACE;
            c[1].id.typeB = (sbyte)ContactID.Type.VERTEX;
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
        /// <param name="polygon1"></param>
        /// <param name="xf1"></param>
        /// <param name="polygon2"></param>
        /// <param name="xf2"></param>
        public void collidePolygons(Manifold manifold, PolygonShape polyA, Transform xfA, PolygonShape polyB, Transform xfB)
        {
            // Find edge normal of max separation on A - return if separating axis is found
            // Find edge normal of max separation on B - return if separation axis is found
            // Choose reference edge as min(minA, minB)
            // Find incident edge
            // Clip

            // The normal points from 1 to 2

            manifold.pointCount = 0;
            float totalRadius = polyA.m_radius + polyB.m_radius;

            findMaxSeparation(results1, polyA, xfA, polyB, xfB);
            if (results1.separation > totalRadius)
            {
                return;
            }

            findMaxSeparation(results2, polyB, xfB, polyA, xfA);
            if (results2.separation > totalRadius)
            {
                return;
            }

            PolygonShape poly1; // reference polygon
            PolygonShape poly2; // incident polygon
            Transform xf1, xf2;
            int edge1; // reference edge
            bool flip;
            float k_relativeTol = 0.98f;
            float k_absoluteTol = 0.001f;

            if (results2.separation > k_relativeTol * results1.separation + k_absoluteTol)
            {
                poly1 = polyB;
                poly2 = polyA;
                xf1 = xfB;
                xf2 = xfA;
                edge1 = results2.edgeIndex;
                manifold.type = Manifold.ManifoldType.FACE_B;
                flip = true;
            }
            else
            {
                poly1 = polyA;
                poly2 = polyB;
                xf1 = xfA;
                xf2 = xfB;
                edge1 = results1.edgeIndex;
                manifold.type = Manifold.ManifoldType.FACE_A;
                flip = false;
            }

            findIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

            int count1 = poly1.m_count;
            Vec2[] vertices1 = poly1.m_vertices;

            int iv1 = edge1;
            int iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;
            v11.set_Renamed(vertices1[iv1]);
            v12.set_Renamed(vertices1[iv2]);
            localTangent.set_Renamed(v12).subLocal(v11);
            localTangent.normalize();

            Vec2.crossToOutUnsafe(localTangent, 1f, localNormal); // Vec2 localNormal = Cross(dv,
            // 1.0f);

            planePoint.set_Renamed(v11).addLocal(v12).mulLocal(.5f); // Vec2 planePoint = 0.5f * (v11
            // + v12);

            Rot.mulToOutUnsafe(xf1.q, localTangent, tangent); // Vec2 sideNormal = Mul(xf1.R, v12
            // - v11);
            Vec2.crossToOutUnsafe(tangent, 1f, normal); // Vec2 frontNormal = Cross(sideNormal,
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
            np = clipSegmentToLine(clipPoints1, incidentEdge, tangent, sideOffset1, iv1);
            tangent.negateLocal();

            if (np < 2)
            {
                return;
            }

            // Clip to negative box side 1
            np = clipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2);

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
                    cp.id.set_Renamed(clipPoints2[i].id);
                    if (flip)
                    {
                        // Swap features
                        cp.id.flip();
                    }
                    ++pointCount;
                }
            }

            manifold.pointCount = pointCount;
        }

        /// <summary> Java-specific class for returning edge results</summary>
        public class EdgeResults
        {
            public float separation;
            public int edgeIndex;
        }

        /// <summary> Used for computing contact manifolds.</summary>
        public class ClipVertex
        {
            public readonly Vec2 v;
            public readonly ContactID id;

            public ClipVertex()
            {
                v = new Vec2();
                id = new ContactID();
            }

            public virtual void set_Renamed(ClipVertex cv)
            {
                v.set_Renamed(cv.v);
                id.set_Renamed(cv.id);
            }
        }

        /// <summary>
        /// This is used for determining the state of contact points.
        /// </summary>
        /// <author>Daniel Murphy</author>
        public enum PointState
        {
            /// <summary> point does not exist</summary>
            NULL_STATE,
            /// <summary> point was added in the update</summary>
            ADD_STATE,
            /// <summary> point persisted across the update</summary>
            PERSIST_STATE,
            /// <summary> point was removed in the update</summary>
            REMOVE_STATE
        }
    }
}