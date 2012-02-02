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

using Box2D.Common;

namespace Box2D.Collision
{

    /// <summary>
    /// This is used to compute the current state of a contact manifold.
    /// </summary>
    /// <author>daniel</author>
    public class WorldManifold
    {
        /// <summary> 
        /// World vector pointing from A to B
        /// </summary>
        public readonly Vec2 Normal;

        /// <summary>
        ///  World contact point (point of intersection)
        /// </summary>
        public readonly Vec2[] Points;

        public WorldManifold()
        {
            Normal = new Vec2();
            Points = new Vec2[Settings.maxManifoldPoints];
            for (int i = 0; i < Settings.maxManifoldPoints; i++)
            {
                Points[i] = new Vec2();
            }
        }

        private readonly Vec2 pool3 = new Vec2();
        private readonly Vec2 pool4 = new Vec2();

        public void Initialize(Manifold manifold, Transform xfA, float radiusA, Transform xfB, float radiusB)
        {
            if (manifold.PointCount == 0)
            {
                return;
            }

            switch (manifold.Type)
            {

                case Manifold.ManifoldType.Circles:
                    {
                        // final Vec2 pointA = pool3;
                        // final Vec2 pointB = pool4;
                        //
                        // normal.set(1, 0);
                        // Transform.mulToOut(xfA, manifold.localPoint, pointA);
                        // Transform.mulToOut(xfB, manifold.points[0].localPoint, pointB);
                        //
                        // if (MathUtils.distanceSquared(pointA, pointB) > Settings.EPSILON * Settings.EPSILON) {
                        // normal.set(pointB).subLocal(pointA);
                        // normal.normalize();
                        // }
                        //
                        // cA.set(normal).mulLocal(radiusA).addLocal(pointA);
                        // cB.set(normal).mulLocal(radiusB).subLocal(pointB).negateLocal();
                        // points[0].set(cA).addLocal(cB).mulLocal(0.5f);
                        Vec2 pointA = pool3;
                        Vec2 pointB = pool4;

                        Normal.x = 1;
                        Normal.y = 0;
                        // pointA.x = xfA.p.x + xfA.q.ex.x * manifold.localPoint.x + xfA.q.ey.x *
                        // manifold.localPoint.y;
                        // pointA.y = xfA.p.y + xfA.q.ex.y * manifold.localPoint.x + xfA.q.ey.y *
                        // manifold.localPoint.y;
                        // pointB.x = xfB.p.x + xfB.q.ex.x * manifold.points[0].localPoint.x + xfB.q.ey.x *
                        // manifold.points[0].localPoint.y;
                        // pointB.y = xfB.p.y + xfB.q.ex.y * manifold.points[0].localPoint.x + xfB.q.ey.y *
                        // manifold.points[0].localPoint.y;
                        Transform.mulToOut(xfA, manifold.LocalPoint, pointA);
                        Transform.mulToOut(xfB, manifold.Points[0].LocalPoint, pointB);

                        if (MathUtils.distanceSquared(pointA, pointB) > Settings.EPSILON * Settings.EPSILON)
                        {
                            Normal.x = pointB.x - pointA.x;
                            Normal.y = pointB.y - pointA.y;
                            Normal.normalize();
                        }

                        float cAx = Normal.x * radiusA + pointA.x;
                        float cAy = Normal.y * radiusA + pointA.y;

                        float cBx = (-Normal.x) * radiusB + pointB.x;
                        float cBy = (-Normal.y) * radiusB + pointB.y;

                        Points[0].x = (cAx + cBx) * .5f;
                        Points[0].y = (cAy + cBy) * .5f;
                    }
                    break;

                case Manifold.ManifoldType.FaceA:
                    {
                        Vec2 planePoint = pool3;

                        Rot.mulToOutUnsafe(xfA.q, manifold.LocalNormal, Normal);
                        Transform.mulToOut(xfA, manifold.LocalPoint, planePoint);

                        Vec2 clipPoint = pool4;

                        for (int i = 0; i < manifold.PointCount; i++)
                        {
                            // b2Vec2 clipPoint = b2Mul(xfB, manifold->points[i].localPoint);
                            // b2Vec2 cA = clipPoint + (radiusA - b2Dot(clipPoint - planePoint,
                            // normal)) * normal;
                            // b2Vec2 cB = clipPoint - radiusB * normal;
                            // points[i] = 0.5f * (cA + cB);
                            Transform.mulToOut(xfB, manifold.Points[i].LocalPoint, clipPoint);
                            // use cA as temporary for now
                            // cA.set(clipPoint).subLocal(planePoint);
                            // float scalar = radiusA - Vec2.dot(cA, normal);
                            // cA.set(normal).mulLocal(scalar).addLocal(clipPoint);
                            // cB.set(normal).mulLocal(radiusB).subLocal(clipPoint).negateLocal();
                            // points[i].set(cA).addLocal(cB).mulLocal(0.5f);

                            float scalar = radiusA - ((clipPoint.x - planePoint.x) * Normal.x + (clipPoint.y - planePoint.y) * Normal.y);

                            float cAx = Normal.x * scalar + clipPoint.x;
                            float cAy = Normal.y * scalar + clipPoint.y;

                            float cBx = (-Normal.x) * radiusB + clipPoint.x;
                            float cBy = (-Normal.y) * radiusB + clipPoint.y;

                            Points[i].x = (cAx + cBx) * .5f;
                            Points[i].y = (cAy + cBy) * .5f;
                        }
                    }
                    break;

                case Manifold.ManifoldType.FaceB:
                    Vec2 planePoint2 = pool3;
                    Rot.mulToOutUnsafe(xfB.q, manifold.LocalNormal, Normal);
                    Transform.mulToOut(xfB, manifold.LocalPoint, planePoint2);

                    // final Mat22 R = xfB.q;
                    // normal.x = R.ex.x * manifold.localNormal.x + R.ey.x * manifold.localNormal.y;
                    // normal.y = R.ex.y * manifold.localNormal.x + R.ey.y * manifold.localNormal.y;
                    // final Vec2 v = manifold.localPoint;
                    // planePoint.x = xfB.p.x + xfB.q.ex.x * v.x + xfB.q.ey.x * v.y;
                    // planePoint.y = xfB.p.y + xfB.q.ex.y * v.x + xfB.q.ey.y * v.y;

                    Vec2 clipPoint2 = pool4;

                    for (int i = 0; i < manifold.PointCount; i++)
                    {
                        // b2Vec2 clipPoint = b2Mul(xfA, manifold->points[i].localPoint);
                        // b2Vec2 cB = clipPoint + (radiusB - b2Dot(clipPoint - planePoint,
                        // normal)) * normal;
                        // b2Vec2 cA = clipPoint - radiusA * normal;
                        // points[i] = 0.5f * (cA + cB);

                        Transform.mulToOut(xfA, manifold.Points[i].LocalPoint, clipPoint2);
                        // cB.set(clipPoint).subLocal(planePoint);
                        // float scalar = radiusB - Vec2.dot(cB, normal);
                        // cB.set(normal).mulLocal(scalar).addLocal(clipPoint);
                        // cA.set(normal).mulLocal(radiusA).subLocal(clipPoint).negateLocal();
                        // points[i].set(cA).addLocal(cB).mulLocal(0.5f);

                        // points[i] = 0.5f * (cA + cB);

                        //
                        // clipPoint.x = xfA.p.x + xfA.q.ex.x * manifold.points[i].localPoint.x + xfA.q.ey.x *
                        // manifold.points[i].localPoint.y;
                        // clipPoint.y = xfA.p.y + xfA.q.ex.y * manifold.points[i].localPoint.x + xfA.q.ey.y *
                        // manifold.points[i].localPoint.y;

                        float scalar = radiusB - ((clipPoint2.x - planePoint2.x) * Normal.x + (clipPoint2.y - planePoint2.y) * Normal.y);

                        float cBx = Normal.x * scalar + clipPoint2.x;
                        float cBy = Normal.y * scalar + clipPoint2.y;

                        float cAx = (-Normal.x) * radiusA + clipPoint2.x;
                        float cAy = (-Normal.y) * radiusA + clipPoint2.y;

                        Points[i].x = (cAx + cBx) * .5f;
                        Points[i].y = (cAy + cBy) * .5f;
                    }
                    // Ensure normal points from A to B.
                    Normal.x = -Normal.x;
                    Normal.y = -Normal.y;
                    break;
            }
        }
    }
}