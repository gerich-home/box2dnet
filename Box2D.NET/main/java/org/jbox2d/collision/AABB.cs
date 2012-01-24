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
using MathUtils = org.jbox2d.common.MathUtils;
using Settings = org.jbox2d.common.Settings;
using Vec2 = org.jbox2d.common.Vec2;
using IWorldPool = org.jbox2d.pooling.IWorldPool;
using DefaultWorldPool = org.jbox2d.pooling.normal.DefaultWorldPool;

namespace org.jbox2d.collision
{

    /// <summary>An axis-aligned bounding box. </summary>
    public class AABB
    {
        /// <summary>Verify that the bounds are sorted </summary>
        virtual public bool Valid
        {
            get
            {
                float dx = upperBound.x - lowerBound.x;
                if (dx < 0f)
                {
                    return false;
                }

                float dy = upperBound.y - lowerBound.y;
                if (dy < 0)
                {
                    return false;
                }
                return lowerBound.Valid && upperBound.Valid;
            }

        }

        /// <summary>
        /// Get the center of the AABB
        /// </summary>
        /// <returns></returns>
        virtual public Vec2 Center
        {
            get
            {
                Vec2 center = new Vec2(lowerBound);
                center.addLocal(upperBound);
                center.mulLocal(.5f);
                return center;
            }

        }

        /// <summary>
        /// Get the extents of the AABB (half-widths).
        /// </summary>
        /// <returns></returns>
        virtual public Vec2 Extents
        {
            get
            {
                Vec2 center = new Vec2(upperBound);
                center.subLocal(lowerBound);
                center.mulLocal(.5f);
                return center;
            }

        }

        /// <summary>
        /// Gets the perimeter length
        /// </summary>
        /// <returns></returns>
        virtual public float Perimeter
        {
            get
            {
                return 2.0f * (upperBound.x - lowerBound.x + upperBound.y - lowerBound.y);
            }

        }

        /// <summary>Bottom left vertex of bounding box. </summary>
        public readonly Vec2 lowerBound;
        /// <summary>Top right vertex of bounding box. </summary>
        public readonly Vec2 upperBound;

        /// <summary> Creates the default object, with vertices at 0,0 and 0,0.</summary>
        public AABB()
        {
            lowerBound = new Vec2();
            upperBound = new Vec2();
        }

        /// <summary>
        /// Copies from the given object
        /// </summary>
        /// <param name="copy">the object to copy from</param>
        public AABB(AABB copy) :
            this(copy.lowerBound, copy.upperBound)
        {
        }

        /// <summary>
        /// Creates an AABB object using the given bounding vertices.
        /// </summary>
        /// <param name="lowerVertex">the bottom left vertex of the bounding box</param>
        /// <param name="maxVertex">the top right vertex of the bounding box</param>
        public AABB(Vec2 lowerVertex, Vec2 upperVertex)
        {
            this.lowerBound = lowerVertex.Clone() as Vec2; // clone to be safe
            this.upperBound = upperVertex.Clone() as Vec2;
        }

        /// <summary>
        /// Sets this object from the given object
        /// </summary>
        /// <param name="aabb">the object to copy from</param>
        public void set_Renamed(AABB aabb)
        {
            Vec2 v = aabb.lowerBound;
            lowerBound.x = v.x;
            lowerBound.y = v.y;
            Vec2 v1 = aabb.upperBound;
            upperBound.x = v1.x;
            upperBound.y = v1.y;
        }

        public void getCenterToOut(Vec2 out_Renamed)
        {
            out_Renamed.x = (lowerBound.x + upperBound.x) * .5f;
            out_Renamed.y = (lowerBound.y + upperBound.y) * .5f;
        }

        public void getExtentsToOut(Vec2 out_Renamed)
        {
            out_Renamed.x = (upperBound.x - lowerBound.x) * .5f;
            out_Renamed.y = (upperBound.y - lowerBound.y) * .5f; // thanks FDN1
        }

        public void getVertices(Vec2[] argRay)
        {
            argRay[0].set_Renamed(lowerBound);
            argRay[1].set_Renamed(lowerBound);
            argRay[1].x += upperBound.x - lowerBound.x;
            argRay[2].set_Renamed(upperBound);
            argRay[3].set_Renamed(upperBound);
            argRay[3].x -= (upperBound.x - lowerBound.x);
        }

        /// <summary>
        /// Combine two AABBs into this one.
        /// </summary>
        /// <param name="aabb1"></param>
        /// <param name="aab"></param>
        public void combine(AABB aabb1, AABB aab)
        {
            lowerBound.x = aabb1.lowerBound.x < aab.lowerBound.x ? aabb1.lowerBound.x : aab.lowerBound.x;
            lowerBound.y = aabb1.lowerBound.y < aab.lowerBound.y ? aabb1.lowerBound.y : aab.lowerBound.y;
            upperBound.x = aabb1.upperBound.x > aab.upperBound.x ? aabb1.upperBound.x : aab.upperBound.x;
            upperBound.y = aabb1.upperBound.y > aab.upperBound.y ? aabb1.upperBound.y : aab.upperBound.y;
        }

        /// <summary>
        /// Combines another aabb with this one
        /// </summary>
        /// <param name="aabb"></param>
        public void combine(AABB aabb)
        {
            lowerBound.x = lowerBound.x < aabb.lowerBound.x ? lowerBound.x : aabb.lowerBound.x;
            lowerBound.y = lowerBound.y < aabb.lowerBound.y ? lowerBound.y : aabb.lowerBound.y;
            upperBound.x = upperBound.x > aabb.upperBound.x ? upperBound.x : aabb.upperBound.x;
            upperBound.y = upperBound.y > aabb.upperBound.y ? upperBound.y : aabb.upperBound.y;
        }

        /// <summary>
        /// Does this aabb contain the provided AABB.
        /// </summary>
        /// <returns></returns>
        public bool contains(AABB aabb)
        {
            /*
            * boolean result = true; result = result && lowerBound.x <= aabb.lowerBound.x; result = result
            * && lowerBound.y <= aabb.lowerBound.y; result = result && aabb.upperBound.x <= upperBound.x;
            * result = result && aabb.upperBound.y <= upperBound.y; return result;
            */
            // djm: faster putting all of them together, as if one is false we leave the logic
            // early
            return lowerBound.x > aabb.lowerBound.x && lowerBound.y > aabb.lowerBound.y && aabb.upperBound.x > upperBound.x && aabb.upperBound.y > upperBound.y;
        }

        /// <deprecated> please use {@link #raycast(RayCastOutput, RayCastInput, IWorldPool)} for better performance
        /// </deprecated>
        /// <param name="output"></param>
        /// <param name="input"></param>
        /// <returns></returns>
        public bool raycast(RayCastOutput output, RayCastInput input)
        {
            return raycast(output, input, new DefaultWorldPool(4, 4));
        }

        /// <summary>
        /// From Real-time Collision Detection, p179.
        /// </summary>
        /// <param name="output"></param>
        /// <param name="input"></param>
        public bool raycast(RayCastOutput output, RayCastInput input, IWorldPool argPool)
        {
            //UPGRADE_TODO: The equivalent in .NET for field 'java.lang.Float.MIN_VALUE' may return a different value. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1043'"
            float tmin = Single.Epsilon;
            float tmax = Single.MaxValue;

            Vec2 p = argPool.popVec2();
            Vec2 d = argPool.popVec2();
            Vec2 absD = argPool.popVec2();
            Vec2 normal = argPool.popVec2();

            p.set_Renamed(input.p1);
            d.set_Renamed(input.p2).subLocal(input.p1);
            Vec2.absToOut(d, absD);

            // x then y
            if (absD.x < Settings.EPSILON)
            {
                // Parallel.
                if (p.x < lowerBound.x || upperBound.x < p.x)
                {
                    argPool.pushVec2(4);
                    return false;
                }
            }
            else
            {
                float inv_d = 1.0f / d.x;
                float t1 = (lowerBound.x - p.x) * inv_d;
                float t2 = (upperBound.x - p.x) * inv_d;

                // Sign of the normal vector.
                float s = -1.0f;

                if (t1 > t2)
                {
                    float temp = t1;
                    t1 = t2;
                    t2 = temp;
                    s = 1.0f;
                }

                // Push the min up
                if (t1 > tmin)
                {
                    normal.setZero();
                    normal.x = s;
                    tmin = t1;
                }

                // Pull the max down
                tmax = MathUtils.min(tmax, t2);

                if (tmin > tmax)
                {
                    argPool.pushVec2(4);
                    return false;
                }
            }

            if (absD.y < Settings.EPSILON)
            {
                // Parallel.
                if (p.y < lowerBound.y || upperBound.y < p.y)
                {
                    argPool.pushVec2(4);
                    return false;
                }
            }
            else
            {
                float inv_d = 1.0f / d.y;
                float t1 = (lowerBound.y - p.y) * inv_d;
                float t2 = (upperBound.y - p.y) * inv_d;

                // Sign of the normal vector.
                float s = -1.0f;

                if (t1 > t2)
                {
                    float temp = t1;
                    t1 = t2;
                    t2 = temp;
                    s = 1.0f;
                }

                // Push the min up
                if (t1 > tmin)
                {
                    normal.setZero();
                    normal.y = s;
                    tmin = t1;
                }

                // Pull the max down
                tmax = MathUtils.min(tmax, t2);

                if (tmin > tmax)
                {
                    argPool.pushVec2(4);
                    return false;
                }
            }

            // Does the ray start inside the box?
            // Does the ray intersect beyond the max fraction?
            if (tmin < 0.0f || input.maxFraction < tmin)
            {
                argPool.pushVec2(4);
                return false;
            }

            // Intersection.
            output.fraction = tmin;
            output.normal.x = normal.x;
            output.normal.y = normal.y;
            argPool.pushVec2(4);
            return true;
        }

        public static bool testOverlap(AABB a, AABB b)
        {
            if (b.lowerBound.x - a.upperBound.x > 0.0f || b.lowerBound.y - a.upperBound.y > 0.0f)
            {
                return false;
            }

            if (a.lowerBound.x - b.upperBound.x > 0.0f || a.lowerBound.y - b.upperBound.y > 0.0f)
            {
                return false;
            }

            return true;
        }

        public override String ToString()
        {
            return "AABB[" + lowerBound + " . " + upperBound + "]";
        }
    }
}