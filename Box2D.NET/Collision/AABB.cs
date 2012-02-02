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
using Box2D.Common;
using Box2D.Pooling;
using Box2D.Pooling.Normal;

namespace Box2D.Collision
{

    /// <summary>An axis-aligned bounding box. </summary>
    public class AABB
    {
        /// <summary>Verify that the bounds are sorted </summary>
        virtual public bool Valid
        {
            get
            {
                float dx = UpperBound.x - LowerBound.x;
                if (dx < 0f)
                {
                    return false;
                }

                float dy = UpperBound.y - LowerBound.y;
                if (dy < 0)
                {
                    return false;
                }
                return LowerBound.Valid && UpperBound.Valid;
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
                Vec2 center = new Vec2(LowerBound);
                center.addLocal(UpperBound);
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
                Vec2 center = new Vec2(UpperBound);
                center.subLocal(LowerBound);
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
                return 2.0f * (UpperBound.x - LowerBound.x + UpperBound.y - LowerBound.y);
            }
        }

        /// <summary>Bottom left vertex of bounding box. </summary>
        public readonly Vec2 LowerBound;
        /// <summary>Top right vertex of bounding box. </summary>
        public readonly Vec2 UpperBound;

        /// <summary> Creates the default object, with vertices at 0,0 and 0,0.</summary>
        public AABB()
        {
            LowerBound = new Vec2();
            UpperBound = new Vec2();
        }

        /// <summary>
        /// Copies from the given object
        /// </summary>
        /// <param name="copy">the object to copy from</param>
        public AABB(AABB copy) :
            this(copy.LowerBound, copy.UpperBound)
        {
        }

        /// <summary>
        /// Creates an AABB object using the given bounding vertices.
        /// </summary>
        /// <param name="lowerVertex">the bottom left vertex of the bounding box</param>
        /// <param name="upperVertex">the top right vertex of the bounding box</param>
        public AABB(Vec2 lowerVertex, Vec2 upperVertex)
        {
            LowerBound = lowerVertex.Clone(); // clone to be safe
            UpperBound = upperVertex.Clone();
        }

        /// <summary>
        /// Sets this object from the given object
        /// </summary>
        /// <param name="aabb">the object to copy from</param>
        public void Set(AABB aabb)
        {
            Vec2 v = aabb.LowerBound;
            LowerBound.x = v.x;
            LowerBound.y = v.y;
            Vec2 v1 = aabb.UpperBound;
            UpperBound.x = v1.x;
            UpperBound.y = v1.y;
        }

        public void GetCenterToOut(Vec2 result)
        {
            result.x = (LowerBound.x + UpperBound.x) * .5f;
            result.y = (LowerBound.y + UpperBound.y) * .5f;
        }

        public void GetExtentsToOut(Vec2 result)
        {
            result.x = (UpperBound.x - LowerBound.x) * .5f;
            result.y = (UpperBound.y - LowerBound.y) * .5f; // thanks FDN1
        }

        public void GetVertices(Vec2[] argRay)
        {
            argRay[0].set_Renamed(LowerBound);
            argRay[1].set_Renamed(LowerBound);
            argRay[1].x += UpperBound.x - LowerBound.x;
            argRay[2].set_Renamed(UpperBound);
            argRay[3].set_Renamed(UpperBound);
            argRay[3].x -= (UpperBound.x - LowerBound.x);
        }

        /// <summary>
        /// Combine two AABBs into this one.
        /// </summary>
        /// <param name="aabb1"></param>
        /// <param name="aab"></param>
        public void Combine(AABB aabb1, AABB aab)
        {
            LowerBound.x = aabb1.LowerBound.x < aab.LowerBound.x ? aabb1.LowerBound.x : aab.LowerBound.x;
            LowerBound.y = aabb1.LowerBound.y < aab.LowerBound.y ? aabb1.LowerBound.y : aab.LowerBound.y;
            UpperBound.x = aabb1.UpperBound.x > aab.UpperBound.x ? aabb1.UpperBound.x : aab.UpperBound.x;
            UpperBound.y = aabb1.UpperBound.y > aab.UpperBound.y ? aabb1.UpperBound.y : aab.UpperBound.y;
        }

        /// <summary>
        /// Combines another aabb with this one
        /// </summary>
        /// <param name="aabb"></param>
        public void Combine(AABB aabb)
        {
            LowerBound.x = LowerBound.x < aabb.LowerBound.x ? LowerBound.x : aabb.LowerBound.x;
            LowerBound.y = LowerBound.y < aabb.LowerBound.y ? LowerBound.y : aabb.LowerBound.y;
            UpperBound.x = UpperBound.x > aabb.UpperBound.x ? UpperBound.x : aabb.UpperBound.x;
            UpperBound.y = UpperBound.y > aabb.UpperBound.y ? UpperBound.y : aabb.UpperBound.y;
        }

        /// <summary>
        /// Does this aabb contain the provided AABB.
        /// </summary>
        /// <returns></returns>
        public bool Contains(AABB aabb)
        {
            /*
            * boolean result = true; result = result && lowerBound.x <= aabb.lowerBound.x; result = result
            * && lowerBound.y <= aabb.lowerBound.y; result = result && aabb.upperBound.x <= upperBound.x;
            * result = result && aabb.upperBound.y <= upperBound.y; return result;
            */
            // djm: faster putting all of them together, as if one is false we leave the logic
            // early
            return LowerBound.x > aabb.LowerBound.x && LowerBound.y > aabb.LowerBound.y && aabb.UpperBound.x > UpperBound.x && aabb.UpperBound.y > UpperBound.y;
        }

        /// <deprecated> please use {@link #raycast(RayCastOutput, RayCastInput, IWorldPool)} for better performance
        /// </deprecated>
        /// <param name="output"></param>
        /// <param name="input"></param>
        /// <returns></returns>
        public bool Raycast(RayCastOutput output, RayCastInput input)
        {
            return Raycast(output, input, new DefaultWorldPool(4, 4));
        }

        /// <summary>
        /// From Real-time Collision Detection, p179.
        /// </summary>
        /// <param name="output"></param>
        /// <param name="input"></param>
        /// <param name="argPool"></param>
        public bool Raycast(RayCastOutput output, RayCastInput input, IWorldPool argPool)
        {
            //UPGRADE_TODO: The equivalent in .NET for field 'java.lang.Float.MIN_VALUE' may return a different value. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1043'"
            float tmin = Single.Epsilon;
            float tmax = Single.MaxValue;

            Vec2 p = argPool.PopVec2();
            Vec2 d = argPool.PopVec2();
            Vec2 absD = argPool.PopVec2();
            Vec2 normal = argPool.PopVec2();

            p.set_Renamed(input.p1);
            d.set_Renamed(input.p2).subLocal(input.p1);
            Vec2.absToOut(d, absD);

            // x then y
            if (absD.x < Settings.EPSILON)
            {
                // Parallel.
                if (p.x < LowerBound.x || UpperBound.x < p.x)
                {
                    argPool.PushVec2(4);
                    return false;
                }
            }
            else
            {
                float inv_d = 1.0f / d.x;
                float t1 = (LowerBound.x - p.x) * inv_d;
                float t2 = (UpperBound.x - p.x) * inv_d;

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
                    argPool.PushVec2(4);
                    return false;
                }
            }

            if (absD.y < Settings.EPSILON)
            {
                // Parallel.
                if (p.y < LowerBound.y || UpperBound.y < p.y)
                {
                    argPool.PushVec2(4);
                    return false;
                }
            }
            else
            {
                float inv_d = 1.0f / d.y;
                float t1 = (LowerBound.y - p.y) * inv_d;
                float t2 = (UpperBound.y - p.y) * inv_d;

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
                    argPool.PushVec2(4);
                    return false;
                }
            }

            // Does the ray start inside the box?
            // Does the ray intersect beyond the max fraction?
            if (tmin < 0.0f || input.maxFraction < tmin)
            {
                argPool.PushVec2(4);
                return false;
            }

            // Intersection.
            output.fraction = tmin;
            output.normal.x = normal.x;
            output.normal.y = normal.y;
            argPool.PushVec2(4);
            return true;
        }

        public static bool TestOverlap(AABB a, AABB b)
        {
            if (b.LowerBound.x - a.UpperBound.x > 0.0f || b.LowerBound.y - a.UpperBound.y > 0.0f)
            {
                return false;
            }

            if (a.LowerBound.x - b.UpperBound.x > 0.0f || a.LowerBound.y - b.UpperBound.y > 0.0f)
            {
                return false;
            }

            return true;
        }

        public override String ToString()
        {
            return string.Format("AABB[{0} . {1}]", LowerBound, UpperBound);
        }
    }
}