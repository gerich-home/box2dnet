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
        public bool Valid
        {
            get
            {
                float dx = UpperBound.X - LowerBound.X;
                if (dx < 0f)
                {
                    return false;
                }

                float dy = UpperBound.Y - LowerBound.Y;
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
        public Vec2 Center
        {
            get
            {
                Vec2 center = new Vec2(LowerBound);
                center.AddLocal(UpperBound);
                center.MulLocal(.5f);
                return center;
            }
        }

        /// <summary>
        /// Get the extents of the AABB (half-widths).
        /// </summary>
        /// <returns></returns>
        public Vec2 Extents
        {
            get
            {
                Vec2 center = new Vec2(UpperBound);
                center.SubLocal(LowerBound);
                center.MulLocal(.5f);
                return center;
            }
        }

        /// <summary>
        /// Gets the perimeter length
        /// </summary>
        /// <returns></returns>
        public float Perimeter
        {
            get
            {
                return 2.0f * (UpperBound.X - LowerBound.X + UpperBound.Y - LowerBound.Y);
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
            LowerBound.X = v.X;
            LowerBound.Y = v.Y;
            Vec2 v1 = aabb.UpperBound;
            UpperBound.X = v1.X;
            UpperBound.Y = v1.Y;
        }

        public void GetCenterToOut(Vec2 result)
        {
            result.X = (LowerBound.X + UpperBound.X) * .5f;
            result.Y = (LowerBound.Y + UpperBound.Y) * .5f;
        }

        public void GetExtentsToOut(Vec2 result)
        {
            result.X = (UpperBound.X - LowerBound.X) * .5f;
            result.Y = (UpperBound.Y - LowerBound.Y) * .5f; // thanks FDN1
        }

        public void GetVertices(Vec2[] argRay)
        {
            argRay[0].Set(LowerBound);
            argRay[1].Set(LowerBound);
            argRay[1].X += UpperBound.X - LowerBound.X;
            argRay[2].Set(UpperBound);
            argRay[3].Set(UpperBound);
            argRay[3].X -= (UpperBound.X - LowerBound.X);
        }

        /// <summary>
        /// Combine two AABBs into this one.
        /// </summary>
        /// <param name="aabb1"></param>
        /// <param name="aab"></param>
        public void Combine(AABB aabb1, AABB aab)
        {
            LowerBound.X = aabb1.LowerBound.X < aab.LowerBound.X ? aabb1.LowerBound.X : aab.LowerBound.X;
            LowerBound.Y = aabb1.LowerBound.Y < aab.LowerBound.Y ? aabb1.LowerBound.Y : aab.LowerBound.Y;
            UpperBound.X = aabb1.UpperBound.X > aab.UpperBound.X ? aabb1.UpperBound.X : aab.UpperBound.X;
            UpperBound.Y = aabb1.UpperBound.Y > aab.UpperBound.Y ? aabb1.UpperBound.Y : aab.UpperBound.Y;
        }

        /// <summary>
        /// Combines another aabb with this one
        /// </summary>
        /// <param name="aabb"></param>
        public void Combine(AABB aabb)
        {
            LowerBound.X = LowerBound.X < aabb.LowerBound.X ? LowerBound.X : aabb.LowerBound.X;
            LowerBound.Y = LowerBound.Y < aabb.LowerBound.Y ? LowerBound.Y : aabb.LowerBound.Y;
            UpperBound.X = UpperBound.X > aabb.UpperBound.X ? UpperBound.X : aabb.UpperBound.X;
            UpperBound.Y = UpperBound.Y > aabb.UpperBound.Y ? UpperBound.Y : aabb.UpperBound.Y;
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
            return LowerBound.X > aabb.LowerBound.X && LowerBound.Y > aabb.LowerBound.Y && aabb.UpperBound.X > UpperBound.X && aabb.UpperBound.Y > UpperBound.Y;
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

            p.Set(input.P1);
            d.Set(input.P2).SubLocal(input.P1);
            Vec2.AbsToOut(d, absD);

            // x then y
            if (absD.X < Settings.EPSILON)
            {
                // Parallel.
                if (p.X < LowerBound.X || UpperBound.X < p.X)
                {
                    argPool.PushVec2(4);
                    return false;
                }
            }
            else
            {
                float inv_d = 1.0f / d.X;
                float t1 = (LowerBound.X - p.X) * inv_d;
                float t2 = (UpperBound.X - p.X) * inv_d;

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
                    normal.SetZero();
                    normal.X = s;
                    tmin = t1;
                }

                // Pull the max down
                tmax = MathUtils.Min(tmax, t2);

                if (tmin > tmax)
                {
                    argPool.PushVec2(4);
                    return false;
                }
            }

            if (absD.Y < Settings.EPSILON)
            {
                // Parallel.
                if (p.Y < LowerBound.Y || UpperBound.Y < p.Y)
                {
                    argPool.PushVec2(4);
                    return false;
                }
            }
            else
            {
                float inv_d = 1.0f / d.Y;
                float t1 = (LowerBound.Y - p.Y) * inv_d;
                float t2 = (UpperBound.Y - p.Y) * inv_d;

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
                    normal.SetZero();
                    normal.Y = s;
                    tmin = t1;
                }

                // Pull the max down
                tmax = MathUtils.Min(tmax, t2);

                if (tmin > tmax)
                {
                    argPool.PushVec2(4);
                    return false;
                }
            }

            // Does the ray start inside the box?
            // Does the ray intersect beyond the max fraction?
            if (tmin < 0.0f || input.MaxFraction < tmin)
            {
                argPool.PushVec2(4);
                return false;
            }

            // Intersection.
            output.Fraction = tmin;
            output.Normal.X = normal.X;
            output.Normal.Y = normal.Y;
            argPool.PushVec2(4);
            return true;
        }

        public static bool TestOverlap(AABB a, AABB b)
        {
            if (b.LowerBound.X - a.UpperBound.X > 0.0f || b.LowerBound.Y - a.UpperBound.Y > 0.0f)
            {
                return false;
            }

            if (a.LowerBound.X - b.UpperBound.X > 0.0f || a.LowerBound.Y - b.UpperBound.Y > 0.0f)
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