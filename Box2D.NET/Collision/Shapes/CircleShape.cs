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

namespace Box2D.Collision.Shapes
{
    /// <summary>
    /// A circle shape.
    /// </summary>
    public class CircleShape : Shape
    {
        public readonly Vec2 P;

        private readonly Vec2 pool1 = new Vec2();
        private readonly Vec2 pool2 = new Vec2();
        private readonly Vec2 pool3 = new Vec2();

        /// <summary>
        /// this is used internally, instead use {@link Body#createShape(ShapeDef)} with a
        /// {@link CircleDef}
        /// </summary>
        /// <seealso>
        ///   <cref>Body.CreateShape(ShapeDef)</cref>
        /// </seealso>
        /// <seealso>
        ///   <cref>CircleDef</cref>
        /// </seealso>
        public CircleShape() :
            base(ShapeType.Circle)
        {
            P = new Vec2();
            Radius = 0;
        }


        public override Shape Clone()
        {
            CircleShape shape = new CircleShape();
            shape.P.Set(P);
            shape.Radius = Radius;
            return shape;
        }

        override public int ChildCount
        {
            get
            {
                return 1;
            }
        }

        /// <summary>
        /// Get the supporting vertex index in the given direction.
        /// </summary>
        /// <param name="d"></param>
        /// <returns></returns>
        public int GetSupport(Vec2 d)
        {
            return 0;
        }

        /// <summary>
        /// Get the supporting vertex in the given direction.
        /// </summary>
        /// <param name="d"></param>
        /// <returns></returns>
        public Vec2 GetSupportVertex(Vec2 d)
        {
            return P;
        }

        /// <summary>
        /// Get the vertex count.
        /// </summary>
        /// <returns></returns>
        public int VertexCount
        {
            get
            {
                return 1;
            }
        }

        /// <summary>
        /// Get a vertex by index.
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public Vec2 GetVertex(int index)
        {
            Debug.Assert(index == 0);
            return P;
        }

        public override bool TestPoint(Transform transform, Vec2 p)
        {
            Vec2 center = pool1;
            Rot.MulToOutUnsafe(transform.q, P, center);
            center.AddLocal(transform.p);

            Vec2 d = center.SubLocal(p).NegateLocal();
            return Vec2.Dot(d, d) <= Radius * Radius;
        }

        // Collision Detection in Interactive 3D Environments by Gino van den Bergen
        // From Section 3.1.2
        // x = s + a * r
        // norm(x) = radius
        public override bool Raycast(RayCastOutput output, RayCastInput input, Transform transform, int childIndex)
        {
            Vec2 position = pool1;
            Vec2 s = pool2;
            Vec2 r = pool3;

            Rot.MulToOutUnsafe(transform.q, P, position);
            position.AddLocal(transform.p);
            s.Set(input.P1).SubLocal(position);
            float b = Vec2.Dot(s, s) - Radius * Radius;

            // Solve quadratic equation.
            r.Set(input.P2).SubLocal(input.P1);
            float c = Vec2.Dot(s, r);
            float rr = Vec2.Dot(r, r);
            float sigma = c * c - rr * b;

            // Check for negative discriminant and short segment.
            if (sigma < 0.0f || rr < Settings.EPSILON)
            {
                return false;
            }

            // Find the point of intersection of the line with the circle.
            float a = -(c + MathUtils.Sqrt(sigma));

            // Is the intersection point on the segment?
            if (0.0f <= a && a <= input.MaxFraction * rr)
            {
                a /= rr;
                output.Fraction = a;
                output.Normal.Set(r).MulLocal(a);
                output.Normal.AddLocal(s);
                output.Normal.Normalize();
                return true;
            }

            return false;
        }

        public override void ComputeAABB(AABB aabb, Transform transform, int childIndex)
        {
            Vec2 p = pool1;
            Rot.MulToOutUnsafe(transform.q, P, p);
            p.AddLocal(transform.p);

            aabb.LowerBound.X = p.X - Radius;
            aabb.LowerBound.Y = p.Y - Radius;
            aabb.UpperBound.X = p.X + Radius;
            aabb.UpperBound.Y = p.Y + Radius;
        }

        public override void ComputeMass(MassData massData, float density)
        {
            massData.Mass = density * Settings.PI * Radius * Radius;
            massData.Center.Set(P);

            // inertia about the local origin
            massData.I = massData.Mass * (0.5f * Radius * Radius + Vec2.Dot(P, P));
        }

        // djm pooled from above
        /*
        * @see Shape#computeSubmergedArea(Vec2, float, Vec2, Vec2)
        * @Override
        * public final float computeSubmergedArea( final Vec2 normal, final float offset,
        * final Transform xf, final Vec2 c) {
        * final Vec2 p = tlp.get();
        * Transform.mulToOut(xf,m_p, p);
        * final float l = -( Vec2.dot(normal,p) - offset);
        * if( l < -m_radius + Settings.EPSILON){
        * //Completely dry
        * return 0;
        * }
        * if(l > m_radius){
        * //Completely wet
        * c.set(p);
        * return (float)Math.PI*m_radius*m_radius;
        * }
        * //Magic
        * final float r2 = m_radius*m_radius;
        * final float l2 = l*l;
        * //Erin TODO: write Sqrt to handle fixed point case.
        * final float area = (float) (r2 * (Math.asin(l/m_radius) + Math.PI/2)+ l *
        * Math.sqrt(r2 - l2));
        * final float com = (float) (-2.0/3.0* Math.pow(r2-l2,1.5f)/area);
        * c.x = p.x + normal.x * com;
        * c.y = p.y + normal.y * com;
        * return area;
        * }
        */
    }
}