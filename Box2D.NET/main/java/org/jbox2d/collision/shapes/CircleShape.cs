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
using AABB = org.jbox2d.collision.AABB;
using RayCastInput = org.jbox2d.collision.RayCastInput;
using RayCastOutput = org.jbox2d.collision.RayCastOutput;
using MathUtils = org.jbox2d.common.MathUtils;
using Rot = org.jbox2d.common.Rot;
using Settings = org.jbox2d.common.Settings;
using Transform = org.jbox2d.common.Transform;
using Vec2 = org.jbox2d.common.Vec2;
using System.Diagnostics;

namespace org.jbox2d.collision.shapes
{

    /// <summary>
    /// A circle shape.
    /// </summary>
    public class CircleShape : Shape
    {
        public readonly Vec2 m_p;

        private readonly Vec2 pool1 = new Vec2();
        private readonly Vec2 pool2 = new Vec2();
        private readonly Vec2 pool3 = new Vec2();

        /// <summary>
        /// this is used internally, instead use {@link Body#createShape(ShapeDef)} with a
        /// {@link CircleDef}
        /// </summary>
        /// <seealso cref="Body.createShape(ShapeDef)"></seealso>
        /// <seealso cref="CircleDef"></seealso>
        /// <param name="def"></param>
        public CircleShape() :
            base(ShapeType.CIRCLE)
        {
            m_p = new Vec2();
            m_radius = 0;
        }


        public override Shape Clone()
        {
            CircleShape shape = new CircleShape();
            shape.m_p.set_Renamed(m_p);
            shape.m_radius = m_radius;
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
        public int getSupport(Vec2 d)
        {
            return 0;
        }

        /// <summary>
        /// Get the supporting vertex in the given direction.
        /// </summary>
        /// <param name="d"></param>
        /// <returns></returns>
        public Vec2 getSupportVertex(Vec2 d)
        {
            return m_p;
        }

        /// <summary>
        /// Get the vertex count.
        /// </summary>
        /// <returns></returns>
        virtual public int VertexCount
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
        public Vec2 getVertex(int index)
        {
            Debug.Assert(index == 0);
            return m_p;
        }

        public override bool testPoint(Transform transform, Vec2 p)
        {
            Vec2 center = pool1;
            Rot.mulToOutUnsafe(transform.q, m_p, center);
            center.addLocal(transform.p);

            Vec2 d = center.subLocal(p).negateLocal();
            return Vec2.dot(d, d) <= m_radius * m_radius;
        }

        // Collision Detection in Interactive 3D Environments by Gino van den Bergen
        // From Section 3.1.2
        // x = s + a * r
        // norm(x) = radius
        public override bool raycast(RayCastOutput output, RayCastInput input, Transform transform, int childIndex)
        {
            Vec2 position = pool1;
            Vec2 s = pool2;
            Vec2 r = pool3;

            Rot.mulToOutUnsafe(transform.q, m_p, position);
            position.addLocal(transform.p);
            s.set_Renamed(input.p1).subLocal(position);
            float b = Vec2.dot(s, s) - m_radius * m_radius;

            // Solve quadratic equation.
            r.set_Renamed(input.p2).subLocal(input.p1);
            float c = Vec2.dot(s, r);
            float rr = Vec2.dot(r, r);
            float sigma = c * c - rr * b;

            // Check for negative discriminant and short segment.
            if (sigma < 0.0f || rr < Settings.EPSILON)
            {
                return false;
            }

            // Find the point of intersection of the line with the circle.
            float a = -(c + MathUtils.sqrt(sigma));

            // Is the intersection point on the segment?
            if (0.0f <= a && a <= input.maxFraction * rr)
            {
                a /= rr;
                output.fraction = a;
                output.normal.set_Renamed(r).mulLocal(a);
                output.normal.addLocal(s);
                output.normal.normalize();
                return true;
            }

            return false;
        }

        public override void computeAABB(AABB aabb, Transform transform, int childIndex)
        {
            Vec2 p = pool1;
            Rot.mulToOutUnsafe(transform.q, m_p, p);
            p.addLocal(transform.p);

            aabb.lowerBound.x = p.x - m_radius;
            aabb.lowerBound.y = p.y - m_radius;
            aabb.upperBound.x = p.x + m_radius;
            aabb.upperBound.y = p.y + m_radius;
        }

        public override void computeMass(MassData massData, float density)
        {
            massData.mass = density * Settings.PI * m_radius * m_radius;
            massData.center.set_Renamed(m_p);

            // inertia about the local origin
            massData.I = massData.mass * (0.5f * m_radius * m_radius + Vec2.dot(m_p, m_p));
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