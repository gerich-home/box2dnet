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

namespace Box2D.Collision.Shapes
{
    /// <summary>
    /// A line segment (edge) shape. These can be connected in chains or loops to other edge shapes. The
    /// connectivity information is used to ensure correct contact normals.
    /// </summary>
    /// <author>Daniel</author>
    public class EdgeShape : Shape
    {
        /// <summary>
        /// edge vertex 1
        /// </summary>
        public readonly Vec2 m_vertex1 = new Vec2();
        /// <summary>
        /// edge vertex 2
        /// </summary>
        public readonly Vec2 m_vertex2 = new Vec2();

        /// <summary>
        /// optional adjacent vertex 1. Used for smooth collision
        /// </summary>
        public readonly Vec2 m_vertex0 = new Vec2();
        /// <summary>
        /// optional adjacent vertex 2. Used for smooth collision
        /// </summary>
        public readonly Vec2 m_vertex3 = new Vec2();
        public bool m_hasVertex0 = false, m_hasVertex3 = false;

        private readonly Vec2 pool0 = new Vec2();
        private readonly Vec2 pool1 = new Vec2();
        private readonly Vec2 pool2 = new Vec2();
        private readonly Vec2 pool3 = new Vec2();
        private readonly Vec2 pool4 = new Vec2();
        private readonly Vec2 pool5 = new Vec2();

        public EdgeShape() :
            base(ShapeType.Edge)
        {
            Radius = Settings.polygonRadius;
        }

        override public int ChildCount
        {
            get
            {
                return 1;
            }
        }

        public virtual void set_Renamed(Vec2 v1, Vec2 v2)
        {
            m_vertex1.set_Renamed(v1);
            m_vertex2.set_Renamed(v2);
            m_hasVertex0 = m_hasVertex3 = false;
        }

        public override bool TestPoint(Transform xf, Vec2 p)
        {
            return false;
        }

        public override bool Raycast(RayCastOutput output, RayCastInput input, Transform xf, int childIndex)
        {

            // Put the ray into the edge's frame of reference.
            Vec2 p1 = pool0.set_Renamed(input.p1).subLocal(xf.p);
            Rot.mulTrans(xf.q, p1, p1);
            Vec2 p2 = pool1.set_Renamed(input.p2).subLocal(xf.p);
            Rot.mulTrans(xf.q, p1, p1);
            Vec2 d = p2.subLocal(p1); // we don't use p2 later

            Vec2 v1 = m_vertex1;
            Vec2 v2 = m_vertex2;
            Vec2 normal = pool2.set_Renamed(v2).subLocal(v1);
            normal.set_Renamed(normal.y, -normal.x);
            normal.normalize();

            // q = p1 + t * d
            // dot(normal, q - v1) = 0
            // dot(normal, p1 - v1) + t * dot(normal, d) = 0
            pool3.set_Renamed(v1).subLocal(p1);
            float numerator = Vec2.dot(normal, pool3);
            float denominator = Vec2.dot(normal, d);

            if (denominator == 0.0f)
            {
                return false;
            }

            float t = numerator / denominator;
            if (t < 0.0f || 1.0f < t)
            {
                return false;
            }

            Vec2 q = pool3;
            Vec2 r = pool4;

            // Vec2 q = p1 + t * d;
            q.set_Renamed(d).mulLocal(t).addLocal(p1);

            // q = v1 + s * r
            // s = dot(q - v1, r) / dot(r, r)
            // Vec2 r = v2 - v1;
            r.set_Renamed(v2).subLocal(v1);
            float rr = Vec2.dot(r, r);
            if (rr == 0.0f)
            {
                return false;
            }

            pool5.set_Renamed(q).subLocal(v1);
            float s = Vec2.dot(pool5, r) / rr;
            if (s < 0.0f || 1.0f < s)
            {
                return false;
            }

            output.fraction = t;
            if (numerator > 0.0f)
            {
                // argOutput.normal = -normal;
                output.normal.set_Renamed(normal).negateLocal();
            }
            else
            {
                // output.normal = normal;
                output.normal.set_Renamed(normal);
            }
            return true;
        }

        public override void ComputeAABB(AABB aabb, Transform xf, int childIndex)
        {
            Vec2 v1 = pool1;
            Vec2 v2 = pool2;

            Transform.mulToOutUnsafe(xf, m_vertex1, v1);
            Transform.mulToOutUnsafe(xf, m_vertex2, v2);

            Vec2.minToOut(v1, v2, aabb.lowerBound);
            Vec2.maxToOut(v1, v2, aabb.upperBound);

            aabb.lowerBound.x -= Radius;
            aabb.lowerBound.y -= Radius;
            aabb.upperBound.x += Radius;
            aabb.upperBound.y += Radius;
        }

        public override void ComputeMass(MassData massData, float density)
        {
            massData.Mass = 0.0f;
            massData.Center.set_Renamed(m_vertex1).addLocal(m_vertex2).mulLocal(0.5f);
            massData.I = 0.0f;
        }

        public override Shape Clone()
        {
            EdgeShape edge = new EdgeShape();
            edge.Radius = this.Radius;
            edge.m_hasVertex0 = this.m_hasVertex0;
            edge.m_hasVertex3 = this.m_hasVertex3;
            edge.m_vertex0.set_Renamed(this.m_vertex0);
            edge.m_vertex1.set_Renamed(this.m_vertex1);
            edge.m_vertex2.set_Renamed(this.m_vertex2);
            edge.m_vertex3.set_Renamed(this.m_vertex3);
            return edge;
        }
    }
}