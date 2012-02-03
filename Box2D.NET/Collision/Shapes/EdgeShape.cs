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
        public readonly Vec2 Vertex1 = new Vec2();

        /// <summary>
        /// edge vertex 2
        /// </summary>
        public readonly Vec2 Vertex2 = new Vec2();

        /// <summary>
        /// optional adjacent vertex 1. Used for smooth collision
        /// </summary>
        public readonly Vec2 Vertex0 = new Vec2();

        /// <summary>
        /// optional adjacent vertex 2. Used for smooth collision
        /// </summary>
        public readonly Vec2 Vertex3 = new Vec2();

        public bool HasVertex0;
        public bool HasVertex3;

        private readonly Vec2 pool0 = new Vec2();
        private readonly Vec2 pool1 = new Vec2();
        private readonly Vec2 pool2 = new Vec2();
        private readonly Vec2 pool3 = new Vec2();
        private readonly Vec2 pool4 = new Vec2();
        private readonly Vec2 pool5 = new Vec2();

        public EdgeShape() :
            base(ShapeType.Edge)
        {
            Radius = Settings.POLYGON_RADIUS;
        }

        override public int ChildCount
        {
            get
            {
                return 1;
            }
        }

        public virtual void Set(Vec2 v1, Vec2 v2)
        {
            Vertex1.Set(v1);
            Vertex2.Set(v2);
            HasVertex0 = HasVertex3 = false;
        }

        public override bool TestPoint(Transform xf, Vec2 p)
        {
            return false;
        }

        public override bool Raycast(RayCastOutput output, RayCastInput input, Transform xf, int childIndex)
        {

            // Put the ray into the edge's frame of reference.
            Vec2 p1 = pool0.Set(input.P1).SubLocal(xf.p);
            Rot.MulTrans(xf.q, p1, p1);
            Vec2 p2 = pool1.Set(input.P2).SubLocal(xf.p);
            Rot.MulTrans(xf.q, p1, p1);
            Vec2 d = p2.SubLocal(p1); // we don't use p2 later

            Vec2 v1 = Vertex1;
            Vec2 v2 = Vertex2;
            Vec2 normal = pool2.Set(v2).SubLocal(v1);
            normal.Set(normal.Y, -normal.X);
            normal.Normalize();

            // q = p1 + t * d
            // dot(normal, q - v1) = 0
            // dot(normal, p1 - v1) + t * dot(normal, d) = 0
            pool3.Set(v1).SubLocal(p1);
            float numerator = Vec2.Dot(normal, pool3);
            float denominator = Vec2.Dot(normal, d);

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
            q.Set(d).MulLocal(t).AddLocal(p1);

            // q = v1 + s * r
            // s = dot(q - v1, r) / dot(r, r)
            // Vec2 r = v2 - v1;
            r.Set(v2).SubLocal(v1);
            float rr = Vec2.Dot(r, r);
            if (rr == 0.0f)
            {
                return false;
            }

            pool5.Set(q).SubLocal(v1);
            float s = Vec2.Dot(pool5, r) / rr;
            if (s < 0.0f || 1.0f < s)
            {
                return false;
            }

            output.Fraction = t;
            if (numerator > 0.0f)
            {
                // argOutput.normal = -normal;
                output.Normal.Set(normal).NegateLocal();
            }
            else
            {
                // output.normal = normal;
                output.Normal.Set(normal);
            }
            return true;
        }

        public override void ComputeAABB(AABB aabb, Transform xf, int childIndex)
        {
            Vec2 v1 = pool1;
            Vec2 v2 = pool2;

            Transform.mulToOutUnsafe(xf, Vertex1, v1);
            Transform.mulToOutUnsafe(xf, Vertex2, v2);

            Vec2.MinToOut(v1, v2, aabb.LowerBound);
            Vec2.MaxToOut(v1, v2, aabb.UpperBound);

            aabb.LowerBound.X -= Radius;
            aabb.LowerBound.Y -= Radius;
            aabb.UpperBound.X += Radius;
            aabb.UpperBound.Y += Radius;
        }

        public override void ComputeMass(MassData massData, float density)
        {
            massData.Mass = 0.0f;
            massData.Center.Set(Vertex1).AddLocal(Vertex2).MulLocal(0.5f);
            massData.I = 0.0f;
        }

        public override Shape Clone()
        {
            EdgeShape edge = new EdgeShape();
            edge.Radius = this.Radius;
            edge.HasVertex0 = this.HasVertex0;
            edge.HasVertex3 = this.HasVertex3;
            edge.Vertex0.Set(this.Vertex0);
            edge.Vertex1.Set(this.Vertex1);
            edge.Vertex2.Set(this.Vertex2);
            edge.Vertex3.Set(this.Vertex3);
            return edge;
        }
    }
}