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
    /// A chain shape is a free form sequence of line segments. The chain has two-sided collision, so you
    /// can use inside and outside collision. Therefore, you may use any winding order. Since there may
    /// be many vertices, they are allocated using Alloc. Connectivity information is used to create
    /// smooth collisions. WARNING The chain will not collide properly if there are self-intersections.
    /// </summary>
    /// <author>Daniel</author>
    public class ChainShape : Shape
    {
        public Vec2[] Vertices;
        public int Count;
        public bool HasPrevVertex;
        public bool HasNextVertex;

        private readonly EdgeShape pool0 = new EdgeShape();
        private readonly Vec2 pool1 = new Vec2();
        private readonly Vec2 pool2 = new Vec2();
        private readonly Vec2 m_prevVertex = new Vec2();
        private readonly Vec2 m_nextVertex = new Vec2();

        public ChainShape() :
            base(ShapeType.Chain)
        {
            Vertices = null;
            Radius = Settings.polygonRadius;
            Count = 0;
        }

        override public int ChildCount
        {
            get
            {
                return Count - 1;
            }
        }

        /// <summary>
        /// Get a child edge.
        /// </summary>
        public virtual void GetChildEdge(EdgeShape edge, int index)
        {
            Debug.Assert(0 <= index && index < Count - 1);
            edge.Radius = Radius;

            edge.Vertex1.set_Renamed(Vertices[index + 0]);
            edge.Vertex2.set_Renamed(Vertices[index + 1]);

            if (index > 0)
            {
                edge.Vertex0.set_Renamed(Vertices[index - 1]);
                edge.HasVertex0 = true;
            }
            else
            {
                edge.Vertex0.set_Renamed(m_prevVertex);
                edge.HasVertex0 = HasPrevVertex;
            }

            if (index < Count - 2)
            {
                edge.Vertex3.set_Renamed(Vertices[index + 2]);
                edge.HasVertex3 = true;
            }
            else
            {
                edge.Vertex3.set_Renamed(m_nextVertex);
                edge.HasVertex3 = HasNextVertex;
            }
        }

        public override bool TestPoint(Transform xf, Vec2 p)
        {
            return false;
        }

        public override bool Raycast(RayCastOutput output, RayCastInput input, Transform xf, int childIndex)
        {
            Debug.Assert(childIndex < Count);

            EdgeShape edgeShape = pool0;

            int i1 = childIndex;
            int i2 = childIndex + 1;
            if (i2 == Count)
            {
                i2 = 0;
            }

            edgeShape.Vertex1.set_Renamed(Vertices[i1]);
            edgeShape.Vertex2.set_Renamed(Vertices[i2]);

            return edgeShape.Raycast(output, input, xf, 0);
        }

        public override void ComputeAABB(AABB aabb, Transform xf, int childIndex)
        {
            Debug.Assert(childIndex < Count);

            int i1 = childIndex;
            int i2 = childIndex + 1;
            if (i2 == Count)
            {
                i2 = 0;
            }

            Vec2 v1 = pool1;
            Vec2 v2 = pool2;
            Transform.mulToOutUnsafe(xf, Vertices[i1], v1);
            Transform.mulToOutUnsafe(xf, Vertices[i2], v2);

            Vec2.minToOut(v1, v2, aabb.LowerBound);
            Vec2.maxToOut(v1, v2, aabb.UpperBound);
        }

        public override void ComputeMass(MassData massData, float density)
        {
            massData.Mass = 0.0f;
            massData.Center.setZero();
            massData.I = 0.0f;
        }

        public override Shape Clone()
        {
            ChainShape clone = new ChainShape();
            clone.CreateChain(Vertices, Count);
            clone.m_prevVertex.set_Renamed(m_prevVertex);
            clone.m_nextVertex.set_Renamed(m_nextVertex);
            clone.HasPrevVertex = HasPrevVertex;
            clone.HasNextVertex = HasNextVertex;
            return clone;
        }

        /// <summary>
        /// Create a loop. This automatically adjusts connectivity.
        /// </summary>
        /// <param name="vertices">an array of vertices, these are copied</param>
        /// <param name="count">the vertex count</param>
        public virtual void CreateLoop(Vec2[] vertices, int count)
        {
            Debug.Assert(Vertices == null && Count == 0);
            Debug.Assert(count >= 3);
            Count = count + 1;
            Vertices = new Vec2[Count];
            for (int i = 0; i < count; i++)
            {
                Vertices[i] = new Vec2(vertices[i]);
            }
            Vertices[count] = Vertices[0];
            m_prevVertex.set_Renamed(Vertices[Count - 2]);
            m_nextVertex.set_Renamed(Vertices[1]);
            HasPrevVertex = true;
            HasNextVertex = true;
        }

        /// <summary>
        /// Create a chain with isolated end vertices.
        /// </summary>
        /// <param name="vertices">an array of vertices, these are copied</param>
        /// <param name="count">the vertex count</param>
        public virtual void CreateChain(Vec2[] vertices, int count)
        {
            Debug.Assert(Vertices == null && Count == 0);
            Debug.Assert(count >= 2);
            Count = count;
            Vertices = new Vec2[Count];
            for (int i = 0; i < Count; i++)
            {
                Vertices[i] = new Vec2(vertices[i]);
            }
            HasPrevVertex = false;
            HasNextVertex = false;
        }

        /// <summary>
        /// Establish connectivity to a vertex that precedes the first vertex. Don't call this for loops.
        /// </summary>
        virtual public Vec2 PrevVertex
        {
            set
            {
                m_prevVertex.set_Renamed(value);
                HasPrevVertex = true;
            }
        }
        /// <summary>
        /// Establish connectivity to a vertex that follows the last vertex. Don't call this for loops.
        /// </summary>
        virtual public Vec2 NextVertex
        {
            set
            {
                m_nextVertex.set_Renamed(value);
                HasNextVertex = true;
            }
        }
    }
}