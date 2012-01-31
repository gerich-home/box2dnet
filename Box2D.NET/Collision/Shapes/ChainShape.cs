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
        public Vec2[] m_vertices;
        public int m_count;
        public readonly Vec2 m_prevVertex = new Vec2();
        public readonly Vec2 m_nextVertex = new Vec2();
        public bool m_hasPrevVertex = false;
        public bool m_hasNextVertex = false;

        private readonly EdgeShape pool0 = new EdgeShape();
        private readonly Vec2 pool1 = new Vec2();
        private readonly Vec2 pool2 = new Vec2();

        public ChainShape() :
            base(ShapeType.CHAIN)
        {
            m_vertices = null;
            m_radius = Settings.polygonRadius;
            m_count = 0;
        }

        override public int ChildCount
        {
            get
            {
                return m_count - 1;
            }
        }

        /// <summary>
        /// Get a child edge.
        /// </summary>
        public virtual void getChildEdge(EdgeShape edge, int index)
        {
            Debug.Assert(0 <= index && index < m_count - 1);
            edge.m_radius = m_radius;

            edge.m_vertex1.set_Renamed(m_vertices[index + 0]);
            edge.m_vertex2.set_Renamed(m_vertices[index + 1]);

            if (index > 0)
            {
                edge.m_vertex0.set_Renamed(m_vertices[index - 1]);
                edge.m_hasVertex0 = true;
            }
            else
            {
                edge.m_vertex0.set_Renamed(m_prevVertex);
                edge.m_hasVertex0 = m_hasPrevVertex;
            }

            if (index < m_count - 2)
            {
                edge.m_vertex3.set_Renamed(m_vertices[index + 2]);
                edge.m_hasVertex3 = true;
            }
            else
            {
                edge.m_vertex3.set_Renamed(m_nextVertex);
                edge.m_hasVertex3 = m_hasNextVertex;
            }
        }

        public override bool testPoint(Transform xf, Vec2 p)
        {
            return false;
        }

        public override bool raycast(RayCastOutput output, RayCastInput input, Transform xf, int childIndex)
        {
            Debug.Assert(childIndex < m_count);

            EdgeShape edgeShape = pool0;

            int i1 = childIndex;
            int i2 = childIndex + 1;
            if (i2 == m_count)
            {
                i2 = 0;
            }

            edgeShape.m_vertex1.set_Renamed(m_vertices[i1]);
            edgeShape.m_vertex2.set_Renamed(m_vertices[i2]);

            return edgeShape.raycast(output, input, xf, 0);
        }

        public override void computeAABB(AABB aabb, Transform xf, int childIndex)
        {
            Debug.Assert(childIndex < m_count);

            int i1 = childIndex;
            int i2 = childIndex + 1;
            if (i2 == m_count)
            {
                i2 = 0;
            }

            Vec2 v1 = pool1;
            Vec2 v2 = pool2;
            Transform.mulToOutUnsafe(xf, m_vertices[i1], v1);
            Transform.mulToOutUnsafe(xf, m_vertices[i2], v2);

            Vec2.minToOut(v1, v2, aabb.lowerBound);
            Vec2.maxToOut(v1, v2, aabb.upperBound);
        }

        public override void computeMass(MassData massData, float density)
        {
            massData.mass = 0.0f;
            massData.center.setZero();
            massData.I = 0.0f;
        }

        public override Shape Clone()
        {
            ChainShape clone = new ChainShape();
            clone.createChain(m_vertices, m_count);
            clone.m_prevVertex.set_Renamed(m_prevVertex);
            clone.m_nextVertex.set_Renamed(m_nextVertex);
            clone.m_hasPrevVertex = m_hasPrevVertex;
            clone.m_hasNextVertex = m_hasNextVertex;
            return clone;
        }

        /// <summary>
        /// Create a loop. This automatically adjusts connectivity.
        /// </summary>
        /// <param name="vertices">an array of vertices, these are copied</param>
        /// <param name="count">the vertex count</param>
        public virtual void createLoop(Vec2[] vertices, int count)
        {
            Debug.Assert(m_vertices == null && m_count == 0);
            Debug.Assert(count >= 3);
            m_count = count + 1;
            m_vertices = new Vec2[m_count];
            for (int i = 0; i < count; i++)
            {
                m_vertices[i] = new Vec2(vertices[i]);
            }
            m_vertices[count] = m_vertices[0];
            m_prevVertex.set_Renamed(m_vertices[m_count - 2]);
            m_nextVertex.set_Renamed(m_vertices[1]);
            m_hasPrevVertex = true;
            m_hasNextVertex = true;
        }

        /// <summary>
        /// Create a chain with isolated end vertices.
        /// </summary>
        /// <param name="vertices">an array of vertices, these are copied</param>
        /// <param name="count">the vertex count</param>
        public virtual void createChain(Vec2[] vertices, int count)
        {
            Debug.Assert(m_vertices == null && m_count == 0);
            Debug.Assert(count >= 2);
            m_count = count;
            m_vertices = new Vec2[m_count];
            for (int i = 0; i < m_count; i++)
            {
                m_vertices[i] = new Vec2(vertices[i]);
            }
            m_hasPrevVertex = false;
            m_hasNextVertex = false;
        }

        /// <summary>
        /// Establish connectivity to a vertex that precedes the first vertex. Don't call this for loops.
        /// </summary>
        /// <param name="prevVertex"></param>
        virtual public Vec2 PrevVertex
        {
            set
            {
                m_prevVertex.set_Renamed(value);
                m_hasPrevVertex = true;
            }
        }
        /// <summary>
        /// Establish connectivity to a vertex that follows the last vertex. Don't call this for loops.
        /// </summary>
        /// <param name="nextVertex"></param>
        virtual public Vec2 NextVertex
        {
            set
            {
                m_nextVertex.set_Renamed(value);
                m_hasNextVertex = true;
            }
        }
    }
}