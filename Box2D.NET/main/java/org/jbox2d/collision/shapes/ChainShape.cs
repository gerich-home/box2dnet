/// <summary>****************************************************************************
/// Copyright (c) 2011, Daniel Murphy
/// All rights reserved.
/// 
/// Redistribution and use in source and binary forms, with or without modification,
/// are permitted provided that the following conditions are met:
/// * Redistributions of source code must retain the above copyright notice,
/// this list of conditions and the following disclaimer.
/// * Redistributions in binary form must reproduce the above copyright notice,
/// this list of conditions and the following disclaimer in the documentation
/// and/or other materials provided with the distribution.
/// 
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
/// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
/// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
/// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
/// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
/// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
/// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
/// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
/// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
/// ****************************************************************************
/// </summary>
using System;
using AABB = org.jbox2d.collision.AABB;
using RayCastInput = org.jbox2d.collision.RayCastInput;
using RayCastOutput = org.jbox2d.collision.RayCastOutput;
using Settings = org.jbox2d.common.Settings;
using Transform = org.jbox2d.common.Transform;
using Vec2 = org.jbox2d.common.Vec2;
namespace org.jbox2d.collision.shapes
{
	
	/// <summary> A chain shape is a free form sequence of line segments. The chain has two-sided collision, so you
	/// can use inside and outside collision. Therefore, you may use any winding order. Since there may
	/// be many vertices, they are allocated using Alloc. Connectivity information is used to create
	/// smooth collisions. WARNING The chain will not collide properly if there are self-intersections.
	/// 
	/// </summary>
	/// <author>  Daniel
	/// </author>
	public class ChainShape:Shape, System.ICloneable
	{
		override public int ChildCount
		{
			get
			{
				return m_count - 1;
			}
			
		}
		/// <summary> Establish connectivity to a vertex that precedes the first vertex. Don't call this for loops.
		/// 
		/// </summary>
		/// <param name="prevVertex">
		/// </param>
		virtual public Vec2 PrevVertex
		{
			set
			{
				m_prevVertex.set_Renamed(value);
				m_hasPrevVertex = true;
			}
			
		}
		/// <summary> Establish connectivity to a vertex that follows the last vertex. Don't call this for loops.
		/// 
		/// </summary>
		/// <param name="nextVertex">
		/// </param>
		virtual public Vec2 NextVertex
		{
			set
			{
				m_nextVertex.set_Renamed(value);
				m_hasNextVertex = true;
			}
			
		}
		
		public Vec2[] m_vertices;
		public int m_count;
		//UPGRADE_NOTE: Final was removed from the declaration of 'm_prevVertex '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		//UPGRADE_NOTE: Final was removed from the declaration of 'm_nextVertex '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Vec2 m_prevVertex = new Vec2();
		public Vec2 m_nextVertex = new Vec2();
		public bool m_hasPrevVertex = false, m_hasNextVertex = false;
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'pool0 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private EdgeShape pool0 = new EdgeShape();
		//UPGRADE_NOTE: Final was removed from the declaration of 'pool1 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Vec2 pool1 = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'pool2 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Vec2 pool2 = new Vec2();
		
		public ChainShape():base(ShapeType.CHAIN)
		{
			m_vertices = null;
			m_radius = Settings.polygonRadius;
			m_count = 0;
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		
		/// <summary> Get a child edge.</summary>
		public virtual void  getChildEdge(EdgeShape edge, int index)
		{
			assert(0 <= index && index < m_count - 1);
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
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public override bool testPoint(Transform xf, Vec2 p)
		{
			return false;
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public override bool raycast(RayCastOutput output, RayCastInput input, Transform xf, int childIndex)
		{
			assert(childIndex < m_count);
			
			//UPGRADE_NOTE: Final was removed from the declaration of 'edgeShape '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
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
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public override void  computeAABB(AABB aabb, Transform xf, int childIndex)
		{
			assert(childIndex < m_count);
			
			int i1 = childIndex;
			int i2 = childIndex + 1;
			if (i2 == m_count)
			{
				i2 = 0;
			}
			
			//UPGRADE_NOTE: Final was removed from the declaration of 'v1 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Vec2 v1 = pool1;
			//UPGRADE_NOTE: Final was removed from the declaration of 'v2 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Vec2 v2 = pool2;
			Transform.mulToOutUnsafe(xf, m_vertices[i1], v1);
			Transform.mulToOutUnsafe(xf, m_vertices[i2], v1);
			
			Vec2.minToOut(v1, v2, aabb.lowerBound);
			Vec2.maxToOut(v1, v2, aabb.upperBound);
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public override void  computeMass(MassData massData, float density)
		{
			massData.mass = 0.0f;
			massData.center.setZero();
			massData.I = 0.0f;
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		//UPGRADE_ISSUE: The equivalent in .NET for method 'java.lang.Object.clone' returns a different type. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1224'"
		public override System.Object Clone()
		{
			ChainShape clone = new ChainShape();
			clone.createChain(m_vertices, m_count);
			clone.m_prevVertex.set_Renamed(m_prevVertex);
			clone.m_nextVertex.set_Renamed(m_nextVertex);
			clone.m_hasPrevVertex = m_hasPrevVertex;
			clone.m_hasNextVertex = m_hasNextVertex;
			return clone;
		}
		
		/// <summary> Create a loop. This automatically adjusts connectivity.
		/// 
		/// </summary>
		/// <param name="vertices">an array of vertices, these are copied
		/// </param>
		/// <param name="count">the vertex count
		/// </param>
		public virtual void  createLoop(Vec2[] vertices, int count)
		{
			assert(m_vertices == null && m_count == 0);
			assert(count >= 3);
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
		
		/// <summary> Create a chain with isolated end vertices.
		/// 
		/// </summary>
		/// <param name="vertices">an array of vertices, these are copied
		/// </param>
		/// <param name="count">the vertex count
		/// </param>
		public virtual void  createChain(Vec2[] vertices, int count)
		{
			assert(m_vertices == null && m_count == 0);
			assert(count >= 2);
			m_count = count;
			m_vertices = new Vec2[m_count];
			for (int i = 0; i < m_count; i++)
			{
				m_vertices[i] = new Vec2(vertices[i]);
			}
			m_hasPrevVertex = false;
			m_hasNextVertex = false;
		}
	}
}