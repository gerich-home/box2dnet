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
using ContactListener = org.jbox2d.callbacks.ContactListener;
using ContactID = org.jbox2d.collision.ContactID;
using Manifold = org.jbox2d.collision.Manifold;
using ManifoldPoint = org.jbox2d.collision.ManifoldPoint;
using WorldManifold = org.jbox2d.collision.WorldManifold;
using Shape = org.jbox2d.collision.shapes.Shape;
using MathUtils = org.jbox2d.common.MathUtils;
using Transform = org.jbox2d.common.Transform;
using Body = org.jbox2d.dynamics.Body;
using Fixture = org.jbox2d.dynamics.Fixture;
using IWorldPool = org.jbox2d.pooling.IWorldPool;
namespace org.jbox2d.dynamics.contacts
{
	
	/// <summary> The class manages contact between two shapes. A contact exists for each overlapping AABB in the
	/// broad-phase (except if filtered). Therefore a contact object may exist that has no contact
	/// points.
	/// 
	/// </summary>
	/// <author>  daniel
	/// </author>
	public abstract class Contact
	{
		/// <summary> Get the contact manifold. Do not set the point count to zero. Instead call Disable.</summary>
		virtual public Manifold Manifold
		{
			get
			{
				return m_manifold;
			}
			
		}
		/// <summary> Is this contact touching
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		virtual public bool Touching
		{
			get
			{
				return (m_flags & TOUCHING_FLAG) == TOUCHING_FLAG;
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary> Has this contact been disabled?
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		/// <summary> Enable/disable this contact. This can be used inside the pre-solve contact listener. The
		/// contact is only disabled for the current time step (or sub-step in continuous collisions).
		/// 
		/// </summary>
		/// <param name="flag">
		/// </param>
		virtual public bool Enabled
		{
			get
			{
				return (m_flags & ENABLED_FLAG) == ENABLED_FLAG;
			}
			
			set
			{
				if (value)
				{
					m_flags |= ENABLED_FLAG;
				}
				else
				{
					m_flags &= ~ ENABLED_FLAG;
				}
			}
			
		}
		/// <summary> Get the next contact in the world's contact list.
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		virtual public Contact Next
		{
			get
			{
				return m_next;
			}
			
		}
		/// <summary> Get the first fixture in this contact.
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		virtual public Fixture FixtureA
		{
			get
			{
				return m_fixtureA;
			}
			
		}
		virtual public int ChildIndexA
		{
			get
			{
				return m_indexA;
			}
			
		}
		/// <summary> Get the second fixture in this contact.
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		virtual public Fixture FixtureB
		{
			get
			{
				return m_fixtureB;
			}
			
		}
		virtual public int ChildIndexB
		{
			get
			{
				return m_indexB;
			}
			
		}
		virtual public float Friction
		{
			get
			{
				return m_friction;
			}
			
			set
			{
				m_friction = value;
			}
			
		}
		virtual public float Restitution
		{
			get
			{
				return m_restitution;
			}
			
			set
			{
				m_restitution = value;
			}
			
		}
		virtual public float TangentSpeed
		{
			get
			{
				return m_tangentSpeed;
			}
			
			set
			{
				m_tangentSpeed = value;
			}
			
		}
		
		// Flags stored in m_flags
		// Used when crawling contact graph when forming islands.
		public const int ISLAND_FLAG = 0x0001;
		// Set when the shapes are touching.
		public const int TOUCHING_FLAG = 0x0002; // NO_UCD
		// This contact can be disabled (by user)
		public const int ENABLED_FLAG = 0x0004;
		// This contact needs filtering because a fixture filter was changed.
		public const int FILTER_FLAG = 0x0008;
		// This bullet contact had a TOI event
		public const int BULLET_HIT_FLAG = 0x0010;
		
		public const int TOI_FLAG = 0x0020;
		
		public int m_flags;
		
		// World pool and list pointers.
		public Contact m_prev;
		public Contact m_next;
		
		// Nodes for connecting bodies.
		public ContactEdge m_nodeA = null;
		public ContactEdge m_nodeB = null;
		
		public Fixture m_fixtureA;
		public Fixture m_fixtureB;
		
		public int m_indexA;
		public int m_indexB;
		
		public Manifold m_manifold;
		
		public float m_toiCount;
		public float m_toi;
		
		public float m_friction;
		public float m_restitution;
		
		public float m_tangentSpeed;
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'pool '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		protected internal IWorldPool pool;
		
		protected internal Contact(IWorldPool argPool)
		{
			m_fixtureA = null;
			m_fixtureB = null;
			m_nodeA = new ContactEdge();
			m_nodeB = new ContactEdge();
			m_manifold = new Manifold();
			pool = argPool;
		}
		
		/// <summary>initialization for pooling </summary>
		public virtual void  init(Fixture fA, int indexA, Fixture fB, int indexB)
		{
			m_flags = 0;
			
			m_fixtureA = fA;
			m_fixtureB = fB;
			
			m_indexA = indexA;
			m_indexB = indexB;
			
			m_manifold.pointCount = 0;
			
			m_prev = null;
			m_next = null;
			
			m_nodeA.contact = null;
			m_nodeA.prev = null;
			m_nodeA.next = null;
			m_nodeA.other = null;
			
			m_nodeB.contact = null;
			m_nodeB.prev = null;
			m_nodeB.next = null;
			m_nodeB.other = null;
			
			m_toiCount = 0;
			m_friction = Contact.mixFriction(fA.m_friction, fB.m_friction);
			m_restitution = Contact.mixRestitution(fA.m_restitution, fB.m_restitution);
			
			m_tangentSpeed = 0;
		}
		
		/// <summary> Get the world manifold.</summary>
		public virtual void  getWorldManifold(WorldManifold worldManifold)
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'bodyA '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Body bodyA = m_fixtureA.Body;
			//UPGRADE_NOTE: Final was removed from the declaration of 'bodyB '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Body bodyB = m_fixtureB.Body;
			//UPGRADE_NOTE: Final was removed from the declaration of 'shapeA '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Shape shapeA = m_fixtureA.Shape;
			//UPGRADE_NOTE: Final was removed from the declaration of 'shapeB '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Shape shapeB = m_fixtureB.Shape;
			
			worldManifold.initialize(m_manifold, bodyA.getTransform(), shapeA.m_radius, bodyB.getTransform(), shapeB.m_radius);
		}
		
		public virtual void  resetFriction()
		{
			m_friction = Contact.mixFriction(m_fixtureA.m_friction, m_fixtureB.m_friction);
		}
		
		public virtual void  resetRestitution()
		{
			m_restitution = Contact.mixRestitution(m_fixtureA.m_restitution, m_fixtureB.m_restitution);
		}
		
		public abstract void  evaluate(Manifold manifold, Transform xfA, Transform xfB);
		
		/// <summary> Flag this contact for filtering. Filtering will occur the next time step.</summary>
		public virtual void  flagForFiltering()
		{
			m_flags |= FILTER_FLAG;
		}
		
		// djm pooling
		//UPGRADE_NOTE: Final was removed from the declaration of 'oldManifold '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Manifold oldManifold = new Manifold();
		
		public virtual void  update(ContactListener listener)
		{
			
			oldManifold.set_Renamed(m_manifold);
			
			// Re-enable this contact.
			m_flags |= ENABLED_FLAG;
			
			bool touching = false;
			bool wasTouching = (m_flags & TOUCHING_FLAG) == TOUCHING_FLAG;
			
			bool sensorA = m_fixtureA.Sensor;
			bool sensorB = m_fixtureB.Sensor;
			bool sensor = sensorA || sensorB;
			
			Body bodyA = m_fixtureA.Body;
			Body bodyB = m_fixtureB.Body;
			Transform xfA = bodyA.getTransform();
			Transform xfB = bodyB.getTransform();
			// log.debug("TransformA: "+xfA);
			// log.debug("TransformB: "+xfB);
			
			if (sensor)
			{
				Shape shapeA = m_fixtureA.Shape;
				Shape shapeB = m_fixtureB.Shape;
				touching = pool.getCollision().testOverlap(shapeA, m_indexA, shapeB, m_indexB, xfA, xfB);
				
				// Sensors don't generate manifolds.
				m_manifold.pointCount = 0;
			}
			else
			{
				evaluate(m_manifold, xfA, xfB);
				touching = m_manifold.pointCount > 0;
				
				// Match old contact ids to new contact ids and copy the
				// stored impulses to warm start the solver.
				for (int i = 0; i < m_manifold.pointCount; ++i)
				{
					ManifoldPoint mp2 = m_manifold.points[i];
					mp2.normalImpulse = 0.0f;
					mp2.tangentImpulse = 0.0f;
					ContactID id2 = mp2.id;
					
					for (int j = 0; j < oldManifold.pointCount; ++j)
					{
						ManifoldPoint mp1 = oldManifold.points[j];
						
						if (mp1.id.isEqual(id2))
						{
							mp2.normalImpulse = mp1.normalImpulse;
							mp2.tangentImpulse = mp1.tangentImpulse;
							break;
						}
					}
				}
				
				if (touching != wasTouching)
				{
					bodyA.Awake = true;
					bodyB.Awake = true;
				}
			}
			
			if (touching)
			{
				m_flags |= TOUCHING_FLAG;
			}
			else
			{
				m_flags &= ~ TOUCHING_FLAG;
			}
			
			if (listener == null)
			{
				return ;
			}
			
			if (wasTouching == false && touching == true)
			{
				listener.beginContact(this);
			}
			
			if (wasTouching == true && touching == false)
			{
				listener.endContact(this);
			}
			
			if (sensor == false && touching)
			{
				listener.preSolve(this, oldManifold);
			}
		}
		
		/// <summary> Friction mixing law. The idea is to allow either fixture to drive the restitution to zero. For
		/// example, anything slides on ice.
		/// 
		/// </summary>
		/// <param name="friction1">
		/// </param>
		/// <param name="friction2">
		/// </param>
		/// <returns>
		/// </returns>
		public static float mixFriction(float friction1, float friction2)
		{
			return MathUtils.sqrt(friction1 * friction2);
		}
		
		/// <summary> Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface. For
		/// example, a superball bounces on anything.
		/// 
		/// </summary>
		/// <param name="restitution1">
		/// </param>
		/// <param name="restitution2">
		/// </param>
		/// <returns>
		/// </returns>
		public static float mixRestitution(float restitution1, float restitution2)
		{
			return restitution1 > restitution2?restitution1:restitution2;
		}
	}
}