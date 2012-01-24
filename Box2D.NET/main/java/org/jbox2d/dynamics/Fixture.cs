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
using BroadPhase = org.jbox2d.collision.broadphase.BroadPhase;
using MassData = org.jbox2d.collision.shapes.MassData;
using Shape = org.jbox2d.collision.shapes.Shape;
//UPGRADE_TODO: The type 'org.jbox2d.collision.shapes.ShapeType' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
using ShapeType = org.jbox2d.collision.shapes.ShapeType;
using MathUtils = org.jbox2d.common.MathUtils;
using Transform = org.jbox2d.common.Transform;
using Vec2 = org.jbox2d.common.Vec2;
using Contact = org.jbox2d.dynamics.contacts.Contact;
using ContactEdge = org.jbox2d.dynamics.contacts.ContactEdge;
namespace org.jbox2d.dynamics
{
	
	/// <summary> A fixture is used to attach a shape to a body for collision detection. A fixture inherits its
	/// transform from its parent. Fixtures hold additional non-geometric data such as friction,
	/// collision filters, etc. Fixtures are created via Body::CreateFixture.
	/// 
	/// </summary>
	/// <warning>  you cannot reuse fixtures. </warning>
	/// <summary> 
	/// </summary>
	/// <author>  daniel
	/// </author>
	public class Fixture
	{
		/// <summary> Get the type of the child shape. You can use this to down cast to the concrete shape.
		/// 
		/// </summary>
		/// <returns> the shape type.
		/// </returns>
		virtual public ShapeType Type
		{
			get
			{
				return m_shape.Type;
			}
			
		}
		/// <summary> Get the child shape. You can modify the child shape, however you should not change the number
		/// of vertices because this will crash some collision caching mechanisms.
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		virtual public Shape Shape
		{
			get
			{
				return m_shape;
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary> Is this fixture a sensor (non-solid)?
		/// 
		/// </summary>
		/// <returns> the true if the shape is a sensor.
		/// </returns>
		/// <returns>
		/// </returns>
		/// <summary> Set if this fixture is a sensor.
		/// 
		/// </summary>
		/// <param name="sensor">
		/// </param>
		virtual public bool Sensor
		{
			get
			{
				return m_isSensor;
			}
			
			set
			{
				if (value != m_isSensor)
				{
					m_body.Awake = true;
					m_isSensor = value;
				}
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary> Get the contact filtering data.
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		/// <summary> Set the contact filtering data. This is an expensive operation and should not be called
		/// frequently. This will not update contacts until the next time step when either parent body is
		/// awake. This automatically calls refilter.
		/// 
		/// </summary>
		/// <param name="filter">
		/// </param>
		virtual public Filter FilterData
		{
			get
			{
				return m_filter;
			}
			
			set
			{
				m_filter.set_Renamed(value);
				
				refilter();
			}
			
		}
		/// <summary> Get the parent body of this fixture. This is NULL if the fixture is not attached.
		/// 
		/// </summary>
		/// <returns> the parent body.
		/// </returns>
		/// <returns>
		/// </returns>
		virtual public Body Body
		{
			get
			{
				return m_body;
			}
			
		}
		/// <summary> Get the next fixture in the parent body's fixture list.
		/// 
		/// </summary>
		/// <returns> the next shape.
		/// </returns>
		/// <returns>
		/// </returns>
		virtual public Fixture Next
		{
			get
			{
				return m_next;
			}
			
		}
		virtual public float Density
		{
			get
			{
				return m_density;
			}
			
			set
			{
				assert(value >= 0f);
				m_density = value;
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary> Get the user data that was assigned in the fixture definition. Use this to store your
		/// application specific data.
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		/// <summary> Set the user data. Use this to store your application specific data.
		/// 
		/// </summary>
		/// <param name="data">
		/// </param>
		virtual public System.Object UserData
		{
			get
			{
				return m_userData;
			}
			
			set
			{
				m_userData = value;
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary> Get the coefficient of friction.
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		/// <summary> Set the coefficient of friction. This will _not_ change the friction of existing contacts.
		/// 
		/// </summary>
		/// <param name="friction">
		/// </param>
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
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary> Get the coefficient of restitution.
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		/// <summary> Set the coefficient of restitution. This will _not_ change the restitution of existing
		/// contacts.
		/// 
		/// </summary>
		/// <param name="restitution">
		/// </param>
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
		
		public float m_density;
		
		public Fixture m_next;
		public Body m_body;
		
		public Shape m_shape;
		
		public float m_friction;
		public float m_restitution;
		
		public FixtureProxy[] m_proxies;
		public int m_proxyCount;
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'm_filter '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Filter m_filter;
		
		public bool m_isSensor;
		
		public System.Object m_userData;
		
		public Fixture()
		{
			m_userData = null;
			m_body = null;
			m_next = null;
			m_proxies = null;
			m_proxyCount = 0;
			m_shape = null;
			m_filter = new Filter();
		}
		
		/// <summary> Call this if you want to establish collision that was previously disabled by
		/// ContactFilter::ShouldCollide.
		/// </summary>
		public virtual void  refilter()
		{
			if (m_body == null)
			{
				return ;
			}
			
			// Flag associated contacts for filtering.
			ContactEdge edge = m_body.ContactList;
			while (edge != null)
			{
				Contact contact = edge.contact;
				Fixture fixtureA = contact.FixtureA;
				Fixture fixtureB = contact.FixtureB;
				if (fixtureA == this || fixtureB == this)
				{
					contact.flagForFiltering();
				}
				edge = edge.next;
			}
			
			World world = m_body.World;
			
			if (world == null)
			{
				return ;
			}
			
			// Touch each proxy so that new pairs may be created
			BroadPhase broadPhase = world.m_contactManager.m_broadPhase;
			for (int i = 0; i < m_proxyCount; ++i)
			{
				broadPhase.touchProxy(m_proxies[i].proxyId);
			}
		}
		
		/// <summary> Test a point for containment in this fixture. This only works for convex shapes.
		/// 
		/// </summary>
		/// <param name="p">a point in world coordinates.
		/// </param>
		/// <returns>
		/// </returns>
		public virtual bool testPoint(Vec2 p)
		{
			return m_shape.testPoint(m_body.m_xf, p);
		}
		
		/// <summary> Cast a ray against this shape.
		/// 
		/// </summary>
		/// <param name="output">the ray-cast results.
		/// </param>
		/// <param name="input">the ray-cast input parameters.
		/// </param>
		/// <param name="output">
		/// </param>
		/// <param name="input">
		/// </param>
		public virtual bool raycast(RayCastOutput output, RayCastInput input, int childIndex)
		{
			return m_shape.raycast(output, input, m_body.m_xf, childIndex);
		}
		
		/// <summary> Get the mass data for this fixture. The mass data is based on the density and the shape. The
		/// rotational inertia is about the shape's origin.
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		public virtual void  getMassData(MassData massData)
		{
			m_shape.computeMass(massData, m_density);
		}
		
		/// <summary> Get the fixture's AABB. This AABB may be enlarge and/or stale. If you need a more accurate
		/// AABB, compute it using the shape and the body transform.
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		public virtual AABB getAABB(int childIndex)
		{
			assert(childIndex >= 0 && childIndex < m_proxyCount);
			return m_proxies[childIndex].aabb;
		}
		
		/// <summary> Dump this fixture to the log file.
		/// 
		/// </summary>
		/// <param name="bodyIndex">
		/// </param>
		public virtual void  dump(int bodyIndex)
		{
			
		}
		
		
		// We need separation create/destroy functions from the constructor/destructor because
		// the destructor cannot access the allocator (no destructor arguments allowed by C++).
		
		public virtual void  create(Body body, FixtureDef def)
		{
			m_userData = def.userData;
			m_friction = def.friction;
			m_restitution = def.restitution;
			
			m_body = body;
			m_next = null;
			
			
			m_filter.set_Renamed(def.filter);
			
			m_isSensor = def.isSensor;
			
			m_shape = def.shape.Clone();
			
			// Reserve proxy space
			int childCount = m_shape.ChildCount;
			if (m_proxies == null)
			{
				m_proxies = new FixtureProxy[childCount];
				for (int i = 0; i < childCount; i++)
				{
					m_proxies[i] = new FixtureProxy();
					m_proxies[i].fixture = null;
					m_proxies[i].proxyId = BroadPhase.NULL_PROXY;
				}
			}
			
			if (m_proxies.Length < childCount)
			{
				FixtureProxy[] old = m_proxies;
				int newLen = MathUtils.max(old.Length * 2, childCount);
				m_proxies = new FixtureProxy[newLen];
				Array.Copy(old, 0, m_proxies, 0, old.Length);
				for (int i = 0; i < newLen; i++)
				{
					if (i >= old.Length)
					{
						m_proxies[i] = new FixtureProxy();
					}
					m_proxies[i].fixture = null;
					m_proxies[i].proxyId = BroadPhase.NULL_PROXY;
				}
			}
			m_proxyCount = 0;
			
			m_density = def.density;
		}
		
		public virtual void  destroy()
		{
			// The proxies must be destroyed before calling this.
			assert(m_proxyCount == 0);
			
			// Free the child shape.
			m_shape = null;
			m_proxies = null;
			m_next = null;
			
			// TODO pool shapes
			// TODO pool fixtures
		}
		
		// These support body activation/deactivation.
		public virtual void  createProxies(BroadPhase broadPhase, Transform xf)
		{
			assert(m_proxyCount == 0);
			
			// Create proxies in the broad-phase.
			m_proxyCount = m_shape.ChildCount;
			
			for (int i = 0; i < m_proxyCount; ++i)
			{
				FixtureProxy proxy = m_proxies[i];
				m_shape.computeAABB(proxy.aabb, xf, i);
				proxy.proxyId = broadPhase.createProxy(proxy.aabb, proxy);
				proxy.fixture = this;
				proxy.childIndex = i;
			}
		}
		
		/// <summary> Internal method
		/// 
		/// </summary>
		/// <param name="broadPhase">
		/// </param>
		public virtual void  destroyProxies(BroadPhase broadPhase)
		{
			// Destroy proxies in the broad-phase.
			for (int i = 0; i < m_proxyCount; ++i)
			{
				FixtureProxy proxy = m_proxies[i];
				broadPhase.destroyProxy(proxy.proxyId);
				proxy.proxyId = BroadPhase.NULL_PROXY;
			}
			
			m_proxyCount = 0;
		}
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'pool1 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private AABB pool1 = new AABB();
		//UPGRADE_NOTE: Final was removed from the declaration of 'pool2 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private AABB pool2 = new AABB();
		//UPGRADE_NOTE: Final was removed from the declaration of 'displacement '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Vec2 displacement = new Vec2();
		
		/// <summary> Internal method
		/// 
		/// </summary>
		/// <param name="broadPhase">
		/// </param>
		/// <param name="xf1">
		/// </param>
		/// <param name="xf2">
		/// </param>
		protected internal virtual void  synchronize(BroadPhase broadPhase, Transform transform1, Transform transform2)
		{
			if (m_proxyCount == 0)
			{
				return ;
			}
			
			for (int i = 0; i < m_proxyCount; ++i)
			{
				FixtureProxy proxy = m_proxies[i];
				
				// Compute an AABB that covers the swept shape (may miss some rotation effect).
				//UPGRADE_NOTE: Final was removed from the declaration of 'aabb1 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
				AABB aabb1 = pool1;
				//UPGRADE_NOTE: Final was removed from the declaration of 'aab '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
				AABB aab = pool2;
				m_shape.computeAABB(aabb1, transform1, proxy.childIndex);
				m_shape.computeAABB(aab, transform2, proxy.childIndex);
				
				proxy.aabb.lowerBound.x = aabb1.lowerBound.x < aab.lowerBound.x?aabb1.lowerBound.x:aab.lowerBound.x;
				proxy.aabb.lowerBound.y = aabb1.lowerBound.y < aab.lowerBound.y?aabb1.lowerBound.y:aab.lowerBound.y;
				proxy.aabb.upperBound.x = aabb1.upperBound.x > aab.upperBound.x?aabb1.upperBound.x:aab.upperBound.x;
				proxy.aabb.upperBound.y = aabb1.upperBound.y > aab.upperBound.y?aabb1.upperBound.y:aab.upperBound.y;
				displacement.x = transform2.p.x - transform1.p.x;
				displacement.y = transform2.p.y - transform1.p.y;
				
				broadPhase.moveProxy(proxy.proxyId, proxy.aabb, displacement);
			}
		}
	}
}