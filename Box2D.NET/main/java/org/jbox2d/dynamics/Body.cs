/// <summary>****************************************************************************
/// Copyright (c) 2011, Daniel Murphy All rights reserved.
/// 
/// Redistribution and use in source and binary forms, with or without modification, are permitted
/// provided that the following conditions are met: * Redistributions of source code must retain the
/// above copyright notice, this list of conditions and the following disclaimer. * Redistributions
/// in binary form must reproduce the above copyright notice, this list of conditions and the
/// following disclaimer in the documentation and/or other materials provided with the distribution.
/// 
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
/// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
/// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
/// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
/// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
/// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
/// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
/// WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
/// ****************************************************************************
/// </summary>
using System;
using BroadPhase = org.jbox2d.collision.broadphase.BroadPhase;
using MassData = org.jbox2d.collision.shapes.MassData;
using Shape = org.jbox2d.collision.shapes.Shape;
using MathUtils = org.jbox2d.common.MathUtils;
using Rot = org.jbox2d.common.Rot;
using Sweep = org.jbox2d.common.Sweep;
using Transform = org.jbox2d.common.Transform;
using Vec2 = org.jbox2d.common.Vec2;
using Contact = org.jbox2d.dynamics.contacts.Contact;
using ContactEdge = org.jbox2d.dynamics.contacts.ContactEdge;
using JointEdge = org.jbox2d.dynamics.joints.JointEdge;
namespace org.jbox2d.dynamics
{
	
	/// <summary> A rigid body. These are created via World.createBody.
	/// 
	/// </summary>
	/// <author>  Daniel Murphy
	/// </author>
	public class Body
	{
		/// <summary> Get the world body origin position. Do not modify.
		/// 
		/// </summary>
		/// <returns> the world position of the body's origin.
		/// </returns>
		virtual public Vec2 Position
		{
			get
			{
				return m_xf.p;
			}
			
		}
		/// <summary> Get the angle in radians.
		/// 
		/// </summary>
		/// <returns> the current world rotation angle in radians.
		/// </returns>
		virtual public float Angle
		{
			get
			{
				return m_sweep.a;
			}
			
		}
		/// <summary> Get the world position of the center of mass. Do not modify.</summary>
		virtual public Vec2 WorldCenter
		{
			get
			{
				return m_sweep.c;
			}
			
		}
		/// <summary> Get the local position of the center of mass. Do not modify.</summary>
		virtual public Vec2 LocalCenter
		{
			get
			{
				return m_sweep.localCenter;
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary> Get the linear velocity of the center of mass. Do not modify, instead use
		/// {@link #setLinearVelocity(Vec2)}.
		/// 
		/// </summary>
		/// <returns> the linear velocity of the center of mass.
		/// </returns>
		/// <summary> Set the linear velocity of the center of mass.
		/// 
		/// </summary>
		/// <param name="v">the new linear velocity of the center of mass.
		/// </param>
		virtual public Vec2 LinearVelocity
		{
			get
			{
				return m_linearVelocity;
			}
			
			set
			{
				if (m_type == BodyType.STATIC)
				{
					return ;
				}
				
				if (Vec2.dot(value, value) > 0.0f)
				{
					Awake = true;
				}
				
				m_linearVelocity.set_Renamed(value);
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary> Get the angular velocity.
		/// 
		/// </summary>
		/// <returns> the angular velocity in radians/second.
		/// </returns>
		/// <summary> Set the angular velocity.
		/// 
		/// </summary>
		/// <param name="omega">the new angular velocity in radians/second.
		/// </param>
		virtual public float AngularVelocity
		{
			get
			{
				return m_angularVelocity;
			}
			
			set
			{
				if (m_type == BodyType.STATIC)
				{
					return ;
				}
				
				if (value * value > 0f)
				{
					Awake = true;
				}
				
				m_angularVelocity = value;
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary> Get the gravity scale of the body.
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		/// <summary> Set the gravity scale of the body.
		/// 
		/// </summary>
		/// <param name="gravityScale">
		/// </param>
		virtual public float GravityScale
		{
			get
			{
				return m_gravityScale;
			}
			
			set
			{
				this.m_gravityScale = value;
			}
			
		}
		/// <summary> Get the total mass of the body.
		/// 
		/// </summary>
		/// <returns> the mass, usually in kilograms (kg).
		/// </returns>
		virtual public float Mass
		{
			get
			{
				return m_mass;
			}
			
		}
		/// <summary> Get the central rotational inertia of the body.
		/// 
		/// </summary>
		/// <returns> the rotational inertia, usually in kg-m^2.
		/// </returns>
		virtual public float Inertia
		{
			get
			{
				return m_I + m_mass * (m_sweep.localCenter.x * m_sweep.localCenter.x + m_sweep.localCenter.y * m_sweep.localCenter.y);
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary>Get the linear damping of the body. </summary>
		/// <summary>Set the linear damping of the body. </summary>
		virtual public float LinearDamping
		{
			get
			{
				return m_linearDamping;
			}
			
			set
			{
				m_linearDamping = value;
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary>Get the angular damping of the body. </summary>
		/// <summary>Set the angular damping of the body. </summary>
		virtual public float AngularDamping
		{
			get
			{
				return m_angularDamping;
			}
			
			set
			{
				m_angularDamping = value;
			}
			
		}
		/// <summary> Set the type of this body. This may alter the mass and velocity.
		/// 
		/// </summary>
		/// <param name="type">
		/// </param>
		virtual public BodyType Type
		{
			get
			{
				return m_type;
			}
			
			set
			{
				assert(m_world.Locked == false);
				if (m_world.Locked == true)
				{
					return ;
				}
				
				if (m_type == value)
				{
					return ;
				}
				
				m_type = value;
				
				resetMassData();
				
				if (m_type == BodyType.STATIC)
				{
					m_linearVelocity.setZero();
					m_angularVelocity = 0.0f;
					m_sweep.a0 = m_sweep.a;
					m_sweep.c0.set_Renamed(m_sweep.c);
					synchronizeFixtures();
				}
				
				Awake = true;
				
				m_force.setZero();
				m_torque = 0.0f;
				
				// Delete the attached contacts.
				ContactEdge ce = m_contactList;
				while (ce != null)
				{
					ContactEdge ce0 = ce;
					ce = ce.next;
					m_world.m_contactManager.destroy(ce0.contact);
				}
				m_contactList = null;
				
				// Touch the proxies so that new contacts will be created (when appropriate)
				BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
				for (Fixture f = m_fixtureList; f != null; f = f.m_next)
				{
					int proxyCount = f.m_proxyCount;
					for (int i = 0; i < proxyCount; ++i)
					{
						broadPhase.touchProxy(f.m_proxies[i].proxyId);
					}
				}
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary>Is this body treated like a bullet for continuous collision detection? </summary>
		/// <summary>Should this body be treated like a bullet for continuous collision detection? </summary>
		virtual public bool Bullet
		{
			get
			{
				return (m_flags & e_bulletFlag) == e_bulletFlag;
			}
			
			set
			{
				if (value)
				{
					m_flags |= e_bulletFlag;
				}
				else
				{
					m_flags &= ~ e_bulletFlag;
				}
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary> Is this body allowed to sleep
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		/// <summary> You can disable sleeping on this body. If you disable sleeping, the body will be woken.
		/// 
		/// </summary>
		/// <param name="flag">
		/// </param>
		virtual public bool SleepingAllowed
		{
			get
			{
				return (m_flags & e_autoSleepFlag) == e_autoSleepFlag;
			}
			
			set
			{
				if (value)
				{
					m_flags |= e_autoSleepFlag;
				}
				else
				{
					m_flags &= ~ e_autoSleepFlag;
					Awake = true;
				}
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary> Get the sleeping state of this body.
		/// 
		/// </summary>
		/// <returns> true if the body is sleeping.
		/// </returns>
		/// <summary> Set the sleep state of the body. A sleeping body has very low CPU cost.
		/// 
		/// </summary>
		/// <param name="flag">set to true to put body to sleep, false to wake it.
		/// </param>
		/// <param name="flag">
		/// </param>
		virtual public bool Awake
		{
			get
			{
				return (m_flags & e_awakeFlag) == e_awakeFlag;
			}
			
			set
			{
				if (value)
				{
					if ((m_flags & e_awakeFlag) == 0)
					{
						m_flags |= e_awakeFlag;
						m_sleepTime = 0.0f;
					}
				}
				else
				{
					m_flags &= ~ e_awakeFlag;
					m_sleepTime = 0.0f;
					m_linearVelocity.setZero();
					m_angularVelocity = 0.0f;
					m_force.setZero();
					m_torque = 0.0f;
				}
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary> Get the active state of the body.
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		/// <summary> Set the active state of the body. An inactive body is not simulated and cannot be collided with
		/// or woken up. If you pass a flag of true, all fixtures will be added to the broad-phase. If you
		/// pass a flag of false, all fixtures will be removed from the broad-phase and all contacts will
		/// be destroyed. Fixtures and joints are otherwise unaffected. You may continue to create/destroy
		/// fixtures and joints on inactive bodies. Fixtures on an inactive body are implicitly inactive
		/// and will not participate in collisions, ray-casts, or queries. Joints connected to an inactive
		/// body are implicitly inactive. An inactive body is still owned by a World object and remains in
		/// the body list.
		/// 
		/// </summary>
		/// <param name="flag">
		/// </param>
		virtual public bool Active
		{
			get
			{
				return (m_flags & e_activeFlag) == e_activeFlag;
			}
			
			set
			{
				assert(m_world.Locked == false);
				
				if (value == Active)
				{
					return ;
				}
				
				if (value)
				{
					m_flags |= e_activeFlag;
					
					// Create all proxies.
					BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
					for (Fixture f = m_fixtureList; f != null; f = f.m_next)
					{
						f.createProxies(broadPhase, m_xf);
					}
					
					// Contacts are created the next time step.
				}
				else
				{
					m_flags &= ~ e_activeFlag;
					
					// Destroy all proxies.
					BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
					for (Fixture f = m_fixtureList; f != null; f = f.m_next)
					{
						f.destroyProxies(broadPhase);
					}
					
					// Destroy the attached contacts.
					ContactEdge ce = m_contactList;
					while (ce != null)
					{
						ContactEdge ce0 = ce;
						ce = ce.next;
						m_world.m_contactManager.destroy(ce0.contact);
					}
					m_contactList = null;
				}
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary> Does this body have fixed rotation?
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		/// <summary> Set this body to have fixed rotation. This causes the mass to be reset.
		/// 
		/// </summary>
		/// <param name="flag">
		/// </param>
		virtual public bool FixedRotation
		{
			get
			{
				return (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
			}
			
			set
			{
				if (value)
				{
					m_flags |= e_fixedRotationFlag;
				}
				else
				{
					m_flags &= ~ e_fixedRotationFlag;
				}
				
				resetMassData();
			}
			
		}
		/// <summary>Get the list of all fixtures attached to this body. </summary>
		virtual public Fixture FixtureList
		{
			get
			{
				return m_fixtureList;
			}
			
		}
		/// <summary>Get the list of all joints attached to this body. </summary>
		virtual public JointEdge JointList
		{
			get
			{
				return m_jointList;
			}
			
		}
		/// <summary> Get the list of all contacts attached to this body.
		/// 
		/// </summary>
		/// <warning>  this list changes during the time step and you may miss some collisions if you don't </warning>
		/// <summary>          use ContactListener.
		/// </summary>
		virtual public ContactEdge ContactList
		{
			get
			{
				return m_contactList;
			}
			
		}
		/// <summary>Get the next body in the world's body list. </summary>
		virtual public Body Next
		{
			get
			{
				return m_next;
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary>Get the user data pointer that was provided in the body definition. </summary>
		/// <summary> Set the user data. Use this to store your application specific data.</summary>
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
		/// <summary> Get the parent world of this body.</summary>
		virtual public World World
		{
			get
			{
				return m_world;
			}
			
		}
		public const int e_islandFlag = 0x0001;
		public const int e_awakeFlag = 0x0002;
		public const int e_autoSleepFlag = 0x0004;
		public const int e_bulletFlag = 0x0008;
		public const int e_fixedRotationFlag = 0x0010;
		public const int e_activeFlag = 0x0020;
		public const int e_toiFlag = 0x0040;
		
		public BodyType m_type;
		
		public int m_flags;
		
		public int m_islandIndex;
		
		/// <summary> The body origin transform.</summary>
		//UPGRADE_NOTE: Final was removed from the declaration of 'm_xf '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Transform m_xf = new Transform();
		
		/// <summary> The swept motion for CCD</summary>
		//UPGRADE_NOTE: Final was removed from the declaration of 'm_sweep '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Sweep m_sweep = new Sweep();
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'm_linearVelocity '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Vec2 m_linearVelocity = new Vec2();
		public float m_angularVelocity = 0;
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'm_force '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Vec2 m_force = new Vec2();
		public float m_torque = 0;
		
		public World m_world;
		public Body m_prev;
		public Body m_next;
		
		public Fixture m_fixtureList;
		public int m_fixtureCount;
		
		public JointEdge m_jointList;
		public ContactEdge m_contactList;
		
		public float m_mass, m_invMass;
		
		// Rotational inertia about the center of mass.
		public float m_I, m_invI;
		
		public float m_linearDamping;
		public float m_angularDamping;
		public float m_gravityScale;
		
		public float m_sleepTime;
		
		public System.Object m_userData;
		
		public Body(BodyDef bd, World world)
		{
			assert(bd.position.Valid);
			assert(bd.linearVelocity.Valid);
			assert(bd.gravityScale >= 0.0f);
			assert(bd.angularDamping >= 0.0f);
			assert(bd.linearDamping >= 0.0f);
			
			m_flags = 0;
			
			if (bd.bullet)
			{
				m_flags |= e_bulletFlag;
			}
			if (bd.fixedRotation)
			{
				m_flags |= e_fixedRotationFlag;
			}
			if (bd.allowSleep)
			{
				m_flags |= e_autoSleepFlag;
			}
			if (bd.awake)
			{
				m_flags |= e_awakeFlag;
			}
			if (bd.active)
			{
				m_flags |= e_activeFlag;
			}
			
			m_world = world;
			
			m_xf.p.set_Renamed(bd.position);
			m_xf.q.set_Renamed(bd.angle);
			
			m_sweep.localCenter.setZero();
			m_sweep.c0.set_Renamed(m_xf.p);
			m_sweep.c.set_Renamed(m_xf.p);
			m_sweep.a0 = bd.angle;
			m_sweep.a = bd.angle;
			m_sweep.alpha0 = 0.0f;
			
			m_jointList = null;
			m_contactList = null;
			m_prev = null;
			m_next = null;
			
			m_linearVelocity.set_Renamed(bd.linearVelocity);
			m_angularVelocity = bd.angularVelocity;
			
			m_linearDamping = bd.linearDamping;
			m_angularDamping = bd.angularDamping;
			m_gravityScale = bd.gravityScale;
			
			m_force.setZero();
			m_torque = 0.0f;
			
			m_sleepTime = 0.0f;
			
			m_type = bd.type;
			
			if (m_type == BodyType.DYNAMIC)
			{
				m_mass = 1f;
				m_invMass = 1f;
			}
			else
			{
				m_mass = 0f;
				m_invMass = 0f;
			}
			
			m_I = 0.0f;
			m_invI = 0.0f;
			
			m_userData = bd.userData;
			
			m_fixtureList = null;
			m_fixtureCount = 0;
		}
		
		// TODO djm: check out about this new fixture here
		/// <summary> Creates a fixture and attach it to this body. Use this function if you need to set some fixture
		/// parameters, like friction. Otherwise you can create the fixture directly from a shape. If the
		/// density is non-zero, this function automatically updates the mass of the body. Contacts are not
		/// created until the next time step.
		/// 
		/// </summary>
		/// <param name="def">the fixture definition.
		/// </param>
		/// <warning>  This function is locked during callbacks. </warning>
		public Fixture createFixture(FixtureDef def)
		{
			assert(m_world.Locked == false);
			
			if (m_world.Locked == true)
			{
				return null;
			}
			
			// djm TODO from pool?
			Fixture fixture = new Fixture();
			fixture.create(this, def);
			
			if ((m_flags & e_activeFlag) == e_activeFlag)
			{
				BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
				fixture.createProxies(broadPhase, m_xf);
			}
			
			fixture.m_next = m_fixtureList;
			m_fixtureList = fixture;
			++m_fixtureCount;
			
			fixture.m_body = this;
			
			// Adjust mass properties if needed.
			if (fixture.m_density > 0.0f)
			{
				resetMassData();
			}
			
			// Let the world know we have a new fixture. This will cause new contacts
			// to be created at the beginning of the next time step.
			m_world.m_flags |= World.NEW_FIXTURE;
			
			return fixture;
		}
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'fixDef '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private FixtureDef fixDef = new FixtureDef();
		
		/// <summary> Creates a fixture from a shape and attach it to this body. This is a convenience function. Use
		/// FixtureDef if you need to set parameters like friction, restitution, user data, or filtering.
		/// If the density is non-zero, this function automatically updates the mass of the body.
		/// 
		/// </summary>
		/// <param name="shape">the shape to be cloned.
		/// </param>
		/// <param name="density">the shape density (set to zero for static bodies).
		/// </param>
		/// <warning>  This function is locked during callbacks. </warning>
		public Fixture createFixture(Shape shape, float density)
		{
			fixDef.shape = shape;
			fixDef.density = density;
			
			return createFixture(fixDef);
		}
		
		/// <summary> Destroy a fixture. This removes the fixture from the broad-phase and destroys all contacts
		/// associated with this fixture. This will automatically adjust the mass of the body if the body
		/// is dynamic and the fixture has positive density. All fixtures attached to a body are implicitly
		/// destroyed when the body is destroyed.
		/// 
		/// </summary>
		/// <param name="fixture">the fixture to be removed.
		/// </param>
		/// <warning>  This function is locked during callbacks. </warning>
		public void  destroyFixture(Fixture fixture)
		{
			assert(m_world.Locked == false);
			if (m_world.Locked == true)
			{
				return ;
			}
			
			assert(fixture.m_body == this);
			
			// Remove the fixture from this body's singly linked list.
			assert(m_fixtureCount > 0);
			Fixture node = m_fixtureList;
			Fixture last = null; // java change
			bool found = false;
			while (node != null)
			{
				if (node == fixture)
				{
					node = fixture.m_next;
					found = true;
					break;
				}
				last = node;
				node = node.m_next;
			}
			
			// You tried to remove a shape that is not attached to this body.
			assert(found);
			
			// java change, remove it from the list
			if (last == null)
			{
				m_fixtureList = fixture.m_next;
			}
			else
			{
				last.m_next = fixture.m_next;
			}
			
			// Destroy any contacts associated with the fixture.
			ContactEdge edge = m_contactList;
			while (edge != null)
			{
				Contact c = edge.contact;
				edge = edge.next;
				
				Fixture fixtureA = c.FixtureA;
				Fixture fixtureB = c.FixtureB;
				
				if (fixture == fixtureA || fixture == fixtureB)
				{
					// This destroys the contact and removes it from
					// this body's contact list.
					m_world.m_contactManager.destroy(c);
				}
			}
			
			if ((m_flags & e_activeFlag) == e_activeFlag)
			{
				BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
				fixture.destroyProxies(broadPhase);
			}
			
			fixture.destroy();
			fixture.m_body = null;
			fixture.m_next = null;
			fixture = null;
			
			--m_fixtureCount;
			
			// Reset the mass data.
			resetMassData();
		}
		
		/// <summary> Set the position of the body's origin and rotation. This breaks any contacts and wakes the
		/// other bodies. Manipulating a body's transform may cause non-physical behavior.
		/// 
		/// </summary>
		/// <param name="position">the world position of the body's local origin.
		/// </param>
		/// <param name="angle">the world rotation in radians.
		/// </param>
		public void  setTransform(Vec2 position, float angle)
		{
			assert(m_world.Locked == false);
			if (m_world.Locked == true)
			{
				return ;
			}
			
			m_xf.q.set_Renamed(angle);
			m_xf.p.set_Renamed(position);
			
			// m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
			Transform.mulToOutUnsafe(m_xf, m_sweep.localCenter, m_sweep.c);
			m_sweep.a = angle;
			
			m_sweep.c0.set_Renamed(m_sweep.c);
			m_sweep.a0 = m_sweep.a;
			
			BroadPhase broadPhase = m_world.m_contactManager.m_broadPhase;
			for (Fixture f = m_fixtureList; f != null; f = f.m_next)
			{
				f.synchronize(broadPhase, m_xf, m_xf);
			}
			
			m_world.m_contactManager.findNewContacts();
		}
		
		/// <summary> Get the body transform for the body's origin.
		/// 
		/// </summary>
		/// <returns> the world transform of the body's origin.
		/// </returns>
		public Transform getTransform()
		{
			return m_xf;
		}
		
		/// <summary> Apply a force at a world point. If the force is not applied at the center of mass, it will
		/// generate a torque and affect the angular velocity. This wakes up the body.
		/// 
		/// </summary>
		/// <param name="force">the world force vector, usually in Newtons (N).
		/// </param>
		/// <param name="point">the world position of the point of application.
		/// </param>
		public void  applyForce(Vec2 force, Vec2 point)
		{
			if (m_type != BodyType.DYNAMIC)
			{
				return ;
			}
			
			if (Awake == false)
			{
				Awake = true;
			}
			
			// m_force.addLocal(force);
			// Vec2 temp = tltemp.get();
			// temp.set(point).subLocal(m_sweep.c);
			// m_torque += Vec2.cross(temp, force);
			
			m_force.x += force.x;
			m_force.y += force.y;
			
			m_torque += (point.x - m_sweep.c.x) * force.y - (point.y - m_sweep.c.y) * force.x;
		}
		
		/// <summary> Apply a force to the center of mass. This wakes up the body.
		/// 
		/// </summary>
		/// <param name="force">the world force vector, usually in Newtons (N).
		/// </param>
		public void  applyForceToCenter(Vec2 force)
		{
			if (m_type != BodyType.DYNAMIC)
			{
				return ;
			}
			
			if (Awake == false)
			{
				Awake = true;
			}
			
			m_force.x += force.x;
			m_force.y += force.y;
		}
		
		/// <summary> Apply a torque. This affects the angular velocity without affecting the linear velocity of the
		/// center of mass. This wakes up the body.
		/// 
		/// </summary>
		/// <param name="torque">about the z-axis (out of the screen), usually in N-m.
		/// </param>
		public void  applyTorque(float torque)
		{
			if (m_type != BodyType.DYNAMIC)
			{
				return ;
			}
			
			if (Awake == false)
			{
				Awake = true;
			}
			
			m_torque += torque;
		}
		
		/// <summary> Apply an impulse at a point. This immediately modifies the velocity. It also modifies the
		/// angular velocity if the point of application is not at the center of mass. This wakes up the
		/// body.
		/// 
		/// </summary>
		/// <param name="impulse">the world impulse vector, usually in N-seconds or kg-m/s.
		/// </param>
		/// <param name="point">the world position of the point of application.
		/// </param>
		public void  applyLinearImpulse(Vec2 impulse, Vec2 point)
		{
			if (m_type != BodyType.DYNAMIC)
			{
				return ;
			}
			
			if (Awake == false)
			{
				Awake = true;
			}
			
			// Vec2 temp = tltemp.get();
			// temp.set(impulse).mulLocal(m_invMass);
			// m_linearVelocity.addLocal(temp);
			//
			// temp.set(point).subLocal(m_sweep.c);
			// m_angularVelocity += m_invI * Vec2.cross(temp, impulse);
			
			m_linearVelocity.x += impulse.x * m_invMass;
			m_linearVelocity.y += impulse.y * m_invMass;
			
			m_angularVelocity += m_invI * ((point.x - m_sweep.c.x) * impulse.y - (point.y - m_sweep.c.y) * impulse.x);
		}
		
		/// <summary> Apply an angular impulse.
		/// 
		/// </summary>
		/// <param name="impulse">the angular impulse in units of kg*m*m/s
		/// </param>
		public virtual void  applyAngularImpulse(float impulse)
		{
			if (m_type != BodyType.DYNAMIC)
			{
				return ;
			}
			
			if (Awake == false)
			{
				Awake = true;
			}
			m_angularVelocity += m_invI * impulse;
		}
		
		/// <summary> Get the mass data of the body. The rotational inertia is relative to the center of mass.
		/// 
		/// </summary>
		/// <returns> a struct containing the mass, inertia and center of the body.
		/// </returns>
		public void  getMassData(MassData data)
		{
			// data.mass = m_mass;
			// data.I = m_I + m_mass * Vec2.dot(m_sweep.localCenter, m_sweep.localCenter);
			// data.center.set(m_sweep.localCenter);
			
			data.mass = m_mass;
			data.I = m_I + m_mass * (m_sweep.localCenter.x * m_sweep.localCenter.x + m_sweep.localCenter.y * m_sweep.localCenter.y);
			data.center.x = m_sweep.localCenter.x;
			data.center.y = m_sweep.localCenter.y;
		}
		
		/// <summary> Set the mass properties to override the mass properties of the fixtures. Note that this changes
		/// the center of mass position. Note that creating or destroying fixtures can also alter the mass.
		/// This function has no effect if the body isn't dynamic.
		/// 
		/// </summary>
		/// <param name="massData">the mass properties.
		/// </param>
		public void  setMassData(MassData massData)
		{
			// TODO_ERIN adjust linear velocity and torque to account for movement of center.
			assert(m_world.Locked == false);
			if (m_world.Locked == true)
			{
				return ;
			}
			
			if (m_type != BodyType.DYNAMIC)
			{
				return ;
			}
			
			m_invMass = 0.0f;
			m_I = 0.0f;
			m_invI = 0.0f;
			
			m_mass = massData.mass;
			if (m_mass <= 0.0f)
			{
				m_mass = 1f;
			}
			
			m_invMass = 1.0f / m_mass;
			
			if (massData.I > 0.0f && (m_flags & e_fixedRotationFlag) == 0)
			{
				m_I = massData.I - m_mass * Vec2.dot(massData.center, massData.center);
				assert(m_I > 0.0f);
				m_invI = 1.0f / m_I;
			}
			
			//UPGRADE_NOTE: Final was removed from the declaration of 'oldCenter '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Vec2 oldCenter = m_world.Pool.popVec2();
			// Move center of mass.
			oldCenter.set_Renamed(m_sweep.c);
			m_sweep.localCenter.set_Renamed(massData.center);
			// m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
			Transform.mulToOutUnsafe(m_xf, m_sweep.localCenter, m_sweep.c0);
			m_sweep.c.set_Renamed(m_sweep.c0);
			
			// Update center of mass velocity.
			// m_linearVelocity += Cross(m_angularVelocity, m_sweep.c - oldCenter);
			//UPGRADE_NOTE: Final was removed from the declaration of 'temp '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Vec2 temp = m_world.Pool.popVec2();
			temp.set_Renamed(m_sweep.c).subLocal(oldCenter);
			Vec2.crossToOut(m_angularVelocity, temp, temp);
			m_linearVelocity.addLocal(temp);
			
			m_world.Pool.pushVec2(2);
		}
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'pmd '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private MassData pmd = new MassData();
		
		/// <summary> This resets the mass properties to the sum of the mass properties of the fixtures. This
		/// normally does not need to be called unless you called setMassData to override the mass and you
		/// later want to reset the mass.
		/// </summary>
		public void  resetMassData()
		{
			// Compute mass data from shapes. Each shape has its own density.
			m_mass = 0.0f;
			m_invMass = 0.0f;
			m_I = 0.0f;
			m_invI = 0.0f;
			m_sweep.localCenter.setZero();
			
			// Static and kinematic bodies have zero mass.
			if (m_type == BodyType.STATIC || m_type == BodyType.KINEMATIC)
			{
				// m_sweep.c0 = m_sweep.c = m_xf.position;
				m_sweep.c0.set_Renamed(m_xf.p);
				m_sweep.c.set_Renamed(m_xf.p);
				m_sweep.a0 = m_sweep.a;
				return ;
			}
			
			assert(m_type == BodyType.DYNAMIC);
			
			// Accumulate mass over all fixtures.
			//UPGRADE_NOTE: Final was removed from the declaration of 'localCenter '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Vec2 localCenter = m_world.Pool.popVec2();
			localCenter.setZero();
			//UPGRADE_NOTE: Final was removed from the declaration of 'temp '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Vec2 temp = m_world.Pool.popVec2();
			//UPGRADE_NOTE: Final was removed from the declaration of 'massData '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			MassData massData = pmd;
			for (Fixture f = m_fixtureList; f != null; f = f.m_next)
			{
				if (f.m_density == 0.0f)
				{
					continue;
				}
				f.getMassData(massData);
				m_mass += massData.mass;
				// center += massData.mass * massData.center;
				temp.set_Renamed(massData.center).mulLocal(massData.mass);
				localCenter.addLocal(temp);
				m_I += massData.I;
			}
			
			// Compute center of mass.
			if (m_mass > 0.0f)
			{
				m_invMass = 1.0f / m_mass;
				localCenter.mulLocal(m_invMass);
			}
			else
			{
				// Force all dynamic bodies to have a positive mass.
				m_mass = 1.0f;
				m_invMass = 1.0f;
			}
			
			if (m_I > 0.0f && (m_flags & e_fixedRotationFlag) == 0)
			{
				// Center the inertia about the center of mass.
				m_I -= m_mass * Vec2.dot(localCenter, localCenter);
				assert(m_I > 0.0f);
				m_invI = 1.0f / m_I;
			}
			else
			{
				m_I = 0.0f;
				m_invI = 0.0f;
			}
			
			Vec2 oldCenter = m_world.Pool.popVec2();
			// Move center of mass.
			oldCenter.set_Renamed(m_sweep.c);
			m_sweep.localCenter.set_Renamed(localCenter);
			// m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
			Transform.mulToOutUnsafe(m_xf, m_sweep.localCenter, m_sweep.c0);
			m_sweep.c.set_Renamed(m_sweep.c0);
			
			// Update center of mass velocity.
			// m_linearVelocity += Cross(m_angularVelocity, m_sweep.c - oldCenter);
			temp.set_Renamed(m_sweep.c).subLocal(oldCenter);
			
			//UPGRADE_NOTE: Final was removed from the declaration of 'temp2 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Vec2 temp2 = oldCenter;
			Vec2.crossToOutUnsafe(m_angularVelocity, temp, temp2);
			m_linearVelocity.addLocal(temp2);
			
			m_world.Pool.pushVec2(3);
		}
		
		/// <summary> Get the world coordinates of a point given the local coordinates.
		/// 
		/// </summary>
		/// <param name="localPoint">a point on the body measured relative the the body's origin.
		/// </param>
		/// <returns> the same point expressed in world coordinates.
		/// </returns>
		public Vec2 getWorldPoint(Vec2 localPoint)
		{
			Vec2 v = new Vec2();
			getWorldPointToOut(localPoint, v);
			return v;
		}
		
		public void  getWorldPointToOut(Vec2 localPoint, Vec2 out_Renamed)
		{
			Transform.mulToOut(m_xf, localPoint, out_Renamed);
		}
		
		/// <summary> Get the world coordinates of a vector given the local coordinates.
		/// 
		/// </summary>
		/// <param name="localVector">a vector fixed in the body.
		/// </param>
		/// <returns> the same vector expressed in world coordinates.
		/// </returns>
		public Vec2 getWorldVector(Vec2 localVector)
		{
			Vec2 out_Renamed = new Vec2();
			getWorldVectorToOut(localVector, out_Renamed);
			return out_Renamed;
		}
		
		public void  getWorldVectorToOut(Vec2 localVector, Vec2 out_Renamed)
		{
			Rot.mulToOut(m_xf.q, localVector, out_Renamed);
		}
		
		public void  getWorldVectorToOutUnsafe(Vec2 localVector, Vec2 out_Renamed)
		{
			Rot.mulToOutUnsafe(m_xf.q, localVector, out_Renamed);
		}
		
		/// <summary> Gets a local point relative to the body's origin given a world point.
		/// 
		/// </summary>
		/// <param name="a">point in world coordinates.
		/// </param>
		/// <returns> the corresponding local point relative to the body's origin.
		/// </returns>
		public Vec2 getLocalPoint(Vec2 worldPoint)
		{
			Vec2 out_Renamed = new Vec2();
			getLocalPointToOut(worldPoint, out_Renamed);
			return out_Renamed;
		}
		
		public void  getLocalPointToOut(Vec2 worldPoint, Vec2 out_Renamed)
		{
			Transform.mulTransToOut(m_xf, worldPoint, out_Renamed);
		}
		
		/// <summary> Gets a local vector given a world vector.
		/// 
		/// </summary>
		/// <param name="a">vector in world coordinates.
		/// </param>
		/// <returns> the corresponding local vector.
		/// </returns>
		public Vec2 getLocalVector(Vec2 worldVector)
		{
			Vec2 out_Renamed = new Vec2();
			getLocalVectorToOut(worldVector, out_Renamed);
			return out_Renamed;
		}
		
		public void  getLocalVectorToOut(Vec2 worldVector, Vec2 out_Renamed)
		{
			Rot.mulTrans(m_xf.q, worldVector, out_Renamed);
		}
		
		public void  getLocalVectorToOutUnsafe(Vec2 worldVector, Vec2 out_Renamed)
		{
			Rot.mulTransUnsafe(m_xf.q, worldVector, out_Renamed);
		}
		
		/// <summary> Get the world linear velocity of a world point attached to this body.
		/// 
		/// </summary>
		/// <param name="a">point in world coordinates.
		/// </param>
		/// <returns> the world velocity of a point.
		/// </returns>
		public Vec2 getLinearVelocityFromWorldPoint(Vec2 worldPoint)
		{
			Vec2 out_Renamed = new Vec2();
			getLinearVelocityFromWorldPointToOut(worldPoint, out_Renamed);
			return out_Renamed;
		}
		
		public void  getLinearVelocityFromWorldPointToOut(Vec2 worldPoint, Vec2 out_Renamed)
		{
			out_Renamed.set_Renamed(worldPoint).subLocal(m_sweep.c);
			Vec2.crossToOut(m_angularVelocity, out_Renamed, out_Renamed);
			out_Renamed.addLocal(m_linearVelocity);
		}
		
		/// <summary> Get the world velocity of a local point.
		/// 
		/// </summary>
		/// <param name="a">point in local coordinates.
		/// </param>
		/// <returns> the world velocity of a point.
		/// </returns>
		public Vec2 getLinearVelocityFromLocalPoint(Vec2 localPoint)
		{
			Vec2 out_Renamed = new Vec2();
			getLinearVelocityFromLocalPointToOut(localPoint, out_Renamed);
			return out_Renamed;
		}
		
		public void  getLinearVelocityFromLocalPointToOut(Vec2 localPoint, Vec2 out_Renamed)
		{
			getWorldPointToOut(localPoint, out_Renamed);
			getLinearVelocityFromWorldPointToOut(out_Renamed, out_Renamed);
		}
		
		// djm pooling
		//UPGRADE_NOTE: Final was removed from the declaration of 'pxf '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Transform pxf = new Transform();
		
		protected internal void  synchronizeFixtures()
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'xf1 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Transform xf1 = pxf;
			// xf1.position = m_sweep.c0 - Mul(xf1.R, m_sweep.localCenter);
			
			// xf1.q.set(m_sweep.a0);
			// Rot.mulToOutUnsafe(xf1.q, m_sweep.localCenter, xf1.p);
			// xf1.p.mulLocal(-1).addLocal(m_sweep.c0);
			// inlined:
			xf1.q.s = MathUtils.sin(m_sweep.a0);
			xf1.q.c = MathUtils.cos(m_sweep.a0);
			xf1.p.x = m_sweep.c0.x - xf1.q.c * m_sweep.localCenter.x - xf1.q.s * m_sweep.localCenter.y;
			xf1.p.y = m_sweep.c0.y - xf1.q.s * m_sweep.localCenter.x + xf1.q.c * m_sweep.localCenter.y;
			// end inline
			
			for (Fixture f = m_fixtureList; f != null; f = f.m_next)
			{
				f.synchronize(m_world.m_contactManager.m_broadPhase, xf1, m_xf);
			}
		}
		
		public void  synchronizeTransform()
		{
			m_xf.q.set_Renamed(m_sweep.a);
			
			// m_xf.position = m_sweep.c - Mul(m_xf.R, m_sweep.localCenter);
			Rot.mulToOutUnsafe(m_xf.q, m_sweep.localCenter, m_xf.p);
			m_xf.p.mulLocal(- 1).addLocal(m_sweep.c);
			
			// final float c = MathUtils.cos(m_sweep.a), s = MathUtils.sin(m_sweep.a);
			// m_xf.q.ex.x = c;
			// m_xf.q.ey.x = -s;
			// m_xf.q.ex.y = s;
			// m_xf.q.ey.y = c;
			// m_xf.p.x = m_xf.q.ex.x * m_sweep.localCenter.x + m_xf.q.ey.x * m_sweep.localCenter.y;
			// m_xf.p.y = m_xf.q.ex.y * m_sweep.localCenter.x + m_xf.q.ey.y * m_sweep.localCenter.y;
			// m_xf.p.x *= (float) (-1);
			// m_xf.p.y *= (float) (-1);
			// m_xf.p.x += m_sweep.c.x;
			// m_xf.p.y += m_sweep.c.y;
		}
		
		/// <summary> This is used to prevent connected bodies from colliding. It may lie, depending on the
		/// collideConnected flag.
		/// 
		/// </summary>
		/// <param name="other">
		/// </param>
		/// <returns>
		/// </returns>
		public virtual bool shouldCollide(Body other)
		{
			// At least one body should be dynamic.
			if (m_type != BodyType.DYNAMIC && other.m_type != BodyType.DYNAMIC)
			{
				return false;
			}
			
			// Does a joint prevent collision?
			for (JointEdge jn = m_jointList; jn != null; jn = jn.next)
			{
				if (jn.other == other)
				{
					if (jn.joint.m_collideConnected == false)
					{
						return false;
					}
				}
			}
			
			return true;
		}
		
		protected internal void  advance(float t)
		{
			// Advance to the new safe time. This doesn't sync the broad-phase.
			m_sweep.advance(t);
			m_sweep.c.set_Renamed(m_sweep.c0);
			m_sweep.a = m_sweep.a0;
			m_xf.q.set_Renamed(m_sweep.a);
			// m_xf.position = m_sweep.c - Mul(m_xf.R, m_sweep.localCenter);
			Rot.mulToOutUnsafe(m_xf.q, m_sweep.localCenter, m_xf.p);
			m_xf.p.mulLocal(- 1).addLocal(m_sweep.c);
		}
	}
}