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
using ContactFilter = org.jbox2d.callbacks.ContactFilter;
using ContactListener = org.jbox2d.callbacks.ContactListener;
using DebugDraw = org.jbox2d.callbacks.DebugDraw;
using DestructionListener = org.jbox2d.callbacks.DestructionListener;
using QueryCallback = org.jbox2d.callbacks.QueryCallback;
using RayCastCallback = org.jbox2d.callbacks.RayCastCallback;
using TreeCallback = org.jbox2d.callbacks.TreeCallback;
using TreeRayCastCallback = org.jbox2d.callbacks.TreeRayCastCallback;
using AABB = org.jbox2d.collision.AABB;
using RayCastInput = org.jbox2d.collision.RayCastInput;
using RayCastOutput = org.jbox2d.collision.RayCastOutput;
using TOIInput = org.jbox2d.collision.TimeOfImpact.TOIInput;
using TOIOutput = org.jbox2d.collision.TimeOfImpact.TOIOutput;
//UPGRADE_TODO: The type 'org.jbox2d.collision.TimeOfImpact.TOIOutputState' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
using TOIOutputState = org.jbox2d.collision.TimeOfImpact.TOIOutputState;
using BroadPhase = org.jbox2d.collision.broadphase.BroadPhase;
using ChainShape = org.jbox2d.collision.shapes.ChainShape;
using CircleShape = org.jbox2d.collision.shapes.CircleShape;
using EdgeShape = org.jbox2d.collision.shapes.EdgeShape;
using PolygonShape = org.jbox2d.collision.shapes.PolygonShape;
//UPGRADE_TODO: The type 'org.jbox2d.collision.shapes.ShapeType' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
using ShapeType = org.jbox2d.collision.shapes.ShapeType;
using Color3f = org.jbox2d.common.Color3f;
using MathUtils = org.jbox2d.common.MathUtils;
using Settings = org.jbox2d.common.Settings;
using Sweep = org.jbox2d.common.Sweep;
using Timer = org.jbox2d.common.Timer;
using Transform = org.jbox2d.common.Transform;
using Vec2 = org.jbox2d.common.Vec2;
using Contact = org.jbox2d.dynamics.contacts.Contact;
using ContactEdge = org.jbox2d.dynamics.contacts.ContactEdge;
using ContactRegister = org.jbox2d.dynamics.contacts.ContactRegister;
using Joint = org.jbox2d.dynamics.joints.Joint;
using JointDef = org.jbox2d.dynamics.joints.JointDef;
using JointEdge = org.jbox2d.dynamics.joints.JointEdge;
using PulleyJoint = org.jbox2d.dynamics.joints.PulleyJoint;
//UPGRADE_TODO: The type 'org.jbox2d.pooling.IDynamicStack' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
using IDynamicStack = org.jbox2d.pooling.IDynamicStack;
using IWorldPool = org.jbox2d.pooling.IWorldPool;
using Vec2Array = org.jbox2d.pooling.arrays.Vec2Array;
using DefaultWorldPool = org.jbox2d.pooling.normal.DefaultWorldPool;
namespace org.jbox2d.dynamics
{
	
	/// <summary> The world class manages all physics entities, dynamic simulation, and asynchronous queries. The
	/// world also contains efficient memory management facilities.
	/// 
	/// </summary>
	/// <author>  Daniel Murphy
	/// </author>
	public class World
	{
		private void  InitBlock()
		{
			for (int i = 0; i < ShapeType.values().length; i++)
			{
				contactStacks[i] = new ContactRegister[ShapeType.values().length];
			}
			ContactRegister register = new ContactRegister();
			register.creator = creator;
			register.primary = true;
			contactStacks[type1.ordinal()][type2.ordinal()] = register;
			
			if (type1 != type2)
			{
				ContactRegister register2 = new ContactRegister();
				register2.creator = creator;
				register2.primary = false;
				contactStacks[type2.ordinal()][type1.ordinal()] = register2;
			}
		}
		virtual public bool AllowSleep
		{
			get
			{
				return m_allowSleep;
			}
			
			set
			{
				if (value == m_allowSleep)
				{
					return ;
				}
				
				m_allowSleep = value;
				if (m_allowSleep == false)
				{
					for (Body b = m_bodyList; b != null; b = b.m_next)
					{
						b.Awake = true;
					}
				}
			}
			
		}
		virtual public IWorldPool Pool
		{
			get
			{
				return pool;
			}
			
		}
		/// <summary> Register a destruction listener. The listener is owned by you and must remain in scope.
		/// 
		/// </summary>
		/// <param name="listener">
		/// </param>
		virtual public DestructionListener DestructionListener
		{
			set
			{
				m_destructionListener = value;
			}
			
		}
		/// <summary> Register a contact filter to provide specific control over collision. Otherwise the default
		/// filter is used (_defaultFilter). The listener is owned by you and must remain in scope.
		/// 
		/// </summary>
		/// <param name="filter">
		/// </param>
		virtual public ContactFilter ContactFilter
		{
			set
			{
				m_contactManager.m_contactFilter = value;
			}
			
		}
		/// <summary> Register a contact event listener. The listener is owned by you and must remain in scope.
		/// 
		/// </summary>
		/// <param name="listener">
		/// </param>
		virtual public ContactListener ContactListener
		{
			set
			{
				m_contactManager.m_contactListener = value;
			}
			
		}
		/// <summary> Register a routine for debug drawing. The debug draw functions are called inside with
		/// World.DrawDebugData method. The debug draw object is owned by you and must remain in scope.
		/// 
		/// </summary>
		/// <param name="debugDraw">
		/// </param>
		virtual public DebugDraw DebugDraw
		{
			set
			{
				m_debugDraw = value;
			}
			
		}
		/// <summary> Get the world body list. With the returned body, use Body.getNext to get the next body in the
		/// world list. A null body indicates the end of the list.
		/// 
		/// </summary>
		/// <returns> the head of the world body list.
		/// </returns>
		virtual public Body BodyList
		{
			get
			{
				return m_bodyList;
			}
			
		}
		/// <summary> Get the world joint list. With the returned joint, use Joint.getNext to get the next joint in
		/// the world list. A null joint indicates the end of the list.
		/// 
		/// </summary>
		/// <returns> the head of the world joint list.
		/// </returns>
		virtual public Joint JointList
		{
			get
			{
				return m_jointList;
			}
			
		}
		/// <summary> Get the world contact list. With the returned contact, use Contact.getNext to get the next
		/// contact in the world list. A null contact indicates the end of the list.
		/// 
		/// </summary>
		/// <returns> the head of the world contact list.
		/// </returns>
		/// <warning>  contacts are created and destroyed in the middle of a time step. Use ContactListener </warning>
		/// <summary>          to avoid missing contacts.
		/// </summary>
		virtual public Contact ContactList
		{
			get
			{
				return m_contactManager.m_contactList;
			}
			
		}
		virtual public bool SleepingAllowed
		{
			get
			{
				return m_allowSleep;
			}
			
			set
			{
				m_allowSleep = value;
			}
			
		}
		/// <summary> Enable/disable warm starting. For testing.
		/// 
		/// </summary>
		/// <param name="flag">
		/// </param>
		virtual public bool WarmStarting
		{
			get
			{
				return m_warmStarting;
			}
			
			set
			{
				m_warmStarting = value;
			}
			
		}
		/// <summary> Enable/disable continuous physics. For testing.
		/// 
		/// </summary>
		/// <param name="flag">
		/// </param>
		virtual public bool ContinuousPhysics
		{
			get
			{
				return m_continuousPhysics;
			}
			
			set
			{
				m_continuousPhysics = value;
			}
			
		}
		/// <summary> Get the number of broad-phase proxies.
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		virtual public int ProxyCount
		{
			get
			{
				return m_contactManager.m_broadPhase.ProxyCount;
			}
			
		}
		/// <summary> Get the number of bodies.
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		virtual public int BodyCount
		{
			get
			{
				return m_bodyCount;
			}
			
		}
		/// <summary> Get the number of joints.
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		virtual public int JointCount
		{
			get
			{
				return m_jointCount;
			}
			
		}
		/// <summary> Get the number of contacts (each may have 0 or more contact points).
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		virtual public int ContactCount
		{
			get
			{
				return m_contactManager.m_contactCount;
			}
			
		}
		/// <summary> Gets the height of the dynamic tree
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		virtual public int TreeHeight
		{
			get
			{
				return m_contactManager.m_broadPhase.TreeHeight;
			}
			
		}
		/// <summary> Gets the balance of the dynamic tree
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		virtual public int TreeBalance
		{
			get
			{
				return m_contactManager.m_broadPhase.TreeBalance;
			}
			
		}
		/// <summary> Gets the quality of the dynamic tree
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		virtual public float TreeQuality
		{
			get
			{
				return m_contactManager.m_broadPhase.TreeQuality;
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary> Get the global gravity vector.
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		/// <summary> Change the global gravity vector.
		/// 
		/// </summary>
		/// <param name="gravity">
		/// </param>
		virtual public Vec2 Gravity
		{
			get
			{
				return m_gravity;
			}
			
			set
			{
				m_gravity.set_Renamed(value);
			}
			
		}
		/// <summary> Is the world locked (in the middle of a time step).
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		virtual public bool Locked
		{
			get
			{
				return (m_flags & LOCKED) == LOCKED;
			}
			
		}
		//UPGRADE_NOTE: Respective javadoc comments were merged.  It should be changed in order to comply with .NET documentation conventions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1199'"
		/// <summary> Get the flag that controls automatic clearing of forces after each time step.
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		/// <summary> Set flag to control automatic clearing of forces after each time step.
		/// 
		/// </summary>
		/// <param name="flag">
		/// </param>
		virtual public bool AutoClearForces
		{
			get
			{
				return (m_flags & CLEAR_FORCES) == CLEAR_FORCES;
			}
			
			set
			{
				if (value)
				{
					m_flags |= CLEAR_FORCES;
				}
				else
				{
					m_flags &= ~ CLEAR_FORCES;
				}
			}
			
		}
		/// <summary> Get the contact manager for testing purposes
		/// 
		/// </summary>
		/// <returns>
		/// </returns>
		virtual public ContactManager ContactManager
		{
			get
			{
				return m_contactManager;
			}
			
		}
		virtual public Profile Profile
		{
			get
			{
				return m_profile;
			}
			
		}
		public const int WORLD_POOL_SIZE = 100;
		public const int WORLD_POOL_CONTAINER_SIZE = 10;
		
		public const int NEW_FIXTURE = 0x0001;
		public const int LOCKED = 0x0002;
		public const int CLEAR_FORCES = 0x0004;
		
		
		// statistics gathering
		public int activeContacts = 0;
		public int contactPoolCount = 0;
		
		protected internal int m_flags;
		
		protected internal ContactManager m_contactManager;
		
		private Body m_bodyList;
		private Joint m_jointList;
		
		private int m_bodyCount;
		private int m_jointCount;
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'm_gravity '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Vec2 m_gravity = new Vec2();
		private bool m_allowSleep;
		
		// private Body m_groundBody;
		
		private DestructionListener m_destructionListener;
		private DebugDraw m_debugDraw;
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'pool '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private IWorldPool pool;
		
		/// <summary> This is used to compute the time step ratio to support a variable time step.</summary>
		private float m_inv_dt0;
		
		// these are for debugging the solver
		private bool m_warmStarting;
		private bool m_continuousPhysics;
		private bool m_subStepping;
		
		private bool m_stepComplete;
		
		private Profile m_profile;
		
		
		private ContactRegister[][] contactStacks = new ContactRegister[ShapeType.values().length][];
		
		public World(Vec2 gravity):this(gravity, new DefaultWorldPool(WORLD_POOL_SIZE, WORLD_POOL_CONTAINER_SIZE))
		{
		}
		
		/// <summary> Construct a world object.
		/// 
		/// </summary>
		/// <param name="gravity">the world gravity vector.
		/// </param>
		/// <param name="doSleep">improve performance by not simulating inactive bodies.
		/// </param>
		public World(Vec2 gravity, IWorldPool argPool)
		{
			InitBlock();
			pool = argPool;
			m_destructionListener = null;
			m_debugDraw = null;
			
			m_bodyList = null;
			m_jointList = null;
			
			m_bodyCount = 0;
			m_jointCount = 0;
			
			m_warmStarting = true;
			m_continuousPhysics = true;
			m_subStepping = false;
			m_stepComplete = true;
			
			m_allowSleep = true;
			m_gravity.set_Renamed(gravity);
			
			m_flags = CLEAR_FORCES;
			
			m_inv_dt0 = 0f;
			
			m_contactManager = new ContactManager(this);
			m_profile = new Profile();
			
			initializeRegisters();
		}
		
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		private
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		void addType(IDynamicStack < Contact > creator, ShapeType type1, ShapeType type2)
		
		private void  initializeRegisters()
		{
			addType(pool.getCircleContactStack(), ShapeType.CIRCLE, ShapeType.CIRCLE);
			addType(pool.getPolyCircleContactStack(), ShapeType.POLYGON, ShapeType.CIRCLE);
			addType(pool.getPolyContactStack(), ShapeType.POLYGON, ShapeType.POLYGON);
		}
		
		public virtual Contact popContact(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB)
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'type1 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			ShapeType type1 = fixtureA.Type;
			//UPGRADE_NOTE: Final was removed from the declaration of 'type2 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			ShapeType type2 = fixtureB.Type;
			
			//UPGRADE_NOTE: Final was removed from the declaration of 'reg '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			ContactRegister reg = contactStacks[type1.ordinal()][type2.ordinal()];
			//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
			final IDynamicStack < Contact > creator = reg.creator;
			if (creator != null)
			{
				if (reg.primary)
				{
					Contact c = creator.pop();
					c.init(fixtureA, indexA, fixtureB, indexB);
					return c;
				}
				else
				{
					Contact c = creator.pop();
					c.init(fixtureB, indexB, fixtureA, indexA);
					return c;
				}
			}
			else
			{
				return null;
			}
		}
		
		public virtual void  pushContact(Contact contact)
		{
			
			if (contact.m_manifold.pointCount > 0)
			{
				contact.FixtureA.Body.Awake = true;
				contact.FixtureB.Body.Awake = true;
			}
			
			ShapeType type1 = contact.FixtureA.Type;
			ShapeType type2 = contact.FixtureB.Type;
			
			//UPGRADE_NOTE: There is an untranslated Statement.  Please refer to original code. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1153'"
			creator.push(contact);
		}
		
		/// <summary> create a rigid body given a definition. No reference to the definition is retained.
		/// 
		/// </summary>
		/// <warning>  This function is locked during callbacks. </warning>
		/// <param name="def">
		/// </param>
		/// <returns>
		/// </returns>
		public virtual Body createBody(BodyDef def)
		{
			assert(Locked == false);
			if (Locked)
			{
				return null;
			}
			// TODO djm pooling
			Body b = new Body(def, this);
			
			// add to world doubly linked list
			b.m_prev = null;
			b.m_next = m_bodyList;
			if (m_bodyList != null)
			{
				m_bodyList.m_prev = b;
			}
			m_bodyList = b;
			++m_bodyCount;
			
			return b;
		}
		
		/// <summary> destroy a rigid body given a definition. No reference to the definition is retained. This
		/// function is locked during callbacks.
		/// 
		/// </summary>
		/// <warning>  This automatically deletes all associated shapes and joints. </warning>
		/// <warning>  This function is locked during callbacks. </warning>
		/// <param name="body">
		/// </param>
		public virtual void  destroyBody(Body body)
		{
			assert(m_bodyCount > 0);
			assert(Locked == false);
			if (Locked)
			{
				return ;
			}
			
			// Delete the attached joints.
			JointEdge je = body.m_jointList;
			while (je != null)
			{
				JointEdge je0 = je;
				je = je.next;
				if (m_destructionListener != null)
				{
					m_destructionListener.sayGoodbye(je0.joint);
				}
				
				destroyJoint(je0.joint);
				
				body.m_jointList = je;
			}
			body.m_jointList = null;
			
			// Delete the attached contacts.
			ContactEdge ce = body.m_contactList;
			while (ce != null)
			{
				ContactEdge ce0 = ce;
				ce = ce.next;
				m_contactManager.destroy(ce0.contact);
			}
			body.m_contactList = null;
			
			Fixture f = body.m_fixtureList;
			while (f != null)
			{
				Fixture f0 = f;
				f = f.m_next;
				
				if (m_destructionListener != null)
				{
					m_destructionListener.sayGoodbye(f0);
				}
				
				f0.destroyProxies(m_contactManager.m_broadPhase);
				f0.destroy();
				// TODO djm recycle fixtures (here or in that destroy method)
				body.m_fixtureList = f;
				body.m_fixtureCount -= 1;
			}
			body.m_fixtureList = null;
			body.m_fixtureCount = 0;
			
			// Remove world body list.
			if (body.m_prev != null)
			{
				body.m_prev.m_next = body.m_next;
			}
			
			if (body.m_next != null)
			{
				body.m_next.m_prev = body.m_prev;
			}
			
			if (body == m_bodyList)
			{
				m_bodyList = body.m_next;
			}
			
			--m_bodyCount;
			// TODO djm recycle body
		}
		
		/// <summary> create a joint to constrain bodies together. No reference to the definition is retained. This
		/// may cause the connected bodies to cease colliding.
		/// 
		/// </summary>
		/// <warning>  This function is locked during callbacks. </warning>
		/// <param name="def">
		/// </param>
		/// <returns>
		/// </returns>
		public virtual Joint createJoint(JointDef def)
		{
			assert(Locked == false);
			if (Locked)
			{
				return null;
			}
			
			Joint j = Joint.create(this, def);
			
			// Connect to the world list.
			j.m_prev = null;
			j.m_next = m_jointList;
			if (m_jointList != null)
			{
				m_jointList.m_prev = j;
			}
			m_jointList = j;
			++m_jointCount;
			
			// Connect to the bodies' doubly linked lists.
			j.m_edgeA.joint = j;
			j.m_edgeA.other = j.m_bodyB;
			j.m_edgeA.prev = null;
			j.m_edgeA.next = j.m_bodyA.m_jointList;
			if (j.m_bodyA.m_jointList != null)
			{
				j.m_bodyA.m_jointList.prev = j.m_edgeA;
			}
			j.m_bodyA.m_jointList = j.m_edgeA;
			
			j.m_edgeB.joint = j;
			j.m_edgeB.other = j.m_bodyA;
			j.m_edgeB.prev = null;
			j.m_edgeB.next = j.m_bodyB.m_jointList;
			if (j.m_bodyB.m_jointList != null)
			{
				j.m_bodyB.m_jointList.prev = j.m_edgeB;
			}
			j.m_bodyB.m_jointList = j.m_edgeB;
			
			Body bodyA = def.bodyA;
			Body bodyB = def.bodyB;
			
			// If the joint prevents collisions, then flag any contacts for filtering.
			if (def.collideConnected == false)
			{
				ContactEdge edge = bodyB.ContactList;
				while (edge != null)
				{
					if (edge.other == bodyA)
					{
						// Flag the contact for filtering at the next time step (where either
						// body is awake).
						edge.contact.flagForFiltering();
					}
					
					edge = edge.next;
				}
			}
			
			// Note: creating a joint doesn't wake the bodies.
			
			return j;
		}
		
		/// <summary> destroy a joint. This may cause the connected bodies to begin colliding.
		/// 
		/// </summary>
		/// <warning>  This function is locked during callbacks. </warning>
		/// <param name="joint">
		/// </param>
		public virtual void  destroyJoint(Joint j)
		{
			assert(Locked == false);
			if (Locked)
			{
				return ;
			}
			
			bool collideConnected = j.m_collideConnected;
			
			// Remove from the doubly linked list.
			if (j.m_prev != null)
			{
				j.m_prev.m_next = j.m_next;
			}
			
			if (j.m_next != null)
			{
				j.m_next.m_prev = j.m_prev;
			}
			
			if (j == m_jointList)
			{
				m_jointList = j.m_next;
			}
			
			// Disconnect from island graph.
			Body bodyA = j.m_bodyA;
			Body bodyB = j.m_bodyB;
			
			// Wake up connected bodies.
			bodyA.Awake = true;
			bodyB.Awake = true;
			
			// Remove from body 1.
			if (j.m_edgeA.prev != null)
			{
				j.m_edgeA.prev.next = j.m_edgeA.next;
			}
			
			if (j.m_edgeA.next != null)
			{
				j.m_edgeA.next.prev = j.m_edgeA.prev;
			}
			
			if (j.m_edgeA == bodyA.m_jointList)
			{
				bodyA.m_jointList = j.m_edgeA.next;
			}
			
			j.m_edgeA.prev = null;
			j.m_edgeA.next = null;
			
			// Remove from body 2
			if (j.m_edgeB.prev != null)
			{
				j.m_edgeB.prev.next = j.m_edgeB.next;
			}
			
			if (j.m_edgeB.next != null)
			{
				j.m_edgeB.next.prev = j.m_edgeB.prev;
			}
			
			if (j.m_edgeB == bodyB.m_jointList)
			{
				bodyB.m_jointList = j.m_edgeB.next;
			}
			
			j.m_edgeB.prev = null;
			j.m_edgeB.next = null;
			
			Joint.destroy(j);
			
			assert(m_jointCount > 0);
			--m_jointCount;
			
			// If the joint prevents collisions, then flag any contacts for filtering.
			if (collideConnected == false)
			{
				ContactEdge edge = bodyB.ContactList;
				while (edge != null)
				{
					if (edge.other == bodyA)
					{
						// Flag the contact for filtering at the next time step (where either
						// body is awake).
						edge.contact.flagForFiltering();
					}
					
					edge = edge.next;
				}
			}
		}
		
		// djm pooling
		//UPGRADE_NOTE: Final was removed from the declaration of 'step '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private TimeStep step_Renamed_Field = new TimeStep();
		//UPGRADE_NOTE: Final was removed from the declaration of 'stepTimer '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Timer stepTimer = new Timer();
		//UPGRADE_NOTE: Final was removed from the declaration of 'tempTimer '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Timer tempTimer = new Timer();
		
		/// <summary> Take a time step. This performs collision detection, integration, and constraint solution.
		/// 
		/// </summary>
		/// <param name="timeStep">the amount of time to simulate, this should not vary.
		/// </param>
		/// <param name="velocityIterations">for the velocity constraint solver.
		/// </param>
		/// <param name="positionIterations">for the position constraint solver.
		/// </param>
		public virtual void  step(float dt, int velocityIterations, int positionIterations)
		{
			stepTimer.reset();
			// log.debug("Starting step");
			// If new fixtures were added, we need to find the new contacts.
			if ((m_flags & NEW_FIXTURE) == NEW_FIXTURE)
			{
				// log.debug("There's a new fixture, lets look for new contacts");
				m_contactManager.findNewContacts();
				m_flags &= ~ NEW_FIXTURE;
			}
			
			m_flags |= LOCKED;
			
			step_Renamed_Field.dt = dt;
			step_Renamed_Field.velocityIterations = velocityIterations;
			step_Renamed_Field.positionIterations = positionIterations;
			if (dt > 0.0f)
			{
				step_Renamed_Field.inv_dt = 1.0f / dt;
			}
			else
			{
				step_Renamed_Field.inv_dt = 0.0f;
			}
			
			step_Renamed_Field.dtRatio = m_inv_dt0 * dt;
			
			step_Renamed_Field.warmStarting = m_warmStarting;
			
			// Update contacts. This is where some contacts are destroyed.
			tempTimer.reset();
			m_contactManager.collide();
			m_profile.collide = tempTimer.Milliseconds;
			
			// Integrate velocities, solve velocity constraints, and integrate positions.
			if (m_stepComplete && step_Renamed_Field.dt > 0.0f)
			{
				tempTimer.reset();
				solve(step_Renamed_Field);
				m_profile.solve = tempTimer.Milliseconds;
			}
			
			// Handle TOI events.
			if (m_continuousPhysics && step_Renamed_Field.dt > 0.0f)
			{
				tempTimer.reset();
				solveTOI(step_Renamed_Field);
				m_profile.solveTOI = tempTimer.Milliseconds;
			}
			
			if (step_Renamed_Field.dt > 0.0f)
			{
				m_inv_dt0 = step_Renamed_Field.inv_dt;
			}
			
			if ((m_flags & CLEAR_FORCES) == CLEAR_FORCES)
			{
				clearForces();
			}
			
			m_flags &= ~ LOCKED;
			// log.debug("ending step");
			
			m_profile.step = stepTimer.Milliseconds;
		}
		
		/// <summary> Call this after you are done with time steps to clear the forces. You normally call this after
		/// each call to Step, unless you are performing sub-steps. By default, forces will be
		/// automatically cleared, so you don't need to call this function.
		/// 
		/// </summary>
		/// <seealso cref="setAutoClearForces">
		/// </seealso>
		public virtual void  clearForces()
		{
			for (Body body = m_bodyList; body != null; body = body.Next)
			{
				body.m_force.setZero();
				body.m_torque = 0.0f;
			}
		}
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'color '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Color3f color = new Color3f();
		//UPGRADE_NOTE: Final was removed from the declaration of 'xf '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Transform xf = new Transform();
		//UPGRADE_NOTE: Final was removed from the declaration of 'cA '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Vec2 cA = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'cB '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Vec2 cB = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'avs '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Vec2Array avs = new Vec2Array();
		
		/// <summary> Call this to draw shapes and other debug draw data.</summary>
		public virtual void  drawDebugData()
		{
			if (m_debugDraw == null)
			{
				return ;
			}
			
			int flags = m_debugDraw.Flags;
			
			if ((flags & DebugDraw.e_shapeBit) == DebugDraw.e_shapeBit)
			{
				for (Body b = m_bodyList; b != null; b = b.Next)
				{
					xf.set_Renamed(b.getTransform());
					for (Fixture f = b.FixtureList; f != null; f = f.Next)
					{
						if (b.Active == false)
						{
							color.set_Renamed(0.5f, 0.5f, 0.3f);
							drawShape(f, xf, color);
						}
						else if (b.Type == BodyType.STATIC)
						{
							color.set_Renamed(0.5f, 0.9f, 0.3f);
							drawShape(f, xf, color);
						}
						else if (b.Type == BodyType.KINEMATIC)
						{
							color.set_Renamed(0.5f, 0.5f, 0.9f);
							drawShape(f, xf, color);
						}
						else if (b.Awake == false)
						{
							color.set_Renamed(0.5f, 0.5f, 0.5f);
							drawShape(f, xf, color);
						}
						else
						{
							color.set_Renamed(0.9f, 0.7f, 0.7f);
							drawShape(f, xf, color);
						}
					}
				}
			}
			
			if ((flags & DebugDraw.e_jointBit) == DebugDraw.e_jointBit)
			{
				for (Joint j = m_jointList; j != null; j = j.Next)
				{
					drawJoint(j);
				}
			}
			
			if ((flags & DebugDraw.e_pairBit) == DebugDraw.e_pairBit)
			{
				color.set_Renamed(0.3f, 0.9f, 0.9f);
				for (Contact c = m_contactManager.m_contactList; c != null; c = c.Next)
				{
					// Fixture fixtureA = c.getFixtureA();
					// Fixture fixtureB = c.getFixtureB();
					//
					// fixtureA.getAABB(childIndex).getCenterToOut(cA);
					// fixtureB.getAABB().getCenterToOut(cB);
					//
					// m_debugDraw.drawSegment(cA, cB, color);
				}
			}
			
			if ((flags & DebugDraw.e_aabbBit) == DebugDraw.e_aabbBit)
			{
				color.set_Renamed(0.9f, 0.3f, 0.9f);
				
				for (Body b = m_bodyList; b != null; b = b.Next)
				{
					if (b.Active == false)
					{
						continue;
					}
					
					for (Fixture f = b.FixtureList; f != null; f = f.Next)
					{
						
						for (int i = 0; i < f.m_proxyCount; ++i)
						{
							FixtureProxy proxy = f.m_proxies[i];
							AABB aabb = m_contactManager.m_broadPhase.getFatAABB(proxy.proxyId);
							Vec2[] vs = avs.get_Renamed(4);
							vs[0].set_Renamed(aabb.lowerBound.x, aabb.lowerBound.y);
							vs[1].set_Renamed(aabb.upperBound.x, aabb.lowerBound.y);
							vs[2].set_Renamed(aabb.upperBound.x, aabb.upperBound.y);
							vs[3].set_Renamed(aabb.lowerBound.x, aabb.upperBound.y);
							
							m_debugDraw.drawPolygon(vs, 4, color);
						}
					}
				}
			}
			
			if ((flags & DebugDraw.e_centerOfMassBit) == DebugDraw.e_centerOfMassBit)
			{
				for (Body b = m_bodyList; b != null; b = b.Next)
				{
					xf.set_Renamed(b.getTransform());
					xf.p.set_Renamed(b.WorldCenter);
					m_debugDraw.drawTransform(xf);
				}
			}
			
			if ((flags & DebugDraw.e_dynamicTreeBit) == DebugDraw.e_dynamicTreeBit)
			{
				m_contactManager.m_broadPhase.drawTree(m_debugDraw);
			}
		}
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'wqwrapper '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private WorldQueryWrapper wqwrapper = new WorldQueryWrapper();
		
		/// <summary> Query the world for all fixtures that potentially overlap the provided AABB.
		/// 
		/// </summary>
		/// <param name="callback">a user implemented callback class.
		/// </param>
		/// <param name="aabb">the query box.
		/// </param>
		public virtual void  queryAABB(QueryCallback callback, AABB aabb)
		{
			wqwrapper.broadPhase = m_contactManager.m_broadPhase;
			wqwrapper.callback = callback;
			m_contactManager.m_broadPhase.query(wqwrapper, aabb);
		}
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'wrcwrapper '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private WorldRayCastWrapper wrcwrapper = new WorldRayCastWrapper();
		//UPGRADE_NOTE: Final was removed from the declaration of 'input '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private RayCastInput input = new RayCastInput();
		
		/// <summary> Ray-cast the world for all fixtures in the path of the ray. Your callback controls whether you
		/// get the closest point, any point, or n-points. The ray-cast ignores shapes that contain the
		/// starting point.
		/// 
		/// </summary>
		/// <param name="callback">a user implemented callback class.
		/// </param>
		/// <param name="point1">the ray starting point
		/// </param>
		/// <param name="point2">the ray ending point
		/// </param>
		public virtual void  raycast(RayCastCallback callback, Vec2 point1, Vec2 point2)
		{
			wrcwrapper.broadPhase = m_contactManager.m_broadPhase;
			wrcwrapper.callback = callback;
			input.maxFraction = 1.0f;
			input.p1.set_Renamed(point1);
			input.p2.set_Renamed(point2);
			m_contactManager.m_broadPhase.raycast(wrcwrapper, input);
		}
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'island '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Island island = new Island();
		private Body[] stack = new Body[10]; // TODO djm find a good initial stack number;
		//UPGRADE_NOTE: Final was removed from the declaration of 'islandProfile '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Profile islandProfile = new Profile();
		//UPGRADE_NOTE: Final was removed from the declaration of 'broadphaseTimer '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Timer broadphaseTimer = new Timer();
		
		private void  solve(TimeStep step)
		{
			m_profile.solveInit = 0;
			m_profile.solveVelocity = 0;
			m_profile.solvePosition = 0;
			
			// Size the island for the worst case.
			island.init(m_bodyCount, m_contactManager.m_contactCount, m_jointCount, m_contactManager.m_contactListener);
			
			// Clear all the island flags.
			for (Body b = m_bodyList; b != null; b = b.m_next)
			{
				b.m_flags &= ~ Body.e_islandFlag;
			}
			for (Contact c = m_contactManager.m_contactList; c != null; c = c.m_next)
			{
				c.m_flags &= ~ Contact.ISLAND_FLAG;
			}
			for (Joint j = m_jointList; j != null; j = j.m_next)
			{
				j.m_islandFlag = false;
			}
			
			// Build and simulate all awake islands.
			int stackSize = m_bodyCount;
			if (stack.Length < stackSize)
			{
				stack = new Body[stackSize];
			}
			for (Body seed = m_bodyList; seed != null; seed = seed.m_next)
			{
				if ((seed.m_flags & Body.e_islandFlag) == Body.e_islandFlag)
				{
					continue;
				}
				
				if (seed.Awake == false || seed.Active == false)
				{
					continue;
				}
				
				// The seed can be dynamic or kinematic.
				if (seed.Type == BodyType.STATIC)
				{
					continue;
				}
				
				// Reset island and stack.
				island.clear();
				int stackCount = 0;
				stack[stackCount++] = seed;
				seed.m_flags |= Body.e_islandFlag;
				
				// Perform a depth first search (DFS) on the constraint graph.
				while (stackCount > 0)
				{
					// Grab the next body off the stack and add it to the island.
					Body b = stack[--stackCount];
					assert(b.Active == true);
					island.add(b);
					
					// Make sure the body is awake.
					b.Awake = true;
					
					// To keep islands as small as possible, we don't
					// propagate islands across static bodies.
					if (b.Type == BodyType.STATIC)
					{
						continue;
					}
					
					// Search all contacts connected to this body.
					for (ContactEdge ce = b.m_contactList; ce != null; ce = ce.next)
					{
						Contact contact = ce.contact;
						
						// Has this contact already been added to an island?
						if ((contact.m_flags & Contact.ISLAND_FLAG) == Contact.ISLAND_FLAG)
						{
							continue;
						}
						
						// Is this contact solid and touching?
						if (contact.Enabled == false || contact.Touching == false)
						{
							continue;
						}
						
						// Skip sensors.
						bool sensorA = contact.m_fixtureA.m_isSensor;
						bool sensorB = contact.m_fixtureB.m_isSensor;
						if (sensorA || sensorB)
						{
							continue;
						}
						
						island.add(contact);
						contact.m_flags |= Contact.ISLAND_FLAG;
						
						Body other = ce.other;
						
						// Was the other body already added to this island?
						if ((other.m_flags & Body.e_islandFlag) == Body.e_islandFlag)
						{
							continue;
						}
						
						assert(stackCount < stackSize);
						stack[stackCount++] = other;
						other.m_flags |= Body.e_islandFlag;
					}
					
					// Search all joints connect to this body.
					for (JointEdge je = b.m_jointList; je != null; je = je.next)
					{
						if (je.joint.m_islandFlag == true)
						{
							continue;
						}
						
						Body other = je.other;
						
						// Don't simulate joints connected to inactive bodies.
						if (other.Active == false)
						{
							continue;
						}
						
						island.add(je.joint);
						je.joint.m_islandFlag = true;
						
						if ((other.m_flags & Body.e_islandFlag) == Body.e_islandFlag)
						{
							continue;
						}
						
						assert(stackCount < stackSize);
						stack[stackCount++] = other;
						other.m_flags |= Body.e_islandFlag;
					}
				}
				island.solve(islandProfile, step, m_gravity, m_allowSleep);
				m_profile.solveInit += islandProfile.solveInit;
				m_profile.solveVelocity += islandProfile.solveVelocity;
				m_profile.solvePosition += islandProfile.solvePosition;
				
				// Post solve cleanup.
				for (int i = 0; i < island.m_bodyCount; ++i)
				{
					// Allow static bodies to participate in other islands.
					Body b = island.m_bodies[i];
					if (b.Type == BodyType.STATIC)
					{
						b.m_flags &= ~ Body.e_islandFlag;
					}
				}
			}
			
			broadphaseTimer.reset();
			// Synchronize fixtures, check for out of range bodies.
			for (Body b = m_bodyList; b != null; b = b.Next)
			{
				// If a body was not in an island then it did not move.
				if ((b.m_flags & Body.e_islandFlag) == 0)
				{
					continue;
				}
				
				if (b.Type == BodyType.STATIC)
				{
					continue;
				}
				
				// Update fixtures (for broad-phase).
				b.synchronizeFixtures();
			}
			
			// Look for new contacts.
			m_contactManager.findNewContacts();
			m_profile.broadphase = broadphaseTimer.Milliseconds;
		}
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'toiIsland '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Island toiIsland = new Island();
		//UPGRADE_NOTE: Final was removed from the declaration of 'toiInput '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private TOIInput toiInput = new TOIInput();
		//UPGRADE_NOTE: Final was removed from the declaration of 'toiOutput '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private TOIOutput toiOutput = new TOIOutput();
		//UPGRADE_NOTE: Final was removed from the declaration of 'subStep '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private TimeStep subStep = new TimeStep();
		//UPGRADE_NOTE: Final was removed from the declaration of 'tempBodies '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Body[] tempBodies = new Body[2];
		//UPGRADE_NOTE: Final was removed from the declaration of 'backup1 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Sweep backup1 = new Sweep();
		//UPGRADE_NOTE: Final was removed from the declaration of 'backup2 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Sweep backup2 = new Sweep();
		
		private void  solveTOI(TimeStep step)
		{
			
			//UPGRADE_NOTE: Final was removed from the declaration of 'island '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Island island = toiIsland;
			island.init(2 * Settings.maxTOIContacts, Settings.maxTOIContacts, 0, m_contactManager.m_contactListener);
			if (m_stepComplete)
			{
				for (Body b = m_bodyList; b != null; b = b.m_next)
				{
					b.m_flags &= ~ Body.e_islandFlag;
					b.m_sweep.alpha0 = 0.0f;
				}
				
				for (Contact c = m_contactManager.m_contactList; c != null; c = c.m_next)
				{
					// Invalidate TOI
					c.m_flags &= ~ (Contact.TOI_FLAG | Contact.ISLAND_FLAG);
					c.m_toiCount = 0;
					c.m_toi = 1.0f;
				}
			}
			
			// Find TOI events and solve them.
			for (; ; )
			{
				// Find the first TOI.
				Contact minContact = null;
				float minAlpha = 1.0f;
				
				for (Contact c = m_contactManager.m_contactList; c != null; c = c.m_next)
				{
					// Is this contact disabled?
					if (c.Enabled == false)
					{
						continue;
					}
					
					// Prevent excessive sub-stepping.
					if (c.m_toiCount > Settings.maxSubSteps)
					{
						continue;
					}
					
					float alpha = 1.0f;
					if ((c.m_flags & Contact.TOI_FLAG) != 0)
					{
						// This contact has a valid cached TOI.
						alpha = c.m_toi;
					}
					else
					{
						Fixture fA = c.FixtureA;
						Fixture fB = c.FixtureB;
						
						// Is there a sensor?
						if (fA.Sensor || fB.Sensor)
						{
							continue;
						}
						
						Body bA = fA.Body;
						Body bB = fB.Body;
						
						BodyType typeA = bA.m_type;
						BodyType typeB = bB.m_type;
						assert(typeA == BodyType.DYNAMIC || typeB == BodyType.DYNAMIC);
						
						bool activeA = bA.Awake && typeA != BodyType.STATIC;
						bool activeB = bB.Awake && typeB != BodyType.STATIC;
						
						// Is at least one body active (awake and dynamic or kinematic)?
						if (activeA == false && activeB == false)
						{
							continue;
						}
						
						bool collideA = bA.Bullet || typeA != BodyType.DYNAMIC;
						bool collideB = bB.Bullet || typeB != BodyType.DYNAMIC;
						
						// Are these two non-bullet dynamic bodies?
						if (collideA == false && collideB == false)
						{
							continue;
						}
						
						// Compute the TOI for this contact.
						// Put the sweeps onto the same time interval.
						float alpha0 = bA.m_sweep.alpha0;
						
						if (bA.m_sweep.alpha0 < bB.m_sweep.alpha0)
						{
							alpha0 = bB.m_sweep.alpha0;
							bA.m_sweep.advance(alpha0);
						}
						else if (bB.m_sweep.alpha0 < bA.m_sweep.alpha0)
						{
							alpha0 = bA.m_sweep.alpha0;
							bB.m_sweep.advance(alpha0);
						}
						
						assert(alpha0 < 1.0f);
						
						int indexA = c.ChildIndexA;
						int indexB = c.ChildIndexB;
						
						// Compute the time of impact in interval [0, minTOI]
						//UPGRADE_NOTE: Final was removed from the declaration of 'input '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
						TOIInput input = toiInput;
						input.proxyA.set_Renamed(fA.Shape, indexA);
						input.proxyB.set_Renamed(fB.Shape, indexB);
						input.sweepA.set_Renamed(bA.m_sweep);
						input.sweepB.set_Renamed(bB.m_sweep);
						input.tMax = 1.0f;
						
						pool.getTimeOfImpact().timeOfImpact(toiOutput, input);
						
						// Beta is the fraction of the remaining portion of the .
						float beta = toiOutput.t;
						if (toiOutput.state == TOIOutputState.TOUCHING)
						{
							alpha = MathUtils.min(alpha0 + (1.0f - alpha0) * beta, 1.0f);
						}
						else
						{
							alpha = 1.0f;
						}
						
						c.m_toi = alpha;
						c.m_flags |= Contact.TOI_FLAG;
					}
					
					if (alpha < minAlpha)
					{
						// This is the minimum TOI found so far.
						minContact = c;
						minAlpha = alpha;
					}
				}
				
				if (minContact == null || 1.0f - 10.0f * Settings.EPSILON < minAlpha)
				{
					// No more TOI events. Done!
					m_stepComplete = true;
					break;
				}
				
				// Advance the bodies to the TOI.
				Fixture fA2 = minContact.FixtureA;
				Fixture fB2 = minContact.FixtureB;
				Body bA2 = fA2.Body;
				Body bB2 = fB2.Body;
				
				backup1.set_Renamed(bA2.m_sweep);
				backup2.set_Renamed(bB2.m_sweep);
				
				bA2.advance(minAlpha);
				bB2.advance(minAlpha);
				
				// The TOI contact likely has some new contact points.
				minContact.update(m_contactManager.m_contactListener);
				minContact.m_flags &= ~ Contact.TOI_FLAG;
				++minContact.m_toiCount;
				
				// Is the contact solid?
				if (minContact.Enabled == false || minContact.Touching == false)
				{
					// Restore the sweeps.
					minContact.Enabled = false;
					bA2.m_sweep.set_Renamed(backup1);
					bB2.m_sweep.set_Renamed(backup2);
					bA2.synchronizeTransform();
					bB2.synchronizeTransform();
					continue;
				}
				
				bA2.Awake = true;
				bB2.Awake = true;
				
				// Build the island
				island.clear();
				island.add(bA2);
				island.add(bB2);
				island.add(minContact);
				
				bA2.m_flags |= Body.e_islandFlag;
				bB2.m_flags |= Body.e_islandFlag;
				minContact.m_flags |= Contact.ISLAND_FLAG;
				
				// Get contacts on bodyA and bodyB.
				tempBodies[0] = bA2;
				tempBodies[1] = bB2;
				for (int i = 0; i < 2; ++i)
				{
					Body body = tempBodies[i];
					if (body.m_type == BodyType.DYNAMIC)
					{
						for (ContactEdge ce = body.m_contactList; ce != null; ce = ce.next)
						{
							if (island.m_bodyCount == island.m_bodyCapacity)
							{
								break;
							}
							
							if (island.m_contactCount == island.m_contactCapacity)
							{
								break;
							}
							
							Contact contact = ce.contact;
							
							// Has this contact already been added to the island?
							if ((contact.m_flags & Contact.ISLAND_FLAG) != 0)
							{
								continue;
							}
							
							// Only add static, kinematic, or bullet bodies.
							Body other = ce.other;
							if (other.m_type == BodyType.DYNAMIC && body.Bullet == false && other.Bullet == false)
							{
								continue;
							}
							
							// Skip sensors.
							bool sensorA = contact.m_fixtureA.m_isSensor;
							bool sensorB = contact.m_fixtureB.m_isSensor;
							if (sensorA || sensorB)
							{
								continue;
							}
							
							// Tentatively advance the body to the TOI.
							backup1.set_Renamed(other.m_sweep);
							if ((other.m_flags & Body.e_islandFlag) == 0)
							{
								other.advance(minAlpha);
							}
							
							// Update the contact points
							contact.update(m_contactManager.m_contactListener);
							
							// Was the contact disabled by the user?
							if (contact.Enabled == false)
							{
								other.m_sweep.set_Renamed(backup1);
								other.synchronizeTransform();
								continue;
							}
							
							// Are there contact points?
							if (contact.Touching == false)
							{
								other.m_sweep.set_Renamed(backup1);
								other.synchronizeTransform();
								continue;
							}
							
							// Add the contact to the island
							contact.m_flags |= Contact.ISLAND_FLAG;
							island.add(contact);
							
							// Has the other body already been added to the island?
							if ((other.m_flags & Body.e_islandFlag) != 0)
							{
								continue;
							}
							
							// Add the other body to the island.
							other.m_flags |= Body.e_islandFlag;
							
							if (other.m_type != BodyType.STATIC)
							{
								other.Awake = true;
							}
							
							island.add(other);
						}
					}
				}
				
				subStep.dt = (1.0f - minAlpha) * step.dt;
				subStep.inv_dt = 1.0f / subStep.dt;
				subStep.dtRatio = 1.0f;
				subStep.positionIterations = 20;
				subStep.velocityIterations = step.velocityIterations;
				subStep.warmStarting = false;
				island.solveTOI(subStep, bA2.m_islandIndex, bB2.m_islandIndex);
				
				// Reset island flags and synchronize broad-phase proxies.
				for (int i = 0; i < island.m_bodyCount; ++i)
				{
					Body body = island.m_bodies[i];
					body.m_flags &= ~ Body.e_islandFlag;
					
					if (body.m_type != BodyType.DYNAMIC)
					{
						continue;
					}
					
					body.synchronizeFixtures();
					
					// Invalidate all contact TOIs on this displaced body.
					for (ContactEdge ce = body.m_contactList; ce != null; ce = ce.next)
					{
						ce.contact.m_flags &= ~ (Contact.TOI_FLAG | Contact.ISLAND_FLAG);
					}
				}
				
				// Commit fixture proxy movements to the broad-phase so that new contacts are created.
				// Also, some contacts can be destroyed.
				m_contactManager.findNewContacts();
				
				if (m_subStepping)
				{
					m_stepComplete = false;
					break;
				}
			}
		}
		
		private void  drawJoint(Joint joint)
		{
			Body bodyA = joint.BodyA;
			Body bodyB = joint.BodyB;
			Transform xf1 = bodyA.getTransform();
			Transform xf2 = bodyB.getTransform();
			Vec2 x1 = xf1.p;
			Vec2 x2 = xf2.p;
			Vec2 p1 = pool.popVec2();
			Vec2 p2 = pool.popVec2();
			joint.getAnchorA(p1);
			joint.getAnchorB(p2);
			
			color.set_Renamed(0.5f, 0.8f, 0.8f);
			
			switch (joint.Type)
			{
				
				// TODO djm write after writing joints
				case DISTANCE: 
					m_debugDraw.drawSegment(p1, p2, color);
					break;
				
				
				case PULLEY:  {
						PulleyJoint pulley = (PulleyJoint) joint;
						Vec2 s1 = pulley.GroundAnchorA;
						Vec2 s2 = pulley.GroundAnchorB;
						m_debugDraw.drawSegment(s1, p1, color);
						m_debugDraw.drawSegment(s2, p2, color);
						m_debugDraw.drawSegment(s1, s2, color);
					}
					break;
				
				case CONSTANT_VOLUME: 
				case MOUSE: 
					// don't draw this
					break;
				
				default: 
					m_debugDraw.drawSegment(x1, p1, color);
					m_debugDraw.drawSegment(p1, p2, color);
					m_debugDraw.drawSegment(x2, p2, color);
					break;
				
			}
			pool.pushVec2(2);
		}
		
		// NOTE this corresponds to the liquid test, so the debugdraw can draw
		// the liquid particles correctly. They should be the same.
		private static System.Int32 LIQUID_INT = 1234598372;
		private float liquidLength = .12f;
		private float averageLinearVel = - 1;
		//UPGRADE_NOTE: Final was removed from the declaration of 'liquidOffset '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Vec2 liquidOffset = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'circCenterMoved '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Vec2 circCenterMoved = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'liquidColor '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Color3f liquidColor = new Color3f(.4f, .4f, 1f);
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'center '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Vec2 center = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'axis '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Vec2 axis = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'v1 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Vec2 v1 = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'v2 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Vec2 v2 = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'tlvertices '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Vec2Array tlvertices = new Vec2Array();
		
		private void  drawShape(Fixture fixture, Transform xf, Color3f color)
		{
			switch (fixture.Type)
			{
				
				case CIRCLE:  {
						CircleShape circle = (CircleShape) fixture.Shape;
						
						// Vec2 center = Mul(xf, circle.m_p);
						Transform.mulToOutUnsafe(xf, circle.m_p, center);
						float radius = circle.m_radius;
						xf.q.getXAxis(axis);
						
						if (fixture.UserData != null && fixture.UserData.Equals(LIQUID_INT))
						{
							Body b = fixture.Body;
							liquidOffset.set_Renamed(b.m_linearVelocity);
							float linVelLength = b.m_linearVelocity.length();
							if (averageLinearVel == - 1)
							{
								averageLinearVel = linVelLength;
							}
							else
							{
								averageLinearVel = .98f * averageLinearVel + .02f * linVelLength;
							}
							liquidOffset.mulLocal(liquidLength / averageLinearVel / 2);
							circCenterMoved.set_Renamed(center).addLocal(liquidOffset);
							center.subLocal(liquidOffset);
							m_debugDraw.drawSegment(center, circCenterMoved, liquidColor);
							return ;
						}
						
						m_debugDraw.drawSolidCircle(center, radius, axis, color);
					}
					break;
				
				
				case POLYGON:  {
						PolygonShape poly = (PolygonShape) fixture.Shape;
						int vertexCount = poly.m_count;
						assert(vertexCount <= Settings.maxPolygonVertices);
						Vec2[] vertices = tlvertices.get_Renamed(Settings.maxPolygonVertices);
						
						for (int i = 0; i < vertexCount; ++i)
						{
							// vertices[i] = Mul(xf, poly.m_vertices[i]);
							Transform.mulToOutUnsafe(xf, poly.m_vertices[i], vertices[i]);
						}
						
						m_debugDraw.drawSolidPolygon(vertices, vertexCount, color);
					}
					break;
				
				case EDGE:  {
						EdgeShape edge = (EdgeShape) fixture.Shape;
						Transform.mulToOutUnsafe(xf, edge.m_vertex1, v1);
						Transform.mulToOutUnsafe(xf, edge.m_vertex2, v2);
						m_debugDraw.drawSegment(v1, v2, color);
					}
					break;
				
				
				case CHAIN:  {
						ChainShape chain = (ChainShape) fixture.Shape;
						int count = chain.m_count;
						Vec2[] vertices = chain.m_vertices;
						
						Transform.mulToOutUnsafe(xf, vertices[0], v1);
						for (int i = 1; i < count; ++i)
						{
							Transform.mulToOutUnsafe(xf, vertices[i], v2);
							m_debugDraw.drawSegment(v1, v2, color);
							m_debugDraw.drawCircle(v1, 0.05f, color);
							v1.set_Renamed(v2);
						}
					}
					break;
				
				default: 
					break;
				
			}
		}
	}
	
	
	class WorldQueryWrapper : TreeCallback
	{
		public virtual bool treeCallback(int nodeId)
		{
			FixtureProxy proxy = (FixtureProxy) broadPhase.getUserData(nodeId);
			return callback.reportFixture(proxy.fixture);
		}
		
		internal BroadPhase broadPhase;
		internal QueryCallback callback;
	}
	
	
	
	class WorldRayCastWrapper : TreeRayCastCallback
	{
		
		// djm pooling
		//UPGRADE_NOTE: Final was removed from the declaration of 'output '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private RayCastOutput output = new RayCastOutput();
		//UPGRADE_NOTE: Final was removed from the declaration of 'temp '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Vec2 temp = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'point '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private Vec2 point = new Vec2();
		
		public virtual float raycastCallback(RayCastInput input, int nodeId)
		{
			System.Object userData = broadPhase.getUserData(nodeId);
			FixtureProxy proxy = (FixtureProxy) userData;
			Fixture fixture = proxy.fixture;
			int index = proxy.childIndex;
			bool hit = fixture.raycast(output, input, index);
			
			if (hit)
			{
				float fraction = output.fraction;
				// Vec2 point = (1.0f - fraction) * input.p1 + fraction * input.p2;
				temp.set_Renamed(input.p2).mulLocal(fraction);
				point.set_Renamed(input.p1).mulLocal(1 - fraction).addLocal(temp);
				return callback.reportFixture(fixture, point, output.normal, fraction);
			}
			
			return input.maxFraction;
		}
		
		internal BroadPhase broadPhase;
		internal RayCastCallback callback;
	}
	
}