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
using System.Diagnostics;
using Box2D.Callbacks;
using Box2D.Collision;
using Box2D.Collision.Broadphase;
using Box2D.Collision.Shapes;
using Box2D.Common;
using Box2D.Dynamics.Contacts;
using Box2D.Dynamics.Joints;
using Box2D.Pooling;
using Box2D.Pooling.Arrays;
using Box2D.Pooling.Normal;

namespace Box2D.Dynamics
{

    /// <summary>
    /// The world class manages all physics entities, dynamic simulation, and asynchronous queries. The
    /// world also contains efficient memory management facilities.
    /// </summary>
    /// <author>Daniel Murphy</author>
    public class World
    {
        public const int WORLD_POOL_SIZE = 100;
        public const int WORLD_POOL_CONTAINER_SIZE = 10;

        public const int NEW_FIXTURE = 0x0001;
        public const int LOCKED = 0x0002;
        public const int CLEAR_FORCES = 0x0004;


        // statistics gathering
        public int ActiveContacts;
        public int ContactPoolCount;

        protected internal int Flags;

        private readonly Vec2 m_gravity = new Vec2();

        // private Body m_groundBody;

        /// <summary>
        /// This is used to compute the time step ratio to support a variable time step.
        /// </summary>
        private float invDt0;

        // these are for debugging the solver
        private readonly bool m_subStepping;

        private bool m_stepComplete;

        private static readonly int ShapeTypesCount = Enum.GetValues(typeof(ShapeType)).Length;

        private readonly ContactRegister[][] contactStacks;

        public World(Vec2 gravity) :
            this(gravity, new DefaultWorldPool(WORLD_POOL_SIZE, WORLD_POOL_CONTAINER_SIZE))
        {
        }

        /// <summary>
        /// Construct a world object.
        /// </summary>
        /// <param name="gravity">the world gravity vector.</param>
        /// <param name="argPool"> </param>
        public World(Vec2 gravity, IWorldPool argPool)
        {
            contactStacks = new ContactRegister[ShapeTypesCount][];
            for (int i = 0; i < ShapeTypesCount; i++)
            {
                contactStacks[i] = new ContactRegister[ShapeTypesCount];
            }

            Pool = argPool;
            DestructionListener = null;
            DebugDraw = null;

            BodyList = null;
            JointList = null;

            BodyCount = 0;
            JointCount = 0;

            WarmStarting = true;
            ContinuousPhysics = true;
            m_subStepping = false;
            m_stepComplete = true;

            SleepingAllowed = true;
            m_gravity.Set(gravity);

            Flags = CLEAR_FORCES;

            invDt0 = 0f;

            ContactManager = new ContactManager(this);
            Profile = new Profile();

            InitializeRegisters();
        }

        public bool AllowSleep
        {
            get
            {
                return SleepingAllowed;
            }
            set
            {
                if (value == SleepingAllowed)
                {
                    return;
                }

                SleepingAllowed = value;
                if (SleepingAllowed == false)
                {
                    for (Body b = BodyList; b != null; b = b.Next)
                    {
                        b.Awake = true;
                    }
                }
            }
        }

        private void AddType(IDynamicStack<Contact> creator, ShapeType type1, ShapeType type2)
        {
            ContactRegister register = new ContactRegister {Creator = creator, Primary = true};
            contactStacks[(int)type1][(int)type2] = register;

            if (type1 != type2)
            {
                ContactRegister register2 = new ContactRegister {Creator = creator, Primary = false};
                contactStacks[(int)type2][(int)type1] = register2;
            }
        }

        private void InitializeRegisters()
        {
            AddType(Pool.GetCircleContactStack(), ShapeType.Circle, ShapeType.Circle);
            AddType(Pool.GetPolyCircleContactStack(), ShapeType.Polygon, ShapeType.Circle);
            AddType(Pool.GetPolyContactStack(), ShapeType.Polygon, ShapeType.Polygon);
            AddType(Pool.GetEdgeCircleContactStack(), ShapeType.Edge, ShapeType.Circle);
            AddType(Pool.GetEdgePolyContactStack(), ShapeType.Edge, ShapeType.Polygon);
            AddType(Pool.GetChainCircleContactStack(), ShapeType.Chain, ShapeType.Circle);
            AddType(Pool.GetChainPolyContactStack(), ShapeType.Chain, ShapeType.Polygon);
        }

        public Contact PopContact(Fixture fixtureA, int indexA, Fixture fixtureB, int indexB)
        {
            ShapeType type1 = fixtureA.Type;
            ShapeType type2 = fixtureB.Type;

            ContactRegister reg = contactStacks[(int)type1][(int)type2];
            IDynamicStack<Contact> creator = reg.Creator;

            if (creator != null)
            {
                if (reg.Primary)
                {
                    Contact c = creator.Pop();
                    c.Init(fixtureA, indexA, fixtureB, indexB);
                    return c;
                }
                else
                {
                    Contact c = creator.Pop();
                    c.Init(fixtureB, indexB, fixtureA, indexA);
                    return c;
                }
            }

            return null;
        }

        public void PushContact(Contact contact)
        {

            if (contact.Manifold.PointCount > 0)
            {
                contact.FixtureA.Body.Awake = true;
                contact.FixtureB.Body.Awake = true;
            }

            ShapeType type1 = contact.FixtureA.Type;
            ShapeType type2 = contact.FixtureB.Type;

            IDynamicStack<Contact> creator = contactStacks[(int)type1][(int)type2].Creator;
            creator.Push(contact);
        }

        /// <summary>
        /// create a rigid body given a definition. No reference to the definition is retained.
        /// </summary>
        /// <warning>This function is locked during callbacks.</warning>
        /// <param name="def"></param>
        /// <returns></returns>
        public Body CreateBody(BodyDef def)
        {
            Debug.Assert(Locked == false);
            if (Locked)
            {
                return null;
            }
            // TODO djm pooling
            Body b = new Body(def, this);

            // add to world doubly linked list
            b.Prev = null;
            b.Next = BodyList;
            if (BodyList != null)
            {
                BodyList.Prev = b;
            }
            BodyList = b;
            ++BodyCount;

            return b;
        }

        /// <summary>
        /// destroy a rigid body given a definition. No reference to the definition is retained. This
        /// function is locked during callbacks.
        /// </summary>
        /// <warning>This automatically deletes all associated shapes and joints.</warning>
        /// <warning>This function is locked during callbacks.</warning>
        /// <param name="body"></param>
        public void DestroyBody(Body body)
        {
            Debug.Assert(BodyCount > 0);
            Debug.Assert(Locked == false);
            if (Locked)
            {
                return;
            }

            // Delete the attached joints.
            JointEdge je = body.JointList;
            while (je != null)
            {
                JointEdge je0 = je;
                je = je.next;
                if (DestructionListener != null)
                {
                    DestructionListener.SayGoodbye(je0.joint);
                }

                DestroyJoint(je0.joint);

                body.JointList = je;
            }
            body.JointList = null;

            // Delete the attached contacts.
            ContactEdge ce = body.ContactList;
            while (ce != null)
            {
                ContactEdge ce0 = ce;
                ce = ce.Next;
                ContactManager.Destroy(ce0.Contact);
            }
            body.ContactList = null;

            Fixture f = body.FixtureList;
            while (f != null)
            {
                Fixture f0 = f;
                f = f.Next;

                if (DestructionListener != null)
                {
                    DestructionListener.SayGoodbye(f0);
                }

                f0.DestroyProxies(ContactManager.BroadPhase);
                f0.Destroy();
                // TODO djm recycle fixtures (here or in that destroy method)
                body.FixtureList = f;
                body.FixtureCount -= 1;
            }
            body.FixtureList = null;
            body.FixtureCount = 0;

            // Remove world body list.
            if (body.Prev != null)
            {
                body.Prev.Next = body.Next;
            }

            if (body.Next != null)
            {
                body.Next.Prev = body.Prev;
            }

            if (body == BodyList)
            {
                BodyList = body.Next;
            }

            --BodyCount;
            // TODO djm recycle body
        }

        /// <summary>
        /// create a joint to constrain bodies together. No reference to the definition is retained. This
        /// may cause the connected bodies to cease colliding.
        /// </summary>
        /// <warning>This function is locked during callbacks.</warning>
        /// <param name="def"></param>
        /// <returns></returns>
        public Joint CreateJoint(JointDef def)
        {
            Debug.Assert(Locked == false);
            if (Locked)
            {
                return null;
            }

            Joint j = Joint.Create(this, def);

            // Connect to the world list.
            j.Prev = null;
            j.Next = JointList;
            if (JointList != null)
            {
                JointList.Prev = j;
            }
            JointList = j;
            ++JointCount;

            // Connect to the bodies' doubly linked lists.
            j.EdgeA.joint = j;
            j.EdgeA.other = j.BodyB;
            j.EdgeA.prev = null;
            j.EdgeA.next = j.BodyA.JointList;
            if (j.BodyA.JointList != null)
            {
                j.BodyA.JointList.prev = j.EdgeA;
            }
            j.BodyA.JointList = j.EdgeA;

            j.EdgeB.joint = j;
            j.EdgeB.other = j.BodyA;
            j.EdgeB.prev = null;
            j.EdgeB.next = j.BodyB.JointList;
            if (j.BodyB.JointList != null)
            {
                j.BodyB.JointList.prev = j.EdgeB;
            }
            j.BodyB.JointList = j.EdgeB;

            Body bodyA = def.bodyA;
            Body bodyB = def.bodyB;

            // If the joint prevents collisions, then flag any contacts for filtering.
            if (def.collideConnected == false)
            {
                ContactEdge edge = bodyB.ContactList;
                while (edge != null)
                {
                    if (edge.Other == bodyA)
                    {
                        // Flag the contact for filtering at the next time step (where either
                        // body is awake).
                        edge.Contact.SetFlagForFiltering();
                    }

                    edge = edge.Next;
                }
            }

            // Note: creating a joint doesn't wake the bodies.

            return j;
        }

        /// <summary>
        /// destroy a joint. This may cause the connected bodies to begin colliding.
        /// </summary>
        /// <warning>This function is locked during callbacks.</warning>
        /// <param name="j"></param>
        public void DestroyJoint(Joint j)
        {
            Debug.Assert(Locked == false);
            if (Locked)
            {
                return;
            }

            bool collideConnected = j.CollideConnected;

            // Remove from the doubly linked list.
            if (j.Prev != null)
            {
                j.Prev.Next = j.Next;
            }

            if (j.Next != null)
            {
                j.Next.Prev = j.Prev;
            }

            if (j == JointList)
            {
                JointList = j.Next;
            }

            // Disconnect from island graph.
            Body bodyA = j.BodyA;
            Body bodyB = j.BodyB;

            // Wake up connected bodies.
            bodyA.Awake = true;
            bodyB.Awake = true;

            // Remove from body 1.
            if (j.EdgeA.prev != null)
            {
                j.EdgeA.prev.next = j.EdgeA.next;
            }

            if (j.EdgeA.next != null)
            {
                j.EdgeA.next.prev = j.EdgeA.prev;
            }

            if (j.EdgeA == bodyA.JointList)
            {
                bodyA.JointList = j.EdgeA.next;
            }

            j.EdgeA.prev = null;
            j.EdgeA.next = null;

            // Remove from body 2
            if (j.EdgeB.prev != null)
            {
                j.EdgeB.prev.next = j.EdgeB.next;
            }

            if (j.EdgeB.next != null)
            {
                j.EdgeB.next.prev = j.EdgeB.prev;
            }

            if (j.EdgeB == bodyB.JointList)
            {
                bodyB.JointList = j.EdgeB.next;
            }

            j.EdgeB.prev = null;
            j.EdgeB.next = null;

            Joint.Destroy(j);

            Debug.Assert(JointCount > 0);
            --JointCount;

            // If the joint prevents collisions, then flag any contacts for filtering.
            if (collideConnected == false)
            {
                ContactEdge edge = bodyB.ContactList;
                while (edge != null)
                {
                    if (edge.Other == bodyA)
                    {
                        // Flag the contact for filtering at the next time step (where either
                        // body is awake).
                        edge.Contact.SetFlagForFiltering();
                    }

                    edge = edge.Next;
                }
            }
        }

        // djm pooling
        private readonly TimeStep step = new TimeStep();
        private readonly Timer stepTimer = new Timer();
        private readonly Timer tempTimer = new Timer();

        /// <summary>
        /// Take a time step. This performs collision detection, integration, and constraint solution.
        /// </summary>
        /// <param name="dt">the amount of time to simulate, this should not vary.</param>
        /// <param name="velocityIterations">for the velocity constraint solver.</param>
        /// <param name="positionIterations">for the position constraint solver.</param>
        public void Step(float dt, int velocityIterations, int positionIterations)
        {
            stepTimer.Reset();
            // log.debug("Starting step");
            // If new fixtures were added, we need to find the new contacts.
            if ((Flags & NEW_FIXTURE) == NEW_FIXTURE)
            {
                // log.debug("There's a new fixture, lets look for new contacts");
                ContactManager.FindNewContacts();
                Flags &= ~NEW_FIXTURE;
            }

            Flags |= LOCKED;

            step.Dt = dt;
            step.VelocityIterations = velocityIterations;
            step.PositionIterations = positionIterations;
            if (dt > 0.0f)
            {
                step.InvDt = 1.0f / dt;
            }
            else
            {
                step.InvDt = 0.0f;
            }

            step.DtRatio = invDt0 * dt;

            step.WarmStarting = WarmStarting;

            // Update contacts. This is where some contacts are destroyed.
            tempTimer.Reset();
            ContactManager.Collide();
            Profile.Collide = tempTimer.Milliseconds;

            // Integrate velocities, solve velocity constraints, and integrate positions.
            if (m_stepComplete && step.Dt > 0.0f)
            {
                tempTimer.Reset();
                Solve(step);
                Profile.Solve = tempTimer.Milliseconds;
            }

            // Handle TOI events.
            if (ContinuousPhysics && step.Dt > 0.0f)
            {
                tempTimer.Reset();
                SolveToi(step);
                Profile.SolveToi = tempTimer.Milliseconds;
            }

            if (step.Dt > 0.0f)
            {
                invDt0 = step.InvDt;
            }

            if ((Flags & CLEAR_FORCES) == CLEAR_FORCES)
            {
                ClearForces();
            }

            Flags &= ~LOCKED;
            // log.debug("ending step");

            Profile.Step = stepTimer.Milliseconds;
        }

        /// <summary>
        /// Call this after you are done with time steps to clear the forces. You normally call this after
        /// each call to Step, unless you are performing sub-steps. By default, forces will be
        /// automatically cleared, so you don't need to call this function.
        /// </summary>
        /// <seealso>
        ///   <cref>setAutoClearForces</cref>
        /// </seealso>
        public void ClearForces()
        {
            for (Body body = BodyList; body != null; body = body.Next)
            {
                body.Force.SetZero();
                body.Torque = 0.0f;
            }
        }

        private readonly Color3f color = new Color3f();
        private readonly Transform xf = new Transform();
        private readonly Vec2Array avs = new Vec2Array();

        /// <summary>
        /// Call this to draw shapes and other debug draw data.
        /// </summary>
        public void DrawDebugData()
        {
            if (DebugDraw == null)
            {
                return;
            }

            DebugDraw.DrawFlags flags = DebugDraw.Flags;

            if ((flags & DebugDraw.DrawFlags.Shape) == DebugDraw.DrawFlags.Shape)
            {
                for (Body b = BodyList; b != null; b = b.Next)
                {
                    xf.Set(b.GetTransform());
                    for (Fixture f = b.FixtureList; f != null; f = f.Next)
                    {
                        if (b.Active == false)
                        {
                            color.Set(0.5f, 0.5f, 0.3f);
                            DrawShape(f, xf, color);
                        }
                        else if (b.Type == BodyType.Static)
                        {
                            color.Set(0.5f, 0.9f, 0.3f);
                            DrawShape(f, xf, color);
                        }
                        else if (b.Type == BodyType.Kinematic)
                        {
                            color.Set(0.5f, 0.5f, 0.9f);
                            DrawShape(f, xf, color);
                        }
                        else if (b.Awake == false)
                        {
                            color.Set(0.5f, 0.5f, 0.5f);
                            DrawShape(f, xf, color);
                        }
                        else
                        {
                            color.Set(0.9f, 0.7f, 0.7f);
                            DrawShape(f, xf, color);
                        }
                    }
                }
            }

            if ((flags & DebugDraw.DrawFlags.Joint) == DebugDraw.DrawFlags.Joint)
            {
                for (Joint j = JointList; j != null; j = j.Next)
                {
                    DrawJoint(j);
                }
            }

            if ((flags & DebugDraw.DrawFlags.Pair) == DebugDraw.DrawFlags.Pair)
            {
                color.Set(0.3f, 0.9f, 0.9f);
                //for (Contact c = ContactManager.ContactList; c != null; c = c.Next)
                //{
                //    Fixture fixtureA = c.getFixtureA();
                //    Fixture fixtureB = c.getFixtureB();

                //    fixtureA.getAABB(childIndex).getCenterToOut(cA);
                //    fixtureB.getAABB().getCenterToOut(cB);

                //    m_debugDraw.drawSegment(cA, cB, color);
                //}
            }

            if ((flags & DebugDraw.DrawFlags.AABB) == DebugDraw.DrawFlags.AABB)
            {
                color.Set(0.9f, 0.3f, 0.9f);

                for (Body b = BodyList; b != null; b = b.Next)
                {
                    if (b.Active == false)
                    {
                        continue;
                    }

                    for (Fixture f = b.FixtureList; f != null; f = f.Next)
                    {

                        for (int i = 0; i < f.ProxyCount; ++i)
                        {
                            FixtureProxy proxy = f.Proxies[i];
                            AABB aabb = ContactManager.BroadPhase.GetFatAABB(proxy.ProxyId);
                            Vec2[] vs = avs.Get(4);
                            vs[0].Set(aabb.LowerBound.X, aabb.LowerBound.Y);
                            vs[1].Set(aabb.UpperBound.X, aabb.LowerBound.Y);
                            vs[2].Set(aabb.UpperBound.X, aabb.UpperBound.Y);
                            vs[3].Set(aabb.LowerBound.X, aabb.UpperBound.Y);

                            DebugDraw.DrawPolygon(vs, 4, color);
                        }
                    }
                }
            }

            if ((flags & DebugDraw.DrawFlags.CenterOfMass) == DebugDraw.DrawFlags.CenterOfMass)
            {
                for (Body b = BodyList; b != null; b = b.Next)
                {
                    xf.Set(b.GetTransform());
                    xf.P.Set(b.WorldCenter);
                    DebugDraw.DrawTransform(xf);
                }
            }

            if ((flags & DebugDraw.DrawFlags.DynamicTree) == DebugDraw.DrawFlags.DynamicTree)
            {
                ContactManager.BroadPhase.DrawTree(DebugDraw);
            }
        }

        private readonly WorldQueryWrapper wqwrapper = new WorldQueryWrapper();

        /// <summary>
        /// Query the world for all fixtures that potentially overlap the provided AABB.
        /// </summary>
        /// <param name="callback">a user implemented callback class.</param>
        /// <param name="aabb">the query box.</param>
        public void QueryAABB(IQueryCallback callback, AABB aabb)
        {
            wqwrapper.BroadPhase = ContactManager.BroadPhase;
            wqwrapper.Callback = callback;
            ContactManager.BroadPhase.Query(wqwrapper, aabb);
        }

        private readonly WorldRayCastWrapper wrcwrapper = new WorldRayCastWrapper();
        private readonly RayCastInput input = new RayCastInput();

        /// <summary>
        /// Ray-cast the world for all fixtures in the path of the ray. Your callback controls whether you
        /// get the closest point, any point, or n-points. The ray-cast ignores shapes that contain the
        /// starting point.
        /// </summary>
        /// <param name="callback">a user implemented callback class.</param>
        /// <param name="point1">the ray starting point</param>
        /// <param name="point2">the ray ending point</param>
        public void Raycast(IRayCastCallback callback, Vec2 point1, Vec2 point2)
        {
            wrcwrapper.BroadPhase = ContactManager.BroadPhase;
            wrcwrapper.Callback = callback;
            input.MaxFraction = 1.0f;
            input.P1.Set(point1);
            input.P2.Set(point2);
            ContactManager.BroadPhase.Raycast(wrcwrapper, input);
        }

        public IWorldPool Pool { get; private set; }

        /// <summary>
        /// Register a destruction listener. The listener is owned by you and must remain in scope.
        /// </summary>
        public IDestructionListener DestructionListener { private get; set; }

        /// <summary>
        /// Register a contact filter to provide specific control over collision. Otherwise the default
        /// filter is used (_defaultFilter). The listener is owned by you and must remain in scope.
        /// </summary>
        public ContactFilter ContactFilter
        {
            set
            {
                ContactManager.ContactFilter = value;
            }
        }

        /// <summary>
        /// Register a contact event listener. The listener is owned by you and must remain in scope.
        /// </summary>
        public IContactListener ContactListener
        {
            set
            {
                ContactManager.ContactListener = value;
            }
        }

        /// <summary>
        /// Register a routine for debug drawing. The debug draw functions are called inside with
        /// World.DrawDebugData method. The debug draw object is owned by you and must remain in scope.
        /// </summary>
        public DebugDraw DebugDraw { private get; set; }

        /// <summary>
        /// Get the world body list. With the returned body, use Body.getNext to get the next body in the
        /// world list. A null body indicates the end of the list.
        /// </summary>
        /// <returns>the head of the world body list.</returns>
        public Body BodyList { get; private set; }

        /// <summary>
        /// Get the world joint list. With the returned joint, use Joint.getNext to get the next joint in
        /// the world list. A null joint indicates the end of the list.
        /// </summary>
        /// <returns>the head of the world joint list.</returns>
        public Joint JointList { get; private set; }

        /// <summary>
        /// Get the world contact list. With the returned contact, use Contact.getNext to get the next
        /// contact in the world list. A null contact indicates the end of the list.
        /// </summary>
        /// <returns>the head of the world contact list.</returns>
        /// <warning>contacts are created and destroyed in the middle of a time step. Use ContactListener to avoid missing contacts.</warning>
        public Contact ContactList
        {
            get
            {
                return ContactManager.ContactList;
            }
        }

        public bool SleepingAllowed { get; set; }

        /// <summary>
        /// Enable/disable warm starting. For testing.
        /// </summary>
        public bool WarmStarting { get; set; }

        /// <summary>
        /// Enable/disable continuous physics. For testing.
        /// </summary>
        public bool ContinuousPhysics { get; set; }

        /// <summary>
        /// Get the number of broad-phase proxies.
        /// </summary>
        /// <returns></returns>
        public int ProxyCount
        {
            get
            {
                return ContactManager.BroadPhase.ProxyCount;
            }
        }

        /// <summary>
        /// Get the number of bodies.
        /// </summary>
        /// <returns></returns>
        public int BodyCount { get; private set; }

        /// <summary>
        /// Get the number of joints.
        /// </summary>
        /// <returns></returns>
        public int JointCount { get; private set; }

        /// <summary>
        /// Get the number of contacts (each may have 0 or more contact points).
        /// </summary>
        /// <returns></returns>
        public int ContactCount
        {
            get
            {
                return ContactManager.ContactCount;
            }
        }

        /// <summary>
        /// Gets the height of the dynamic tree
        /// </summary>
        /// <returns></returns>
        public int TreeHeight
        {
            get
            {
                return ContactManager.BroadPhase.TreeHeight;
            }
        }

        /// <summary>
        /// Gets the balance of the dynamic tree
        /// </summary>
        /// <returns></returns>
        public int TreeBalance
        {
            get
            {
                return ContactManager.BroadPhase.TreeBalance;
            }
        }

        /// <summary>
        /// Gets the quality of the dynamic tree
        /// </summary>
        /// <returns></returns>
        public float TreeQuality
        {
            get
            {
                return ContactManager.BroadPhase.TreeQuality;
            }
        }

        /// <summary>
        /// Gets or sets the global gravity vector.
        /// </summary>
        public Vec2 Gravity
        {
            get
            {
                return m_gravity;
            }
            set
            {
                m_gravity.Set(value);
            }
        }

        /// <summary>
        /// Is the world locked (in the middle of a time step).
        /// </summary>
        /// <returns></returns>
        public bool Locked
        {
            get
            {
                return (Flags & LOCKED) == LOCKED;
            }
        }

        /// <summary>
        /// Gets or sets the flag that controls automatic clearing of forces after each time step.
        /// </summary>
        public bool AutoClearForces
        {
            get
            {
                return (Flags & CLEAR_FORCES) == CLEAR_FORCES;
            }
            set
            {
                if (value)
                {
                    Flags |= CLEAR_FORCES;
                }
                else
                {
                    Flags &= ~CLEAR_FORCES;
                }
            }
        }

        /// <summary>
        /// Get the contact manager for testing purposes
        /// </summary>
        /// <returns></returns>
        public ContactManager ContactManager { get; protected internal set; }

        public Profile Profile { get; private set; }

        private readonly Island island = new Island();
        private Body[] stack = new Body[10]; // TODO djm find a good initial stack number;
        private readonly Profile islandProfile = new Profile();
        private readonly Timer broadphaseTimer = new Timer();

        private void Solve(TimeStep step)
        {
            Profile.SolveInit = 0;
            Profile.SolveVelocity = 0;
            Profile.SolvePosition = 0;

            // Size the island for the worst case.
            island.Init(BodyCount, ContactManager.ContactCount, JointCount, ContactManager.ContactListener);

            // Clear all the island flags.
            for (Body b = BodyList; b != null; b = b.Next)
            {
                b.Flags &= ~Body.TypeFlags.Island;
            }
            for (Contact c = ContactManager.ContactList; c != null; c = c.Next)
            {
                c.Flags &= ~Contact.ContactFlags.Island;
            }
            for (Joint j = JointList; j != null; j = j.Next)
            {
                j.IslandFlag = false;
            }

            // Build and simulate all awake islands.
            int stackSize = BodyCount;
            if (stack.Length < stackSize)
            {
                stack = new Body[stackSize];
            }
            for (Body seed = BodyList; seed != null; seed = seed.Next)
            {
                if ((seed.Flags & Body.TypeFlags.Island) == Body.TypeFlags.Island)
                {
                    continue;
                }

                if (seed.Awake == false || seed.Active == false)
                {
                    continue;
                }

                // The seed can be dynamic or kinematic.
                if (seed.Type == BodyType.Static)
                {
                    continue;
                }

                // Reset island and stack.
                island.Clear();
                int stackCount = 0;
                stack[stackCount++] = seed;
                seed.Flags |= Body.TypeFlags.Island;

                // Perform a depth first search (DFS) on the constraint graph.
                while (stackCount > 0)
                {
                    // Grab the next body off the stack and add it to the island.
                    Body b = stack[--stackCount];
                    Debug.Assert(b.Active == true);
                    island.Add(b);

                    // Make sure the body is awake.
                    b.Awake = true;

                    // To keep islands as small as possible, we don't
                    // propagate islands across static bodies.
                    if (b.Type == BodyType.Static)
                    {
                        continue;
                    }

                    // Search all contacts connected to this body.
                    for (ContactEdge ce = b.ContactList; ce != null; ce = ce.Next)
                    {
                        Contact contact = ce.Contact;

                        // Has this contact already been added to an island?
                        if ((contact.Flags & Contact.ContactFlags.Island) == Contact.ContactFlags.Island)
                        {
                            continue;
                        }

                        // Is this contact solid and touching?
                        if (contact.Enabled == false || contact.Touching == false)
                        {
                            continue;
                        }

                        // Skip sensors.
                        bool sensorA = contact.FixtureA.IsSensor;
                        bool sensorB = contact.FixtureB.IsSensor;
                        if (sensorA || sensorB)
                        {
                            continue;
                        }

                        island.Add(contact);
                        contact.Flags |= Contact.ContactFlags.Island;

                        Body other = ce.Other;

                        // Was the other body already added to this island?
                        if ((other.Flags & Body.TypeFlags.Island) == Body.TypeFlags.Island)
                        {
                            continue;
                        }

                        Debug.Assert(stackCount < stackSize);
                        stack[stackCount++] = other;
                        other.Flags |= Body.TypeFlags.Island;
                    }

                    // Search all joints connect to this body.
                    for (JointEdge je = b.JointList; je != null; je = je.next)
                    {
                        if (je.joint.IslandFlag)
                        {
                            continue;
                        }

                        Body other = je.other;

                        // Don't simulate joints connected to inactive bodies.
                        if (other.Active == false)
                        {
                            continue;
                        }

                        island.Add(je.joint);
                        je.joint.IslandFlag = true;

                        if ((other.Flags & Body.TypeFlags.Island) == Body.TypeFlags.Island)
                        {
                            continue;
                        }

                        Debug.Assert(stackCount < stackSize);
                        stack[stackCount++] = other;
                        other.Flags |= Body.TypeFlags.Island;
                    }
                }
                island.Solve(islandProfile, step, m_gravity, SleepingAllowed);
                Profile.SolveInit += islandProfile.SolveInit;
                Profile.SolveVelocity += islandProfile.SolveVelocity;
                Profile.SolvePosition += islandProfile.SolvePosition;

                // Post solve cleanup.
                for (int i = 0; i < island.BodyCount; ++i)
                {
                    // Allow static bodies to participate in other islands.
                    Body b = island.Bodies[i];
                    if (b.Type == BodyType.Static)
                    {
                        b.Flags &= ~Body.TypeFlags.Island;
                    }
                }
            }

            broadphaseTimer.Reset();
            // Synchronize fixtures, check for out of range bodies.
            for (Body b = BodyList; b != null; b = b.Next)
            {
                // If a body was not in an island then it did not move.
                if ((b.Flags & Body.TypeFlags.Island) == 0)
                {
                    continue;
                }

                if (b.Type == BodyType.Static)
                {
                    continue;
                }

                // Update fixtures (for broad-phase).
                b.SynchronizeFixtures();
            }

            // Look for new contacts.
            ContactManager.FindNewContacts();
            Profile.Broadphase = broadphaseTimer.Milliseconds;
        }

        private readonly Island toiIsland = new Island();
        private readonly TimeOfImpact.TOIInput toiInput = new TimeOfImpact.TOIInput();
        private readonly TimeOfImpact.TOIOutput toiOutput = new TimeOfImpact.TOIOutput();
        private readonly TimeStep subStep = new TimeStep();
        private readonly Body[] tempBodies = new Body[2];
        private readonly Sweep backup1 = new Sweep();
        private readonly Sweep backup2 = new Sweep();

        private void SolveToi(TimeStep step)
        {
            Island island = toiIsland;
            island.Init(2 * Settings.MAX_TOI_CONTACTS, Settings.MAX_TOI_CONTACTS, 0, ContactManager.ContactListener);
            if (m_stepComplete)
            {
                for (Body b = BodyList; b != null; b = b.Next)
                {
                    b.Flags &= ~Body.TypeFlags.Island;
                    b.Sweep.Alpha0 = 0.0f;
                }

                for (Contact c = ContactManager.ContactList; c != null; c = c.Next)
                {
                    // Invalidate TOI
                    c.Flags &= ~(Contact.ContactFlags.ToiFlag | Contact.ContactFlags.Island);
                    c.ToiCount = 0;
                    c.Toi = 1.0f;
                }
            }

            // Find TOI events and solve them.
            for (; ; )
            {
                // Find the first TOI.
                Contact minContact = null;
                float minAlpha = 1.0f;

                for (Contact c = ContactManager.ContactList; c != null; c = c.Next)
                {
                    // Is this contact disabled?
                    if (c.Enabled == false)
                    {
                        continue;
                    }

                    // Prevent excessive sub-stepping.
                    if (c.ToiCount > Settings.MAX_SUB_STEPS)
                    {
                        continue;
                    }

                    float alpha;
                    if ((c.Flags & Contact.ContactFlags.ToiFlag) != 0)
                    {
                        // This contact has a valid cached TOI.
                        alpha = c.Toi;
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

                        BodyType typeA = bA.Type;
                        BodyType typeB = bB.Type;
                        Debug.Assert(typeA == BodyType.Dynamic || typeB == BodyType.Dynamic);

                        bool activeA = bA.Awake && typeA != BodyType.Static;
                        bool activeB = bB.Awake && typeB != BodyType.Static;

                        // Is at least one body active (awake and dynamic or kinematic)?
                        if (activeA == false && activeB == false)
                        {
                            continue;
                        }

                        bool collideA = bA.Bullet || typeA != BodyType.Dynamic;
                        bool collideB = bB.Bullet || typeB != BodyType.Dynamic;

                        // Are these two non-bullet dynamic bodies?
                        if (collideA == false && collideB == false)
                        {
                            continue;
                        }

                        // Compute the TOI for this contact.
                        // Put the sweeps onto the same time interval.
                        float alpha0 = bA.Sweep.Alpha0;

                        if (bA.Sweep.Alpha0 < bB.Sweep.Alpha0)
                        {
                            alpha0 = bB.Sweep.Alpha0;
                            bA.Sweep.Advance(alpha0);
                        }
                        else if (bB.Sweep.Alpha0 < bA.Sweep.Alpha0)
                        {
                            alpha0 = bA.Sweep.Alpha0;
                            bB.Sweep.Advance(alpha0);
                        }

                        Debug.Assert(alpha0 < 1.0f);

                        int indexA = c.ChildIndexA;
                        int indexB = c.ChildIndexB;

                        // Compute the time of impact in interval [0, minTOI]
                        TimeOfImpact.TOIInput input = toiInput;
                        input.ProxyA.Set(fA.Shape, indexA);
                        input.ProxyB.Set(fB.Shape, indexB);
                        input.SweepA.Set(bA.Sweep);
                        input.SweepB.Set(bB.Sweep);
                        input.tMax = 1.0f;

                        Pool.GetTimeOfImpact().GetTimeOfImpact(toiOutput, input);

                        // Beta is the fraction of the remaining portion of the .
                        float beta = toiOutput.T;
                        if (toiOutput.State == TimeOfImpact.TOIOutputState.Touching)
                        {
                            alpha = MathUtils.Min(alpha0 + (1.0f - alpha0) * beta, 1.0f);
                        }
                        else
                        {
                            alpha = 1.0f;
                        }

                        c.Toi = alpha;
                        c.Flags |= Contact.ContactFlags.ToiFlag;
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

                backup1.Set(bA2.Sweep);
                backup2.Set(bB2.Sweep);

                bA2.Advance(minAlpha);
                bB2.Advance(minAlpha);

                // The TOI contact likely has some new contact points.
                minContact.Update(ContactManager.ContactListener);
                minContact.Flags &= ~Contact.ContactFlags.ToiFlag;
                ++minContact.ToiCount;

                // Is the contact solid?
                if (minContact.Enabled == false || minContact.Touching == false)
                {
                    // Restore the sweeps.
                    minContact.Enabled = false;
                    bA2.Sweep.Set(backup1);
                    bB2.Sweep.Set(backup2);
                    bA2.SynchronizeTransform();
                    bB2.SynchronizeTransform();
                    continue;
                }

                bA2.Awake = true;
                bB2.Awake = true;

                // Build the island
                island.Clear();
                island.Add(bA2);
                island.Add(bB2);
                island.Add(minContact);

                bA2.Flags |= Body.TypeFlags.Island;
                bB2.Flags |= Body.TypeFlags.Island;
                minContact.Flags |= Contact.ContactFlags.Island;

                // Get contacts on bodyA and bodyB.
                tempBodies[0] = bA2;
                tempBodies[1] = bB2;
                for (int i = 0; i < 2; ++i)
                {
                    Body body = tempBodies[i];
                    if (body.Type == BodyType.Dynamic)
                    {
                        for (ContactEdge ce = body.ContactList; ce != null; ce = ce.Next)
                        {
                            if (island.BodyCount == island.BodyCapacity)
                            {
                                break;
                            }

                            if (island.ContactCount == island.ContactCapacity)
                            {
                                break;
                            }

                            Contact contact = ce.Contact;

                            // Has this contact already been added to the island?
                            if ((contact.Flags & Contact.ContactFlags.Island) != 0)
                            {
                                continue;
                            }

                            // Only add static, kinematic, or bullet bodies.
                            Body other = ce.Other;
                            if (other.Type == BodyType.Dynamic && body.Bullet == false && other.Bullet == false)
                            {
                                continue;
                            }

                            // Skip sensors.
                            bool sensorA = contact.FixtureA.IsSensor;
                            bool sensorB = contact.FixtureB.IsSensor;
                            if (sensorA || sensorB)
                            {
                                continue;
                            }

                            // Tentatively advance the body to the TOI.
                            backup1.Set(other.Sweep);
                            if ((other.Flags & Body.TypeFlags.Island) == 0)
                            {
                                other.Advance(minAlpha);
                            }

                            // Update the contact points
                            contact.Update(ContactManager.ContactListener);

                            // Was the contact disabled by the user?
                            if (contact.Enabled == false)
                            {
                                other.Sweep.Set(backup1);
                                other.SynchronizeTransform();
                                continue;
                            }

                            // Are there contact points?
                            if (contact.Touching == false)
                            {
                                other.Sweep.Set(backup1);
                                other.SynchronizeTransform();
                                continue;
                            }

                            // Add the contact to the island
                            contact.Flags |= Contact.ContactFlags.Island;
                            island.Add(contact);

                            // Has the other body already been added to the island?
                            if ((other.Flags & Body.TypeFlags.Island) != 0)
                            {
                                continue;
                            }

                            // Add the other body to the island.
                            other.Flags |= Body.TypeFlags.Island;

                            if (other.Type != BodyType.Static)
                            {
                                other.Awake = true;
                            }

                            island.Add(other);
                        }
                    }
                }

                subStep.Dt = (1.0f - minAlpha) * step.Dt;
                subStep.InvDt = 1.0f / subStep.Dt;
                subStep.DtRatio = 1.0f;
                subStep.PositionIterations = 20;
                subStep.VelocityIterations = step.VelocityIterations;
                subStep.WarmStarting = false;
                island.SolveToi(subStep, bA2.IslandIndex, bB2.IslandIndex);

                // Reset island flags and synchronize broad-phase proxies.
                for (int i = 0; i < island.BodyCount; ++i)
                {
                    Body body = island.Bodies[i];
                    body.Flags &= ~Body.TypeFlags.Island;

                    if (body.Type != BodyType.Dynamic)
                    {
                        continue;
                    }

                    body.SynchronizeFixtures();

                    // Invalidate all contact TOIs on this displaced body.
                    for (ContactEdge ce = body.ContactList; ce != null; ce = ce.Next)
                    {
                        ce.Contact.Flags &= ~(Contact.ContactFlags.ToiFlag | Contact.ContactFlags.ToiFlag);
                    }
                }

                // Commit fixture proxy movements to the broad-phase so that new contacts are created.
                // Also, some contacts can be destroyed.
                ContactManager.FindNewContacts();

                if (m_subStepping)
                {
                    m_stepComplete = false;
                    break;
                }
            }
        }

        private void DrawJoint(Joint joint)
        {
            Body bodyA = joint.BodyA;
            Body bodyB = joint.BodyB;
            Transform xf1 = bodyA.GetTransform();
            Transform xf2 = bodyB.GetTransform();
            Vec2 x1 = xf1.P;
            Vec2 x2 = xf2.P;
            Vec2 p1 = Pool.PopVec2();
            Vec2 p2 = Pool.PopVec2();
            joint.GetAnchorA(p1);
            joint.GetAnchorB(p2);

            color.Set(0.5f, 0.8f, 0.8f);

            switch (joint.Type)
            {

                // TODO djm write after writing joints
                case JointType.Distance:
                    DebugDraw.DrawSegment(p1, p2, color);
                    break;


                case JointType.Pulley:
                    {
                        PulleyJoint pulley = (PulleyJoint)joint;
                        Vec2 s1 = pulley.GroundAnchorA;
                        Vec2 s2 = pulley.GroundAnchorB;
                        DebugDraw.DrawSegment(s1, p1, color);
                        DebugDraw.DrawSegment(s2, p2, color);
                        DebugDraw.DrawSegment(s1, s2, color);
                    }
                    break;

                case JointType.ConstantVolume:
                case JointType.Mouse:
                    // don't draw this
                    break;

                default:
                    DebugDraw.DrawSegment(x1, p1, color);
                    DebugDraw.DrawSegment(p1, p2, color);
                    DebugDraw.DrawSegment(x2, p2, color);
                    break;

            }
            Pool.PushVec2(2);
        }

        // NOTE this corresponds to the liquid test, so the debugdraw can draw
        // the liquid particles correctly. They should be the same.
        private const int LIQUID_INT = 1234598372;
        private const float LIQUID_LENGTH = .12f;
        private float averageLinearVel = -1;
        private readonly Vec2 liquidOffset = new Vec2();
        private readonly Vec2 circCenterMoved = new Vec2();
        private readonly Color3f liquidColor = new Color3f(.4f, .4f, 1f);

        private readonly Vec2 center = new Vec2();
        private readonly Vec2 axis = new Vec2();
        private readonly Vec2 v1 = new Vec2();
        private readonly Vec2 v2 = new Vec2();
        private readonly Vec2Array tlvertices = new Vec2Array();

        private void DrawShape(Fixture fixture, Transform xf, Color3f color)
        {
            switch (fixture.Type)
            {

                case ShapeType.Circle:
                    {
                        CircleShape circle = (CircleShape)fixture.Shape;

                        // Vec2 center = Mul(xf, circle.m_p);
                        Transform.MulToOutUnsafe(xf, circle.P, center);
                        float radius = circle.Radius;
                        xf.Q.GetXAxis(axis);

                        if (fixture.UserData != null && fixture.UserData.Equals(LIQUID_INT))
                        {
                            Body b = fixture.Body;
                            liquidOffset.Set(b.LinearVelocity);
                            float linVelLength = b.LinearVelocity.Length();
                            if (averageLinearVel == -1)
                            {
                                averageLinearVel = linVelLength;
                            }
                            else
                            {
                                averageLinearVel = .98f * averageLinearVel + .02f * linVelLength;
                            }
                            liquidOffset.MulLocal(LIQUID_LENGTH / averageLinearVel / 2);
                            circCenterMoved.Set(center).AddLocal(liquidOffset);
                            center.SubLocal(liquidOffset);
                            DebugDraw.DrawSegment(center, circCenterMoved, liquidColor);
                            return;
                        }

                        DebugDraw.DrawSolidCircle(center, radius, axis, color);
                    }
                    break;


                case ShapeType.Polygon:
                    {
                        PolygonShape poly = (PolygonShape)fixture.Shape;
                        int vertexCount = poly.VertexCount;
                        Debug.Assert(vertexCount <= Settings.MAX_POLYGON_VERTICES);
                        Vec2[] vertices = tlvertices.Get(Settings.MAX_POLYGON_VERTICES);

                        for (int i = 0; i < vertexCount; ++i)
                        {
                            // vertices[i] = Mul(xf, poly.m_vertices[i]);
                            Transform.MulToOutUnsafe(xf, poly.Vertices[i], vertices[i]);
                        }

                        DebugDraw.DrawSolidPolygon(vertices, vertexCount, color);
                    }
                    break;

                case ShapeType.Edge:
                    {
                        EdgeShape edge = (EdgeShape)fixture.Shape;
                        Transform.MulToOutUnsafe(xf, edge.Vertex1, v1);
                        Transform.MulToOutUnsafe(xf, edge.Vertex2, v2);
                        DebugDraw.DrawSegment(v1, v2, color);
                    }
                    break;


                case ShapeType.Chain:
                    {
                        ChainShape chain = (ChainShape)fixture.Shape;
                        int count = chain.Count;
                        Vec2[] vertices = chain.Vertices;

                        Transform.MulToOutUnsafe(xf, vertices[0], v1);
                        for (int i = 1; i < count; ++i)
                        {
                            Transform.MulToOutUnsafe(xf, vertices[i], v2);
                            DebugDraw.DrawSegment(v1, v2, color);
                            DebugDraw.DrawCircle(v1, 0.05f, color);
                            v1.Set(v2);
                        }
                    }
                    break;
            }
        }
    }


    class WorldQueryWrapper : ITreeCallback
    {
        public bool TreeCallback(int nodeId)
        {
            FixtureProxy proxy = (FixtureProxy)BroadPhase.GetUserData(nodeId);
            return Callback.ReportFixture(proxy.Fixture);
        }

        internal BroadPhase BroadPhase;
        internal IQueryCallback Callback;
    }



    class WorldRayCastWrapper : ITreeRayCastCallback
    {

        // djm pooling
        private readonly RayCastOutput output = new RayCastOutput();
        private readonly Vec2 temp = new Vec2();
        private readonly Vec2 point = new Vec2();

        public float RaycastCallback(RayCastInput input, int nodeId)
        {
            Object userData = BroadPhase.GetUserData(nodeId);
            FixtureProxy proxy = (FixtureProxy)userData;
            Fixture fixture = proxy.Fixture;
            int index = proxy.ChildIndex;
            bool hit = fixture.Raycast(output, input, index);

            if (hit)
            {
                float fraction = output.Fraction;
                // Vec2 point = (1.0f - fraction) * input.p1 + fraction * input.p2;
                temp.Set(input.P2).MulLocal(fraction);
                point.Set(input.P1).MulLocal(1 - fraction).AddLocal(temp);
                return Callback.ReportFixture(fixture, point, output.Normal, fraction);
            }

            return input.MaxFraction;
        }

        internal BroadPhase BroadPhase;
        internal IRayCastCallback Callback;
    }

}