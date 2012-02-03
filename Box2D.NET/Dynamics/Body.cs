// ****************************************************************************
// Copyright (c) 2011, Daniel Murphy All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without modification, are permitted
// provided that the following conditions are met: * Redistributions of source code must retain the
// above copyright notice, this list of conditions and the following disclaimer. * Redistributions
// in binary form must reproduce the above copyright notice, this list of conditions and the
// following disclaimer in the documentation and/or other materials provided with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
// WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ****************************************************************************

using System;
using System.Diagnostics;
using Box2D.Collision.Broadphase;
using Box2D.Collision.Shapes;
using Box2D.Common;
using Box2D.Dynamics.Contacts;
using Box2D.Dynamics.Joints;

namespace Box2D.Dynamics
{

    /// <summary>
    /// A rigid body. These are created via World.createBody.
    /// </summary>
    /// <author>Daniel Murphy</author>
    public sealed class Body
    {
        [Flags]
        public enum TypeFlags
        {
            None = 0x0000,
            Island = 0x0001,
            Awake = 0x0002,
            AutoSleep = 0x0004,
            Bullet = 0x0008,
            FixedRotation = 0x0010,
            Active = 0x0020,
            Toi = 0x0040,
        }

        public BodyType m_type;

        public TypeFlags Flags;

        public int IslandIndex;

        /// <summary>
        /// The body origin transform.
        /// </summary>
        public readonly Transform Xf = new Transform();

        /// <summary>
        /// The swept motion for CCD
        /// </summary>
        public readonly Sweep Sweep = new Sweep();

        public readonly Vec2 m_linearVelocity = new Vec2();
        public float m_angularVelocity;

        public readonly Vec2 Force = new Vec2();
        public float Torque = 0;

        public Body Prev;

        public int FixtureCount;

        public float InvMass;

        // Rotational inertia about the center of mass.
        public float I;
        public float InvI;

        public float SleepTime;

        public Body(BodyDef bd, World world)
        {
            Debug.Assert(bd.Position.Valid);
            Debug.Assert(bd.LinearVelocity.Valid);
            Debug.Assert(bd.GravityScale >= 0.0f);
            Debug.Assert(bd.AngularDamping >= 0.0f);
            Debug.Assert(bd.LinearDamping >= 0.0f);

            Flags = TypeFlags.None;

            if (bd.Bullet)
            {
                Flags |= TypeFlags.Bullet;
            }
            if (bd.FixedRotation)
            {
                Flags |= TypeFlags.FixedRotation;
            }
            if (bd.AllowSleep)
            {
                Flags |= TypeFlags.AutoSleep;
            }
            if (bd.Awake)
            {
                Flags |= TypeFlags.Awake;
            }
            if (bd.Active)
            {
                Flags |= TypeFlags.Active;
            }

            World = world;

            Xf.p.Set(bd.Position);
            Xf.q.set_Renamed(bd.Angle);

            Sweep.localCenter.SetZero();
            Sweep.c0.Set(Xf.p);
            Sweep.c.Set(Xf.p);
            Sweep.a0 = bd.Angle;
            Sweep.a = bd.Angle;
            Sweep.alpha0 = 0.0f;

            JointList = null;
            ContactList = null;
            Prev = null;
            Next = null;

            m_linearVelocity.Set(bd.LinearVelocity);
            m_angularVelocity = bd.AngularVelocity;

            LinearDamping = bd.LinearDamping;
            AngularDamping = bd.AngularDamping;
            GravityScale = bd.GravityScale;

            Force.SetZero();
            Torque = 0.0f;

            SleepTime = 0.0f;

            m_type = bd.Type;

            if (m_type == BodyType.Dynamic)
            {
                Mass = 1f;
                InvMass = 1f;
            }
            else
            {
                Mass = 0f;
                InvMass = 0f;
            }

            I = 0.0f;
            InvI = 0.0f;

            UserData = bd.UserData;

            FixtureList = null;
            FixtureCount = 0;
        }

        // TODO djm: check out about this new fixture here
        /// <summary>
        /// Creates a fixture and attach it to this body. Use this function if you need to set some fixture
        /// parameters, like friction. Otherwise you can create the fixture directly from a shape. If the
        /// density is non-zero, this function automatically updates the mass of the body. Contacts are not
        /// created until the next time step.
        /// </summary>
        /// <param name="def">the fixture definition.</param>
        /// <warning>This function is locked during callbacks.</warning>
        public Fixture CreateFixture(FixtureDef def)
        {
            Debug.Assert(World.Locked == false);

            if (World.Locked == true)
            {
                return null;
            }

            // djm TODO from pool?
            Fixture fixture = new Fixture();
            fixture.Create(this, def);

            if ((Flags & TypeFlags.Active) == TypeFlags.Active)
            {
                BroadPhase broadPhase = World.ContactManager.BroadPhase;
                fixture.CreateProxies(broadPhase, Xf);
            }

            fixture.Next = FixtureList;
            FixtureList = fixture;
            ++FixtureCount;

            fixture.Body = this;

            // Adjust mass properties if needed.
            if (fixture.m_density > 0.0f)
            {
                ResetMassData();
            }

            // Let the world know we have a new fixture. This will cause new contacts
            // to be created at the beginning of the next time step.
            World.Flags |= World.NEW_FIXTURE;

            return fixture;
        }

        private readonly FixtureDef fixDef = new FixtureDef();

        /// <summary>
        /// Creates a fixture from a shape and attach it to this body. This is a convenience function. Use
        /// FixtureDef if you need to set parameters like friction, restitution, user data, or filtering.
        /// If the density is non-zero, this function automatically updates the mass of the body.
        /// </summary>
        /// <param name="shape">the shape to be cloned.</param>
        /// <param name="density">the shape density (set to zero for static bodies).</param>
        /// <warning>This function is locked during callbacks.</warning>
        public Fixture CreateFixture(Shape shape, float density)
        {
            fixDef.Shape = shape;
            fixDef.Density = density;

            return CreateFixture(fixDef);
        }

        /// <summary>
        /// Destroy a fixture. This removes the fixture from the broad-phase and destroys all contacts
        /// associated with this fixture. This will automatically adjust the mass of the body if the body
        /// is dynamic and the fixture has positive density. All fixtures attached to a body are implicitly
        /// destroyed when the body is destroyed.
        /// </summary>
        /// <param name="fixture">the fixture to be removed.</param>
        /// <warning>This function is locked during callbacks.</warning>
        public void DestroyFixture(Fixture fixture)
        {
            Debug.Assert(World.Locked == false);
            if (World.Locked == true)
            {
                return;
            }

            Debug.Assert(fixture.Body == this);

            // Remove the fixture from this body's singly linked list.
            Debug.Assert(FixtureCount > 0);
            Fixture node = FixtureList;
            Fixture last = null; // java change
            bool found = false;
            while (node != null)
            {
                if (node == fixture)
                {
                    node = fixture.Next;
                    found = true;
                    break;
                }
                last = node;
                node = node.Next;
            }

            // You tried to remove a shape that is not attached to this body.
            Debug.Assert(found);

            // java change, remove it from the list
            if (last == null)
            {
                FixtureList = fixture.Next;
            }
            else
            {
                last.Next = fixture.Next;
            }

            // Destroy any contacts associated with the fixture.
            ContactEdge edge = ContactList;
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
                    World.ContactManager.Destroy(c);
                }
            }

            if ((Flags & TypeFlags.Active) == TypeFlags.Active)
            {
                BroadPhase broadPhase = World.ContactManager.BroadPhase;
                fixture.DestroyProxies(broadPhase);
            }

            fixture.Destroy();
            fixture.Body = null;
            fixture.Next = null;

            --FixtureCount;

            // Reset the mass data.
            ResetMassData();
        }

        /// <summary>
        /// Set the position of the body's origin and rotation. This breaks any contacts and wakes the
        /// other bodies. Manipulating a body's transform may cause non-physical behavior.
        /// </summary>
        /// <param name="position">the world position of the body's local origin.</param>
        /// <param name="angle">the world rotation in radians.</param>
        public void SetTransform(Vec2 position, float angle)
        {
            Debug.Assert(World.Locked == false);
            if (World.Locked == true)
            {
                return;
            }

            Xf.q.set_Renamed(angle);
            Xf.p.Set(position);

            // m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
            Transform.mulToOutUnsafe(Xf, Sweep.localCenter, Sweep.c);
            Sweep.a = angle;

            Sweep.c0.Set(Sweep.c);
            Sweep.a0 = Sweep.a;

            BroadPhase broadPhase = World.ContactManager.BroadPhase;
            for (Fixture f = FixtureList; f != null; f = f.Next)
            {
                f.Synchronize(broadPhase, Xf, Xf);
            }

            World.ContactManager.FindNewContacts();
        }

        /// <summary>
        /// Get the body transform for the body's origin.
        /// </summary>
        /// <returns>the world transform of the body's origin.</returns>
        public Transform GetTransform()
        {
            return Xf;
        }

        /// <summary>
        /// Get the world body origin position. Do not modify.
        /// </summary>
        /// <returns>the world position of the body's origin.</returns>
        public Vec2 Position
        {
            get
            {
                return Xf.p;
            }
        }

        /// <summary>
        /// Get the angle in radians.
        /// </summary>
        /// <returns>the current world rotation angle in radians.</returns>
        public float Angle
        {
            get
            {
                return Sweep.a;
            }
        }

        /// <summary>
        /// Get the world position of the center of mass. Do not modify.
        /// </summary>
        public Vec2 WorldCenter
        {
            get
            {
                return Sweep.c;
            }
        }

        /// <summary>
        /// Get the local position of the center of mass. Do not modify.
        /// </summary>
        public Vec2 LocalCenter
        {
            get
            {
                return Sweep.localCenter;
            }
        }

        /// <summary>
        /// Get or sets the linear velocity of the center of mass.
        /// </summary>
        public Vec2 LinearVelocity
        {
            get
            {
                return m_linearVelocity;
            }
            set
            {
                if (m_type == BodyType.Static)
                {
                    return;
                }

                if (Vec2.Dot(value, value) > 0.0f)
                {
                    Awake = true;
                }

                m_linearVelocity.Set(value);
            }
        }

        /// <summary>
        /// Get or sets the angular velocity in radians/second.
        /// </summary>
        public float AngularVelocity
        {
            get
            {
                return m_angularVelocity;
            }
            set
            {
                if (m_type == BodyType.Static)
                {
                    return;
                }

                if (value * value > 0f)
                {
                    Awake = true;
                }

                m_angularVelocity = value;
            }
        }

        /// <summary>
        /// Gets or sets the gravity scale of the body.
        /// </summary>
        public float GravityScale { get; set; }

        /// <summary>
        /// Get the total mass of the body.
        /// </summary>
        /// <returns>the mass, usually in kilograms (kg).</returns>
        public float Mass { get; set; }

        /// <summary>
        /// Get the central rotational inertia of the body.
        /// </summary>
        /// <returns>the rotational inertia, usually in kg-m^2.</returns>
        public float Inertia
        {
            get
            {
                return I + Mass * (Sweep.localCenter.X * Sweep.localCenter.X + Sweep.localCenter.Y * Sweep.localCenter.Y);
            }
        }

        /// <summary>
        /// Gets or sets the linear damping of the body.
        /// </summary>
        public float LinearDamping { get; set; }

        /// <summary>
        /// Gets or sets the angular damping of the body.
        /// </summary>
        public float AngularDamping { get; set; }

        /// <summary>
        /// Gets or sets the type of this body.
        /// Setter may alter the mass and velocity.
        /// </summary>
        /// <param name="type"></param>
        public BodyType Type
        {
            get
            {
                return m_type;
            }
            set
            {
                Debug.Assert(World.Locked == false);
                if (World.Locked == true)
                {
                    return;
                }

                if (m_type == value)
                {
                    return;
                }

                m_type = value;

                ResetMassData();

                if (m_type == BodyType.Static)
                {
                    m_linearVelocity.SetZero();
                    m_angularVelocity = 0.0f;
                    Sweep.a0 = Sweep.a;
                    Sweep.c0.Set(Sweep.c);
                    SynchronizeFixtures();
                }

                Awake = true;

                Force.SetZero();
                Torque = 0.0f;

                // Delete the attached contacts.
                ContactEdge ce = ContactList;
                while (ce != null)
                {
                    ContactEdge ce0 = ce;
                    ce = ce.next;
                    World.ContactManager.Destroy(ce0.contact);
                }
                ContactList = null;

                // Touch the proxies so that new contacts will be created (when appropriate)
                BroadPhase broadPhase = World.ContactManager.BroadPhase;
                for (Fixture f = FixtureList; f != null; f = f.Next)
                {
                    int proxyCount = f.ProxyCount;
                    for (int i = 0; i < proxyCount; ++i)
                    {
                        broadPhase.TouchProxy(f.Proxies[i].ProxyId);
                    }
                }
            }
        }

        /// <summary>
        /// Is this body treated like a bullet for continuous collision detection?
        /// </summary>
        public bool Bullet
        {
            get
            {
                return (Flags & TypeFlags.Bullet) == TypeFlags.Bullet;
            }
            set
            {
                if (value)
                {
                    Flags |= TypeFlags.Bullet;
                }
                else
                {
                    Flags &= ~TypeFlags.Bullet;
                }
            }
        }

        /// <summary>
        /// Is this body allowed to sleep.
        /// You can disable sleeping on this body. If you disable sleeping, the body will be woken.
        /// </summary>
        public bool SleepingAllowed
        {
            get
            {
                return (Flags & TypeFlags.AutoSleep) == TypeFlags.AutoSleep;
            }
            set
            {
                if (value)
                {
                    Flags |= TypeFlags.AutoSleep;
                }
                else
                {
                    Flags &= ~TypeFlags.AutoSleep;
                    Awake = true;
                }
            }
        }

        /// <summary>
        /// Gets or sets the sleeping state of this body.
        /// A sleeping body has very low CPU cost.
        /// </summary>
        public bool Awake
        {
            get
            {
                return (Flags & TypeFlags.Awake) == TypeFlags.Awake;
            }
            set
            {
                if (value)
                {
                    if ((Flags & TypeFlags.Awake) == 0)
                    {
                        Flags |= TypeFlags.Awake;
                        SleepTime = 0.0f;
                    }
                }
                else
                {
                    Flags &= ~TypeFlags.Awake;
                    SleepTime = 0.0f;
                    m_linearVelocity.SetZero();
                    m_angularVelocity = 0.0f;
                    Force.SetZero();
                    Torque = 0.0f;
                }
            }
        }

        /// <summary>
        /// Gets or sets the active state of the body.
        /// 
        /// An inactive body is not simulated and cannot be collided with
        /// or woken up. If you pass a flag of true, all fixtures will be added to the broad-phase. If you
        /// pass a flag of false, all fixtures will be removed from the broad-phase and all contacts will
        /// be destroyed. Fixtures and joints are otherwise unaffected. You may continue to create/destroy
        /// fixtures and joints on inactive bodies. Fixtures on an inactive body are implicitly inactive
        /// and will not participate in collisions, ray-casts, or queries. Joints connected to an inactive
        /// body are implicitly inactive. An inactive body is still owned by a World object and remains in
        /// the body list.
        /// </summary>
        public bool Active
        {
            get
            {
                return (Flags & TypeFlags.Active) == TypeFlags.Active;
            }
            set
            {
                Debug.Assert(World.Locked == false);

                if (value == Active)
                {
                    return;
                }

                if (value)
                {
                    Flags |= TypeFlags.Active;

                    // Create all proxies.
                    BroadPhase broadPhase = World.ContactManager.BroadPhase;
                    for (Fixture f = FixtureList; f != null; f = f.Next)
                    {
                        f.CreateProxies(broadPhase, Xf);
                    }

                    // Contacts are created the next time step.
                }
                else
                {
                    Flags &= ~TypeFlags.Active;

                    // Destroy all proxies.
                    BroadPhase broadPhase = World.ContactManager.BroadPhase;
                    for (Fixture f = FixtureList; f != null; f = f.Next)
                    {
                        f.DestroyProxies(broadPhase);
                    }

                    // Destroy the attached contacts.
                    ContactEdge ce = ContactList;
                    while (ce != null)
                    {
                        ContactEdge ce0 = ce;
                        ce = ce.next;
                        World.ContactManager.Destroy(ce0.contact);
                    }
                    ContactList = null;
                }
            }
        }

        /// <summary>
        /// Does this body have fixed rotation?
        /// Setter causes the mass to be reset.
        /// </summary>
        public bool FixedRotation
        {
            get
            {
                return (Flags & TypeFlags.FixedRotation) == TypeFlags.FixedRotation;
            }
            set
            {
                if (value)
                {
                    Flags |= TypeFlags.FixedRotation;
                }
                else
                {
                    Flags &= ~TypeFlags.FixedRotation;
                }

                ResetMassData();
            }
        }

        /// <summary>
        /// Get the list of all fixtures attached to this body.
        /// </summary>
        public Fixture FixtureList { get; set; }

        /// <summary>
        /// Get the list of all joints attached to this body.
        /// </summary>
        public JointEdge JointList { get; set; }

        /// <summary>
        /// Get the list of all contacts attached to this body.
        /// </summary>
        /// <warning>this list changes during the time step and you may miss some collisions if you don't use ContactListener.</warning>
        public ContactEdge ContactList { get; set; }

        /// <summary>
        /// Get the next body in the world's body list.
        /// </summary>
        public Body Next { get; set; }

        /// <summary>Gets or sets the user data pointer that was provided in the body definition.</summary>
        public object UserData { get; set; }

        /// <summary>Gets the parent world of this body.</summary>
        public World World { get; set; }

        /// <summary>
        /// Apply a force at a world point. If the force is not applied at the center of mass, it will
        /// generate a torque and affect the angular velocity. This wakes up the body.
        /// </summary>
        /// <param name="force">the world force vector, usually in Newtons (N).</param>
        /// <param name="point">the world position of the point of application.</param>
        public void ApplyForce(Vec2 force, Vec2 point)
        {
            if (m_type != BodyType.Dynamic)
            {
                return;
            }

            if (Awake == false)
            {
                Awake = true;
            }

            // m_force.addLocal(force);
            // Vec2 temp = tltemp.get();
            // temp.set(point).subLocal(m_sweep.c);
            // m_torque += Vec2.cross(temp, force);

            Force.X += force.X;
            Force.Y += force.Y;

            Torque += (point.X - Sweep.c.X) * force.Y - (point.Y - Sweep.c.Y) * force.X;
        }

        /// <summary>
        /// Apply a force to the center of mass. This wakes up the body.
        /// </summary>
        /// <param name="force">the world force vector, usually in Newtons (N).</param>
        public void ApplyForceToCenter(Vec2 force)
        {
            if (m_type != BodyType.Dynamic)
            {
                return;
            }

            if (Awake == false)
            {
                Awake = true;
            }

            Force.X += force.X;
            Force.Y += force.Y;
        }

        /// <summary>
        /// Apply a torque. This affects the angular velocity without affecting the linear velocity of the
        /// center of mass. This wakes up the body.
        /// </summary>
        /// <param name="torque">about the z-axis (out of the screen), usually in N-m.</param>
        public void ApplyTorque(float torque)
        {
            if (m_type != BodyType.Dynamic)
            {
                return;
            }

            if (Awake == false)
            {
                Awake = true;
            }

            Torque += torque;
        }

        /// <summary>
        /// Apply an impulse at a point. This immediately modifies the velocity. It also modifies the
        /// angular velocity if the point of application is not at the center of mass. This wakes up the
        /// body.
        /// </summary>
        /// <param name="impulse">the world impulse vector, usually in N-seconds or kg-m/s.</param>
        /// <param name="point">the world position of the point of application.</param>
        public void ApplyLinearImpulse(Vec2 impulse, Vec2 point)
        {
            if (m_type != BodyType.Dynamic)
            {
                return;
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

            m_linearVelocity.X += impulse.X * InvMass;
            m_linearVelocity.Y += impulse.Y * InvMass;

            m_angularVelocity += InvI * ((point.X - Sweep.c.X) * impulse.Y - (point.Y - Sweep.c.Y) * impulse.X);
        }

        /// <summary>
        /// Apply an angular impulse.
        /// </summary>
        /// <param name="impulse">the angular impulse in units of kg*m*m/s</param>
        public void ApplyAngularImpulse(float impulse)
        {
            if (m_type != BodyType.Dynamic)
            {
                return;
            }

            if (Awake == false)
            {
                Awake = true;
            }
            m_angularVelocity += InvI * impulse;
        }

        /// <summary>
        /// Get the mass data of the body. The rotational inertia is relative to the center of mass.
        /// </summary>
        /// <returns>a struct containing the mass, inertia and center of the body.</returns>
        public void GetMassData(MassData data)
        {
            // data.mass = m_mass;
            // data.I = m_I + m_mass * Vec2.dot(m_sweep.localCenter, m_sweep.localCenter);
            // data.center.set(m_sweep.localCenter);

            data.Mass = Mass;
            data.I = I + Mass * (Sweep.localCenter.X * Sweep.localCenter.X + Sweep.localCenter.Y * Sweep.localCenter.Y);
            data.Center.X = Sweep.localCenter.X;
            data.Center.Y = Sweep.localCenter.Y;
        }

        /// <summary>
        /// Set the mass properties to override the mass properties of the fixtures. Note that this changes
        /// the center of mass position. Note that creating or destroying fixtures can also alter the mass.
        /// This function has no effect if the body isn't dynamic.
        /// </summary>
        /// <param name="massData">the mass properties.</param>
        public void SetMassData(MassData massData)
        {
            // TODO_ERIN adjust linear velocity and torque to account for movement of center.
            Debug.Assert(World.Locked == false);
            if (World.Locked == true)
            {
                return;
            }

            if (m_type != BodyType.Dynamic)
            {
                return;
            }

            InvMass = 0.0f;
            I = 0.0f;
            InvI = 0.0f;

            Mass = massData.Mass;
            if (Mass <= 0.0f)
            {
                Mass = 1f;
            }

            InvMass = 1.0f / Mass;

            if (massData.I > 0.0f && (Flags & TypeFlags.FixedRotation) == 0)
            {
                I = massData.I - Mass * Vec2.Dot(massData.Center, massData.Center);
                Debug.Assert(I > 0.0f);
                InvI = 1.0f / I;
            }

            Vec2 oldCenter = World.Pool.PopVec2();
            // Move center of mass.
            oldCenter.Set(Sweep.c);
            Sweep.localCenter.Set(massData.Center);
            // m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
            Transform.mulToOutUnsafe(Xf, Sweep.localCenter, Sweep.c0);
            Sweep.c.Set(Sweep.c0);

            // Update center of mass velocity.
            // m_linearVelocity += Cross(m_angularVelocity, m_sweep.c - oldCenter);
            Vec2 temp = World.Pool.PopVec2();
            temp.Set(Sweep.c).SubLocal(oldCenter);
            Vec2.CrossToOut(m_angularVelocity, temp, temp);
            m_linearVelocity.AddLocal(temp);

            World.Pool.PushVec2(2);
        }

        private readonly MassData pmd = new MassData();

        /// <summary>
        /// This resets the mass properties to the sum of the mass properties of the fixtures. This
        /// normally does not need to be called unless you called setMassData to override the mass and you
        /// later want to reset the mass.
        /// </summary>
        public void ResetMassData()
        {
            // Compute mass data from shapes. Each shape has its own density.
            Mass = 0.0f;
            InvMass = 0.0f;
            I = 0.0f;
            InvI = 0.0f;
            Sweep.localCenter.SetZero();

            // Static and kinematic bodies have zero mass.
            if (m_type == BodyType.Static || m_type == BodyType.Kinematic)
            {
                // m_sweep.c0 = m_sweep.c = m_xf.position;
                Sweep.c0.Set(Xf.p);
                Sweep.c.Set(Xf.p);
                Sweep.a0 = Sweep.a;
                return;
            }

            Debug.Assert(m_type == BodyType.Dynamic);

            // Accumulate mass over all fixtures.
            Vec2 localCenter = World.Pool.PopVec2();
            localCenter.SetZero();
            Vec2 temp = World.Pool.PopVec2();
            MassData massData = pmd;
            for (Fixture f = FixtureList; f != null; f = f.Next)
            {
                if (f.m_density == 0.0f)
                {
                    continue;
                }
                f.GetMassData(massData);
                Mass += massData.Mass;
                // center += massData.mass * massData.center;
                temp.Set(massData.Center).MulLocal(massData.Mass);
                localCenter.AddLocal(temp);
                I += massData.I;
            }

            // Compute center of mass.
            if (Mass > 0.0f)
            {
                InvMass = 1.0f / Mass;
                localCenter.MulLocal(InvMass);
            }
            else
            {
                // Force all dynamic bodies to have a positive mass.
                Mass = 1.0f;
                InvMass = 1.0f;
            }

            if (I > 0.0f && (Flags & TypeFlags.FixedRotation) == 0)
            {
                // Center the inertia about the center of mass.
                I -= Mass * Vec2.Dot(localCenter, localCenter);
                Debug.Assert(I > 0.0f);
                InvI = 1.0f / I;
            }
            else
            {
                I = 0.0f;
                InvI = 0.0f;
            }

            Vec2 oldCenter = World.Pool.PopVec2();
            // Move center of mass.
            oldCenter.Set(Sweep.c);
            Sweep.localCenter.Set(localCenter);
            // m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
            Transform.mulToOutUnsafe(Xf, Sweep.localCenter, Sweep.c0);
            Sweep.c.Set(Sweep.c0);

            // Update center of mass velocity.
            // m_linearVelocity += Cross(m_angularVelocity, m_sweep.c - oldCenter);
            temp.Set(Sweep.c).SubLocal(oldCenter);

            Vec2 temp2 = oldCenter;
            Vec2.CrossToOutUnsafe(m_angularVelocity, temp, temp2);
            m_linearVelocity.AddLocal(temp2);

            World.Pool.PushVec2(3);
        }

        /// <summary>
        /// Get the world coordinates of a point given the local coordinates.
        /// </summary>
        /// <param name="localPoint">a point on the body measured relative the the body's origin.</param>
        /// <returns> the same point expressed in world coordinates.</returns>
        public Vec2 GetWorldPoint(Vec2 localPoint)
        {
            Vec2 v = new Vec2();
            GetWorldPointToOut(localPoint, v);
            return v;
        }

        public void GetWorldPointToOut(Vec2 localPoint, Vec2 result)
        {
            Transform.mulToOut(Xf, localPoint, result);
        }

        /// <summary>
        /// Get the world coordinates of a vector given the local coordinates.
        /// </summary>
        /// <param name="localVector">a vector fixed in the body.</param>
        /// <returns>the same vector expressed in world coordinates.</returns>
        public Vec2 GetWorldVector(Vec2 localVector)
        {
            Vec2 result = new Vec2();
            GetWorldVectorToOut(localVector, result);
            return result;
        }

        public void GetWorldVectorToOut(Vec2 localVector, Vec2 result)
        {
            Rot.mulToOut(Xf.q, localVector, result);
        }

        public void GetWorldVectorToOutUnsafe(Vec2 localVector, Vec2 result)
        {
            Rot.mulToOutUnsafe(Xf.q, localVector, result);
        }

        /// <summary>
        /// Gets a local point relative to the body's origin given a world point.
        /// </summary>
        /// <param name="worldPoint">point in world coordinates.</param>
        /// <returns>the corresponding local point relative to the body's origin.</returns>
        public Vec2 GetLocalPoint(Vec2 worldPoint)
        {
            Vec2 result = new Vec2();
            GetLocalPointToOut(worldPoint, result);
            return result;
        }

        public void GetLocalPointToOut(Vec2 worldPoint, Vec2 result)
        {
            Transform.mulTransToOut(Xf, worldPoint, result);
        }

        /// <summary>
        /// Gets a local vector given a world vector.
        /// </summary>
        /// <param name="worldVector">vector in world coordinates.</param>
        /// <returns>the corresponding local vector.</returns>
        public Vec2 GetLocalVector(Vec2 worldVector)
        {
            Vec2 result = new Vec2();
            GetLocalVectorToOut(worldVector, result);
            return result;
        }

        public void GetLocalVectorToOut(Vec2 worldVector, Vec2 result)
        {
            Rot.mulTrans(Xf.q, worldVector, result);
        }

        public void GetLocalVectorToOutUnsafe(Vec2 worldVector, Vec2 result)
        {
            Rot.mulTransUnsafe(Xf.q, worldVector, result);
        }

        /// <summary>
        /// Get the world linear velocity of a world point attached to this body.
        /// </summary>
        /// <param name="worldPoint">point in world coordinates.</param>
        /// <returns>the world velocity of a point.</returns>
        public Vec2 GetLinearVelocityFromWorldPoint(Vec2 worldPoint)
        {
            Vec2 result = new Vec2();
            GetLinearVelocityFromWorldPointToOut(worldPoint, result);
            return result;
        }

        public void GetLinearVelocityFromWorldPointToOut(Vec2 worldPoint, Vec2 result)
        {
            result.Set(worldPoint).SubLocal(Sweep.c);
            Vec2.CrossToOut(m_angularVelocity, result, result);
            result.AddLocal(m_linearVelocity);
        }

        /// <summary>
        /// Get the world velocity of a local point.
        /// </summary>
        /// <param name="localPoint">point in local coordinates.</param>
        /// <returns>the world velocity of a point.</returns>
        public Vec2 GetLinearVelocityFromLocalPoint(Vec2 localPoint)
        {
            Vec2 result = new Vec2();
            GetLinearVelocityFromLocalPointToOut(localPoint, result);
            return result;
        }

        public void GetLinearVelocityFromLocalPointToOut(Vec2 localPoint, Vec2 result)
        {
            GetWorldPointToOut(localPoint, result);
            GetLinearVelocityFromWorldPointToOut(result, result);
        }

        // djm pooling
        private readonly Transform pxf = new Transform();

        internal void SynchronizeFixtures()
        {
            Transform xf1 = pxf;
            // xf1.position = m_sweep.c0 - Mul(xf1.R, m_sweep.localCenter);

            // xf1.q.set(m_sweep.a0);
            // Rot.mulToOutUnsafe(xf1.q, m_sweep.localCenter, xf1.p);
            // xf1.p.mulLocal(-1).addLocal(m_sweep.c0);
            // inlined:
            xf1.q.s = MathUtils.Sin(Sweep.a0);
            xf1.q.c = MathUtils.Cos(Sweep.a0);
            xf1.p.X = Sweep.c0.X - xf1.q.c * Sweep.localCenter.X + xf1.q.s * Sweep.localCenter.Y;
            xf1.p.Y = Sweep.c0.Y - xf1.q.s * Sweep.localCenter.X - xf1.q.c * Sweep.localCenter.Y;
            // end inline

            for (Fixture f = FixtureList; f != null; f = f.Next)
            {
                f.Synchronize(World.ContactManager.BroadPhase, xf1, Xf);
            }
        }

        public void SynchronizeTransform()
        {
            // m_xf.q.set(m_sweep.a);
            //
            // // m_xf.position = m_sweep.c - Mul(m_xf.R, m_sweep.localCenter);
            // Rot.mulToOutUnsafe(m_xf.q, m_sweep.localCenter, m_xf.p);
            // m_xf.p.mulLocal(-1).addLocal(m_sweep.c);
            //
            Xf.q.s = MathUtils.Sin(Sweep.a);
            Xf.q.c = MathUtils.Cos(Sweep.a);
            Rot q = Xf.q;
            Vec2 v = Sweep.localCenter;
            Xf.p.X = Sweep.c.X - q.c * v.X + q.s * v.Y;
            Xf.p.Y = Sweep.c.Y - q.s * v.X - q.c * v.Y;
        }

        /// <summary>
        /// This is used to prevent connected bodies from colliding. It may lie, depending on the
        /// collideConnected flag.
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public bool ShouldCollide(Body other)
        {
            // At least one body should be dynamic.
            if (m_type != BodyType.Dynamic && other.m_type != BodyType.Dynamic)
            {
                return false;
            }

            // Does a joint prevent collision?
            for (JointEdge jn = JointList; jn != null; jn = jn.next)
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

        internal void Advance(float t)
        {
            // Advance to the new safe time. This doesn't sync the broad-phase.
            Sweep.advance(t);
            Sweep.c.Set(Sweep.c0);
            Sweep.a = Sweep.a0;
            Xf.q.set_Renamed(Sweep.a);
            // m_xf.position = m_sweep.c - Mul(m_xf.R, m_sweep.localCenter);
            Rot.mulToOutUnsafe(Xf.q, Sweep.localCenter, Xf.p);
            Xf.p.MulLocal(-1).AddLocal(Sweep.c);
        }
    }
}