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

using Box2D.Callbacks;
using Box2D.Collision;
using Box2D.Collision.Shapes;
using Box2D.Common;
using Box2D.Pooling;
using System;

namespace Box2D.Dynamics.Contacts
{

    /// <summary>
    /// The class manages contact between two shapes. A contact exists for each overlapping AABB in the
    /// broad-phase (except if filtered). Therefore a contact object may exist that has no contact
    /// points.
    /// </summary>
    /// <author>daniel</author>
    public abstract class Contact
    {
        // Flags stored in m_flags
        [Flags]
        public enum ContactFlags
        {
            None = 0x0000,

            /// <summary>
            /// Used when crawling contact graph when forming islands.
            /// </summary>
            Island = 0x0001,

            /// <summary>
            /// Set when the shapes are touching.
            /// </summary>
            Touching = 0x0002, // NO_UCD

            /// <summary>
            /// This contact can be disabled (by user)
            /// </summary>
            Enabled = 0x0004,

            /// <summary>
            /// This contact needs filtering because a fixture filter was changed.
            /// </summary>
            Filter = 0x0008,

            /// <summary>
            /// This bullet contact had a TOI event
            /// </summary>
            BulletHit = 0x0010,

            ToiFlag = 0x0020,
        }


        public ContactFlags Flags;

        // World pool and list pointers.
        public Contact Prev;

        // Nodes for connecting bodies.
        public ContactEdge NodeA;
        public ContactEdge NodeB;

        public float ToiCount;
        public float Toi;

        protected readonly internal IWorldPool Pool;

        protected internal Contact(IWorldPool argPool)
        {
            FixtureA = null;
            FixtureB = null;
            NodeA = new ContactEdge();
            NodeB = new ContactEdge();
            Manifold = new Manifold();
            Pool = argPool;
        }

        /// <summary>
        /// initialization for pooling
        /// </summary>
        public virtual void Init(Fixture fA, int indexA, Fixture fB, int indexB)
        {
            Flags = 0;

            FixtureA = fA;
            FixtureB = fB;

            ChildIndexA = indexA;
            ChildIndexB = indexB;

            Manifold.PointCount = 0;

            Prev = null;
            Next = null;

            NodeA.Contact = null;
            NodeA.Prev = null;
            NodeA.Next = null;
            NodeA.Other = null;

            NodeB.Contact = null;
            NodeB.Prev = null;
            NodeB.Next = null;
            NodeB.Other = null;

            ToiCount = 0;
            Friction = MixFriction(fA.Friction, fB.Friction);
            Restitution = MixRestitution(fA.Restitution, fB.Restitution);

            TangentSpeed = 0;
        }

        /// <summary>
        /// Get the contact manifold. Do not set the point count to zero. Instead call Disable.
        /// </summary>
        public Manifold Manifold { get; private set; }

        /// <summary>
        /// Is this contact touching
        /// </summary>
        /// <returns></returns>
        public bool Touching
        {
            get
            {
                return (Flags & ContactFlags.Touching) == ContactFlags.Touching;
            }
        }

        /// <summary>
        /// Gets or sets enabled state of this contact.
        /// This can be used inside the pre-solve contact listener.
        /// The contact is only disabled for the current time step (or sub-step in continuous collisions).
        /// </summary>
        public bool Enabled
        {
            get
            {
                return (Flags & ContactFlags.Enabled) == ContactFlags.Enabled;
            }
            set
            {
                if (value)
                {
                    Flags |= ContactFlags.Enabled;
                }
                else
                {
                    Flags &= ~ContactFlags.Enabled;
                }
            }
        }

        /// <summary>
        /// Get the next contact in the world's contact list.
        /// </summary>
        /// <returns></returns>
        public Contact Next { get; set; }

        /// <summary>
        /// Get the first fixture in this contact.
        /// </summary>
        /// <returns></returns>
        public Fixture FixtureA { get; set; }

        public int ChildIndexA { get; set; }

        /// <summary>
        /// Get the second fixture in this contact.
        /// </summary>
        /// <returns></returns>
        public Fixture FixtureB { get; set; }

        public int ChildIndexB { get; set; }

        public float Friction { get; set; }

        public float Restitution { get; set; }

        public float TangentSpeed { get; set; }

        /// <summary>
        /// Get the world manifold.
        /// </summary>
        public void GetWorldManifold(WorldManifold worldManifold)
        {
            Body bodyA = FixtureA.Body;
            Body bodyB = FixtureB.Body;
            Shape shapeA = FixtureA.Shape;
            Shape shapeB = FixtureB.Shape;

            worldManifold.Initialize(Manifold, bodyA.GetTransform(), shapeA.Radius, bodyB.GetTransform(), shapeB.Radius);
        }

        public void ResetFriction()
        {
            Friction = MixFriction(FixtureA.Friction, FixtureB.Friction);
        }

        public void ResetRestitution()
        {
            Restitution = MixRestitution(FixtureA.Restitution, FixtureB.Restitution);
        }

        public abstract void Evaluate(Manifold manifold, Transform xfA, Transform xfB);

        /// <summary>
        /// Flag this contact for filtering. Filtering will occur the next time step.
        /// </summary>
        public void SetFlagForFiltering()
        {
            Flags |= ContactFlags.Filter;
        }

        // djm pooling
        private readonly Manifold oldManifold = new Manifold();

        public void Update(IContactListener listener)
        {

            oldManifold.Set(Manifold);

            // Re-enable this contact.
            Flags |= ContactFlags.Enabled;

            bool touching;
            bool wasTouching = (Flags & ContactFlags.Touching) == ContactFlags.Touching;

            bool sensorA = FixtureA.Sensor;
            bool sensorB = FixtureB.Sensor;
            bool sensor = sensorA || sensorB;

            Body bodyA = FixtureA.Body;
            Body bodyB = FixtureB.Body;
            Transform xfA = bodyA.GetTransform();
            Transform xfB = bodyB.GetTransform();
            // log.debug("TransformA: "+xfA);
            // log.debug("TransformB: "+xfB);

            if (sensor)
            {
                Shape shapeA = FixtureA.Shape;
                Shape shapeB = FixtureB.Shape;
                touching = Pool.GetCollision().TestOverlap(shapeA, ChildIndexA, shapeB, ChildIndexB, xfA, xfB);

                // Sensors don't generate manifolds.
                Manifold.PointCount = 0;
            }
            else
            {
                Evaluate(Manifold, xfA, xfB);
                touching = Manifold.PointCount > 0;

                // Match old contact ids to new contact ids and copy the
                // stored impulses to warm start the solver.
                for (int i = 0; i < Manifold.PointCount; ++i)
                {
                    ManifoldPoint mp2 = Manifold.Points[i];
                    mp2.NormalImpulse = 0.0f;
                    mp2.TangentImpulse = 0.0f;
                    ContactID id2 = mp2.Id;

                    for (int j = 0; j < oldManifold.PointCount; ++j)
                    {
                        ManifoldPoint mp1 = oldManifold.Points[j];

                        if (mp1.Id.IsEqual(id2))
                        {
                            mp2.NormalImpulse = mp1.NormalImpulse;
                            mp2.TangentImpulse = mp1.TangentImpulse;
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
                Flags |= ContactFlags.Touching;
            }
            else
            {
                Flags &= ~ContactFlags.Touching;
            }

            if (listener == null)
            {
                return;
            }

            if (!wasTouching && touching)
            {
                listener.BeginContact(this);
            }

            if (wasTouching && !touching)
            {
                listener.EndContact(this);
            }

            if (!sensor && touching)
            {
                listener.PreSolve(this, oldManifold);
            }
        }

        /// <summary>
        /// Friction mixing law. The idea is to allow either fixture to drive the restitution to zero. For
        /// example, anything slides on ice.
        /// </summary>
        /// <param name="friction1"></param>
        /// <param name="friction2"></param>
        /// <returns></returns>
        public static float MixFriction(float friction1, float friction2)
        {
            return MathUtils.Sqrt(friction1 * friction2);
        }

        /// <summary>
        /// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
        /// For example, a superball bounces on anything.
        /// </summary>
        /// <param name="restitution1"></param>
        /// <param name="restitution2"></param>
        /// <returns></returns>
        public static float MixRestitution(float restitution1, float restitution2)
        {
            return restitution1 > restitution2 ? restitution1 : restitution2;
        }
    }
}