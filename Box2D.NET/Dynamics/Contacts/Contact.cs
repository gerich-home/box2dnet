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

        public readonly Manifold m_manifold;

        public float m_toiCount;
        public float m_toi;

        public float m_friction;
        public float m_restitution;

        public float m_tangentSpeed;

        protected readonly internal IWorldPool pool;

        protected internal Contact(IWorldPool argPool)
        {
            m_fixtureA = null;
            m_fixtureB = null;
            m_nodeA = new ContactEdge();
            m_nodeB = new ContactEdge();
            m_manifold = new Manifold();
            pool = argPool;
        }

        /// <summary>
        /// initialization for pooling
        /// </summary>
        public virtual void init(Fixture fA, int indexA, Fixture fB, int indexB)
        {
            m_flags = 0;

            m_fixtureA = fA;
            m_fixtureB = fB;

            m_indexA = indexA;
            m_indexB = indexB;

            m_manifold.PointCount = 0;

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
            m_friction = mixFriction(fA.Friction, fB.Friction);
            m_restitution = mixRestitution(fA.Restitution, fB.Restitution);

            m_tangentSpeed = 0;
        }

        /// <summary>
        /// Get the contact manifold. Do not set the point count to zero. Instead call Disable.
        /// </summary>
        public Manifold Manifold
        {
            get
            {
                return m_manifold;
            }
        }

        /// <summary>
        /// Is this contact touching
        /// </summary>
        /// <returns></returns>
        public bool Touching
        {
            get
            {
                return (m_flags & TOUCHING_FLAG) == TOUCHING_FLAG;
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
                    m_flags &= ~ENABLED_FLAG;
                }
            }
        }

        /// <summary>
        /// Get the next contact in the world's contact list.
        /// </summary>
        /// <returns></returns>
        public Contact Next
        {
            get
            {
                return m_next;
            }
        }

        /// <summary>
        /// Get the first fixture in this contact.
        /// </summary>
        /// <returns></returns>
        public Fixture FixtureA
        {
            get
            {
                return m_fixtureA;
            }
        }

        public int ChildIndexA
        {
            get
            {
                return m_indexA;
            }
        }

        /// <summary>
        /// Get the second fixture in this contact.
        /// </summary>
        /// <returns></returns>
        public Fixture FixtureB
        {
            get
            {
                return m_fixtureB;
            }
        }

        public int ChildIndexB
        {
            get
            {
                return m_indexB;
            }
        }

        public float Friction
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

        public float Restitution
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

        public float TangentSpeed
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

        /// <summary>
        /// Get the world manifold.
        /// </summary>
        public void getWorldManifold(WorldManifold worldManifold)
        {
            Body bodyA = m_fixtureA.Body;
            Body bodyB = m_fixtureB.Body;
            Shape shapeA = m_fixtureA.Shape;
            Shape shapeB = m_fixtureB.Shape;

            worldManifold.Initialize(m_manifold, bodyA.GetTransform(), shapeA.Radius, bodyB.GetTransform(), shapeB.Radius);
        }

        public void resetFriction()
        {
            m_friction = mixFriction(m_fixtureA.Friction, m_fixtureB.Friction);
        }

        public void resetRestitution()
        {
            m_restitution = mixRestitution(m_fixtureA.Restitution, m_fixtureB.Restitution);
        }

        public abstract void evaluate(Manifold manifold, Transform xfA, Transform xfB);

        /// <summary>
        /// Flag this contact for filtering. Filtering will occur the next time step.
        /// </summary>
        public void flagForFiltering()
        {
            m_flags |= FILTER_FLAG;
        }

        // djm pooling
        private readonly Manifold oldManifold = new Manifold();

        public void update(IContactListener listener)
        {

            oldManifold.Set(m_manifold);

            // Re-enable this contact.
            m_flags |= ENABLED_FLAG;

            bool touching = false;
            bool wasTouching = (m_flags & TOUCHING_FLAG) == TOUCHING_FLAG;

            bool sensorA = m_fixtureA.Sensor;
            bool sensorB = m_fixtureB.Sensor;
            bool sensor = sensorA || sensorB;

            Body bodyA = m_fixtureA.Body;
            Body bodyB = m_fixtureB.Body;
            Transform xfA = bodyA.GetTransform();
            Transform xfB = bodyB.GetTransform();
            // log.debug("TransformA: "+xfA);
            // log.debug("TransformB: "+xfB);

            if (sensor)
            {
                Shape shapeA = m_fixtureA.Shape;
                Shape shapeB = m_fixtureB.Shape;
                touching = pool.GetCollision().TestOverlap(shapeA, m_indexA, shapeB, m_indexB, xfA, xfB);

                // Sensors don't generate manifolds.
                m_manifold.PointCount = 0;
            }
            else
            {
                evaluate(m_manifold, xfA, xfB);
                touching = m_manifold.PointCount > 0;

                // Match old contact ids to new contact ids and copy the
                // stored impulses to warm start the solver.
                for (int i = 0; i < m_manifold.PointCount; ++i)
                {
                    ManifoldPoint mp2 = m_manifold.Points[i];
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
                m_flags |= TOUCHING_FLAG;
            }
            else
            {
                m_flags &= ~TOUCHING_FLAG;
            }

            if (listener == null)
            {
                return;
            }

            if (wasTouching == false && touching == true)
            {
                listener.BeginContact(this);
            }

            if (wasTouching == true && touching == false)
            {
                listener.EndContact(this);
            }

            if (sensor == false && touching)
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
        public static float mixFriction(float friction1, float friction2)
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
        public static float mixRestitution(float restitution1, float restitution2)
        {
            return restitution1 > restitution2 ? restitution1 : restitution2;
        }
    }
}