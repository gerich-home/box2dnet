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
using Box2D.Common;
using Box2D.Pooling;

namespace Box2D.Dynamics.Joints
{

    // updated to rev 100
    /// <summary>
    /// The base joint class. Joints are used to raint two bodies together in
    /// various fashions. Some joints also feature limits and motors.
    /// </summary>
    /// <author>Daniel Murphy</author>
    public abstract class Joint
    {
        public static Joint Create(World argWorld, JointDef def)
        {
            //Joint joint = null;
            switch (def.Type)
            {

                case JointType.Mouse:
                    return new MouseJoint(argWorld.Pool, (MouseJointDef)def);

                case JointType.Distance:
                    return new DistanceJoint(argWorld.Pool, (DistanceJointDef)def);

                case JointType.Prismatic:
                    return new PrismaticJoint(argWorld.Pool, (PrismaticJointDef)def);

                case JointType.Revolute:
                    return new RevoluteJoint(argWorld.Pool, (RevoluteJointDef)def);

                case JointType.Weld:
                    return new WeldJoint(argWorld.Pool, (WeldJointDef)def);

                case JointType.Friction:
                    return new FrictionJoint(argWorld.Pool, (FrictionJointDef)def);

                //case JointType.WHEEL:
                //    return new WheelJoint(argWorld.Pool, (LineJointDef)def);

                //case JointType.GEAR:
                //    return new GearJoint(argWorld.Pool, (GearJointDef)def);

                case JointType.Pulley:
                    return new PulleyJoint(argWorld.Pool, (PulleyJointDef)def);

                case JointType.ConstantVolume:
                    return new ConstantVolumeJoint(argWorld, (ConstantVolumeJointDef)def);
            }
            return null;
        }

        public static void Destroy(Joint joint)
        {
            joint.Destructor();
        }

        /// <summary>
        /// The type of the concrete joint.
        /// </summary>
        public JointType Type;

        public Joint Prev;

        /// <summary>
        /// The next joint the world joint list.
        /// </summary>
        public Joint Next;

        public JointEdge EdgeA;
        public JointEdge EdgeB;

        /// <summary>
        /// The first body attached to this joint.
        /// </summary>
        public Body BodyA;

        /// <summary>
        /// get the second body attached to this joint.
        /// </summary>
        public Body BodyB;
        public int Index;

        public bool IslandFlag;

        /// <summary>
        /// Get collide connected.
        /// Note: modifying the collide connect flag won't work correctly because
        /// the flag is only checked when fixture AABBs begin to overlap.
        /// </summary>
        public bool CollideConnected;

        /// <summary>
        /// The user data pointer.
        /// </summary>
        public Object UserData;

        protected internal IWorldPool Pool;

        // Cache here per time step to reduce cache misses.
        //	final Vec2 m_localCenterA, m_localCenterB;
        //	float m_invMassA, m_invIA;
        //	float m_invMassB, m_invIB;

        protected internal Joint(IWorldPool argWorldPool, JointDef def)
        {
            Debug.Assert(def.BodyA != def.BodyB);

            Pool = argWorldPool;
            Type = def.Type;
            Prev = null;
            Next = null;
            BodyA = def.BodyA;
            BodyB = def.BodyB;
            CollideConnected = def.CollideConnected;
            IslandFlag = false;
            UserData = def.UserData;
            Index = 0;

            EdgeA = new JointEdge();
            EdgeA.Joint = null;
            EdgeA.Other = null;
            EdgeA.Prev = null;
            EdgeA.Next = null;

            EdgeB = new JointEdge();
            EdgeB.Joint = null;
            EdgeB.Other = null;
            EdgeB.Prev = null;
            EdgeB.Next = null;

            //		m_localCenterA = new Vec2();
            //		m_localCenterB = new Vec2();
        }


        /// <summary>
        /// get the anchor point on bodyA in world coordinates.
        /// </summary>
        /// <returns></returns>
        public abstract void GetAnchorA(Vec2 argOut);

        /// <summary>
        /// get the anchor point on bodyB in world coordinates.
        /// </summary>
        /// <returns></returns>
        public abstract void GetAnchorB(Vec2 argOut);

        /// <summary>
        /// get the reaction force on body2 at the joint anchor in Newtons.
        /// </summary>
        /// <param name="inv_dt"></param>
        /// <returns></returns>
        public abstract void GetReactionForce(float inv_dt, Vec2 argOut);

        /// <summary>
        /// get the reaction torque on body2 in N*m.
        /// </summary>
        /// <param name="inv_dt"></param>
        /// <returns></returns>
        public abstract float GetReactionTorque(float inv_dt);

        /// <summary>
        /// Short-cut function to determine if either body is inactive.
        /// </summary>
        /// <returns></returns>
        public bool Active
        {
            get
            {
                return BodyA.Active && BodyB.Active;
            }
        }

        public abstract void InitVelocityConstraints(SolverData data);

        public abstract void SolveVelocityConstraints(SolverData data);

        /// <summary>
        /// This returns true if the position errors are within tolerance.
        /// </summary>
        /// <param name="baumgarte"></param>
        /// <returns></returns>
        public abstract bool SolvePositionConstraints(SolverData data);

        /// <summary>
        /// Override to handle destruction of joint
        /// </summary>
        public virtual void Destructor()
        {
        }
    }
}