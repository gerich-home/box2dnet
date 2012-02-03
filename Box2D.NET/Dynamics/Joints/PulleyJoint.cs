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

// Created at 12:12:02 PM Jan 23, 2011

using System.Diagnostics;
using Box2D.Common;
using Box2D.Pooling;

namespace Box2D.Dynamics.Joints
{

    /// <summary>
    /// The pulley joint is connected to two bodies and two fixed ground points. The pulley supports a
    /// ratio such that: length1 + ratio * length2 &lt;= constant Yes, the force transmitted is scaled by
    /// the ratio. Warning: the pulley joint can get a bit squirrelly by itself. They often work better
    /// when combined with prismatic joints. You should also cover the the anchor points with static
    /// shapes to prevent one side from going to zero length.
    /// </summary>
    /// <author>Daniel Murphy</author>
    public class PulleyJoint : Joint
    {
        public const float MIN_PULLEY_LENGTH = 2.0f;

        public readonly Vec2 m_groundAnchorA = new Vec2();
        public readonly Vec2 m_groundAnchorB = new Vec2();
        private float m_lengthA;
        private float m_LengthB;

        // Solver shared
        public readonly Vec2 m_localAnchorA = new Vec2();
        public readonly Vec2 m_localAnchorB = new Vec2();
        private float m_constant;
        private float m_ratio;
        private float m_impulse;

        // Solver temp
        public int m_indexA;
        public int m_indexB;
        private readonly Vec2 m_uA = new Vec2();
        private readonly Vec2 m_uB = new Vec2();
        public readonly Vec2 m_rA = new Vec2();
        public readonly Vec2 m_rB = new Vec2();
        private readonly Vec2 m_localCenterA = new Vec2();
        private readonly Vec2 m_localCenterB = new Vec2();
        public float m_invMassA;
        public float m_invMassB;
        public float m_invIA;
        public float m_invIB;
        private float m_mass;

        /// <param name="argWorldPool"></param>
        /// <param name="def"></param>
        public PulleyJoint(IWorldPool argWorldPool, PulleyJointDef def)
            : base(argWorldPool, def)
        {
            m_groundAnchorA.Set(def.groundAnchorA);
            m_groundAnchorB.Set(def.groundAnchorB);
            m_localAnchorA.Set(def.localAnchorA);
            m_localAnchorB.Set(def.localAnchorB);

            Debug.Assert(def.ratio != 0.0f);
            m_ratio = def.ratio;

            m_lengthA = def.lengthA;
            m_LengthB = def.lengthB;

            m_constant = def.lengthA + m_ratio * def.lengthB;
            m_impulse = 0.0f;
        }


        virtual public float LengthA
        {
            get
            {
                return m_lengthA;
            }
        }

        virtual public float LengthB
        {
            get
            {
                return m_LengthB;
            }
        }

        virtual public float CurrentLengthA
        {
            get
            {
                Vec2 p = pool.PopVec2();
                m_bodyA.GetWorldPointToOut(m_localAnchorA, p);
                p.SubLocal(m_groundAnchorA);
                float length = p.Length();
                pool.PushVec2(1);
                return length;
            }
        }

        virtual public float CurrentLengthB
        {
            get
            {
                Vec2 p = pool.PopVec2();
                m_bodyB.GetWorldPointToOut(m_localAnchorB, p);
                p.SubLocal(m_groundAnchorB);
                float length = p.Length();
                pool.PushVec2(1);
                return length;
            }
        }

        public override void getAnchorA(Vec2 argOut)
        {
            m_bodyA.GetWorldPointToOut(m_localAnchorA, argOut);
        }

        public override void getAnchorB(Vec2 argOut)
        {
            m_bodyB.GetWorldPointToOut(m_localAnchorB, argOut);
        }

        public override void getReactionForce(float inv_dt, Vec2 argOut)
        {
            argOut.Set(m_uB).MulLocal(m_impulse).MulLocal(inv_dt);
        }

        public override float getReactionTorque(float inv_dt)
        {
            return 0f;
        }

        virtual public Vec2 GroundAnchorA
        {
            get
            {
                return m_groundAnchorA;
            }
        }

        virtual public Vec2 GroundAnchorB
        {
            get
            {
                return m_groundAnchorB;
            }
        }

        virtual public float Length1
        {
            get
            {
                Vec2 p = pool.PopVec2();
                m_bodyA.GetWorldPointToOut(m_localAnchorA, p);
                p.SubLocal(m_groundAnchorA);

                float len = p.Length();
                pool.PushVec2(1);
                return len;
            }
        }

        virtual public float Length2
        {
            get
            {
                Vec2 p = pool.PopVec2();
                m_bodyB.GetWorldPointToOut(m_localAnchorB, p);
                p.SubLocal(m_groundAnchorB);

                float len = p.Length();
                pool.PushVec2(1);
                return len;
            }
        }

        virtual public float Ratio
        {
            get
            {
                return m_ratio;
            }
        }

        public override void initVelocityConstraints(SolverData data)
        {
            m_indexA = m_bodyA.IslandIndex;
            m_indexB = m_bodyB.IslandIndex;
            m_localCenterA.Set(m_bodyA.Sweep.localCenter);
            m_localCenterB.Set(m_bodyB.Sweep.localCenter);
            m_invMassA = m_bodyA.InvMass;
            m_invMassB = m_bodyB.InvMass;
            m_invIA = m_bodyA.InvI;
            m_invIB = m_bodyB.InvI;

            Vec2 cA = data.Positions[m_indexA].c;
            float aA = data.Positions[m_indexA].a;
            Vec2 vA = data.Velocities[m_indexA].v;
            float wA = data.Velocities[m_indexA].w;

            Vec2 cB = data.Positions[m_indexB].c;
            float aB = data.Positions[m_indexB].a;
            Vec2 vB = data.Velocities[m_indexB].v;
            float wB = data.Velocities[m_indexB].w;

            Rot qA = pool.PopRot();
            Rot qB = pool.PopRot();
            Vec2 temp = pool.PopVec2();

            qA.set_Renamed(aA);
            qB.set_Renamed(aB);

            // Compute the effective masses.
            Rot.mulToOutUnsafe(qA, temp.Set(m_localAnchorA).SubLocal(m_localCenterA), m_rA);
            Rot.mulToOutUnsafe(qB, temp.Set(m_localAnchorB).SubLocal(m_localCenterB), m_rB);

            m_uA.Set(cA).AddLocal(m_rA).SubLocal(m_groundAnchorA);
            m_uB.Set(cB).AddLocal(m_rB).SubLocal(m_groundAnchorB);

            float lengthA = m_uA.Length();
            float lengthB = m_uB.Length();

            if (lengthA > 10f * Settings.linearSlop)
            {
                m_uA.MulLocal(1.0f / lengthA);
            }
            else
            {
                m_uA.SetZero();
            }

            if (lengthB > 10f * Settings.linearSlop)
            {
                m_uB.MulLocal(1.0f / lengthB);
            }
            else
            {
                m_uB.SetZero();
            }

            // Compute effective mass.
            float ruA = Vec2.Cross(m_rA, m_uA);
            float ruB = Vec2.Cross(m_rB, m_uB);

            float mA = m_invMassA + m_invIA * ruA * ruA;
            float mB = m_invMassB + m_invIB * ruB * ruB;

            m_mass = mA + m_ratio * m_ratio * mB;

            if (m_mass > 0.0f)
            {
                m_mass = 1.0f / m_mass;
            }

            if (data.Step.WarmStarting)
            {

                // Scale impulses to support variable time steps.
                m_impulse *= data.Step.DtRatio;

                // Warm starting.
                Vec2 PA = pool.PopVec2();
                Vec2 PB = pool.PopVec2();

                PA.Set(m_uA).MulLocal(-m_impulse);
                PB.Set(m_uB).MulLocal((-m_ratio) * m_impulse);

                vA.X += m_invMassA * PA.X;
                vA.Y += m_invMassA * PA.Y;
                wA += m_invIA * Vec2.Cross(m_rA, PA);
                vB.X += m_invMassB * PB.X;
                vB.Y += m_invMassB * PB.Y;
                wB += m_invIB * Vec2.Cross(m_rB, PB);

                pool.PushVec2(2);
            }
            else
            {
                m_impulse = 0.0f;
            }
            data.Velocities[m_indexA].v.Set(vA);
            data.Velocities[m_indexA].w = wA;
            data.Velocities[m_indexB].v.Set(vB);
            data.Velocities[m_indexB].w = wB;

            pool.PushVec2(1);
            pool.PushRot(2);
        }

        public override void solveVelocityConstraints(SolverData data)
        {
            Vec2 vA = data.Velocities[m_indexA].v;
            float wA = data.Velocities[m_indexA].w;
            Vec2 vB = data.Velocities[m_indexB].v;
            float wB = data.Velocities[m_indexB].w;

            Vec2 vpA = pool.PopVec2();
            Vec2 vpB = pool.PopVec2();
            Vec2 PA = pool.PopVec2();
            Vec2 PB = pool.PopVec2();

            Vec2.CrossToOutUnsafe(wA, m_rA, vpA);
            vpA.AddLocal(vA);
            Vec2.CrossToOutUnsafe(wB, m_rB, vpB);
            vpB.AddLocal(vB);

            float Cdot = -Vec2.Dot(m_uA, vpA) - m_ratio * Vec2.Dot(m_uB, vpB);
            float impulse = (-m_mass) * Cdot;
            m_impulse += impulse;

            PA.Set(m_uA).MulLocal(-impulse);
            PB.Set(m_uB).MulLocal((-m_ratio) * impulse);
            vA.X += m_invMassA * PA.X;
            vA.Y += m_invMassA * PA.Y;
            wA += m_invIA * Vec2.Cross(m_rA, PA);
            vB.X += m_invMassB * PB.X;
            vB.Y += m_invMassB * PB.Y;
            wB += m_invIB * Vec2.Cross(m_rB, PB);

            data.Velocities[m_indexA].v.Set(vA);
            data.Velocities[m_indexA].w = wA;
            data.Velocities[m_indexB].v.Set(vB);
            data.Velocities[m_indexB].w = wB;

            pool.PushVec2(4);
        }
        public override bool solvePositionConstraints(SolverData data)
        {
            Rot qA = pool.PopRot();
            Rot qB = pool.PopRot();
            Vec2 rA = pool.PopVec2();
            Vec2 rB = pool.PopVec2();
            Vec2 uA = pool.PopVec2();
            Vec2 uB = pool.PopVec2();
            Vec2 temp = pool.PopVec2();
            Vec2 PA = pool.PopVec2();
            Vec2 PB = pool.PopVec2();

            Vec2 cA = data.Positions[m_indexA].c;
            float aA = data.Positions[m_indexA].a;
            Vec2 cB = data.Positions[m_indexB].c;
            float aB = data.Positions[m_indexB].a;

            qA.set_Renamed(aA);
            qB.set_Renamed(aB);

            Rot.mulToOutUnsafe(qA, temp.Set(m_localAnchorA).SubLocal(m_localCenterA), rA);
            Rot.mulToOutUnsafe(qB, temp.Set(m_localAnchorB).SubLocal(m_localCenterB), rB);

            uA.Set(cA).AddLocal(rA).SubLocal(m_groundAnchorA);
            uB.Set(cB).AddLocal(rB).SubLocal(m_groundAnchorB);

            float lengthA = uA.Length();
            float lengthB = uB.Length();

            if (lengthA > 10.0f * Settings.linearSlop)
            {
                uA.MulLocal(1.0f / lengthA);
            }
            else
            {
                uA.SetZero();
            }

            if (lengthB > 10.0f * Settings.linearSlop)
            {
                uB.MulLocal(1.0f / lengthB);
            }
            else
            {
                uB.SetZero();
            }

            // Compute effective mass.
            float ruA = Vec2.Cross(rA, uA);
            float ruB = Vec2.Cross(rB, uB);

            float mA = m_invMassA + m_invIA * ruA * ruA;
            float mB = m_invMassB + m_invIB * ruB * ruB;

            float mass = mA + m_ratio * m_ratio * mB;

            if (mass > 0.0f)
            {
                mass = 1.0f / mass;
            }

            float C = m_constant - lengthA - m_ratio * lengthB;
            float linearError = MathUtils.Abs(C);

            float impulse = (-mass) * C;

            PA.Set(uA).MulLocal(-impulse);
            PB.Set(uB).MulLocal((-m_ratio) * impulse);

            cA.X += m_invMassA * PA.X;
            cA.Y += m_invMassA * PA.Y;
            aA += m_invIA * Vec2.Cross(rA, PA);
            cB.X += m_invMassB * PB.X;
            cB.Y += m_invMassB * PB.Y;
            aB += m_invIB * Vec2.Cross(rB, PB);

            data.Positions[m_indexA].c.Set(cA);
            data.Positions[m_indexA].a = aA;
            data.Positions[m_indexB].c.Set(cB);
            data.Positions[m_indexB].a = aB;

            pool.PushRot(2);
            pool.PushVec2(7);

            return linearError < Settings.linearSlop;
        }
    }
}