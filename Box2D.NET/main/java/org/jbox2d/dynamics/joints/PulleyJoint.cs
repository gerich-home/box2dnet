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

using System;
using MathUtils = org.jbox2d.common.MathUtils;
using Rot = org.jbox2d.common.Rot;
using Settings = org.jbox2d.common.Settings;
using Vec2 = org.jbox2d.common.Vec2;
using SolverData = org.jbox2d.dynamics.SolverData;
using IWorldPool = org.jbox2d.pooling.IWorldPool;
using System.Diagnostics;

namespace org.jbox2d.dynamics.joints
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
            m_groundAnchorA.set_Renamed(def.groundAnchorA);
            m_groundAnchorB.set_Renamed(def.groundAnchorB);
            m_localAnchorA.set_Renamed(def.localAnchorA);
            m_localAnchorB.set_Renamed(def.localAnchorB);

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
                Vec2 p = pool.popVec2();
                m_bodyA.getWorldPointToOut(m_localAnchorA, p);
                p.subLocal(m_groundAnchorA);
                float length = p.length();
                pool.pushVec2(1);
                return length;
            }
        }

        virtual public float CurrentLengthB
        {
            get
            {
                Vec2 p = pool.popVec2();
                m_bodyB.getWorldPointToOut(m_localAnchorB, p);
                p.subLocal(m_groundAnchorB);
                float length = p.length();
                pool.pushVec2(1);
                return length;
            }
        }

        public override void getAnchorA(Vec2 argOut)
        {
            m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
        }

        public override void getAnchorB(Vec2 argOut)
        {
            m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
        }

        public override void getReactionForce(float inv_dt, Vec2 argOut)
        {
            argOut.set_Renamed(m_uB).mulLocal(m_impulse).mulLocal(inv_dt);
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
                Vec2 p = pool.popVec2();
                m_bodyA.getWorldPointToOut(m_localAnchorA, p);
                p.subLocal(m_groundAnchorA);

                float len = p.length();
                pool.pushVec2(1);
                return len;
            }
        }

        virtual public float Length2
        {
            get
            {
                Vec2 p = pool.popVec2();
                m_bodyB.getWorldPointToOut(m_localAnchorB, p);
                p.subLocal(m_groundAnchorB);

                float len = p.length();
                pool.pushVec2(1);
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
            m_indexA = m_bodyA.m_islandIndex;
            m_indexB = m_bodyB.m_islandIndex;
            m_localCenterA.set_Renamed(m_bodyA.m_sweep.localCenter);
            m_localCenterB.set_Renamed(m_bodyB.m_sweep.localCenter);
            m_invMassA = m_bodyA.m_invMass;
            m_invMassB = m_bodyB.m_invMass;
            m_invIA = m_bodyA.m_invI;
            m_invIB = m_bodyB.m_invI;

            Vec2 cA = data.positions[m_indexA].c;
            float aA = data.positions[m_indexA].a;
            Vec2 vA = data.velocities[m_indexA].v;
            float wA = data.velocities[m_indexA].w;

            Vec2 cB = data.positions[m_indexB].c;
            float aB = data.positions[m_indexB].a;
            Vec2 vB = data.velocities[m_indexB].v;
            float wB = data.velocities[m_indexB].w;

            Rot qA = pool.popRot();
            Rot qB = pool.popRot();
            Vec2 temp = pool.popVec2();

            qA.set_Renamed(aA);
            qB.set_Renamed(aB);

            // Compute the effective masses.
            Rot.mulToOutUnsafe(qA, temp.set_Renamed(m_localAnchorA).subLocal(m_localCenterA), m_rA);
            Rot.mulToOutUnsafe(qB, temp.set_Renamed(m_localAnchorB).subLocal(m_localCenterB), m_rB);

            m_uA.set_Renamed(cA).addLocal(m_rA).subLocal(m_groundAnchorA);
            m_uB.set_Renamed(cB).addLocal(m_rB).subLocal(m_groundAnchorB);

            float lengthA = m_uA.length();
            float lengthB = m_uB.length();

            if (lengthA > 10f * Settings.linearSlop)
            {
                m_uA.mulLocal(1.0f / lengthA);
            }
            else
            {
                m_uA.setZero();
            }

            if (lengthB > 10f * Settings.linearSlop)
            {
                m_uB.mulLocal(1.0f / lengthB);
            }
            else
            {
                m_uB.setZero();
            }

            // Compute effective mass.
            float ruA = Vec2.cross(m_rA, m_uA);
            float ruB = Vec2.cross(m_rB, m_uB);

            float mA = m_invMassA + m_invIA * ruA * ruA;
            float mB = m_invMassB + m_invIB * ruB * ruB;

            m_mass = mA + m_ratio * m_ratio * mB;

            if (m_mass > 0.0f)
            {
                m_mass = 1.0f / m_mass;
            }

            if (data.step.warmStarting)
            {

                // Scale impulses to support variable time steps.
                m_impulse *= data.step.dtRatio;

                // Warm starting.
                Vec2 PA = pool.popVec2();
                Vec2 PB = pool.popVec2();

                PA.set_Renamed(m_uA).mulLocal(-m_impulse);
                PB.set_Renamed(m_uB).mulLocal((-m_ratio) * m_impulse);

                vA.x += m_invMassA * PA.x;
                vA.y += m_invMassA * PA.y;
                wA += m_invIA * Vec2.cross(m_rA, PA);
                vB.x += m_invMassB * PB.x;
                vB.y += m_invMassB * PB.y;
                wB += m_invIB * Vec2.cross(m_rB, PB);

                pool.pushVec2(2);
            }
            else
            {
                m_impulse = 0.0f;
            }
            data.velocities[m_indexA].v.set_Renamed(vA);
            data.velocities[m_indexA].w = wA;
            data.velocities[m_indexB].v.set_Renamed(vB);
            data.velocities[m_indexB].w = wB;

            pool.pushVec2(1);
            pool.pushRot(2);
        }

        public override void solveVelocityConstraints(SolverData data)
        {
            Vec2 vA = data.velocities[m_indexA].v;
            float wA = data.velocities[m_indexA].w;
            Vec2 vB = data.velocities[m_indexB].v;
            float wB = data.velocities[m_indexB].w;

            Vec2 vpA = pool.popVec2();
            Vec2 vpB = pool.popVec2();
            Vec2 PA = pool.popVec2();
            Vec2 PB = pool.popVec2();

            Vec2.crossToOutUnsafe(wA, m_rA, vpA);
            vpA.addLocal(vA);
            Vec2.crossToOutUnsafe(wB, m_rB, vpB);
            vpB.addLocal(vB);

            float Cdot = -Vec2.dot(m_uA, vpA) - m_ratio * Vec2.dot(m_uB, vpB);
            float impulse = (-m_mass) * Cdot;
            m_impulse += impulse;

            PA.set_Renamed(m_uA).mulLocal(-impulse);
            PB.set_Renamed(m_uB).mulLocal((-m_ratio) * impulse);
            vA.x += m_invMassA * PA.x;
            vA.y += m_invMassA * PA.y;
            wA += m_invIA * Vec2.cross(m_rA, PA);
            vB.x += m_invMassB * PB.x;
            vB.y += m_invMassB * PB.y;
            wB += m_invIB * Vec2.cross(m_rB, PB);

            data.velocities[m_indexA].v.set_Renamed(vA);
            data.velocities[m_indexA].w = wA;
            data.velocities[m_indexB].v.set_Renamed(vB);
            data.velocities[m_indexB].w = wB;

            pool.pushVec2(4);
        }
        public override bool solvePositionConstraints(SolverData data)
        {
            Rot qA = pool.popRot();
            Rot qB = pool.popRot();
            Vec2 rA = pool.popVec2();
            Vec2 rB = pool.popVec2();
            Vec2 uA = pool.popVec2();
            Vec2 uB = pool.popVec2();
            Vec2 temp = pool.popVec2();
            Vec2 PA = pool.popVec2();
            Vec2 PB = pool.popVec2();

            Vec2 cA = data.positions[m_indexA].c;
            float aA = data.positions[m_indexA].a;
            Vec2 cB = data.positions[m_indexB].c;
            float aB = data.positions[m_indexB].a;

            qA.set_Renamed(aA);
            qB.set_Renamed(aB);

            Rot.mulToOutUnsafe(qA, temp.set_Renamed(m_localAnchorA).subLocal(m_localCenterA), rA);
            Rot.mulToOutUnsafe(qB, temp.set_Renamed(m_localAnchorB).subLocal(m_localCenterB), rB);

            uA.set_Renamed(cA).addLocal(rA).subLocal(m_groundAnchorA);
            uB.set_Renamed(cB).addLocal(rB).subLocal(m_groundAnchorB);

            float lengthA = uA.length();
            float lengthB = uB.length();

            if (lengthA > 10.0f * Settings.linearSlop)
            {
                uA.mulLocal(1.0f / lengthA);
            }
            else
            {
                uA.setZero();
            }

            if (lengthB > 10.0f * Settings.linearSlop)
            {
                uB.mulLocal(1.0f / lengthB);
            }
            else
            {
                uB.setZero();
            }

            // Compute effective mass.
            float ruA = Vec2.cross(rA, uA);
            float ruB = Vec2.cross(rB, uB);

            float mA = m_invMassA + m_invIA * ruA * ruA;
            float mB = m_invMassB + m_invIB * ruB * ruB;

            float mass = mA + m_ratio * m_ratio * mB;

            if (mass > 0.0f)
            {
                mass = 1.0f / mass;
            }

            float C = m_constant - lengthA - m_ratio * lengthB;
            float linearError = MathUtils.abs(C);

            float impulse = (-mass) * C;

            PA.set_Renamed(uA).mulLocal(-impulse);
            PB.set_Renamed(uB).mulLocal((-m_ratio) * impulse);

            cA.x += m_invMassA * PA.x;
            cA.y += m_invMassA * PA.y;
            aA += m_invIA * Vec2.cross(rA, PA);
            cB.x += m_invMassB * PB.x;
            cB.y += m_invMassB * PB.y;
            aB += m_invIB * Vec2.cross(rB, PB);

            data.positions[m_indexA].c.set_Renamed(cA);
            data.positions[m_indexA].a = aA;
            data.positions[m_indexB].c.set_Renamed(cB);
            data.positions[m_indexB].a = aB;

            pool.pushRot(2);
            pool.pushVec2(7);

            return linearError < Settings.linearSlop;
        }
    }
}