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

        private readonly Vec2 m_groundAnchorA = new Vec2();
        private readonly Vec2 m_groundAnchorB = new Vec2();
        private float m_lengthA;
        private float m_LengthB;

        // Solver shared
        public readonly Vec2 LocalAnchorA = new Vec2();
        public readonly Vec2 LocalAnchorB = new Vec2();
        private float m_constant;
        private float m_ratio;
        private float m_impulse;

        // Solver temp
        public int IndexA;
        public int IndexB;
        private readonly Vec2 m_uA = new Vec2();
        private readonly Vec2 m_uB = new Vec2();
        public readonly Vec2 RA = new Vec2();
        public readonly Vec2 RB = new Vec2();
        private readonly Vec2 m_localCenterA = new Vec2();
        private readonly Vec2 m_localCenterB = new Vec2();
        public float InvMassA;
        public float InvMassB;
        public float InvIA;
        public float InvIB;
        private float m_mass;

        /// <param name="argWorldPool"></param>
        /// <param name="def"></param>
        public PulleyJoint(IWorldPool argWorldPool, PulleyJointDef def)
            : base(argWorldPool, def)
        {
            m_groundAnchorA.Set(def.GroundAnchorA);
            m_groundAnchorB.Set(def.GroundAnchorB);
            LocalAnchorA.Set(def.LocalAnchorA);
            LocalAnchorB.Set(def.LocalAnchorB);

            Debug.Assert(def.Ratio != 0.0f);
            m_ratio = def.Ratio;

            m_lengthA = def.LengthA;
            m_LengthB = def.LengthB;

            m_constant = def.LengthA + m_ratio * def.LengthB;
            m_impulse = 0.0f;
        }


        public float LengthA
        {
            get
            {
                return m_lengthA;
            }
        }

        public float LengthB
        {
            get
            {
                return m_LengthB;
            }
        }

        public float CurrentLengthA
        {
            get
            {
                Vec2 p = Pool.PopVec2();
                BodyA.GetWorldPointToOut(LocalAnchorA, p);
                p.SubLocal(m_groundAnchorA);
                float length = p.Length();
                Pool.PushVec2(1);
                return length;
            }
        }

        public float CurrentLengthB
        {
            get
            {
                Vec2 p = Pool.PopVec2();
                BodyB.GetWorldPointToOut(LocalAnchorB, p);
                p.SubLocal(m_groundAnchorB);
                float length = p.Length();
                Pool.PushVec2(1);
                return length;
            }
        }

        public override void GetAnchorA(Vec2 argOut)
        {
            BodyA.GetWorldPointToOut(LocalAnchorA, argOut);
        }

        public override void GetAnchorB(Vec2 argOut)
        {
            BodyB.GetWorldPointToOut(LocalAnchorB, argOut);
        }

        public override void GetReactionForce(float inv_dt, Vec2 argOut)
        {
            argOut.Set(m_uB).MulLocal(m_impulse).MulLocal(inv_dt);
        }

        public override float GetReactionTorque(float inv_dt)
        {
            return 0f;
        }

        public Vec2 GroundAnchorA
        {
            get
            {
                return m_groundAnchorA;
            }
        }

        public Vec2 GroundAnchorB
        {
            get
            {
                return m_groundAnchorB;
            }
        }

        public float Length1
        {
            get
            {
                Vec2 p = Pool.PopVec2();
                BodyA.GetWorldPointToOut(LocalAnchorA, p);
                p.SubLocal(m_groundAnchorA);

                float len = p.Length();
                Pool.PushVec2(1);
                return len;
            }
        }

        public float Length2
        {
            get
            {
                Vec2 p = Pool.PopVec2();
                BodyB.GetWorldPointToOut(LocalAnchorB, p);
                p.SubLocal(m_groundAnchorB);

                float len = p.Length();
                Pool.PushVec2(1);
                return len;
            }
        }

        public float Ratio
        {
            get
            {
                return m_ratio;
            }
        }

        public override void InitVelocityConstraints(SolverData data)
        {
            IndexA = BodyA.IslandIndex;
            IndexB = BodyB.IslandIndex;
            m_localCenterA.Set(BodyA.Sweep.LocalCenter);
            m_localCenterB.Set(BodyB.Sweep.LocalCenter);
            InvMassA = BodyA.InvMass;
            InvMassB = BodyB.InvMass;
            InvIA = BodyA.InvI;
            InvIB = BodyB.InvI;

            Vec2 cA = data.Positions[IndexA].C;
            float aA = data.Positions[IndexA].A;
            Vec2 vA = data.Velocities[IndexA].V;
            float wA = data.Velocities[IndexA].W;

            Vec2 cB = data.Positions[IndexB].C;
            float aB = data.Positions[IndexB].A;
            Vec2 vB = data.Velocities[IndexB].V;
            float wB = data.Velocities[IndexB].W;

            Rot qA = Pool.PopRot();
            Rot qB = Pool.PopRot();
            Vec2 temp = Pool.PopVec2();

            qA.Set(aA);
            qB.Set(aB);

            // Compute the effective masses.
            Rot.MulToOutUnsafe(qA, temp.Set(LocalAnchorA).SubLocal(m_localCenterA), RA);
            Rot.MulToOutUnsafe(qB, temp.Set(LocalAnchorB).SubLocal(m_localCenterB), RB);

            m_uA.Set(cA).AddLocal(RA).SubLocal(m_groundAnchorA);
            m_uB.Set(cB).AddLocal(RB).SubLocal(m_groundAnchorB);

            float lengthA = m_uA.Length();
            float lengthB = m_uB.Length();

            if (lengthA > 10f * Settings.LINEAR_SLOP)
            {
                m_uA.MulLocal(1.0f / lengthA);
            }
            else
            {
                m_uA.SetZero();
            }

            if (lengthB > 10f * Settings.LINEAR_SLOP)
            {
                m_uB.MulLocal(1.0f / lengthB);
            }
            else
            {
                m_uB.SetZero();
            }

            // Compute effective mass.
            float ruA = Vec2.Cross(RA, m_uA);
            float ruB = Vec2.Cross(RB, m_uB);

            float mA = InvMassA + InvIA * ruA * ruA;
            float mB = InvMassB + InvIB * ruB * ruB;

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
                Vec2 PA = Pool.PopVec2();
                Vec2 PB = Pool.PopVec2();

                PA.Set(m_uA).MulLocal(-m_impulse);
                PB.Set(m_uB).MulLocal((-m_ratio) * m_impulse);

                vA.X += InvMassA * PA.X;
                vA.Y += InvMassA * PA.Y;
                wA += InvIA * Vec2.Cross(RA, PA);
                vB.X += InvMassB * PB.X;
                vB.Y += InvMassB * PB.Y;
                wB += InvIB * Vec2.Cross(RB, PB);

                Pool.PushVec2(2);
            }
            else
            {
                m_impulse = 0.0f;
            }
            data.Velocities[IndexA].V.Set(vA);
            data.Velocities[IndexA].W = wA;
            data.Velocities[IndexB].V.Set(vB);
            data.Velocities[IndexB].W = wB;

            Pool.PushVec2(1);
            Pool.PushRot(2);
        }

        public override void SolveVelocityConstraints(SolverData data)
        {
            Vec2 vA = data.Velocities[IndexA].V;
            float wA = data.Velocities[IndexA].W;
            Vec2 vB = data.Velocities[IndexB].V;
            float wB = data.Velocities[IndexB].W;

            Vec2 vpA = Pool.PopVec2();
            Vec2 vpB = Pool.PopVec2();
            Vec2 PA = Pool.PopVec2();
            Vec2 PB = Pool.PopVec2();

            Vec2.CrossToOutUnsafe(wA, RA, vpA);
            vpA.AddLocal(vA);
            Vec2.CrossToOutUnsafe(wB, RB, vpB);
            vpB.AddLocal(vB);

            float Cdot = -Vec2.Dot(m_uA, vpA) - m_ratio * Vec2.Dot(m_uB, vpB);
            float impulse = (-m_mass) * Cdot;
            m_impulse += impulse;

            PA.Set(m_uA).MulLocal(-impulse);
            PB.Set(m_uB).MulLocal((-m_ratio) * impulse);
            vA.X += InvMassA * PA.X;
            vA.Y += InvMassA * PA.Y;
            wA += InvIA * Vec2.Cross(RA, PA);
            vB.X += InvMassB * PB.X;
            vB.Y += InvMassB * PB.Y;
            wB += InvIB * Vec2.Cross(RB, PB);

            data.Velocities[IndexA].V.Set(vA);
            data.Velocities[IndexA].W = wA;
            data.Velocities[IndexB].V.Set(vB);
            data.Velocities[IndexB].W = wB;

            Pool.PushVec2(4);
        }
        public override bool SolvePositionConstraints(SolverData data)
        {
            Rot qA = Pool.PopRot();
            Rot qB = Pool.PopRot();
            Vec2 rA = Pool.PopVec2();
            Vec2 rB = Pool.PopVec2();
            Vec2 uA = Pool.PopVec2();
            Vec2 uB = Pool.PopVec2();
            Vec2 temp = Pool.PopVec2();
            Vec2 PA = Pool.PopVec2();
            Vec2 PB = Pool.PopVec2();

            Vec2 cA = data.Positions[IndexA].C;
            float aA = data.Positions[IndexA].A;
            Vec2 cB = data.Positions[IndexB].C;
            float aB = data.Positions[IndexB].A;

            qA.Set(aA);
            qB.Set(aB);

            Rot.MulToOutUnsafe(qA, temp.Set(LocalAnchorA).SubLocal(m_localCenterA), rA);
            Rot.MulToOutUnsafe(qB, temp.Set(LocalAnchorB).SubLocal(m_localCenterB), rB);

            uA.Set(cA).AddLocal(rA).SubLocal(m_groundAnchorA);
            uB.Set(cB).AddLocal(rB).SubLocal(m_groundAnchorB);

            float lengthA = uA.Length();
            float lengthB = uB.Length();

            if (lengthA > 10.0f * Settings.LINEAR_SLOP)
            {
                uA.MulLocal(1.0f / lengthA);
            }
            else
            {
                uA.SetZero();
            }

            if (lengthB > 10.0f * Settings.LINEAR_SLOP)
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

            float mA = InvMassA + InvIA * ruA * ruA;
            float mB = InvMassB + InvIB * ruB * ruB;

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

            cA.X += InvMassA * PA.X;
            cA.Y += InvMassA * PA.Y;
            aA += InvIA * Vec2.Cross(rA, PA);
            cB.X += InvMassB * PB.X;
            cB.Y += InvMassB * PB.Y;
            aB += InvIB * Vec2.Cross(rB, PB);

            data.Positions[IndexA].C.Set(cA);
            data.Positions[IndexA].A = aA;
            data.Positions[IndexB].C.Set(cB);
            data.Positions[IndexB].A = aB;

            Pool.PushRot(2);
            Pool.PushVec2(7);

            return linearError < Settings.LINEAR_SLOP;
        }
    }
}