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

// Created at 7:27:32 AM Jan 20, 2011

using System.Diagnostics;
using Box2D.Common;
using Box2D.Pooling;

namespace Box2D.Dynamics.Joints
{

    /// <author>Daniel Murphy</author>
    public class FrictionJoint : Joint
    {
        private readonly Vec2 m_localAnchorA;
        private readonly Vec2 m_localAnchorB;

        // Solver shared
        private readonly Vec2 m_linearImpulse;
        private float m_angularImpulse;
        private float m_maxForce;
        private float m_maxTorque;

        // Solver temp
        public int m_indexA;
        public int m_indexB;
        public readonly Vec2 m_rA = new Vec2();
        public readonly Vec2 m_rB = new Vec2();
        public readonly Vec2 m_localCenterA = new Vec2();
        public readonly Vec2 m_localCenterB = new Vec2();
        public float m_invMassA;
        public float m_invMassB;
        public float m_invIA;
        public float m_invIB;
        public readonly Mat22 m_linearMass = new Mat22();
        public float m_angularMass;

        /// <param name="argWorldPool"></param>
        /// <param name="def"></param>
        public FrictionJoint(IWorldPool argWorldPool, FrictionJointDef def)
            : base(argWorldPool, def)
        {
            m_localAnchorA = new Vec2(def.localAnchorA);
            m_localAnchorB = new Vec2(def.localAnchorB);

            m_linearImpulse = new Vec2();
            m_angularImpulse = 0.0f;

            m_maxForce = def.maxForce;
            m_maxTorque = def.maxTorque;
        }

        public Vec2 LocalAnchorA
        {
            get
            {
                return m_localAnchorA;
            }
        }

        public Vec2 LocalAnchorB
        {
            get
            {
                return m_localAnchorB;
            }
        }

        public override void GetAnchorA(Vec2 argOut)
        {
            BodyA.GetWorldPointToOut(m_localAnchorA, argOut);
        }

        public override void GetAnchorB(Vec2 argOut)
        {
            BodyB.GetWorldPointToOut(m_localAnchorB, argOut);
        }

        public override void GetReactionForce(float inv_dt, Vec2 argOut)
        {
            argOut.Set(m_linearImpulse).MulLocal(inv_dt);
        }

        public override float GetReactionTorque(float inv_dt)
        {
            return inv_dt * m_angularImpulse;
        }

        public float MaxForce
        {
            get
            {
                return m_maxForce;
            }
            set
            {
                Debug.Assert(value >= 0.0f);
                m_maxForce = value;
            }
        }
        public float MaxTorque
        {
            get
            {
                return m_maxTorque;
            }
            set
            {
                Debug.Assert(value >= 0.0f);
                m_maxTorque = value;
            }
        }

        /// <seealso cref="Joint.initVelocityConstraints(TimeStep)"></seealso>
        public override void InitVelocityConstraints(SolverData data)
        {
            m_indexA = BodyA.IslandIndex;
            m_indexB = BodyB.IslandIndex;
            m_localCenterA.Set(BodyA.Sweep.LocalCenter);
            m_localCenterB.Set(BodyB.Sweep.LocalCenter);
            m_invMassA = BodyA.InvMass;
            m_invMassB = BodyB.InvMass;
            m_invIA = BodyA.InvI;
            m_invIB = BodyB.InvI;

            float aA = data.Positions[m_indexA].A;
            Vec2 vA = data.Velocities[m_indexA].V;
            float wA = data.Velocities[m_indexA].W;

            float aB = data.Positions[m_indexB].A;
            Vec2 vB = data.Velocities[m_indexB].V;
            float wB = data.Velocities[m_indexB].W;


            Vec2 temp = Pool.PopVec2();
            Rot qA = Pool.PopRot();
            Rot qB = Pool.PopRot();

            qA.Set(aA);
            qB.Set(aB);

            // Compute the effective mass matrix.
            Rot.MulToOutUnsafe(qA, temp.Set(m_localAnchorA).SubLocal(m_localCenterA), m_rA);
            Rot.MulToOutUnsafe(qB, temp.Set(m_localAnchorB).SubLocal(m_localCenterB), m_rB);

            // J = [-I -r1_skew I r2_skew]
            // [ 0 -1 0 1]
            // r_skew = [-ry; rx]

            // Matlab
            // K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x, -r1y*iA-r2y*iB]
            // [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
            // [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            Mat22 K = Pool.PopMat22();
            K.Ex.X = mA + mB + iA * m_rA.Y * m_rA.Y + iB * m_rB.Y * m_rB.Y;
            K.Ex.Y = (-iA) * m_rA.X * m_rA.Y - iB * m_rB.X * m_rB.Y;
            K.Ey.X = K.Ex.Y;
            K.Ey.Y = mA + mB + iA * m_rA.X * m_rA.X + iB * m_rB.X * m_rB.X;

            K.InvertToOut(m_linearMass);

            m_angularMass = iA + iB;
            if (m_angularMass > 0.0f)
            {
                m_angularMass = 1.0f / m_angularMass;
            }

            if (data.Step.WarmStarting)
            {
                // Scale impulses to support a variable time step.
                m_linearImpulse.MulLocal(data.Step.DtRatio);
                m_angularImpulse *= data.Step.DtRatio;

                Vec2 P = Pool.PopVec2();
                P.Set(m_linearImpulse);

                temp.Set(P).MulLocal(mA);
                vA.SubLocal(temp);
                wA -= iA * (Vec2.Cross(m_rA, P) + m_angularImpulse);

                temp.Set(P).MulLocal(mB);
                vB.AddLocal(temp);
                wB += iB * (Vec2.Cross(m_rB, P) + m_angularImpulse);

                Pool.PushVec2(1);
            }
            else
            {
                m_linearImpulse.SetZero();
                m_angularImpulse = 0.0f;
            }
            data.Velocities[m_indexA].V.Set(vA);
            if (data.Velocities[m_indexA].W != wA)
            {
                Debug.Assert(data.Velocities[m_indexA].W != wA);
            }
            data.Velocities[m_indexA].W = wA;
            data.Velocities[m_indexB].V.Set(vB);
            data.Velocities[m_indexB].W = wB;

            Pool.PushRot(2);
            Pool.PushVec2(1);
            Pool.PushMat22(1);
        }

        public override void SolveVelocityConstraints(SolverData data)
        {
            Vec2 vA = data.Velocities[m_indexA].V;
            float wA = data.Velocities[m_indexA].W;
            Vec2 vB = data.Velocities[m_indexB].V;
            float wB = data.Velocities[m_indexB].W;

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            float h = data.Step.Dt;

            // Solve angular friction
            {
                float Cdot = wB - wA;
                float impulse = (-m_angularMass) * Cdot;

                float oldImpulse = m_angularImpulse;
                float maxImpulse = h * m_maxTorque;
                m_angularImpulse = MathUtils.Clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = m_angularImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Solve linear friction
            {
                Vec2 Cdot = Pool.PopVec2();
                Vec2 temp = Pool.PopVec2();

                Vec2.CrossToOutUnsafe(wA, m_rA, temp);
                Vec2.CrossToOutUnsafe(wB, m_rB, Cdot);
                Cdot.AddLocal(vB).SubLocal(vA).SubLocal(temp);

                Vec2 impulse = Pool.PopVec2();
                Mat22.MulToOutUnsafe(m_linearMass, Cdot, impulse);
                impulse.NegateLocal();


                Vec2 oldImpulse = Pool.PopVec2();
                oldImpulse.Set(m_linearImpulse);
                m_linearImpulse.AddLocal(impulse);

                float maxImpulse = h * m_maxForce;

                if (m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse)
                {
                    m_linearImpulse.Normalize();
                    m_linearImpulse.MulLocal(maxImpulse);
                }

                impulse.Set(m_linearImpulse).SubLocal(oldImpulse);

                temp.Set(impulse).MulLocal(mA);
                vA.SubLocal(temp);
                wA -= iA * Vec2.Cross(m_rA, impulse);

                temp.Set(impulse).MulLocal(mB);
                vB.AddLocal(temp);
                wB += iB * Vec2.Cross(m_rB, impulse);
            }

            data.Velocities[m_indexA].V.Set(vA);
            if (data.Velocities[m_indexA].W != wA)
            {
                Debug.Assert(data.Velocities[m_indexA].W != wA);
            }
            data.Velocities[m_indexA].W = wA;
            data.Velocities[m_indexB].V.Set(vB);
            data.Velocities[m_indexB].W = wB;

            Pool.PushVec2(4);
        }

        public override bool SolvePositionConstraints(SolverData data)
        {
            return true;
        }
    }
}