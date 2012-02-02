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

        virtual public Vec2 LocalAnchorA
        {
            get
            {
                return m_localAnchorA;
            }
        }

        virtual public Vec2 LocalAnchorB
        {
            get
            {
                return m_localAnchorB;
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
            argOut.set_Renamed(m_linearImpulse).mulLocal(inv_dt);
        }

        public override float getReactionTorque(float inv_dt)
        {
            return inv_dt * m_angularImpulse;
        }

        virtual public float MaxForce
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
        virtual public float MaxTorque
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

            float aA = data.positions[m_indexA].a;
            Vec2 vA = data.velocities[m_indexA].v;
            float wA = data.velocities[m_indexA].w;

            float aB = data.positions[m_indexB].a;
            Vec2 vB = data.velocities[m_indexB].v;
            float wB = data.velocities[m_indexB].w;


            Vec2 temp = pool.PopVec2();
            Rot qA = pool.PopRot();
            Rot qB = pool.PopRot();

            qA.set_Renamed(aA);
            qB.set_Renamed(aB);

            // Compute the effective mass matrix.
            Rot.mulToOutUnsafe(qA, temp.set_Renamed(m_localAnchorA).subLocal(m_localCenterA), m_rA);
            Rot.mulToOutUnsafe(qB, temp.set_Renamed(m_localAnchorB).subLocal(m_localCenterB), m_rB);

            // J = [-I -r1_skew I r2_skew]
            // [ 0 -1 0 1]
            // r_skew = [-ry; rx]

            // Matlab
            // K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x, -r1y*iA-r2y*iB]
            // [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
            // [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            Mat22 K = pool.PopMat22();
            K.ex.x = mA + mB + iA * m_rA.y * m_rA.y + iB * m_rB.y * m_rB.y;
            K.ex.y = (-iA) * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y;
            K.ey.x = K.ex.y;
            K.ey.y = mA + mB + iA * m_rA.x * m_rA.x + iB * m_rB.x * m_rB.x;

            K.invertToOut(m_linearMass);

            m_angularMass = iA + iB;
            if (m_angularMass > 0.0f)
            {
                m_angularMass = 1.0f / m_angularMass;
            }

            if (data.step.warmStarting)
            {
                // Scale impulses to support a variable time step.
                m_linearImpulse.mulLocal(data.step.dtRatio);
                m_angularImpulse *= data.step.dtRatio;

                Vec2 P = pool.PopVec2();
                P.set_Renamed(m_linearImpulse);

                temp.set_Renamed(P).mulLocal(mA);
                vA.subLocal(temp);
                wA -= iA * (Vec2.cross(m_rA, P) + m_angularImpulse);

                temp.set_Renamed(P).mulLocal(mB);
                vB.addLocal(temp);
                wB += iB * (Vec2.cross(m_rB, P) + m_angularImpulse);

                pool.PushVec2(1);
            }
            else
            {
                m_linearImpulse.setZero();
                m_angularImpulse = 0.0f;
            }
            data.velocities[m_indexA].v.set_Renamed(vA);
            if (data.velocities[m_indexA].w != wA)
            {
                Debug.Assert(data.velocities[m_indexA].w != wA);
            }
            data.velocities[m_indexA].w = wA;
            data.velocities[m_indexB].v.set_Renamed(vB);
            data.velocities[m_indexB].w = wB;

            pool.PushRot(2);
            pool.PushVec2(1);
            pool.PushMat22(1);
        }

        public override void solveVelocityConstraints(SolverData data)
        {
            Vec2 vA = data.velocities[m_indexA].v;
            float wA = data.velocities[m_indexA].w;
            Vec2 vB = data.velocities[m_indexB].v;
            float wB = data.velocities[m_indexB].w;

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            float h = data.step.dt;

            // Solve angular friction
            {
                float Cdot = wB - wA;
                float impulse = (-m_angularMass) * Cdot;

                float oldImpulse = m_angularImpulse;
                float maxImpulse = h * m_maxTorque;
                m_angularImpulse = MathUtils.clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = m_angularImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }

            // Solve linear friction
            {
                Vec2 Cdot = pool.PopVec2();
                Vec2 temp = pool.PopVec2();

                Vec2.crossToOutUnsafe(wA, m_rA, temp);
                Vec2.crossToOutUnsafe(wB, m_rB, Cdot);
                Cdot.addLocal(vB).subLocal(vA).subLocal(temp);

                Vec2 impulse = pool.PopVec2();
                Mat22.mulToOutUnsafe(m_linearMass, Cdot, impulse);
                impulse.negateLocal();


                Vec2 oldImpulse = pool.PopVec2();
                oldImpulse.set_Renamed(m_linearImpulse);
                m_linearImpulse.addLocal(impulse);

                float maxImpulse = h * m_maxForce;

                if (m_linearImpulse.lengthSquared() > maxImpulse * maxImpulse)
                {
                    m_linearImpulse.normalize();
                    m_linearImpulse.mulLocal(maxImpulse);
                }

                impulse.set_Renamed(m_linearImpulse).subLocal(oldImpulse);

                temp.set_Renamed(impulse).mulLocal(mA);
                vA.subLocal(temp);
                wA -= iA * Vec2.cross(m_rA, impulse);

                temp.set_Renamed(impulse).mulLocal(mB);
                vB.addLocal(temp);
                wB += iB * Vec2.cross(m_rB, impulse);
            }

            data.velocities[m_indexA].v.set_Renamed(vA);
            if (data.velocities[m_indexA].w != wA)
            {
                Debug.Assert(data.velocities[m_indexA].w != wA);
            }
            data.velocities[m_indexA].w = wA;
            data.velocities[m_indexB].v.set_Renamed(vB);
            data.velocities[m_indexB].w = wB;

            pool.PushVec2(4);
        }

        public override bool solvePositionConstraints(SolverData data)
        {
            return true;
        }
    }
}