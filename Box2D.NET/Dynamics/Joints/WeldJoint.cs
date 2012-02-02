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

// Created at 3:38:38 AM Jan 15, 2011

using Box2D.Common;
using Box2D.Pooling;

namespace Box2D.Dynamics.Joints
{

    //Point-to-point constraint
    //C = p2 - p1
    //Cdot = v2 - v1
    //   = v2 + cross(w2, r2) - v1 - cross(w1, r1)
    //J = [-I -r1_skew I r2_skew ]
    //Identity used:
    //w k % (rx i + ry j) = w * (-ry i + rx j)

    //Angle constraint
    //C = angle2 - angle1 - referenceAngle
    //Cdot = w2 - w1
    //J = [0 0 -1 0 0 1]
    //K = invI1 + invI2

    /// <summary>
    /// A weld joint essentially glues two bodies together. A weld joint may distort somewhat because the
    /// island constraint solver is approximate.
    /// </summary>
    /// <author>Daniel Murphy</author>
    public class WeldJoint : Joint
    {
        private float m_frequencyHz;
        private float m_dampingRatio;
        private float m_bias;

        // Solver shared
        private readonly Vec2 m_localAnchorA;
        private readonly Vec2 m_localAnchorB;
        private float m_referenceAngle;
        private float m_gamma;
        private readonly Vec3 m_impulse;


        // Solver temp
        private int m_indexA;
        private int m_indexB;
        private readonly Vec2 m_rA = new Vec2();
        private readonly Vec2 m_rB = new Vec2();
        private readonly Vec2 m_localCenterA = new Vec2();
        private readonly Vec2 m_localCenterB = new Vec2();
        private float m_invMassA;
        private float m_invMassB;
        private float m_invIA;
        private float m_invIB;
        private readonly Mat33 m_mass = new Mat33();

        /// <param name="argWorld"></param>
        /// <param name="def"></param>
        protected internal WeldJoint(IWorldPool argWorld, WeldJointDef def)
            : base(argWorld, def)
        {
            m_localAnchorA = new Vec2(def.localAnchorA);
            m_localAnchorB = new Vec2(def.localAnchorB);
            m_referenceAngle = def.referenceAngle;
            m_frequencyHz = def.frequencyHz;
            m_dampingRatio = def.dampingRatio;

            m_impulse = new Vec3();
            m_impulse.setZero();
        }

        public override void getAnchorA(Vec2 argOut)
        {
            m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
        }

        public override void getAnchorB(Vec2 argOut)
        {
            m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
        }

        /// <seealso cref="Joint.getReactionForce(float, Vec2)"></seealso>
        public override void getReactionForce(float inv_dt, Vec2 argOut)
        {
            argOut.set_Renamed(m_impulse.x, m_impulse.y);
            argOut.mulLocal(inv_dt);
        }

        /// <seealso cref="Joint.getReactionTorque(float)"></seealso>
        public override float getReactionTorque(float inv_dt)
        {
            return inv_dt * m_impulse.z;
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

        virtual public float Frequency
        {
            get
            {
                return m_frequencyHz;
            }
            set
            {
                this.m_frequencyHz = value;
            }
        }

        virtual public float DampingRatio
        {
            get
            {
                return m_dampingRatio;
            }
            set
            {
                this.m_dampingRatio = value;
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

            //    Vec2 cA = data.positions[m_indexA].c;
            float aA = data.positions[m_indexA].a;
            Vec2 vA = data.velocities[m_indexA].v;
            float wA = data.velocities[m_indexA].w;

            //    Vec2 cB = data.positions[m_indexB].c;
            float aB = data.positions[m_indexB].a;
            Vec2 vB = data.velocities[m_indexB].v;
            float wB = data.velocities[m_indexB].w;

            Rot qA = pool.PopRot();
            Rot qB = pool.PopRot();
            Vec2 temp = pool.PopVec2();

            qA.set_Renamed(aA);
            qB.set_Renamed(aB);

            // Compute the effective masses.
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

            Mat33 K = pool.PopMat33();

            K.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
            K.ey.x = (-m_rA.y) * m_rA.x * iA - m_rB.y * m_rB.x * iB;
            K.ez.x = (-m_rA.y) * iA - m_rB.y * iB;
            K.ex.y = K.ey.x;
            K.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
            K.ez.y = m_rA.x * iA + m_rB.x * iB;
            K.ex.z = K.ez.x;
            K.ey.z = K.ez.y;
            K.ez.z = iA + iB;

            if (m_frequencyHz > 0.0f)
            {
                K.getInverse22(m_mass);

                float invM = iA + iB;
                float m = invM > 0.0f ? 1.0f / invM : 0.0f;

                float C = aB - aA - m_referenceAngle;

                // Frequency
                float omega = 2.0f * MathUtils.PI * m_frequencyHz;

                // Damping coefficient
                float d = 2.0f * m * m_dampingRatio * omega;

                // Spring stiffness
                float k = m * omega * omega;

                // magic formulas
                float h = data.step.dt;
                m_gamma = h * (d + h * k);
                m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
                m_bias = C * h * k * m_gamma;

                invM += m_gamma;
                m_mass.ez.z = invM != 0.0f ? 1.0f / invM : 0.0f;
            }
            else
            {
                K.getSymInverse33(m_mass);
                m_gamma = 0.0f;
                m_bias = 0.0f;
            }

            if (data.step.warmStarting)
            {
                Vec2 P = pool.PopVec2();
                // Scale impulses to support a variable time step.
                m_impulse.mulLocal(data.step.dtRatio);

                P.set_Renamed(m_impulse.x, m_impulse.y);

                vA.x -= mA * P.x;
                vA.y -= mA * P.y;
                wA -= iA * (Vec2.cross(m_rA, P) + m_impulse.z);

                vB.x += mB * P.x;
                vB.y += mB * P.y;
                wB += iB * (Vec2.cross(m_rB, P) + m_impulse.z);
                pool.PushVec2(1);
            }
            else
            {
                m_impulse.setZero();
            }

            data.velocities[m_indexA].v.set_Renamed(vA);
            data.velocities[m_indexA].w = wA;
            data.velocities[m_indexB].v.set_Renamed(vB);
            data.velocities[m_indexB].w = wB;

            pool.PushVec2(1);
            pool.PushRot(2);
            pool.PushMat33(1);
        }

        /// <seealso cref="Joint.solveVelocityConstraints(TimeStep)"></seealso>
        public override void solveVelocityConstraints(SolverData data)
        {
            Vec2 vA = data.velocities[m_indexA].v;
            float wA = data.velocities[m_indexA].w;
            Vec2 vB = data.velocities[m_indexB].v;
            float wB = data.velocities[m_indexB].w;

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            Vec2 Cdot1 = pool.PopVec2();
            Vec2 P = pool.PopVec2();
            Vec2 temp = pool.PopVec2();

            if (m_frequencyHz > 0.0f)
            {
                float Cdot2 = wB - wA;

                float impulse2 = (-m_mass.ez.z) * (Cdot2 + m_bias + m_gamma * m_impulse.z);
                m_impulse.z += impulse2;

                wA -= iA * impulse2;
                wB += iB * impulse2;

                Vec2.crossToOutUnsafe(wB, m_rB, Cdot1);
                Vec2.crossToOutUnsafe(wA, m_rA, temp);
                Cdot1.addLocal(vB).subLocal(vA).subLocal(temp);

                Vec2 impulse1 = P;
                Mat33.mul22ToOutUnsafe(m_mass, Cdot1, impulse1);
                impulse1.negateLocal();

                m_impulse.x += impulse1.x;
                m_impulse.y += impulse1.y;

                vA.x -= mA * P.x;
                vA.y -= mA * P.y;
                wA -= iA * Vec2.cross(m_rA, P);

                vB.x += mB * P.x;
                vB.y += mB * P.y;
                wB += iB * Vec2.cross(m_rB, P);
            }
            else
            {
                Vec2.crossToOutUnsafe(wA, m_rA, temp);
                Vec2.crossToOutUnsafe(wB, m_rB, Cdot1);
                Cdot1.addLocal(vB).subLocal(vA).subLocal(temp);
                float Cdot2 = wB - wA;

                Vec3 Cdot = pool.PopVec3();
                Cdot.set_Renamed(Cdot1.x, Cdot1.y, Cdot2);

                Vec3 impulse = pool.PopVec3();
                Mat33.mulToOutUnsafe(m_mass, Cdot, impulse);
                impulse.negateLocal();
                m_impulse.addLocal(impulse);

                P.set_Renamed(impulse.x, impulse.y);

                vA.x -= mA * P.x;
                vA.y -= mA * P.y;
                wA -= iA * (Vec2.cross(m_rA, P) + impulse.z);

                vB.x += mB * P.x;
                vB.y += mB * P.y;
                wB += iB * (Vec2.cross(m_rB, P) + impulse.z);

                pool.PushVec3(2);
            }

            data.velocities[m_indexA].v.set_Renamed(vA);
            data.velocities[m_indexA].w = wA;
            data.velocities[m_indexB].v.set_Renamed(vB);
            data.velocities[m_indexB].w = wB;

            pool.PushVec2(3);
        }

        /// <seealso cref="Joint.solvePositionConstraints(float)"></seealso>
        public override bool solvePositionConstraints(SolverData data)
        {
            Vec2 cA = data.positions[m_indexA].c;
            float aA = data.positions[m_indexA].a;
            Vec2 cB = data.positions[m_indexB].c;
            float aB = data.positions[m_indexB].a;
            Rot qA = pool.PopRot();
            Rot qB = pool.PopRot();
            Vec2 temp = pool.PopVec2();
            Vec2 rA = pool.PopVec2();
            Vec2 rB = pool.PopVec2();

            qA.set_Renamed(aA);
            qB.set_Renamed(aB);

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            Rot.mulToOutUnsafe(qA, temp.set_Renamed(m_localAnchorA).subLocal(m_localCenterA), rA);
            Rot.mulToOutUnsafe(qB, temp.set_Renamed(m_localAnchorB).subLocal(m_localCenterB), rB);
            float positionError, angularError;

            Mat33 K = pool.PopMat33();
            Vec2 C1 = pool.PopVec2();
            Vec2 P = pool.PopVec2();

            K.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
            K.ey.x = (-rA.y) * rA.x * iA - rB.y * rB.x * iB;
            K.ez.x = (-rA.y) * iA - rB.y * iB;
            K.ex.y = K.ey.x;
            K.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
            K.ez.y = rA.x * iA + rB.x * iB;
            K.ex.z = K.ez.x;
            K.ey.z = K.ez.y;
            K.ez.z = iA + iB;

            if (m_frequencyHz > 0.0f)
            {
                C1.set_Renamed(cB).addLocal(rB).subLocal(cA).subLocal(rA);

                positionError = C1.length();
                angularError = 0.0f;

                K.solve22ToOut(C1, P);
                P.negateLocal();

                cA.x -= mA * P.x;
                cA.y -= mA * P.y;
                aA -= iA * Vec2.cross(rA, P);

                cB.x += mB * P.x;
                cB.y += mB * P.y;
                aB += iB * Vec2.cross(rB, P);
            }
            else
            {
                C1.set_Renamed(cB).addLocal(rB).subLocal(cA).subLocal(rA);
                float C2 = aB - aA - m_referenceAngle;

                positionError = C1.length();
                angularError = MathUtils.abs(C2);

                Vec3 C = pool.PopVec3();
                Vec3 impulse = pool.PopVec3();
                C.set_Renamed(C1.x, C1.y, C2);

                K.solve33ToOut(C, impulse);
                impulse.negateLocal();
                P.set_Renamed(impulse.x, impulse.y);

                cA.x -= mA * P.x;
                cA.y -= mA * P.y;
                aA -= iA * (Vec2.cross(rA, P) + impulse.z);

                cB.x += mB * P.x;
                cB.y += mB * P.y;
                aB += iB * (Vec2.cross(rB, P) + impulse.z);
            }

            data.positions[m_indexA].c.set_Renamed(cA);
            data.positions[m_indexA].a = aA;
            data.positions[m_indexB].c.set_Renamed(cB);
            data.positions[m_indexB].a = aB;

            pool.PushVec2(5);
            pool.PushRot(2);
            pool.PushMat33(1);

            return positionError <= Settings.linearSlop && angularError <= Settings.angularSlop;
        }
    }
}