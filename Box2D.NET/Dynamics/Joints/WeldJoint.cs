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
            m_bodyA.GetWorldPointToOut(m_localAnchorA, argOut);
        }

        public override void getAnchorB(Vec2 argOut)
        {
            m_bodyB.GetWorldPointToOut(m_localAnchorB, argOut);
        }

        /// <seealso cref="Joint.getReactionForce(float, Vec2)"></seealso>
        public override void getReactionForce(float inv_dt, Vec2 argOut)
        {
            argOut.Set(m_impulse.x, m_impulse.y);
            argOut.MulLocal(inv_dt);
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
            m_indexA = m_bodyA.IslandIndex;
            m_indexB = m_bodyB.IslandIndex;
            m_localCenterA.Set(m_bodyA.Sweep.localCenter);
            m_localCenterB.Set(m_bodyB.Sweep.localCenter);
            m_invMassA = m_bodyA.InvMass;
            m_invMassB = m_bodyB.InvMass;
            m_invIA = m_bodyA.InvI;
            m_invIB = m_bodyB.InvI;

            //    Vec2 cA = data.positions[m_indexA].c;
            float aA = data.Positions[m_indexA].a;
            Vec2 vA = data.Velocities[m_indexA].v;
            float wA = data.Velocities[m_indexA].w;

            //    Vec2 cB = data.positions[m_indexB].c;
            float aB = data.Positions[m_indexB].a;
            Vec2 vB = data.Velocities[m_indexB].v;
            float wB = data.Velocities[m_indexB].w;

            Rot qA = pool.PopRot();
            Rot qB = pool.PopRot();
            Vec2 temp = pool.PopVec2();

            qA.Set(aA);
            qB.Set(aB);

            // Compute the effective masses.
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

            Mat33 K = pool.PopMat33();

            K.Ex.x = mA + mB + m_rA.Y * m_rA.Y * iA + m_rB.Y * m_rB.Y * iB;
            K.Ey.x = (-m_rA.Y) * m_rA.X * iA - m_rB.Y * m_rB.X * iB;
            K.Ez.x = (-m_rA.Y) * iA - m_rB.Y * iB;
            K.Ex.y = K.Ey.x;
            K.Ey.y = mA + mB + m_rA.X * m_rA.X * iA + m_rB.X * m_rB.X * iB;
            K.Ez.y = m_rA.X * iA + m_rB.X * iB;
            K.Ex.z = K.Ez.x;
            K.Ey.z = K.Ez.y;
            K.Ez.z = iA + iB;

            if (m_frequencyHz > 0.0f)
            {
                K.GetInverse22(m_mass);

                float invM = iA + iB;
                float m = invM > 0.0f ? 1.0f / invM : 0.0f;

                float C = aB - aA - m_referenceAngle;

                // Frequency
                float omega = 2.0f * MathUtils.Pi * m_frequencyHz;

                // Damping coefficient
                float d = 2.0f * m * m_dampingRatio * omega;

                // Spring stiffness
                float k = m * omega * omega;

                // magic formulas
                float h = data.Step.Dt;
                m_gamma = h * (d + h * k);
                m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
                m_bias = C * h * k * m_gamma;

                invM += m_gamma;
                m_mass.Ez.z = invM != 0.0f ? 1.0f / invM : 0.0f;
            }
            else
            {
                K.GetSymInverse33(m_mass);
                m_gamma = 0.0f;
                m_bias = 0.0f;
            }

            if (data.Step.WarmStarting)
            {
                Vec2 P = pool.PopVec2();
                // Scale impulses to support a variable time step.
                m_impulse.mulLocal(data.Step.DtRatio);

                P.Set(m_impulse.x, m_impulse.y);

                vA.X -= mA * P.X;
                vA.Y -= mA * P.Y;
                wA -= iA * (Vec2.Cross(m_rA, P) + m_impulse.z);

                vB.X += mB * P.X;
                vB.Y += mB * P.Y;
                wB += iB * (Vec2.Cross(m_rB, P) + m_impulse.z);
                pool.PushVec2(1);
            }
            else
            {
                m_impulse.setZero();
            }

            data.Velocities[m_indexA].v.Set(vA);
            data.Velocities[m_indexA].w = wA;
            data.Velocities[m_indexB].v.Set(vB);
            data.Velocities[m_indexB].w = wB;

            pool.PushVec2(1);
            pool.PushRot(2);
            pool.PushMat33(1);
        }

        /// <seealso cref="Joint.solveVelocityConstraints(TimeStep)"></seealso>
        public override void solveVelocityConstraints(SolverData data)
        {
            Vec2 vA = data.Velocities[m_indexA].v;
            float wA = data.Velocities[m_indexA].w;
            Vec2 vB = data.Velocities[m_indexB].v;
            float wB = data.Velocities[m_indexB].w;

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            Vec2 Cdot1 = pool.PopVec2();
            Vec2 P = pool.PopVec2();
            Vec2 temp = pool.PopVec2();

            if (m_frequencyHz > 0.0f)
            {
                float Cdot2 = wB - wA;

                float impulse2 = (-m_mass.Ez.z) * (Cdot2 + m_bias + m_gamma * m_impulse.z);
                m_impulse.z += impulse2;

                wA -= iA * impulse2;
                wB += iB * impulse2;

                Vec2.CrossToOutUnsafe(wB, m_rB, Cdot1);
                Vec2.CrossToOutUnsafe(wA, m_rA, temp);
                Cdot1.AddLocal(vB).SubLocal(vA).SubLocal(temp);

                Vec2 impulse1 = P;
                Mat33.Mul22ToOutUnsafe(m_mass, Cdot1, impulse1);
                impulse1.NegateLocal();

                m_impulse.x += impulse1.X;
                m_impulse.y += impulse1.Y;

                vA.X -= mA * P.X;
                vA.Y -= mA * P.Y;
                wA -= iA * Vec2.Cross(m_rA, P);

                vB.X += mB * P.X;
                vB.Y += mB * P.Y;
                wB += iB * Vec2.Cross(m_rB, P);
            }
            else
            {
                Vec2.CrossToOutUnsafe(wA, m_rA, temp);
                Vec2.CrossToOutUnsafe(wB, m_rB, Cdot1);
                Cdot1.AddLocal(vB).SubLocal(vA).SubLocal(temp);
                float Cdot2 = wB - wA;

                Vec3 Cdot = pool.PopVec3();
                Cdot.set_Renamed(Cdot1.X, Cdot1.Y, Cdot2);

                Vec3 impulse = pool.PopVec3();
                Mat33.MulToOutUnsafe(m_mass, Cdot, impulse);
                impulse.negateLocal();
                m_impulse.addLocal(impulse);

                P.Set(impulse.x, impulse.y);

                vA.X -= mA * P.X;
                vA.Y -= mA * P.Y;
                wA -= iA * (Vec2.Cross(m_rA, P) + impulse.z);

                vB.X += mB * P.X;
                vB.Y += mB * P.Y;
                wB += iB * (Vec2.Cross(m_rB, P) + impulse.z);

                pool.PushVec3(2);
            }

            data.Velocities[m_indexA].v.Set(vA);
            data.Velocities[m_indexA].w = wA;
            data.Velocities[m_indexB].v.Set(vB);
            data.Velocities[m_indexB].w = wB;

            pool.PushVec2(3);
        }

        /// <seealso cref="Joint.solvePositionConstraints(float)"></seealso>
        public override bool solvePositionConstraints(SolverData data)
        {
            Vec2 cA = data.Positions[m_indexA].c;
            float aA = data.Positions[m_indexA].a;
            Vec2 cB = data.Positions[m_indexB].c;
            float aB = data.Positions[m_indexB].a;
            Rot qA = pool.PopRot();
            Rot qB = pool.PopRot();
            Vec2 temp = pool.PopVec2();
            Vec2 rA = pool.PopVec2();
            Vec2 rB = pool.PopVec2();

            qA.Set(aA);
            qB.Set(aB);

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            Rot.MulToOutUnsafe(qA, temp.Set(m_localAnchorA).SubLocal(m_localCenterA), rA);
            Rot.MulToOutUnsafe(qB, temp.Set(m_localAnchorB).SubLocal(m_localCenterB), rB);
            float positionError, angularError;

            Mat33 K = pool.PopMat33();
            Vec2 C1 = pool.PopVec2();
            Vec2 P = pool.PopVec2();

            K.Ex.x = mA + mB + rA.Y * rA.Y * iA + rB.Y * rB.Y * iB;
            K.Ey.x = (-rA.Y) * rA.X * iA - rB.Y * rB.X * iB;
            K.Ez.x = (-rA.Y) * iA - rB.Y * iB;
            K.Ex.y = K.Ey.x;
            K.Ey.y = mA + mB + rA.X * rA.X * iA + rB.X * rB.X * iB;
            K.Ez.y = rA.X * iA + rB.X * iB;
            K.Ex.z = K.Ez.x;
            K.Ey.z = K.Ez.y;
            K.Ez.z = iA + iB;

            if (m_frequencyHz > 0.0f)
            {
                C1.Set(cB).AddLocal(rB).SubLocal(cA).SubLocal(rA);

                positionError = C1.Length();
                angularError = 0.0f;

                K.Solve22ToOut(C1, P);
                P.NegateLocal();

                cA.X -= mA * P.X;
                cA.Y -= mA * P.Y;
                aA -= iA * Vec2.Cross(rA, P);

                cB.X += mB * P.X;
                cB.Y += mB * P.Y;
                aB += iB * Vec2.Cross(rB, P);
            }
            else
            {
                C1.Set(cB).AddLocal(rB).SubLocal(cA).SubLocal(rA);
                float C2 = aB - aA - m_referenceAngle;

                positionError = C1.Length();
                angularError = MathUtils.Abs(C2);

                Vec3 C = pool.PopVec3();
                Vec3 impulse = pool.PopVec3();
                C.set_Renamed(C1.X, C1.Y, C2);

                K.Solve33ToOut(C, impulse);
                impulse.negateLocal();
                P.Set(impulse.x, impulse.y);

                cA.X -= mA * P.X;
                cA.Y -= mA * P.Y;
                aA -= iA * (Vec2.Cross(rA, P) + impulse.z);

                cB.X += mB * P.X;
                cB.Y += mB * P.Y;
                aB += iB * (Vec2.Cross(rB, P) + impulse.z);
            }

            data.Positions[m_indexA].c.Set(cA);
            data.Positions[m_indexA].a = aA;
            data.Positions[m_indexB].c.Set(cB);
            data.Positions[m_indexB].a = aB;

            pool.PushVec2(5);
            pool.PushRot(2);
            pool.PushMat33(1);

            return positionError <= Settings.LINEAR_SLOP && angularError <= Settings.ANGULAR_SLOP;
        }
    }
}