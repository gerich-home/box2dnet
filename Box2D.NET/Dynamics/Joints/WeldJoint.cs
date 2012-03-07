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
        public float Frequency;
        public float DampingRatio;
        private float m_bias;

        // Solver shared
        public readonly Vec2 LocalAnchorA;
        public readonly Vec2 LocalAnchorB;
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
            LocalAnchorA = new Vec2(def.LocalAnchorA);
            LocalAnchorB = new Vec2(def.LocalAnchorB);
            m_referenceAngle = def.ReferenceAngle;
            Frequency = def.FrequencyHz;
            DampingRatio = def.DampingRatio;

            m_impulse = new Vec3();
            m_impulse.SetZero();
        }

        public override void GetAnchorA(Vec2 argOut)
        {
            BodyA.GetWorldPointToOut(LocalAnchorA, argOut);
        }

        public override void GetAnchorB(Vec2 argOut)
        {
            BodyB.GetWorldPointToOut(LocalAnchorB, argOut);
        }

        /// <seealso cref="Joint.GetReactionForce(float, Vec2)"></seealso>
        public override void GetReactionForce(float inv_dt, Vec2 argOut)
        {
            argOut.Set(m_impulse.X, m_impulse.Y);
            argOut.MulLocal(inv_dt);
        }

        /// <seealso cref="Joint.GetReactionTorque(float)"></seealso>
        public override float GetReactionTorque(float inv_dt)
        {
            return inv_dt * m_impulse.Z;
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

            //    Vec2 cA = data.positions[m_indexA].c;
            float aA = data.Positions[m_indexA].A;
            Vec2 vA = data.Velocities[m_indexA].V;
            float wA = data.Velocities[m_indexA].W;

            //    Vec2 cB = data.positions[m_indexB].c;
            float aB = data.Positions[m_indexB].A;
            Vec2 vB = data.Velocities[m_indexB].V;
            float wB = data.Velocities[m_indexB].W;

            Rot qA = Pool.PopRot();
            Rot qB = Pool.PopRot();
            Vec2 temp = Pool.PopVec2();

            qA.Set(aA);
            qB.Set(aB);

            // Compute the effective masses.
            Rot.MulToOutUnsafe(qA, temp.Set(LocalAnchorA).SubLocal(m_localCenterA), m_rA);
            Rot.MulToOutUnsafe(qB, temp.Set(LocalAnchorB).SubLocal(m_localCenterB), m_rB);

            // J = [-I -r1_skew I r2_skew]
            // [ 0 -1 0 1]
            // r_skew = [-ry; rx]

            // Matlab
            // K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x, -r1y*iA-r2y*iB]
            // [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
            // [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            Mat33 K = Pool.PopMat33();

            K.Ex.X = mA + mB + m_rA.Y * m_rA.Y * iA + m_rB.Y * m_rB.Y * iB;
            K.Ey.X = (-m_rA.Y) * m_rA.X * iA - m_rB.Y * m_rB.X * iB;
            K.Ez.X = (-m_rA.Y) * iA - m_rB.Y * iB;
            K.Ex.Y = K.Ey.X;
            K.Ey.Y = mA + mB + m_rA.X * m_rA.X * iA + m_rB.X * m_rB.X * iB;
            K.Ez.Y = m_rA.X * iA + m_rB.X * iB;
            K.Ex.Z = K.Ez.X;
            K.Ey.Z = K.Ez.Y;
            K.Ez.Z = iA + iB;

            if (Frequency > 0.0f)
            {
                K.GetInverse22(m_mass);

                float invM = iA + iB;
                float m = invM > 0.0f ? 1.0f / invM : 0.0f;

                float C = aB - aA - m_referenceAngle;

                // Frequency
                float omega = 2.0f * MathUtils.PI * Frequency;

                // Damping coefficient
                float d = 2.0f * m * DampingRatio * omega;

                // Spring stiffness
                float k = m * omega * omega;

                // magic formulas
                float h = data.Step.Dt;
                m_gamma = h * (d + h * k);
                m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
                m_bias = C * h * k * m_gamma;

                invM += m_gamma;
                m_mass.Ez.Z = invM != 0.0f ? 1.0f / invM : 0.0f;
            }
            else
            {
                K.GetSymInverse33(m_mass);
                m_gamma = 0.0f;
                m_bias = 0.0f;
            }

            if (data.Step.WarmStarting)
            {
                Vec2 P = Pool.PopVec2();
                // Scale impulses to support a variable time step.
                m_impulse.MulLocal(data.Step.DtRatio);

                P.Set(m_impulse.X, m_impulse.Y);

                vA.X -= mA * P.X;
                vA.Y -= mA * P.Y;
                wA -= iA * (Vec2.Cross(m_rA, P) + m_impulse.Z);

                vB.X += mB * P.X;
                vB.Y += mB * P.Y;
                wB += iB * (Vec2.Cross(m_rB, P) + m_impulse.Z);
                Pool.PushVec2(1);
            }
            else
            {
                m_impulse.SetZero();
            }

            data.Velocities[m_indexA].V.Set(vA);
            data.Velocities[m_indexA].W = wA;
            data.Velocities[m_indexB].V.Set(vB);
            data.Velocities[m_indexB].W = wB;

            Pool.PushVec2(1);
            Pool.PushRot(2);
            Pool.PushMat33(1);
        }

        /// <seealso cref="Joint.solveVelocityConstraints(TimeStep)"></seealso>
        public override void SolveVelocityConstraints(SolverData data)
        {
            Vec2 vA = data.Velocities[m_indexA].V;
            float wA = data.Velocities[m_indexA].W;
            Vec2 vB = data.Velocities[m_indexB].V;
            float wB = data.Velocities[m_indexB].W;

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            Vec2 Cdot1 = Pool.PopVec2();
            Vec2 P = Pool.PopVec2();
            Vec2 temp = Pool.PopVec2();

            if (Frequency > 0.0f)
            {
                float Cdot2 = wB - wA;

                float impulse2 = (-m_mass.Ez.Z) * (Cdot2 + m_bias + m_gamma * m_impulse.Z);
                m_impulse.Z += impulse2;

                wA -= iA * impulse2;
                wB += iB * impulse2;

                Vec2.CrossToOutUnsafe(wB, m_rB, Cdot1);
                Vec2.CrossToOutUnsafe(wA, m_rA, temp);
                Cdot1.AddLocal(vB).SubLocal(vA).SubLocal(temp);

                Vec2 impulse1 = P;
                Mat33.Mul22ToOutUnsafe(m_mass, Cdot1, impulse1);
                impulse1.NegateLocal();

                m_impulse.X += impulse1.X;
                m_impulse.Y += impulse1.Y;

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

                Vec3 Cdot = Pool.PopVec3();
                Cdot.Set(Cdot1.X, Cdot1.Y, Cdot2);

                Vec3 impulse = Pool.PopVec3();
                Mat33.MulToOutUnsafe(m_mass, Cdot, impulse);
                impulse.NegateLocal();
                m_impulse.AddLocal(impulse);

                P.Set(impulse.X, impulse.Y);

                vA.X -= mA * P.X;
                vA.Y -= mA * P.Y;
                wA -= iA * (Vec2.Cross(m_rA, P) + impulse.Z);

                vB.X += mB * P.X;
                vB.Y += mB * P.Y;
                wB += iB * (Vec2.Cross(m_rB, P) + impulse.Z);

                Pool.PushVec3(2);
            }

            data.Velocities[m_indexA].V.Set(vA);
            data.Velocities[m_indexA].W = wA;
            data.Velocities[m_indexB].V.Set(vB);
            data.Velocities[m_indexB].W = wB;

            Pool.PushVec2(3);
        }

        /// <seealso cref="Joint.solvePositionConstraints(float)"></seealso>
        public override bool SolvePositionConstraints(SolverData data)
        {
            Vec2 cA = data.Positions[m_indexA].C;
            float aA = data.Positions[m_indexA].A;
            Vec2 cB = data.Positions[m_indexB].C;
            float aB = data.Positions[m_indexB].A;
            Rot qA = Pool.PopRot();
            Rot qB = Pool.PopRot();
            Vec2 temp = Pool.PopVec2();
            Vec2 rA = Pool.PopVec2();
            Vec2 rB = Pool.PopVec2();

            qA.Set(aA);
            qB.Set(aB);

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            Rot.MulToOutUnsafe(qA, temp.Set(LocalAnchorA).SubLocal(m_localCenterA), rA);
            Rot.MulToOutUnsafe(qB, temp.Set(LocalAnchorB).SubLocal(m_localCenterB), rB);
            float positionError, angularError;

            Mat33 K = Pool.PopMat33();
            Vec2 C1 = Pool.PopVec2();
            Vec2 P = Pool.PopVec2();

            K.Ex.X = mA + mB + rA.Y * rA.Y * iA + rB.Y * rB.Y * iB;
            K.Ey.X = (-rA.Y) * rA.X * iA - rB.Y * rB.X * iB;
            K.Ez.X = (-rA.Y) * iA - rB.Y * iB;
            K.Ex.Y = K.Ey.X;
            K.Ey.Y = mA + mB + rA.X * rA.X * iA + rB.X * rB.X * iB;
            K.Ez.Y = rA.X * iA + rB.X * iB;
            K.Ex.Z = K.Ez.X;
            K.Ey.Z = K.Ez.Y;
            K.Ez.Z = iA + iB;

            if (Frequency > 0.0f)
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

                Vec3 C = Pool.PopVec3();
                Vec3 impulse = Pool.PopVec3();
                C.Set(C1.X, C1.Y, C2);

                K.Solve33ToOut(C, impulse);
                impulse.NegateLocal();
                P.Set(impulse.X, impulse.Y);

                cA.X -= mA * P.X;
                cA.Y -= mA * P.Y;
                aA -= iA * (Vec2.Cross(rA, P) + impulse.Z);

                cB.X += mB * P.X;
                cB.Y += mB * P.Y;
                aB += iB * (Vec2.Cross(rB, P) + impulse.Z);
            }

            data.Positions[m_indexA].C.Set(cA);
            data.Positions[m_indexA].A = aA;
            data.Positions[m_indexB].C.Set(cB);
            data.Positions[m_indexB].A = aB;

            Pool.PushVec2(5);
            Pool.PushRot(2);
            Pool.PushMat33(1);

            return positionError <= Settings.LINEAR_SLOP && angularError <= Settings.ANGULAR_SLOP;
        }
    }
}