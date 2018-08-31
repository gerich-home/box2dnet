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

    //Point-to-point constraint
    //C = p2 - p1
    //Cdot = v2 - v1
    //   = v2 + cross(w2, r2) - v1 - cross(w1, r1)
    //J = [-I -r1_skew I r2_skew ]
    //Identity used:
    //w k % (rx i + ry j) = w * (-ry i + rx j)

    //Motor constraint
    //Cdot = w2 - w1
    //J = [0 0 -1 0 0 1]
    //K = invI1 + invI2

    /// <summary>
    /// A revolute joint constrains two bodies to share a common point while they are free to rotate
    /// about the point. The relative rotation about the shared point is the joint angle. You can limit
    /// the relative rotation with a joint limit that specifies a lower and upper angle. You can use a
    /// motor to drive the relative rotation about the shared point. A maximum motor torque is provided
    /// so that infinite forces are not generated.
    /// </summary>
    /// <author>Daniel Murphy</author>
    public class RevoluteJoint : Joint
    {
        // Solver shared
        public readonly Vec2 LocalAnchorA = new Vec2();
        public readonly Vec2 LocalAnchorB = new Vec2();
        public readonly Vec3 Impulse = new Vec3();
        public float MotorImpulse;

        private bool m_motorEnabled;
        private float m_maxMotorTorque;
        private float m_motorSpeed;

        private bool m_limitEnabled;
        public float ReferenceAngle;
        public float LowerAngle;
        public float UpperAngle;

        // Solver temp
        public int IndexA;
        public int IndexB;
        public readonly Vec2 RA = new Vec2();
        public readonly Vec2 RB = new Vec2();
        public readonly Vec2 LocalCenterA = new Vec2();
        public readonly Vec2 LocalCenterB = new Vec2();
        public float InvMassA;
        public float InvMassB;
        public float InvIA;
        public float InvIB;
        public readonly Mat33 Mass = new Mat33(); // effective mass for point-to-point constraint.
        public float MotorMass; // effective mass for motor/limit angular constraint.
        public LimitState LimitState;

        public RevoluteJoint(IWorldPool argWorld, RevoluteJointDef def)
            : base(argWorld, def)
        {
            LocalAnchorA.Set(def.LocalAnchorA);
            LocalAnchorB.Set(def.LocalAnchorB);
            ReferenceAngle = def.ReferenceAngle;

            MotorImpulse = 0;

            LowerAngle = def.LowerAngle;
            UpperAngle = def.UpperAngle;
            m_maxMotorTorque = def.MaxMotorTorque;
            m_motorSpeed = def.MotorSpeed;
            m_limitEnabled = def.EnableLimit;
            m_motorEnabled = def.EnableMotor;
            LimitState = LimitState.Inactive;
        }

        public override void InitVelocityConstraints(SolverData data)
        {
            IndexA = BodyA.IslandIndex;
            IndexB = BodyB.IslandIndex;
            LocalCenterA.Set(BodyA.Sweep.LocalCenter);
            LocalCenterB.Set(BodyB.Sweep.LocalCenter);
            InvMassA = BodyA.InvMass;
            InvMassB = BodyB.InvMass;
            InvIA = BodyA.InvI;
            InvIB = BodyB.InvI;

            // Vec2 cA = data.positions[m_indexA].c;
            float aA = data.Positions[IndexA].A;
            Vec2 vA = data.Velocities[IndexA].V;
            float wA = data.Velocities[IndexA].W;

            // Vec2 cB = data.positions[m_indexB].c;
            float aB = data.Positions[IndexB].A;
            Vec2 vB = data.Velocities[IndexB].V;
            float wB = data.Velocities[IndexB].W;

            Rot qA = Pool.PopRot();
            Rot qB = Pool.PopRot();
            Vec2 temp = Pool.PopVec2();

            qA.Set(aA);
            qB.Set(aB);

            // Compute the effective masses.
            Rot.MulToOutUnsafe(qA, temp.Set(LocalAnchorA).SubLocal(LocalCenterA), RA);
            Rot.MulToOutUnsafe(qB, temp.Set(LocalAnchorB).SubLocal(LocalCenterB), RB);

            // J = [-I -r1_skew I r2_skew]
            // [ 0 -1 0 1]
            // r_skew = [-ry; rx]

            // Matlab
            // K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x, -r1y*iA-r2y*iB]
            // [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
            // [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]

            float mA = InvMassA, mB = InvMassB;
            float iA = InvIA, iB = InvIB;

            bool fixedRotation = (iA + iB == 0.0f);

            Mass.Ex.X = mA + mB + RA.Y * RA.Y * iA + RB.Y * RB.Y * iB;
            Mass.Ey.X = (-RA.Y) * RA.X * iA - RB.Y * RB.X * iB;
            Mass.Ez.X = (-RA.Y) * iA - RB.Y * iB;
            Mass.Ex.Y = Mass.Ey.X;
            Mass.Ey.Y = mA + mB + RA.X * RA.X * iA + RB.X * RB.X * iB;
            Mass.Ez.Y = RA.X * iA + RB.X * iB;
            Mass.Ex.Z = Mass.Ez.X;
            Mass.Ey.Z = Mass.Ez.Y;
            Mass.Ez.Z = iA + iB;

            MotorMass = iA + iB;
            if (MotorMass > 0.0f)
            {
                MotorMass = 1.0f / MotorMass;
            }

            if (m_motorEnabled == false || fixedRotation)
            {
                MotorImpulse = 0.0f;
            }

            if (m_limitEnabled && fixedRotation == false)
            {
                float jointAngle = aB - aA - ReferenceAngle;
                if (MathUtils.Abs(UpperAngle - LowerAngle) < 2.0f * Settings.ANGULAR_SLOP)
                {
                    LimitState = LimitState.Equal;
                }
                else if (jointAngle <= LowerAngle)
                {
                    if (LimitState != LimitState.AtLower)
                    {
                        Impulse.Z = 0.0f;
                    }
                    LimitState = LimitState.AtLower;
                }
                else if (jointAngle >= UpperAngle)
                {
                    if (LimitState != LimitState.AtUpper)
                    {
                        Impulse.Z = 0.0f;
                    }
                    LimitState = LimitState.AtUpper;
                }
                else
                {
                    LimitState = LimitState.Inactive;
                    Impulse.Z = 0.0f;
                }
            }
            else
            {
                LimitState = LimitState.Inactive;
            }

            if (data.Step.WarmStarting)
            {
                Vec2 P = Pool.PopVec2();
                // Scale impulses to support a variable time step.
                Impulse.X *= data.Step.DtRatio;
                Impulse.Y *= data.Step.DtRatio;
                MotorImpulse *= data.Step.DtRatio;

                P.X = Impulse.X;
                P.Y = Impulse.Y;

                vA.X -= mA * P.X;
                vA.Y -= mA * P.Y;
                wA -= iA * (Vec2.Cross(RA, P) + MotorImpulse + Impulse.Z);

                vB.X += mB * P.X;
                vB.Y += mB * P.Y;
                wB += iB * (Vec2.Cross(RB, P) + MotorImpulse + Impulse.Z);
                Pool.PushVec2(1);
            }
            else
            {
                Impulse.SetZero();
                MotorImpulse = 0.0f;
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

            float mA = InvMassA, mB = InvMassB;
            float iA = InvIA, iB = InvIB;

            bool fixedRotation = (iA + iB == 0.0f);

            // Solve motor constraint.
            if (m_motorEnabled && LimitState != LimitState.Equal && fixedRotation == false)
            {
                float Cdot = wB - wA - m_motorSpeed;
                float impulse = (-MotorMass) * Cdot;
                float oldImpulse = MotorImpulse;
                float maxImpulse = data.Step.Dt * m_maxMotorTorque;
                MotorImpulse = MathUtils.Clamp(MotorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = MotorImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }
            Vec2 temp = Pool.PopVec2();

            // Solve limit constraint.
            if (m_limitEnabled && LimitState != LimitState.Inactive && fixedRotation == false)
            {
                Vec2 Cdot1 = Pool.PopVec2();
                Vec3 Cdot = Pool.PopVec3();

                // Solve point-to-point constraint
                Vec2.CrossToOutUnsafe(wA, RA, temp);
                Vec2.CrossToOutUnsafe(wB, RB, Cdot1);
                Cdot1.AddLocal(vB).SubLocal(vA).SubLocal(temp);
                float Cdot2 = wB - wA;
                Cdot.Set(Cdot1.X, Cdot1.Y, Cdot2);

                Vec3 impulse = Pool.PopVec3();
                Mass.Solve33ToOut(Cdot, impulse);
                impulse.NegateLocal();

                if (LimitState == LimitState.Equal)
                {
                    Impulse.AddLocal(impulse);
                }
                else if (LimitState == LimitState.AtLower)
                {
                    float newImpulse = Impulse.Z + impulse.Z;
                    if (newImpulse < 0.0f)
                    {
                        //UPGRADE_NOTE: Final was removed from the declaration of 'rhs '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
                        Vec2 rhs = Pool.PopVec2();
                        rhs.Set(Mass.Ez.X, Mass.Ez.Y).MulLocal(Impulse.Z).SubLocal(Cdot1);
                        Mass.Solve22ToOut(rhs, temp);
                        impulse.X = temp.X;
                        impulse.Y = temp.Y;
                        impulse.Z = -Impulse.Z;
                        Impulse.X += temp.X;
                        Impulse.Y += temp.Y;
                        Impulse.Z = 0.0f;
                        Pool.PushVec2(1);
                    }
                    else
                    {
                        Impulse.AddLocal(impulse);
                    }
                }
                else if (LimitState == LimitState.AtUpper)
                {
                    float newImpulse = Impulse.Z + impulse.Z;
                    if (newImpulse > 0.0f)
                    {
                        Vec2 rhs = Pool.PopVec2();
                        rhs.Set(Mass.Ez.X, Mass.Ez.Y).MulLocal(Impulse.Z).SubLocal(Cdot1);
                        Mass.Solve22ToOut(rhs, temp);
                        impulse.X = temp.X;
                        impulse.Y = temp.Y;
                        impulse.Z = -Impulse.Z;
                        Impulse.X += temp.X;
                        Impulse.Y += temp.Y;
                        Impulse.Z = 0.0f;
                        Pool.PushVec2(1);
                    }
                    else
                    {
                        Impulse.AddLocal(impulse);
                    }
                }
                Vec2 P = Pool.PopVec2();

                P.Set(impulse.X, impulse.Y);

                vA.X -= mA * P.X;
                vA.Y -= mA * P.Y;
                wA -= iA * (Vec2.Cross(RA, P) + impulse.Z);

                vB.X += mB * P.X;
                vB.Y += mB * P.Y;
                wB += iB * (Vec2.Cross(RB, P) + impulse.Z);

                Pool.PushVec2(2);
                Pool.PushVec3(2);
            }
            else
            {

                // Solve point-to-point constraint
                Vec2 Cdot = Pool.PopVec2();
                Vec2 impulse = Pool.PopVec2();

                Vec2.CrossToOutUnsafe(wA, RA, temp);
                Vec2.CrossToOutUnsafe(wB, RB, Cdot);
                Cdot.AddLocal(vB).SubLocal(vA).SubLocal(temp);
                Mass.Solve22ToOut(Cdot.NegateLocal(), impulse); // just leave negated

                Impulse.X += impulse.X;
                Impulse.Y += impulse.Y;

                vA.X -= mA * impulse.X;
                vA.Y -= mA * impulse.Y;
                wA -= iA * Vec2.Cross(RA, impulse);

                vB.X += mB * impulse.X;
                vB.Y += mB * impulse.Y;
                wB += iB * Vec2.Cross(RB, impulse);

                Pool.PushVec2(2);
            }

            data.Velocities[IndexA].V.Set(vA);
            data.Velocities[IndexA].W = wA;
            data.Velocities[IndexB].V.Set(vB);
            data.Velocities[IndexB].W = wB;

            Pool.PushVec2(1);
        }

        public override bool SolvePositionConstraints(SolverData data)
        {
            Rot qA = Pool.PopRot();
            Rot qB = Pool.PopRot();
            Vec2 cA = data.Positions[IndexA].C;
            float aA = data.Positions[IndexA].A;
            Vec2 cB = data.Positions[IndexB].C;
            float aB = data.Positions[IndexB].A;

            qA.Set(aA);
            qB.Set(aB);

            float angularError = 0.0f;
            float positionError = 0.0f;

            bool fixedRotation = (InvIA + InvIB == 0.0f);

            // Solve angular limit constraint.
            if (m_limitEnabled && LimitState != LimitState.Inactive && fixedRotation == false)
            {
                float angle = aB - aA - ReferenceAngle;
                float limitImpulse = 0.0f;

                if (LimitState == LimitState.Equal)
                {
                    // Prevent large angular corrections
                    float C = MathUtils.Clamp(angle - LowerAngle, -Settings.MAX_ANGULAR_CORRECTION, Settings.MAX_ANGULAR_CORRECTION);
                    limitImpulse = (-MotorMass) * C;
                    angularError = MathUtils.Abs(C);
                }
                else if (LimitState == LimitState.AtLower)
                {
                    float C = angle - LowerAngle;
                    angularError = -C;

                    // Prevent large angular corrections and allow some slop.
                    C = MathUtils.Clamp(C + Settings.ANGULAR_SLOP, -Settings.MAX_ANGULAR_CORRECTION, 0.0f);
                    limitImpulse = (-MotorMass) * C;
                }
                else if (LimitState == LimitState.AtUpper)
                {
                    float C = angle - UpperAngle;
                    angularError = C;

                    // Prevent large angular corrections and allow some slop.
                    C = MathUtils.Clamp(C - Settings.ANGULAR_SLOP, 0.0f, Settings.MAX_ANGULAR_CORRECTION);
                    limitImpulse = (-MotorMass) * C;
                }

                aA -= InvIA * limitImpulse;
                aB += InvIB * limitImpulse;
            }
            // Solve point-to-point constraint.
            {
                qA.Set(aA);
                qB.Set(aB);

                Vec2 rA = Pool.PopVec2();
                Vec2 rB = Pool.PopVec2();
                Vec2 C = Pool.PopVec2();
                Vec2 impulse = Pool.PopVec2();

                Rot.MulToOutUnsafe(qA, C.Set(LocalAnchorA).SubLocal(LocalCenterA), rA);
                Rot.MulToOutUnsafe(qB, C.Set(LocalAnchorB).SubLocal(LocalCenterB), rB);
                C.Set(cB).AddLocal(rB).SubLocal(cA).SubLocal(rA);
                positionError = C.Length();

                float mA = InvMassA, mB = InvMassB;
                float iA = InvIA, iB = InvIB;

                Mat22 K = Pool.PopMat22();
                K.Ex.X = mA + mB + iA * rA.Y * rA.Y + iB * rB.Y * rB.Y;
                K.Ex.Y = (-iA) * rA.X * rA.Y - iB * rB.X * rB.Y;
                K.Ey.X = K.Ex.Y;
                K.Ey.Y = mA + mB + iA * rA.X * rA.X + iB * rB.X * rB.X;

                K.SolveToOut(C, impulse);
                impulse.NegateLocal();

                cA.X -= mA * impulse.X;
                cA.Y -= mA * impulse.Y;
                aA -= iA * Vec2.Cross(rA, impulse);

                cB.X += mB * impulse.X;
                cB.Y += mB * impulse.Y;
                aB += iB * Vec2.Cross(rB, impulse);

                Pool.PushVec2(4);
                Pool.PushMat22(1);
            }
            data.Positions[IndexA].C.Set(cA);
            data.Positions[IndexA].A = aA;
            data.Positions[IndexB].C.Set(cB);
            data.Positions[IndexB].A = aB;

            Pool.PushRot(2);

            return positionError <= Settings.LINEAR_SLOP && angularError <= Settings.ANGULAR_SLOP;
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
            argOut.Set(Impulse.X, Impulse.Y).MulLocal(inv_dt);
        }

        public override float GetReactionTorque(float inv_dt)
        {
            return inv_dt * Impulse.Z;
        }

        public float JointAngle
        {
            get
            {
                Body b1 = BodyA;
                Body b2 = BodyB;
                return b2.Sweep.A - b1.Sweep.A - ReferenceAngle;
            }
        }

        public float JointSpeed
        {
            get
            {
                Body b1 = BodyA;
                Body b2 = BodyB;
                return b2.AngularVelocity - b1.AngularVelocity;
            }
        }

        public bool MotorEnabled
        {
            get
            {
                return m_motorEnabled;
            }
            set
            {
                BodyA.Awake = true;
                BodyB.Awake = true;
                m_motorEnabled = value;
            }
        }

        public float GetMotorTorque(float inv_dt)
        {
            return MotorImpulse * inv_dt;
        }

        public float MotorSpeed
        {
            get
            {
                return m_motorSpeed;
            }
            set
            {
                BodyA.Awake = true;
                BodyB.Awake = true;
                m_motorSpeed = value;
            }
        }

        public float MaxMotorTorque
        {
            get
            {
                return m_maxMotorTorque;
            }
            set
            {
                BodyA.Awake = true;
                BodyB.Awake = true;
                m_maxMotorTorque = value;
            }
        }

        public bool LimitEnabled
        {
            get
            {
                return m_limitEnabled;
            }
            set
            {
                if (value != m_limitEnabled)
                {
                    BodyA.Awake = true;
                    BodyB.Awake = true;
                    m_limitEnabled = value;
                    Impulse.Z = 0.0f;
                }
            }
        }

        public float LowerLimit
        {
            get
            {
                return LowerAngle;
            }
        }

        public float UpperLimit
        {
            get
            {
                return UpperAngle;
            }
        }

        public void setLimits(float lower, float upper)
        {
            Debug.Assert(lower <= upper);
            if (lower != LowerAngle || upper != UpperAngle)
            {
                BodyA.Awake = true;
                BodyB.Awake = true;
                Impulse.Z = 0.0f;
                LowerAngle = lower;
                UpperAngle = upper;
            }
        }
    }
}