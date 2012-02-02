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
        public readonly Vec2 m_localAnchorA = new Vec2();
        public readonly Vec2 m_localAnchorB = new Vec2();
        public readonly Vec3 m_impulse = new Vec3();
        public float m_motorImpulse;

        public bool m_enableMotor;
        public float m_maxMotorTorque;
        public float m_motorSpeed;

        public bool m_enableLimit;
        public float m_referenceAngle;
        public float m_lowerAngle;
        public float m_upperAngle;

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
        public readonly Mat33 m_mass = new Mat33(); // effective mass for point-to-point constraint.
        public float m_motorMass; // effective mass for motor/limit angular constraint.
        public LimitState m_limitState;

        public RevoluteJoint(IWorldPool argWorld, RevoluteJointDef def)
            : base(argWorld, def)
        {
            m_localAnchorA.Set(def.localAnchorA);
            m_localAnchorB.Set(def.localAnchorB);
            m_referenceAngle = def.referenceAngle;

            m_motorImpulse = 0;

            m_lowerAngle = def.lowerAngle;
            m_upperAngle = def.upperAngle;
            m_maxMotorTorque = def.maxMotorTorque;
            m_motorSpeed = def.motorSpeed;
            m_enableLimit = def.enableLimit;
            m_enableMotor = def.enableMotor;
            m_limitState = LimitState.INACTIVE;
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

            // Vec2 cA = data.positions[m_indexA].c;
            float aA = data.Positions[m_indexA].a;
            Vec2 vA = data.Velocities[m_indexA].v;
            float wA = data.Velocities[m_indexA].w;

            // Vec2 cB = data.positions[m_indexB].c;
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

            // J = [-I -r1_skew I r2_skew]
            // [ 0 -1 0 1]
            // r_skew = [-ry; rx]

            // Matlab
            // K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x, -r1y*iA-r2y*iB]
            // [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
            // [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            bool fixedRotation = (iA + iB == 0.0f);

            m_mass.ex.x = mA + mB + m_rA.Y * m_rA.Y * iA + m_rB.Y * m_rB.Y * iB;
            m_mass.ey.x = (-m_rA.Y) * m_rA.X * iA - m_rB.Y * m_rB.X * iB;
            m_mass.ez.x = (-m_rA.Y) * iA - m_rB.Y * iB;
            m_mass.ex.y = m_mass.ey.x;
            m_mass.ey.y = mA + mB + m_rA.X * m_rA.X * iA + m_rB.X * m_rB.X * iB;
            m_mass.ez.y = m_rA.X * iA + m_rB.X * iB;
            m_mass.ex.z = m_mass.ez.x;
            m_mass.ey.z = m_mass.ez.y;
            m_mass.ez.z = iA + iB;

            m_motorMass = iA + iB;
            if (m_motorMass > 0.0f)
            {
                m_motorMass = 1.0f / m_motorMass;
            }

            if (m_enableMotor == false || fixedRotation)
            {
                m_motorImpulse = 0.0f;
            }

            if (m_enableLimit && fixedRotation == false)
            {
                float jointAngle = aB - aA - m_referenceAngle;
                if (MathUtils.abs(m_upperAngle - m_lowerAngle) < 2.0f * Settings.angularSlop)
                {
                    m_limitState = LimitState.EQUAL;
                }
                else if (jointAngle <= m_lowerAngle)
                {
                    if (m_limitState != LimitState.AT_LOWER)
                    {
                        m_impulse.z = 0.0f;
                    }
                    m_limitState = LimitState.AT_LOWER;
                }
                else if (jointAngle >= m_upperAngle)
                {
                    if (m_limitState != LimitState.AT_UPPER)
                    {
                        m_impulse.z = 0.0f;
                    }
                    m_limitState = LimitState.AT_UPPER;
                }
                else
                {
                    m_limitState = LimitState.INACTIVE;
                    m_impulse.z = 0.0f;
                }
            }
            else
            {
                m_limitState = LimitState.INACTIVE;
            }

            if (data.Step.WarmStarting)
            {
                Vec2 P = pool.PopVec2();
                // Scale impulses to support a variable time step.
                m_impulse.x *= data.Step.DtRatio;
                m_impulse.y *= data.Step.DtRatio;
                m_motorImpulse *= data.Step.DtRatio;

                P.X = m_impulse.x;
                P.Y = m_impulse.y;

                vA.X -= mA * P.X;
                vA.Y -= mA * P.Y;
                wA -= iA * (Vec2.Cross(m_rA, P) + m_motorImpulse + m_impulse.z);

                vB.X += mB * P.X;
                vB.Y += mB * P.Y;
                wB += iB * (Vec2.Cross(m_rB, P) + m_motorImpulse + m_impulse.z);
                pool.PushVec2(1);
            }
            else
            {
                m_impulse.setZero();
                m_motorImpulse = 0.0f;
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

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            bool fixedRotation = (iA + iB == 0.0f);

            // Solve motor constraint.
            if (m_enableMotor && m_limitState != LimitState.EQUAL && fixedRotation == false)
            {
                float Cdot = wB - wA - m_motorSpeed;
                float impulse = (-m_motorMass) * Cdot;
                float oldImpulse = m_motorImpulse;
                float maxImpulse = data.Step.Dt * m_maxMotorTorque;
                m_motorImpulse = MathUtils.clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = m_motorImpulse - oldImpulse;

                wA -= iA * impulse;
                wB += iB * impulse;
            }
            Vec2 temp = pool.PopVec2();

            // Solve limit constraint.
            if (m_enableLimit && m_limitState != LimitState.INACTIVE && fixedRotation == false)
            {
                Vec2 Cdot1 = pool.PopVec2();
                Vec3 Cdot = pool.PopVec3();

                // Solve point-to-point constraint
                Vec2.CrossToOutUnsafe(wA, m_rA, temp);
                Vec2.CrossToOutUnsafe(wB, m_rB, Cdot1);
                Cdot1.AddLocal(vB).SubLocal(vA).SubLocal(temp);
                float Cdot2 = wB - wA;
                Cdot.set_Renamed(Cdot1.X, Cdot1.Y, Cdot2);

                Vec3 impulse = pool.PopVec3();
                m_mass.solve33ToOut(Cdot, impulse);
                impulse.negateLocal();

                if (m_limitState == LimitState.EQUAL)
                {
                    m_impulse.addLocal(impulse);
                }
                else if (m_limitState == LimitState.AT_LOWER)
                {
                    float newImpulse = m_impulse.z + impulse.z;
                    if (newImpulse < 0.0f)
                    {
                        //UPGRADE_NOTE: Final was removed from the declaration of 'rhs '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
                        Vec2 rhs = pool.PopVec2();
                        rhs.Set(m_mass.ez.x, m_mass.ez.y).MulLocal(m_impulse.z).SubLocal(Cdot1);
                        m_mass.solve22ToOut(rhs, temp);
                        impulse.x = temp.X;
                        impulse.y = temp.Y;
                        impulse.z = -m_impulse.z;
                        m_impulse.x += temp.X;
                        m_impulse.y += temp.Y;
                        m_impulse.z = 0.0f;
                        pool.PushVec2(1);
                    }
                    else
                    {
                        m_impulse.addLocal(impulse);
                    }
                }
                else if (m_limitState == LimitState.AT_UPPER)
                {
                    float newImpulse = m_impulse.z + impulse.z;
                    if (newImpulse > 0.0f)
                    {
                        Vec2 rhs = pool.PopVec2();
                        rhs.Set(m_mass.ez.x, m_mass.ez.y).MulLocal(m_impulse.z).SubLocal(Cdot1);
                        m_mass.solve22ToOut(rhs, temp);
                        impulse.x = temp.X;
                        impulse.y = temp.Y;
                        impulse.z = -m_impulse.z;
                        m_impulse.x += temp.X;
                        m_impulse.y += temp.Y;
                        m_impulse.z = 0.0f;
                        pool.PushVec2(1);
                    }
                    else
                    {
                        m_impulse.addLocal(impulse);
                    }
                }
                Vec2 P = pool.PopVec2();

                P.Set(impulse.x, impulse.y);

                vA.X -= mA * P.X;
                vA.Y -= mA * P.Y;
                wA -= iA * (Vec2.Cross(m_rA, P) + impulse.z);

                vB.X += mB * P.X;
                vB.Y += mB * P.Y;
                wB += iB * (Vec2.Cross(m_rB, P) + impulse.z);

                pool.PushVec2(2);
                pool.PushVec3(2);
            }
            else
            {

                // Solve point-to-point constraint
                Vec2 Cdot = pool.PopVec2();
                Vec2 impulse = pool.PopVec2();

                Vec2.CrossToOutUnsafe(wA, m_rA, temp);
                Vec2.CrossToOutUnsafe(wB, m_rB, Cdot);
                Cdot.AddLocal(vB).SubLocal(vA).SubLocal(temp);
                m_mass.solve22ToOut(Cdot.NegateLocal(), impulse); // just leave negated

                m_impulse.x += impulse.X;
                m_impulse.y += impulse.Y;

                vA.X -= mA * impulse.X;
                vA.Y -= mA * impulse.Y;
                wA -= iA * Vec2.Cross(m_rA, impulse);

                vB.X += mB * impulse.X;
                vB.Y += mB * impulse.Y;
                wB += iB * Vec2.Cross(m_rB, impulse);

                pool.PushVec2(2);
            }

            data.Velocities[m_indexA].v.Set(vA);
            data.Velocities[m_indexA].w = wA;
            data.Velocities[m_indexB].v.Set(vB);
            data.Velocities[m_indexB].w = wB;

            pool.PushVec2(1);
        }

        public override bool solvePositionConstraints(SolverData data)
        {
            Rot qA = pool.PopRot();
            Rot qB = pool.PopRot();
            Vec2 cA = data.Positions[m_indexA].c;
            float aA = data.Positions[m_indexA].a;
            Vec2 cB = data.Positions[m_indexB].c;
            float aB = data.Positions[m_indexB].a;

            qA.set_Renamed(aA);
            qB.set_Renamed(aB);

            float angularError = 0.0f;
            float positionError = 0.0f;

            bool fixedRotation = (m_invIA + m_invIB == 0.0f);

            // Solve angular limit constraint.
            if (m_enableLimit && m_limitState != LimitState.INACTIVE && fixedRotation == false)
            {
                float angle = aB - aA - m_referenceAngle;
                float limitImpulse = 0.0f;

                if (m_limitState == LimitState.EQUAL)
                {
                    // Prevent large angular corrections
                    float C = MathUtils.clamp(angle - m_lowerAngle, -Settings.maxAngularCorrection, Settings.maxAngularCorrection);
                    limitImpulse = (-m_motorMass) * C;
                    angularError = MathUtils.abs(C);
                }
                else if (m_limitState == LimitState.AT_LOWER)
                {
                    float C = angle - m_lowerAngle;
                    angularError = -C;

                    // Prevent large angular corrections and allow some slop.
                    C = MathUtils.clamp(C + Settings.angularSlop, -Settings.maxAngularCorrection, 0.0f);
                    limitImpulse = (-m_motorMass) * C;
                }
                else if (m_limitState == LimitState.AT_UPPER)
                {
                    float C = angle - m_upperAngle;
                    angularError = C;

                    // Prevent large angular corrections and allow some slop.
                    C = MathUtils.clamp(C - Settings.angularSlop, 0.0f, Settings.maxAngularCorrection);
                    limitImpulse = (-m_motorMass) * C;
                }

                aA -= m_invIA * limitImpulse;
                aB += m_invIB * limitImpulse;
            }
            // Solve point-to-point constraint.
            {
                qA.set_Renamed(aA);
                qB.set_Renamed(aB);

                Vec2 rA = pool.PopVec2();
                Vec2 rB = pool.PopVec2();
                Vec2 C = pool.PopVec2();
                Vec2 impulse = pool.PopVec2();

                Rot.mulToOutUnsafe(qA, C.Set(m_localAnchorA).SubLocal(m_localCenterA), rA);
                Rot.mulToOutUnsafe(qB, C.Set(m_localAnchorB).SubLocal(m_localCenterB), rB);
                C.Set(cB).AddLocal(rB).SubLocal(cA).SubLocal(rA);
                positionError = C.Length();

                float mA = m_invMassA, mB = m_invMassB;
                float iA = m_invIA, iB = m_invIB;

                Mat22 K = pool.PopMat22();
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

                pool.PushVec2(4);
                pool.PushMat22(1);
            }
            data.Positions[m_indexA].c.Set(cA);
            data.Positions[m_indexA].a = aA;
            data.Positions[m_indexB].c.Set(cB);
            data.Positions[m_indexB].a = aB;

            pool.PushRot(2);

            return positionError <= Settings.linearSlop && angularError <= Settings.angularSlop;
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
            argOut.Set(m_impulse.x, m_impulse.y).MulLocal(inv_dt);
        }

        public override float getReactionTorque(float inv_dt)
        {
            return inv_dt * m_impulse.z;
        }

        virtual public float JointAngle
        {
            get
            {
                Body b1 = m_bodyA;
                Body b2 = m_bodyB;
                return b2.Sweep.a - b1.Sweep.a - m_referenceAngle;
            }
        }

        virtual public float JointSpeed
        {
            get
            {
                Body b1 = m_bodyA;
                Body b2 = m_bodyB;
                return b2.m_angularVelocity - b1.m_angularVelocity;
            }
        }

        virtual public bool MotorEnabled
        {
            get
            {
                return m_enableMotor;
            }
        }

        public virtual void enableMotor(bool flag)
        {
            m_bodyA.Awake = true;
            m_bodyB.Awake = true;
            m_enableMotor = flag;
        }

        public virtual float getMotorTorque(float inv_dt)
        {
            return m_motorImpulse * inv_dt;
        }

        virtual public float MotorSpeed
        {
            set
            {
                m_bodyA.Awake = true;
                m_bodyB.Awake = true;
                m_motorSpeed = value;
            }
        }

        virtual public float MaxMotorTorque
        {
            set
            {
                m_bodyA.Awake = true;
                m_bodyB.Awake = true;
                m_maxMotorTorque = value;
            }
        }

        virtual public bool LimitEnabled
        {
            get
            {
                return m_enableLimit;
            }
        }

        public virtual void enableLimit(bool flag)
        {
            if (flag != m_enableLimit)
            {
                m_bodyA.Awake = true;
                m_bodyB.Awake = true;
                m_enableLimit = flag;
                m_impulse.z = 0.0f;
            }
        }

        virtual public float LowerLimit
        {
            get
            {
                return m_lowerAngle;
            }
        }

        virtual public float UpperLimit
        {
            get
            {
                return m_upperAngle;
            }
        }

        public virtual void setLimits(float lower, float upper)
        {
            Debug.Assert(lower <= upper);
            if (lower != m_lowerAngle || upper != m_upperAngle)
            {
                m_bodyA.Awake = true;
                m_bodyB.Awake = true;
                m_impulse.z = 0.0f;
                m_lowerAngle = lower;
                m_upperAngle = upper;
            }
        }
    }
}