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

// Created at 9:06:02 PM Jan 21, 2011

//using System;
//using Mat22 = org.jbox2d.common.Mat22;
//using MathUtils = org.jbox2d.common.MathUtils;
//using Rot = org.jbox2d.common.Rot;
//using Settings = org.jbox2d.common.Settings;
//using Transform = org.jbox2d.common.Transform;
//using Vec2 = org.jbox2d.common.Vec2;
//using Body = org.jbox2d.dynamics.Body;
//using TimeStep = org.jbox2d.dynamics.TimeStep;
//using IWorldPool = org.jbox2d.pooling.IWorldPool;
//using System.Diagnostics;

//namespace org.jbox2d.dynamics.joints
//{

//    /// <author>Daniel Murphy</author>
//    public class WheelJoint : Joint
//    {
//        public readonly Vec2 m_localAnchor1 = new Vec2();
//        public readonly Vec2 m_localAnchor2 = new Vec2();
//        public readonly Vec2 m_localXAxis1 = new Vec2();
//        private readonly Vec2 m_localYAxis1 = new Vec2();

//        private readonly Vec2 m_axis = new Vec2();
//        private readonly Vec2 m_perp = new Vec2();
//        private float m_s1, m_s2;
//        private float m_a1, m_a2;

//        private readonly Mat22 m_K = new Mat22();
//        private readonly Vec2 m_impulse = new Vec2();

//        private float m_motorMass; // effective mass for motor/limit translational
//        // constraint.
//        private float m_motorImpulse;

//        private float m_lowerTranslation;
//        private float m_upperTranslation;
//        private float m_maxMotorForce;
//        private float m_motorSpeed;

//        private bool m_enableLimit;
//        private bool m_enableMotor;
//        private LimitState m_limitState;

//        public WheelJoint(IWorldPool argPool, LineJointDef def) :
//            base(argPool, def)
//        {
//            m_localAnchor1.set_Renamed(def.localAnchorA);
//            m_localAnchor2.set_Renamed(def.localAnchorB);
//            m_localXAxis1.set_Renamed(def.localAxisA);
//            Vec2.crossToOutUnsafe(1.0f, m_localXAxis1, m_localYAxis1);

//            m_impulse.setZero();
//            m_motorMass = 0.0f;
//            m_motorImpulse = 0.0f;

//            m_lowerTranslation = def.lowerTranslation;
//            m_upperTranslation = def.upperTranslation;
//            m_maxMotorForce = def.maxMotorForce;
//            m_motorSpeed = def.motorSpeed;
//            m_enableLimit = def.enableLimit;
//            m_enableMotor = def.enableMotor;
//            m_limitState = LimitState.INACTIVE;

//            m_axis.setZero();
//            m_perp.setZero();
//        }

//        /// <seealso cref="org.jbox2d.dynamics.joints.Joint.getAnchorA(org.jbox2d.common.Vec2)"></seealso>
//        public override void getAnchorA(Vec2 argOut)
//        {
//            m_bodyA.getWorldPointToOut(m_localAnchor1, argOut);
//        }

//        /// <seealso cref="org.jbox2d.dynamics.joints.Joint.getAnchorB(org.jbox2d.common.Vec2)"></seealso>
//        public override void getAnchorB(Vec2 argOut)
//        {
//            m_bodyB.getWorldPointToOut(m_localAnchor2, argOut);
//        }

//        /// <seealso cref="org.jbox2d.dynamics.joints.Joint.getReactionForce(float, org.jbox2d.common.Vec2)"></seealso>
//        public override void getReactionForce(float inv_dt, Vec2 argOut)
//        {
//            Vec2 temp = pool.popVec2();
//            temp.set_Renamed(m_perp).mulLocal(m_impulse.x);
//            argOut.set_Renamed(m_axis).mulLocal(m_motorImpulse + m_impulse.y).addLocal(temp).mulLocal(inv_dt);
//            pool.pushVec2(1);
//        }

//        /// <seealso cref="org.jbox2d.dynamics.joints.Joint.getReactionTorque(float)"></seealso>
//        public override float getReactionTorque(float inv_dt)
//        {
//            return 0.0f;
//        }

//        public float JointTranslation
//        {
//            get
//            {
//                Body b1 = m_bodyA;
//                Body b2 = m_bodyB;

//                Vec2 p1 = pool.popVec2();
//                Vec2 p2 = pool.popVec2();
//                Vec2 axis = pool.popVec2();
//                b1.getWorldPointToOut(m_localAnchor1, p1);
//                b2.getWorldPointToOut(m_localAnchor1, p2);
//                p2.subLocal(p1);
//                b1.getWorldVectorToOut(m_localXAxis1, axis);

//                float translation = Vec2.dot(p2, axis);
//                pool.pushVec2(3);
//                return translation;
//            }
//        }

//        public float JointSpeed
//        {
//            get
//            {
//                Body b1 = m_bodyA;
//                Body b2 = m_bodyB;

//                Vec2 r1 = pool.popVec2();
//                Vec2 r2 = pool.popVec2();
//                Vec2 p1 = pool.popVec2();
//                Vec2 p2 = pool.popVec2();

//                r1.set_Renamed(m_localAnchor1).subLocal(b1.LocalCenter);
//                r2.set_Renamed(m_localAnchor2).subLocal(b2.LocalCenter);
//                Rot.mulToOut(b1.getTransform().q, r1, r1);
//                Rot.mulToOut(b2.getTransform().q, r2, r2);

//                p1.set_Renamed(b1.m_sweep.c).addLocal(r1);
//                p2.set_Renamed(b2.m_sweep.c).addLocal(r2);
//                p2.subLocal(p1);

//                Vec2 axis = pool.popVec2();
//                b1.getWorldPointToOut(m_localXAxis1, axis);

//                Vec2 v1 = b1.m_linearVelocity;
//                Vec2 v2 = b2.m_linearVelocity;
//                float w1 = b1.m_angularVelocity;
//                float w2 = b2.m_angularVelocity;

//                Vec2 temp1 = pool.popVec2();
//                Vec2 temp2 = pool.popVec2();

//                Vec2.crossToOutUnsafe(w1, r1, temp1);
//                Vec2.crossToOutUnsafe(w2, r2, temp2);
//                temp2.addLocal(v2).subLocal(v1).subLocal(temp1);
//                float s2 = Vec2.dot(axis, temp2);

//                Vec2.crossToOutUnsafe(w1, axis, temp1);
//                float speed = Vec2.dot(p2, temp1) + s2;

//                pool.pushVec2(7);
//                return speed;
//            }
//        }

//        public bool LimitEnabled
//        {
//            get
//            {
//                return m_enableLimit;
//            }
//        }

//        public float LowerLimit
//        {
//            get
//            {
//                return m_lowerTranslation;
//            }
//        }

//        public float UpperLimit
//        {
//            get
//            {
//                return m_upperTranslation;
//            }
//        }

//        public bool MotorEnabled
//        {
//            get
//            {
//                return m_enableMotor;
//            }
//        }

//        public float MotorSpeed
//        {
//            get
//            {
//                return m_motorSpeed;
//            }
//            set
//            {
//                m_bodyA.Awake = true;
//                m_bodyB.Awake = true;
//                m_motorSpeed = value;
//            }
//        }

//        public float MaxMotorForce
//        {
//            get
//            {
//                return m_maxMotorForce;
//            }
//            set
//            {
//                m_bodyA.Awake = true;
//                m_bodyB.Awake = true;
//                m_maxMotorForce = value;
//            }
//        }

//        public float MotorForce
//        {
//            get
//            {
//                return m_motorImpulse;
//            }
//        }

//        public void EnableLimit(bool flag)
//        {
//            m_bodyA.Awake = true;
//            m_bodyB.Awake = true;
//            m_enableLimit = flag;
//        }

//        public void setLimits(float lower, float upper)
//        {
//            Debug.Assert(lower <= upper);
//            m_bodyA.Awake = true;
//            m_bodyB.Awake = true;
//            m_lowerTranslation = lower;
//            m_upperTranslation = upper;
//        }

//        public void EnableMotor(bool flag)
//        {
//            m_bodyA.Awake = true;
//            m_bodyB.Awake = true;
//            m_enableMotor = flag;
//        }

//        /// <seealso cref="org.jbox2d.dynamics.joints.Joint.initVelocityConstraints(org.jbox2d.dynamics.TimeStep)"></seealso>
//        public void initVelocityConstraints(TimeStep step)
//        {
//            Body b1 = m_bodyA;
//            Body b2 = m_bodyB;

//            m_localCenterA.set_Renamed(b1.LocalCenter);
//            m_localCenterB.set_Renamed(b2.LocalCenter);

//            Transform xf1 = b1.getTransform();
//            Transform xf2 = b2.getTransform();

//            // Compute the effective masses.
//            Vec2 r1 = pool.popVec2();
//            Vec2 r2 = pool.popVec2();
//            Vec2 temp = pool.popVec2();

//            r1.set_Renamed(m_localAnchor1).subLocal(m_localCenterA);
//            r2.set_Renamed(m_localAnchor2).subLocal(m_localCenterB);
//            Rot.mulToOut(xf1.q, r1, r1);
//            Rot.mulToOut(xf2.q, r2, r2);

//            Vec2 d = pool.popVec2();
//            d.set_Renamed(b2.m_sweep.c).addLocal(r2).subLocal(b1.m_sweep.c).subLocal(r1);

//            m_invMassA = b1.m_invMass;
//            m_invIA = b1.m_invI;
//            m_invMassB = b2.m_invMass;
//            m_invIB = b2.m_invI;

//            // Compute motor Jacobian and effective mass.
//            {
//                Rot.mulToOutUnsafe(xf1.q, m_localXAxis1, m_axis);
//                temp.set_Renamed(d).addLocal(r1);
//                m_a1 = Vec2.cross(temp, m_axis);
//                m_a2 = Vec2.cross(r2, m_axis);

//                m_motorMass = m_invMassA + m_invMassB + m_invIA * m_a1 * m_a1 + m_invIB * m_a2 * m_a2;
//                if (m_motorMass > Settings.EPSILON)
//                {
//                    m_motorMass = 1.0f / m_motorMass;
//                }
//                else
//                {
//                    m_motorMass = 0.0f;
//                }
//            }

//            // Prismatic constraint.
//            {
//                Rot.mulToOutUnsafe(xf1.q, m_localYAxis1, m_perp);

//                temp.set_Renamed(d).addLocal(r1);
//                m_s1 = Vec2.cross(temp, m_perp);
//                m_s2 = Vec2.cross(r2, m_perp);

//                float m1 = m_invMassA, m2 = m_invMassB;
//                float i1 = m_invIA, i2 = m_invIB;

//                float k11 = m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;
//                float k12 = i1 * m_s1 * m_a1 + i2 * m_s2 * m_a2;
//                float k22 = m1 + m2 + i1 * m_a1 * m_a1 + i2 * m_a2 * m_a2;

//                m_K.ex.set_Renamed(k11, k12);
//                m_K.ey.set_Renamed(k12, k22);
//            }

//            // Compute motor and limit terms.
//            if (m_enableLimit)
//            {
//                float jointTranslation = Vec2.dot(m_axis, d);
//                if (MathUtils.abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings.linearSlop)
//                {
//                    m_limitState = LimitState.EQUAL;
//                }
//                else if (jointTranslation <= m_lowerTranslation)
//                {
//                    if (m_limitState != LimitState.AT_LOWER)
//                    {
//                        m_limitState = LimitState.AT_LOWER;
//                        m_impulse.y = 0.0f;
//                    }
//                }
//                else if (jointTranslation >= m_upperTranslation)
//                {
//                    if (m_limitState != LimitState.AT_UPPER)
//                    {
//                        m_limitState = LimitState.AT_UPPER;
//                        m_impulse.y = 0.0f;
//                    }
//                }
//                else
//                {
//                    m_limitState = LimitState.INACTIVE;
//                    m_impulse.y = 0.0f;
//                }
//            }
//            else
//            {
//                m_limitState = LimitState.INACTIVE;
//            }

//            if (m_enableMotor == false)
//            {
//                m_motorImpulse = 0.0f;
//            }

//            if (step.warmStarting)
//            {
//                // Account for variable time step.
//                m_impulse.mulLocal(step.dtRatio);
//                m_motorImpulse *= step.dtRatio;

//                Vec2 P = pool.popVec2();
//                temp.set_Renamed(m_axis).mulLocal(m_motorImpulse + m_impulse.y);
//                P.set_Renamed(m_perp).mulLocal(m_impulse.x).addLocal(temp);

//                float L1 = m_impulse.x * m_s1 + (m_motorImpulse + m_impulse.y) * m_a1;
//                float L2 = m_impulse.x * m_s2 + (m_motorImpulse + m_impulse.y) * m_a2;

//                temp.set_Renamed(P).mulLocal(m_invMassA);
//                b1.m_linearVelocity.subLocal(temp);
//                b1.m_angularVelocity -= m_invIA * L1;

//                temp.set_Renamed(P).mulLocal(m_invMassB);
//                b2.m_linearVelocity.addLocal(temp);
//                b2.m_angularVelocity += m_invIB * L2;
//                pool.pushVec2(1);
//            }
//            else
//            {
//                m_impulse.setZero();
//                m_motorImpulse = 0.0f;
//            }
//            pool.pushVec2(4);
//        }

//        /// <seealso cref="org.jbox2d.dynamics.joints.Joint.solveVelocityConstraints(org.jbox2d.dynamics.TimeStep)"></seealso>
//        public void solveVelocityConstraints(TimeStep step)
//        {
//            Body b1 = m_bodyA;
//            Body b2 = m_bodyB;

//            Vec2 v1 = b1.m_linearVelocity;
//            float w1 = b1.m_angularVelocity;
//            Vec2 v2 = b2.m_linearVelocity;
//            float w2 = b2.m_angularVelocity;

//            Vec2 temp = pool.popVec2();

//            // Solve linear motor constraint.
//            if (m_enableMotor && m_limitState != LimitState.EQUAL)
//            {
//                temp.set_Renamed(v2).subLocal(v1);
//                float Cdot = Vec2.dot(m_axis, temp) + m_a2 * w2 - m_a1 * w1;
//                float impulse = m_motorMass * (m_motorSpeed - Cdot);
//                float oldImpulse = m_motorImpulse;
//                float maxImpulse = step.dt * m_maxMotorForce;
//                m_motorImpulse = MathUtils.clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
//                impulse = m_motorImpulse - oldImpulse;

//                Vec2 P = pool.popVec2();
//                P.set_Renamed(m_axis).mulLocal(impulse);
//                float L1 = impulse * m_a1;
//                float L2 = impulse * m_a2;

//                temp.set_Renamed(P).mulLocal(m_invMassA);
//                v1.subLocal(temp);
//                w1 -= m_invIA * L1;

//                temp.set_Renamed(P).mulLocal(m_invMassB);
//                v2.addLocal(temp);
//                w2 += m_invIB * L2;
//                pool.pushVec2(1);
//            }

//            temp.set_Renamed(v2).subLocal(v1);
//            float Cdot1 = Vec2.dot(m_perp, temp) + m_s2 * w2 - m_s1 * w1;

//            if (m_enableLimit && m_limitState != LimitState.INACTIVE)
//            {
//                // Solve prismatic and limit constraint in block form.
//                temp.set_Renamed(v2).subLocal(v1);
//                float Cdot2 = Vec2.dot(m_axis, temp) + m_a2 * w2 - m_a1 * w1;

//                Vec2 Cdot = pool.popVec2();
//                Cdot.set_Renamed(Cdot1, Cdot2);

//                Vec2 f1 = pool.popVec2();
//                f1.set_Renamed(m_impulse);
//                Vec2 df = pool.popVec2();
//                m_K.solveToOut(Cdot.negateLocal(), df); // just leave negated
//                m_impulse.addLocal(df);

//                if (m_limitState == LimitState.AT_LOWER)
//                {
//                    m_impulse.y = MathUtils.max(m_impulse.y, 0.0f);
//                }
//                else if (m_limitState == LimitState.AT_UPPER)
//                {
//                    m_impulse.y = MathUtils.min(m_impulse.y, 0.0f);
//                }

//                // f2(1) = invK(1,1) * (-Cdot(1) - K(1,2) * (f2(2) - f1(2))) + f1(1)
//                float b = -Cdot1 - (m_impulse.y - f1.y) * m_K.ey.x;
//                float f2r;
//                if (m_K.ex.x != 0.0f)
//                {
//                    f2r = b / m_K.ex.x + f1.x;
//                }
//                else
//                {
//                    f2r = f1.x;
//                }

//                m_impulse.x = f2r;

//                df.set_Renamed(m_impulse).subLocal(f1);

//                Vec2 P = pool.popVec2();
//                temp.set_Renamed(m_axis).mulLocal(df.y);
//                P.set_Renamed(m_perp).mulLocal(df.x).addLocal(temp);

//                float L1 = df.x * m_s1 + df.y * m_a1;
//                float L2 = df.x * m_s2 + df.y * m_a2;

//                temp.set_Renamed(P).mulLocal(m_invMassA);
//                v1.subLocal(temp);
//                w1 -= m_invIA * L1;

//                temp.set_Renamed(P).mulLocal(m_invMassB);
//                v2.addLocal(temp);
//                w2 += m_invIB * L2;
//                pool.pushVec2(4);
//            }
//            else
//            {
//                // Limit is inactive, just solve the prismatic constraint in block
//                // form.
//                float df;
//                if (m_K.ex.x != 0.0f)
//                {
//                    df = (-Cdot1) / m_K.ex.x;
//                }
//                else
//                {
//                    df = 0.0f;
//                }
//                m_impulse.x += df;

//                Vec2 P = pool.popVec2();
//                P.set_Renamed(m_perp).mulLocal(df);

//                float L1 = df * m_s1;
//                float L2 = df * m_s2;

//                temp.set_Renamed(P).mulLocal(m_invMassA);
//                v1.subLocal(temp);
//                w1 -= m_invIA * L1;

//                temp.set_Renamed(P).mulLocal(m_invMassB);
//                v2.addLocal(temp);
//                w2 += m_invIB * L2;
//                pool.pushVec2(1);
//            }

//            pool.pushVec2(1);

//            b1.m_angularVelocity = w1;
//            b2.m_angularVelocity = w2;
//        }

//        /// <seealso cref="org.jbox2d.dynamics.joints.Joint.solvePositionConstraints(float)"></seealso>
//        public bool solvePositionConstraints(float baumgarte)
//        {
//            Body b1 = m_bodyA;
//            Body b2 = m_bodyB;

//            Vec2 c1 = b1.m_sweep.c;
//            float a1 = b1.m_sweep.a;

//            Vec2 c2 = b2.m_sweep.c;
//            float a2 = b2.m_sweep.a;

//            // Solve linear limit constraint.
//            float linearError = 0.0f, angularError = 0.0f;
//            bool active = false;
//            float C2 = 0.0f;

//            Mat22 R1 = pool.popMat22();
//            Mat22 R2 = pool.popMat22();
//            R1.set_Renamed(a1);
//            R2.set_Renamed(a2);

//            Vec2 r1 = pool.popVec2();
//            Vec2 r2 = pool.popVec2();
//            Vec2 temp = pool.popVec2();
//            Vec2 d = pool.popVec2();

//            r1.set_Renamed(m_localAnchor1).subLocal(m_localCenterA);
//            r2.set_Renamed(m_localAnchor2).subLocal(m_localCenterB);
//            Mat22.mulToOut(R1, r1, r1);
//            Mat22.mulToOut(R2, r2, r2);
//            d.set_Renamed(c2).addLocal(r2).subLocal(c1).subLocal(r1);

//            if (m_enableLimit)
//            {
//                Mat22.mulToOutUnsafe(R1, m_localXAxis1, m_axis);

//                temp.set_Renamed(d).addLocal(r1);
//                m_a1 = Vec2.cross(temp, m_axis);
//                m_a2 = Vec2.cross(r2, m_axis);

//                float translation = Vec2.dot(m_axis, d);
//                if (MathUtils.abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings.linearSlop)
//                {
//                    // Prevent large angular corrections
//                    C2 = MathUtils.clamp(translation, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);
//                    linearError = MathUtils.abs(translation);
//                    active = true;
//                }
//                else if (translation <= m_lowerTranslation)
//                {
//                    // Prevent large linear corrections and allow some slop.
//                    C2 = MathUtils.clamp(translation - m_lowerTranslation + Settings.linearSlop, -Settings.maxLinearCorrection, 0.0f);
//                    linearError = m_lowerTranslation - translation;
//                    active = true;
//                }
//                else if (translation >= m_upperTranslation)
//                {
//                    // Prevent large linear corrections and allow some slop.
//                    C2 = MathUtils.clamp(translation - m_upperTranslation - Settings.linearSlop, 0.0f, Settings.maxLinearCorrection);
//                    linearError = translation - m_upperTranslation;
//                    active = true;
//                }
//            }

//            Mat22.mulToOutUnsafe(R1, m_localYAxis1, m_perp);

//            temp.set_Renamed(d).addLocal(r1);
//            m_s1 = Vec2.cross(temp, m_perp);
//            m_s2 = Vec2.cross(r2, m_perp);

//            Vec2 impulse = pool.popVec2();
//            float C1;
//            C1 = Vec2.dot(m_perp, d);

//            linearError = MathUtils.max(linearError, MathUtils.abs(C1));
//            angularError = 0.0f;

//            if (active)
//            {
//                float m1 = m_invMassA, m2 = m_invMassB;
//                float i1 = m_invIA, i2 = m_invIB;

//                float k11 = m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;
//                float k12 = i1 * m_s1 * m_a1 + i2 * m_s2 * m_a2;
//                float k22 = m1 + m2 + i1 * m_a1 * m_a1 + i2 * m_a2 * m_a2;

//                m_K.ex.set_Renamed(k11, k12);
//                m_K.ey.set_Renamed(k12, k22);

//                Vec2 C = pool.popVec2();
//                C.x = C1;
//                C.y = C2;

//                m_K.solveToOut(C.negateLocal(), impulse);
//                pool.pushVec2(1);
//            }
//            else
//            {
//                float m1 = m_invMassA, m2 = m_invMassB;
//                float i1 = m_invIA, i2 = m_invIB;

//                float k11 = m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;

//                float impulse1;
//                if (k11 != 0.0f)
//                {
//                    impulse1 = (-C1) / k11;
//                }
//                else
//                {
//                    impulse1 = 0.0f;
//                }

//                impulse.x = impulse1;
//                impulse.y = 0.0f;
//            }

//            Vec2 P = pool.popVec2();
//            temp.set_Renamed(m_axis).mulLocal(impulse.y);
//            P.set_Renamed(m_perp).mulLocal(impulse.x).add(temp);

//            float L1 = impulse.x * m_s1 + impulse.y * m_a1;
//            float L2 = impulse.x * m_s2 + impulse.y * m_a2;

//            temp.set_Renamed(P).mulLocal(m_invMassA);
//            c1.subLocal(temp);
//            a1 -= m_invIA * L1;

//            temp.set_Renamed(P).mulLocal(m_invMassB);
//            c2.addLocal(temp);
//            a2 += m_invIB * L2;

//            // TODO_ERIN remove need for this.
//            b1.m_sweep.a = a1;
//            b2.m_sweep.a = a2;
//            b1.synchronizeTransform();
//            b2.synchronizeTransform();

//            pool.pushVec2(6);
//            pool.pushMat22(2);

//            return linearError <= Settings.linearSlop && angularError <= Settings.angularSlop;
//        }
//    }
//}