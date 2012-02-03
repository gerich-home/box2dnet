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

using System.Diagnostics;
using Box2D.Common;
using Box2D.Pooling;

namespace Box2D.Dynamics.Joints
{

    //Linear constraint (point-to-line)
    //d = p2 - p1 = x2 + r2 - x1 - r1
    //C = dot(perp, d)
    //Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
    //   = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
    //J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
    //
    //Angular constraint
    //C = a2 - a1 + a_initial
    //Cdot = w2 - w1
    //J = [0 0 -1 0 0 1]
    //
    //K = J * invM * JT
    //
    //J = [-a -s1 a s2]
    //  [0  -1  0  1]
    //a = perp
    //s1 = cross(d + r1, a) = cross(p2 - x1, a)
    //s2 = cross(r2, a) = cross(p2 - x2, a)


    //Motor/Limit linear constraint
    //C = dot(ax1, d)
    //Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
    //J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

    //Block Solver
    //We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
    //when the mass has poor distribution (leading to large torques about the joint anchor points).
    //
    //The Jacobian has 3 rows:
    //J = [-uT -s1 uT s2] // linear
    //  [0   -1   0  1] // angular
    //  [-vT -a1 vT a2] // limit
    //
    //u = perp
    //v = axis
    //s1 = cross(d + r1, u), s2 = cross(r2, u)
    //a1 = cross(d + r1, v), a2 = cross(r2, v)

    //M * (v2 - v1) = JT * df
    //J * v2 = bias
    //
    //v2 = v1 + invM * JT * df
    //J * (v1 + invM * JT * df) = bias
    //K * df = bias - J * v1 = -Cdot
    //K = J * invM * JT
    //Cdot = J * v1 - bias
    //
    //Now solve for f2.
    //df = f2 - f1
    //K * (f2 - f1) = -Cdot
    //f2 = invK * (-Cdot) + f1
    //
    //Clamp accumulated limit impulse.
    //lower: f2(3) = max(f2(3), 0)
    //upper: f2(3) = min(f2(3), 0)
    //
    //Solve for correct f2(1:2)
    //K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
    //                    = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
    //K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
    //f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
    //
    //Now compute impulse to be applied:
    //df = f2 - f1

    /// <summary>
    ///   A prismatic joint. This joint provides one degree of freedom: translation along an axis fixed in
    ///   bodyA. Relative rotation is prevented. You can use a joint limit to restrict the range of motion
    ///   and a joint motor to drive the motion or to model joint friction.
    /// </summary>
    /// <author>Daniel</author>
    public class PrismaticJoint : Joint
    {
        // Solver shared
        public readonly Vec2 m_localAnchorA;
        public readonly Vec2 m_localAnchorB;
        public readonly Vec2 m_localXAxisA;
        public readonly Vec2 m_localYAxisA;
        public float m_referenceAngle;
        public readonly Vec3 m_impulse;
        public float m_motorImpulse;
        public float m_lowerTranslation;
        public float m_upperTranslation;
        public float m_maxMotorForce;
        public float m_motorSpeed;
        public bool m_enableLimit;
        public bool m_enableMotor;
        public LimitState m_limitState;

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
        public readonly Vec2 m_axis;
        public readonly Vec2 m_perp;
        public float m_s1, m_s2;
        public float m_a1, m_a2;
        public readonly Mat33 m_K;
        public float m_motorMass; // effective mass for motor/limit translational constraint.

        public PrismaticJoint(IWorldPool argWorld, PrismaticJointDef def)
            : base(argWorld, def)
        {
            m_localAnchorA = new Vec2(def.localAnchorA);
            m_localAnchorB = new Vec2(def.localAnchorB);
            m_localXAxisA = new Vec2(def.localAxisA);
            m_localXAxisA.Normalize();
            m_localYAxisA = new Vec2();
            Vec2.CrossToOutUnsafe(1f, m_localXAxisA, m_localYAxisA);
            m_referenceAngle = def.referenceAngle;

            m_impulse = new Vec3();
            m_motorMass = 0.0f;
            m_motorImpulse = 0.0f;

            m_lowerTranslation = def.lowerTranslation;
            m_upperTranslation = def.upperTranslation;
            m_maxMotorForce = def.maxMotorForce;
            m_motorSpeed = def.motorSpeed;
            m_enableLimit = def.enableLimit;
            m_enableMotor = def.enableMotor;
            m_limitState = LimitState.INACTIVE;

            m_K = new Mat33();
            m_axis = new Vec2();
            m_perp = new Vec2();
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
            m_bodyA.GetWorldPointToOut(m_localAnchorA, argOut);
        }

        public override void getAnchorB(Vec2 argOut)
        {
            m_bodyB.GetWorldPointToOut(m_localAnchorB, argOut);
        }

        public override void getReactionForce(float inv_dt, Vec2 argOut)
        {
            Vec2 temp = pool.PopVec2();
            temp.Set(m_axis).MulLocal(m_motorImpulse + m_impulse.z);
            argOut.Set(m_perp).MulLocal(m_impulse.x).AddLocal(temp).MulLocal(inv_dt);
            pool.PushVec2(1);
        }

        public override float getReactionTorque(float inv_dt)
        {
            return inv_dt * m_impulse.y;
        }

        ///// <summary>
        ///// Get the current joint translation, usually in meters.
        ///// </summary>
        ///// <returns></returns>
        //public float getJointTranslation()
        //{
        //    Vec2 pA = pool.popVec2();
        //    Vec2 pB = pool.popVec2();
        //    Vec2 axis = pool.popVec2();

        //    m_bodyA.getWorldPointToOut(m_localAnchorA, pA);
        //    m_bodyB.getWorldPointToOut(m_localAnchorB, pB);
        //    pB.subLocal(pA);
        //    m_bodyA.getWorldVectorToOut(m_localXAxisA, axis);

        //    float translation = Vec2.dot(pB, axis);

        //    pool.pushVec2(3);
        //    return translation;
        //}

        /// <summary>
        /// Get the current joint translation speed, usually in meters per second.
        /// </summary>
        virtual public float JointSpeed
        {
            get
            {
                Body bA = m_bodyA;
                Body bB = m_bodyB;

                Vec2[] pc = pool.PopVec2(9);
                Vec2 temp = pc[0];
                Vec2 rA = pc[1];
                Vec2 rB = pc[2];
                Vec2 p1 = pc[3];
                Vec2 p2 = pc[4];
                Vec2 d = pc[5];
                Vec2 axis = pc[6];
                Vec2 temp2 = pc[7];
                Vec2 temp3 = pc[8];

                temp.Set(m_localAnchorA).SubLocal(bA.Sweep.localCenter);
                Rot.mulToOutUnsafe(bA.Xf.q, temp, rA);

                temp.Set(m_localAnchorB).SubLocal(bB.Sweep.localCenter);
                Rot.mulToOutUnsafe(bB.Xf.q, temp, rB);

                p1.Set(bA.Sweep.c).AddLocal(rA);
                p2.Set(bB.Sweep.c).AddLocal(rB);

                d.Set(p2).SubLocal(p1);
                Rot.mulToOutUnsafe(bA.Xf.q, m_localXAxisA, axis);

                Vec2 vA = bA.m_linearVelocity;
                Vec2 vB = bB.m_linearVelocity;
                float wA = bA.m_angularVelocity;
                float wB = bB.m_angularVelocity;


                Vec2.CrossToOutUnsafe(wA, axis, temp);
                Vec2.CrossToOutUnsafe(wB, rB, temp2);
                Vec2.CrossToOutUnsafe(wA, rA, temp3);

                temp2.AddLocal(vB).SubLocal(vA).SubLocal(temp3);
                float speed = Vec2.Dot(d, temp) + Vec2.Dot(axis, temp2);

                pool.PushVec2(9);

                return speed;
            }
        }

        /// <summary>
        /// Is the joint limit enabled?
        /// </summary>
        /// <returns></returns>
        virtual public bool LimitEnabled
        {
            get
            {
                return m_enableLimit;
            }

        }

        /// <summary>
        /// Enable/disable the joint limit.
        /// </summary>
        /// <param name="flag"></param>
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

        /// <summary>
        /// Get the lower joint limit, usually in meters.
        /// </summary>
        /// <returns></returns>
        virtual public float LowerLimit
        {
            get
            {
                return m_lowerTranslation;
            }
        }

        /// <summary>
        /// Get the upper joint limit, usually in meters.</summary>
        /// <returns></returns>
        virtual public float UpperLimit
        {
            get
            {
                return m_upperTranslation;
            }
        }

        /// <summary>
        /// Set the joint limits, usually in meters.
        /// </summary>
        /// <param name="lower"></param>
        /// <param name="upper"></param>
        public virtual void setLimits(float lower, float upper)
        {
            Debug.Assert(lower <= upper);
            if (lower != m_lowerTranslation || upper != m_upperTranslation)
            {
                m_bodyA.Awake = true;
                m_bodyB.Awake = true;
                m_lowerTranslation = lower;
                m_upperTranslation = upper;
                m_impulse.z = 0.0f;
            }
        }

        /// <summary>
        /// Is the joint motor enabled?
        /// </summary>
        /// <returns></returns>
        virtual public bool MotorEnabled
        {
            get
            {
                return m_enableMotor;
            }
        }

        /// <summary>
        /// Enable/disable the joint motor.
        /// </summary>
        /// <param name="flag"></param>
        public virtual void enableMotor(bool flag)
        {
            m_bodyA.Awake = true;
            m_bodyB.Awake = true;
            m_enableMotor = flag;
        }

        /// <summary>
        /// Gets or sets the motor speed, usually in meters per second.
        /// </summary>
        virtual public float MotorSpeed
        {
            get
            {
                return m_motorSpeed;
            }
            set
            {
                m_bodyA.Awake = true;
                m_bodyB.Awake = true;
                m_motorSpeed = value;
            }
        }

        /// <summary>
        /// Set the maximum motor force, usually in N.
        /// </summary>
        /// <param name="force"></param>
        virtual public float MaxMotorForce
        {
            set
            {
                m_bodyA.Awake = true;
                m_bodyB.Awake = true;
                m_maxMotorForce = value;
            }
        }

        /// <summary>
        /// Get the current motor force, usually in N.
        /// </summary>
        /// <param name="inv_dt"></param>
        /// <returns></returns>
        public virtual float getMotorForce(float inv_dt)
        {
            return m_motorImpulse * inv_dt;
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

            Vec2 cA = data.Positions[m_indexA].c;
            float aA = data.Positions[m_indexA].a;
            Vec2 vA = data.Velocities[m_indexA].v;
            float wA = data.Velocities[m_indexA].w;

            Vec2 cB = data.Positions[m_indexB].c;
            float aB = data.Positions[m_indexB].a;
            Vec2 vB = data.Velocities[m_indexB].v;
            float wB = data.Velocities[m_indexB].w;

            Rot qA = pool.PopRot();
            Rot qB = pool.PopRot();
            Vec2 d = pool.PopVec2();
            Vec2 temp = pool.PopVec2();
            Vec2 rA = pool.PopVec2();
            Vec2 rB = pool.PopVec2();

            qA.set_Renamed(aA);
            qB.set_Renamed(aB);

            // Compute the effective masses.
            Rot.mulToOutUnsafe(qA, d.Set(m_localAnchorA).SubLocal(m_localCenterA), rA);
            Rot.mulToOutUnsafe(qB, d.Set(m_localAnchorB).SubLocal(m_localCenterB), rB);
            d.Set(cB).SubLocal(cA).AddLocal(rB).SubLocal(rA);

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            // Compute motor Jacobian and effective mass.
            {
                Rot.mulToOutUnsafe(qA, m_localXAxisA, m_axis);
                temp.Set(d).AddLocal(rA);
                m_a1 = Vec2.Cross(temp, m_axis);
                m_a2 = Vec2.Cross(rB, m_axis);

                m_motorMass = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;
                if (m_motorMass > 0.0f)
                {
                    m_motorMass = 1.0f / m_motorMass;
                }
            }

            // Prismatic constraint.
            {
                Rot.mulToOutUnsafe(qA, m_localYAxisA, m_perp);

                temp.Set(d).AddLocal(rA);
                m_s1 = Vec2.Cross(temp, m_perp);
                m_s2 = Vec2.Cross(rB, m_perp);

                float k11 = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2;
                float k12 = iA * m_s1 + iB * m_s2;
                float k13 = iA * m_s1 * m_a1 + iB * m_s2 * m_a2;
                float k22 = iA + iB;
                if (k22 == 0.0f)
                {
                    // For bodies with fixed rotation.
                    k22 = 1.0f;
                }
                float k23 = iA * m_a1 + iB * m_a2;
                float k33 = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;

                m_K.Ex.set_Renamed(k11, k12, k13);
                m_K.Ey.set_Renamed(k12, k22, k23);
                m_K.Ez.set_Renamed(k13, k23, k33);
            }

            // Compute motor and limit terms.
            if (m_enableLimit)
            {

                float jointTranslation = Vec2.Dot(m_axis, d);
                if (MathUtils.abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings.linearSlop)
                {
                    m_limitState = LimitState.EQUAL;
                }
                else if (jointTranslation <= m_lowerTranslation)
                {
                    if (m_limitState != LimitState.AT_LOWER)
                    {
                        m_limitState = LimitState.AT_LOWER;
                        m_impulse.z = 0.0f;
                    }
                }
                else if (jointTranslation >= m_upperTranslation)
                {
                    if (m_limitState != LimitState.AT_UPPER)
                    {
                        m_limitState = LimitState.AT_UPPER;
                        m_impulse.z = 0.0f;
                    }
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
                m_impulse.z = 0.0f;
            }

            if (m_enableMotor == false)
            {
                m_motorImpulse = 0.0f;
            }

            if (data.Step.WarmStarting)
            {
                // Account for variable time step.
                m_impulse.mulLocal(data.Step.DtRatio);
                m_motorImpulse *= data.Step.DtRatio;

                Vec2 P = pool.PopVec2();
                temp.Set(m_axis).MulLocal(m_motorImpulse + m_impulse.z);
                P.Set(m_perp).MulLocal(m_impulse.x).AddLocal(temp);

                float LA = m_impulse.x * m_s1 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a1;
                float LB = m_impulse.x * m_s2 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a2;

                vA.X -= mA * P.X;
                vA.Y -= mA * P.Y;
                wA -= iA * LA;

                vB.X += mB * P.X;
                vB.Y += mB * P.Y;
                wB += iB * LB;

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

            pool.PushRot(2);
            pool.PushVec2(4);
        }

        public override void solveVelocityConstraints(SolverData data)
        {
            Vec2 vA = data.Velocities[m_indexA].v;
            float wA = data.Velocities[m_indexA].w;
            Vec2 vB = data.Velocities[m_indexB].v;
            float wB = data.Velocities[m_indexB].w;

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            Vec2 temp = pool.PopVec2();

            // Solve linear motor constraint.
            if (m_enableMotor && m_limitState != LimitState.EQUAL)
            {
                temp.Set(vB).SubLocal(vA);
                float Cdot = Vec2.Dot(m_axis, temp) + m_a2 * wB - m_a1 * wA;
                float impulse = m_motorMass * (m_motorSpeed - Cdot);
                float oldImpulse = m_motorImpulse;
                float maxImpulse = data.Step.Dt * m_maxMotorForce;
                m_motorImpulse = MathUtils.clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = m_motorImpulse - oldImpulse;

                Vec2 P = pool.PopVec2();
                P.Set(m_axis).MulLocal(impulse);
                float LA = impulse * m_a1;
                float LB = impulse * m_a2;

                vA.X -= mA * P.X;
                vA.Y -= mA * P.Y;
                wA -= iA * LA;

                vB.X += mB * P.X;
                vB.Y += mB * P.Y;
                wB += iB * LB;

                pool.PushVec2(1);
            }

            Vec2 Cdot1 = pool.PopVec2();
            temp.Set(vB).SubLocal(vA);
            Cdot1.X = Vec2.Dot(m_perp, temp) + m_s2 * wB - m_s1 * wA;
            Cdot1.Y = wB - wA;
            // System.out.println(Cdot1);

            if (m_enableLimit && m_limitState != LimitState.INACTIVE)
            {
                // Solve prismatic and limit constraint in block form.
                float Cdot2;
                temp.Set(vB).SubLocal(vA);
                Cdot2 = Vec2.Dot(m_axis, temp) + m_a2 * wB - m_a1 * wA;

                Vec3 Cdot = pool.PopVec3();
                Cdot.set_Renamed(Cdot1.X, Cdot1.Y, Cdot2);
                Cdot.negateLocal();

                Vec3 f1 = pool.PopVec3();
                Vec3 df = pool.PopVec3();

                f1.set_Renamed(m_impulse);
                m_K.Solve33ToOut(Cdot.negateLocal(), df);
                //Cdot.negateLocal(); not used anymore
                m_impulse.addLocal(df);

                if (m_limitState == LimitState.AT_LOWER)
                {
                    m_impulse.z = MathUtils.max(m_impulse.z, 0.0f);
                }
                else if (m_limitState == LimitState.AT_UPPER)
                {
                    m_impulse.z = MathUtils.min(m_impulse.z, 0.0f);
                }

                // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) +
                // f1(1:2)
                Vec2 b = pool.PopVec2();
                Vec2 f2r = pool.PopVec2();

                temp.Set(m_K.Ez.x, m_K.Ez.y).MulLocal(m_impulse.z - f1.z);
                b.Set(Cdot1).NegateLocal().SubLocal(temp);

                temp.Set(f1.x, f1.y);
                m_K.Solve22ToOut(b, f2r);
                f2r.AddLocal(temp);
                m_impulse.x = f2r.X;
                m_impulse.y = f2r.Y;

                df.set_Renamed(m_impulse).subLocal(f1);

                Vec2 P = pool.PopVec2();
                temp.Set(m_axis).MulLocal(df.z);
                P.Set(m_perp).MulLocal(df.x).AddLocal(temp);

                float LA = df.x * m_s1 + df.y + df.z * m_a1;
                float LB = df.x * m_s2 + df.y + df.z * m_a2;

                vA.X -= mA * P.X;
                vA.Y -= mA * P.Y;
                wA -= iA * LA;

                vB.X += mB * P.X;
                vB.Y += mB * P.Y;
                wB += iB * LB;

                pool.PushVec2(3);
                pool.PushVec3(3);
            }
            else
            {
                // Limit is inactive, just solve the prismatic constraint in block form.
                Vec2 df = pool.PopVec2();
                m_K.Solve22ToOut(Cdot1.NegateLocal(), df);
                Cdot1.NegateLocal();

                m_impulse.x += df.X;
                m_impulse.y += df.Y;

                Vec2 P = pool.PopVec2();
                P.Set(m_perp).MulLocal(df.X);
                float LA = df.X * m_s1 + df.Y;
                float LB = df.X * m_s2 + df.Y;

                vA.X -= mA * P.X;
                vA.Y -= mA * P.Y;
                wA -= iA * LA;

                vB.X += mB * P.X;
                vB.Y += mB * P.Y;
                wB += iB * LB;

                Vec2 Cdot10 = pool.PopVec2();
                Cdot10.Set(Cdot1);

                Cdot1.X = Vec2.Dot(m_perp, temp.Set(vB).SubLocal(vA)) + m_s2 * wB - m_s1 * wA;
                Cdot1.Y = wB - wA;

                if (MathUtils.abs(Cdot1.X) > 0.01f || MathUtils.abs(Cdot1.Y) > 0.01f)
                {
                    // djm note: what's happening here?
                    Mat33.Mul22ToOutUnsafe(m_K, df, temp);
                    Cdot1.X += 0.0f;
                }

                pool.PushVec2(3);
            }

            data.Velocities[m_indexA].v.Set(vA);
            data.Velocities[m_indexA].w = wA;
            data.Velocities[m_indexB].v.Set(vB);
            data.Velocities[m_indexB].w = wB;

            pool.PushVec2(2);
        }

        public override bool solvePositionConstraints(SolverData data)
        {
            Rot qA = pool.PopRot();
            Rot qB = pool.PopRot();
            Vec2 rA = pool.PopVec2();
            Vec2 rB = pool.PopVec2();
            Vec2 d = pool.PopVec2();
            Vec2 axis = pool.PopVec2();
            Vec2 perp = pool.PopVec2();
            Vec2 temp = pool.PopVec2();
            Vec2 C1 = pool.PopVec2();

            Vec3 impulse = pool.PopVec3();

            Vec2 cA = data.Positions[m_indexA].c;
            float aA = data.Positions[m_indexA].a;
            Vec2 cB = data.Positions[m_indexB].c;
            float aB = data.Positions[m_indexB].a;

            qA.set_Renamed(aA);
            qB.set_Renamed(aB);

            float mA = m_invMassA, mB = m_invMassB;
            float iA = m_invIA, iB = m_invIB;

            // Compute fresh Jacobians
            Rot.mulToOutUnsafe(qA, temp.Set(m_localAnchorA).SubLocal(m_localCenterA), rA);
            Rot.mulToOutUnsafe(qB, temp.Set(m_localAnchorB).SubLocal(m_localCenterB), rB);
            d.Set(cB).AddLocal(rB).SubLocal(cA).SubLocal(rA);

            Rot.mulToOutUnsafe(qA, m_localXAxisA, axis);
            float a1 = Vec2.Cross(temp.Set(d).AddLocal(rA), axis);
            float a2 = Vec2.Cross(rB, axis);
            Rot.mulToOutUnsafe(qA, m_localYAxisA, perp);

            float s1 = Vec2.Cross(temp.Set(d).AddLocal(rA), perp);
            float s2 = Vec2.Cross(rB, perp);

            C1.X = Vec2.Dot(perp, d);
            C1.Y = aB - aA - m_referenceAngle;

            float linearError = MathUtils.abs(C1.X);
            float angularError = MathUtils.abs(C1.Y);

            bool active = false;
            float C2 = 0.0f;
            if (m_enableLimit)
            {
                float translation = Vec2.Dot(axis, d);
                if (MathUtils.abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings.linearSlop)
                {
                    // Prevent large angular corrections
                    C2 = MathUtils.clamp(translation, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);
                    linearError = MathUtils.max(linearError, MathUtils.abs(translation));
                    active = true;
                }
                else if (translation <= m_lowerTranslation)
                {
                    // Prevent large linear corrections and allow some slop.
                    C2 = MathUtils.clamp(translation - m_lowerTranslation + Settings.linearSlop, -Settings.maxLinearCorrection, 0.0f);
                    linearError = MathUtils.max(linearError, m_lowerTranslation - translation);
                    active = true;
                }
                else if (translation >= m_upperTranslation)
                {
                    // Prevent large linear corrections and allow some slop.
                    C2 = MathUtils.clamp(translation - m_upperTranslation - Settings.linearSlop, 0.0f, Settings.maxLinearCorrection);
                    linearError = MathUtils.max(linearError, translation - m_upperTranslation);
                    active = true;
                }
            }

            if (active)
            {
                float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                float k12 = iA * s1 + iB * s2;
                float k13 = iA * s1 * a1 + iB * s2 * a2;
                float k22 = iA + iB;
                if (k22 == 0.0f)
                {
                    // For fixed rotation
                    k22 = 1.0f;
                }
                float k23 = iA * a1 + iB * a2;
                float k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

                Mat33 K = pool.PopMat33();
                K.Ex.set_Renamed(k11, k12, k13);
                K.Ey.set_Renamed(k12, k22, k23);
                K.Ez.set_Renamed(k13, k23, k33);

                Vec3 C = pool.PopVec3();
                C.x = C1.X;
                C.y = C1.Y;
                C.z = C2;

                K.Solve33ToOut(C.negateLocal(), impulse);
                pool.PushVec3(1);
                pool.PushMat33(1);
            }
            else
            {
                float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                float k12 = iA * s1 + iB * s2;
                float k22 = iA + iB;
                if (k22 == 0.0f)
                {
                    k22 = 1.0f;
                }

                Mat22 K = pool.PopMat22();
                K.Ex.Set(k11, k12);
                K.Ey.Set(k12, k22);

                // temp is impulse1
                K.SolveToOut(C1.NegateLocal(), temp);
                C1.NegateLocal();

                impulse.x = temp.X;
                impulse.y = temp.Y;
                impulse.z = 0.0f;

                pool.PushMat22(1);
            }

            float Px = impulse.x * perp.X + impulse.z * axis.X;
            float Py = impulse.x * perp.Y + impulse.z * axis.Y;
            float LA = impulse.x * s1 + impulse.y + impulse.z * a1;
            float LB = impulse.x * s2 + impulse.y + impulse.z * a2;

            cA.X -= mA * Px;
            cA.Y -= mA * Py;
            aA -= iA * LA;
            cB.X += mB * Px;
            cB.Y += mB * Py;
            aB += iB * LB;

            data.Positions[m_indexA].c.Set(cA);
            data.Positions[m_indexA].a = aA;
            data.Positions[m_indexB].c.Set(cB);
            data.Positions[m_indexB].a = aB;

            pool.PushVec2(7);
            pool.PushVec3(1);
            pool.PushRot(2);

            return linearError <= Settings.linearSlop && angularError <= Settings.angularSlop;
        }
    }
}