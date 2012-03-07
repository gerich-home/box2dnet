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
        public readonly Vec2 LocalAnchorA;
        public readonly Vec2 LocalAnchorB;
        public readonly Vec2 LocalXAxisA;
        public readonly Vec2 LocalYAxisA;
        public float ReferenceAngle;
        public readonly Vec3 Impulse;
        public float MotorImpulse;
        public float LowerTranslation;
        public float UpperTranslation;
        private float m_maxMotorForce;
        private float m_motorSpeed;
        private bool m_limitEnabled;
        private bool m_motorEnabled;
        public LimitState LimitState;

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
        public readonly Vec2 Axis;
        public readonly Vec2 Perp;
        public float S1;
        public float S2;
        public float A1;
        public float A2;
        public readonly Mat33 K;
        public float MotorMass; // effective mass for motor/limit translational constraint.

        public PrismaticJoint(IWorldPool argWorld, PrismaticJointDef def)
            : base(argWorld, def)
        {
            LocalAnchorA = new Vec2(def.LocalAnchorA);
            LocalAnchorB = new Vec2(def.LocalAnchorB);
            LocalXAxisA = new Vec2(def.LocalAxisA);
            LocalXAxisA.Normalize();
            LocalYAxisA = new Vec2();
            Vec2.CrossToOutUnsafe(1f, LocalXAxisA, LocalYAxisA);
            ReferenceAngle = def.ReferenceAngle;

            Impulse = new Vec3();
            MotorMass = 0.0f;
            MotorImpulse = 0.0f;

            LowerTranslation = def.LowerTranslation;
            UpperTranslation = def.UpperTranslation;
            m_maxMotorForce = def.MaxMotorForce;
            m_motorSpeed = def.MotorSpeed;
            m_limitEnabled = def.EnableLimit;
            m_motorEnabled = def.EnableMotor;
            LimitState = LimitState.Inactive;

            K = new Mat33();
            Axis = new Vec2();
            Perp = new Vec2();
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
            Vec2 temp = Pool.PopVec2();
            temp.Set(Axis).MulLocal(MotorImpulse + Impulse.Z);
            argOut.Set(Perp).MulLocal(Impulse.X).AddLocal(temp).MulLocal(inv_dt);
            Pool.PushVec2(1);
        }

        public override float GetReactionTorque(float inv_dt)
        {
            return inv_dt * Impulse.Y;
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
        public float JointSpeed
        {
            get
            {
                Body bA = BodyA;
                Body bB = BodyB;

                Vec2[] pc = Pool.PopVec2(9);
                Vec2 temp = pc[0];
                Vec2 rA = pc[1];
                Vec2 rB = pc[2];
                Vec2 p1 = pc[3];
                Vec2 p2 = pc[4];
                Vec2 d = pc[5];
                Vec2 axis = pc[6];
                Vec2 temp2 = pc[7];
                Vec2 temp3 = pc[8];

                temp.Set(LocalAnchorA).SubLocal(bA.Sweep.LocalCenter);
                Rot.MulToOutUnsafe(bA.Xf.Q, temp, rA);

                temp.Set(LocalAnchorB).SubLocal(bB.Sweep.LocalCenter);
                Rot.MulToOutUnsafe(bB.Xf.Q, temp, rB);

                p1.Set(bA.Sweep.C).AddLocal(rA);
                p2.Set(bB.Sweep.C).AddLocal(rB);

                d.Set(p2).SubLocal(p1);
                Rot.MulToOutUnsafe(bA.Xf.Q, LocalXAxisA, axis);

                Vec2 vA = bA.LinearVelocity;
                Vec2 vB = bB.LinearVelocity;
                float wA = bA.AngularVelocity;
                float wB = bB.AngularVelocity;


                Vec2.CrossToOutUnsafe(wA, axis, temp);
                Vec2.CrossToOutUnsafe(wB, rB, temp2);
                Vec2.CrossToOutUnsafe(wA, rA, temp3);

                temp2.AddLocal(vB).SubLocal(vA).SubLocal(temp3);
                float speed = Vec2.Dot(d, temp) + Vec2.Dot(axis, temp2);

                Pool.PushVec2(9);

                return speed;
            }
        }

        /// <summary>
        /// Is the joint limit enabled?
        /// </summary>
        /// <returns></returns>
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

        /// <summary>
        /// Get the lower joint limit, usually in meters.
        /// </summary>
        /// <returns></returns>
        public float LowerLimit
        {
            get
            {
                return LowerTranslation;
            }
        }

        /// <summary>
        /// Get the upper joint limit, usually in meters.</summary>
        /// <returns></returns>
        public float UpperLimit
        {
            get
            {
                return UpperTranslation;
            }
        }

        /// <summary>
        /// Set the joint limits, usually in meters.
        /// </summary>
        /// <param name="lower"></param>
        /// <param name="upper"></param>
        public void SetLimits(float lower, float upper)
        {
            Debug.Assert(lower <= upper);
            if (lower != LowerTranslation || upper != UpperTranslation)
            {
                BodyA.Awake = true;
                BodyB.Awake = true;
                LowerTranslation = lower;
                UpperTranslation = upper;
                Impulse.Z = 0.0f;
            }
        }

        /// <summary>
        /// Is the joint motor enabled?
        /// </summary>
        /// <returns></returns>
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

        /// <summary>
        /// Gets or sets the motor speed, usually in meters per second.
        /// </summary>
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

        /// <summary>
        /// Set the maximum motor force, usually in N.
        /// </summary>
        /// <param name="force"></param>
        public float MaxMotorForce
        {
            set
            {
                BodyA.Awake = true;
                BodyB.Awake = true;
                m_maxMotorForce = value;
            }
        }

        /// <summary>
        /// Get the current motor force, usually in N.
        /// </summary>
        /// <param name="inv_dt"></param>
        /// <returns></returns>
        public float GetMotorForce(float inv_dt)
        {
            return MotorImpulse * inv_dt;
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
            Vec2 d = Pool.PopVec2();
            Vec2 temp = Pool.PopVec2();
            Vec2 rA = Pool.PopVec2();
            Vec2 rB = Pool.PopVec2();

            qA.Set(aA);
            qB.Set(aB);

            // Compute the effective masses.
            Rot.MulToOutUnsafe(qA, d.Set(LocalAnchorA).SubLocal(LocalCenterA), rA);
            Rot.MulToOutUnsafe(qB, d.Set(LocalAnchorB).SubLocal(LocalCenterB), rB);
            d.Set(cB).SubLocal(cA).AddLocal(rB).SubLocal(rA);

            float mA = InvMassA, mB = InvMassB;
            float iA = InvIA, iB = InvIB;

            // Compute motor Jacobian and effective mass.
            {
                Rot.MulToOutUnsafe(qA, LocalXAxisA, Axis);
                temp.Set(d).AddLocal(rA);
                A1 = Vec2.Cross(temp, Axis);
                A2 = Vec2.Cross(rB, Axis);

                MotorMass = mA + mB + iA * A1 * A1 + iB * A2 * A2;
                if (MotorMass > 0.0f)
                {
                    MotorMass = 1.0f / MotorMass;
                }
            }

            // Prismatic constraint.
            {
                Rot.MulToOutUnsafe(qA, LocalYAxisA, Perp);

                temp.Set(d).AddLocal(rA);
                S1 = Vec2.Cross(temp, Perp);
                S2 = Vec2.Cross(rB, Perp);

                float k11 = mA + mB + iA * S1 * S1 + iB * S2 * S2;
                float k12 = iA * S1 + iB * S2;
                float k13 = iA * S1 * A1 + iB * S2 * A2;
                float k22 = iA + iB;
                if (k22 == 0.0f)
                {
                    // For bodies with fixed rotation.
                    k22 = 1.0f;
                }
                float k23 = iA * A1 + iB * A2;
                float k33 = mA + mB + iA * A1 * A1 + iB * A2 * A2;

                K.Ex.Set(k11, k12, k13);
                K.Ey.Set(k12, k22, k23);
                K.Ez.Set(k13, k23, k33);
            }

            // Compute motor and limit terms.
            if (m_limitEnabled)
            {

                float jointTranslation = Vec2.Dot(Axis, d);
                if (MathUtils.Abs(UpperTranslation - LowerTranslation) < 2.0f * Settings.LINEAR_SLOP)
                {
                    LimitState = LimitState.Equal;
                }
                else if (jointTranslation <= LowerTranslation)
                {
                    if (LimitState != LimitState.AtLower)
                    {
                        LimitState = LimitState.AtLower;
                        Impulse.Z = 0.0f;
                    }
                }
                else if (jointTranslation >= UpperTranslation)
                {
                    if (LimitState != LimitState.AtUpper)
                    {
                        LimitState = LimitState.AtUpper;
                        Impulse.Z = 0.0f;
                    }
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
                Impulse.Z = 0.0f;
            }

            if (m_motorEnabled == false)
            {
                MotorImpulse = 0.0f;
            }

            if (data.Step.WarmStarting)
            {
                // Account for variable time step.
                Impulse.MulLocal(data.Step.DtRatio);
                MotorImpulse *= data.Step.DtRatio;

                Vec2 P = Pool.PopVec2();
                temp.Set(Axis).MulLocal(MotorImpulse + Impulse.Z);
                P.Set(Perp).MulLocal(Impulse.X).AddLocal(temp);

                float LA = Impulse.X * S1 + Impulse.Y + (MotorImpulse + Impulse.Z) * A1;
                float LB = Impulse.X * S2 + Impulse.Y + (MotorImpulse + Impulse.Z) * A2;

                vA.X -= mA * P.X;
                vA.Y -= mA * P.Y;
                wA -= iA * LA;

                vB.X += mB * P.X;
                vB.Y += mB * P.Y;
                wB += iB * LB;

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

            Pool.PushRot(2);
            Pool.PushVec2(4);
        }

        public override void SolveVelocityConstraints(SolverData data)
        {
            Vec2 vA = data.Velocities[IndexA].V;
            float wA = data.Velocities[IndexA].W;
            Vec2 vB = data.Velocities[IndexB].V;
            float wB = data.Velocities[IndexB].W;

            float mA = InvMassA, mB = InvMassB;
            float iA = InvIA, iB = InvIB;

            Vec2 temp = Pool.PopVec2();

            // Solve linear motor constraint.
            if (m_motorEnabled && LimitState != LimitState.Equal)
            {
                temp.Set(vB).SubLocal(vA);
                float Cdot = Vec2.Dot(Axis, temp) + A2 * wB - A1 * wA;
                float impulse = MotorMass * (m_motorSpeed - Cdot);
                float oldImpulse = MotorImpulse;
                float maxImpulse = data.Step.Dt * m_maxMotorForce;
                MotorImpulse = MathUtils.Clamp(MotorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = MotorImpulse - oldImpulse;

                Vec2 P = Pool.PopVec2();
                P.Set(Axis).MulLocal(impulse);
                float LA = impulse * A1;
                float LB = impulse * A2;

                vA.X -= mA * P.X;
                vA.Y -= mA * P.Y;
                wA -= iA * LA;

                vB.X += mB * P.X;
                vB.Y += mB * P.Y;
                wB += iB * LB;

                Pool.PushVec2(1);
            }

            Vec2 Cdot1 = Pool.PopVec2();
            temp.Set(vB).SubLocal(vA);
            Cdot1.X = Vec2.Dot(Perp, temp) + S2 * wB - S1 * wA;
            Cdot1.Y = wB - wA;
            // System.out.println(Cdot1);

            if (m_limitEnabled && LimitState != LimitState.Inactive)
            {
                // Solve prismatic and limit constraint in block form.
                float Cdot2;
                temp.Set(vB).SubLocal(vA);
                Cdot2 = Vec2.Dot(Axis, temp) + A2 * wB - A1 * wA;

                Vec3 Cdot = Pool.PopVec3();
                Cdot.Set(Cdot1.X, Cdot1.Y, Cdot2);
                Cdot.NegateLocal();

                Vec3 f1 = Pool.PopVec3();
                Vec3 df = Pool.PopVec3();

                f1.Set(Impulse);
                K.Solve33ToOut(Cdot.NegateLocal(), df);
                //Cdot.negateLocal(); not used anymore
                Impulse.AddLocal(df);

                if (LimitState == LimitState.AtLower)
                {
                    Impulse.Z = MathUtils.Max(Impulse.Z, 0.0f);
                }
                else if (LimitState == LimitState.AtUpper)
                {
                    Impulse.Z = MathUtils.Min(Impulse.Z, 0.0f);
                }

                // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) +
                // f1(1:2)
                Vec2 b = Pool.PopVec2();
                Vec2 f2r = Pool.PopVec2();

                temp.Set(K.Ez.X, K.Ez.Y).MulLocal(Impulse.Z - f1.Z);
                b.Set(Cdot1).NegateLocal().SubLocal(temp);

                temp.Set(f1.X, f1.Y);
                K.Solve22ToOut(b, f2r);
                f2r.AddLocal(temp);
                Impulse.X = f2r.X;
                Impulse.Y = f2r.Y;

                df.Set(Impulse).SubLocal(f1);

                Vec2 P = Pool.PopVec2();
                temp.Set(Axis).MulLocal(df.Z);
                P.Set(Perp).MulLocal(df.X).AddLocal(temp);

                float LA = df.X * S1 + df.Y + df.Z * A1;
                float LB = df.X * S2 + df.Y + df.Z * A2;

                vA.X -= mA * P.X;
                vA.Y -= mA * P.Y;
                wA -= iA * LA;

                vB.X += mB * P.X;
                vB.Y += mB * P.Y;
                wB += iB * LB;

                Pool.PushVec2(3);
                Pool.PushVec3(3);
            }
            else
            {
                // Limit is inactive, just solve the prismatic constraint in block form.
                Vec2 df = Pool.PopVec2();
                K.Solve22ToOut(Cdot1.NegateLocal(), df);
                Cdot1.NegateLocal();

                Impulse.X += df.X;
                Impulse.Y += df.Y;

                Vec2 P = Pool.PopVec2();
                P.Set(Perp).MulLocal(df.X);
                float LA = df.X * S1 + df.Y;
                float LB = df.X * S2 + df.Y;

                vA.X -= mA * P.X;
                vA.Y -= mA * P.Y;
                wA -= iA * LA;

                vB.X += mB * P.X;
                vB.Y += mB * P.Y;
                wB += iB * LB;

                Vec2 Cdot10 = Pool.PopVec2();
                Cdot10.Set(Cdot1);

                Cdot1.X = Vec2.Dot(Perp, temp.Set(vB).SubLocal(vA)) + S2 * wB - S1 * wA;
                Cdot1.Y = wB - wA;

                if (MathUtils.Abs(Cdot1.X) > 0.01f || MathUtils.Abs(Cdot1.Y) > 0.01f)
                {
                    // djm note: what's happening here?
                    Mat33.Mul22ToOutUnsafe(K, df, temp);
                    Cdot1.X += 0.0f;
                }

                Pool.PushVec2(3);
            }

            data.Velocities[IndexA].V.Set(vA);
            data.Velocities[IndexA].W = wA;
            data.Velocities[IndexB].V.Set(vB);
            data.Velocities[IndexB].W = wB;

            Pool.PushVec2(2);
        }

        public override bool SolvePositionConstraints(SolverData data)
        {
            Rot qA = Pool.PopRot();
            Rot qB = Pool.PopRot();
            Vec2 rA = Pool.PopVec2();
            Vec2 rB = Pool.PopVec2();
            Vec2 d = Pool.PopVec2();
            Vec2 axis = Pool.PopVec2();
            Vec2 perp = Pool.PopVec2();
            Vec2 temp = Pool.PopVec2();
            Vec2 C1 = Pool.PopVec2();

            Vec3 impulse = Pool.PopVec3();

            Vec2 cA = data.Positions[IndexA].C;
            float aA = data.Positions[IndexA].A;
            Vec2 cB = data.Positions[IndexB].C;
            float aB = data.Positions[IndexB].A;

            qA.Set(aA);
            qB.Set(aB);

            float mA = InvMassA, mB = InvMassB;
            float iA = InvIA, iB = InvIB;

            // Compute fresh Jacobians
            Rot.MulToOutUnsafe(qA, temp.Set(LocalAnchorA).SubLocal(LocalCenterA), rA);
            Rot.MulToOutUnsafe(qB, temp.Set(LocalAnchorB).SubLocal(LocalCenterB), rB);
            d.Set(cB).AddLocal(rB).SubLocal(cA).SubLocal(rA);

            Rot.MulToOutUnsafe(qA, LocalXAxisA, axis);
            float a1 = Vec2.Cross(temp.Set(d).AddLocal(rA), axis);
            float a2 = Vec2.Cross(rB, axis);
            Rot.MulToOutUnsafe(qA, LocalYAxisA, perp);

            float s1 = Vec2.Cross(temp.Set(d).AddLocal(rA), perp);
            float s2 = Vec2.Cross(rB, perp);

            C1.X = Vec2.Dot(perp, d);
            C1.Y = aB - aA - ReferenceAngle;

            float linearError = MathUtils.Abs(C1.X);
            float angularError = MathUtils.Abs(C1.Y);

            bool active = false;
            float C2 = 0.0f;
            if (m_limitEnabled)
            {
                float translation = Vec2.Dot(axis, d);
                if (MathUtils.Abs(UpperTranslation - LowerTranslation) < 2.0f * Settings.LINEAR_SLOP)
                {
                    // Prevent large angular corrections
                    C2 = MathUtils.Clamp(translation, -Settings.MAX_LINEAR_CORRECTION, Settings.MAX_LINEAR_CORRECTION);
                    linearError = MathUtils.Max(linearError, MathUtils.Abs(translation));
                    active = true;
                }
                else if (translation <= LowerTranslation)
                {
                    // Prevent large linear corrections and allow some slop.
                    C2 = MathUtils.Clamp(translation - LowerTranslation + Settings.LINEAR_SLOP, -Settings.MAX_LINEAR_CORRECTION, 0.0f);
                    linearError = MathUtils.Max(linearError, LowerTranslation - translation);
                    active = true;
                }
                else if (translation >= UpperTranslation)
                {
                    // Prevent large linear corrections and allow some slop.
                    C2 = MathUtils.Clamp(translation - UpperTranslation - Settings.LINEAR_SLOP, 0.0f, Settings.MAX_LINEAR_CORRECTION);
                    linearError = MathUtils.Max(linearError, translation - UpperTranslation);
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

                Mat33 K = Pool.PopMat33();
                K.Ex.Set(k11, k12, k13);
                K.Ey.Set(k12, k22, k23);
                K.Ez.Set(k13, k23, k33);

                Vec3 C = Pool.PopVec3();
                C.X = C1.X;
                C.Y = C1.Y;
                C.Z = C2;

                K.Solve33ToOut(C.NegateLocal(), impulse);
                Pool.PushVec3(1);
                Pool.PushMat33(1);
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

                Mat22 K = Pool.PopMat22();
                K.Ex.Set(k11, k12);
                K.Ey.Set(k12, k22);

                // temp is impulse1
                K.SolveToOut(C1.NegateLocal(), temp);
                C1.NegateLocal();

                impulse.X = temp.X;
                impulse.Y = temp.Y;
                impulse.Z = 0.0f;

                Pool.PushMat22(1);
            }

            float Px = impulse.X * perp.X + impulse.Z * axis.X;
            float Py = impulse.X * perp.Y + impulse.Z * axis.Y;
            float LA = impulse.X * s1 + impulse.Y + impulse.Z * a1;
            float LB = impulse.X * s2 + impulse.Y + impulse.Z * a2;

            cA.X -= mA * Px;
            cA.Y -= mA * Py;
            aA -= iA * LA;
            cB.X += mB * Px;
            cB.Y += mB * Py;
            aB += iB * LB;

            data.Positions[IndexA].C.Set(cA);
            data.Positions[IndexA].A = aA;
            data.Positions[IndexB].C.Set(cB);
            data.Positions[IndexB].A = aB;

            Pool.PushVec2(7);
            Pool.PushVec3(1);
            Pool.PushRot(2);

            return linearError <= Settings.LINEAR_SLOP && angularError <= Settings.ANGULAR_SLOP;
        }
    }
}