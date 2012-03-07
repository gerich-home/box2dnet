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

    /// <summary>
    /// A mouse joint is used to make a point on a body track a specified world point. This a soft
    /// constraint with a maximum force. This allows the constraint to stretch and without applying huge
    /// forces. NOTE: this joint is not documented in the manual because it was developed to be used in
    /// the testbed. If you want to learn how to use the mouse joint, look at the testbed.
    /// </summary>
    /// <author>Daniel</author>
    public class MouseJoint : Joint
    {
        private readonly Vec2 m_localAnchorB = new Vec2();
        private readonly Vec2 m_targetA = new Vec2();
        private float m_frequencyHz;
        private float m_dampingRatio;
        private float m_beta;

        // Solver shared
        private readonly Vec2 m_impulse = new Vec2();
        private float m_maxForce;
        private float m_gamma;

        // Solver temp
        public int m_indexA;
        public int m_indexB;
        public readonly Vec2 m_rB = new Vec2();
        public readonly Vec2 m_localCenterB = new Vec2();
        public float m_invMassB;
        public float m_invIB;
        private readonly Mat22 m_mass = new Mat22();
        private readonly Vec2 m_C = new Vec2();


        protected internal MouseJoint(IWorldPool argWorld, MouseJointDef def)
            : base(argWorld, def)
        {
            Debug.Assert(def.Target.Valid);
            Debug.Assert(def.MaxForce >= 0);
            Debug.Assert(def.FrequencyHz >= 0);
            Debug.Assert(def.DampingRatio >= 0);

            m_targetA.Set(def.Target);
            Transform.MulTransToOutUnsafe(BodyB.GetTransform(), m_targetA, m_localAnchorB);

            m_maxForce = def.MaxForce;
            m_impulse.SetZero();

            m_frequencyHz = def.FrequencyHz;
            m_dampingRatio = def.DampingRatio;

            m_beta = 0;
            m_gamma = 0;
        }

        public override void GetAnchorA(Vec2 argOut)
        {
            argOut.Set(m_targetA);
        }

        public override void GetAnchorB(Vec2 argOut)
        {
            BodyB.GetWorldPointToOut(m_localAnchorB, argOut);
        }

        public override void GetReactionForce(float invDt, Vec2 argOut)
        {
            argOut.Set(m_impulse).MulLocal(invDt);
        }

        public override float GetReactionTorque(float invDt)
        {
            return invDt * 0.0f;
        }

        public Vec2 Target
        {
            get
            {
                return m_targetA;
            }
            set
            {
                if (BodyB.Awake == false)
                {
                    BodyB.Awake = true;
                }
                m_targetA.Set(value);
            }
        }

        /// <summary>
        /// Gets or sets the maximum force in Newtons.
        /// </summary>
        public float MaxForce
        {
            get
            {
                return m_maxForce;
            }
            set
            {
                m_maxForce = value;
            }
        }

        /// <summary>
        /// Gets or sets the frequency in Hertz.
        /// </summary>
        public float Frequency
        {
            get
            {
                return m_frequencyHz;
            }
            set
            {
                m_frequencyHz = value;
            }
        }

        /// <summary>
        /// Gets or sets the damping ratio (dimensionless).
        /// </summary>
        public float DampingRatio
        {
            get
            {
                return m_dampingRatio;
            }
            set
            {
                m_dampingRatio = value;
            }
        }

        public override void InitVelocityConstraints(SolverData data)
        {
            m_indexA = BodyA.IslandIndex;
            m_indexB = BodyB.IslandIndex;
            m_localCenterB.Set(BodyB.Sweep.LocalCenter);
            m_invMassB = BodyB.InvMass;
            m_invIB = BodyB.InvI;

            Vec2 cB = data.Positions[m_indexB].C;
            float aB = data.Positions[m_indexB].A;
            Vec2 vB = data.Velocities[m_indexB].V;
            float wB = data.Velocities[m_indexB].W;

            Rot qB = Pool.PopRot();

            qB.Set(aB);

            float mass = BodyB.Mass;

            // Frequency
            float omega = 2.0f * MathUtils.PI * m_frequencyHz;

            // Damping coefficient
            float d = 2.0f * mass * m_dampingRatio * omega;

            // Spring stiffness
            float k = mass * (omega * omega);

            // magic formulas
            // gamma has units of inverse mass.
            // beta has units of inverse time.
            float h = data.Step.Dt;
            Debug.Assert(d + h * k > Settings.EPSILON);
            m_gamma = h * (d + h * k);
            if (m_gamma != 0.0f)
            {
                m_gamma = 1.0f / m_gamma;
            }
            m_beta = h * k * m_gamma;

            Vec2 temp = Pool.PopVec2();

            // Compute the effective mass matrix.
            Rot.MulToOutUnsafe(qB, temp.Set(m_localAnchorB).SubLocal(m_localCenterB), m_rB);

            // K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
            // = [1/m1+1/m2 0 ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
            // [ 0 1/m1+1/m2] [-r1.x*r1.y r1.x*r1.x] [-r1.x*r1.y r1.x*r1.x]
            Mat22 K = Pool.PopMat22();
            K.Ex.X = m_invMassB + m_invIB * m_rB.Y * m_rB.Y + m_gamma;
            K.Ex.Y = (-m_invIB) * m_rB.X * m_rB.Y;
            K.Ey.X = K.Ex.Y;
            K.Ey.Y = m_invMassB + m_invIB * m_rB.X * m_rB.X + m_gamma;

            K.InvertToOut(m_mass);

            m_C.Set(cB).AddLocal(m_rB).SubLocal(m_targetA);
            m_C.MulLocal(m_beta);

            // Cheat with some damping
            wB *= 0.98f;

            if (data.Step.WarmStarting)
            {
                m_impulse.MulLocal(data.Step.DtRatio);
                vB.X += m_invMassB * m_impulse.X;
                vB.Y += m_invMassB * m_impulse.Y;
                wB += m_invIB * Vec2.Cross(m_rB, m_impulse);
            }
            else
            {
                m_impulse.SetZero();
            }

            data.Velocities[m_indexB].V.Set(vB);
            data.Velocities[m_indexB].W = wB;

            Pool.PushVec2(1);
            Pool.PushMat22(1);
            Pool.PushRot(1);
        }

        public override bool SolvePositionConstraints(SolverData data)
        {
            return true;
        }

        public override void SolveVelocityConstraints(SolverData data)
        {

            Vec2 vB = data.Velocities[m_indexB].V;
            float wB = data.Velocities[m_indexB].W;

            // Cdot = v + cross(w, r)
            Vec2 Cdot = Pool.PopVec2();
            Vec2.CrossToOutUnsafe(wB, m_rB, Cdot);
            Cdot.AddLocal(vB);

            Vec2 impulse = Pool.PopVec2();
            Vec2 temp = Pool.PopVec2();

            temp.Set(m_impulse).MulLocal(m_gamma).AddLocal(m_C).AddLocal(Cdot).NegateLocal();
            Mat22.MulToOutUnsafe(m_mass, temp, impulse);

            Vec2 oldImpulse = temp;
            oldImpulse.Set(m_impulse);
            m_impulse.AddLocal(impulse);
            float maxImpulse = data.Step.Dt * m_maxForce;
            if (m_impulse.LengthSquared() > maxImpulse * maxImpulse)
            {
                m_impulse.MulLocal(maxImpulse / m_impulse.Length());
            }
            impulse.Set(m_impulse).SubLocal(oldImpulse);

            vB.X += m_invMassB * m_impulse.X;
            vB.Y += m_invMassB * m_impulse.Y;
            wB += m_invIB * Vec2.Cross(m_rB, impulse);

            data.Velocities[m_indexB].V.Set(vB);
            data.Velocities[m_indexB].W = wB;

            Pool.PushVec2(3);
        }
    }
}