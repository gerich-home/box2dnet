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

/*
* JBox2D - A Java Port of Erin Catto's Box2D
* 
* JBox2D homepage: http://jbox2d.sourceforge.net/
* Box2D homepage: http://www.box2d.org
* 
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

using Box2D.Common;
using Box2D.Pooling;

namespace Box2D.Dynamics.Joints
{

    //C = norm(p2 - p1) - L
    //u = (p2 - p1) / norm(p2 - p1)
    //Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
    //J = [-u -cross(r1, u) u cross(r2, u)]
    //K = J * invM * JT
    //= invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

    /// <summary> A distance joint constrains two points on two bodies to remain at a fixed distance from each
    /// other. You can view this as a massless, rigid rod.
    /// </summary>
    public class DistanceJoint : Joint
    {
        public float m_frequencyHz;
        public float m_dampingRatio;
        public float m_bias;

        // Solver shared
        public readonly Vec2 m_localAnchorA;
        public readonly Vec2 m_localAnchorB;
        public float m_gamma;
        public float m_impulse;
        public float m_length;

        // Solver temp
        public int m_indexA;
        public int m_indexB;
        public readonly Vec2 m_u = new Vec2();
        public readonly Vec2 m_rA = new Vec2();
        public readonly Vec2 m_rB = new Vec2();
        public readonly Vec2 m_localCenterA = new Vec2();
        public readonly Vec2 m_localCenterB = new Vec2();
        public float m_invMassA;
        public float m_invMassB;
        public float m_invIA;
        public float m_invIB;
        public float m_mass;

        public DistanceJoint(IWorldPool argWorld, DistanceJointDef def)
            : base(argWorld, def)
        {
            m_localAnchorA = def.localAnchorA.Clone();
            m_localAnchorB = def.localAnchorB.Clone();
            m_length = def.length;
            m_impulse = 0.0f;
            m_frequencyHz = def.frequencyHz;
            m_dampingRatio = def.dampingRatio;
            m_gamma = 0.0f;
            m_bias = 0.0f;
        }

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

        public float Length
        {
            get
            {
                return m_length;
            }
            set
            {
                m_length = value;
            }
        }

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

        public Vec2 LocalAnchorA
        {
            get
            {
                return m_localAnchorA;
            }
        }

        public Vec2 LocalAnchorB
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

        /// <summary>
        /// Get the reaction force given the inverse time step. Unit is N.
        /// </summary>
        public override void getReactionForce(float inv_dt, Vec2 argOut)
        {
            argOut.X = m_impulse * m_u.X * inv_dt;
            argOut.Y = m_impulse * m_u.Y * inv_dt;
        }

        /// <summary>
        /// Get the reaction torque given the inverse time step. Unit is N*m. This is always zero for a distance joint.
        /// </summary>
        public override float getReactionTorque(float inv_dt)
        {
            return 0.0f;
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

            qA.Set(aA);
            qB.Set(aB);

            // use m_u as temporary variable
            Rot.MulToOutUnsafe(qA, m_u.Set(m_localAnchorA).SubLocal(m_localCenterA), m_rA);
            Rot.MulToOutUnsafe(qB, m_u.Set(m_localAnchorB).SubLocal(m_localCenterB), m_rB);
            m_u.Set(cB).AddLocal(m_rB).SubLocal(cA).SubLocal(m_rA);

            pool.PushRot(2);

            // Handle singularity.
            float length = m_u.Length();
            if (length > Settings.LINEAR_SLOP)
            {
                m_u.X *= 1.0f / length;
                m_u.Y *= 1.0f / length;
            }
            else
            {
                m_u.Set(0.0f, 0.0f);
            }


            float crAu = Vec2.Cross(m_rA, m_u);
            float crBu = Vec2.Cross(m_rB, m_u);
            float invMass = m_invMassA + m_invIA * crAu * crAu + m_invMassB + m_invIB * crBu * crBu;

            // Compute the effective mass matrix.
            m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

            if (m_frequencyHz > 0.0f)
            {
                float C = length - m_length;

                // Frequency
                float omega = 2.0f * MathUtils.Pi * m_frequencyHz;

                // Damping coefficient
                float d = 2.0f * m_mass * m_dampingRatio * omega;

                // Spring stiffness
                float k = m_mass * omega * omega;

                // magic formulas
                float h = data.Step.Dt;
                m_gamma = h * (d + h * k);
                m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
                m_bias = C * h * k * m_gamma;

                invMass += m_gamma;
                m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;
            }
            else
            {
                m_gamma = 0.0f;
                m_bias = 0.0f;
            }
            if (data.Step.WarmStarting)
            {

                // Scale the impulse to support a variable time step.
                m_impulse *= data.Step.DtRatio;

                Vec2 P = pool.PopVec2();
                P.Set(m_u).MulLocal(m_impulse);

                vA.X -= m_invMassA * P.X;
                vA.Y -= m_invMassA * P.Y;
                wA -= m_invIA * Vec2.Cross(m_rA, P);

                vB.X += m_invMassB * P.X;
                vB.Y += m_invMassB * P.Y;
                wB += m_invIB * Vec2.Cross(m_rB, P);

                pool.PushVec2(1);
            }
            else
            {
                m_impulse = 0.0f;
            }
            data.Velocities[m_indexA].v.Set(vA);
            data.Velocities[m_indexA].w = wA;
            data.Velocities[m_indexB].v.Set(vB);
            data.Velocities[m_indexB].w = wB;
        }

        public override void solveVelocityConstraints(SolverData data)
        {
            Vec2 vA = data.Velocities[m_indexA].v;
            float wA = data.Velocities[m_indexA].w;
            Vec2 vB = data.Velocities[m_indexB].v;
            float wB = data.Velocities[m_indexB].w;

            Vec2 vpA = pool.PopVec2();
            Vec2 vpB = pool.PopVec2();

            // Cdot = dot(u, v + cross(w, r))
            Vec2.CrossToOutUnsafe(wA, m_rA, vpA);
            vpA.AddLocal(vA);
            Vec2.CrossToOutUnsafe(wB, m_rB, vpB);
            vpB.AddLocal(vB);
            float Cdot = Vec2.Dot(m_u, vpB.SubLocal(vpA));

            float impulse = (-m_mass) * (Cdot + m_bias + m_gamma * m_impulse);
            m_impulse += impulse;


            float Px = impulse * m_u.X;
            float Py = impulse * m_u.Y;

            vA.X -= m_invMassA * Px;
            vA.Y -= m_invMassA * Py;
            wA -= m_invIA * (m_rA.X * Py - m_rA.Y * Px);
            vB.X += m_invMassB * Px;
            vB.Y += m_invMassB * Py;
            wB += m_invIB * (m_rB.X * Py - m_rB.Y * Px);

            data.Velocities[m_indexA].v.Set(vA);
            data.Velocities[m_indexA].w = wA;
            data.Velocities[m_indexB].v.Set(vB);
            data.Velocities[m_indexB].w = wB;

            pool.PushVec2(2);
        }

        public override bool solvePositionConstraints(SolverData data)
        {
            if (m_frequencyHz > 0.0f)
            {
                return true;
            }
            Rot qA = pool.PopRot();
            Rot qB = pool.PopRot();
            Vec2 rA = pool.PopVec2();
            Vec2 rB = pool.PopVec2();
            Vec2 u = pool.PopVec2();

            Vec2 cA = data.Positions[m_indexA].c;
            float aA = data.Positions[m_indexA].a;
            Vec2 cB = data.Positions[m_indexB].c;
            float aB = data.Positions[m_indexB].a;

            qA.Set(aA);
            qB.Set(aB);

            Rot.MulToOutUnsafe(qA, u.Set(m_localAnchorA).SubLocal(m_localCenterA), rA);
            Rot.MulToOutUnsafe(qB, u.Set(m_localAnchorB).SubLocal(m_localCenterB), rB);
            u.Set(cB).AddLocal(rB).SubLocal(cA).SubLocal(rA);


            float length = u.Normalize();
            float C = length - m_length;
            C = MathUtils.Clamp(C, -Settings.MAX_LINEAR_CORRECTION, Settings.MAX_LINEAR_CORRECTION);

            float impulse = (-m_mass) * C;
            float Px = impulse * u.X;
            float Py = impulse * u.Y;

            cA.X -= m_invMassA * Px;
            cA.Y -= m_invMassA * Py;
            aA -= m_invIA * (rA.X * Py - rA.Y * Px);
            cB.X += m_invMassB * Px;
            cB.Y += m_invMassB * Py;
            aB += m_invIB * (rB.X * Py - rB.Y * Px);

            data.Positions[m_indexA].c.Set(cA);
            data.Positions[m_indexA].a = aA;
            data.Positions[m_indexB].c.Set(cB);
            data.Positions[m_indexB].a = aB;

            pool.PushVec2(3);
            pool.PushRot(2);

            return MathUtils.Abs(C) < Settings.LINEAR_SLOP;
        }
    }
}