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
        public float FrequencyHz;
        public float DampingRatio;
        public float Bias;

        // Solver shared
        public readonly Vec2 LocalAnchorA;
        public readonly Vec2 LocalAnchorB;
        public float Gamma;
        public float Impulse;
        public float Length;

        // Solver temp
        public int IndexA;
        public int IndexB;
        public readonly Vec2 U = new Vec2();
        public readonly Vec2 RA = new Vec2();
        public readonly Vec2 RB = new Vec2();
        public readonly Vec2 LocalCenterA = new Vec2();
        public readonly Vec2 LocalCenterB = new Vec2();
        public float InvMassA;
        public float InvMassB;
        public float InvIA;
        public float InvIB;
        public float Mass;

        public DistanceJoint(IWorldPool argWorld, DistanceJointDef def)
            : base(argWorld, def)
        {
            LocalAnchorA = def.LocalAnchorA.Clone();
            LocalAnchorB = def.LocalAnchorB.Clone();
            Length = def.Length;
            Impulse = 0.0f;
            FrequencyHz = def.FrequencyHz;
            DampingRatio = def.DampingRatio;
            Gamma = 0.0f;
            Bias = 0.0f;
        }

        public override void GetAnchorA(Vec2 argOut)
        {
            BodyA.GetWorldPointToOut(LocalAnchorA, argOut);
        }

        public override void GetAnchorB(Vec2 argOut)
        {
            BodyB.GetWorldPointToOut(LocalAnchorB, argOut);
        }

        /// <summary>
        /// Get the reaction force given the inverse time step. Unit is N.
        /// </summary>
        public override void GetReactionForce(float inv_dt, Vec2 argOut)
        {
            argOut.X = Impulse * U.X * inv_dt;
            argOut.Y = Impulse * U.Y * inv_dt;
        }

        /// <summary>
        /// Get the reaction torque given the inverse time step. Unit is N*m. This is always zero for a distance joint.
        /// </summary>
        public override float GetReactionTorque(float inv_dt)
        {
            return 0.0f;
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

            qA.Set(aA);
            qB.Set(aB);

            // use m_u as temporary variable
            Rot.MulToOutUnsafe(qA, U.Set(LocalAnchorA).SubLocal(LocalCenterA), RA);
            Rot.MulToOutUnsafe(qB, U.Set(LocalAnchorB).SubLocal(LocalCenterB), RB);
            U.Set(cB).AddLocal(RB).SubLocal(cA).SubLocal(RA);

            Pool.PushRot(2);

            // Handle singularity.
            float length = U.Length();
            if (length > Settings.LINEAR_SLOP)
            {
                U.X *= 1.0f / length;
                U.Y *= 1.0f / length;
            }
            else
            {
                U.Set(0.0f, 0.0f);
            }


            float crAu = Vec2.Cross(RA, U);
            float crBu = Vec2.Cross(RB, U);
            float invMass = InvMassA + InvIA * crAu * crAu + InvMassB + InvIB * crBu * crBu;

            // Compute the effective mass matrix.
            Mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

            if (FrequencyHz > 0.0f)
            {
                float C = length - Length;

                // Frequency
                float omega = 2.0f * MathUtils.PI * FrequencyHz;

                // Damping coefficient
                float d = 2.0f * Mass * DampingRatio * omega;

                // Spring stiffness
                float k = Mass * omega * omega;

                // magic formulas
                float h = data.Step.Dt;
                Gamma = h * (d + h * k);
                Gamma = Gamma != 0.0f ? 1.0f / Gamma : 0.0f;
                Bias = C * h * k * Gamma;

                invMass += Gamma;
                Mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;
            }
            else
            {
                Gamma = 0.0f;
                Bias = 0.0f;
            }
            if (data.Step.WarmStarting)
            {

                // Scale the impulse to support a variable time step.
                Impulse *= data.Step.DtRatio;

                Vec2 P = Pool.PopVec2();
                P.Set(U).MulLocal(Impulse);

                vA.X -= InvMassA * P.X;
                vA.Y -= InvMassA * P.Y;
                wA -= InvIA * Vec2.Cross(RA, P);

                vB.X += InvMassB * P.X;
                vB.Y += InvMassB * P.Y;
                wB += InvIB * Vec2.Cross(RB, P);

                Pool.PushVec2(1);
            }
            else
            {
                Impulse = 0.0f;
            }
            data.Velocities[IndexA].V.Set(vA);
            data.Velocities[IndexA].W = wA;
            data.Velocities[IndexB].V.Set(vB);
            data.Velocities[IndexB].W = wB;
        }

        public override void SolveVelocityConstraints(SolverData data)
        {
            Vec2 vA = data.Velocities[IndexA].V;
            float wA = data.Velocities[IndexA].W;
            Vec2 vB = data.Velocities[IndexB].V;
            float wB = data.Velocities[IndexB].W;

            Vec2 vpA = Pool.PopVec2();
            Vec2 vpB = Pool.PopVec2();

            // Cdot = dot(u, v + cross(w, r))
            Vec2.CrossToOutUnsafe(wA, RA, vpA);
            vpA.AddLocal(vA);
            Vec2.CrossToOutUnsafe(wB, RB, vpB);
            vpB.AddLocal(vB);
            float Cdot = Vec2.Dot(U, vpB.SubLocal(vpA));

            float impulse = (-Mass) * (Cdot + Bias + Gamma * Impulse);
            Impulse += impulse;


            float Px = impulse * U.X;
            float Py = impulse * U.Y;

            vA.X -= InvMassA * Px;
            vA.Y -= InvMassA * Py;
            wA -= InvIA * (RA.X * Py - RA.Y * Px);
            vB.X += InvMassB * Px;
            vB.Y += InvMassB * Py;
            wB += InvIB * (RB.X * Py - RB.Y * Px);

            data.Velocities[IndexA].V.Set(vA);
            data.Velocities[IndexA].W = wA;
            data.Velocities[IndexB].V.Set(vB);
            data.Velocities[IndexB].W = wB;

            Pool.PushVec2(2);
        }

        public override bool SolvePositionConstraints(SolverData data)
        {
            if (FrequencyHz > 0.0f)
            {
                return true;
            }
            Rot qA = Pool.PopRot();
            Rot qB = Pool.PopRot();
            Vec2 rA = Pool.PopVec2();
            Vec2 rB = Pool.PopVec2();
            Vec2 u = Pool.PopVec2();

            Vec2 cA = data.Positions[IndexA].C;
            float aA = data.Positions[IndexA].A;
            Vec2 cB = data.Positions[IndexB].C;
            float aB = data.Positions[IndexB].A;

            qA.Set(aA);
            qB.Set(aB);

            Rot.MulToOutUnsafe(qA, u.Set(LocalAnchorA).SubLocal(LocalCenterA), rA);
            Rot.MulToOutUnsafe(qB, u.Set(LocalAnchorB).SubLocal(LocalCenterB), rB);
            u.Set(cB).AddLocal(rB).SubLocal(cA).SubLocal(rA);


            float length = u.Normalize();
            float C = length - Length;
            C = MathUtils.Clamp(C, -Settings.MAX_LINEAR_CORRECTION, Settings.MAX_LINEAR_CORRECTION);

            float impulse = (-Mass) * C;
            float Px = impulse * u.X;
            float Py = impulse * u.Y;

            cA.X -= InvMassA * Px;
            cA.Y -= InvMassA * Py;
            aA -= InvIA * (rA.X * Py - rA.Y * Px);
            cB.X += InvMassB * Px;
            cB.Y += InvMassB * Py;
            aB += InvIB * (rB.X * Py - rB.Y * Px);

            data.Positions[IndexA].C.Set(cA);
            data.Positions[IndexA].A = aA;
            data.Positions[IndexB].C.Set(cB);
            data.Positions[IndexB].A = aB;

            Pool.PushVec2(3);
            Pool.PushRot(2);

            return MathUtils.Abs(C) < Settings.LINEAR_SLOP;
        }
    }
}