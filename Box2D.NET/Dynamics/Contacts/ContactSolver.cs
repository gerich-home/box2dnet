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
using Box2D.Collision;
using Box2D.Collision.Shapes;
using Box2D.Common;

namespace Box2D.Dynamics.Contacts
{
    /// <author>Daniel</author>
    public class ContactSolver
    {
        public const bool DEBUG_SOLVER = false;
        public const float ERROR_TO_I = 1e-3f;

        /// <summary>
        /// For each solver, this is the initial number of constraints in the array, which expands as
        /// needed.
        /// </summary>
        public const int INITIAL_NUM_CONSTRAINTS = 256;

        /// <summary>
        /// Ensure a reasonable condition number. for the block solver
        /// </summary>
        public const float MAX_CONDITION_NUMBER = 100.0f;

        public TimeStep Step;
        public Position[] Positions;
        public Velocity[] Velocities;
        public ContactPositionConstraint[] PositionConstraints;
        public ContactVelocityConstraint[] VelocityConstraints;
        public Contact[] Contacts;
        public int Count;

        public ContactSolver()
        {
            PositionConstraints = new ContactPositionConstraint[INITIAL_NUM_CONSTRAINTS];
            VelocityConstraints = new ContactVelocityConstraint[INITIAL_NUM_CONSTRAINTS];
            for (int i = 0; i < INITIAL_NUM_CONSTRAINTS; i++)
            {
                PositionConstraints[i] = new ContactPositionConstraint();
                VelocityConstraints[i] = new ContactVelocityConstraint();
            }
        }

        // djm pooling
        private readonly Vec2 tangent = new Vec2();
        private readonly Vec2 temp1 = new Vec2();
        private readonly Vec2 temp2 = new Vec2();

        public void Init(ContactSolverDef def)
        {
            //Console.WriteLine("Initializing contact solver");
            Step = def.Step;
            Count = def.Count;


            if (PositionConstraints.Length < Count)
            {
                ContactPositionConstraint[] old = PositionConstraints;
                PositionConstraints = new ContactPositionConstraint[MathUtils.Max(old.Length * 2, Count)];
                Array.Copy(old, 0, PositionConstraints, 0, old.Length);
                for (int i = old.Length; i < PositionConstraints.Length; i++)
                {
                    PositionConstraints[i] = new ContactPositionConstraint();
                }
            }

            if (VelocityConstraints.Length < Count)
            {
                ContactVelocityConstraint[] old = VelocityConstraints;
                VelocityConstraints = new ContactVelocityConstraint[MathUtils.Max(old.Length * 2, Count)];
                Array.Copy(old, 0, VelocityConstraints, 0, old.Length);
                for (int i = old.Length; i < VelocityConstraints.Length; i++)
                {
                    VelocityConstraints[i] = new ContactVelocityConstraint();
                }
            }

            Positions = def.Positions;
            Velocities = def.Velocities;
            Contacts = def.Contacts;

            for (int i = 0; i < Count; ++i)
            {
                //Console.WriteLine("contacts: " + m_count);
                Contact contact = Contacts[i];

                Fixture fixtureA = contact.FixtureA;
                Fixture fixtureB = contact.FixtureB;
                Shape shapeA = fixtureA.Shape;
                Shape shapeB = fixtureB.Shape;
                float radiusA = shapeA.Radius;
                float radiusB = shapeB.Radius;
                Body bodyA = fixtureA.Body;
                Body bodyB = fixtureB.Body;
                Manifold manifold = contact.Manifold;

                int pointCount = manifold.PointCount;
                Debug.Assert(pointCount > 0);

                ContactVelocityConstraint vc = VelocityConstraints[i];
                vc.Friction = contact.Friction;
                vc.Restitution = contact.Restitution;
                vc.TangentSpeed = contact.TangentSpeed;
                vc.IndexA = bodyA.IslandIndex;
                vc.IndexB = bodyB.IslandIndex;
                vc.InvMassA = bodyA.InvMass;
                vc.InvMassB = bodyB.InvMass;
                vc.InvIA = bodyA.InvI;
                vc.InvIB = bodyB.InvI;
                vc.ContactIndex = i;
                vc.PointCount = pointCount;
                vc.K.SetZero();
                vc.NormalMass.SetZero();

                ContactPositionConstraint pc = PositionConstraints[i];
                pc.IndexA = bodyA.IslandIndex;
                pc.IndexB = bodyB.IslandIndex;
                pc.InvMassA = bodyA.InvMass;
                pc.InvMassB = bodyB.InvMass;
                pc.LocalCenterA.Set(bodyA.Sweep.LocalCenter);
                pc.LocalCenterB.Set(bodyB.Sweep.LocalCenter);
                pc.InvIA = bodyA.InvI;
                pc.InvIB = bodyB.InvI;
                pc.LocalNormal.Set(manifold.LocalNormal);
                pc.LocalPoint.Set(manifold.LocalPoint);
                pc.PointCount = pointCount;
                pc.RadiusA = radiusA;
                pc.RadiusB = radiusB;
                pc.Type = manifold.Type;

                //Console.WriteLine("contact point count: " + pointCount);
                for (int j = 0; j < pointCount; j++)
                {
                    ManifoldPoint cp = manifold.Points[j];
                    ContactVelocityConstraint.VelocityConstraintPoint vcp = vc.Points[j];

                    if (Step.WarmStarting)
                    {
                        //Debug.Assert(cp.normalImpulse == 0);
                        //Console.WriteLine("contact normal impulse: " + cp.normalImpulse);
                        vcp.NormalImpulse = Step.DtRatio * cp.NormalImpulse;
                        vcp.TangentImpulse = Step.DtRatio * cp.TangentImpulse;
                    }
                    else
                    {
                        vcp.NormalImpulse = 0;
                        vcp.TangentImpulse = 0;
                    }

                    vcp.rA.SetZero();
                    vcp.rB.SetZero();
                    vcp.NormalMass = 0;
                    vcp.TangentMass = 0;
                    vcp.VelocityBias = 0;

                    pc.LocalPoints[j].Set(cp.LocalPoint);
                }
            }
        }

        // djm pooling, and from above
        private readonly Vec2 P = new Vec2();
        private readonly Vec2 temp = new Vec2();

        public void WarmStart()
        {
            // Warm start.
            for (int i = 0; i < Count; ++i)
            {
                ContactVelocityConstraint vc = VelocityConstraints[i];

                int indexA = vc.IndexA;
                int indexB = vc.IndexB;
                float mA = vc.InvMassA;
                float iA = vc.InvIA;
                float mB = vc.InvMassB;
                float iB = vc.InvIB;
                int pointCount = vc.PointCount;


                Vec2 vA = Velocities[indexA].V;
                float wA = Velocities[indexA].W;
                Vec2 vB = Velocities[indexB].V;
                float wB = Velocities[indexB].W;

                Vec2 normal = vc.Normal;
                Vec2.CrossToOutUnsafe(normal, 1.0f, tangent);

                for (int j = 0; j < pointCount; ++j)
                {
                    ContactVelocityConstraint.VelocityConstraintPoint vcp = vc.Points[j];
                    //Console.WriteLine("vcp normal impulse is " + vcp.normalImpulse);
                    temp.Set(normal).MulLocal(vcp.NormalImpulse);
                    P.Set(tangent).MulLocal(vcp.TangentImpulse).AddLocal(temp);
                    wA -= iA * Vec2.Cross(vcp.rA, P);
                    vA.SubLocal(temp.Set(P).MulLocal(mA));
                    //Debug.Assert(vA.x == 0);
                    //Debug.Assert(wA == 0);
                    wB += iB * Vec2.Cross(vcp.rB, P);
                    vB.AddLocal(temp.Set(P).MulLocal(mB));
                    //Debug.Assert(vB.x == 0);
                    //Debug.Assert(wB == 0);
                }
                Velocities[indexA].W = wA;
                Velocities[indexB].W = wB;

                //Console.WriteLine("Ending velocity for " + indexA + " is " + vA.x + "," + vA.y + " - " + wA);
                //Console.WriteLine("Ending velocity for " + indexB + " is " + vB.x + "," + vB.y + " - " + wB);
            }
        }

        // djm pooling, and from above
        private readonly Transform xfA = new Transform();
        private readonly Transform xfB = new Transform();
        private readonly WorldManifold worldManifold = new WorldManifold();

        public void InitializeVelocityConstraints()
        {
            //Console.WriteLine("Initializing velocity constraints for " + m_count + " contacts");
            // Warm start.
            for (int i = 0; i < Count; ++i)
            {
                ContactVelocityConstraint vc = VelocityConstraints[i];
                ContactPositionConstraint pc = PositionConstraints[i];

                float radiusA = pc.RadiusA;
                float radiusB = pc.RadiusB;
                Manifold manifold = Contacts[vc.ContactIndex].Manifold;

                int indexA = vc.IndexA;
                int indexB = vc.IndexB;

                float mA = vc.InvMassA;
                float mB = vc.InvMassB;
                float iA = vc.InvIA;
                float iB = vc.InvIB;
                Vec2 localCenterA = pc.LocalCenterA;
                Vec2 localCenterB = pc.LocalCenterB;

                Vec2 cA = Positions[indexA].C;
                float aA = Positions[indexA].A;
                Vec2 vA = Velocities[indexA].V;
                float wA = Velocities[indexA].W;

                Vec2 cB = Positions[indexB].C;
                float aB = Positions[indexB].A;
                Vec2 vB = Velocities[indexB].V;
                float wB = Velocities[indexB].W;

                Debug.Assert(manifold.PointCount > 0);

                xfA.Q.Set(aA);
                xfB.Q.Set(aB);
                Rot.MulToOutUnsafe(xfA.Q, localCenterA, temp);
                xfA.P.Set(cA).SubLocal(temp);
                Rot.MulToOutUnsafe(xfB.Q, localCenterB, temp);
                xfB.P.Set(cB).SubLocal(temp);

                worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB);

                vc.Normal.Set(worldManifold.Normal);

                int pointCount = vc.PointCount;
                for (int j = 0; j < pointCount; ++j)
                {
                    ContactVelocityConstraint.VelocityConstraintPoint vcp = vc.Points[j];

                    vcp.rA.Set(worldManifold.Points[j]).SubLocal(cA);
                    vcp.rB.Set(worldManifold.Points[j]).SubLocal(cB);

                    float rnA = Vec2.Cross(vcp.rA, vc.Normal);
                    float rnB = Vec2.Cross(vcp.rB, vc.Normal);

                    float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                    vcp.NormalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                    Vec2.CrossToOutUnsafe(vc.Normal, 1.0f, tangent);

                    float rtA = Vec2.Cross(vcp.rA, tangent);
                    float rtB = Vec2.Cross(vcp.rB, tangent);

                    float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

                    vcp.TangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                    // Setup a velocity bias for restitution.
                    vcp.VelocityBias = 0.0f;
                    Vec2.CrossToOutUnsafe(wB, vcp.rB, temp1);
                    Vec2.CrossToOutUnsafe(wA, vcp.rA, temp2);
                    temp.Set(vB).AddLocal(temp1).SubLocal(vA).SubLocal(temp2);
                    float vRel = Vec2.Dot(vc.Normal, temp);
                    if (vRel < -Settings.VELOCITY_THRESHOLD)
                    {
                        vcp.VelocityBias = (-vc.Restitution) * vRel;
                    }
                }

                // If we have two points, then prepare the block solver.
                if (vc.PointCount == 2)
                {
                    ContactVelocityConstraint.VelocityConstraintPoint vcp1 = vc.Points[0];
                    ContactVelocityConstraint.VelocityConstraintPoint vcp2 = vc.Points[1];

                    float rn1A = Vec2.Cross(vcp1.rA, vc.Normal);
                    float rn1B = Vec2.Cross(vcp1.rB, vc.Normal);
                    float rn2A = Vec2.Cross(vcp2.rA, vc.Normal);
                    float rn2B = Vec2.Cross(vcp2.rB, vc.Normal);

                    float k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
                    float k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
                    float k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;
                    if (k11 * k11 < MAX_CONDITION_NUMBER * (k11 * k22 - k12 * k12))
                    {
                        // K is safe to invert.
                        vc.K.Ex.Set(k11, k12);
                        vc.K.Ey.Set(k12, k22);
                        vc.K.InvertToOut(vc.NormalMass);
                    }
                    else
                    {
                        // The constraints are redundant, just use one.
                        // TODO_ERIN use deepest?
                        vc.PointCount = 1;
                    }
                }
            }
        }

        // djm pooling from above
        private readonly Vec2 dv = new Vec2();
        private readonly Vec2 a = new Vec2();
        private readonly Vec2 b = new Vec2();
        private readonly Vec2 dv1 = new Vec2();
        private readonly Vec2 dv2 = new Vec2();
        private readonly Vec2 x = new Vec2();
        private readonly Vec2 d = new Vec2();
        private readonly Vec2 P1 = new Vec2();
        private readonly Vec2 P2 = new Vec2();

        public void SolveVelocityConstraints()
        {
            for (int i = 0; i < Count; ++i)
            {
                ContactVelocityConstraint vc = VelocityConstraints[i];

                int indexA = vc.IndexA;
                int indexB = vc.IndexB;

                float mA = vc.InvMassA;
                float mB = vc.InvMassB;
                float iA = vc.InvIA;
                float iB = vc.InvIB;
                int pointCount = vc.PointCount;

                Vec2 vA = Velocities[indexA].V;
                float wA = Velocities[indexA].W;
                Vec2 vB = Velocities[indexB].V;
                float wB = Velocities[indexB].W;
                //Debug.Assert(wA == 0);
                //Debug.Assert(wB == 0);

                Vec2 normal = vc.Normal;
                //Vec2.crossToOutUnsafe(normal, 1f, tangent);
                tangent.X = 1.0f * vc.Normal.Y;
                tangent.Y = (-1.0f) * vc.Normal.X;
                float friction = vc.Friction;

                Debug.Assert(pointCount == 1 || pointCount == 2);

                // Solve tangent constraints
                for (int j = 0; j < pointCount; ++j)
                {
                    ContactVelocityConstraint.VelocityConstraintPoint vcp = vc.Points[j];
                    //Vec2.crossToOutUnsafe(wA, vcp.rA, temp);
                    //Vec2.crossToOutUnsafe(wB, vcp.rB, dv);
                    //dv.addLocal(vB).subLocal(vA).subLocal(temp);
                    Vec2 a = vcp.rA;

                    dv.X = (-wB) * vcp.rB.Y + vB.X - vA.X + wA * a.Y;
                    dv.Y = wB * vcp.rB.X + vB.Y - vA.Y - wA * a.X;

                    // Compute tangent force
                    float vt = dv.X * tangent.X + dv.Y * tangent.Y - vc.TangentSpeed;
                    float lambda = vcp.TangentMass * (-vt);

                    // Clamp the accumulated force
                    float maxFriction = friction * vcp.NormalImpulse;
                    float newImpulse = MathUtils.Clamp(vcp.TangentImpulse + lambda, -maxFriction, maxFriction);
                    lambda = newImpulse - vcp.TangentImpulse;
                    vcp.TangentImpulse = newImpulse;

                    // Apply contact impulse
                    // Vec2 P = lambda * tangent;

                    float Px = tangent.X * lambda;
                    float Py = tangent.Y * lambda;

                    // vA -= invMassA * P;
                    vA.X -= Px * mA;
                    vA.Y -= Py * mA;
                    wA -= iA * (vcp.rA.X * Py - vcp.rA.Y * Px);

                    // vB += invMassB * P;
                    vB.X += Px * mB;
                    vB.Y += Py * mB;
                    wB += iB * (vcp.rB.X * Py - vcp.rB.Y * Px);

                    //Console.WriteLine("tangent solve velocity (point "+j+") for " + indexA + " is " + vA.x + "," + vA.y + " rot " + wA);
                    //Console.WriteLine("tangent solve velocity (point "+j+") for " + indexB + " is " + vB.x + "," + vB.y + " rot " + wB);
                }

                // Solve normal constraints
                if (vc.PointCount == 1)
                {
                    ContactVelocityConstraint.VelocityConstraintPoint vcp = vc.Points[0];
                    Vec2 a1 = vcp.rA;

                    // Relative velocity at contact
                    //Vec2 dv = vB + Cross(wB, vcp.rB) - vA - Cross(wA, vcp.rA);

                    //Vec2.crossToOut(wA, vcp.rA, temp1);
                    //Vec2.crossToOut(wB, vcp.rB, dv);
                    //dv.addLocal(vB).subLocal(vA).subLocal(temp1);

                    dv.X = (-wB) * vcp.rB.Y + vB.X - vA.X + wA * a1.Y;
                    dv.Y = wB * vcp.rB.X + vB.Y - vA.Y - wA * a1.X;

                    // Compute normal impulse
                    float vn = dv.X * normal.X + dv.Y * normal.Y;
                    float lambda = (-vcp.NormalMass) * (vn - vcp.VelocityBias);

                    // Clamp the accumulated impulse
                    float a = vcp.NormalImpulse + lambda;
                    float newImpulse = (a > 0.0f ? a : 0.0f);
                    lambda = newImpulse - vcp.NormalImpulse;
                    //Debug.Assert(newImpulse == 0);
                    vcp.NormalImpulse = newImpulse;

                    // Apply contact impulse
                    float Px = normal.X * lambda;
                    float Py = normal.Y * lambda;

                    // vA -= invMassA * P;
                    vA.X -= Px * mA;
                    vA.Y -= Py * mA;
                    wA -= iA * (vcp.rA.X * Py - vcp.rA.Y * Px);
                    //Debug.Assert(vA.x == 0);

                    // vB += invMassB * P;
                    vB.X += Px * mB;
                    vB.Y += Py * mB;
                    wB += iB * (vcp.rB.X * Py - vcp.rB.Y * Px);
                    //Debug.Assert(vB.x == 0);
                }
                else
                {
                    // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on
                    // Box2D_Lite).
                    // Build the mini LCP for this contact patch
                    //
                    // vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
                    //
                    // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
                    // b = vn_0 - velocityBias
                    //
                    // The system is solved using the "Total enumeration method" (s. Murty). The complementary
                    // constraint vn_i * x_i
                    // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D
                    // contact problem the cases
                    // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be
                    // tested. The first valid
                    // solution that satisfies the problem is chosen.
                    //
                    // In order to account of the accumulated impulse 'a' (because of the iterative nature of
                    // the solver which only requires
                    // that the accumulated impulse is clamped and not the incremental impulse) we change the
                    // impulse variable (x_i).
                    //
                    // Substitute:
                    //
                    // x = a + d
                    //
                    // a := old total impulse
                    // x := new total impulse
                    // d := incremental impulse
                    //
                    // For the current iteration we extend the formula for the incremental impulse
                    // to compute the new total impulse:
                    //
                    // vn = A * d + b
                    // = A * (x - a) + b
                    // = A * x + b - A * a
                    // = A * x + b'
                    // b' = b - A * a;

                    ContactVelocityConstraint.VelocityConstraintPoint cp1 = vc.Points[0];
                    ContactVelocityConstraint.VelocityConstraintPoint cp2 = vc.Points[1];
                    a.X = cp1.NormalImpulse;
                    a.Y = cp2.NormalImpulse;

                    Debug.Assert(a.X >= 0.0f && a.Y >= 0.0f);
                    // Relative velocity at contact
                    // Vec2 dv1 = vB + Cross(wB, cp1.rB) - vA - Cross(wA, cp1.rA);
                    dv1.X = (-wB) * cp1.rB.Y + vB.X - vA.X + wA * cp1.rA.Y;
                    dv1.Y = wB * cp1.rB.X + vB.Y - vA.Y - wA * cp1.rA.X;

                    // Vec2 dv2 = vB + Cross(wB, cp2.rB) - vA - Cross(wA, cp2.rA);
                    dv2.X = (-wB) * cp2.rB.Y + vB.X - vA.X + wA * cp2.rA.Y;
                    dv2.Y = wB * cp2.rB.X + vB.Y - vA.Y - wA * cp2.rA.X;

                    // Compute normal velocity
                    float vn1 = dv1.X * normal.X + dv1.Y * normal.Y;
                    float vn2 = dv2.X * normal.X + dv2.Y * normal.Y;

                    b.X = vn1 - cp1.VelocityBias;
                    b.Y = vn2 - cp2.VelocityBias;
                    //Console.WriteLine("b is " + b.x + "," + b.y);

                    // Compute b'
                    Mat22 R = vc.K;
                    b.X -= (R.Ex.X * a.X + R.Ey.X * a.Y);
                    b.Y -= (R.Ex.Y * a.X + R.Ey.Y * a.Y);
                    //Console.WriteLine("b' is " + b.x + "," + b.y);

                    // final float k_errorTol = 1e-3f;
                    // B2_NOT_USED(k_errorTol);
                    for (; ; )
                    {
                        //
                        // Case 1: vn = 0
                        //
                        // 0 = A * x' + b'
                        //
                        // Solve for x':
                        //
                        // x' = - inv(A) * b'
                        //
                        // Vec2 x = - Mul(c.normalMass, b);
                        Mat22.MulToOutUnsafe(vc.NormalMass, b, x);
                        x.MulLocal(-1);

                        if (x.X >= 0.0f && x.Y >= 0.0f)
                        {
                            //Console.WriteLine("case 1");
                            // Get the incremental impulse
                            // Vec2 d = x - a;
                            d.Set(x).SubLocal(a);

                            // Apply incremental impulse
                            // Vec2 P1 = d.x * normal;
                            // Vec2 P2 = d.y * normal;
                            P1.Set(normal).MulLocal(d.X);
                            P2.Set(normal).MulLocal(d.Y);

                            /*
                            * vA -= invMassA * (P1 + P2); wA -= invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
                            * 
                            * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
                            */

                            temp1.Set(P1).AddLocal(P2);
                            temp2.Set(temp1).MulLocal(mA);
                            vA.SubLocal(temp2);
                            temp2.Set(temp1).MulLocal(mB);
                            vB.AddLocal(temp2);
                            //Debug.Assert(vA.x == 0);
                            //Debug.Assert(vB.x == 0);

                            wA -= iA * (Vec2.Cross(cp1.rA, P1) + Vec2.Cross(cp2.rA, P2));
                            wB += iB * (Vec2.Cross(cp1.rB, P1) + Vec2.Cross(cp2.rB, P2));

                            // Accumulate
                            cp1.NormalImpulse = x.X;
                            cp2.NormalImpulse = x.Y;

                            /*
                            * #if B2_DEBUG_SOLVER == 1 // Postconditions dv1 = vB + Cross(wB, cp1.rB) - vA -
                            * Cross(wA, cp1.rA); dv2 = vB + Cross(wB, cp2.rB) - vA - Cross(wA, cp2.rA);
                            * 
                            * // Compute normal velocity vn1 = Dot(dv1, normal); vn2 = Dot(dv2, normal);
                            * 
                            * Debug.Assert(Abs(vn1 - cp1.velocityBias) < k_errorTol); Debug.Assert(Abs(vn2 - cp2.velocityBias)
                            * < k_errorTol); #endif
                            */
                            if (DEBUG_SOLVER)
                            {
                                // Postconditions
                                Vec2 _dv1 = vB.Add(Vec2.Cross(wB, cp1.rB).SubLocal(vA).SubLocal(Vec2.Cross(wA, cp1.rA)));
                                Vec2 _dv2 = vB.Add(Vec2.Cross(wB, cp2.rB).SubLocal(vA).SubLocal(Vec2.Cross(wA, cp2.rA)));
                                // Compute normal velocity
                                vn1 = Vec2.Dot(_dv1, normal);
                                vn2 = Vec2.Dot(_dv2, normal);

                                Debug.Assert(MathUtils.Abs(vn1 - cp1.VelocityBias) < ERROR_TO_I);
                                Debug.Assert(MathUtils.Abs(vn2 - cp2.VelocityBias) < ERROR_TO_I);
                            }
                            break;
                        }

                        //
                        // Case 2: vn1 = 0 and x2 = 0
                        //
                        // 0 = a11 * x1' + a12 * 0 + b1'
                        // vn2 = a21 * x1' + a22 * 0 + '
                        //
                        x.X = (-cp1.NormalMass) * b.X;
                        x.Y = 0.0f;
                        vn1 = 0.0f;
                        vn2 = vc.K.Ex.Y * x.X + b.Y;

                        if (x.X >= 0.0f && vn2 >= 0.0f)
                        {
                            //Console.WriteLine("case 2");
                            // Get the incremental impulse
                            d.Set(x).SubLocal(a);

                            // Apply incremental impulse
                            // Vec2 P1 = d.x * normal;
                            // Vec2 P2 = d.y * normal;
                            P1.Set(normal).MulLocal(d.X);
                            P2.Set(normal).MulLocal(d.Y);

                            /*
                            * Vec2 P1 = d.x * normal; Vec2 P2 = d.y * normal; vA -= invMassA * (P1 + P2); wA -=
                            * invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
                            * 
                            * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
                            */

                            temp1.Set(P1).AddLocal(P2);
                            temp2.Set(temp1).MulLocal(mA);
                            vA.SubLocal(temp2);
                            temp2.Set(temp1).MulLocal(mB);
                            vB.AddLocal(temp2);
                            //Debug.Assert(vA.x == 0);
                            //Debug.Assert(vB.x == 0);

                            wA -= iA * (Vec2.Cross(cp1.rA, P1) + Vec2.Cross(cp2.rA, P2));
                            wB += iB * (Vec2.Cross(cp1.rB, P1) + Vec2.Cross(cp2.rB, P2));


                            // Accumulate
                            //Debug.Assert(x.x == 0 && x.y == 0);
                            cp1.NormalImpulse = x.X;
                            cp2.NormalImpulse = x.Y;

                            /*
                            * #if B2_DEBUG_SOLVER == 1 // Postconditions dv1 = vB + Cross(wB, cp1.rB) - vA -
                            * Cross(wA, cp1.rA);
                            * 
                            * // Compute normal velocity vn1 = Dot(dv1, normal);
                            * 
                            * Debug.Assert(Abs(vn1 - cp1.velocityBias) < k_errorTol); #endif
                            */
                            if (DEBUG_SOLVER)
                            {
                                // Postconditions
                                Vec2 _dv1 = vB.Add(Vec2.Cross(wB, cp1.rB).SubLocal(vA).SubLocal(Vec2.Cross(wA, cp1.rA)));
                                // Compute normal velocity
                                vn1 = Vec2.Dot(_dv1, normal);

                                Debug.Assert(MathUtils.Abs(vn1 - cp1.VelocityBias) < ERROR_TO_I);
                            }
                            break;
                        }


                        //
                        // Case 3: wB = 0 and x1 = 0
                        //
                        // vn1 = a11 * 0 + a12 * x2' + b1'
                        // 0 = a21 * 0 + a22 * x2' + '
                        //
                        x.X = 0.0f;
                        x.Y = (-cp2.NormalMass) * b.Y;
                        vn1 = vc.K.Ey.X * x.Y + b.X;
                        vn2 = 0.0f;

                        if (x.Y >= 0.0f && vn1 >= 0.0f)
                        {
                            //Console.WriteLine("case 3");
                            // Resubstitute for the incremental impulse
                            d.Set(x).SubLocal(a);

                            // Apply incremental impulse
                            /*
                            * Vec2 P1 = d.x * normal; Vec2 P2 = d.y * normal; vA -= invMassA * (P1 + P2); wA -=
                            * invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
                            * 
                            * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
                            */

                            P1.Set(normal).MulLocal(d.X);
                            P2.Set(normal).MulLocal(d.Y);

                            temp1.Set(P1).AddLocal(P2);
                            temp2.Set(temp1).MulLocal(mA);
                            vA.SubLocal(temp2);
                            temp2.Set(temp1).MulLocal(mB);
                            vB.AddLocal(temp2);
                            //Debug.Assert(vA.x == 0);
                            //Debug.Assert(vB.x == 0);

                            wA -= iA * (Vec2.Cross(cp1.rA, P1) + Vec2.Cross(cp2.rA, P2));
                            wB += iB * (Vec2.Cross(cp1.rB, P1) + Vec2.Cross(cp2.rB, P2));

                            // Accumulate
                            //Debug.Assert(x.x == 0 && x.y == 0);
                            cp1.NormalImpulse = x.X;
                            cp2.NormalImpulse = x.Y;

                            /*
                            * #if B2_DEBUG_SOLVER == 1 // Postconditions dv2 = vB + Cross(wB, cp2.rB) - vA -
                            * Cross(wA, cp2.rA);
                            * 
                            * // Compute normal velocity vn2 = Dot(dv2, normal);
                            * 
                            * Debug.Assert(Abs(vn2 - cp2.velocityBias) < k_errorTol); #endif
                            */
                            if (DEBUG_SOLVER)
                            {
                                // Postconditions
                                Vec2 _dv2 =
                                    vB.Add(Vec2.Cross(wB, cp2.rB).SubLocal(vA).SubLocal(Vec2.Cross(wA, cp2.rA)));
                                // Compute normal velocity
                                vn2 = Vec2.Dot(_dv2, normal);

                                Debug.Assert(MathUtils.Abs(vn2 - cp2.VelocityBias) < ERROR_TO_I);
                            }
                            break;
                        }

                        //
                        // Case 4: x1 = 0 and x2 = 0
                        //
                        // vn1 = b1
                        // vn2 = ;
                        x.X = 0.0f;
                        x.Y = 0.0f;
                        vn1 = b.X;
                        vn2 = b.Y;

                        if (vn1 >= 0.0f && vn2 >= 0.0f)
                        {
                            //Console.WriteLine("case 4");
                            // Resubstitute for the incremental impulse
                            d.Set(x).SubLocal(a);

                            // Apply incremental impulse
                            /*
                            * Vec2 P1 = d.x * normal; Vec2 P2 = d.y * normal; vA -= invMassA * (P1 + P2); wA -=
                            * invIA * (Cross(cp1.rA, P1) + Cross(cp2.rA, P2));
                            * 
                            * vB += invMassB * (P1 + P2); wB += invIB * (Cross(cp1.rB, P1) + Cross(cp2.rB, P2));
                            */

                            P1.Set(normal).MulLocal(d.X);
                            P2.Set(normal).MulLocal(d.Y);

                            temp1.Set(P1).AddLocal(P2);
                            temp2.Set(temp1).MulLocal(mA);
                            vA.SubLocal(temp2);
                            temp2.Set(temp1).MulLocal(mB);
                            vB.AddLocal(temp2);
                            //Debug.Assert(vA.x == 0);
                            //Debug.Assert(vB.x == 0);

                            wA -= iA * (Vec2.Cross(cp1.rA, P1) + Vec2.Cross(cp2.rA, P2));
                            wB += iB * (Vec2.Cross(cp1.rB, P1) + Vec2.Cross(cp2.rB, P2));


                            // Accumulate
                            //Debug.Assert(x.x == 0 && x.y == 0);
                            cp1.NormalImpulse = x.X;
                            cp2.NormalImpulse = x.Y;

                            break;
                        }

                        // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
                        break;
                    }
                }

                Velocities[indexA].V.Set(vA);
                Velocities[indexA].W = wA;
                Velocities[indexB].V.Set(vB);
                Velocities[indexB].W = wB;

                //Console.WriteLine("Ending velocity for " + indexA + " is " + vA.x + "," + vA.y + " rot " + wA);
                //Console.WriteLine("Ending velocity for " + indexB + " is " + vB.x + "," + vB.y + " rot " + wB);
            }
        }

        public void StoreImpulses()
        {
            for (int i = 0; i < Count; i++)
            {
                ContactVelocityConstraint vc = VelocityConstraints[i];
                Manifold manifold = Contacts[vc.ContactIndex].Manifold;

                for (int j = 0; j < vc.PointCount; j++)
                {
                    manifold.Points[j].NormalImpulse = vc.Points[j].NormalImpulse;
                    manifold.Points[j].TangentImpulse = vc.Points[j].TangentImpulse;
                }
            }
        }

        /*
        * #if 0 // Sequential solver. bool ContactSolver::SolvePositionConstraints(float baumgarte) {
        * float minSeparation = 0.0f;
        * 
        * for (int i = 0; i < m_constraintCount; ++i) { ContactConstraint* c = m_constraints + i; Body*
        * bodyA = c.bodyA; Body* bodyB = c.bodyB; float invMassA = bodyA.m_mass * bodyA.m_invMass; float
        * invIA = bodyA.m_mass * bodyA.m_invI; float invMassB = bodyB.m_mass * bodyB.m_invMass; float
        * invIB = bodyB.m_mass * bodyB.m_invI;
        * 
        * Vec2 normal = c.normal;
        * 
        * // Solve normal constraints for (int j = 0; j < c.pointCount; ++j) { ContactConstraintPoint*
        * ccp = c.points + j;
        * 
        * Vec2 r1 = Mul(bodyA.GetXForm().R, ccp.localAnchorA - bodyA.GetLocalCenter()); Vec2 r2 =
        * Mul(bodyB.GetXForm().R, ccp.localAnchorB - bodyB.GetLocalCenter());
        * 
        * Vec2 p1 = bodyA.m_sweep.c + r1; Vec2 p2 = bodyB.m_sweep.c + r2; Vec2 dp = p2 - p1;
        * 
        * // Approximate the current separation. float separation = Dot(dp, normal) + ccp.separation;
        * 
        * // Track max constraint error. minSeparation = Min(minSeparation, separation);
        * 
        * // Prevent large corrections and allow slop. float C = Clamp(baumgarte * (separation +
        * _linearSlop), -_maxLinearCorrection, 0.0f);
        * 
        * // Compute normal impulse float impulse = -ccp.equalizedMass * C;
        * 
        * Vec2 P = impulse * normal;
        * 
        * bodyA.m_sweep.c -= invMassA * P; bodyA.m_sweep.a -= invIA * Cross(r1, P);
        * bodyA.SynchronizeTransform();
        * 
        * bodyB.m_sweep.c += invMassB * P; bodyB.m_sweep.a += invIB * Cross(r2, P);
        * bodyB.SynchronizeTransform(); } }
        * 
        * // We can't expect minSpeparation >= -_linearSlop because we don't // push the separation above
        * -_linearSlop. return minSeparation >= -1.5f * _linearSlop; }
        */

        // djm pooling, and from above
        private readonly PositionSolverManifold psolver = new PositionSolverManifold();
        private readonly Vec2 rA = new Vec2();
        private readonly Vec2 rB = new Vec2();

        /// <summary>
        /// Sequential solver.
        /// </summary>
        public bool SolvePositionConstraints()
        {
            float minSeparation = 0.0f;

            for (int i = 0; i < Count; ++i)
            {
                ContactPositionConstraint pc = PositionConstraints[i];

                int indexA = pc.IndexA;
                int indexB = pc.IndexB;

                float mA = pc.InvMassA;
                float iA = pc.InvIA;
                Vec2 localCenterA = pc.LocalCenterA;
                float mB = pc.InvMassB;
                float iB = pc.InvIB;
                Vec2 localCenterB = pc.LocalCenterB;
                int pointCount = pc.PointCount;

                Vec2 cA = Positions[indexA].C;
                float aA = Positions[indexA].A;
                Vec2 cB = Positions[indexB].C;
                float aB = Positions[indexB].A;
                //Console.WriteLine("cA: " + cA.x + "," + cA.y + " - rot " + aA);
                //Console.WriteLine("cB: " + cB.x + "," + cB.y + " - rot " + aB);

                // Solve normal constraints
                for (int j = 0; j < pointCount; ++j)
                {
                    xfA.Q.Set(aA);
                    xfB.Q.Set(aB);
                    Rot.MulToOutUnsafe(xfA.Q, localCenterA, xfA.P);
                    xfA.P.NegateLocal().AddLocal(cA);
                    Rot.MulToOutUnsafe(xfB.Q, localCenterB, xfB.P);
                    xfB.P.NegateLocal().AddLocal(cB);

                    PositionSolverManifold psm = psolver;
                    psm.Initialize(pc, xfA, xfB, j);
                    Vec2 normal = psm.Normal;

                    Vec2 point = psm.Point;
                    float separation = psm.Separation;

                    rA.Set(point).SubLocal(cA);
                    rB.Set(point).SubLocal(cB);

                    // Track max constraint error.
                    minSeparation = MathUtils.Min(minSeparation, separation);

                    // Prevent large corrections and allow slop.
                    float C = MathUtils.Clamp(Settings.BAUMGARTE * (separation + Settings.LINEAR_SLOP), -Settings.MAX_LINEAR_CORRECTION, 0.0f);

                    // Compute the effective mass.
                    float rnA = Vec2.Cross(rA, normal);
                    float rnB = Vec2.Cross(rB, normal);
                    float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                    // Compute normal impulse
                    float impulse = K > 0.0f ? (-C) / K : 0.0f;

                    P.Set(normal).MulLocal(impulse);

                    cA.SubLocal(temp.Set(P).MulLocal(mA));
                    aA -= iA * Vec2.Cross(rA, P);


                    cB.AddLocal(temp.Set(P).MulLocal(mB));
                    aB += iB * Vec2.Cross(rB, P);
                }

                Positions[indexA].C.Set(cA);
                Positions[indexA].A = aA;

                Positions[indexB].C.Set(cB);
                Positions[indexB].A = aB;
                //Console.WriteLine("ending pos "+indexA+": " + cA.x + "," + cA.y + " - rot " + aA);
                //Console.WriteLine("ending pos "+indexB+": " + cB.x + "," + cB.y + " - rot " + aB);
            }

            // We can't expect minSpeparation >= -linearSlop because we don't
            // push the separation above -linearSlop.
            return minSeparation >= (-3.0f) * Settings.LINEAR_SLOP;
        }

        // Sequential position solver for position constraints.
        public bool SolveTOIPositionConstraints(int toiIndexA, int toiIndexB)
        {
            float minSeparation = 0.0f;

            for (int i = 0; i < Count; ++i)
            {
                ContactPositionConstraint pc = PositionConstraints[i];

                int indexA = pc.IndexA;
                int indexB = pc.IndexB;
                Vec2 localCenterA = pc.LocalCenterA;
                Vec2 localCenterB = pc.LocalCenterB;
                int pointCount = pc.PointCount;

                float mA = 0.0f;
                float iA = 0.0f;
                if (indexA == toiIndexA || indexA == toiIndexB)
                {
                    mA = pc.InvMassA;
                    iA = pc.InvIA;
                }

                float mB = pc.InvMassB;
                float iB = pc.InvIB;
                if (indexB == toiIndexA || indexB == toiIndexB)
                {
                    mB = pc.InvMassB;
                    iB = pc.InvIB;
                }

                Vec2 cA = Positions[indexA].C;
                float aA = Positions[indexA].A;

                Vec2 cB = Positions[indexB].C;
                float aB = Positions[indexB].A;

                // Solve normal constraints
                for (int j = 0; j < pointCount; ++j)
                {
                    xfA.Q.Set(aA);
                    xfB.Q.Set(aB);
                    Rot.MulToOutUnsafe(xfA.Q, localCenterA, xfA.P);
                    xfA.P.NegateLocal().AddLocal(cA);
                    Rot.MulToOutUnsafe(xfB.Q, localCenterB, xfB.P);
                    xfB.P.NegateLocal().AddLocal(cB);

                    PositionSolverManifold psm = psolver;
                    psm.Initialize(pc, xfA, xfB, j);
                    Vec2 normal = psm.Normal;

                    Vec2 point = psm.Point;
                    float separation = psm.Separation;

                    rA.Set(point).SubLocal(cA);
                    rB.Set(point).SubLocal(cB);

                    // Track max constraint error.
                    minSeparation = MathUtils.Min(minSeparation, separation);

                    // Prevent large corrections and allow slop.
                    float C = MathUtils.Clamp(Settings.TOI_BAUGARTE * (separation + Settings.LINEAR_SLOP), -Settings.MAX_LINEAR_CORRECTION, 0.0f);

                    // Compute the effective mass.
                    float rnA = Vec2.Cross(rA, normal);
                    float rnB = Vec2.Cross(rB, normal);
                    float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                    // Compute normal impulse
                    float impulse = K > 0.0f ? (-C) / K : 0.0f;

                    P.Set(normal).MulLocal(impulse);

                    cA.SubLocal(temp.Set(P).MulLocal(mA));
                    aA -= iA * Vec2.Cross(rA, P);

                    cB.AddLocal(temp.Set(P).MulLocal(mB));
                    aB += iB * Vec2.Cross(rB, P);
                }

                Positions[indexA].C.Set(cA);
                Positions[indexA].A = aA;

                Positions[indexB].C.Set(cB);
                Positions[indexB].A = aB;
            }

            // We can't expect minSpeparation >= -_linearSlop because we don't
            // push the separation above -_linearSlop.
            return minSeparation >= (-1.5f) * Settings.LINEAR_SLOP;
        }

        public class ContactSolverDef
        {
            public TimeStep Step;
            public Contact[] Contacts;
            public int Count;
            public Position[] Positions;
            public Velocity[] Velocities;
        }
    }


    class PositionSolverManifold
    {
        public readonly Vec2 Normal = new Vec2();
        public readonly Vec2 Point = new Vec2();
        public float Separation;

        // djm pooling
        private readonly Vec2 pointA = new Vec2();
        private readonly Vec2 pointB = new Vec2();
        private readonly Vec2 temp = new Vec2();
        private readonly Vec2 planePoint = new Vec2();
        private readonly Vec2 clipPoint = new Vec2();

        public void Initialize(ContactPositionConstraint pc, Transform xfA, Transform xfB, int index)
        {
            Debug.Assert(pc.PointCount > 0);

            switch (pc.Type)
            {

                case Manifold.ManifoldType.Circles:
                    {
                        Transform.MulToOutUnsafe(xfA, pc.LocalPoint, pointA);
                        Transform.MulToOutUnsafe(xfB, pc.LocalPoints[0], pointB);
                        Normal.Set(pointB).SubLocal(pointA);
                        Normal.Normalize();

                        Point.Set(pointA).AddLocal(pointB).MulLocal(.5f);
                        temp.Set(pointB).SubLocal(pointA);
                        Separation = Vec2.Dot(temp, Normal) - pc.RadiusA - pc.RadiusB;
                        break;
                    }


                case Manifold.ManifoldType.FaceA:
                    {
                        Rot.MulToOutUnsafe(xfA.Q, pc.LocalNormal, Normal);
                        Transform.MulToOutUnsafe(xfA, pc.LocalPoint, planePoint);

                        Transform.MulToOutUnsafe(xfB, pc.LocalPoints[index], clipPoint);
                        temp.Set(clipPoint).SubLocal(planePoint);
                        Separation = Vec2.Dot(temp, Normal) - pc.RadiusA - pc.RadiusB;
                        Point.Set(clipPoint);
                        break;
                    }


                case Manifold.ManifoldType.FaceB:
                    {
                        Rot.MulToOutUnsafe(xfB.Q, pc.LocalNormal, Normal);
                        Transform.MulToOutUnsafe(xfB, pc.LocalPoint, planePoint);

                        Transform.MulToOutUnsafe(xfA, pc.LocalPoints[index], clipPoint);
                        temp.Set(clipPoint).SubLocal(planePoint);
                        Separation = Vec2.Dot(temp, Normal) - pc.RadiusA - pc.RadiusB;
                        Point.Set(clipPoint);

                        // Ensure normal points from A to B
                        Normal.NegateLocal();
                    }
                    break;
            }
        }
    }
}