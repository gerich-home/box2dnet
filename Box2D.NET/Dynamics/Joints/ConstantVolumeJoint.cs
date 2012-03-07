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
using Box2D.Common;

namespace Box2D.Dynamics.Joints
{
    // TODO(dmurph): clean this up a bit, add docs
    public class ConstantVolumeJoint : Joint
    {
        public readonly Body[] Bodies;
        internal float[] TargetLengths;
        public float TargetVolume;
        // float relaxationFactor;//1.0 is perfectly stiff (but doesn't work, unstable)

        internal Vec2[] Normals;

        internal TimeStep Step;
        private float m_impulse = 0.0f;

        private World world;

        internal DistanceJoint[] Joints;

        public readonly float FrequencyHz;
        public readonly float DampingRatio;

        public void Inflate(float factor)
        {
            TargetVolume *= factor;
        }

        public ConstantVolumeJoint(World argWorld, ConstantVolumeJointDef def)
            : base(argWorld.Pool, def)
        {
            world = argWorld;
            if (def.Bodies.Count <= 2)
            {
                throw new ArgumentException("You cannot create a constant volume joint with less than three bodies.");
            }
            Bodies = def.Bodies.ToArray();

            TargetLengths = new float[Bodies.Length];
            for (int i = 0; i < TargetLengths.Length; ++i)
            {
                int next = (i == TargetLengths.Length - 1) ? 0 : i + 1;
                float dist = Bodies[i].WorldCenter.Sub(Bodies[next].WorldCenter).Length();
                TargetLengths[i] = dist;
            }
            TargetVolume = Area;

            if (def.Joints != null && def.Joints.Count != def.Bodies.Count)
            {
                throw new ArgumentException("Incorrect joint definition.  Joints have to correspond to the bodies");
            }
            if (def.Joints == null)
            {
                DistanceJointDef djd = new DistanceJointDef();
                Joints = new DistanceJoint[Bodies.Length];
                for (int i = 0; i < TargetLengths.Length; ++i)
                {
                    int next = (i == TargetLengths.Length - 1) ? 0 : i + 1;
                    djd.frequencyHz = def.FrequencyHz; // 20.0f;
                    djd.dampingRatio = def.DampingRatio; // 50.0f;
                    djd.initialize(Bodies[i], Bodies[next], Bodies[i].WorldCenter, Bodies[next].WorldCenter);
                    Joints[i] = (DistanceJoint)world.CreateJoint(djd);
                }
            }
            else
            {
                Joints = def.Joints.ToArray();
            }

            FrequencyHz = def.FrequencyHz;
            DampingRatio = def.DampingRatio;

            Normals = new Vec2[Bodies.Length];
            for (int i = 0; i < Normals.Length; ++i)
            {
                Normals[i] = new Vec2();
            }

            this.BodyA = Bodies[0];
            this.BodyB = Bodies[1];
            this.CollideConnected = false;
        }


        public override void Destructor()
        {
            for (int i = 0; i < Joints.Length; ++i)
            {
                world.DestroyJoint(Joints[i]);
            }
        }

        private float Area
        {
            get
            {
                float area = 0.0f;
                // i'm glad i changed these all to member access
                area += Bodies[Bodies.Length - 1].WorldCenter.X * Bodies[0].WorldCenter.Y - Bodies[0].WorldCenter.X * Bodies[Bodies.Length - 1].WorldCenter.Y;
                for (int i = 0; i < Bodies.Length - 1; ++i)
                {
                    area += Bodies[i].WorldCenter.X * Bodies[i + 1].WorldCenter.Y - Bodies[i + 1].WorldCenter.X * Bodies[i].WorldCenter.Y;
                }
                area *= .5f;
                return area;
            }
        }

        /// <summary>
        /// Apply the position correction to the particles.
        /// </summary>
        /// <param name="step"></param>
        public bool ConstrainEdges(TimeStep step)
        {
            float perimeter = 0.0f;
            for (int i = 0; i < Bodies.Length; ++i)
            {
                int next = (i == Bodies.Length - 1) ? 0 : i + 1;
                float dx = Bodies[next].WorldCenter.X - Bodies[i].WorldCenter.X;
                float dy = Bodies[next].WorldCenter.Y - Bodies[i].WorldCenter.Y;
                float dist = MathUtils.Sqrt(dx * dx + dy * dy);
                if (dist < Settings.EPSILON)
                {
                    dist = 1.0f;
                }
                Normals[i].X = dy / dist;
                Normals[i].Y = (-dx) / dist;
                perimeter += dist;
            }

            Vec2 delta = Pool.PopVec2();

            float deltaArea = TargetVolume - Area;
            float toExtrude = 0.5f * deltaArea / perimeter; // *relaxationFactor
            // float sumdeltax = 0.0f;
            bool done = true;
            for (int i = 0; i < Bodies.Length; ++i)
            {
                int next = (i == Bodies.Length - 1) ? 0 : i + 1;
                delta.Set(toExtrude * (Normals[i].X + Normals[next].X), toExtrude * (Normals[i].Y + Normals[next].Y));
                // sumdeltax += dx;
                float norm = delta.Length();
                if (norm > Settings.MAX_LINEAR_CORRECTION)
                {
                    delta.MulLocal(Settings.MAX_LINEAR_CORRECTION / norm);
                }
                if (norm > Settings.LINEAR_SLOP)
                {
                    done = false;
                }
                Bodies[next].Sweep.C.X += delta.X;
                Bodies[next].Sweep.C.Y += delta.Y;
                Bodies[next].SynchronizeTransform();
                // bodies[next].m_linearVelocity.x += delta.x * step.inv_dt;
                // bodies[next].m_linearVelocity.y += delta.y * step.inv_dt;
            }

            Pool.PushVec2(1);
            // System.out.println(sumdeltax);
            return done;
        }

        public override void InitVelocityConstraints(SolverData data)
        {
            Vec2[] d = Pool.GetVec2Array(Bodies.Length);

            for (int i = 0; i < Bodies.Length; ++i)
            {
                int prev = (i == 0) ? Bodies.Length - 1 : i - 1;
                int next = (i == Bodies.Length - 1) ? 0 : i + 1;
                d[i].Set(Bodies[next].WorldCenter);
                d[i].SubLocal(Bodies[prev].WorldCenter);
            }

            if (data.Step.WarmStarting)
            {
                m_impulse *= data.Step.DtRatio;
                // float lambda = -2.0f * crossMassSum / dotMassSum;
                // System.out.println(crossMassSum + " " +dotMassSum);
                // lambda = MathUtils.clamp(lambda, -Settings.maxLinearCorrection,
                // Settings.maxLinearCorrection);
                // m_impulse = lambda;
                for (int i = 0; i < Bodies.Length; ++i)
                {
                    Bodies[i].LinearVelocity.X += Bodies[i].InvMass * d[i].Y * .5f * m_impulse;
                    Bodies[i].LinearVelocity.Y += Bodies[i].InvMass * (-d[i].X) * .5f * m_impulse;
                }
            }
            else
            {
                m_impulse = 0.0f;
            }
        }

        public override bool SolvePositionConstraints(SolverData data)
        {
            return ConstrainEdges(data.Step);
        }

        public override void SolveVelocityConstraints(SolverData data)
        {
            float crossMassSum = 0.0f;
            float dotMassSum = 0.0f;


            Vec2[] d = Pool.GetVec2Array(Bodies.Length);

            for (int i = 0; i < Bodies.Length; ++i)
            {
                int prev = (i == 0) ? Bodies.Length - 1 : i - 1;
                int next = (i == Bodies.Length - 1) ? 0 : i + 1;
                d[i].Set(Bodies[next].WorldCenter);
                d[i].SubLocal(Bodies[prev].WorldCenter);
                dotMassSum += (d[i].LengthSquared()) / Bodies[i].Mass;
                crossMassSum += Vec2.Cross(Bodies[i].LinearVelocity, d[i]);
            }
            float lambda = (-2.0f) * crossMassSum / dotMassSum;
            // System.out.println(crossMassSum + " " +dotMassSum);
            // lambda = MathUtils.clamp(lambda, -Settings.maxLinearCorrection,
            // Settings.maxLinearCorrection);
            m_impulse += lambda;
            // System.out.println(m_impulse);
            for (int i = 0; i < Bodies.Length; ++i)
            {
                Bodies[i].LinearVelocity.X += Bodies[i].InvMass * d[i].Y * .5f * lambda;
                Bodies[i].LinearVelocity.Y += Bodies[i].InvMass * (-d[i].X) * .5f * lambda;
            }
        }

        public override void GetAnchorA(Vec2 argOut)
        {
        }

        public override void GetAnchorB(Vec2 argOut)
        {
        }

        public override void GetReactionForce(float inv_dt, Vec2 argOut)
        {
        }

        public override float GetReactionTorque(float inv_dt)
        {
            return 0;
        }
    }
}