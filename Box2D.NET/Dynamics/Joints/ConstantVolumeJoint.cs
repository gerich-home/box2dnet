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
        public readonly Body[] bodies;
        internal float[] targetLengths;
        public float targetVolume;
        // float relaxationFactor;//1.0 is perfectly stiff (but doesn't work, unstable)

        internal Vec2[] normals;

        internal TimeStep m_step;
        private float m_impulse = 0.0f;

        private World world;

        internal DistanceJoint[] distanceJoints;

        public readonly float frequencyHz;
        public readonly float dampingRatio;

        virtual public Body[] Bodies
        {
            get
            {
                return bodies;
            }
        }

        virtual public DistanceJoint[] Joints
        {
            get
            {
                return distanceJoints;
            }
        }

        private float Area
        {
            get
            {
                float area = 0.0f;
                // i'm glad i changed these all to member access
                area += bodies[bodies.Length - 1].WorldCenter.X * bodies[0].WorldCenter.Y - bodies[0].WorldCenter.X * bodies[bodies.Length - 1].WorldCenter.Y;
                for (int i = 0; i < bodies.Length - 1; ++i)
                {
                    area += bodies[i].WorldCenter.X * bodies[i + 1].WorldCenter.Y - bodies[i + 1].WorldCenter.X * bodies[i].WorldCenter.Y;
                }
                area *= .5f;
                return area;
            }
        }

        public virtual void inflate(float factor)
        {
            targetVolume *= factor;
        }

        public ConstantVolumeJoint(World argWorld, ConstantVolumeJointDef def)
            : base(argWorld.Pool, def)
        {
            world = argWorld;
            if (def.bodies.Count <= 2)
            {
                throw new ArgumentException("You cannot create a constant volume joint with less than three bodies.");
            }
            bodies = def.bodies.ToArray();

            targetLengths = new float[bodies.Length];
            for (int i = 0; i < targetLengths.Length; ++i)
            {
                int next = (i == targetLengths.Length - 1) ? 0 : i + 1;
                float dist = bodies[i].WorldCenter.Sub(bodies[next].WorldCenter).Length();
                targetLengths[i] = dist;
            }
            targetVolume = Area;

            if (def.joints != null && def.joints.Count != def.bodies.Count)
            {
                throw new ArgumentException("Incorrect joint definition.  Joints have to correspond to the bodies");
            }
            if (def.joints == null)
            {
                DistanceJointDef djd = new DistanceJointDef();
                distanceJoints = new DistanceJoint[bodies.Length];
                for (int i = 0; i < targetLengths.Length; ++i)
                {
                    int next = (i == targetLengths.Length - 1) ? 0 : i + 1;
                    djd.frequencyHz = def.frequencyHz; // 20.0f;
                    djd.dampingRatio = def.dampingRatio; // 50.0f;
                    djd.initialize(bodies[i], bodies[next], bodies[i].WorldCenter, bodies[next].WorldCenter);
                    distanceJoints[i] = (DistanceJoint)world.CreateJoint(djd);
                }
            }
            else
            {
                distanceJoints = def.joints.ToArray();
            }

            frequencyHz = def.frequencyHz;
            dampingRatio = def.dampingRatio;

            normals = new Vec2[bodies.Length];
            for (int i = 0; i < normals.Length; ++i)
            {
                normals[i] = new Vec2();
            }

            this.m_bodyA = bodies[0];
            this.m_bodyB = bodies[1];
            this.m_collideConnected = false;
        }


        public override void destructor()
        {
            for (int i = 0; i < distanceJoints.Length; ++i)
            {
                world.DestroyJoint(distanceJoints[i]);
            }
        }

        /// <summary>
        /// Apply the position correction to the particles.
        /// </summary>
        /// <param name="step"></param>
        public virtual bool constrainEdges(TimeStep step)
        {
            float perimeter = 0.0f;
            for (int i = 0; i < bodies.Length; ++i)
            {
                int next = (i == bodies.Length - 1) ? 0 : i + 1;
                float dx = bodies[next].WorldCenter.X - bodies[i].WorldCenter.X;
                float dy = bodies[next].WorldCenter.Y - bodies[i].WorldCenter.Y;
                float dist = MathUtils.Sqrt(dx * dx + dy * dy);
                if (dist < Settings.EPSILON)
                {
                    dist = 1.0f;
                }
                normals[i].X = dy / dist;
                normals[i].Y = (-dx) / dist;
                perimeter += dist;
            }

            Vec2 delta = pool.PopVec2();

            float deltaArea = targetVolume - Area;
            float toExtrude = 0.5f * deltaArea / perimeter; // *relaxationFactor
            // float sumdeltax = 0.0f;
            bool done = true;
            for (int i = 0; i < bodies.Length; ++i)
            {
                int next = (i == bodies.Length - 1) ? 0 : i + 1;
                delta.Set(toExtrude * (normals[i].X + normals[next].X), toExtrude * (normals[i].Y + normals[next].Y));
                // sumdeltax += dx;
                float norm = delta.Length();
                if (norm > Settings.maxLinearCorrection)
                {
                    delta.MulLocal(Settings.maxLinearCorrection / norm);
                }
                if (norm > Settings.linearSlop)
                {
                    done = false;
                }
                bodies[next].Sweep.c.X += delta.X;
                bodies[next].Sweep.c.Y += delta.Y;
                bodies[next].SynchronizeTransform();
                // bodies[next].m_linearVelocity.x += delta.x * step.inv_dt;
                // bodies[next].m_linearVelocity.y += delta.y * step.inv_dt;
            }

            pool.PushVec2(1);
            // System.out.println(sumdeltax);
            return done;
        }

        public override void initVelocityConstraints(SolverData data)
        {
            Vec2[] d = pool.GetVec2Array(bodies.Length);

            for (int i = 0; i < bodies.Length; ++i)
            {
                int prev = (i == 0) ? bodies.Length - 1 : i - 1;
                int next = (i == bodies.Length - 1) ? 0 : i + 1;
                d[i].Set(bodies[next].WorldCenter);
                d[i].SubLocal(bodies[prev].WorldCenter);
            }

            if (data.Step.WarmStarting)
            {
                m_impulse *= data.Step.DtRatio;
                // float lambda = -2.0f * crossMassSum / dotMassSum;
                // System.out.println(crossMassSum + " " +dotMassSum);
                // lambda = MathUtils.clamp(lambda, -Settings.maxLinearCorrection,
                // Settings.maxLinearCorrection);
                // m_impulse = lambda;
                for (int i = 0; i < bodies.Length; ++i)
                {
                    bodies[i].m_linearVelocity.X += bodies[i].InvMass * d[i].Y * .5f * m_impulse;
                    bodies[i].m_linearVelocity.Y += bodies[i].InvMass * (-d[i].X) * .5f * m_impulse;
                }
            }
            else
            {
                m_impulse = 0.0f;
            }
        }

        public override bool solvePositionConstraints(SolverData data)
        {
            return constrainEdges(data.Step);
        }

        public override void solveVelocityConstraints(SolverData data)
        {
            float crossMassSum = 0.0f;
            float dotMassSum = 0.0f;


            Vec2[] d = pool.GetVec2Array(bodies.Length);

            for (int i = 0; i < bodies.Length; ++i)
            {
                int prev = (i == 0) ? bodies.Length - 1 : i - 1;
                int next = (i == bodies.Length - 1) ? 0 : i + 1;
                d[i].Set(bodies[next].WorldCenter);
                d[i].SubLocal(bodies[prev].WorldCenter);
                dotMassSum += (d[i].LengthSquared()) / bodies[i].Mass;
                crossMassSum += Vec2.Cross(bodies[i].LinearVelocity, d[i]);
            }
            float lambda = (-2.0f) * crossMassSum / dotMassSum;
            // System.out.println(crossMassSum + " " +dotMassSum);
            // lambda = MathUtils.clamp(lambda, -Settings.maxLinearCorrection,
            // Settings.maxLinearCorrection);
            m_impulse += lambda;
            // System.out.println(m_impulse);
            for (int i = 0; i < bodies.Length; ++i)
            {
                bodies[i].m_linearVelocity.X += bodies[i].InvMass * d[i].Y * .5f * lambda;
                bodies[i].m_linearVelocity.Y += bodies[i].InvMass * (-d[i].X) * .5f * lambda;
            }
        }

        public override void getAnchorA(Vec2 argOut)
        {
        }

        public override void getAnchorB(Vec2 argOut)
        {
        }

        public override void getReactionForce(float inv_dt, Vec2 argOut)
        {
        }

        public override float getReactionTorque(float inv_dt)
        {
            return 0;
        }
    }
}