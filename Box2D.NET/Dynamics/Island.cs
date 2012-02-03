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
using Box2D.Callbacks;
using Box2D.Common;
using Box2D.Dynamics.Contacts;
using Box2D.Dynamics.Joints;

namespace Box2D.Dynamics
{

	/*
	Position Correction Notes
	=========================
	I tried the several algorithms for position correction of the 2D revolute joint.
	I looked at these systems:
	- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
	- suspension bridge with 30 1m long planks of length 1m.
	- multi-link chain with 30 1m long links.
	
	Here are the algorithms:
	
	Baumgarte - A fraction of the position error is added to the velocity error. There is no
	separate position solver.
	
	Pseudo Velocities - After the velocity solver and position integration,
	the position error, Jacobian, and effective mass are recomputed. Then
	the velocity constraints are solved with pseudo velocities and a fraction
	of the position error is added to the pseudo velocity error. The pseudo
	velocities are initialized to zero and there is no warm-starting. After
	the position solver, the pseudo velocities are added to the positions.
	This is also called the First Order World method or the Position LCP method.
	
	Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
	position error is re-computed for each raint and the positions are updated
	after the raint is solved. The radius vectors (aka Jacobians) are
	re-computed too (otherwise the algorithm has horrible instability). The pseudo
	velocity states are not needed because they are effectively zero at the beginning
	of each iteration. Since we have the current position error, we allow the
	iterations to terminate early if the error becomes smaller than Settings.linearSlop.
	
	Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
	each time a raint is solved.
	
	Here are the results:
	Baumgarte - this is the cheapest algorithm but it has some stability problems,
	especially with the bridge. The chain links separate easily close to the root
	and they jitter as they struggle to pull together. This is one of the most common
	methods in the field. The big drawback is that the position correction artificially
	affects the momentum, thus leading to instabilities and false bounce. I used a
	bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
	factor makes joints and contacts more spongy.
	
	Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
	stable. However, joints still separate with large angular velocities. Drag the
	simple pendulum in a circle quickly and the joint will separate. The chain separates
	easily and does not recover. I used a bias factor of 0.2. A larger value lead to
	the bridge collapsing when a heavy cube drops on it.
	
	Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
	Velocities, but in other ways it is worse. The bridge and chain are much more
	stable, but the simple pendulum goes unstable at high angular velocities.
	
	Full NGS - stable in all tests. The joints display good stiffness. The bridge
	still sags, but this is better than infinite forces.
	
	Recommendations
	Pseudo Velocities are not really worthwhile because the bridge and chain cannot
	recover from joint separation. In other cases the benefit over Baumgarte is small.
	
	Modified NGS is not a robust method for the revolute joint due to the violent
	instability seen in the simple pendulum. Perhaps it is viable with other raint
	types, especially scalar constraints where the effective mass is a scalar.
	
	This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
	and is very fast. I don't think we can escape Baumgarte, especially in highly
	demanding cases where high raint fidelity is not needed.
	
	Full NGS is robust and easy on the eyes. I recommend this as an option for
	higher fidelity simulation and certainly for suspension bridges and long chains.
	Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
	joint separation can be problematic. The number of NGS iterations can be reduced
	for better performance without harming robustness much.
	
	Each joint in a can be handled differently in the position solver. So I recommend
	a system where the user can select the algorithm on a per joint basis. I would
	probably default to the slower Full NGS and let the user select the faster
	Baumgarte method in performance critical scenarios.
	*/

	/*
	Cache Performance
	
	The Box2D solvers are dominated by cache misses. Data structures are designed
	to increase the number of cache hits. Much of misses are due to random access
	to body data. The raint structures are iterated over linearly, which leads
	to few cache misses.
	
	The bodies are not accessed during iteration. Instead read only data, such as
	the mass values are stored with the constraints. The mutable data are the raint
	impulses and the bodies velocities/positions. The impulses are held inside the
	raint structures. The body velocities/positions are held in compact, temporary
	arrays to increase the number of cache hits. Linear and angular velocity are
	stored in a single array since multiple arrays lead to multiple misses.
	*/

	/*
	2D Rotation
	
	R = [cos(theta) -sin(theta)]
	[sin(theta) cos(theta) ]
	
	thetaDot = omega
	
	Let q1 = cos(theta), q2 = sin(theta).
	R = [q1 -q2]
	[q2  q1]
	
	q1Dot = -thetaDot * q2
	q2Dot = thetaDot * q1
	
	q1_new = q1_old - dt * w * q2
	q2_new = q2_old + dt * w * q1
	then normalize.
	
	This might be faster than computing sin+cos.
	However, we can compute sin+cos of the same angle fast.
	*/

	/// <summary>
	/// This is an internal class.
	/// </summary>
	/// <author>Daniel Murphy</author>
	public class Island
	{

		public IContactListener Listener;

		public Body[] Bodies;
		public Contact[] Contacts;
		public Joint[] Joints;

		public Position[] Positions;
		public Velocity[] Velocities;

		public int BodyCount;
		public int JointCount;
		public int ContactCount;

		public int BodyCapacity;
		public int ContactCapacity;
		public int JointCapacity;

	    public void Init(int bodyCapacity, int contactCapacity, int jointCapacity, IContactListener listener)
		{
			// Console.WriteLine("Initializing Island");
			BodyCapacity = bodyCapacity;
			ContactCapacity = contactCapacity;
			JointCapacity = jointCapacity;
			BodyCount = 0;
			ContactCount = 0;
			JointCount = 0;

			Listener = listener;

			if (Bodies == null || BodyCapacity > Bodies.Length)
			{
				Bodies = new Body[BodyCapacity];
			}
			if (Joints == null || JointCapacity > Joints.Length)
			{
				Joints = new Joint[JointCapacity];
			}
			if (Contacts == null || ContactCapacity > Contacts.Length)
			{
				Contacts = new Contact[ContactCapacity];
			}

			// dynamic array
			if (Velocities == null || BodyCapacity > Velocities.Length)
			{
				Velocity[] old = Velocities ?? new Velocity[0];
				Velocities = new Velocity[BodyCapacity];
				Array.Copy(old, 0, Velocities, 0, old.Length);
				for (int i = old.Length; i < Velocities.Length; i++)
				{
					Velocities[i] = new Velocity();
				}
			}

			// dynamic array
			if (Positions == null || BodyCapacity > Positions.Length)
			{
				Position[] old = Positions ?? new Position[0];
				Positions = new Position[BodyCapacity];
				Array.Copy(old, 0, Positions, 0, old.Length);
				for (int i = old.Length; i < Positions.Length; i++)
				{
					Positions[i] = new Position();
				}
			}
		}

		public void Clear()
		{
			BodyCount = 0;
			ContactCount = 0;
			JointCount = 0;
		}

		private readonly ContactSolver contactSolver = new ContactSolver();
		private readonly Vec2 translation = new Vec2();
		private readonly Timer timer = new Timer();
		private readonly SolverData solverData = new SolverData();
		private readonly ContactSolver.ContactSolverDef solverDef = new ContactSolver.ContactSolverDef();

		public void Solve(Profile profile, TimeStep step, Vec2 gravity, bool allowSleep)
		{
			// Console.WriteLine("Solving Island");

			float h = step.Dt;

			// Integrate velocities and apply damping. Initialize the body state.
			for (int i = 0; i < BodyCount; ++i)
			{
				Body b = Bodies[i];
				Vec2 c = b.Sweep.c;
				float a = b.Sweep.a;
				Vec2 v = b.m_linearVelocity;
				float w = b.m_angularVelocity;

				// Store positions for continuous collision.
				b.Sweep.c0.Set(b.Sweep.c);
				b.Sweep.a0 = b.Sweep.a;

				if (b.m_type == BodyType.Dynamic)
				{
					// Integrate velocities.
					// v += h * (b.m_gravityScale * gravity + b.m_invMass * b.m_force);
					v.X += h * (b.GravityScale * gravity.X + b.InvMass * b.Force.X);
					v.Y += h * (b.GravityScale * gravity.Y + b.InvMass * b.Force.Y);
					w += h * b.InvI * b.Torque;

					// Apply damping.
					// ODE: dv/dt + c * v = 0
					// Solution: v(t) = v0 * exp(-c * t)
					// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v *
					// exp(-c * dt)
					// v2 = exp(-c * dt) * v1
					// Taylor expansion:
					// v2 = (1.0f - c * dt) * v1
					v.MulLocal(MathUtils.Clamp(1.0f - h * b.LinearDamping, 0.0f, 1.0f));
					w *= MathUtils.Clamp(1.0f - h * b.AngularDamping, 0.0f, 1.0f);
				}
				//Debug.Assert (v.x == 0);

				Positions[i].c.Set(c);
				Positions[i].a = a;
				Velocities[i].v.Set(v);
				Velocities[i].w = w;
			}

			timer.reset();

			// Solver data
			solverData.Step = step;
			solverData.Positions = Positions;
			solverData.Velocities = Velocities;

			// Initialize velocity constraints.
			solverDef.step = step;
			solverDef.contacts = Contacts;
			solverDef.count = ContactCount;
			solverDef.positions = Positions;
			solverDef.velocities = Velocities;

			contactSolver.init(solverDef);
			//Console.WriteLine("island init vel");
			contactSolver.initializeVelocityConstraints();

			if (step.WarmStarting)
			{
				//Console.WriteLine("island warm start");
				contactSolver.warmStart();
			}

			for (int i = 0; i < JointCount; ++i)
			{
				Joints[i].initVelocityConstraints(solverData);
			}

			profile.SolveInit = timer.Milliseconds;

			// Solve velocity constraints
			timer.reset();
			//Console.WriteLine("island solving velocities");
			for (int i = 0; i < step.VelocityIterations; ++i)
			{
				for (int j = 0; j < JointCount; ++j)
				{
					Joints[j].solveVelocityConstraints(solverData);
				}

				contactSolver.solveVelocityConstraints();
			}

			// Store impulses for warm starting
			contactSolver.storeImpulses();
			profile.SolveVelocity = timer.Milliseconds;

			// Integrate positions
			for (int i = 0; i < BodyCount; ++i)
			{
				Vec2 c = Positions[i].c;
				float a = Positions[i].a;
				Vec2 v = Velocities[i].v;
				float w = Velocities[i].w;

				// Check for large velocities
				translation.X = v.X * h;
				translation.Y = v.Y * h;

				if (Vec2.Dot(translation, translation) > Settings.MAX_TRANSLATION_SQUARED)
				{
					float ratio = Settings.MAX_TRANSLATION / translation.Length();
					v.X *= ratio;
					v.Y *= ratio;
				}

				float rotation = h * w;
				if (rotation * rotation > Settings.MaxRotationSquared)
				{
					float ratio = Settings.MAX_ROTATION / MathUtils.Abs(rotation);
					w *= ratio;
				}

				// Integrate
				c.X += h * v.X;
				c.Y += h * v.Y;
				a += h * w;

				Positions[i].a = a;
				Velocities[i].w = w;
			}

			// Solve position constraints
			timer.reset();
			bool positionSolved = false;
			for (int i = 0; i < step.PositionIterations; ++i)
			{
				bool contactsOkay = contactSolver.solvePositionConstraints();

				bool jointsOkay = true;
				for (int j = 0; j < JointCount; ++j)
				{
					bool jointOkay = Joints[j].solvePositionConstraints(solverData);
					jointsOkay = jointsOkay && jointOkay;
				}

				if (contactsOkay && jointsOkay)
				{
					// Exit early if the position errors are small.
					positionSolved = true;
					break;
				}
			}

			// Copy state buffers back to the bodies
			for (int i = 0; i < BodyCount; ++i)
			{
				Body body = Bodies[i];
				body.Sweep.c.Set(Positions[i].c);
				body.Sweep.a = Positions[i].a;
				body.m_linearVelocity.Set(Velocities[i].v);
				body.m_angularVelocity = Velocities[i].w;
				body.SynchronizeTransform();
			}

			profile.SolvePosition = timer.Milliseconds;

			Report(contactSolver.m_velocityConstraints);

			if (allowSleep)
			{
				float minSleepTime = Single.MaxValue;

				const float linTolSqr = Settings.LINEAR_SLEEP_TOLERANCE * Settings.LINEAR_SLEEP_TOLERANCE;
				float angTolSqr = Settings.ANGULAR_SLEEP_TOLERANCE * Settings.ANGULAR_SLEEP_TOLERANCE;

				for (int i = 0; i < BodyCount; ++i)
				{
					Body b = Bodies[i];
					if (b.Type == BodyType.Static)
					{
						continue;
					}

					if ((b.Flags & Body.TypeFlags.AutoSleep) == 0 || b.m_angularVelocity * b.m_angularVelocity > angTolSqr || Vec2.Dot(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr)
					{
						b.SleepTime = 0.0f;
						minSleepTime = 0.0f;
					}
					else
					{
						b.SleepTime += h;
						minSleepTime = MathUtils.Min(minSleepTime, b.SleepTime);
					}
				}

				if (minSleepTime >= Settings.TIME_TO_SLEEP && positionSolved)
				{
					for (int i = 0; i < BodyCount; ++i)
					{
						Body b = Bodies[i];
						b.Awake = false;
					}
				}
			}
		}

		private readonly ContactSolver toiContactSolver = new ContactSolver();
		private readonly ContactSolver.ContactSolverDef toiSolverDef = new ContactSolver.ContactSolverDef();

		public void SolveToi(TimeStep subStep, int toiIndexA, int toiIndexB)
		{
			Debug.Assert(toiIndexA < BodyCount);
			Debug.Assert(toiIndexB < BodyCount);

			// Initialize the body state.
			for (int i = 0; i < BodyCount; ++i)
			{
				Body b = Bodies[i];
				Positions[i].c.Set(b.Sweep.c);
				Positions[i].a = b.Sweep.a;
				Velocities[i].v.Set(b.m_linearVelocity);
				Velocities[i].w = b.m_angularVelocity;
			}

			toiSolverDef.contacts = Contacts;
			toiSolverDef.count = ContactCount;
			toiSolverDef.step = subStep;
			toiSolverDef.positions = Positions;
			toiSolverDef.velocities = Velocities;
			toiContactSolver.init(toiSolverDef);

			// Solve position constraints.
			for (int i = 0; i < subStep.PositionIterations; ++i)
			{
				bool contactsOkay = toiContactSolver.solveTOIPositionConstraints(toiIndexA, toiIndexB);
				if (contactsOkay)
				{
					break;
				}
			}

			// #if 0
			// // Is the new position really safe?
			// for (int i = 0; i < m_contactCount; ++i)
			// {
			// Contact* c = m_contacts[i];
			// Fixture* fA = c.GetFixtureA();
			// Fixture* fB = c.GetFixtureB();
			//
			// Body bA = fA.GetBody();
			// Body bB = fB.GetBody();
			//
			// int indexA = c.GetChildIndexA();
			// int indexB = c.GetChildIndexB();
			//
			// DistanceInput input;
			// input.proxyA.Set(fA.GetShape(), indexA);
			// input.proxyB.Set(fB.GetShape(), indexB);
			// input.transformA = bA.GetTransform();
			// input.transformB = bB.GetTransform();
			// input.useRadii = false;
			//
			// DistanceOutput output;
			// SimplexCache cache;
			// cache.count = 0;
			// Distance(&output, &cache, &input);
			//
			// if (output.distance == 0 || cache.count == 3)
			// {
			// cache.count += 0;
			// }
			// }
			// #endif

			// Leap of faith to new safe state.
			Bodies[toiIndexA].Sweep.c0.Set(Positions[toiIndexA].c);
			Bodies[toiIndexA].Sweep.a0 = Positions[toiIndexA].a;
			Bodies[toiIndexB].Sweep.c0.Set(Positions[toiIndexB].c);
			Bodies[toiIndexB].Sweep.a0 = Positions[toiIndexB].a;

			// No warm starting is needed for TOI events because warm
			// starting impulses were applied in the discrete solver.
			toiContactSolver.initializeVelocityConstraints();

			// Solve velocity constraints.
			for (int i = 0; i < subStep.VelocityIterations; ++i)
			{
				toiContactSolver.solveVelocityConstraints();
			}

			// Don't store the TOI contact forces for warm starting
			// because they can be quite large.

			float h = subStep.Dt;

			// Integrate positions
			for (int i = 0; i < BodyCount; ++i)
			{
				Vec2 c = Positions[i].c;
				float a = Positions[i].a;
				Vec2 v = Velocities[i].v;
				float w = Velocities[i].w;

				// Check for large velocities
				translation.Set(v).MulLocal(h);
				if (Vec2.Dot(translation, translation) > Settings.MAX_TRANSLATION_SQUARED)
				{
					float ratio = Settings.MAX_TRANSLATION / translation.Length();
					v.MulLocal(ratio);
				}

				float rotation = h * w;
				if (rotation * rotation > Settings.MaxRotationSquared)
				{
					float ratio = Settings.MAX_ROTATION / MathUtils.Abs(rotation);
					w *= ratio;
				}

				// Integrate
				c.X += v.X * h;
				c.Y += v.Y * h;
				a += h * w;

				Positions[i].c.Set(c);
				Positions[i].a = a;
				Velocities[i].v.Set(v);
				Velocities[i].w = w;

				// Sync bodies
				Body body = Bodies[i];
				body.Sweep.c.Set(c);
				body.Sweep.a = a;
				body.m_linearVelocity.Set(v);
				body.m_angularVelocity = w;
				body.SynchronizeTransform();
			}

			Report(toiContactSolver.m_velocityConstraints);
		}

		public void Add(Body body)
		{
			Debug.Assert(BodyCount < BodyCapacity);
			body.IslandIndex = BodyCount;
			Bodies[BodyCount] = body;
			++BodyCount;
		}

		public void Add(Contact contact)
		{
			Debug.Assert(ContactCount < ContactCapacity);
			Contacts[ContactCount++] = contact;
		}

		public void Add(Joint joint)
		{
			Debug.Assert(JointCount < JointCapacity);
			Joints[JointCount++] = joint;
		}

		private readonly ContactImpulse impulse = new ContactImpulse();

		public void Report(ContactVelocityConstraint[] constraints)
		{
			if (Listener == null)
			{
				return;
			}

			for (int i = 0; i < ContactCount; ++i)
			{
				Contact c = Contacts[i];

				ContactVelocityConstraint vc = constraints[i];
				impulse.Count = vc.pointCount;
				for (int j = 0; j < vc.pointCount; ++j)
				{
					impulse.NormalImpulses[j] = vc.points[j].normalImpulse;
					impulse.TangentImpulses[j] = vc.points[j].tangentImpulse;
				}

				Listener.PostSolve(c, impulse);
			}
		}
	}
}