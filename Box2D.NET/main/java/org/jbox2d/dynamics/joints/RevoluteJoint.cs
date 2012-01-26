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
using Mat22 = org.jbox2d.common.Mat22;
using Mat33 = org.jbox2d.common.Mat33;
using MathUtils = org.jbox2d.common.MathUtils;
using Rot = org.jbox2d.common.Rot;
using Settings = org.jbox2d.common.Settings;
using Vec2 = org.jbox2d.common.Vec2;
using Vec3 = org.jbox2d.common.Vec3;
using Body = org.jbox2d.dynamics.Body;
using SolverData = org.jbox2d.dynamics.SolverData;
using IWorldPool = org.jbox2d.pooling.IWorldPool;
namespace org.jbox2d.dynamics.joints
{
	
	//Point-to-point constraint
	//C = p2 - p1
	//Cdot = v2 - v1
	//   = v2 + cross(w2, r2) - v1 - cross(w1, r1)
	//J = [-I -r1_skew I r2_skew ]
	//Identity used:
	//w k % (rx i + ry j) = w * (-ry i + rx j)
	
	//Motor constraint
	//Cdot = w2 - w1
	//J = [0 0 -1 0 0 1]
	//K = invI1 + invI2
	
	/// <summary> A revolute joint constrains two bodies to share a common point while they are free to rotate
	/// about the point. The relative rotation about the shared point is the joint angle. You can limit
	/// the relative rotation with a joint limit that specifies a lower and upper angle. You can use a
	/// motor to drive the relative rotation about the shared point. A maximum motor torque is provided
	/// so that infinite forces are not generated.
	/// 
	/// </summary>
	/// <author>  Daniel Murphy
	/// </author>
	public class RevoluteJoint:Joint
	{
		virtual public float JointAngle
		{
			get
			{
				//UPGRADE_NOTE: Final was removed from the declaration of 'b1 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
				Body b1 = m_bodyA;
				//UPGRADE_NOTE: Final was removed from the declaration of 'b2 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
				Body b2 = m_bodyB;
				return b2.m_sweep.a - b1.m_sweep.a - m_referenceAngle;
			}
			
		}
		virtual public float JointSpeed
		{
			get
			{
				//UPGRADE_NOTE: Final was removed from the declaration of 'b1 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
				Body b1 = m_bodyA;
				//UPGRADE_NOTE: Final was removed from the declaration of 'b2 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
				Body b2 = m_bodyB;
				return b2.m_angularVelocity - b1.m_angularVelocity;
			}
			
		}
		virtual public bool MotorEnabled
		{
			get
			{
				return m_enableMotor;
			}
			
		}
		virtual public float MotorSpeed
		{
			set
			{
				m_bodyA.Awake = true;
				m_bodyB.Awake = true;
				m_motorSpeed = value;
			}
			
		}
		virtual public float MaxMotorTorque
		{
			set
			{
				m_bodyA.Awake = true;
				m_bodyB.Awake = true;
				m_maxMotorTorque = value;
			}
			
		}
		virtual public bool LimitEnabled
		{
			get
			{
				return m_enableLimit;
			}
			
		}
		virtual public float LowerLimit
		{
			get
			{
				return m_lowerAngle;
			}
			
		}
		virtual public float UpperLimit
		{
			get
			{
				return m_upperAngle;
			}
			
		}
		
		// Solver shared
		//UPGRADE_NOTE: Final was removed from the declaration of 'm_localAnchorA '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Vec2 m_localAnchorA = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'm_localAnchorB '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Vec2 m_localAnchorB = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'm_impulse '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Vec3 m_impulse = new Vec3();
		public float m_motorImpulse;
		
		public bool m_enableMotor;
		public float m_maxMotorTorque;
		public float m_motorSpeed;
		
		public bool m_enableLimit;
		public float m_referenceAngle;
		public float m_lowerAngle;
		public float m_upperAngle;
		
		// Solver temp
		public int m_indexA;
		public int m_indexB;
		//UPGRADE_NOTE: Final was removed from the declaration of 'm_rA '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Vec2 m_rA = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'm_rB '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Vec2 m_rB = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'm_localCenterA '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Vec2 m_localCenterA = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'm_localCenterB '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Vec2 m_localCenterB = new Vec2();
		public float m_invMassA;
		public float m_invMassB;
		public float m_invIA;
		public float m_invIB;
		//UPGRADE_NOTE: Final was removed from the declaration of 'm_mass '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Mat33 m_mass = new Mat33(); // effective mass for point-to-point constraint.
		public float m_motorMass; // effective mass for motor/limit angular constraint.
		public LimitState m_limitState;
		
		public RevoluteJoint(IWorldPool argWorld, RevoluteJointDef def):base(argWorld, def)
		{
			m_localAnchorA.set_Renamed(def.localAnchorA);
			m_localAnchorB.set_Renamed(def.localAnchorB);
			m_referenceAngle = def.referenceAngle;
			
			m_motorImpulse = 0;
			
			m_lowerAngle = def.lowerAngle;
			m_upperAngle = def.upperAngle;
			m_maxMotorTorque = def.maxMotorTorque;
			m_motorSpeed = def.motorSpeed;
			m_enableLimit = def.enableLimit;
			m_enableMotor = def.enableMotor;
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public override void  initVelocityConstraints(SolverData data)
		{
			m_indexA = m_bodyA.m_islandIndex;
			m_indexB = m_bodyB.m_islandIndex;
			m_localCenterA.set_Renamed(m_bodyA.m_sweep.localCenter);
			m_localCenterB.set_Renamed(m_bodyB.m_sweep.localCenter);
			m_invMassA = m_bodyA.m_invMass;
			m_invMassB = m_bodyB.m_invMass;
			m_invIA = m_bodyA.m_invI;
			m_invIB = m_bodyB.m_invI;
			
			// Vec2 cA = data.positions[m_indexA].c;
			float aA = data.positions[m_indexA].a;
			Vec2 vA = data.velocities[m_indexA].v;
			float wA = data.velocities[m_indexA].w;
			
			assert(!System.Single.IsNaN(wA));
			// Vec2 cB = data.positions[m_indexB].c;
			float aB = data.positions[m_indexB].a;
			Vec2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;
			assert(!System.Single.IsNaN(wB));
			
			//UPGRADE_NOTE: Final was removed from the declaration of 'qA '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Rot qA = pool.popRot();
			//UPGRADE_NOTE: Final was removed from the declaration of 'qB '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Rot qB = pool.popRot();
			//UPGRADE_NOTE: Final was removed from the declaration of 'temp '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Vec2 temp = pool.popVec2();
			
			qA.set_Renamed(aA);
			qB.set_Renamed(aB);
			
			// Compute the effective masses.
			Rot.mulToOutUnsafe(qA, temp.set_Renamed(m_localAnchorA).subLocal(m_localCenterA), m_rA);
			Rot.mulToOutUnsafe(qB, temp.set_Renamed(m_localAnchorB).subLocal(m_localCenterB), m_rB);
			
			// J = [-I -r1_skew I r2_skew]
			// [ 0 -1 0 1]
			// r_skew = [-ry; rx]
			
			// Matlab
			// K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x, -r1y*iA-r2y*iB]
			// [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
			// [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]
			
			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;
			
			bool fixedRotation = (iA + iB == 0.0f);
			
			m_mass.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
			m_mass.ey.x = (- m_rA.y) * m_rA.x * iA - m_rB.y * m_rB.x * iB;
			m_mass.ez.x = (- m_rA.y) * iA - m_rB.y * iB;
			m_mass.ex.y = m_mass.ey.x;
			m_mass.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
			m_mass.ez.y = m_rA.x * iA + m_rB.x * iB;
			m_mass.ex.z = m_mass.ez.x;
			m_mass.ey.z = m_mass.ez.y;
			m_mass.ez.z = iA + iB;
			
			m_motorMass = iA + iB;
			if (m_motorMass > 0.0f)
			{
				m_motorMass = 1.0f / m_motorMass;
			}
			
			if (m_enableMotor == false || fixedRotation)
			{
				m_motorImpulse = 0.0f;
			}
			
			if (m_enableLimit && fixedRotation == false)
			{
				float jointAngle = aB - aA - m_referenceAngle;
				if (MathUtils.abs(m_upperAngle - m_lowerAngle) < 2.0f * Settings.angularSlop)
				{
					m_limitState = LimitState.EQUAL;
				}
				else if (jointAngle <= m_lowerAngle)
				{
					if (m_limitState != LimitState.AT_LOWER)
					{
						m_impulse.z = 0.0f;
					}
					m_limitState = LimitState.AT_LOWER;
				}
				else if (jointAngle >= m_upperAngle)
				{
					if (m_limitState != LimitState.AT_UPPER)
					{
						m_impulse.z = 0.0f;
					}
					m_limitState = LimitState.AT_UPPER;
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
			}
			
			if (data.step.warmStarting)
			{
				//UPGRADE_NOTE: Final was removed from the declaration of 'P '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
				Vec2 P = pool.popVec2();
				// Scale impulses to support a variable time step.
				m_impulse.x *= data.step.dtRatio;
				m_impulse.y *= data.step.dtRatio;
				m_motorImpulse *= data.step.dtRatio;
				
				P.x = m_impulse.x;
				P.y = m_impulse.y;
				
				vA.x -= mA * P.x;
				vA.y -= mA * P.y;
				wA -= iA * (Vec2.cross(m_rA, P) + m_motorImpulse + m_impulse.z);
				assert(!System.Single.IsNaN(wA));
				
				vB.x += mB * P.x;
				vB.y += mB * P.y;
				wB += iB * (Vec2.cross(m_rB, P) + m_motorImpulse + m_impulse.z);
				assert(!System.Single.IsNaN(wB));
				pool.pushVec2(1);
			}
			else
			{
				m_impulse.setZero();
				m_motorImpulse = 0.0f;
			}
			
			assert(!System.Single.IsNaN(wA));
			assert(!System.Single.IsNaN(wB));
			data.velocities[m_indexA].v.set_Renamed(vA);
			data.velocities[m_indexA].w = wA;
			data.velocities[m_indexB].v.set_Renamed(vB);
			data.velocities[m_indexB].w = wB;
			
			
			pool.pushVec2(1);
			pool.pushRot(2);
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public override void  solveVelocityConstraints(SolverData data)
		{
			Vec2 vA = data.velocities[m_indexA].v;
			float wA = data.velocities[m_indexA].w;
			Vec2 vB = data.velocities[m_indexB].v;
			float wB = data.velocities[m_indexB].w;
			
			float mA = m_invMassA, mB = m_invMassB;
			float iA = m_invIA, iB = m_invIB;
			
			bool fixedRotation = (iA + iB == 0.0f);
			
			// Solve motor constraint.
			if (m_enableMotor && m_limitState != LimitState.EQUAL && fixedRotation == false)
			{
				float Cdot = wB - wA - m_motorSpeed;
				float impulse = (- m_motorMass) * Cdot;
				float oldImpulse = m_motorImpulse;
				float maxImpulse = data.step.dt * m_maxMotorTorque;
				m_motorImpulse = MathUtils.clamp(m_motorImpulse + impulse, - maxImpulse, maxImpulse);
				impulse = m_motorImpulse - oldImpulse;
				
				wA -= iA * impulse;
				wB += iB * impulse;
			}
			//UPGRADE_NOTE: Final was removed from the declaration of 'temp '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Vec2 temp = pool.popVec2();
			
			// Solve limit constraint.
			if (m_enableLimit && m_limitState != LimitState.INACTIVE && fixedRotation == false)
			{
				
				//UPGRADE_NOTE: Final was removed from the declaration of 'Cdot1 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
				Vec2 Cdot1 = pool.popVec2();
				//UPGRADE_NOTE: Final was removed from the declaration of 'Cdot '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
				Vec3 Cdot = pool.popVec3();
				
				// Solve point-to-point constraint
				Vec2.crossToOutUnsafe(wA, m_rA, temp);
				Vec2.crossToOutUnsafe(wB, m_rB, Cdot1);
				Cdot1.addLocal(vB).subLocal(vA).subLocal(temp);
				float Cdot2 = wB - wA;
				Cdot.set_Renamed(Cdot1.x, Cdot1.y, Cdot2);
				
				Vec3 impulse = pool.popVec3();
				m_mass.solve33ToOut(Cdot, impulse);
				impulse.negateLocal();
				
				if (m_limitState == LimitState.EQUAL)
				{
					m_impulse.addLocal(impulse);
				}
				else if (m_limitState == LimitState.AT_LOWER)
				{
					float newImpulse = m_impulse.z + impulse.z;
					if (newImpulse < 0.0f)
					{
						//UPGRADE_NOTE: Final was removed from the declaration of 'rhs '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
						Vec2 rhs = pool.popVec2();
						rhs.set_Renamed(m_mass.ez.x, m_mass.ez.y).mulLocal(m_impulse.z).subLocal(Cdot1);
						m_mass.solve22ToOut(rhs, temp);
						impulse.x = temp.x;
						impulse.y = temp.y;
						impulse.z = - m_impulse.z;
						m_impulse.x += temp.x;
						m_impulse.y += temp.y;
						m_impulse.z = 0.0f;
						pool.pushVec2(1);
					}
					else
					{
						m_impulse.addLocal(impulse);
					}
				}
				else if (m_limitState == LimitState.AT_UPPER)
				{
					float newImpulse = m_impulse.z + impulse.z;
					if (newImpulse > 0.0f)
					{
						//UPGRADE_NOTE: Final was removed from the declaration of 'rhs '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
						Vec2 rhs = pool.popVec2();
						rhs.set_Renamed(m_mass.ez.x, m_mass.ez.y).mulLocal(m_impulse.z).subLocal(Cdot1);
						m_mass.solve22ToOut(rhs, temp);
						impulse.x = temp.x;
						impulse.y = temp.y;
						impulse.z = - m_impulse.z;
						m_impulse.x += temp.x;
						m_impulse.y += temp.y;
						m_impulse.z = 0.0f;
						pool.pushVec2(1);
					}
					else
					{
						m_impulse.addLocal(impulse);
					}
				}
				//UPGRADE_NOTE: Final was removed from the declaration of 'P '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
				Vec2 P = pool.popVec2();
				
				P.set_Renamed(impulse.x, impulse.y);
				
				vA.x -= mA * P.x;
				vA.y -= mA * P.y;
				wA -= iA * (Vec2.cross(m_rA, P) + impulse.z);
				
				vB.x += mB * P.x;
				vB.y += mB * P.y;
				wB += iB * (Vec2.cross(m_rB, P) + impulse.z);
				
				pool.pushVec2(2);
				pool.pushVec3(2);
			}
			else
			{
				
				// Solve point-to-point constraint
				Vec2 Cdot = pool.popVec2();
				Vec2 impulse = pool.popVec2();
				
				Vec2.crossToOutUnsafe(wA, m_rA, temp);
				Vec2.crossToOutUnsafe(wB, m_rB, Cdot);
				Cdot.addLocal(vB).subLocal(vA).subLocal(temp);
				m_mass.solve22ToOut(Cdot.negateLocal(), impulse); // just leave negated
				
				m_impulse.x += impulse.x;
				m_impulse.y += impulse.y;
				
				vA.x -= mA * impulse.x;
				vA.y -= mA * impulse.y;
				wB -= iA * Vec2.cross(m_rA, impulse);
				
				vB.x += mB * impulse.x;
				vB.y += mB * impulse.y;
				wB += iB * Vec2.cross(m_rB, impulse);
				
				pool.pushVec2(2);
			}
			
			data.velocities[m_indexA].v.set_Renamed(vA);
			data.velocities[m_indexA].w = wA;
			data.velocities[m_indexB].v.set_Renamed(vB);
			data.velocities[m_indexB].w = wB;
			
			pool.pushVec2(1);
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public override bool solvePositionConstraints(SolverData data)
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'qA '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Rot qA = pool.popRot();
			//UPGRADE_NOTE: Final was removed from the declaration of 'qB '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Rot qB = pool.popRot();
			Vec2 cA = data.positions[m_indexA].c;
			float aA = data.positions[m_indexA].a;
			Vec2 cB = data.positions[m_indexB].c;
			float aB = data.positions[m_indexB].a;
			
			qA.set_Renamed(aA);
			qB.set_Renamed(aB);
			
			float angularError = 0.0f;
			float positionError = 0.0f;
			
			bool fixedRotation = (m_invIA + m_invIB == 0.0f);
			
			// Solve angular limit constraint.
			if (m_enableLimit && m_limitState != LimitState.INACTIVE && fixedRotation == false)
			{
				float angle = aB - aA - m_referenceAngle;
				float limitImpulse = 0.0f;
				
				if (m_limitState == LimitState.EQUAL)
				{
					// Prevent large angular corrections
					float C = MathUtils.clamp(angle - m_lowerAngle, - Settings.maxAngularCorrection, Settings.maxAngularCorrection);
					limitImpulse = (- m_motorMass) * C;
					angularError = MathUtils.abs(C);
				}
				else if (m_limitState == LimitState.AT_LOWER)
				{
					float C = angle - m_lowerAngle;
					angularError = - C;
					
					// Prevent large angular corrections and allow some slop.
					C = MathUtils.clamp(C + Settings.angularSlop, - Settings.maxAngularCorrection, 0.0f);
					limitImpulse = (- m_motorMass) * C;
				}
				else if (m_limitState == LimitState.AT_UPPER)
				{
					float C = angle - m_upperAngle;
					angularError = C;
					
					// Prevent large angular corrections and allow some slop.
					C = MathUtils.clamp(C - Settings.angularSlop, 0.0f, Settings.maxAngularCorrection);
					limitImpulse = (- m_motorMass) * C;
				}
				
				aA -= m_invIA * limitImpulse;
				aB += m_invIB * limitImpulse;
			}
			// Solve point-to-point constraint.
			{
				qA.set_Renamed(aA);
				qB.set_Renamed(aB);
				
				//UPGRADE_NOTE: Final was removed from the declaration of 'rA '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
				Vec2 rA = pool.popVec2();
				//UPGRADE_NOTE: Final was removed from the declaration of 'rB '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
				Vec2 rB = pool.popVec2();
				//UPGRADE_NOTE: Final was removed from the declaration of 'C '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
				Vec2 C = pool.popVec2();
				//UPGRADE_NOTE: Final was removed from the declaration of 'impulse '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
				Vec2 impulse = pool.popVec2();
				
				Rot.mulToOutUnsafe(qA, C.set_Renamed(m_localAnchorA).subLocal(m_localCenterA), rA);
				Rot.mulToOutUnsafe(qB, C.set_Renamed(m_localAnchorB).subLocal(m_localCenterB), rB);
				C.set_Renamed(cB).addLocal(rB).subLocal(cA).subLocal(rA);
				positionError = C.length();
				
				float mA = m_invMassA, mB = m_invMassB;
				float iA = m_invIA, iB = m_invIB;
				
				//UPGRADE_NOTE: Final was removed from the declaration of 'K '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
				Mat22 K = pool.popMat22();
				K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
				K.ex.y = (- iA) * rA.x * rA.y - iB * rB.x * rB.y;
				K.ey.x = K.ex.y;
				K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;
				
				K.solveToOut(C, impulse);
				impulse.negateLocal();
				
				cA.x -= mA * impulse.x;
				cA.y -= mA * impulse.y;
				aA -= iA * Vec2.cross(rA, impulse);
				
				cB.x += mB * impulse.x;
				cB.y += mB * impulse.y;
				aB += iB * Vec2.cross(rB, impulse);
				
				pool.pushVec2(4);
				pool.pushMat22(1);
			}
			data.positions[m_indexA].c.set_Renamed(cA);
			data.positions[m_indexA].a = aA;
			data.positions[m_indexB].c.set_Renamed(cB);
			data.positions[m_indexB].a = aB;
			
			pool.pushRot(2);
			
			return positionError <= Settings.linearSlop && angularError <= Settings.angularSlop;
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public override void  getAnchorA(Vec2 argOut)
		{
			m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public override void  getAnchorB(Vec2 argOut)
		{
			m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public override void  getReactionForce(float inv_dt, Vec2 argOut)
		{
			argOut.set_Renamed(m_impulse.x, m_impulse.y).mulLocal(inv_dt);
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public override float getReactionTorque(float inv_dt)
		{
			return inv_dt * m_impulse.z;
		}
		
		public virtual void  enableMotor(bool flag)
		{
			m_bodyA.Awake = true;
			m_bodyB.Awake = true;
			m_enableMotor = flag;
		}
		
		public virtual float getMotorTorque(float inv_dt)
		{
			return m_motorImpulse * inv_dt;
		}
		
		public virtual void  enableLimit(bool flag)
		{
			if (flag != m_enableLimit)
			{
				m_bodyA.Awake = true;
				m_bodyB.Awake = true;
				m_enableLimit = flag;
				m_impulse.z = 0.0f;
			}
		}
		
		public virtual void  setLimits(float lower, float upper)
		{
			assert(lower <= upper);
			if (lower != m_lowerAngle || upper != m_upperAngle)
			{
				m_bodyA.Awake = true;
				m_bodyB.Awake = true;
				m_impulse.z = 0.0f;
				m_lowerAngle = lower;
				m_upperAngle = upper;
			}
		}
	}
}