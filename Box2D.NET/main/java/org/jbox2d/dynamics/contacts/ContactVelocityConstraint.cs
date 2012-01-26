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
using Settings = org.jbox2d.common.Settings;
using Vec2 = org.jbox2d.common.Vec2;
namespace org.jbox2d.dynamics.contacts
{
	
	public class ContactVelocityConstraint
	{
		private void  InitBlock()
		{
			points = new VelocityConstraintPoint[Settings.maxManifoldPoints];
		}
		//UPGRADE_NOTE: The initialization of  'points' was moved to method 'InitBlock'. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1005'"
		public VelocityConstraintPoint[] points;
		//UPGRADE_NOTE: Final was removed from the declaration of 'normal '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Vec2 normal = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'normalMass '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Mat22 normalMass = new Mat22();
		//UPGRADE_NOTE: Final was removed from the declaration of 'K '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Mat22 K = new Mat22();
		public int indexA;
		public int indexB;
		public float invMassA, invMassB;
		public float invIA, invIB;
		public float friction;
		public float restitution;
		public float tangentSpeed;
		public int pointCount;
		public int contactIndex;
		
		public ContactVelocityConstraint()
		{
			InitBlock();
			for (int i = 0; i < points.Length; i++)
			{
				points[i] = new VelocityConstraintPoint();
			}
		}
		
		public class VelocityConstraintPoint
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'rA '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			public Vec2 rA = new Vec2();
			//UPGRADE_NOTE: Final was removed from the declaration of 'rB '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			public Vec2 rB = new Vec2();
			public float normalImpulse;
			public float tangentImpulse;
			public float normalMass;
			public float tangentMass;
			public float velocityBias;
		}
	}
}