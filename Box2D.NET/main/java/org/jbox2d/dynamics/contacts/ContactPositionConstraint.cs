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
//UPGRADE_TODO: The type 'org.jbox2d.collision.Manifold.ManifoldType' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
using ManifoldType = org.jbox2d.collision.Manifold.ManifoldType;
using Settings = org.jbox2d.common.Settings;
using Vec2 = org.jbox2d.common.Vec2;
namespace org.jbox2d.dynamics.contacts
{
	
	public class ContactPositionConstraint
	{
		private void  InitBlock()
		{
			localPoints = new Vec2[Settings.maxManifoldPoints];
		}
		//UPGRADE_NOTE: The initialization of  'localPoints' was moved to method 'InitBlock'. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1005'"
		internal Vec2[] localPoints;
		//UPGRADE_NOTE: Final was removed from the declaration of 'localNormal '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		internal Vec2 localNormal = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'localPoint '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		internal Vec2 localPoint = new Vec2();
		internal int indexA;
		internal int indexB;
		internal float invMassA, invMassB;
		//UPGRADE_NOTE: Final was removed from the declaration of 'localCenterA '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		internal Vec2 localCenterA = new Vec2();
		//UPGRADE_NOTE: Final was removed from the declaration of 'localCenterB '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		internal Vec2 localCenterB = new Vec2();
		internal float invIA, invIB;
		internal ManifoldType type;
		internal float radiusA, radiusB;
		internal int pointCount;
		
		public ContactPositionConstraint()
		{
			InitBlock();
			for (int i = 0; i < localPoints.Length; i++)
			{
				localPoints[i] = new Vec2();
			}
		}
	}
}