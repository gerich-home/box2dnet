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
using Body = org.jbox2d.dynamics.Body;
namespace org.jbox2d.dynamics.joints
{
	
	/// <summary> Definition for a {@link ConstantVolumeJoint}, which connects a group a bodies together
	/// so they maintain a constant volume within them.
	/// </summary>
	public class ConstantVolumeJointDef:JointDef
	{
		public float frequencyHz;
		public float dampingRatio;
		
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		ArrayList < Body > bodies;
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		ArrayList < DistanceJoint > joints;
		
		//public float relaxationFactor;//1.0 is perfectly stiff (but doesn't work, unstable)
		
		public ConstantVolumeJointDef()
		{
			type = JointType.CONSTANT_VOLUME;
			//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
			bodies = new ArrayList < Body >();
			joints = null;
			//relaxationFactor = 0.9f;
			collideConnected = false;
			frequencyHz = 0.0f;
			dampingRatio = 0.0f;
		}
		
		/// <summary> Adds a body to the group</summary>
		/// <param name="argBody">
		/// </param>
		public virtual void  addBody(Body argBody)
		{
			bodies.add(argBody);
			if (bodies.size() == 1)
			{
				bodyA = argBody;
			}
			if (bodies.size() == 2)
			{
				bodyB = argBody;
			}
		}
		
		/// <summary> Adds a body and the pre-made distance joint.  Should only
		/// be used for deserialization.
		/// </summary>
		public virtual void  addBodyAndJoint(Body argBody, DistanceJoint argJoint)
		{
			addBody(argBody);
			if (joints == null)
			{
				//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
				joints = new ArrayList < DistanceJoint >();
			}
			joints.add(argJoint);
		}
	}
}