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

using System.Collections.Generic;

namespace Box2D.Dynamics.Joints
{
    /// <summary>
    /// Definition for a {@link ConstantVolumeJoint}, which connects a group a bodies together
    /// so they maintain a constant volume within them.
    /// </summary>
    public class ConstantVolumeJointDef : JointDef
    {
        public float FrequencyHz;
        public float DampingRatio;

        internal List<Body> Bodies;
        internal List<DistanceJoint> Joints;

        //public float relaxationFactor;//1.0 is perfectly stiff (but doesn't work, unstable)

        public ConstantVolumeJointDef()
        {
            Type = JointType.ConstantVolume;
            Bodies = new List<Body>();
            Joints = null;
            //relaxationFactor = 0.9f;
            CollideConnected = false;
            FrequencyHz = 0.0f;
            DampingRatio = 0.0f;
        }

        /// <summary>
        /// Adds a body to the group
        /// </summary>
        /// <param name="argBody"></param>
        public void AddBody(Body argBody)
        {
            Bodies.Add(argBody);
            if (Bodies.Count == 1)
            {
                BodyA = argBody;
            }
            if (Bodies.Count == 2)
            {
                BodyB = argBody;
            }
        }

        /// <summary>
        /// Adds a body and the pre-made distance joint.
        /// Should only be used for deserialization.
        /// </summary>
        public void AddBodyAndJoint(Body argBody, DistanceJoint argJoint)
        {
            AddBody(argBody);
            if (Joints == null)
            {
                Joints = new List<DistanceJoint>();
            }
            Joints.Add(argJoint);
        }
    }
}