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

// Created at 3:38:52 AM Jan 15, 2011

using Box2D.Common;

namespace Box2D.Dynamics.Joints
{

    /// <author>Daniel Murphy</author>
    public class WeldJointDef : JointDef
    {
        /// <summary>
        /// The local anchor point relative to body1's origin.
        /// </summary>
        public readonly Vec2 LocalAnchorA;

        /// <summary>
        /// The local anchor point relative to body2's origin.
        /// </summary>
        public readonly Vec2 LocalAnchorB;

        /// <summary>
        /// The body2 angle minus body1 angle in the reference state (radians).
        /// </summary>
        public float ReferenceAngle;

        /// <summary>
        /// The mass-spring-damper frequency in Hertz. Rotation only.
        /// Disable softness with a value of 0.
        /// </summary>
        public float FrequencyHz;

        /// <summary>
        /// The damping ratio. 0 = no damping, 1 = critical damping.
        /// </summary>
        public float DampingRatio;

        public WeldJointDef()
        {
            Type = JointType.Weld;
            LocalAnchorA = new Vec2();
            LocalAnchorB = new Vec2();
            ReferenceAngle = 0.0f;
        }

        /// <summary>
        /// Initialize the bodies, anchors, and reference angle using a world anchor point.
        /// </summary>
        /// <param name="bA"></param>
        /// <param name="bB"></param>
        /// <param name="anchor"></param>
        public void Initialize(Body bA, Body bB, Vec2 anchor)
        {
            BodyA = bA;
            BodyB = bB;
            BodyA.GetLocalPointToOut(anchor, LocalAnchorA);
            BodyB.GetLocalPointToOut(anchor, LocalAnchorB);
            ReferenceAngle = BodyB.Angle - BodyA.Angle;
        }
    }
}