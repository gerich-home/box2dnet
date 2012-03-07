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

// Created at 7:27:31 AM Jan 21, 2011

using Box2D.Common;

namespace Box2D.Dynamics.Joints
{
    /// <author>Daniel Murphy</author>
    public class LineJointDef : JointDef
    {
        /// <summary>
        /// The local anchor point relative to body1's origin.
        /// </summary>
        public readonly Vec2 LocalAnchorA = new Vec2();

        /// <summary>
        /// The local anchor point relative to body2's origin.
        /// </summary>
        public readonly Vec2 LocalAnchorB = new Vec2();

        /// <summary>
        /// The local translation axis in body1.
        /// </summary>
        public readonly Vec2 LocalAxisA = new Vec2();

        /// <summary>
        /// Enable/disable the joint limit.
        /// </summary>
        public bool EnableLimit;

        /// <summary>
        /// The lower translation limit, usually in meters.
        /// </summary>
        public float LowerTranslation;

        /// <summary>
        /// The upper translation limit, usually in meters.
        /// </summary>
        public float UpperTranslation;

        /// <summary>
        /// Enable/disable the joint motor.
        /// </summary>
        public bool EnableMotor;

        /// <summary>
        /// The maximum motor torque, usually in N-m.
        /// </summary>
        public float MaxMotorForce;

        /// <summary>
        /// The desired motor speed in radians per second.
        /// </summary>
        public float MotorSpeed;

        public LineJointDef()
        {
            Type = JointType.Wheel;
            LocalAxisA.Set(1, 0);
            EnableLimit = false;
            LowerTranslation = 0;
            UpperTranslation = 0;
            EnableMotor = false;
            MaxMotorForce = 0f;
            MotorSpeed = 0f;
        }

        public void Initialize(Body b1, Body b2, Vec2 anchor, Vec2 axis)
        {
            BodyA = b1;
            BodyB = b2;
            b1.GetLocalPointToOut(anchor, LocalAnchorA);
            b2.GetLocalPointToOut(anchor, LocalAnchorB);
            BodyA.GetLocalVectorToOut(axis, LocalAxisA);
        }
    }
}