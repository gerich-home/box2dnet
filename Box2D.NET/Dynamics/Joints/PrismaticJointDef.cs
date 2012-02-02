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

using Box2D.Common;

namespace Box2D.Dynamics.Joints
{

    /// <summary>
    /// Prismatic joint definition. This requires defining a line of
    /// motion using an axis and an anchor point. The definition uses local
    /// anchor points and a local axis so that the initial configuration
    /// can violate the constraint slightly. The joint translation is zero
    /// when the local anchor points coincide in world space. Using local
    /// anchors and a local axis helps when saving and loading a game.
    /// </summary>
    /// <warning>at least one body should by dynamic with a non-fixed rotation.</warning>
    /// <author>Daniel</author>
    public class PrismaticJointDef : JointDef
    {
        /// <summary>
        /// The local anchor point relative to body1's origin.
        /// </summary>
        public readonly Vec2 localAnchorA;

        /// <summary>
        /// The local anchor point relative to body2's origin.
        /// </summary>
        public readonly Vec2 localAnchorB;

        /// <summary>
        /// The local translation axis in body1.
        /// </summary>
        public readonly Vec2 localAxisA;

        /// <summary>
        /// The constrained angle between the bodies: body2_angle - body1_angle.
        /// </summary>
        public float referenceAngle;

        /// <summary>
        /// Enable/disable the joint limit.
        /// </summary>
        public bool enableLimit;

        /// <summary>
        /// The lower translation limit, usually in meters.
        /// </summary>
        public float lowerTranslation;

        /// <summary>
        /// The upper translation limit, usually in meters.
        /// </summary>
        public float upperTranslation;

        /// <summary>
        /// Enable/disable the joint motor.
        /// </summary>
        public bool enableMotor;

        /// <summary>
        /// The maximum motor torque, usually in N-m.
        /// </summary>
        public float maxMotorForce;

        /// <summary>
        /// The desired motor speed in radians per second.
        /// </summary>
        public float motorSpeed;

        public PrismaticJointDef()
        {
            type = JointType.PRISMATIC;
            localAnchorA = new Vec2();
            localAnchorB = new Vec2();
            localAxisA = new Vec2(1.0f, 0.0f);
            referenceAngle = 0.0f;
            enableLimit = false;
            lowerTranslation = 0.0f;
            upperTranslation = 0.0f;
            enableMotor = false;
            maxMotorForce = 0.0f;
            motorSpeed = 0.0f;
        }

        /// <summary>
        /// Initialize the bodies, anchors, axis, and reference angle using the world anchor and world axis.
        /// </summary>
        public virtual void initialize(Body b1, Body b2, Vec2 anchor, Vec2 axis)
        {
            bodyA = b1;
            bodyB = b2;
            bodyA.GetLocalPointToOut(anchor, localAnchorA);
            bodyB.GetLocalPointToOut(anchor, localAnchorB);
            bodyA.GetLocalVectorToOut(axis, localAxisA);
            referenceAngle = bodyB.Angle - bodyA.Angle;
        }
    }
}