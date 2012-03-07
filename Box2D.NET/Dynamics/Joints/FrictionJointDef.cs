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

// Created at 7:23:39 AM Jan 20, 2011

using Box2D.Common;

namespace Box2D.Dynamics.Joints
{

    /// <summary>
    /// Friction joint definition.
    /// </summary>
    /// <author>Daniel Murphy</author>
    public class FrictionJointDef : JointDef
    {
        /// <summary>
        /// The local anchor point relative to bodyA's origin.
        /// </summary>
        public readonly Vec2 localAnchorA;

        /// <summary>
        /// The local anchor point relative to bodyB's origin.
        /// </summary>
        public readonly Vec2 localAnchorB;

        /// <summary>
        /// The maximum friction force in N.
        /// </summary>
        public float maxForce;

        /// <summary>
        /// The maximum friction torque in N-m.
        /// </summary>
        public float maxTorque;

        public FrictionJointDef()
        {
            type = JointType.Friction;
            localAnchorA = new Vec2();
            localAnchorB = new Vec2();
            maxForce = 0f;
            maxTorque = 0f;
        }

        /// <summary>
        /// Initialize the bodies, anchors, axis, and reference angle using the world anchor and world axis.
        /// </summary>
        public void initialize(Body bA, Body bB, Vec2 anchor)
        {
            bodyA = bA;
            bodyB = bB;
            bA.GetLocalPointToOut(anchor, localAnchorA);
            bB.GetLocalPointToOut(anchor, localAnchorB);
        }
    }
}