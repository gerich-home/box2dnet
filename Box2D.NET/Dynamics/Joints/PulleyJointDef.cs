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

// Created at 12:11:41 PM Jan 23, 2011

using System.Diagnostics;
using Box2D.Common;

namespace Box2D.Dynamics.Joints
{

    /// <summary>
    /// Pulley joint definition. This requires two ground anchors, two dynamic body anchor points, and a pulley ratio.
    /// </summary>
    /// <author>Daniel Murphy</author>
    public class PulleyJointDef : JointDef
    {
        /// <summary>
        /// The first ground anchor in world coordinates. This point never moves.
        /// </summary>
        public Vec2 GroundAnchorA;

        /// <summary>
        /// The second ground anchor in world coordinates. This point never moves.
        /// </summary>
        public Vec2 GroundAnchorB;

        /// <summary>
        /// The local anchor point relative to bodyA's origin.
        /// </summary>
        public Vec2 LocalAnchorA;

        /// <summary>
        /// The local anchor point relative to bodyB's origin.
        /// </summary>
        public Vec2 LocalAnchorB;

        /// <summary>
        /// The a reference length for the segment attached to bodyA.
        /// </summary>
        public float LengthA;

        /// <summary>
        /// The a reference length for the segment attached to bodyB.
        /// </summary>
        public float LengthB;

        /// <summary>
        /// The pulley ratio, used to simulate a block-and-tackle.
        /// </summary>
        public float Ratio;

        public PulleyJointDef()
        {
            Type = JointType.Pulley;
            GroundAnchorA = new Vec2(-1.0f, 1.0f);
            GroundAnchorB = new Vec2(1.0f, 1.0f);
            LocalAnchorA = new Vec2(-1.0f, 0.0f);
            LocalAnchorB = new Vec2(1.0f, 0.0f);
            LengthA = 0.0f;
            LengthB = 0.0f;
            Ratio = 1.0f;
            CollideConnected = true;
        }

        /// <summary>
        /// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
        /// </summary>
        public void Initialize(Body b1, Body b2, Vec2 ga1, Vec2 ga2, Vec2 anchor1, Vec2 anchor2, float r)
        {
            BodyA = b1;
            BodyB = b2;
            GroundAnchorA = ga1;
            GroundAnchorB = ga2;
            LocalAnchorA = BodyA.GetLocalPoint(anchor1);
            LocalAnchorB = BodyB.GetLocalPoint(anchor2);
            Vec2 d1 = anchor1.Sub(ga1);
            LengthA = d1.Length();
            Vec2 d2 = anchor2.Sub(ga2);
            LengthB = d2.Length();
            Ratio = r;
            Debug.Assert(Ratio > Settings.EPSILON);
        }
    }
}