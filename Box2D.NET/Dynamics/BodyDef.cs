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

namespace Box2D.Dynamics
{

    // updated to rev 100
    /// <summary>
    /// A body definition holds all the data needed to construct a rigid body.
    /// You can safely re-use body definitions. Shapes are added to a body
    /// after construction.
    /// </summary>
    /// <author>daniel</author>
    public class BodyDef
    {
        /// <summary>
        /// The body type: static, kinematic, or dynamic.
        /// Note: if a dynamic body would have zero mass, the mass is set to one.
        /// </summary>
        public BodyType Type;

        /// <summary>
        /// Use this to store application specific body data.
        /// </summary>
        public object UserData;

        /// <summary>
        /// The world position of the body. Avoid creating bodies at the origin
        /// since this can lead to many overlapping shapes.
        /// </summary>
        public Vec2 Position;

        /// <summary>
        /// The world angle of the body in radians.
        /// </summary>
        public float Angle;

        /// <summary>
        /// The linear velocity of the body in world co-ordinates.
        /// </summary>
        public Vec2 LinearVelocity;

        /// <summary>
        /// The angular velocity of the body.
        /// </summary>
        public float AngularVelocity;

        /// <summary>
        /// Linear damping is use to reduce the linear velocity. The damping parameter
        /// can be larger than 1.0f but the damping effect becomes sensitive to the
        /// time step when the damping parameter is large.
        /// </summary>
        public float LinearDamping;

        /// <summary>
        /// Angular damping is use to reduce the angular velocity. The damping parameter
        /// can be larger than 1.0f but the damping effect becomes sensitive to the
        /// time step when the damping parameter is large.
        /// </summary>
        public float AngularDamping;

        /// <summary>
        /// Set this flag to false if this body should never fall asleep. Note that
        /// this increases CPU usage.
        /// </summary>
        public bool AllowSleep;

        /// <summary>
        /// Is this body initially sleeping?
        /// </summary>
        public bool Awake;

        /// <summary>
        /// Should this body be prevented from rotating? Useful for characters.
        /// </summary>
        public bool FixedRotation;

        /// <summary>
        /// Is this a fast moving body that should be prevented from tunneling through
        /// other moving bodies? Note that all bodies are prevented from tunneling through
        /// kinematic and static bodies. This setting is only considered on dynamic bodies.
        /// </summary>
        /// <warning>You should use this flag sparingly since it increases processing time.</warning>
        public bool Bullet;

        /// <summary>
        /// Does this body start out active?
        /// </summary>
        public bool Active;

        /// <summary>
        /// Experimental: scales the inertia tensor.
        /// </summary>
        public float GravityScale;

        public BodyDef()
        {
            UserData = null;
            Position = new Vec2();
            Angle = 0f;
            LinearVelocity = new Vec2();
            AngularVelocity = 0f;
            LinearDamping = 0f;
            AngularDamping = 0f;
            AllowSleep = true;
            Awake = true;
            FixedRotation = false;
            Bullet = false;
            Type = BodyType.Static;
            Active = true;
            GravityScale = 1.0f;
        }
    }
}