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

using Box2D.Collision;
using Box2D.Common;

namespace Box2D.Dynamics.Contacts
{
    public class ContactPositionConstraint
    {
        internal Vec2[] LocalPoints = new Vec2[Settings.MAX_MANIFOLD_POINTS];
        internal readonly Vec2 LocalNormal = new Vec2();
        internal readonly Vec2 LocalPoint = new Vec2();
        internal int IndexA;
        internal int IndexB;
        internal float InvMassA;
        internal float InvMassB;
        internal readonly Vec2 LocalCenterA = new Vec2();
        internal readonly Vec2 LocalCenterB = new Vec2();
        internal float InvIA;
        internal float InvIB;
        internal Manifold.ManifoldType Type;
        internal float RadiusA;
        internal float RadiusB;
        internal int PointCount;

        public ContactPositionConstraint()
        {
            for (int i = 0; i < LocalPoints.Length; i++)
            {
                LocalPoints[i] = new Vec2();
            }
        }
    }
}