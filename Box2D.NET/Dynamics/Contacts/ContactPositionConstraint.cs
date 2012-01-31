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
        internal Vec2[] localPoints = new Vec2[Settings.maxManifoldPoints];
        internal readonly Vec2 localNormal = new Vec2();
        internal readonly Vec2 localPoint = new Vec2();
        internal int indexA;
        internal int indexB;
        internal float invMassA, invMassB;
        internal readonly Vec2 localCenterA = new Vec2();
        internal readonly Vec2 localCenterB = new Vec2();
        internal float invIA, invIB;
        internal Manifold.ManifoldType type;
        internal float radiusA, radiusB;
        internal int pointCount;

        public ContactPositionConstraint()
        {
            for (int i = 0; i < localPoints.Length; i++)
            {
                localPoints[i] = new Vec2();
            }
        }
    }
}