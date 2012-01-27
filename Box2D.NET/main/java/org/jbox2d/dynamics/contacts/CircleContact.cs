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
using Manifold = org.jbox2d.collision.Manifold;
using CircleShape = org.jbox2d.collision.shapes.CircleShape;
using ShapeType = org.jbox2d.collision.shapes.ShapeType;
using Transform = org.jbox2d.common.Transform;
using Fixture = org.jbox2d.dynamics.Fixture;
using IWorldPool = org.jbox2d.pooling.IWorldPool;
using System.Diagnostics;

namespace org.jbox2d.dynamics.contacts
{

    public class CircleContact : Contact
    {
        public CircleContact(IWorldPool argPool)
            : base(argPool)
        {
        }

        public virtual void init(Fixture fixtureA, Fixture fixtureB)
        {
            base.init(fixtureA, 0, fixtureB, 0);
            Debug.Assert(m_fixtureA.Type == ShapeType.CIRCLE);
            Debug.Assert(m_fixtureB.Type == ShapeType.CIRCLE);
        }

        public override void evaluate(Manifold manifold, Transform xfA, Transform xfB)
        {
            pool.getCollision().collideCircles(manifold, (CircleShape)m_fixtureA.Shape, xfA, (CircleShape)m_fixtureB.Shape, xfB);
        }
    }
}