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

using System.Diagnostics;
using Box2D.Collision;
using Box2D.Collision.Shapes;
using Box2D.Common;
using Box2D.Pooling;

namespace Box2D.Dynamics.Contacts
{

    public class ChainAndPolygonContact : Contact
    {
        public ChainAndPolygonContact(IWorldPool argPool) :
            base(argPool)
        {
        }

        public override void init(Fixture fA, int indexA, Fixture fB, int indexB)
        {
            base.init(fA, indexA, fB, indexB);
            Debug.Assert(m_fixtureA.Type == ShapeType.Chain);
            Debug.Assert(m_fixtureB.Type == ShapeType.Polygon);
        }

        private readonly EdgeShape edge = new EdgeShape();

        public override void evaluate(Manifold manifold, Transform xfA, Transform xfB)
        {
            ChainShape chain = (ChainShape)m_fixtureA.Shape;
            chain.getChildEdge(edge, m_indexA);
            pool.GetCollision().collideEdgeAndPolygon(manifold, edge, xfA, (PolygonShape)m_fixtureB.Shape, xfB);
        }
    }
}