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

// Created at 4:23:30 AM Jul 15, 2010

using Box2D.Dynamics;
using Box2D.Dynamics.Joints;

namespace Box2D.Callbacks
{

    // updated to rev 100
    /// <summary>
    /// Joints and fixtures are destroyed when their associated
    /// body is destroyed. Implement this listener so that you
    /// may nullify references to these joints and shapes.
    /// </summary>
    /// <author>Daniel Murphy</author>
    public interface DestructionListener
    {

        /// <summary>
        /// Called when any joint is about to be destroyed due
        /// to the destruction of one of its attached bodies.
        /// </summary>
        /// <param name="joint"></param>
        void sayGoodbye(Joint joint);

        /// <summary>
        /// Called when any fixture is about to be destroyed due
        /// to the destruction of its parent body.
        /// </summary>
        /// <param name="fixture"></param>
        void sayGoodbye(Fixture fixture);
    }
}