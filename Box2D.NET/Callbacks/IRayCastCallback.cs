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

// Created at 4:33:10 AM Jul 15, 2010

using Box2D.Common;
using Box2D.Dynamics;

namespace Box2D.Callbacks
{
    // updated to rev 100;
    /// <summary>
    /// Callback class for ray casts.
    /// See World.rayCast
    /// </summary>
    /// <author>Daniel Murphy</author>
    public interface IRayCastCallback
    {
        /// <summary>
        /// Called for each fixture found in the query. You control how the ray cast
        /// proceeds by returning a float:
        /// return -1: ignore this fixture and continue
        /// return 0: terminate the ray cast
        /// return fraction: clip the ray to this point
        /// return 1: don't clip the ray and continue
        /// </summary>
        /// <param name="fixture">the fixture hit by the ray</param>
        /// <param name="point">the point of initial intersection</param>
        /// <param name="normal">the normal vector at the point of intersection</param>
        /// <returns>-1 to filter, 0 to terminate, fraction to clip the ray for closest hit, 1 to continue</returns>
        /// <param name="fraction"></param>
        float ReportFixture(Fixture fixture, Vec2 point, Vec2 normal, float fraction);
    }
}