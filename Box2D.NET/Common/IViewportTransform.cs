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

// 1:13:11 AM, Jul 17, 2009

namespace Box2D.Common
{

    // updated to rev 100
    /// <summary>
    /// This is the viewport transform used from drawing.
    /// Use yFlip if you are drawing from the top-left corner.
    /// </summary>
    /// <author>daniel</author>
    public interface IViewportTransform
    {
        /// <summary>
        /// Determine whether the transform flips the y axis
        /// </summary>
        bool YFlip
        {
            get;
            set;
        }

        /// <summary>
        /// This is the half-width and half-height.
        /// This should be the actual half-width and 
        /// half-height, not anything transformed or scaled.
        /// Not a copy.
        /// </summary>
        /// <returns></returns>
        Vec2 getExtents();

        /// <summary>
        /// This sets the half-width and half-height.
        /// This should be the actual half-width and 
        /// half-height, not anything transformed or scaled.
        /// </summary>
        /// <param name="argExtents"></param>
        void setExtents(Vec2 argExtents);

        /// <summary>
        /// This sets the half-width and half-height of the
        /// viewport. This should be the actual half-width and 
        /// half-height, not anything transformed or scaled.
        /// </summary>
        /// <param name="argHalfWidth"></param>
        /// <param name="argHalfHeight"></param>
        void setExtents(float argHalfWidth, float argHalfHeight);

        /// <summary>
        /// Center of the viewport. Not a copy.</summary>
        /// <returns></returns>
        Vec2 getCenter();

        /// <summary>
        /// Sets the center of the viewport.
        /// </summary>
        /// <param name="argPos"></param>
        void setCenter(Vec2 argPos);

        /// <summary>
        /// sets the center of the viewport.
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        void setCenter(float x, float y);

        /// <summary>
        /// Sets the transform's center to the given x and y coordinates,
        /// and using the given scale.
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="scale"></param>
        void setCamera(float x, float y, float scale);

        /// <summary>
        /// Transforms the given directional vector by the
        /// viewport transform (not positional)
        /// </summary>
        /// <param name="argVec"></param>
        /// <param name="argOut"></param>
        void getWorldVectorToScreen(Vec2 argWorld, Vec2 argScreen);


        /// <summary>
        /// Transforms the given directional screen vector back to
        /// the world direction.
        /// </summary>
        /// <param name="argVec"></param>
        /// <param name="argOut"></param>
        void getScreenVectorToWorld(Vec2 argScreen, Vec2 argWorld);


        /// <summary>
        /// Takes the world coordinate (argWorld) puts the corresponding
        /// screen coordinate in argScreen.  It should be safe to give the
        /// same object as both parameters.
        /// </summary>
        /// <param name="argWorld"></param>
        /// <param name="argScreen"></param>
        void getWorldToScreen(Vec2 argWorld, Vec2 argScreen);


        /// <summary>
        /// Takes the screen coordinates (argScreen) and puts the
        /// corresponding world coordinates in argWorld. It should be safe
        /// to give the same object as both parameters.
        /// </summary>
        /// <param name="argScreen"></param>
        /// <param name="argWorld"></param>
        void getScreenToWorld(Vec2 argScreen, Vec2 argWorld);
    }
}