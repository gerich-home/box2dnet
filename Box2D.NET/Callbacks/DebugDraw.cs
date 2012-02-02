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

// Created at 4:35:29 AM Jul 15, 2010

using System;
using Box2D.Common;

namespace Box2D.Callbacks
{

    // updated to rev 100
    /// <summary>
    /// Implement this abstract class to allow JBox2d to 
    /// automatically draw your physics for debugging purposes.
    /// Not intended to replace your own custom rendering
    /// routines!
    /// </summary>
    /// <author>Daniel Murphy</author>
    public abstract class DebugDraw
    {
        [Flags]
        public enum DrawFlags
        {
            /// <summary>
            /// No draw flags are set
            /// </summary>
            None = 0x0000,

            /// <summary>
            /// Draw shapes
            /// </summary>
            Shape = 0x0001,

            /// <summary>
            /// Draw joint connections
            /// </summary>
            Joint = 0x0002,

            /// <summary>
            /// Draw core (TOI) shapes
            /// </summary>
            AABB = 0x0004,

            /// <summary>
            /// Draw axis aligned bounding boxes
            /// </summary>
            Pair = 0x0008,

            /// <summary>
            /// draw center of mass frame
            /// </summary>
            CenterOfMass = 0x0010,

            DynamicTree = 0x0020,
        }

        public virtual DrawFlags Flags { get; set; }

        public virtual IViewportTransform ViewportTranform { get; private set; }


        protected DebugDraw(IViewportTransform viewport)
        {
            Flags = DrawFlags.None;
            ViewportTranform = viewport;
        }

        public virtual void AppendFlags(DrawFlags flags)
        {
            Flags |= flags;
        }

        public virtual void ClearFlags(DrawFlags flags)
        {
            Flags &= ~flags;
        }

        /// <summary>
        /// Draw a closed polygon provided in CCW order.  This implementation
        /// uses {@link #drawSegment(Vec2, Vec2, Color3f)} to draw each side of the
        /// polygon.
        /// </summary>
        /// <param name="vertices"></param>
        /// <param name="vertexCount"></param>
        /// <param name="color"></param>
        public virtual void DrawPolygon(Vec2[] vertices, int vertexCount, Color3f color)
        {
            if (vertexCount == 1)
            {
                DrawSegment(vertices[0], vertices[0], color);
                return;
            }

            for (int i = 0; i < vertexCount - 1; i += 1)
            {
                DrawSegment(vertices[i], vertices[i + 1], color);
            }

            if (vertexCount > 2)
            {
                DrawSegment(vertices[vertexCount - 1], vertices[0], color);
            }
        }

        public abstract void DrawPoint(Vec2 argPoint, float argRadiusOnScreen, Color3f argColor);

        /// <summary>
        /// Draw a solid closed polygon provided in CCW order.
        /// </summary>
        /// <param name="vertices"></param>
        /// <param name="vertexCount"></param>
        /// <param name="color"></param>
        public abstract void DrawSolidPolygon(Vec2[] vertices, int vertexCount, Color3f color);

        /// <summary>
        /// Draw a circle.
        /// </summary>
        /// <param name="center"></param>
        /// <param name="radius"></param>
        /// <param name="color"></param>
        public abstract void DrawCircle(Vec2 center, float radius, Color3f color);

        /// <summary>
        /// Draw a solid circle.
        /// </summary>
        /// <param name="center"></param>
        /// <param name="radius"></param>
        /// <param name="axis"></param>
        /// <param name="color"></param>
        public abstract void DrawSolidCircle(Vec2 center, float radius, Vec2 axis, Color3f color);

        /// <summary>
        /// Draw a line segment.
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <param name="color"></param>
        public abstract void DrawSegment(Vec2 p1, Vec2 p2, Color3f color);

        /// <summary>
        /// Draw a transform.  Choose your own length scale
        /// </summary>
        /// <param name="xf"></param>
        public abstract void DrawTransform(Transform xf);

        /// <summary>
        /// Draw a string.
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="s"></param>
        /// <param name="color"></param>
        public abstract void DrawString(float x, float y, String s, Color3f color);

        public void DrawString(Vec2 pos, String s, Color3f color)
        {
            DrawString(pos.X, pos.Y, s, color);
        }

        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="scale"></param>
        /// <seealso cref="IViewportTransform.setCamera(float, float, float)"></seealso>
        public virtual void SetCamera(float x, float y, float scale)
        {
            ViewportTranform.setCamera(x, y, scale);
        }


        /// <param name="argScreen"></param>
        /// <param name="argWorld"></param>
        /// <seealso cref="IViewportTransform.getScreenToWorld(Vec2, Vec2)"></seealso>
        public virtual void GetScreenToWorldToOut(Vec2 argScreen, Vec2 argWorld)
        {
            ViewportTranform.getScreenToWorld(argScreen, argWorld);
        }

        /// <param name="argWorld"></param>
        /// <param name="argScreen"></param>
        /// <seealso cref="IViewportTransform.getWorldToScreen(Vec2, Vec2)"></seealso>
        public virtual void GetWorldToScreenToOut(Vec2 argWorld, Vec2 argScreen)
        {
            ViewportTranform.getWorldToScreen(argWorld, argScreen);
        }

        /// <summary>
        /// Takes the world coordinates and puts the corresponding screen
        /// coordinates in argScreen.
        /// </summary>
        /// <param name="worldX"></param>
        /// <param name="worldY"></param>
        /// <param name="argScreen"></param>
        public virtual void GetWorldToScreenToOut(float worldX, float worldY, Vec2 argScreen)
        {
            argScreen.Set(worldX, worldY);
            ViewportTranform.getWorldToScreen(argScreen, argScreen);
        }

        /// <summary>
        /// takes the world coordinate (argWorld) and returns
        /// the screen coordinates.
        /// </summary>
        /// <param name="argWorld"></param>
        public virtual Vec2 GetWorldToScreen(Vec2 argWorld)
        {
            var screen = new Vec2();
            ViewportTranform.getWorldToScreen(argWorld, screen);
            return screen;
        }

        /// <summary>
        /// Takes the world coordinates and returns the screen
        /// coordinates.
        /// </summary>
        /// <param name="worldX"></param>
        /// <param name="worldY"></param>
        public virtual Vec2 GetWorldToScreen(float worldX, float worldY)
        {
            var argScreen = new Vec2(worldX, worldY);
            ViewportTranform.getWorldToScreen(argScreen, argScreen);
            return argScreen;
        }

        /// <summary>
        /// takes the screen coordinates and puts the corresponding 
        /// world coordinates in argWorld.
        /// </summary>
        /// <param name="screenX"></param>
        /// <param name="screenY"></param>
        /// <param name="argWorld"></param>
        public virtual void GetScreenToWorldToOut(float screenX, float screenY, Vec2 argWorld)
        {
            argWorld.Set(screenX, screenY);
            ViewportTranform.getScreenToWorld(argWorld, argWorld);
        }

        /// <summary>
        /// takes the screen coordinates (argScreen) and returns
        /// the world coordinates
        /// </summary>
        /// <param name="argScreen"></param>
        public virtual Vec2 GetScreenToWorld(Vec2 argScreen)
        {
            var world = new Vec2();
            ViewportTranform.getScreenToWorld(argScreen, world);
            return world;
        }

        /// <summary>
        /// takes the screen coordinates and returns the
        /// world coordinates.
        /// </summary>
        /// <param name="screenX"></param>
        /// <param name="screenY"></param>
        public virtual Vec2 GetScreenToWorld(float screenX, float screenY)
        {
            var screen = new Vec2(screenX, screenY);
            ViewportTranform.getScreenToWorld(screen, screen);
            return screen;
        }
    }
}