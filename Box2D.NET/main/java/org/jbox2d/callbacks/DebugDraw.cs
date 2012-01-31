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
using org.jbox2d.common;

namespace org.jbox2d.callbacks
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
        virtual public int Flags
        {
            get
            {
                return m_drawFlags;
            }
            set
            {
                m_drawFlags = value;
            }
        }

        virtual public IViewportTransform ViewportTranform
        {
            get
            {
                return viewportTransform;
            }
        }

        public const int e_shapeBit = 0x0001; ///< draw shapes
        public const int e_jointBit = 0x0002; ///< draw joint connections
        public const int e_aabbBit = 0x0004; ///< draw core (TOI) shapes
        public const int e_pairBit = 0x0008; ///< draw axis aligned bounding boxes
        public const int e_centerOfMassBit = 0x0010; ///< draw center of mass frame
        public const int e_dynamicTreeBit = 0x0020; ///< draw dynamic tree.

        protected internal int m_drawFlags;
        protected internal readonly IViewportTransform viewportTransform;

        public DebugDraw(IViewportTransform viewport)
        {
            m_drawFlags = 0;
            viewportTransform = viewport;
        }

        public virtual void appendFlags(int flags)
        {
            m_drawFlags |= flags;
        }

        public virtual void clearFlags(int flags)
        {
            m_drawFlags &= ~flags;
        }

        /// <summary>
        /// Draw a closed polygon provided in CCW order.  This implementation
        /// uses {@link #drawSegment(Vec2, Vec2, Color3f)} to draw each side of the
        /// polygon.
        /// </summary>
        /// <param name="vertices"></param>
        /// <param name="vertexCount"></param>
        /// <param name="color"></param>
        public virtual void drawPolygon(Vec2[] vertices, int vertexCount, Color3f color)
        {
            if (vertexCount == 1)
            {
                drawSegment(vertices[0], vertices[0], color);
                return;
            }

            for (int i = 0; i < vertexCount - 1; i += 1)
            {
                drawSegment(vertices[i], vertices[i + 1], color);
            }

            if (vertexCount > 2)
            {
                drawSegment(vertices[vertexCount - 1], vertices[0], color);
            }
        }

        public abstract void drawPoint(Vec2 argPoint, float argRadiusOnScreen, Color3f argColor);

        /// <summary>
        /// Draw a solid closed polygon provided in CCW order.
        /// </summary>
        /// <param name="vertices"></param>
        /// <param name="vertexCount"></param>
        /// <param name="color"></param>
        public abstract void drawSolidPolygon(Vec2[] vertices, int vertexCount, Color3f color);

        /// <summary>
        /// Draw a circle.
        /// </summary>
        /// <param name="center"></param>
        /// <param name="radius"></param>
        /// <param name="color"></param>
        public abstract void drawCircle(Vec2 center, float radius, Color3f color);

        /// <summary>
        /// Draw a solid circle.
        /// </summary>
        /// <param name="center"></param>
        /// <param name="radius"></param>
        /// <param name="axis"></param>
        /// <param name="color"></param>
        public abstract void drawSolidCircle(Vec2 center, float radius, Vec2 axis, Color3f color);

        /// <summary>
        /// Draw a line segment.
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <param name="color"></param>
        public abstract void drawSegment(Vec2 p1, Vec2 p2, Color3f color);

        /// <summary>
        /// Draw a transform.  Choose your own length scale
        /// </summary>
        /// <param name="xf"></param>
        public abstract void drawTransform(Transform xf);

        /// <summary>
        /// Draw a string.
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="s"></param>
        /// <param name="color"></param>
        public abstract void drawString(float x, float y, String s, Color3f color);

        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="scale"></param>
        /// <seealso cref="IViewportTransform.setCamera(float, float, float)"></seealso>
        public virtual void setCamera(float x, float y, float scale)
        {
            viewportTransform.setCamera(x, y, scale);
        }


        /// <param name="argScreen"></param>
        /// <param name="argWorld"></param>
        /// <seealso cref="org.jbox2d.common.IViewportTransform.getScreenToWorld(org.jbox2d.common.Vec2, org.jbox2d.common.Vec2)"></seealso>
        public virtual void getScreenToWorldToOut(Vec2 argScreen, Vec2 argWorld)
        {
            viewportTransform.getScreenToWorld(argScreen, argWorld);
        }

        /// <param name="argWorld"></param>
        /// <param name="argScreen"></param>
        /// <seealso cref="org.jbox2d.common.IViewportTransform.getWorldToScreen(org.jbox2d.common.Vec2, org.jbox2d.common.Vec2)"></seealso>
        public virtual void getWorldToScreenToOut(Vec2 argWorld, Vec2 argScreen)
        {
            viewportTransform.getWorldToScreen(argWorld, argScreen);
        }

        /// <summary>
        /// Takes the world coordinates and puts the corresponding screen
        /// coordinates in argScreen.
        /// </summary>
        /// <param name="worldX"></param>
        /// <param name="worldY"></param>
        /// <param name="argScreen"></param>
        public virtual void getWorldToScreenToOut(float worldX, float worldY, Vec2 argScreen)
        {
            argScreen.set_Renamed(worldX, worldY);
            viewportTransform.getWorldToScreen(argScreen, argScreen);
        }

        /// <summary>
        /// takes the world coordinate (argWorld) and returns
        /// the screen coordinates.
        /// </summary>
        /// <param name="argWorld"></param>
        public virtual Vec2 getWorldToScreen(Vec2 argWorld)
        {
            Vec2 screen = new Vec2();
            viewportTransform.getWorldToScreen(argWorld, screen);
            return screen;
        }

        /// <summary>
        /// Takes the world coordinates and returns the screen
        /// coordinates.
        /// </summary>
        /// <param name="worldX"></param>
        /// <param name="worldY"></param>
        public virtual Vec2 getWorldToScreen(float worldX, float worldY)
        {
            Vec2 argScreen = new Vec2(worldX, worldY);
            viewportTransform.getWorldToScreen(argScreen, argScreen);
            return argScreen;
        }

        /// <summary>
        /// takes the screen coordinates and puts the corresponding 
        /// world coordinates in argWorld.
        /// </summary>
        /// <param name="screenX"></param>
        /// <param name="screenY"></param>
        /// <param name="argWorld"></param>
        public virtual void getScreenToWorldToOut(float screenX, float screenY, Vec2 argWorld)
        {
            argWorld.set_Renamed(screenX, screenY);
            viewportTransform.getScreenToWorld(argWorld, argWorld);
        }

        /// <summary>
        /// takes the screen coordinates (argScreen) and returns
        /// the world coordinates
        /// </summary>
        /// <param name="argScreen"></param>
        public virtual Vec2 getScreenToWorld(Vec2 argScreen)
        {
            Vec2 world = new Vec2();
            viewportTransform.getScreenToWorld(argScreen, world);
            return world;
        }

        /// <summary>
        /// takes the screen coordinates and returns the
        /// world coordinates.
        /// </summary>
        /// <param name="screenX"></param>
        /// <param name="screenY"></param>
        public virtual Vec2 getScreenToWorld(float screenX, float screenY)
        {
            Vec2 screen = new Vec2(screenX, screenY);
            viewportTransform.getScreenToWorld(screen, screen);
            return screen;
        }
    }
}