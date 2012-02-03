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

namespace Box2D.Common
{

    /// <summary>
    /// Orientated bounding box viewport transform
    /// </summary>
    /// <author>Daniel Murphy</author>
    public class OBBViewportTransform : IViewportTransform
    {
        /// <summary>
        /// Gets or sets the transform of the viewport, transforms around the center.
        /// </summary>
        public Mat22 Transform
        {
            get
            {
                return Box.R;
            }
            set
            {
                Box.R.Set(value);
            }
        }

        public bool YFlip { get; set; }

        public class OBB
        {
            public readonly Mat22 R = new Mat22();
            public readonly Vec2 Center = new Vec2();
            public readonly Vec2 Extents = new Vec2();
        }

        readonly internal OBB Box = new OBB();
        private readonly Mat22 yFlipMat = new Mat22(1, 0, 0, -1);
        private readonly Mat22 yFlipMatInv;

        public OBBViewportTransform()
        {
            YFlip = false;
            yFlipMatInv = yFlipMat.Invert();
            Box.R.SetIdentity();
        }

        public void Set(OBBViewportTransform vpt)
        {
            Box.Center.Set(vpt.Box.Center);
            Box.Extents.Set(vpt.Box.Extents);
            Box.R.Set(vpt.Box.R);
            YFlip = vpt.YFlip;
        }

        public void SetCamera(float x, float y, float scale)
        {
            Box.Center.Set(x, y);
            Mat22.CreateScaleTransform(scale, Box.R);
        }

        public Vec2 Extents
        {
            get { return Box.Extents; }
            set { Box.Extents.Set(value); }
        }

        public Vec2 Center
        {
            get { return Box.Center; }
            set { Box.Center.Set(value); }
        }

        public void SetCenter(float x, float y)
        {
            Box.Center.Set(x, y);
        }

        /// <summary>
        /// Multiplies the obb transform by the given transform
        /// </summary>
        /// <param name="argTransform"></param>
        public void MulByTransform(Mat22 argTransform)
        {
            Box.R.MulLocal(argTransform);
        }

        // djm pooling
        private readonly Mat22 inv = new Mat22();

        public void GetScreenVectorToWorld(Vec2 argScreen, Vec2 argWorld)
        {
            inv.Set(Box.R);
            inv.InvertLocal();
            inv.MulToOut(argScreen, argWorld);
            if (YFlip)
            {
                yFlipMatInv.MulToOut(argWorld, argWorld);
            }
        }

        public void GetWorldVectorToScreen(Vec2 argWorld, Vec2 argScreen)
        {
            Box.R.MulToOut(argWorld, argScreen);
            if (YFlip)
            {
                yFlipMatInv.MulToOut(argScreen, argScreen);
            }
        }

        public void GetWorldToScreen(Vec2 argWorld, Vec2 argScreen)
        {
            argScreen.Set(argWorld);
            argScreen.SubLocal(Box.Center);
            Box.R.MulToOut(argScreen, argScreen);
            if (YFlip)
            {
                yFlipMat.MulToOut(argScreen, argScreen);
            }
            argScreen.AddLocal(Box.Extents);
        }

        private readonly Mat22 inv2 = new Mat22();

        public void GetScreenToWorld(Vec2 argScreen, Vec2 argWorld)
        {
            argWorld.Set(argScreen);
            argWorld.SubLocal(Box.Extents);
            Box.R.InvertToOut(inv2);
            inv2.MulToOut(argWorld, argWorld);
            if (YFlip)
            {
                yFlipMatInv.MulToOut(argWorld, argWorld);
            }
            argWorld.AddLocal(Box.Center);
        }
    }
}