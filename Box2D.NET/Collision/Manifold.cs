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

using Box2D.Common;

namespace Box2D.Collision
{

    /// <summary>
    /// A manifold for two touching convex shapes. Box2D supports multiple types of contact:
    /// <ul>
    /// <li>clip point versus plane with radius</li>
    /// <li>point versus point with radius (circles)</li>
    /// </ul>
    /// The local point usage depends on the manifold type:
    /// <ul>
    /// <li>e_circles: the local center of circleA</li>
    /// <li>e_faceA: the center of faceA</li>
    /// <li>e_faceB: the center of faceB</li>
    /// </ul>
    /// Similarly the local normal usage:
    /// <ul>
    /// <li>e_circles: not used</li>
    /// <li>e_faceA: the normal on polygonA</li>
    /// <li>e_faceB: the normal on polygonB</li>
    /// </ul>
    /// We store contacts in this way so that position correction can account for movement, which is
    /// critical for continuous physics. All contact scenarios must be expressed in one of these types.
    /// This structure is stored across time steps, so we keep it small.
    /// </summary>
    public class Manifold
    {
        public enum ManifoldType
        {
            Circles,
            FaceA,
            FaceB
        }

        /// <summary>
        /// The points of contact. 
        /// </summary>
        public readonly ManifoldPoint[] Points;

        /// <summary>
        /// not use for Type::e_points
        ///  </summary>
        public readonly Vec2 LocalNormal;

        /// <summary>
        /// usage depends on manifold type 
        /// </summary>
        public readonly Vec2 LocalPoint;

        public ManifoldType Type;

        /// <summary>
        /// The number of manifold points. 
        /// </summary>
        public int PointCount;

        /// <summary> 
        /// creates a manifold with 0 points, with it's points array full of instantiated ManifoldPoints.
        /// </summary>
        public Manifold()
        {
            Points = new ManifoldPoint[Settings.MAX_MANIFOLD_POINTS];
            for (int i = 0; i < Settings.MAX_MANIFOLD_POINTS; i++)
            {
                Points[i] = new ManifoldPoint();
            }
            LocalNormal = new Vec2();
            LocalPoint = new Vec2();
            PointCount = 0;
        }

        /// <summary>
        /// Creates this manifold as a copy of the other
        /// </summary>
        /// <param name="other"></param>
        public Manifold(Manifold other)
        {
            Points = new ManifoldPoint[Settings.MAX_MANIFOLD_POINTS];
            LocalNormal = other.LocalNormal.Clone();
            LocalPoint = other.LocalPoint.Clone();
            PointCount = other.PointCount;
            Type = other.Type;
            // djm: this is correct now
            for (int i = 0; i < Settings.MAX_MANIFOLD_POINTS; i++)
            {
                Points[i] = new ManifoldPoint(other.Points[i]);
            }
        }

        /// <summary>
        /// copies this manifold from the given one
        /// </summary>
        /// <param name="cp">manifold to copy from</param>
        public virtual void Set(Manifold cp)
        {
            for (int i = 0; i < cp.PointCount; i++)
            {
                Points[i].Set(cp.Points[i]);
            }

            Type = cp.Type;
            LocalNormal.Set(cp.LocalNormal);
            LocalPoint.Set(cp.LocalPoint);
            PointCount = cp.PointCount;
        }
    }
}