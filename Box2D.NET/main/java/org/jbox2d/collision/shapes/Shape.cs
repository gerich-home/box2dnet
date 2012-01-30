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
using AABB = org.jbox2d.collision.AABB;
using RayCastInput = org.jbox2d.collision.RayCastInput;
using RayCastOutput = org.jbox2d.collision.RayCastOutput;
using Transform = org.jbox2d.common.Transform;
using Vec2 = org.jbox2d.common.Vec2;

namespace org.jbox2d.collision.shapes
{
    /// <summary>
    /// A shape is used for collision detection. You can create a shape however you like. Shapes used for
    /// simulation in World are created automatically when a Fixture is created. Shapes may encapsulate a
    /// one or more child shapes.
    /// </summary>
    public abstract class Shape
    {
        public readonly ShapeType m_type;
        public float m_radius;

        public Shape(ShapeType type)
        {
            this.m_type = type;
        }

        /// <summary>
        /// Get the type of this shape. You can use this to down cast to the concrete shape.
        /// </summary>
        /// <returns>the shape type.</returns>
        virtual public ShapeType Type
        {
            get
            {
                return m_type;
            }
        }

        /// <summary>
        /// Gets or sets the radius of the underlying shape. This can refer to different things depending on the shape
        /// implementation
        /// </summary>
        virtual public float Radius
        {
            get
            {
                return m_radius;
            }
            set
            {
                this.m_radius = value;
            }

        }

        /// <summary>
        /// Get the number of child primitives
        /// </summary>
        /// <returns></returns>
        public abstract int ChildCount { get; }

        /// <summary>
        /// Test a point for containment in this shape. This only works for convex shapes.
        /// </summary>
        /// <param name="xf">the shape world transform.</param>
        /// <param name="p">a point in world coordinates.</param>
        public abstract bool testPoint(Transform xf, Vec2 p);

        /// <summary>
        /// Cast a ray against a child shape.
        /// </summary>
        /// <param name="argOutput">the ray-cast results.</param>
        /// <param name="argInput">the ray-cast input parameters.</param>
        /// <param name="argTransform">the transform to be applied to the shape.</param>
        /// <param name="argChildIndex">the child shape index</param>
        /// <returns>if hit</returns>
        public abstract bool raycast(RayCastOutput output, RayCastInput input, Transform transform, int childIndex);

        /// <summary>
        /// Given a transform, compute the associated axis aligned bounding box for a child shape.
        /// </summary>
        /// <param name="argAabb">returns the axis aligned box.</param>
        /// <param name="argXf">the world transform of the shape.</param>
        public abstract void computeAABB(AABB aabb, Transform xf, int childIndex);

        /// <summary>
        /// Compute the mass properties of this shape using its dimensions and density. The inertia tensor
        /// is computed about the local origin.
        /// </summary>
        /// <param name="massData">returns the mass data for this shape.</param>
        /// <param name="density">the density in kilograms per meter squared.</param>
        public abstract void computeMass(MassData massData, float density);

        /*
         * /// <summary>
         * /// Compute the volume and centroid of this shape intersected with a half plane
         * /// </summary>
         * /// <param name="normal">the surface normal</param>
         * /// <param name="offset">the surface offset along normal</param>
         * /// <param name="xf">the shape transform</param>
         * /// <param name="c">returns the centroid</param>
         * /// <returns>the total volume less than offset along normal</returns>
         *
         * public abstract float computeSubmergedArea(Vec2 normal, float offset, Transform xf, Vec2 c);
         */

        abstract public Shape Clone();
    }
}