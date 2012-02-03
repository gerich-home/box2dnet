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
using System.Diagnostics;

namespace Box2D.Common
{

    /// <summary>
    /// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to
    /// the body origin, which may no coincide with the center of mass. However, to support dynamics we
    /// must interpolate the center of mass position.
    /// </summary>
    [Serializable]
    public class Sweep
    {
        /// <summary>
        /// Local center of mass position
        /// </summary>
        public readonly Vec2 LocalCenter;

        /// <summary>
        /// Center world positions
        /// </summary>
        public readonly Vec2 C0;
        public readonly Vec2 C;

        /// <summary>
        /// World angles
        /// </summary>
        public float A0;

        /// <summary>
        /// World angles
        /// </summary>
        public float A;

        /// <summary>
        /// Fraction of the current time step in the range [0,1] c0 and a0 are the positions at alpha0.
        /// </summary>
        public float Alpha0;

        public override String ToString()
        {
            return string.Format("Sweep:\nlocalCenter: {0}\nc0: {1}, c: {2}\na0: {3}, a: {4}\n", LocalCenter, C0, C, A0, A);
        }

        public Sweep()
        {
            LocalCenter = new Vec2();
            C0 = new Vec2();
            C = new Vec2();
        }

        public void Normalize()
        {
            float d = MathUtils.TWO_PI * MathUtils.Floor(A0 / MathUtils.TWO_PI);
            A0 -= d;
            A -= d;
        }

        public Sweep Set(Sweep argCloneFrom)
        {
            LocalCenter.Set(argCloneFrom.LocalCenter);
            C0.Set(argCloneFrom.C0);
            C.Set(argCloneFrom.C);
            A0 = argCloneFrom.A0;
            A = argCloneFrom.A;
            return this;
        }

        /// <summary>
        /// Get the interpolated transform at a specific time.
        /// </summary>
        /// <param name="xf">the result is placed here - must not be null</param>
        /// <param name="beta">the normalized time in [0,1].</param>
        public void GetTransform(Transform xf, float beta)
        {
            Debug.Assert(xf != null);
            // if (xf == null)
            // xf = new XForm();
            // center = p + R * localCenter
            /*
            * if (1.0f - t0 > Settings.EPSILON) { float alpha = (t - t0) / (1.0f - t0); xf.position.x =
            * (1.0f - alpha) * c0.x + alpha * c.x; xf.position.y = (1.0f - alpha) * c0.y + alpha * c.y;
            * float angle = (1.0f - alpha) * a0 + alpha * a; xf.R.set(angle); } else { xf.position.set(c);
            * xf.R.set(a); }
            */

            xf.P.X = (1.0f - beta) * C0.X + beta * C.X;
            xf.P.Y = (1.0f - beta) * C0.Y + beta * C.Y;
            // float angle = (1.0f - alpha) * a0 + alpha * a;
            // xf.R.set(angle);
            xf.Q.Set((1.0f - beta) * A0 + beta * A);

            // Shift to origin
            //xf->p -= b2Mul(xf->q, localCenter);
            Rot q = xf.Q;
            xf.P.X -= (q.Cos * LocalCenter.X - q.Sin * LocalCenter.Y);
            xf.P.Y -= (q.Sin * LocalCenter.X + q.Cos * LocalCenter.Y);
        }

        /// <summary>
        /// Advance the sweep forward, yielding a new initial state.
        /// </summary>
        /// <param name="alpha">the new initial time.</param>
        public void Advance(float alpha)
        {
            //    assert (alpha0 < 1f);
            //    // c0 = (1.0f - t) * c0 + t*c;
            //    float beta = (alpha - alpha0) / (1.0f - alpha0);
            //    c0.x = (1.0f - beta) * c0.x + beta * c.x;
            //    c0.y = (1.0f - beta) * c0.y + beta * c.y;
            //    a0 = (1.0f - beta) * a0 + beta * a;
            //    alpha0 = alpha;
            C0.X = (1.0f - alpha) * C0.X + alpha * C.X;
            C0.Y = (1.0f - alpha) * C0.Y + alpha * C.Y;
            A0 = (1.0f - alpha) * A0 + alpha * A;
        }
    }
}