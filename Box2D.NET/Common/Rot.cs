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
    /// Represents a rotation
    /// </summary>
    /// <author>Daniel</author>
    [Serializable]
    public sealed class Rot
    {
        public Rot()
        {
            SetIdentity();
        }

        public Rot(float angle)
        {
            Set(angle);
        }

        public float Sin { get; set; }

        public float Cos { get; set; }

        public override string ToString()
        {
            return "Rot(s:" + Sin + ", c:" + Cos + ")";
        }

        public float Angle
        {
            get
            {
                return MathUtils.Atan2(Sin, Cos);
            }
        }

        public Rot Set(float angle)
        {
            Sin = MathUtils.Sin(angle);
            Cos = MathUtils.Cos(angle);
            return this;
        }

        public Rot Set(Rot other)
        {
            Sin = other.Sin;
            Cos = other.Cos;
            return this;
        }

        public Rot SetIdentity()
        {
            Sin = 0;
            Cos = 1;
            return this;
        }

        public void GetXAxis(Vec2 xAxis)
        {
            xAxis.Set(Cos, Sin);
        }

        public void GetYAxis(Vec2 yAxis)
        {
            yAxis.Set(-Sin, Cos);
        }

        public Rot Clone()
        {
            Rot copy = new Rot();
            copy.Sin = Sin;
            copy.Cos = Cos;
            return copy;
        }

        public static void Mul(Rot q, Rot r, Rot result)
        {
            float tempc = q.Cos * r.Cos - q.Sin * r.Sin;
            result.Sin = q.Sin * r.Cos + q.Cos * r.Sin;
            result.Cos = tempc;
        }

        public static void MulUnsafe(Rot q, Rot r, Rot result)
        {
            Debug.Assert(r != result);
            Debug.Assert(q != result);
            // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
            // [qs qc] [rs rc] [qs*rc+qc*rs -qs*rs+qc*rc]
            // s = qs * rc + qc * rs
            // c = qc * rc - qs * rs
            result.Sin = q.Sin * r.Cos + q.Cos * r.Sin;
            result.Cos = q.Cos * r.Cos - q.Sin * r.Sin;
        }

        public static void MulTrans(Rot q, Rot r, Rot result)
        {
            float tempc = q.Cos * r.Cos + q.Sin * r.Sin;
            result.Sin = q.Cos * r.Sin - q.Sin * r.Cos;
            result.Cos = tempc;
        }

        public static void MulTransUnsafe(Rot q, Rot r, Rot result)
        {
            // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
            // [-qs qc] [rs rc] [-qs*rc+qc*rs qs*rs+qc*rc]
            // s = qc * rs - qs * rc
            // c = qc * rc + qs * rs
            result.Sin = q.Cos * r.Sin - q.Sin * r.Cos;
            result.Cos = q.Cos * r.Cos + q.Sin * r.Sin;
        }

        public static void MulToOut(Rot q, Vec2 v, Vec2 result)
        {
            float tempy = q.Sin * v.X + q.Cos * v.Y;
            result.X = q.Cos * v.X - q.Sin * v.Y;
            result.Y = tempy;
        }

        public static void MulToOutUnsafe(Rot q, Vec2 v, Vec2 result)
        {
            result.X = q.Cos * v.X - q.Sin * v.Y;
            result.Y = q.Sin * v.X + q.Cos * v.Y;
        }

        public static void MulTrans(Rot q, Vec2 v, Vec2 result)
        {
            float tempy = (-q.Sin) * v.X + q.Cos * v.Y;
            result.X = q.Cos * v.X + q.Sin * v.Y;
            result.Y = tempy;
        }

        public static void MulTransUnsafe(Rot q, Vec2 v, Vec2 result)
        {
            result.X = q.Cos * v.X + q.Sin * v.Y;
            result.Y = (-q.Sin) * v.X + q.Cos * v.Y;
        }
    }
}