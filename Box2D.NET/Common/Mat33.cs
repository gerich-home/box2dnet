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
    /// A 3-by-3 matrix. Stored in column-major order.
    /// </summary>
    /// <author>Daniel Murphy</author>
    [Serializable]
    public class Mat33
    {
        public static readonly Mat33 Identity = new Mat33(new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(0, 0, 1));

        public readonly Vec3 Ex;
        public readonly Vec3 Ey;
        public readonly Vec3 Ez;

        public Mat33()
        {
            Ex = new Vec3();
            Ey = new Vec3();
            Ez = new Vec3();
        }

        public Mat33(Vec3 argCol1, Vec3 argCol2, Vec3 argCol3)
        {
            Ex = argCol1.Clone();
            Ey = argCol2.Clone();
            Ez = argCol3.Clone();
        }

        public void SetZero()
        {
            Ex.setZero();
            Ey.setZero();
            Ez.setZero();
        }

        // / Multiply a matrix times a vector.
        public static Vec3 Mul(Mat33 a, Vec3 v)
        {
            return new Vec3(v.x * a.Ex.x + v.y * a.Ey.x + v.z + a.Ez.x, v.x * a.Ex.y + v.y * a.Ey.y + v.z * a.Ez.y, v.x * a.Ex.z + v.y * a.Ey.z + v.z * a.Ez.z);
        }

        public static Vec2 Mul22(Mat33 a, Vec2 v)
        {
            return new Vec2(a.Ex.x * v.X + a.Ey.x * v.Y, a.Ex.y * v.X + a.Ey.y * v.Y);
        }

        public static void Mul22ToOut(Mat33 a, Vec2 v, Vec2 result)
        {
            float tempx = a.Ex.x * v.X + a.Ey.x * v.Y;
            result.Y = a.Ex.y * v.X + a.Ey.y * v.Y;
            result.X = tempx;
        }

        public static void Mul22ToOutUnsafe(Mat33 a, Vec2 v, Vec2 result)
        {
            Debug.Assert(v != result);
            result.Y = a.Ex.y * v.X + a.Ey.y * v.Y;
            result.X = a.Ex.x * v.X + a.Ey.x * v.Y;
        }

        public static void MulToOut(Mat33 a, Vec3 v, Vec3 result)
        {
            float tempy = v.x * a.Ex.y + v.y * a.Ey.y + v.z * a.Ez.y;
            float tempz = v.x * a.Ex.z + v.y * a.Ey.z + v.z * a.Ez.z;
            result.x = v.x * a.Ex.x + v.y * a.Ey.x + v.z * a.Ez.x;
            result.y = tempy;
            result.z = tempz;
        }

        public static void MulToOutUnsafe(Mat33 a, Vec3 v, Vec3 result)
        {
            Debug.Assert(result != v);
            result.x = v.x * a.Ex.x + v.y * a.Ey.x + v.z * a.Ez.x;
            result.y = v.x * a.Ex.y + v.y * a.Ey.y + v.z * a.Ez.y;
            result.z = v.x * a.Ex.z + v.y * a.Ey.z + v.z * a.Ez.z;
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
        /// in one-shot cases.
        /// </summary>
        /// <param name="b"></param>
        /// <returns></returns>
        public Vec2 Solve22(Vec2 b)
        {
            Vec2 x = new Vec2();
            Solve22ToOut(b, x);
            return x;
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
        /// in one-shot cases.
        /// </summary>
        /// <param name="b"></param>
        /// <param name="result"></param>
        /// <returns></returns>
        public void Solve22ToOut(Vec2 b, Vec2 result)
        {
            float a11 = Ex.x;
            float a12 = Ey.x;
            float a21 = Ex.y;
            float a22 = Ey.y;
            float det = a11 * a22 - a12 * a21;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }
            result.X = det * (a22 * b.X - a12 * b.Y);
            result.Y = det * (a11 * b.Y - a21 * b.X);
        }

        // djm pooling from below
        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
        /// in one-shot cases.
        /// </summary>
        /// <param name="b"></param>
        /// <returns></returns>
        public Vec3 Solve33(Vec3 b)
        {
            Vec3 x = new Vec3();
            Solve33ToOut(b, x);
            return x;
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
        /// in one-shot cases.
        /// </summary>
        /// <param name="b"></param>
        /// <param name="result">the result</param>
        public void Solve33ToOut(Vec3 b, Vec3 result)
        {
            Debug.Assert(b != result);
            Vec3.crossToOutUnsafe(Ey, Ez, result);
            float det = Vec3.dot(Ex, result);
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }
            Vec3.crossToOutUnsafe(Ey, Ez, result);
            float x = det * Vec3.dot(b, result);
            Vec3.crossToOutUnsafe(b, Ez, result);
            float y = det * Vec3.dot(Ex, result);
            Vec3.crossToOutUnsafe(Ey, b, result);
            float z = det * Vec3.dot(Ex, result);
            result.x = x;
            result.y = y;
            result.z = z;
        }

        public void GetInverse22(Mat33 m)
        {
            float a = Ex.x, b = Ey.x, c = Ex.y, d = Ey.y;
            float det = a * d - b * c;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }

            m.Ex.x = det * d;
            m.Ey.x = (-det) * b;
            m.Ex.z = 0.0f;
            m.Ex.y = (-det) * c;
            m.Ey.y = det * a;
            m.Ey.z = 0.0f;
            m.Ez.x = 0.0f;
            m.Ez.y = 0.0f;
            m.Ez.z = 0.0f;
        }

        // / Returns the zero matrix if singular.
        public void GetSymInverse33(Mat33 m)
        {
            float bx = Ey.y * Ez.z - Ey.z * Ez.y;
            float by = Ey.z * Ez.x - Ey.x * Ez.z;
            float bz = Ey.x * Ez.y - Ey.y * Ez.x;
            float det = Ex.x * bx + Ex.y * by + Ex.z * bz;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }

            float a11 = Ex.x, a12 = Ey.x, a13 = Ez.x;
            float a22 = Ey.y, a23 = Ez.y;
            float a33 = Ez.z;

            m.Ex.x = det * (a22 * a33 - a23 * a23);
            m.Ex.y = det * (a13 * a23 - a12 * a33);
            m.Ex.z = det * (a12 * a23 - a13 * a22);

            m.Ey.x = m.Ex.y;
            m.Ey.y = det * (a11 * a33 - a13 * a13);
            m.Ey.z = det * (a13 * a12 - a11 * a23);

            m.Ez.x = m.Ex.z;
            m.Ez.y = m.Ey.z;
            m.Ez.z = det * (a11 * a22 - a12 * a12);
        }

        public override int GetHashCode()
        {
            const int prime = 31;
            int result = 1;
            result = prime * result + ((Ex == null) ? 0 : Ex.GetHashCode());
            result = prime * result + ((Ey == null) ? 0 : Ey.GetHashCode());
            result = prime * result + ((Ez == null) ? 0 : Ez.GetHashCode());
            return result;
        }

        public override bool Equals(object obj)
        {
            if (this == obj) return true;
            if (obj == null) return false;
            if (obj is Mat33) return false;
            Mat33 other = (Mat33)obj;
            if (Ex == null)
            {
                if (other.Ex != null) return false;
            }
            else if (!Ex.Equals(other.Ex)) return false;
            if (Ey == null)
            {
                if (other.Ey != null) return false;
            }
            else if (!Ey.Equals(other.Ey)) return false;
            if (Ez == null)
            {
                if (other.Ez != null) return false;
            }
            else if (!Ez.Equals(other.Ez)) return false;
            return true;
        }
    }
}