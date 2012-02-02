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
        private const long serialVersionUID = 2L;

        public static readonly Mat33 IDENTITY = new Mat33(new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(0, 0, 1));

        public readonly Vec3 ex;
        public readonly Vec3 ey;
        public readonly Vec3 ez;

        public Mat33()
        {
            ex = new Vec3();
            ey = new Vec3();
            ez = new Vec3();
        }

        public Mat33(Vec3 argCol1, Vec3 argCol2, Vec3 argCol3)
        {
            ex = argCol1.Clone();
            ey = argCol2.Clone();
            ez = argCol3.Clone();
        }

        public virtual void setZero()
        {
            ex.setZero();
            ey.setZero();
            ez.setZero();
        }

        // / Multiply a matrix times a vector.
        public static Vec3 mul(Mat33 A, Vec3 v)
        {
            return new Vec3(v.x * A.ex.x + v.y * A.ey.x + v.z + A.ez.x, v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y, v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z);
        }

        public static Vec2 mul22(Mat33 A, Vec2 v)
        {
            return new Vec2(A.ex.x * v.X + A.ey.x * v.Y, A.ex.y * v.X + A.ey.y * v.Y);
        }

        public static void mul22ToOut(Mat33 A, Vec2 v, Vec2 out_Renamed)
        {
            float tempx = A.ex.x * v.X + A.ey.x * v.Y;
            out_Renamed.Y = A.ex.y * v.X + A.ey.y * v.Y;
            out_Renamed.X = tempx;
        }

        public static void mul22ToOutUnsafe(Mat33 A, Vec2 v, Vec2 out_Renamed)
        {
            Debug.Assert(v != out_Renamed);
            out_Renamed.Y = A.ex.y * v.X + A.ey.y * v.Y;
            out_Renamed.X = A.ex.x * v.X + A.ey.x * v.Y;
        }

        public static void mulToOut(Mat33 A, Vec3 v, Vec3 out_Renamed)
        {
            float tempy = v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y;
            float tempz = v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z;
            out_Renamed.x = v.x * A.ex.x + v.y * A.ey.x + v.z * A.ez.x;
            out_Renamed.y = tempy;
            out_Renamed.z = tempz;
        }

        public static void mulToOutUnsafe(Mat33 A, Vec3 v, Vec3 out_Renamed)
        {
            Debug.Assert(out_Renamed != v);
            out_Renamed.x = v.x * A.ex.x + v.y * A.ey.x + v.z * A.ez.x;
            out_Renamed.y = v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y;
            out_Renamed.z = v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z;
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
        /// in one-shot cases.
        /// </summary>
        /// <param name="b"></param>
        /// <returns></returns>
        public Vec2 solve22(Vec2 b)
        {
            Vec2 x = new Vec2();
            solve22ToOut(b, x);
            return x;
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
        /// in one-shot cases.
        /// </summary>
        /// <param name="b"></param>
        /// <returns></returns>
        public void solve22ToOut(Vec2 b, Vec2 out_Renamed)
        {
            float a11 = ex.x;
            float a12 = ey.x;
            float a21 = ex.y;
            float a22 = ey.y;
            float det = a11 * a22 - a12 * a21;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }
            out_Renamed.X = det * (a22 * b.X - a12 * b.Y);
            out_Renamed.Y = det * (a11 * b.Y - a21 * b.X);
        }

        // djm pooling from below
        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
        /// in one-shot cases.
        /// </summary>
        /// <param name="b"></param>
        /// <returns></returns>
        public Vec3 solve33(Vec3 b)
        {
            Vec3 x = new Vec3();
            solve33ToOut(b, x);
            return x;
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
        /// in one-shot cases.
        /// </summary>
        /// <param name="b"></param>
        /// <param name="out">the result</param>
        public void solve33ToOut(Vec3 b, Vec3 out_Renamed)
        {
            Debug.Assert(b != out_Renamed);
            Vec3.crossToOutUnsafe(ey, ez, out_Renamed);
            float det = Vec3.dot(ex, out_Renamed);
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }
            Vec3.crossToOutUnsafe(ey, ez, out_Renamed);
            float x = det * Vec3.dot(b, out_Renamed);
            Vec3.crossToOutUnsafe(b, ez, out_Renamed);
            float y = det * Vec3.dot(ex, out_Renamed);
            Vec3.crossToOutUnsafe(ey, b, out_Renamed);
            float z = det * Vec3.dot(ex, out_Renamed);
            out_Renamed.x = x;
            out_Renamed.y = y;
            out_Renamed.z = z;
        }

        public virtual void getInverse22(Mat33 M)
        {
            float a = ex.x, b = ey.x, c = ex.y, d = ey.y;
            float det = a * d - b * c;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }

            M.ex.x = det * d;
            M.ey.x = (-det) * b;
            M.ex.z = 0.0f;
            M.ex.y = (-det) * c;
            M.ey.y = det * a;
            M.ey.z = 0.0f;
            M.ez.x = 0.0f;
            M.ez.y = 0.0f;
            M.ez.z = 0.0f;
        }

        // / Returns the zero matrix if singular.
        public virtual void getSymInverse33(Mat33 M)
        {
            float bx = ey.y * ez.z - ey.z * ez.y;
            float by = ey.z * ez.x - ey.x * ez.z;
            float bz = ey.x * ez.y - ey.y * ez.x;
            float det = ex.x * bx + ex.y * by + ex.z * bz;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }

            float a11 = ex.x, a12 = ey.x, a13 = ez.x;
            float a22 = ey.y, a23 = ez.y;
            float a33 = ez.z;

            M.ex.x = det * (a22 * a33 - a23 * a23);
            M.ex.y = det * (a13 * a23 - a12 * a33);
            M.ex.z = det * (a12 * a23 - a13 * a22);

            M.ey.x = M.ex.y;
            M.ey.y = det * (a11 * a33 - a13 * a13);
            M.ey.z = det * (a13 * a12 - a11 * a23);

            M.ez.x = M.ex.z;
            M.ez.y = M.ey.z;
            M.ez.z = det * (a11 * a22 - a12 * a12);
        }

        public override int GetHashCode()
        {
            int prime = 31;
            int result = 1;
            result = prime * result + ((ex == null) ? 0 : ex.GetHashCode());
            result = prime * result + ((ey == null) ? 0 : ey.GetHashCode());
            result = prime * result + ((ez == null) ? 0 : ez.GetHashCode());
            return result;
        }

        public override bool Equals(object obj)
        {
            if (this == obj) return true;
            if (obj == null) return false;
            if (obj is Mat33) return false;
            Mat33 other = (Mat33)obj;
            if (ex == null)
            {
                if (other.ex != null) return false;
            }
            else if (!ex.Equals(other.ex)) return false;
            if (ey == null)
            {
                if (other.ey != null) return false;
            }
            else if (!ey.Equals(other.ey)) return false;
            if (ez == null)
            {
                if (other.ez != null) return false;
            }
            else if (!ez.Equals(other.ez)) return false;
            return true;
        }
    }
}