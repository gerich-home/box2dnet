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
    /// A 2-by-2 matrix. Stored in column-major order.
    /// </summary>
    [Serializable]
    public class Mat22
    {
        public readonly Vec2 Ex;
        public readonly Vec2 Ey;

        /// <summary>
        /// Convert the matrix to printable format. 
        /// </summary>
        public override String ToString()
        {
            return string.Format("[{0},{1}]\n[{2},{3}]", Ex.X, Ey.X, Ex.Y, Ey.Y);
        }

        /// <summary>
        /// Construct zero matrix. Note: this is NOT an identity matrix! djm fixed double allocation
        /// problem
        /// </summary>
        public Mat22()
        {
            Ex = new Vec2();
            Ey = new Vec2();
        }

        /// <summary>
        /// Create a matrix with given vectors as columns.
        /// </summary>
        /// <param name="c1">Column 1 of matrix</param>
        /// <param name="c2">Column 2 of matrix</param>
        public Mat22(Vec2 c1, Vec2 c2)
        {
            Ex = c1.Clone();
            Ey = c2.Clone();
        }

        /// <summary>
        /// Create a matrix from four floats.
        /// </summary>
        /// <param name="exx"></param>
        /// <param name="col2x"></param>
        /// <param name="exy"></param>
        /// <param name="col2y"></param>
        public Mat22(float exx, float col2x, float exy, float col2y)
        {
            Ex = new Vec2(exx, exy);
            Ey = new Vec2(col2x, col2y);
        }

        /// <summary>
        /// Set as a copy of another matrix.
        /// </summary>
        /// <param name="m">Matrix to copy</param>
        public Mat22 Set(Mat22 m)
        {
            Ex.X = m.Ex.X;
            Ex.Y = m.Ex.Y;
            Ey.X = m.Ey.X;
            Ey.Y = m.Ey.Y;
            return this;
        }

        public Mat22 Set(float exx, float col2x, float exy, float col2y)
        {
            Ex.X = exx;
            Ex.Y = exy;
            Ey.X = col2x;
            Ey.Y = col2y;
            return this;
        }

        /// <summary>
        /// Return a clone of this matrix. djm fixed double allocation
        /// </summary>
        public virtual Mat22 Clone()
        {
            return new Mat22(Ex, Ey);
        }

        /// <summary>
        /// Set as a matrix representing a rotation.
        /// </summary>
        /// <param name="angle">Rotation (in radians) that matrix represents.</param>
        public void Set(float angle)
        {
            float c = MathUtils.Cos(angle);
            float s = MathUtils.Sin(angle);
            Ex.X = c;
            Ey.X = -s;
            Ex.Y = s;
            Ey.Y = c;
        }

        /// <summary>
        /// Set as the identity matrix.
        /// </summary>
        public void SetIdentity()
        {
            Ex.X = 1.0f;
            Ey.X = 0.0f;
            Ex.Y = 0.0f;
            Ey.Y = 1.0f;
        }

        /// <summary>
        /// Set as the zero matrix.
        /// </summary>
        public void SetZero()
        {
            Ex.X = 0.0f;
            Ey.X = 0.0f;
            Ex.Y = 0.0f;
            Ey.Y = 0.0f;
        }

        /// <summary>
        /// Extract the angle from this matrix (assumed to be a rotation matrix).
        /// </summary>
        /// <returns></returns>
        virtual public float Angle
        {
            get
            {
                return MathUtils.Atan2(Ex.Y, Ex.X);
            }
        }

        /// <summary>
        /// Set by column vectors.
        /// </summary>
        /// <param name="c1">Column 1</param>
        /// <param name="c2">Column 2</param>
        public void Set(Vec2 c1, Vec2 c2)
        {
            Ex.X = c1.X;
            Ey.X = c2.X;
            Ex.Y = c1.Y;
            Ey.Y = c2.Y;
        }

        /// <summary>
        /// Returns the inverted Mat22 - does NOT invert the matrix locally!
        /// </summary>
        public Mat22 Invert()
        {
            float a = Ex.X;
            float b = Ey.X;
            float c = Ex.Y;
            float d = Ey.Y;

            Mat22 B = new Mat22();
            float det = a * d - b * c;
            if (det != 0)
            {
                det = 1.0f / det;
            }
            B.Ex.X = det * d;
            B.Ey.X = (-det) * b;
            B.Ex.Y = (-det) * c;
            B.Ey.Y = det * a;
            return B;
        }

        public Mat22 InvertLocal()
        {
            float a = Ex.X;
            float b = Ey.X;
            float c = Ex.Y;
            float d = Ey.Y;
            float det = a * d - b * c;
            if (det != 0)
            {
                det = 1.0f / det;
            }
            Ex.X = det * d;
            Ey.X = (-det) * b;
            Ex.Y = (-det) * c;
            Ey.Y = det * a;
            return this;
        }

        public void InvertToOut(Mat22 result)
        {
            float a = Ex.X;
            float b = Ey.X;
            float c = Ex.Y;
            float d = Ey.Y;
            float det = a * d - b * c;
            // b2Assert(det != 0.0f);
            det = 1.0f / det;
            result.Ex.X = det * d;
            result.Ey.X = (-det) * b;
            result.Ex.Y = (-det) * c;
            result.Ey.Y = det * a;
        }



        /// <summary>
        /// Return the matrix composed of the absolute values of all elements. djm: fixed double allocation
        /// </summary>
        /// <returns>Absolute value matrix</returns>
        public Mat22 Abs()
        {
            return new Mat22(MathUtils.Abs(Ex.X), MathUtils.Abs(Ey.X), MathUtils.Abs(Ex.Y), MathUtils.Abs(Ey.Y));
        }

        /* djm: added */
        public void AbsLocal()
        {
            Ex.AbsLocal();
            Ey.AbsLocal();
        }

        /// <summary>
        /// Return the matrix composed of the absolute values of all elements.
        /// </summary>
        /// <returns>Absolute value matrix</returns>
        public static Mat22 Abs(Mat22 r)
        {
            return r.Abs();
        }

        /* djm created */
        public static void AbsToOut(Mat22 r, Mat22 result)
        {
            result.Ex.X = MathUtils.Abs(r.Ex.X);
            result.Ex.Y = MathUtils.Abs(r.Ex.Y);
            result.Ey.X = MathUtils.Abs(r.Ey.X);
            result.Ey.Y = MathUtils.Abs(r.Ey.Y);
        }

        /// <summary>
        /// Multiply a vector by this matrix.
        /// </summary>
        /// <param name="v">Vector to multiply by matrix.</param>
        /// <returns>Resulting vector</returns>
        public Vec2 Mul(Vec2 v)
        {
            return new Vec2(Ex.X * v.X + Ey.X * v.Y, Ex.Y * v.X + Ey.Y * v.Y);
        }

        public void MulToOut(Vec2 v, Vec2 result)
        {
            float tempy = Ex.Y * v.X + Ey.Y * v.Y;
            result.X = Ex.X * v.X + Ey.X * v.Y;
            result.Y = tempy;
        }

        public void MulToOutUnsafe(Vec2 v, Vec2 result)
        {
            Debug.Assert(v != result);
            result.X = Ex.X * v.X + Ey.X * v.Y;
            result.Y = Ex.Y * v.X + Ey.Y * v.Y;
        }


        /// <summary>
        /// Multiply another matrix by this one (this one on left). djm optimized
        /// </summary>
        /// <param name="r"></param>
        /// <returns></returns>
        public Mat22 Mul(Mat22 r)
        {
            /*
            * Mat22 C = new Mat22();C.set(this.mul(R.ex), this.mul(R.ey));return C;
            */
            Mat22 C = new Mat22();
            C.Ex.X = Ex.X * r.Ex.X + Ey.X * r.Ex.Y;
            C.Ex.Y = Ex.Y * r.Ex.X + Ey.Y * r.Ex.Y;
            C.Ey.X = Ex.X * r.Ey.X + Ey.X * r.Ey.Y;
            C.Ey.Y = Ex.Y * r.Ey.X + Ey.Y * r.Ey.Y;
            // C.set(ex,col2);
            return C;
        }

        public Mat22 MulLocal(Mat22 R)
        {
            MulToOut(R, this);
            return this;
        }

        public void MulToOut(Mat22 r, Mat22 result)
        {
            float tempy1 = Ex.Y * r.Ex.X + Ey.Y * r.Ex.Y;
            float tempx1 = Ex.X * r.Ex.X + Ey.X * r.Ex.Y;
            result.Ex.X = tempx1;
            result.Ex.Y = tempy1;
            float tempy2 = Ex.Y * r.Ey.X + Ey.Y * r.Ey.Y;
            float tempx2 = Ex.X * r.Ey.X + Ey.X * r.Ey.Y;
            result.Ey.X = tempx2;
            result.Ey.Y = tempy2;
        }

        public void MulToOutUnsafe(Mat22 r, Mat22 result)
        {
            Debug.Assert(result != r);
            Debug.Assert(result != this);
            result.Ex.X = Ex.X * r.Ex.X + Ey.X * r.Ex.Y;
            result.Ex.Y = Ex.Y * r.Ex.X + Ey.Y * r.Ex.Y;
            result.Ey.X = Ex.X * r.Ey.X + Ey.X * r.Ey.Y;
            result.Ey.Y = Ex.Y * r.Ey.X + Ey.Y * r.Ey.Y;
        }

        /// <summary>
        /// Multiply another matrix by the transpose of this one (transpose of this one on left). djm:
        /// optimized
        /// </summary>
        /// <param name="b"></param>
        /// <returns></returns>
        public Mat22 MulTrans(Mat22 b)
        {
            /*
            * Vec2 c1 = new Vec2(Vec2.dot(this.ex, B.ex), Vec2.dot(this.ey, B.ex)); Vec2 c2 = new
            * Vec2(Vec2.dot(this.ex, B.ey), Vec2.dot(this.ey, B.ey)); Mat22 C = new Mat22(); C.set(c1, c2);
            * return C;
            */

            Mat22 C = new Mat22();

            C.Ex.X = Vec2.Dot(this.Ex, b.Ex);
            C.Ex.Y = Vec2.Dot(this.Ey, b.Ex);

            C.Ey.X = Vec2.Dot(this.Ex, b.Ey);
            C.Ey.Y = Vec2.Dot(this.Ey, b.Ey);
            return C;
        }

        public Mat22 MulTransLocal(Mat22 b)
        {
            MulTransToOut(b, this);
            return this;
        }

        public void MulTransToOut(Mat22 b, Mat22 result)
        {
            /*
            * out.ex.x = Vec2.dot(this.ex, B.ex); out.ex.y = Vec2.dot(this.ey, B.ex); out.ey.x =
            * Vec2.dot(this.ex, B.ey); out.ey.y = Vec2.dot(this.ey, B.ey);
            */

            float x1 = Ex.X * b.Ex.X + Ex.Y * b.Ex.Y;
            float y1 = Ey.X * b.Ex.X + Ey.Y * b.Ex.Y;
            float x2 = Ex.X * b.Ey.X + Ex.Y * b.Ey.Y;
            float y2 = Ey.X * b.Ey.X + Ey.Y * b.Ey.Y;
            result.Ex.X = x1;
            result.Ey.X = x2;
            result.Ex.Y = y1;
            result.Ey.Y = y2;
        }

        public void MulTransToOutUnsafe(Mat22 b, Mat22 result)
        {
            Debug.Assert(b != result);
            Debug.Assert(this != result);
            result.Ex.X = Ex.X * b.Ex.X + Ex.Y * b.Ex.Y;
            result.Ey.X = Ex.X * b.Ey.X + Ex.Y * b.Ey.Y;
            result.Ex.Y = Ey.X * b.Ex.X + Ey.Y * b.Ex.Y;
            result.Ey.Y = Ey.X * b.Ey.X + Ey.Y * b.Ey.Y;
        }

        /// <summary>
        /// Multiply a vector by the transpose of this matrix.
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public Vec2 MulTrans(Vec2 v)
        {
            // return new Vec2(Vec2.dot(v, ex), Vec2.dot(v, col2));
            return new Vec2((v.X * Ex.X + v.Y * Ex.Y), (v.X * Ey.X + v.Y * Ey.Y));
        }

        /* djm added */
        public void MulTransToOut(Vec2 v, Vec2 result)
        {
            /*
            * out.x = Vec2.dot(v, ex); out.y = Vec2.dot(v, col2);
            */

            float tempx = v.X * Ex.X + v.Y * Ex.Y;
            result.Y = v.X * Ey.X + v.Y * Ey.Y;
            result.X = tempx;
        }

        /// <summary>
        /// Add this matrix to B, return the result.
        /// </summary>
        /// <param name="b"></param>
        /// <returns></returns>
        public Mat22 Add(Mat22 b)
        {
            // return new Mat22(ex.add(B.ex), col2.add(B.ey));
            Mat22 m = new Mat22();
            m.Ex.X = Ex.X + b.Ex.X;
            m.Ex.Y = Ex.Y + b.Ex.Y;
            m.Ey.X = Ey.X + b.Ey.X;
            m.Ey.Y = Ey.Y + b.Ey.Y;
            return m;
        }

        /// <summary>
        /// Add B to this matrix locally.
        /// </summary>
        /// <param name="b"></param>
        /// <returns></returns>
        public Mat22 AddLocal(Mat22 b)
        {
            // ex.addLocal(B.ex);
            // col2.addLocal(B.ey);
            Ex.X += b.Ex.X;
            Ex.Y += b.Ex.Y;
            Ey.X += b.Ey.X;
            Ey.Y += b.Ey.Y;
            return this;
        }

        /// <summary>
        /// Solve A * x = b where A = this matrix.
        /// </summary>
        /// <returns>The vector x that solves the above equation.</returns>
        public Vec2 Solve(Vec2 b)
        {
            float a11 = Ex.X;
            float a12 = Ey.X;
            float a21 = Ex.Y;
            float a22 = Ey.Y;
            float det = a11 * a22 - a12 * a21;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }
            Vec2 x = new Vec2(det * (a22 * b.X - a12 * b.Y), det * (a11 * b.Y - a21 * b.X));
            return x;
        }

        public void SolveToOut(Vec2 b, Vec2 result)
        {
            float a11 = Ex.X;
            float a12 = Ey.X;
            float a21 = Ex.Y;
            float a22 = Ey.Y;
            float det = a11 * a22 - a12 * a21;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }
            float tempy = det * (a11 * b.Y - a21 * b.X);
            result.X = det * (a22 * b.X - a12 * b.Y);
            result.Y = tempy;
        }

        public static Vec2 Mul(Mat22 r, Vec2 v)
        {
            // return R.mul(v);
            return new Vec2(r.Ex.X * v.X + r.Ey.X * v.Y, r.Ex.Y * v.X + r.Ey.Y * v.Y);
        }

        public static void MulToOut(Mat22 r, Vec2 v, Vec2 result)
        {
            float tempy = r.Ex.Y * v.X + r.Ey.Y * v.Y;
            result.X = r.Ex.X * v.X + r.Ey.X * v.Y;
            result.Y = tempy;
        }

        public static void MulToOutUnsafe(Mat22 r, Vec2 v, Vec2 result)
        {
            Debug.Assert(v != result);
            result.X = r.Ex.X * v.X + r.Ey.X * v.Y;
            result.Y = r.Ex.Y * v.X + r.Ey.Y * v.Y;
        }

        public static Mat22 Mul(Mat22 a, Mat22 b)
        {
            // return A.mul(B);
            Mat22 C = new Mat22();
            C.Ex.X = a.Ex.X * b.Ex.X + a.Ey.X * b.Ex.Y;
            C.Ex.Y = a.Ex.Y * b.Ex.X + a.Ey.Y * b.Ex.Y;
            C.Ey.X = a.Ex.X * b.Ey.X + a.Ey.X * b.Ey.Y;
            C.Ey.Y = a.Ex.Y * b.Ey.X + a.Ey.Y * b.Ey.Y;
            return C;
        }

        public static void MulToOut(Mat22 a, Mat22 b, Mat22 result)
        {
            float tempy1 = a.Ex.Y * b.Ex.X + a.Ey.Y * b.Ex.Y;
            float tempx1 = a.Ex.X * b.Ex.X + a.Ey.X * b.Ex.Y;
            float tempy2 = a.Ex.Y * b.Ey.X + a.Ey.Y * b.Ey.Y;
            float tempx2 = a.Ex.X * b.Ey.X + a.Ey.X * b.Ey.Y;
            result.Ex.X = tempx1;
            result.Ex.Y = tempy1;
            result.Ey.X = tempx2;
            result.Ey.Y = tempy2;
        }

        public static void MulToOutUnsafe(Mat22 a, Mat22 b, Mat22 result)
        {
            Debug.Assert(result != a);
            Debug.Assert(result != b);
            result.Ex.X = a.Ex.X * b.Ex.X + a.Ey.X * b.Ex.Y;
            result.Ex.Y = a.Ex.Y * b.Ex.X + a.Ey.Y * b.Ex.Y;
            result.Ey.X = a.Ex.X * b.Ey.X + a.Ey.X * b.Ey.Y;
            result.Ey.Y = a.Ex.Y * b.Ey.X + a.Ey.Y * b.Ey.Y;
        }

        public static Vec2 MulTrans(Mat22 r, Vec2 v)
        {
            return new Vec2((v.X * r.Ex.X + v.Y * r.Ex.Y), (v.X * r.Ey.X + v.Y * r.Ey.Y));
        }

        public static void MulTransToOut(Mat22 r, Vec2 v, Vec2 result)
        {
            float outx = v.X * r.Ex.X + v.Y * r.Ex.Y;
            result.Y = v.X * r.Ey.X + v.Y * r.Ey.Y;
            result.X = outx;
        }

        public static void MulTransToOutUnsafe(Mat22 r, Vec2 v, Vec2 result)
        {
            Debug.Assert(result != v);
            result.Y = v.X * r.Ey.X + v.Y * r.Ey.Y;
            result.X = v.X * r.Ex.X + v.Y * r.Ex.Y;
        }

        public static Mat22 MulTrans(Mat22 a, Mat22 b)
        {
            Mat22 C = new Mat22();
            C.Ex.X = a.Ex.X * b.Ex.X + a.Ex.Y * b.Ex.Y;
            C.Ex.Y = a.Ey.X * b.Ex.X + a.Ey.Y * b.Ex.Y;
            C.Ey.X = a.Ex.X * b.Ey.X + a.Ex.Y * b.Ey.Y;
            C.Ey.Y = a.Ey.X * b.Ey.X + a.Ey.Y * b.Ey.Y;
            return C;
        }

        public static void MulTransToOut(Mat22 a, Mat22 b, Mat22 result)
        {
            float x1 = a.Ex.X * b.Ex.X + a.Ex.Y * b.Ex.Y;
            float y1 = a.Ey.X * b.Ex.X + a.Ey.Y * b.Ex.Y;
            float x2 = a.Ex.X * b.Ey.X + a.Ex.Y * b.Ey.Y;
            float y2 = a.Ey.X * b.Ey.X + a.Ey.Y * b.Ey.Y;

            result.Ex.X = x1;
            result.Ex.Y = y1;
            result.Ey.X = x2;
            result.Ey.Y = y2;
        }

        public static void MulTransToOutUnsafe(Mat22 a, Mat22 b, Mat22 result)
        {
            Debug.Assert(a != result);
            Debug.Assert(b != result);
            result.Ex.X = a.Ex.X * b.Ex.X + a.Ex.Y * b.Ex.Y;
            result.Ex.Y = a.Ey.X * b.Ex.X + a.Ey.Y * b.Ex.Y;
            result.Ey.X = a.Ex.X * b.Ey.X + a.Ex.Y * b.Ey.Y;
            result.Ey.Y = a.Ey.X * b.Ey.X + a.Ey.Y * b.Ey.Y;
        }

        public static Mat22 CreateRotationalTransform(float angle)
        {
            Mat22 mat = new Mat22();
            float c = MathUtils.Cos(angle);
            float s = MathUtils.Sin(angle);
            mat.Ex.X = c;
            mat.Ey.X = -s;
            mat.Ex.Y = s;
            mat.Ey.Y = c;
            return mat;
        }

        public static void CreateRotationalTransform(float angle, Mat22 result)
        {
            float c = MathUtils.Cos(angle);
            float s = MathUtils.Sin(angle);
            result.Ex.X = c;
            result.Ey.X = -s;
            result.Ex.Y = s;
            result.Ey.Y = c;
        }

        public static Mat22 CreateScaleTransform(float scale)
        {
            Mat22 mat = new Mat22();
            mat.Ex.X = scale;
            mat.Ey.Y = scale;
            return mat;
        }

        public static void CreateScaleTransform(float scale, Mat22 result)
        {
            result.Ex.X = scale;
            result.Ey.Y = scale;
        }

        public override int GetHashCode()
        {
            const int prime = 31;
            int result = 1;
            result = prime * result + ((Ex == null) ? 0 : Ex.GetHashCode());
            result = prime * result + ((Ey == null) ? 0 : Ey.GetHashCode());
            return result;
        }

        public override bool Equals(Object obj)
        {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (GetType() != obj.GetType())
                return false;
            Mat22 other = (Mat22)obj;
            if (Ex == null)
            {
                if (other.Ex != null)
                    return false;
            }
            else if (!Ex.Equals(other.Ex))
                return false;
            if (Ey == null)
            {
                if (other.Ey != null)
                    return false;
            }
            else if (!Ey.Equals(other.Ey))
                return false;
            return true;
        }
    }
}