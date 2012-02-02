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
        private const long serialVersionUID = 2L;

        public readonly Vec2 ex;
        public readonly Vec2 ey;

        /// <summary>Convert the matrix to printable format. </summary>
        public override String ToString()
        {
            String s = "";
            s += ("[" + ex.X + "," + ey.X + "]\n");
            s += ("[" + ex.Y + "," + ey.Y + "]");
            return s;
        }

        /// <summary>
        /// Construct zero matrix. Note: this is NOT an identity matrix! djm fixed double allocation
        /// problem
        /// </summary>
        public Mat22()
        {
            ex = new Vec2();
            ey = new Vec2();
        }

        /// <summary>
        /// Create a matrix with given vectors as columns.
        /// </summary>
        /// <param name="c1">Column 1 of matrix</param>
        /// <param name="c2">Column 2 of matrix</param>
        public Mat22(Vec2 c1, Vec2 c2)
        {
            ex = c1.Clone();
            ey = c2.Clone();
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
            ex = new Vec2(exx, exy);
            ey = new Vec2(col2x, col2y);
        }

        /// <summary>
        /// Set as a copy of another matrix.
        /// </summary>
        /// <param name="m">Matrix to copy</param>
        public Mat22 set_Renamed(Mat22 m)
        {
            ex.X = m.ex.X;
            ex.Y = m.ex.Y;
            ey.X = m.ey.X;
            ey.Y = m.ey.Y;
            return this;
        }

        public Mat22 set_Renamed(float exx, float col2x, float exy, float col2y)
        {
            ex.X = exx;
            ex.Y = exy;
            ey.X = col2x;
            ey.Y = col2y;
            return this;
        }

        /// <summary>
        /// Return a clone of this matrix. djm fixed double allocation
        /// </summary>
        public virtual Mat22 Clone()
        {
            return new Mat22(ex, ey);
        }

        /// <summary>
        /// Set as a matrix representing a rotation.
        /// </summary>
        /// <param name="angle">Rotation (in radians) that matrix represents.</param>
        public void set_Renamed(float angle)
        {
            float c = MathUtils.cos(angle);
            float s = MathUtils.sin(angle);
            ex.X = c;
            ey.X = -s;
            ex.Y = s;
            ey.Y = c;
        }

        /// <summary>
        /// Set as the identity matrix.
        /// </summary>
        public void setIdentity()
        {
            ex.X = 1.0f;
            ey.X = 0.0f;
            ex.Y = 0.0f;
            ey.Y = 1.0f;
        }

        /// <summary>
        /// Set as the zero matrix.
        /// </summary>
        public void setZero()
        {
            ex.X = 0.0f;
            ey.X = 0.0f;
            ex.Y = 0.0f;
            ey.Y = 0.0f;
        }

        /// <summary>
        /// Extract the angle from this matrix (assumed to be a rotation matrix).
        /// </summary>
        /// <returns></returns>
        virtual public float Angle
        {
            get
            {
                return MathUtils.atan2(ex.Y, ex.X);
            }

        }

        /// <summary>
        /// Set by column vectors.
        /// </summary>
        /// <param name="c1">Column 1</param>
        /// <param name="c2">Column 2</param>
        public void set_Renamed(Vec2 c1, Vec2 c2)
        {
            ex.X = c1.X;
            ey.X = c2.X;
            ex.Y = c1.Y;
            ey.Y = c2.Y;
        }

        /// <summary>
        /// Returns the inverted Mat22 - does NOT invert the matrix locally!
        /// </summary>
        public Mat22 invert()
        {
            float a = ex.X;
            float b = ey.X;
            float c = ex.Y;
            float d = ey.Y;

            Mat22 B = new Mat22();
            float det = a * d - b * c;
            if (det != 0)
            {
                det = 1.0f / det;
            }
            B.ex.X = det * d;
            B.ey.X = (-det) * b;
            B.ex.Y = (-det) * c;
            B.ey.Y = det * a;
            return B;
        }

        public Mat22 invertLocal()
        {
            float a = ex.X;
            float b = ey.X;
            float c = ex.Y;
            float d = ey.Y;
            float det = a * d - b * c;
            if (det != 0)
            {
                det = 1.0f / det;
            }
            ex.X = det * d;
            ey.X = (-det) * b;
            ex.Y = (-det) * c;
            ey.Y = det * a;
            return this;
        }

        public void invertToOut(Mat22 out_Renamed)
        {
            float a = ex.X;
            float b = ey.X;
            float c = ex.Y;
            float d = ey.Y;
            float det = a * d - b * c;
            // b2Assert(det != 0.0f);
            det = 1.0f / det;
            out_Renamed.ex.X = det * d;
            out_Renamed.ey.X = (-det) * b;
            out_Renamed.ex.Y = (-det) * c;
            out_Renamed.ey.Y = det * a;
        }



        /// <summary>
        /// Return the matrix composed of the absolute values of all elements. djm: fixed double allocation
        /// </summary>
        /// <returns>Absolute value matrix</returns>
        public Mat22 abs()
        {
            return new Mat22(MathUtils.abs(ex.X), MathUtils.abs(ey.X), MathUtils.abs(ex.Y), MathUtils.abs(ey.Y));
        }

        /* djm: added */
        public void absLocal()
        {
            ex.AbsLocal();
            ey.AbsLocal();
        }

        /// <summary>
        /// Return the matrix composed of the absolute values of all elements.
        /// </summary>
        /// <returns>Absolute value matrix</returns>
        public static Mat22 abs(Mat22 R)
        {
            return R.abs();
        }

        /* djm created */
        public static void absToOut(Mat22 R, Mat22 out_Renamed)
        {
            out_Renamed.ex.X = MathUtils.abs(R.ex.X);
            out_Renamed.ex.Y = MathUtils.abs(R.ex.Y);
            out_Renamed.ey.X = MathUtils.abs(R.ey.X);
            out_Renamed.ey.Y = MathUtils.abs(R.ey.Y);
        }

        /// <summary>
        /// Multiply a vector by this matrix.
        /// </summary>
        /// <param name="v">Vector to multiply by matrix.</param>
        /// <returns>Resulting vector</returns>
        public Vec2 mul(Vec2 v)
        {
            return new Vec2(ex.X * v.X + ey.X * v.Y, ex.Y * v.X + ey.Y * v.Y);
        }

        public void mulToOut(Vec2 v, Vec2 out_Renamed)
        {
            float tempy = ex.Y * v.X + ey.Y * v.Y;
            out_Renamed.X = ex.X * v.X + ey.X * v.Y;
            out_Renamed.Y = tempy;
        }

        public void mulToOutUnsafe(Vec2 v, Vec2 out_Renamed)
        {
            Debug.Assert(v != out_Renamed);
            out_Renamed.X = ex.X * v.X + ey.X * v.Y;
            out_Renamed.Y = ex.Y * v.X + ey.Y * v.Y;
        }


        /// <summary>
        /// Multiply another matrix by this one (this one on left). djm optimized
        /// </summary>
        /// <param name="R"></param>
        /// <returns></returns>
        public Mat22 mul(Mat22 R)
        {
            /*
            * Mat22 C = new Mat22();C.set(this.mul(R.ex), this.mul(R.ey));return C;
            */
            Mat22 C = new Mat22();
            C.ex.X = ex.X * R.ex.X + ey.X * R.ex.Y;
            C.ex.Y = ex.Y * R.ex.X + ey.Y * R.ex.Y;
            C.ey.X = ex.X * R.ey.X + ey.X * R.ey.Y;
            C.ey.Y = ex.Y * R.ey.X + ey.Y * R.ey.Y;
            // C.set(ex,col2);
            return C;
        }

        public Mat22 mulLocal(Mat22 R)
        {
            mulToOut(R, this);
            return this;
        }

        public void mulToOut(Mat22 R, Mat22 out_Renamed)
        {
            float tempy1 = this.ex.Y * R.ex.X + this.ey.Y * R.ex.Y;
            float tempx1 = this.ex.X * R.ex.X + this.ey.X * R.ex.Y;
            out_Renamed.ex.X = tempx1;
            out_Renamed.ex.Y = tempy1;
            float tempy2 = this.ex.Y * R.ey.X + this.ey.Y * R.ey.Y;
            float tempx2 = this.ex.X * R.ey.X + this.ey.X * R.ey.Y;
            out_Renamed.ey.X = tempx2;
            out_Renamed.ey.Y = tempy2;
        }

        public void mulToOutUnsafe(Mat22 R, Mat22 out_Renamed)
        {
            Debug.Assert(out_Renamed != R);
            Debug.Assert(out_Renamed != this);
            out_Renamed.ex.X = this.ex.X * R.ex.X + this.ey.X * R.ex.Y;
            out_Renamed.ex.Y = this.ex.Y * R.ex.X + this.ey.Y * R.ex.Y;
            out_Renamed.ey.X = this.ex.X * R.ey.X + this.ey.X * R.ey.Y;
            out_Renamed.ey.Y = this.ex.Y * R.ey.X + this.ey.Y * R.ey.Y;
        }

        /// <summary>
        /// Multiply another matrix by the transpose of this one (transpose of this one on left). djm:
        /// optimized
        /// </summary>
        /// <param name="B"></param>
        /// <returns></returns>
        public Mat22 mulTrans(Mat22 B)
        {
            /*
            * Vec2 c1 = new Vec2(Vec2.dot(this.ex, B.ex), Vec2.dot(this.ey, B.ex)); Vec2 c2 = new
            * Vec2(Vec2.dot(this.ex, B.ey), Vec2.dot(this.ey, B.ey)); Mat22 C = new Mat22(); C.set(c1, c2);
            * return C;
            */

            Mat22 C = new Mat22();

            C.ex.X = Vec2.Dot(this.ex, B.ex);
            C.ex.Y = Vec2.Dot(this.ey, B.ex);

            C.ey.X = Vec2.Dot(this.ex, B.ey);
            C.ey.Y = Vec2.Dot(this.ey, B.ey);
            return C;
        }

        public Mat22 mulTransLocal(Mat22 B)
        {
            mulTransToOut(B, this);
            return this;
        }

        public void mulTransToOut(Mat22 B, Mat22 out_Renamed)
        {
            /*
            * out.ex.x = Vec2.dot(this.ex, B.ex); out.ex.y = Vec2.dot(this.ey, B.ex); out.ey.x =
            * Vec2.dot(this.ex, B.ey); out.ey.y = Vec2.dot(this.ey, B.ey);
            */

            float x1 = this.ex.X * B.ex.X + this.ex.Y * B.ex.Y;
            float y1 = this.ey.X * B.ex.X + this.ey.Y * B.ex.Y;
            float x2 = this.ex.X * B.ey.X + this.ex.Y * B.ey.Y;
            float y2 = this.ey.X * B.ey.X + this.ey.Y * B.ey.Y;
            out_Renamed.ex.X = x1;
            out_Renamed.ey.X = x2;
            out_Renamed.ex.Y = y1;
            out_Renamed.ey.Y = y2;
        }

        public void mulTransToOutUnsafe(Mat22 B, Mat22 out_Renamed)
        {
            Debug.Assert(B != out_Renamed);
            Debug.Assert(this != out_Renamed);
            out_Renamed.ex.X = this.ex.X * B.ex.X + this.ex.Y * B.ex.Y;
            out_Renamed.ey.X = this.ex.X * B.ey.X + this.ex.Y * B.ey.Y;
            out_Renamed.ex.Y = this.ey.X * B.ex.X + this.ey.Y * B.ex.Y;
            out_Renamed.ey.Y = this.ey.X * B.ey.X + this.ey.Y * B.ey.Y;
        }

        /// <summary>
        /// Multiply a vector by the transpose of this matrix.
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public Vec2 mulTrans(Vec2 v)
        {
            // return new Vec2(Vec2.dot(v, ex), Vec2.dot(v, col2));
            return new Vec2((v.X * ex.X + v.Y * ex.Y), (v.X * ey.X + v.Y * ey.Y));
        }

        /* djm added */
        public void mulTransToOut(Vec2 v, Vec2 out_Renamed)
        {
            /*
            * out.x = Vec2.dot(v, ex); out.y = Vec2.dot(v, col2);
            */

            float tempx = v.X * ex.X + v.Y * ex.Y;
            out_Renamed.Y = v.X * ey.X + v.Y * ey.Y;
            out_Renamed.X = tempx;
        }

        /// <summary>
        /// Add this matrix to B, return the result.
        /// </summary>
        /// <param name="B"></param>
        /// <returns></returns>
        public Mat22 add(Mat22 B)
        {
            // return new Mat22(ex.add(B.ex), col2.add(B.ey));
            Mat22 m = new Mat22();
            m.ex.X = ex.X + B.ex.X;
            m.ex.Y = ex.Y + B.ex.Y;
            m.ey.X = ey.X + B.ey.X;
            m.ey.Y = ey.Y + B.ey.Y;
            return m;
        }

        /// <summary>
        /// Add B to this matrix locally.
        /// </summary>
        /// <param name="B"></param>
        /// <returns></returns>
        public Mat22 addLocal(Mat22 B)
        {
            // ex.addLocal(B.ex);
            // col2.addLocal(B.ey);
            ex.X += B.ex.X;
            ex.Y += B.ex.Y;
            ey.X += B.ey.X;
            ey.Y += B.ey.Y;
            return this;
        }

        /// <summary>
        /// Solve A * x = b where A = this matrix.
        /// </summary>
        /// <returns>The vector x that solves the above equation.</returns>
        public Vec2 solve(Vec2 b)
        {
            float a11 = ex.X;
            float a12 = ey.X;
            float a21 = ex.Y;
            float a22 = ey.Y;
            float det = a11 * a22 - a12 * a21;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }
            Vec2 x = new Vec2(det * (a22 * b.X - a12 * b.Y), det * (a11 * b.Y - a21 * b.X));
            return x;
        }

        public void solveToOut(Vec2 b, Vec2 out_Renamed)
        {
            float a11 = ex.X;
            float a12 = ey.X;
            float a21 = ex.Y;
            float a22 = ey.Y;
            float det = a11 * a22 - a12 * a21;
            if (det != 0.0f)
            {
                det = 1.0f / det;
            }
            float tempy = det * (a11 * b.Y - a21 * b.X);
            out_Renamed.X = det * (a22 * b.X - a12 * b.Y);
            out_Renamed.Y = tempy;
        }

        public static Vec2 mul(Mat22 R, Vec2 v)
        {
            // return R.mul(v);
            return new Vec2(R.ex.X * v.X + R.ey.X * v.Y, R.ex.Y * v.X + R.ey.Y * v.Y);
        }

        public static void mulToOut(Mat22 R, Vec2 v, Vec2 out_Renamed)
        {
            float tempy = R.ex.Y * v.X + R.ey.Y * v.Y;
            out_Renamed.X = R.ex.X * v.X + R.ey.X * v.Y;
            out_Renamed.Y = tempy;
        }

        public static void mulToOutUnsafe(Mat22 R, Vec2 v, Vec2 out_Renamed)
        {
            Debug.Assert(v != out_Renamed);
            out_Renamed.X = R.ex.X * v.X + R.ey.X * v.Y;
            out_Renamed.Y = R.ex.Y * v.X + R.ey.Y * v.Y;
        }

        public static Mat22 mul(Mat22 A, Mat22 B)
        {
            // return A.mul(B);
            Mat22 C = new Mat22();
            C.ex.X = A.ex.X * B.ex.X + A.ey.X * B.ex.Y;
            C.ex.Y = A.ex.Y * B.ex.X + A.ey.Y * B.ex.Y;
            C.ey.X = A.ex.X * B.ey.X + A.ey.X * B.ey.Y;
            C.ey.Y = A.ex.Y * B.ey.X + A.ey.Y * B.ey.Y;
            return C;
        }

        public static void mulToOut(Mat22 A, Mat22 B, Mat22 out_Renamed)
        {
            float tempy1 = A.ex.Y * B.ex.X + A.ey.Y * B.ex.Y;
            float tempx1 = A.ex.X * B.ex.X + A.ey.X * B.ex.Y;
            float tempy2 = A.ex.Y * B.ey.X + A.ey.Y * B.ey.Y;
            float tempx2 = A.ex.X * B.ey.X + A.ey.X * B.ey.Y;
            out_Renamed.ex.X = tempx1;
            out_Renamed.ex.Y = tempy1;
            out_Renamed.ey.X = tempx2;
            out_Renamed.ey.Y = tempy2;
        }

        public static void mulToOutUnsafe(Mat22 A, Mat22 B, Mat22 out_Renamed)
        {
            Debug.Assert(out_Renamed != A);
            Debug.Assert(out_Renamed != B);
            out_Renamed.ex.X = A.ex.X * B.ex.X + A.ey.X * B.ex.Y;
            out_Renamed.ex.Y = A.ex.Y * B.ex.X + A.ey.Y * B.ex.Y;
            out_Renamed.ey.X = A.ex.X * B.ey.X + A.ey.X * B.ey.Y;
            out_Renamed.ey.Y = A.ex.Y * B.ey.X + A.ey.Y * B.ey.Y;
        }

        public static Vec2 mulTrans(Mat22 R, Vec2 v)
        {
            return new Vec2((v.X * R.ex.X + v.Y * R.ex.Y), (v.X * R.ey.X + v.Y * R.ey.Y));
        }

        public static void mulTransToOut(Mat22 R, Vec2 v, Vec2 out_Renamed)
        {
            float outx = v.X * R.ex.X + v.Y * R.ex.Y;
            out_Renamed.Y = v.X * R.ey.X + v.Y * R.ey.Y;
            out_Renamed.X = outx;
        }

        public static void mulTransToOutUnsafe(Mat22 R, Vec2 v, Vec2 out_Renamed)
        {
            Debug.Assert(out_Renamed != v);
            out_Renamed.Y = v.X * R.ey.X + v.Y * R.ey.Y;
            out_Renamed.X = v.X * R.ex.X + v.Y * R.ex.Y;
        }

        public static Mat22 mulTrans(Mat22 A, Mat22 B)
        {
            Mat22 C = new Mat22();
            C.ex.X = A.ex.X * B.ex.X + A.ex.Y * B.ex.Y;
            C.ex.Y = A.ey.X * B.ex.X + A.ey.Y * B.ex.Y;
            C.ey.X = A.ex.X * B.ey.X + A.ex.Y * B.ey.Y;
            C.ey.Y = A.ey.X * B.ey.X + A.ey.Y * B.ey.Y;
            return C;
        }

        public static void mulTransToOut(Mat22 A, Mat22 B, Mat22 out_Renamed)
        {
            float x1 = A.ex.X * B.ex.X + A.ex.Y * B.ex.Y;
            float y1 = A.ey.X * B.ex.X + A.ey.Y * B.ex.Y;
            float x2 = A.ex.X * B.ey.X + A.ex.Y * B.ey.Y;
            float y2 = A.ey.X * B.ey.X + A.ey.Y * B.ey.Y;

            out_Renamed.ex.X = x1;
            out_Renamed.ex.Y = y1;
            out_Renamed.ey.X = x2;
            out_Renamed.ey.Y = y2;
        }

        public static void mulTransToOutUnsafe(Mat22 A, Mat22 B, Mat22 out_Renamed)
        {
            Debug.Assert(A != out_Renamed);
            Debug.Assert(B != out_Renamed);
            out_Renamed.ex.X = A.ex.X * B.ex.X + A.ex.Y * B.ex.Y;
            out_Renamed.ex.Y = A.ey.X * B.ex.X + A.ey.Y * B.ex.Y;
            out_Renamed.ey.X = A.ex.X * B.ey.X + A.ex.Y * B.ey.Y;
            out_Renamed.ey.Y = A.ey.X * B.ey.X + A.ey.Y * B.ey.Y;
        }

        public static Mat22 createRotationalTransform(float angle)
        {
            Mat22 mat = new Mat22();
            float c = MathUtils.cos(angle);
            float s = MathUtils.sin(angle);
            mat.ex.X = c;
            mat.ey.X = -s;
            mat.ex.Y = s;
            mat.ey.Y = c;
            return mat;
        }

        public static void createRotationalTransform(float angle, Mat22 out_Renamed)
        {
            float c = MathUtils.cos(angle);
            float s = MathUtils.sin(angle);
            out_Renamed.ex.X = c;
            out_Renamed.ey.X = -s;
            out_Renamed.ex.Y = s;
            out_Renamed.ey.Y = c;
        }

        public static Mat22 createScaleTransform(float scale)
        {
            Mat22 mat = new Mat22();
            mat.ex.X = scale;
            mat.ey.Y = scale;
            return mat;
        }

        public static void createScaleTransform(float scale, Mat22 out_Renamed)
        {
            out_Renamed.ex.X = scale;
            out_Renamed.ey.Y = scale;
        }

        public override int GetHashCode()
        {
            int prime = 31;
            int result = 1;
            result = prime * result + ((ex == null) ? 0 : ex.GetHashCode());
            result = prime * result + ((ey == null) ? 0 : ey.GetHashCode());
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
            if (ex == null)
            {
                if (other.ex != null)
                    return false;
            }
            else if (!ex.Equals(other.ex))
                return false;
            if (ey == null)
            {
                if (other.ey != null)
                    return false;
            }
            else if (!ey.Equals(other.ey))
                return false;
            return true;
        }
    }
}