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
            s += ("[" + ex.x + "," + ey.x + "]\n");
            s += ("[" + ex.y + "," + ey.y + "]");
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
            ex.x = m.ex.x;
            ex.y = m.ex.y;
            ey.x = m.ey.x;
            ey.y = m.ey.y;
            return this;
        }

        public Mat22 set_Renamed(float exx, float col2x, float exy, float col2y)
        {
            ex.x = exx;
            ex.y = exy;
            ey.x = col2x;
            ey.y = col2y;
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
            ex.x = c;
            ey.x = -s;
            ex.y = s;
            ey.y = c;
        }

        /// <summary>
        /// Set as the identity matrix.
        /// </summary>
        public void setIdentity()
        {
            ex.x = 1.0f;
            ey.x = 0.0f;
            ex.y = 0.0f;
            ey.y = 1.0f;
        }

        /// <summary>
        /// Set as the zero matrix.
        /// </summary>
        public void setZero()
        {
            ex.x = 0.0f;
            ey.x = 0.0f;
            ex.y = 0.0f;
            ey.y = 0.0f;
        }

        /// <summary>
        /// Extract the angle from this matrix (assumed to be a rotation matrix).
        /// </summary>
        /// <returns></returns>
        virtual public float Angle
        {
            get
            {
                return MathUtils.atan2(ex.y, ex.x);
            }

        }

        /// <summary>
        /// Set by column vectors.
        /// </summary>
        /// <param name="c1">Column 1</param>
        /// <param name="c2">Column 2</param>
        public void set_Renamed(Vec2 c1, Vec2 c2)
        {
            ex.x = c1.x;
            ey.x = c2.x;
            ex.y = c1.y;
            ey.y = c2.y;
        }

        /// <summary>
        /// Returns the inverted Mat22 - does NOT invert the matrix locally!
        /// </summary>
        public Mat22 invert()
        {
            float a = ex.x;
            float b = ey.x;
            float c = ex.y;
            float d = ey.y;

            Mat22 B = new Mat22();
            float det = a * d - b * c;
            if (det != 0)
            {
                det = 1.0f / det;
            }
            B.ex.x = det * d;
            B.ey.x = (-det) * b;
            B.ex.y = (-det) * c;
            B.ey.y = det * a;
            return B;
        }

        public Mat22 invertLocal()
        {
            float a = ex.x;
            float b = ey.x;
            float c = ex.y;
            float d = ey.y;
            float det = a * d - b * c;
            if (det != 0)
            {
                det = 1.0f / det;
            }
            ex.x = det * d;
            ey.x = (-det) * b;
            ex.y = (-det) * c;
            ey.y = det * a;
            return this;
        }

        public void invertToOut(Mat22 out_Renamed)
        {
            float a = ex.x;
            float b = ey.x;
            float c = ex.y;
            float d = ey.y;
            float det = a * d - b * c;
            // b2Assert(det != 0.0f);
            det = 1.0f / det;
            out_Renamed.ex.x = det * d;
            out_Renamed.ey.x = (-det) * b;
            out_Renamed.ex.y = (-det) * c;
            out_Renamed.ey.y = det * a;
        }



        /// <summary>
        /// Return the matrix composed of the absolute values of all elements. djm: fixed double allocation
        /// </summary>
        /// <returns>Absolute value matrix</returns>
        public Mat22 abs()
        {
            return new Mat22(MathUtils.abs(ex.x), MathUtils.abs(ey.x), MathUtils.abs(ex.y), MathUtils.abs(ey.y));
        }

        /* djm: added */
        public void absLocal()
        {
            ex.absLocal();
            ey.absLocal();
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
            out_Renamed.ex.x = MathUtils.abs(R.ex.x);
            out_Renamed.ex.y = MathUtils.abs(R.ex.y);
            out_Renamed.ey.x = MathUtils.abs(R.ey.x);
            out_Renamed.ey.y = MathUtils.abs(R.ey.y);
        }

        /// <summary>
        /// Multiply a vector by this matrix.
        /// </summary>
        /// <param name="v">Vector to multiply by matrix.</param>
        /// <returns>Resulting vector</returns>
        public Vec2 mul(Vec2 v)
        {
            return new Vec2(ex.x * v.x + ey.x * v.y, ex.y * v.x + ey.y * v.y);
        }

        public void mulToOut(Vec2 v, Vec2 out_Renamed)
        {
            float tempy = ex.y * v.x + ey.y * v.y;
            out_Renamed.x = ex.x * v.x + ey.x * v.y;
            out_Renamed.y = tempy;
        }

        public void mulToOutUnsafe(Vec2 v, Vec2 out_Renamed)
        {
            Debug.Assert(v != out_Renamed);
            out_Renamed.x = ex.x * v.x + ey.x * v.y;
            out_Renamed.y = ex.y * v.x + ey.y * v.y;
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
            C.ex.x = ex.x * R.ex.x + ey.x * R.ex.y;
            C.ex.y = ex.y * R.ex.x + ey.y * R.ex.y;
            C.ey.x = ex.x * R.ey.x + ey.x * R.ey.y;
            C.ey.y = ex.y * R.ey.x + ey.y * R.ey.y;
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
            float tempy1 = this.ex.y * R.ex.x + this.ey.y * R.ex.y;
            float tempx1 = this.ex.x * R.ex.x + this.ey.x * R.ex.y;
            out_Renamed.ex.x = tempx1;
            out_Renamed.ex.y = tempy1;
            float tempy2 = this.ex.y * R.ey.x + this.ey.y * R.ey.y;
            float tempx2 = this.ex.x * R.ey.x + this.ey.x * R.ey.y;
            out_Renamed.ey.x = tempx2;
            out_Renamed.ey.y = tempy2;
        }

        public void mulToOutUnsafe(Mat22 R, Mat22 out_Renamed)
        {
            Debug.Assert(out_Renamed != R);
            Debug.Assert(out_Renamed != this);
            out_Renamed.ex.x = this.ex.x * R.ex.x + this.ey.x * R.ex.y;
            out_Renamed.ex.y = this.ex.y * R.ex.x + this.ey.y * R.ex.y;
            out_Renamed.ey.x = this.ex.x * R.ey.x + this.ey.x * R.ey.y;
            out_Renamed.ey.y = this.ex.y * R.ey.x + this.ey.y * R.ey.y;
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

            C.ex.x = Vec2.dot(this.ex, B.ex);
            C.ex.y = Vec2.dot(this.ey, B.ex);

            C.ey.x = Vec2.dot(this.ex, B.ey);
            C.ey.y = Vec2.dot(this.ey, B.ey);
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

            float x1 = this.ex.x * B.ex.x + this.ex.y * B.ex.y;
            float y1 = this.ey.x * B.ex.x + this.ey.y * B.ex.y;
            float x2 = this.ex.x * B.ey.x + this.ex.y * B.ey.y;
            float y2 = this.ey.x * B.ey.x + this.ey.y * B.ey.y;
            out_Renamed.ex.x = x1;
            out_Renamed.ey.x = x2;
            out_Renamed.ex.y = y1;
            out_Renamed.ey.y = y2;
        }

        public void mulTransToOutUnsafe(Mat22 B, Mat22 out_Renamed)
        {
            Debug.Assert(B != out_Renamed);
            Debug.Assert(this != out_Renamed);
            out_Renamed.ex.x = this.ex.x * B.ex.x + this.ex.y * B.ex.y;
            out_Renamed.ey.x = this.ex.x * B.ey.x + this.ex.y * B.ey.y;
            out_Renamed.ex.y = this.ey.x * B.ex.x + this.ey.y * B.ex.y;
            out_Renamed.ey.y = this.ey.x * B.ey.x + this.ey.y * B.ey.y;
        }

        /// <summary>
        /// Multiply a vector by the transpose of this matrix.
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public Vec2 mulTrans(Vec2 v)
        {
            // return new Vec2(Vec2.dot(v, ex), Vec2.dot(v, col2));
            return new Vec2((v.x * ex.x + v.y * ex.y), (v.x * ey.x + v.y * ey.y));
        }

        /* djm added */
        public void mulTransToOut(Vec2 v, Vec2 out_Renamed)
        {
            /*
            * out.x = Vec2.dot(v, ex); out.y = Vec2.dot(v, col2);
            */

            float tempx = v.x * ex.x + v.y * ex.y;
            out_Renamed.y = v.x * ey.x + v.y * ey.y;
            out_Renamed.x = tempx;
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
            m.ex.x = ex.x + B.ex.x;
            m.ex.y = ex.y + B.ex.y;
            m.ey.x = ey.x + B.ey.x;
            m.ey.y = ey.y + B.ey.y;
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
            ex.x += B.ex.x;
            ex.y += B.ex.y;
            ey.x += B.ey.x;
            ey.y += B.ey.y;
            return this;
        }

        /// <summary>
        /// Solve A * x = b where A = this matrix.
        /// </summary>
        /// <returns>The vector x that solves the above equation.</returns>
        public Vec2 solve(Vec2 b)
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
            Vec2 x = new Vec2(det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x));
            return x;
        }

        public void solveToOut(Vec2 b, Vec2 out_Renamed)
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
            float tempy = det * (a11 * b.y - a21 * b.x);
            out_Renamed.x = det * (a22 * b.x - a12 * b.y);
            out_Renamed.y = tempy;
        }

        public static Vec2 mul(Mat22 R, Vec2 v)
        {
            // return R.mul(v);
            return new Vec2(R.ex.x * v.x + R.ey.x * v.y, R.ex.y * v.x + R.ey.y * v.y);
        }

        public static void mulToOut(Mat22 R, Vec2 v, Vec2 out_Renamed)
        {
            float tempy = R.ex.y * v.x + R.ey.y * v.y;
            out_Renamed.x = R.ex.x * v.x + R.ey.x * v.y;
            out_Renamed.y = tempy;
        }

        public static void mulToOutUnsafe(Mat22 R, Vec2 v, Vec2 out_Renamed)
        {
            Debug.Assert(v != out_Renamed);
            out_Renamed.x = R.ex.x * v.x + R.ey.x * v.y;
            out_Renamed.y = R.ex.y * v.x + R.ey.y * v.y;
        }

        public static Mat22 mul(Mat22 A, Mat22 B)
        {
            // return A.mul(B);
            Mat22 C = new Mat22();
            C.ex.x = A.ex.x * B.ex.x + A.ey.x * B.ex.y;
            C.ex.y = A.ex.y * B.ex.x + A.ey.y * B.ex.y;
            C.ey.x = A.ex.x * B.ey.x + A.ey.x * B.ey.y;
            C.ey.y = A.ex.y * B.ey.x + A.ey.y * B.ey.y;
            return C;
        }

        public static void mulToOut(Mat22 A, Mat22 B, Mat22 out_Renamed)
        {
            float tempy1 = A.ex.y * B.ex.x + A.ey.y * B.ex.y;
            float tempx1 = A.ex.x * B.ex.x + A.ey.x * B.ex.y;
            float tempy2 = A.ex.y * B.ey.x + A.ey.y * B.ey.y;
            float tempx2 = A.ex.x * B.ey.x + A.ey.x * B.ey.y;
            out_Renamed.ex.x = tempx1;
            out_Renamed.ex.y = tempy1;
            out_Renamed.ey.x = tempx2;
            out_Renamed.ey.y = tempy2;
        }

        public static void mulToOutUnsafe(Mat22 A, Mat22 B, Mat22 out_Renamed)
        {
            Debug.Assert(out_Renamed != A);
            Debug.Assert(out_Renamed != B);
            out_Renamed.ex.x = A.ex.x * B.ex.x + A.ey.x * B.ex.y;
            out_Renamed.ex.y = A.ex.y * B.ex.x + A.ey.y * B.ex.y;
            out_Renamed.ey.x = A.ex.x * B.ey.x + A.ey.x * B.ey.y;
            out_Renamed.ey.y = A.ex.y * B.ey.x + A.ey.y * B.ey.y;
        }

        public static Vec2 mulTrans(Mat22 R, Vec2 v)
        {
            return new Vec2((v.x * R.ex.x + v.y * R.ex.y), (v.x * R.ey.x + v.y * R.ey.y));
        }

        public static void mulTransToOut(Mat22 R, Vec2 v, Vec2 out_Renamed)
        {
            float outx = v.x * R.ex.x + v.y * R.ex.y;
            out_Renamed.y = v.x * R.ey.x + v.y * R.ey.y;
            out_Renamed.x = outx;
        }

        public static void mulTransToOutUnsafe(Mat22 R, Vec2 v, Vec2 out_Renamed)
        {
            Debug.Assert(out_Renamed != v);
            out_Renamed.y = v.x * R.ey.x + v.y * R.ey.y;
            out_Renamed.x = v.x * R.ex.x + v.y * R.ex.y;
        }

        public static Mat22 mulTrans(Mat22 A, Mat22 B)
        {
            Mat22 C = new Mat22();
            C.ex.x = A.ex.x * B.ex.x + A.ex.y * B.ex.y;
            C.ex.y = A.ey.x * B.ex.x + A.ey.y * B.ex.y;
            C.ey.x = A.ex.x * B.ey.x + A.ex.y * B.ey.y;
            C.ey.y = A.ey.x * B.ey.x + A.ey.y * B.ey.y;
            return C;
        }

        public static void mulTransToOut(Mat22 A, Mat22 B, Mat22 out_Renamed)
        {
            float x1 = A.ex.x * B.ex.x + A.ex.y * B.ex.y;
            float y1 = A.ey.x * B.ex.x + A.ey.y * B.ex.y;
            float x2 = A.ex.x * B.ey.x + A.ex.y * B.ey.y;
            float y2 = A.ey.x * B.ey.x + A.ey.y * B.ey.y;

            out_Renamed.ex.x = x1;
            out_Renamed.ex.y = y1;
            out_Renamed.ey.x = x2;
            out_Renamed.ey.y = y2;
        }

        public static void mulTransToOutUnsafe(Mat22 A, Mat22 B, Mat22 out_Renamed)
        {
            Debug.Assert(A != out_Renamed);
            Debug.Assert(B != out_Renamed);
            out_Renamed.ex.x = A.ex.x * B.ex.x + A.ex.y * B.ex.y;
            out_Renamed.ex.y = A.ey.x * B.ex.x + A.ey.y * B.ex.y;
            out_Renamed.ey.x = A.ex.x * B.ey.x + A.ex.y * B.ey.y;
            out_Renamed.ey.y = A.ey.x * B.ey.x + A.ey.y * B.ey.y;
        }

        public static Mat22 createRotationalTransform(float angle)
        {
            Mat22 mat = new Mat22();
            float c = MathUtils.cos(angle);
            float s = MathUtils.sin(angle);
            mat.ex.x = c;
            mat.ey.x = -s;
            mat.ex.y = s;
            mat.ey.y = c;
            return mat;
        }

        public static void createRotationalTransform(float angle, Mat22 out_Renamed)
        {
            float c = MathUtils.cos(angle);
            float s = MathUtils.sin(angle);
            out_Renamed.ex.x = c;
            out_Renamed.ey.x = -s;
            out_Renamed.ex.y = s;
            out_Renamed.ey.y = c;
        }

        public static Mat22 createScaleTransform(float scale)
        {
            Mat22 mat = new Mat22();
            mat.ex.x = scale;
            mat.ey.y = scale;
            return mat;
        }

        public static void createScaleTransform(float scale, Mat22 out_Renamed)
        {
            out_Renamed.ex.x = scale;
            out_Renamed.ey.y = scale;
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