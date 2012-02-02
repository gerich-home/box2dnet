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

    /// <summary> A 2D column vector</summary>
    [Serializable]
    public class Vec2
    {
        public float X;
        public float Y;

        public Vec2() :
            this(0, 0)
        {
        }

        public Vec2(float x, float y)
        {
            X = x;
            Y = y;
        }

        public Vec2(Vec2 toCopy) :
            this(toCopy.X, toCopy.Y)
        {
        }

        /// <summary>
        /// Zero out this vector. 
        /// </summary>
        public void SetZero()
        {
            X = 0.0f;
            Y = 0.0f;
        }

        /// <summary>
        /// Set the vector component-wise. 
        /// </summary>
        public Vec2 Set(float x, float y)
        {
            X = x;
            Y = y;
            return this;
        }

        /// <summary>
        /// Set this vector to another vector. 
        /// </summary>
        public Vec2 Set(Vec2 v)
        {
            X = v.X;
            Y = v.Y;
            return this;
        }

        /// <summary>
        /// Return the sum of this vector and another; does not alter either one. 
        /// </summary>
        public Vec2 Add(Vec2 v)
        {
            return new Vec2(X + v.X, Y + v.Y);
        }

        /// <summary>
        /// Return the difference of this vector and another; does not alter either one.
        /// </summary>
        public Vec2 Sub(Vec2 v)
        {
            return new Vec2(X - v.X, Y - v.Y);
        }

        /// <summary>
        /// Return this vector multiplied by a scalar; does not alter this vector.
        /// </summary>
        public Vec2 Mul(float a)
        {
            return new Vec2(X * a, Y * a);
        }

        /// <summary>
        /// Return the negation of this vector; does not alter this vector.
        /// </summary>
        public Vec2 Negate()
        {
            return new Vec2(-X, -Y);
        }

        /// <summary>
        /// Flip the vector and return it - alters this vector. 
        /// </summary>
        public Vec2 NegateLocal()
        {
            X = -X;
            Y = -Y;
            return this;
        }

        /// <summary>
        /// Add another vector to this one and returns result - alters this vector. 
        /// </summary>
        public Vec2 AddLocal(Vec2 v)
        {
            X += v.X;
            Y += v.Y;
            return this;
        }

        /// <summary>
        /// Adds values to this vector and returns result - alters this vector. 
        /// </summary>
        public Vec2 AddLocal(float x, float y)
        {
            X += x;
            Y += y;
            return this;
        }

        /// <summary>
        /// Subtract another vector from this one and return result - alters this vector. 
        /// </summary>
        public Vec2 SubLocal(Vec2 v)
        {
            X -= v.X;
            Y -= v.Y;
            return this;
        }

        /// <summary>
        /// Multiply this vector by a number and return result - alters this vector.
        /// </summary>
        public Vec2 MulLocal(float a)
        {
            X *= a;
            Y *= a;
            return this;
        }

        /// <summary>
        /// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
        /// </summary>
        public Vec2 Skew()
        {
            return new Vec2(-Y, X);
        }

        /// <summary>
        /// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
        /// </summary>
        public void Skew(Vec2 result)
        {
            result.X = -Y;
            result.Y = X;
        }

        /// <summary>
        /// Return the length of this vector.
        /// </summary>
        public float Length()
        {
            return MathUtils.sqrt(X * X + Y * Y);
        }

        /// <summary>
        /// Return the squared length of this vector.
        /// </summary>
        public float LengthSquared()
        {
            return (X * X + Y * Y);
        }

        /// <summary>
        /// Normalize this vector and return the length before normalization. Alters this vector. 
        /// </summary>
        public float Normalize()
        {
            float length = Length();
            if (length < Settings.EPSILON)
            {
                return 0f;
            }

            float invLength = 1.0f / length;
            X *= invLength;
            Y *= invLength;
            return length;
        }

        /// <summary>
        /// Return a new vector that has positive components. 
        /// </summary>
        public Vec2 Abs()
        {
            return new Vec2(MathUtils.abs(X), MathUtils.abs(Y));
        }

        public void AbsLocal()
        {
            X = MathUtils.abs(X);
            Y = MathUtils.abs(Y);
        }

        /// <summary>
        /// Return a copy of this vector. 
        /// </summary>
        public Vec2 Clone()
        {
            return new Vec2(X, Y);
        }

        public override String ToString()
        {
            return string.Format("({0},{1})", X, Y);
        }

        /*
        * Static
        */

        public static Vec2 Abs(Vec2 a)
        {
            return new Vec2(MathUtils.abs(a.X), MathUtils.abs(a.Y));
        }

        public static void AbsToOut(Vec2 a, Vec2 result)
        {
            result.X = MathUtils.abs(a.X);
            result.Y = MathUtils.abs(a.Y);
        }

        public static float Dot(Vec2 a, Vec2 b)
        {
            return a.X * b.X + a.Y * b.Y;
        }

        public static float Cross(Vec2 a, Vec2 b)
        {
            return a.X * b.Y - a.Y * b.X;
        }

        public static Vec2 Cross(Vec2 a, float s)
        {
            return new Vec2(s * a.Y, (-s) * a.X);
        }

        public static void CrossToOut(Vec2 a, float s, Vec2 result)
        {
            float tempy = (-s) * a.X;
            result.X = s * a.Y;
            result.Y = tempy;
        }

        public static void CrossToOutUnsafe(Vec2 a, float s, Vec2 result)
        {
            Debug.Assert(result != a);
            result.X = s * a.Y;
            result.Y = (-s) * a.X;
        }

        public static Vec2 Cross(float s, Vec2 a)
        {
            return new Vec2((-s) * a.Y, s * a.X);
        }

        public static void CrossToOut(float s, Vec2 a, Vec2 result)
        {
            float tempY = s * a.X;
            result.X = (-s) * a.Y;
            result.Y = tempY;
        }

        public static void CrossToOutUnsafe(float s, Vec2 a, Vec2 result)
        {
            Debug.Assert(result != a);
            result.X = (-s) * a.Y;
            result.Y = s * a.X;
        }

        public static void NegateToOut(Vec2 a, Vec2 result)
        {
            result.X = -a.X;
            result.Y = -a.Y;
        }

        public static Vec2 Min(Vec2 a, Vec2 b)
        {
            return new Vec2(a.X < b.X ? a.X : b.X, a.Y < b.Y ? a.Y : b.Y);
        }

        public static Vec2 Max(Vec2 a, Vec2 b)
        {
            return new Vec2(a.X > b.X ? a.X : b.X, a.Y > b.Y ? a.Y : b.Y);
        }

        public static void MinToOut(Vec2 a, Vec2 b, Vec2 result)
        {
            result.X = a.X < b.X ? a.X : b.X;
            result.Y = a.Y < b.Y ? a.Y : b.Y;
        }

        public static void MaxToOut(Vec2 a, Vec2 b, Vec2 result)
        {
            result.X = a.X > b.X ? a.X : b.X;
            result.Y = a.Y > b.Y ? a.Y : b.Y;
        }

        private static int FloatToIntBits(float number)
        {
            return BitConverter.ToInt32(BitConverter.GetBytes(number), 0);
        }

        public override int GetHashCode()
        {
            // automatically generated by Eclipse
            const int prime = 31;
            int result = 1;
            result = prime * result + FloatToIntBits(X);
            result = prime * result + FloatToIntBits(Y);
            return result;
        }

        public override bool Equals(Object obj)
        {
            // automatically generated by Eclipse
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (GetType() != obj.GetType())
                return false;
            Vec2 other = (Vec2)obj;
            if (FloatToIntBits(X) != FloatToIntBits(other.X))
                return false;
            if (FloatToIntBits(Y) != FloatToIntBits(other.Y))
                return false;
            return true;
        }

        /// <summary>
        /// True if the vector represents a pair of valid, non-infinite floating point numbers.
        /// </summary>
        virtual public bool Valid
        {
            get
            {
                return !Single.IsNaN(X) && !Single.IsInfinity(X) && !Single.IsNaN(Y) && !Single.IsInfinity(Y);
            }
        }
    }
}