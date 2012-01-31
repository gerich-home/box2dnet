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

namespace org.jbox2d.common
{

    /// <summary> A 2D column vector</summary>
    [Serializable]
    public class Vec2
    {
        private const long serialVersionUID = 1L;

        public float x, y;

        public Vec2() :
            this(0, 0)
        {
        }

        public Vec2(float x, float y)
        {
            this.x = x;
            this.y = y;
        }

        public Vec2(Vec2 toCopy) :
            this(toCopy.x, toCopy.y)
        {
        }

        /// <summary>Zero out this vector. </summary>
        public void setZero()
        {
            x = 0.0f;
            y = 0.0f;
        }

        /// <summary>Set the vector component-wise. </summary>
        public Vec2 set_Renamed(float x, float y)
        {
            this.x = x;
            this.y = y;
            return this;
        }

        /// <summary>Set this vector to another vector. </summary>
        public Vec2 set_Renamed(Vec2 v)
        {
            this.x = v.x;
            this.y = v.y;
            return this;
        }

        /// <summary>Return the sum of this vector and another; does not alter either one. </summary>
        public Vec2 add(Vec2 v)
        {
            return new Vec2(x + v.x, y + v.y);
        }

        /// <summary>Return the difference of this vector and another; does not alter either one. </summary>
        public Vec2 sub(Vec2 v)
        {
            return new Vec2(x - v.x, y - v.y);
        }

        /// <summary>Return this vector multiplied by a scalar; does not alter this vector. </summary>
        public Vec2 mul(float a)
        {
            return new Vec2(x * a, y * a);
        }

        /// <summary>Return the negation of this vector; does not alter this vector. </summary>
        public Vec2 negate()
        {
            return new Vec2(-x, -y);
        }

        /// <summary>Flip the vector and return it - alters this vector. </summary>
        public Vec2 negateLocal()
        {
            x = -x;
            y = -y;
            return this;
        }

        /// <summary>Add another vector to this one and returns result - alters this vector. </summary>
        public Vec2 addLocal(Vec2 v)
        {
            x += v.x;
            y += v.y;
            return this;
        }

        /// <summary>Adds values to this vector and returns result - alters this vector. </summary>
        public Vec2 addLocal(float x, float y)
        {
            this.x += x;
            this.y += y;
            return this;
        }

        /// <summary>Subtract another vector from this one and return result - alters this vector. </summary>
        public Vec2 subLocal(Vec2 v)
        {
            x -= v.x;
            y -= v.y;
            return this;
        }

        /// <summary>Multiply this vector by a number and return result - alters this vector. </summary>
        public Vec2 mulLocal(float a)
        {
            x *= a;
            y *= a;
            return this;
        }

        /// <summary>Get the skew vector such that dot(skew_vec, other) == cross(vec, other) </summary>
        public Vec2 skew()
        {
            return new Vec2(-y, x);
        }

        /// <summary>Get the skew vector such that dot(skew_vec, other) == cross(vec, other) </summary>
        public void skew(Vec2 out_Renamed)
        {
            out_Renamed.x = -y;
            out_Renamed.y = x;
        }

        /// <summary>Return the length of this vector. </summary>
        public float length()
        {
            return MathUtils.sqrt(x * x + y * y);
        }

        /// <summary>Return the squared length of this vector. </summary>
        public float lengthSquared()
        {
            return (x * x + y * y);
        }

        /// <summary>Normalize this vector and return the length before normalization. Alters this vector. </summary>
        public float normalize()
        {
            float length = this.length();
            if (length < Settings.EPSILON)
            {
                return 0f;
            }

            float invLength = 1.0f / length;
            x *= invLength;
            y *= invLength;
            return length;
        }

        /// <summary>Return a new vector that has positive components. </summary>
        public Vec2 abs()
        {
            return new Vec2(MathUtils.abs(x), MathUtils.abs(y));
        }

        public void absLocal()
        {
            x = MathUtils.abs(x);
            y = MathUtils.abs(y);
        }

        /// <summary>Return a copy of this vector. </summary>
        public Vec2 Clone()
        {
            return new Vec2(x, y);
        }

        public override String ToString()
        {
            return "(" + x + "," + y + ")";
        }

        /*
        * Static
        */

        public static Vec2 abs(Vec2 a)
        {
            return new Vec2(MathUtils.abs(a.x), MathUtils.abs(a.y));
        }

        public static void absToOut(Vec2 a, Vec2 out_Renamed)
        {
            out_Renamed.x = MathUtils.abs(a.x);
            out_Renamed.y = MathUtils.abs(a.y);
        }

        public static float dot(Vec2 a, Vec2 b)
        {
            return a.x * b.x + a.y * b.y;
        }

        public static float cross(Vec2 a, Vec2 b)
        {
            return a.x * b.y - a.y * b.x;
        }

        public static Vec2 cross(Vec2 a, float s)
        {
            return new Vec2(s * a.y, (-s) * a.x);
        }

        public static void crossToOut(Vec2 a, float s, Vec2 out_Renamed)
        {
            float tempy = (-s) * a.x;
            out_Renamed.x = s * a.y;
            out_Renamed.y = tempy;
        }

        public static void crossToOutUnsafe(Vec2 a, float s, Vec2 out_Renamed)
        {
            Debug.Assert(out_Renamed != a);
            out_Renamed.x = s * a.y;
            out_Renamed.y = (-s) * a.x;
        }

        public static Vec2 cross(float s, Vec2 a)
        {
            return new Vec2((-s) * a.y, s * a.x);
        }

        public static void crossToOut(float s, Vec2 a, Vec2 out_Renamed)
        {
            float tempY = s * a.x;
            out_Renamed.x = (-s) * a.y;
            out_Renamed.y = tempY;
        }

        public static void crossToOutUnsafe(float s, Vec2 a, Vec2 out_Renamed)
        {
            Debug.Assert(out_Renamed != a);
            out_Renamed.x = (-s) * a.y;
            out_Renamed.y = s * a.x;
        }

        public static void negateToOut(Vec2 a, Vec2 out_Renamed)
        {
            out_Renamed.x = -a.x;
            out_Renamed.y = -a.y;
        }

        public static Vec2 min(Vec2 a, Vec2 b)
        {
            return new Vec2(a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y);
        }

        public static Vec2 max(Vec2 a, Vec2 b)
        {
            return new Vec2(a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y);
        }

        public static void minToOut(Vec2 a, Vec2 b, Vec2 out_Renamed)
        {
            out_Renamed.x = a.x < b.x ? a.x : b.x;
            out_Renamed.y = a.y < b.y ? a.y : b.y;
        }

        public static void maxToOut(Vec2 a, Vec2 b, Vec2 out_Renamed)
        {
            out_Renamed.x = a.x > b.x ? a.x : b.x;
            out_Renamed.y = a.y > b.y ? a.y : b.y;
        }

        private static int FloatToIntBits(float number)
        {
            return BitConverter.ToInt32(BitConverter.GetBytes(number), 0);
        }

        public override int GetHashCode()
        {
            // automatically generated by Eclipse
            int prime = 31;
            int result = 1;
            result = prime * result + FloatToIntBits(x);
            result = prime * result + FloatToIntBits(y);
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
            if (FloatToIntBits(x) != FloatToIntBits(other.x))
                return false;
            if (FloatToIntBits(y) != FloatToIntBits(other.y))
                return false;
            return true;
        }

        /// <summary>True if the vector represents a pair of valid, non-infinite floating point numbers. </summary>
        virtual public bool Valid
        {
            get
            {
                return !Single.IsNaN(x) && !Single.IsInfinity(x) && !Single.IsNaN(y) && !Single.IsInfinity(y);
            }
        }
    }
}