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

    /// <author>Daniel Murphy</author>
    [Serializable]
    public class Vec3
    {
        public float X;
        public float Y;
        public float Z;

        public Vec3()
        {
            X = Y = Z = 0f;
        }

        public Vec3(float argX, float argY, float argZ)
        {
            X = argX;
            Y = argY;
            Z = argZ;
        }

        public Vec3(Vec3 argCopy)
        {
            X = argCopy.X;
            Y = argCopy.Y;
            Z = argCopy.Z;
        }

        public Vec3 Set(Vec3 argVec)
        {
            X = argVec.X;
            Y = argVec.Y;
            Z = argVec.Z;
            return this;
        }

        public Vec3 Set(float argX, float argY, float argZ)
        {
            X = argX;
            Y = argY;
            Z = argZ;
            return this;
        }

        public Vec3 AddLocal(Vec3 argVec)
        {
            X += argVec.X;
            Y += argVec.Y;
            Z += argVec.Z;
            return this;
        }

        public Vec3 Add(Vec3 argVec)
        {
            return new Vec3(X + argVec.X, Y + argVec.Y, Z + argVec.Z);
        }

        public Vec3 SubLocal(Vec3 argVec)
        {
            X -= argVec.X;
            Y -= argVec.Y;
            Z -= argVec.Z;
            return this;
        }

        public Vec3 Sub(Vec3 argVec)
        {
            return new Vec3(X - argVec.X, Y - argVec.Y, Z - argVec.Z);
        }

        public Vec3 MulLocal(float argScalar)
        {
            X *= argScalar;
            Y *= argScalar;
            Z *= argScalar;
            return this;
        }

        public Vec3 Mul(float argScalar)
        {
            return new Vec3(X * argScalar, Y * argScalar, Z * argScalar);
        }

        public Vec3 Negate()
        {
            return new Vec3(-X, -Y, -Z);
        }

        public Vec3 NegateLocal()
        {
            X = -X;
            Y = -Y;
            Z = -Z;
            return this;
        }

        public void SetZero()
        {
            X = 0;
            Y = 0;
            Z = 0;
        }

        public Vec3 Clone()
        {
            return new Vec3(this);
        }

        public override String ToString()
        {
            return string.Format("({0},{1},{2})", X, Y, Z);
        }

        private static int FloatToIntBits(float number)
        {
            return BitConverter.ToInt32(BitConverter.GetBytes(number), 0);
        }

        public override int GetHashCode()
        {
            const int prime = 31;
            int result = 1;
            result = prime * result + FloatToIntBits(X);
            result = prime * result + FloatToIntBits(Y);
            result = prime * result + FloatToIntBits(Z);
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
            Vec3 other = (Vec3)obj;
            if (FloatToIntBits(X) != FloatToIntBits(other.X))
                return false;
            if (FloatToIntBits(Y) != FloatToIntBits(other.Y))
                return false;
            if (FloatToIntBits(Z) != FloatToIntBits(other.Z))
                return false;
            return true;
        }

        public static float Dot(Vec3 a, Vec3 b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        public static Vec3 Cross(Vec3 a, Vec3 b)
        {
            return new Vec3(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
        }

        public static void CrossToOut(Vec3 a, Vec3 b, Vec3 result)
        {
            float tempy = a.Z * b.X - a.X * b.Z;
            float tempz = a.X * b.Y - a.Y * b.X;
            result.X = a.Y * b.Z - a.Z * b.Y;
            result.Y = tempy;
            result.Z = tempz;
        }

        public static void CrossToOutUnsafe(Vec3 a, Vec3 b, Vec3 result)
        {
            Debug.Assert(result != b);
            Debug.Assert(result != a);
            result.X = a.Y * b.Z - a.Z * b.Y;
            result.Y = a.Z * b.X - a.X * b.Z;
            result.Z = a.X * b.Y - a.Y * b.X;
        }
    }
}