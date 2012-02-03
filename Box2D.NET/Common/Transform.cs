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

    // updated to rev 100

    /// <summary>
    /// A transform contains translation and rotation. It is used to represent the position and
    /// orientation of rigid frames.
    /// </summary>
    [Serializable]
    public class Transform
    {
        /// <summary>
        /// The translation caused by the transform
        /// </summary>
        public readonly Vec2 P;

        /// <summary>
        /// A matrix representing a rotation 
        /// </summary>
        public readonly Rot Q;

        /// <summary>
        /// The default constructor.
        /// </summary>
        public Transform()
        {
            P = new Vec2();
            Q = new Rot();
        }

        /// <summary>
        /// Initialize as a copy of another transform.
        /// </summary>
        public Transform(Transform xf)
        {
            P = xf.P.Clone();
            Q = xf.Q.Clone();
        }

        /// <summary>
        /// Initialize using a position vector and a rotation matrix.
        /// </summary>
        public Transform(Vec2 position, Rot r)
        {
            P = position.Clone();
            Q = r.Clone();
        }

        /// <summary>
        /// Set this to equal another transform.
        /// </summary>
        public Transform Set(Transform xf)
        {
            P.Set(xf.P);
            Q.Set(xf.Q);
            return this;
        }

        /// <summary>
        /// Set this based on the position and angle.
        /// </summary>
        /// <param name="p"></param>
        /// <param name="angle"></param>
        public void Set(Vec2 p, float angle)
        {
            this.P.Set(p);
            Q.Set(angle);
        }

        /// <summary>
        /// Set this to the identity transform.
        /// </summary>
        public void SetIdentity()
        {
            P.SetZero();
            Q.SetIdentity();
        }

        public static Vec2 Mul(Transform T, Vec2 v)
        {
            return new Vec2((T.Q.Cos * v.X - T.Q.Sin * v.Y) + T.P.X, (T.Q.Sin * v.X + T.Q.Cos * v.Y) + T.P.Y);
        }

        public static void MulToOut(Transform T, Vec2 v, Vec2 result)
        {
            float tempy = (T.Q.Sin * v.X + T.Q.Cos * v.Y) + T.P.Y;
            result.X = (T.Q.Cos * v.X - T.Q.Sin * v.Y) + T.P.X;
            result.Y = tempy;
        }

        public static void MulToOutUnsafe(Transform T, Vec2 v, Vec2 result)
        {
            Debug.Assert(v != result);
            result.X = (T.Q.Cos * v.X - T.Q.Sin * v.Y) + T.P.X;
            result.Y = (T.Q.Sin * v.X + T.Q.Cos * v.Y) + T.P.Y;
        }

        public static Vec2 MulTrans(Transform T, Vec2 v)
        {
            float px = v.X - T.P.X;
            float py = v.Y - T.P.Y;
            return new Vec2((T.Q.Cos * px + T.Q.Sin * py), ((-T.Q.Sin) * px + T.Q.Cos * py));
        }

        public static void MulTransToOut(Transform T, Vec2 v, Vec2 result)
        {
            float px = v.X - T.P.X;
            float py = v.Y - T.P.Y;
            float tempy = ((-T.Q.Sin) * px + T.Q.Cos * py);
            result.X = (T.Q.Cos * px + T.Q.Sin * py);
            result.Y = tempy;
        }

        public static void MulTransToOutUnsafe(Transform T, Vec2 v, Vec2 result)
        {
            Debug.Assert(v != result);
            float px = v.X - T.P.X;
            float py = v.Y - T.P.Y;
            result.X = (T.Q.Cos * px + T.Q.Sin * py);
            result.Y = (-T.Q.Sin * px + T.Q.Cos * py);
        }

        public static Transform Mul(Transform A, Transform B)
        {
            Transform C = new Transform();
            Rot.MulUnsafe(A.Q, B.Q, C.Q);
            Rot.MulToOutUnsafe(A.Q, B.P, C.P);
            C.P.AddLocal(A.P);
            return C;
        }

        public static void MulToOut(Transform A, Transform B, Transform result)
        {
            Debug.Assert(result != A);
            Rot.Mul(A.Q, B.Q, result.Q);
            Rot.MulToOut(A.Q, B.P, result.P);
            result.P.AddLocal(A.P);
        }

        public static void MulToOutUnsafe(Transform A, Transform B, Transform result)
        {
            Debug.Assert(result != B);
            Debug.Assert(result != A);
            Rot.MulUnsafe(A.Q, B.Q, result.Q);
            Rot.MulToOutUnsafe(A.Q, B.P, result.P);
            result.P.AddLocal(A.P);
        }

        private static readonly Vec2 pool = new Vec2();

        public static Transform MulTrans(Transform A, Transform B)
        {
            Transform C = new Transform();
            Rot.MulTransUnsafe(A.Q, B.Q, C.Q);
            pool.Set(B.P).SubLocal(A.P);
            Rot.MulTransUnsafe(A.Q, pool, C.P);
            return C;
        }

        public static void MulTransToOut(Transform A, Transform B, Transform result)
        {
            Debug.Assert(result != A);
            Rot.MulTrans(A.Q, B.Q, result.Q);
            pool.Set(B.P).SubLocal(A.P);
            Rot.MulTrans(A.Q, pool, result.P);
        }

        public static void MulTransToOutUnsafe(Transform A, Transform B, Transform result)
        {
            Debug.Assert(result != A);
            Debug.Assert(result != B);
            Rot.MulTransUnsafe(A.Q, B.Q, result.Q);
            pool.Set(B.P).SubLocal(A.P);
            Rot.MulTransUnsafe(A.Q, pool, result.P);
        }

        public override String ToString()
        {
            return string.Format("XForm:\n" + "Position: {0}\n" + "R: \n{1}\n", P, Q);
        }
    }
}