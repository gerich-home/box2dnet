/// <summary>****************************************************************************
/// Copyright (c) 2011, Daniel Murphy
/// All rights reserved.
/// 
/// Redistribution and use in source and binary forms, with or without modification,
/// are permitted provided that the following conditions are met:
/// * Redistributions of source code must retain the above copyright notice,
/// this list of conditions and the following disclaimer.
/// * Redistributions in binary form must reproduce the above copyright notice,
/// this list of conditions and the following disclaimer in the documentation
/// and/or other materials provided with the distribution.
/// 
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
/// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
/// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
/// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
/// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
/// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
/// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
/// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
/// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
/// ****************************************************************************
/// </summary>
using System;
namespace org.jbox2d.common
{
	
	// updated to rev 100
	
	/// <summary> A transform contains translation and rotation. It is used to represent the position and
	/// orientation of rigid frames.
	/// </summary>
	[Serializable]
	public class Transform
	{
		private const long serialVersionUID = 1L;
		
		/// <summary>The translation caused by the transform </summary>
		//UPGRADE_NOTE: Final was removed from the declaration of 'p '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Vec2 p;
		
		/// <summary>A matrix representing a rotation </summary>
		//UPGRADE_NOTE: Final was removed from the declaration of 'q '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Rot q;
		
		/// <summary>The default constructor. </summary>
		public Transform()
		{
			p = new Vec2();
			q = new Rot();
		}
		
		/// <summary>Initialize as a copy of another transform. </summary>
		public Transform(Transform xf)
		{
			p = xf.p.Clone();
			q = xf.q.Clone();
		}
		
		/// <summary>Initialize using a position vector and a rotation matrix. </summary>
		public Transform(Vec2 _position, Rot _R)
		{
			p = _position.Clone();
			q = _R.Clone();
		}
		
		/// <summary>Set this to equal another transform. </summary>
		public Transform set_Renamed(Transform xf)
		{
			p.set_Renamed(xf.p);
			q.set_Renamed(xf.q);
			return this;
		}
		
		/// <summary> Set this based on the position and angle.
		/// 
		/// </summary>
		/// <param name="p">
		/// </param>
		/// <param name="angle">
		/// </param>
		public void  set_Renamed(Vec2 p, float angle)
		{
			this.p.set_Renamed(p);
			q.set_Renamed(angle);
		}
		
		/// <summary>Set this to the identity transform. </summary>
		public void  setIdentity()
		{
			p.setZero();
			q.setIdentity();
		}
		
		public static Vec2 mul(Transform T, Vec2 v)
		{
			return new Vec2((T.q.c * v.x - T.q.s * v.y) + T.p.x, (T.q.s * v.x + T.q.c * v.y) + T.p.y);
		}
		
		public static void  mulToOut(Transform T, Vec2 v, Vec2 out_Renamed)
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'tempy '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float tempy = (T.q.s * v.x + T.q.c * v.y) + T.p.y;
			out_Renamed.x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
			out_Renamed.y = tempy;
		}
		
		public static void  mulToOutUnsafe(Transform T, Vec2 v, Vec2 out_Renamed)
		{
			assert(v != out_Renamed);
			out_Renamed.x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
			out_Renamed.y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;
		}
		
		public static Vec2 mulTrans(Transform T, Vec2 v)
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'px '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float px = v.x - T.p.x;
			//UPGRADE_NOTE: Final was removed from the declaration of 'py '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float py = v.y - T.p.y;
			return new Vec2((T.q.c * px + T.q.s * py), ((- T.q.s) * px + T.q.c * py));
		}
		
		public static void  mulTransToOut(Transform T, Vec2 v, Vec2 out_Renamed)
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'px '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float px = v.x - T.p.x;
			//UPGRADE_NOTE: Final was removed from the declaration of 'py '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float py = v.y - T.p.y;
			//UPGRADE_NOTE: Final was removed from the declaration of 'tempy '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float tempy = ((- T.q.s) * px + T.q.c * py);
			out_Renamed.x = (T.q.c * px + T.q.s * py);
			out_Renamed.y = tempy;
		}
		
		public static Transform mul(Transform A, Transform B)
		{
			Transform C = new Transform();
			Rot.mulUnsafe(A.q, B.q, C.q);
			Rot.mulToOutUnsafe(A.q, B.p, C.p);
			C.p.addLocal(A.p);
			return C;
		}
		
		public static void  mulToOut(Transform A, Transform B, Transform out_Renamed)
		{
			assert(out_Renamed != A);
			Rot.mul(A.q, B.q, out_Renamed.q);
			Rot.mulToOut(A.q, B.p, out_Renamed.p);
			out_Renamed.p.addLocal(A.p);
		}
		
		public static void  mulToOutUnsafe(Transform A, Transform B, Transform out_Renamed)
		{
			assert(out_Renamed != B);
			assert(out_Renamed != A);
			Rot.mulUnsafe(A.q, B.q, out_Renamed.q);
			Rot.mulToOutUnsafe(A.q, B.p, out_Renamed.p);
			out_Renamed.p.addLocal(A.p);
		}
		
		private static Vec2 pool = new Vec2();
		
		public static Transform mulTrans(Transform A, Transform B)
		{
			Transform C = new Transform();
			Rot.mulTransUnsafe(A.q, B.q, C.q);
			pool.set_Renamed(B.p).subLocal(A.p);
			Rot.mulTransUnsafe(A.q, pool, C.p);
			return C;
		}
		
		public static void  mulTransToOut(Transform A, Transform B, Transform out_Renamed)
		{
			assert(out_Renamed != A);
			Rot.mulTrans(A.q, B.q, out_Renamed.q);
			pool.set_Renamed(B.p).subLocal(A.p);
			Rot.mulTrans(A.q, pool, out_Renamed.p);
		}
		
		public static void  mulTransToOutUnsafe(Transform A, Transform B, Transform out_Renamed)
		{
			assert(out_Renamed != A);
			assert(out_Renamed != B);
			Rot.mulTransUnsafe(A.q, B.q, out_Renamed.q);
			pool.set_Renamed(B.p).subLocal(A.p);
			Rot.mulTransUnsafe(A.q, pool, out_Renamed.p);
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public override System.String ToString()
		{
			System.String s = "XForm:\n";
			s += ("Position: " + p + "\n");
			//UPGRADE_TODO: The equivalent in .NET for method 'java.lang.Object.toString' may return a different value. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1043'"
			s += ("R: \n" + q + "\n");
			return s;
		}
	}
}