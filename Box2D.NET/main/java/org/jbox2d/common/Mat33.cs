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
	
	/// <summary> A 3-by-3 matrix. Stored in column-major order.
	/// 
	/// </summary>
	/// <author>  Daniel Murphy
	/// </author>
	[Serializable]
	public class Mat33
	{
		private const long serialVersionUID = 2L;
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'IDENTITY '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public static readonly Mat33 IDENTITY = new Mat33(new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(0, 0, 1));
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'ex '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		//UPGRADE_NOTE: Final was removed from the declaration of 'ey '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		//UPGRADE_NOTE: Final was removed from the declaration of 'ez '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		public Vec3 ex;
		public Vec3 ey;
		public Vec3 ez;
		
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
		
		public virtual void  setZero()
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
			return new Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
		}
		
		public static void  mul22ToOut(Mat33 A, Vec2 v, Vec2 out_Renamed)
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'tempx '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float tempx = A.ex.x * v.x + A.ey.x * v.y;
			out_Renamed.y = A.ex.y * v.x + A.ey.y * v.y;
			out_Renamed.x = tempx;
		}
		
		public static void  mul22ToOutUnsafe(Mat33 A, Vec2 v, Vec2 out_Renamed)
		{
			assert(v != out_Renamed);
			out_Renamed.y = A.ex.y * v.x + A.ey.y * v.y;
			out_Renamed.x = A.ex.x * v.x + A.ey.x * v.y;
		}
		
		public static void  mulToOut(Mat33 A, Vec3 v, Vec3 out_Renamed)
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'tempy '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float tempy = v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y;
			//UPGRADE_NOTE: Final was removed from the declaration of 'tempz '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float tempz = v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z;
			out_Renamed.x = v.x * A.ex.x + v.y * A.ey.x + v.z + A.ez.x;
			out_Renamed.y = tempy;
			out_Renamed.z = tempz;
		}
		
		public static void  mulToOutUnsafe(Mat33 A, Vec3 v, Vec3 out_Renamed)
		{
			assert(out_Renamed != v);
			out_Renamed.x = v.x * A.ex.x + v.y * A.ey.x + v.z + A.ez.x;
			out_Renamed.y = v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y;
			out_Renamed.z = v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z;
		}
		
		/// <summary> Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
		/// in one-shot cases.
		/// 
		/// </summary>
		/// <param name="b">
		/// </param>
		/// <returns>
		/// </returns>
		public Vec2 solve22(Vec2 b)
		{
			Vec2 x = new Vec2();
			solve22ToOut(b, x);
			return x;
		}
		
		/// <summary> Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
		/// in one-shot cases.
		/// 
		/// </summary>
		/// <param name="b">
		/// </param>
		/// <returns>
		/// </returns>
		public void  solve22ToOut(Vec2 b, Vec2 out_Renamed)
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'a11 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			//UPGRADE_NOTE: Final was removed from the declaration of 'a12 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			//UPGRADE_NOTE: Final was removed from the declaration of 'a21 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			//UPGRADE_NOTE: Final was removed from the declaration of 'a22 '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float a11 = ex.x;
			float a12 = ey.x;
			float a21 = ex.y;
			float a22 = ey.y;
			float det = a11 * a22 - a12 * a21;
			if (det != 0.0f)
			{
				det = 1.0f / det;
			}
			out_Renamed.x = det * (a22 * b.x - a12 * b.y);
			out_Renamed.y = det * (a11 * b.y - a21 * b.x);
		}
		
		// djm pooling from below
		/// <summary> Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
		/// in one-shot cases.
		/// 
		/// </summary>
		/// <param name="b">
		/// </param>
		/// <returns>
		/// </returns>
		public Vec3 solve33(Vec3 b)
		{
			Vec3 x = new Vec3();
			solve33ToOut(b, x);
			return x;
		}
		
		/// <summary> Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
		/// in one-shot cases.
		/// 
		/// </summary>
		/// <param name="b">
		/// </param>
		/// <param name="out">the result
		/// </param>
		public void  solve33ToOut(Vec3 b, Vec3 out_Renamed)
		{
			assert(b != out_Renamed);
			Vec3.crossToOut(ey, ez, out_Renamed);
			float det = Vec3.dot(ex, out_Renamed);
			if (det != 0.0f)
			{
				det = 1.0f / det;
			}
			Vec3.crossToOut(ey, ez, out_Renamed);
			//UPGRADE_NOTE: Final was removed from the declaration of 'x '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float x = det * Vec3.dot(b, out_Renamed);
			Vec3.crossToOut(b, ez, out_Renamed);
			//UPGRADE_NOTE: Final was removed from the declaration of 'y '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float y = det * Vec3.dot(ex, out_Renamed);
			Vec3.crossToOut(ey, b, out_Renamed);
			float z = det * Vec3.dot(ex, out_Renamed);
			out_Renamed.x = x;
			out_Renamed.y = y;
			out_Renamed.z = z;
		}
		
		public virtual void  getInverse22(Mat33 M)
		{
			float a = ex.x, b = ey.x, c = ey.y, d = ey.y;
			float det = a * d - b * c;
			if (det != 0.0f)
			{
				det = 1.0f / det;
			}
			
			M.ex.x = det * d;
			M.ey.x = (- det) * b;
			M.ex.z = 0.0f;
			M.ex.y = (- det) * c;
			M.ey.y = det * a;
			M.ey.z = 0.0f;
			M.ey.x = 0.0f;
			M.ey.y = 0.0f;
			M.ey.z = 0.0f;
		}
		
		// / Returns the zero matrix if singular.
		public virtual void  getSymInverse33(Mat33 M)
		{
			float det = ex.x * ey.y * ey.z - ey.z * ey.y + ex.y * ey.z * ey.x - ey.x * ey.z + ex.z * ey.x * ey.y - ey.y * ey.x;
			if (det != 0.0f)
			{
				det = 1.0f / det;
			}
			
			float a11 = ex.x, a12 = ey.x, a13 = ez.x;
			float a22 = ey.y, a23 = ey.y;
			float a33 = ey.z;
			
			M.ex.x = det * (a22 * a33 - a23 * a23);
			M.ex.y = det * (a13 * a23 - a12 * a33);
			M.ex.z = det * (a12 * a23 - a13 * a22);
			
			M.ey.x = M.ex.y;
			M.ey.y = det * (a11 * a33 - a13 * a13);
			M.ey.z = det * (a13 * a12 - a11 * a23);
			
			M.ey.x = M.ex.z;
			M.ey.y = M.ey.z;
			M.ey.z = det * (a11 * a22 - a12 * a12);
		}
	}
}