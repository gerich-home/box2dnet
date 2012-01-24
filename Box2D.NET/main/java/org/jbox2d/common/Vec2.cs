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
	
	/// <summary> A 2D column vector</summary>
	[Serializable]
	public class Vec2 : System.ICloneable
	{
		/// <summary>True if the vector represents a pair of valid, non-infinite floating point numbers. </summary>
		virtual public bool Valid
		{
			get
			{
				return !System.Single.IsNaN(x) && !System.Single.IsInfinity(x) && !System.Single.IsNaN(y) && !System.Single.IsInfinity(y);
			}
			
		}
		private const long serialVersionUID = 1L;
		
		public float x, y;
		
		public Vec2():this(0, 0)
		{
		}
		
		public Vec2(float x, float y)
		{
			this.x = x;
			this.y = y;
		}
		
		public Vec2(Vec2 toCopy):this(toCopy.x, toCopy.y)
		{
		}
		
		/// <summary>Zero out this vector. </summary>
		public void  setZero()
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
			return new Vec2(- x, - y);
		}
		
		/// <summary>Flip the vector and return it - alters this vector. </summary>
		public Vec2 negateLocal()
		{
			x = - x;
			y = - y;
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
			return new Vec2(- y, x);
		}
		
		/// <summary>Get the skew vector such that dot(skew_vec, other) == cross(vec, other) </summary>
		public void  skew(Vec2 out_Renamed)
		{
			out_Renamed.x = - y;
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
			float length = length();
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
		
		public void  absLocal()
		{
			x = MathUtils.abs(x);
			y = MathUtils.abs(y);
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		/// <summary>Return a copy of this vector. </summary>
		//UPGRADE_ISSUE: The equivalent in .NET for method 'java.lang.Object.clone' returns a different type. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1224'"
		public virtual System.Object Clone()
		{
			return new Vec2(x, y);
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public override System.String ToString()
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
		
		public static void  absToOut(Vec2 a, Vec2 out_Renamed)
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
			return new Vec2(s * a.y, (- s) * a.x);
		}
		
		public static void  crossToOut(Vec2 a, float s, Vec2 out_Renamed)
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'tempy '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float tempy = (- s) * a.x;
			out_Renamed.x = s * a.y;
			out_Renamed.y = tempy;
		}
		
		public static void  crossToOutUnsafe(Vec2 a, float s, Vec2 out_Renamed)
		{
			assert(out_Renamed != a);
			out_Renamed.x = s * a.y;
			out_Renamed.y = (- s) * a.x;
		}
		
		public static Vec2 cross(float s, Vec2 a)
		{
			return new Vec2((- s) * a.y, s * a.x);
		}
		
		public static void  crossToOut(float s, Vec2 a, Vec2 out_Renamed)
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'tempY '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float tempY = s * a.x;
			out_Renamed.x = (- s) * a.y;
			out_Renamed.y = tempY;
		}
		
		public static void  crossToOutUnsafe(float s, Vec2 a, Vec2 out_Renamed)
		{
			assert(out_Renamed != a);
			out_Renamed.x = (- s) * a.y;
			out_Renamed.y = s * a.x;
		}
		
		public static void  negateToOut(Vec2 a, Vec2 out_Renamed)
		{
			out_Renamed.x = - a.x;
			out_Renamed.y = - a.y;
		}
		
		public static Vec2 min(Vec2 a, Vec2 b)
		{
			return new Vec2(a.x < b.x?a.x:b.x, a.y < b.y?a.y:b.y);
		}
		
		public static Vec2 max(Vec2 a, Vec2 b)
		{
			return new Vec2(a.x > b.x?a.x:b.x, a.y > b.y?a.y:b.y);
		}
		
		public static void  minToOut(Vec2 a, Vec2 b, Vec2 out_Renamed)
		{
			out_Renamed.x = a.x < b.x?a.x:b.x;
			out_Renamed.y = a.y < b.y?a.y:b.y;
		}
		
		public static void  maxToOut(Vec2 a, Vec2 b, Vec2 out_Renamed)
		{
			out_Renamed.x = a.x > b.x?a.x:b.x;
			out_Renamed.y = a.y > b.y?a.y:b.y;
		}
		
		/// <seealso cref="java.lang.Object.hashCode()">
		/// </seealso>
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public override int GetHashCode()
		{
			// automatically generated by Eclipse
			//UPGRADE_NOTE: Final was removed from the declaration of 'prime '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			int prime = 31;
			int result = 1;
			//UPGRADE_ISSUE: Method 'java.lang.Float.floatToIntBits' was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1000_javalangFloatfloatToIntBits_float'"
			result = prime * result + Float.floatToIntBits(x);
			//UPGRADE_ISSUE: Method 'java.lang.Float.floatToIntBits' was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1000_javalangFloatfloatToIntBits_float'"
			result = prime * result + Float.floatToIntBits(y);
			return result;
		}
		
		/// <seealso cref="java.lang.Object.equals(java.lang.Object)">
		/// </seealso>
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public  override bool Equals(System.Object obj)
		{
			// automatically generated by Eclipse
			if (this == obj)
				return true;
			if (obj == null)
				return false;
			if (GetType() != obj.GetType())
				return false;
			Vec2 other = (Vec2) obj;
			//UPGRADE_ISSUE: Method 'java.lang.Float.floatToIntBits' was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1000_javalangFloatfloatToIntBits_float'"
			if (Float.floatToIntBits(x) != Float.floatToIntBits(other.x))
				return false;
			//UPGRADE_ISSUE: Method 'java.lang.Float.floatToIntBits' was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1000_javalangFloatfloatToIntBits_float'"
			if (Float.floatToIntBits(y) != Float.floatToIntBits(other.y))
				return false;
			return true;
		}
	}
}