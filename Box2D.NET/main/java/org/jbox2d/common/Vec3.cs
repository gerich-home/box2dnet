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
	
	/// <author>  Daniel Murphy
	/// </author>
	[Serializable]
	public class Vec3 : System.ICloneable
	{
		private const long serialVersionUID = 1L;
		
		public float x, y, z;
		
		public Vec3()
		{
			x = y = z = 0f;
		}
		
		public Vec3(float argX, float argY, float argZ)
		{
			x = argX;
			y = argY;
			z = argZ;
		}
		
		public Vec3(Vec3 argCopy)
		{
			x = argCopy.x;
			y = argCopy.y;
			z = argCopy.z;
		}
		
		public virtual Vec3 set_Renamed(Vec3 argVec)
		{
			x = argVec.x;
			y = argVec.y;
			z = argVec.z;
			return this;
		}
		
		public virtual Vec3 set_Renamed(float argX, float argY, float argZ)
		{
			x = argX;
			y = argY;
			z = argZ;
			return this;
		}
		
		public virtual Vec3 addLocal(Vec3 argVec)
		{
			x += argVec.x;
			y += argVec.y;
			z += argVec.z;
			return this;
		}
		
		public virtual Vec3 add(Vec3 argVec)
		{
			return new Vec3(x + argVec.x, y + argVec.y, z + argVec.z);
		}
		
		public virtual Vec3 subLocal(Vec3 argVec)
		{
			x -= argVec.x;
			y -= argVec.y;
			z -= argVec.z;
			return this;
		}
		
		public virtual Vec3 sub(Vec3 argVec)
		{
			return new Vec3(x - argVec.x, y - argVec.y, z - argVec.z);
		}
		
		public virtual Vec3 mulLocal(float argScalar)
		{
			x *= argScalar;
			y *= argScalar;
			z *= argScalar;
			return this;
		}
		
		public virtual Vec3 mul(float argScalar)
		{
			return new Vec3(x * argScalar, y * argScalar, z * argScalar);
		}
		
		public virtual Vec3 negate()
		{
			return new Vec3(- x, - y, - z);
		}
		
		public virtual Vec3 negateLocal()
		{
			x = - x;
			y = - y;
			z = - z;
			return this;
		}
		
		public virtual void  setZero()
		{
			x = 0;
			y = 0;
			z = 0;
		}
		
		//UPGRADE_ISSUE: The equivalent in .NET for method 'java.lang.Object.clone' returns a different type. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1224'"
		public virtual System.Object Clone()
		{
			return new Vec3(this);
		}
		
		public override System.String ToString()
		{
			return "(" + x + "," + y + "," + z + ")";
		}
		
		/// <seealso cref="java.lang.Object.hashCode()">
		/// </seealso>
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public override int GetHashCode()
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'prime '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			int prime = 31;
			int result = 1;
			//UPGRADE_ISSUE: Method 'java.lang.Float.floatToIntBits' was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1000_javalangFloatfloatToIntBits_float'"
			result = prime * result + Float.floatToIntBits(x);
			//UPGRADE_ISSUE: Method 'java.lang.Float.floatToIntBits' was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1000_javalangFloatfloatToIntBits_float'"
			result = prime * result + Float.floatToIntBits(y);
			//UPGRADE_ISSUE: Method 'java.lang.Float.floatToIntBits' was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1000_javalangFloatfloatToIntBits_float'"
			result = prime * result + Float.floatToIntBits(z);
			return result;
		}
		
		/// <seealso cref="java.lang.Object.equals(java.lang.Object)">
		/// </seealso>
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public  override bool Equals(System.Object obj)
		{
			if (this == obj)
				return true;
			if (obj == null)
				return false;
			if (GetType() != obj.GetType())
				return false;
			Vec3 other = (Vec3) obj;
			//UPGRADE_ISSUE: Method 'java.lang.Float.floatToIntBits' was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1000_javalangFloatfloatToIntBits_float'"
			if (Float.floatToIntBits(x) != Float.floatToIntBits(other.x))
				return false;
			//UPGRADE_ISSUE: Method 'java.lang.Float.floatToIntBits' was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1000_javalangFloatfloatToIntBits_float'"
			if (Float.floatToIntBits(y) != Float.floatToIntBits(other.y))
				return false;
			//UPGRADE_ISSUE: Method 'java.lang.Float.floatToIntBits' was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1000_javalangFloatfloatToIntBits_float'"
			if (Float.floatToIntBits(z) != Float.floatToIntBits(other.z))
				return false;
			return true;
		}
		
		public static float dot(Vec3 a, Vec3 b)
		{
			return a.x * b.x + a.y * b.y + a.z * b.z;
		}
		
		public static Vec3 cross(Vec3 a, Vec3 b)
		{
			return new Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
		}
		
		public static void  crossToOut(Vec3 a, Vec3 b, Vec3 out_Renamed)
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'tempy '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float tempy = a.z * b.x - a.x * b.z;
			//UPGRADE_NOTE: Final was removed from the declaration of 'tempz '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float tempz = a.x * b.y - a.y * b.x;
			out_Renamed.x = a.y * b.z - a.z * b.y;
			out_Renamed.y = tempy;
			out_Renamed.z = tempz;
		}
	}
}