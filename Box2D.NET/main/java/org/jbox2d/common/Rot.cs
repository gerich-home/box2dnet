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
	
	/// <summary> Represents a rotation
	/// 
	/// </summary>
	/// <author>  Daniel
	/// </author>
	[Serializable]
	public class Rot : System.ICloneable
	{
		virtual public float Sin
		{
			get
			{
				return s;
			}
			
		}
		virtual public float Cos
		{
			get
			{
				return c;
			}
			
		}
		virtual public float Angle
		{
			get
			{
				return MathUtils.atan2(s, c);
			}
			
		}
		private const long serialVersionUID = 1L;
		
		public float s, c; // sin and cos
		
		public Rot()
		{
			setIdentity();
		}
		
		public Rot(float angle)
		{
			set_Renamed(angle);
		}
		
		public virtual Rot set_Renamed(float angle)
		{
			s = MathUtils.sin(angle);
			c = MathUtils.cos(angle);
			return this;
		}
		
		public virtual Rot set_Renamed(Rot other)
		{
			s = other.s;
			c = other.c;
			return this;
		}
		
		public virtual Rot setIdentity()
		{
			s = 0;
			c = 1;
			return this;
		}
		
		public virtual void  getXAxis(Vec2 xAxis)
		{
			xAxis.set_Renamed(c, s);
		}
		
		public virtual void  getYAxis(Vec2 yAxis)
		{
			yAxis.set_Renamed(- s, c);
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		//UPGRADE_ISSUE: The equivalent in .NET for method 'java.lang.Object.clone' returns a different type. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1224'"
		public virtual System.Object Clone()
		{
			Rot copy = new Rot();
			copy.s = s;
			copy.c = c;
			return copy;
		}
		
		public static void  mul(Rot q, Rot r, Rot out_Renamed)
		{
			float tempc = q.c * r.c - q.s * r.s;
			out_Renamed.s = q.s * r.c + q.c * r.s;
			out_Renamed.c = tempc;
		}
		
		public static void  mulUnsafe(Rot q, Rot r, Rot out_Renamed)
		{
			assert(r != out_Renamed);
			assert(q != out_Renamed);
			// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
			// [qs qc] [rs rc] [qs*rc+qc*rs -qs*rs+qc*rc]
			// s = qs * rc + qc * rs
			// c = qc * rc - qs * rs
			out_Renamed.s = q.s * r.c + q.c * r.s;
			out_Renamed.c = q.c * r.c - q.s * r.s;
		}
		
		public static void  mulTrans(Rot q, Rot r, Rot out_Renamed)
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'tempc '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float tempc = q.c * r.c + q.s * r.s;
			out_Renamed.s = q.c * r.s - q.s * r.c;
			out_Renamed.c = tempc;
		}
		
		public static void  mulTransUnsafe(Rot q, Rot r, Rot out_Renamed)
		{
			// [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
			// [-qs qc] [rs rc] [-qs*rc+qc*rs qs*rs+qc*rc]
			// s = qc * rs - qs * rc
			// c = qc * rc + qs * rs
			out_Renamed.s = q.c * r.s - q.s * r.c;
			out_Renamed.c = q.c * r.c + q.s * r.s;
		}
		
		public static void  mulToOut(Rot q, Vec2 v, Vec2 out_Renamed)
		{
			float tempy = q.s * v.x + q.c * v.y;
			out_Renamed.x = q.c * v.x - q.s * v.y;
			out_Renamed.y = tempy;
		}
		
		public static void  mulToOutUnsafe(Rot q, Vec2 v, Vec2 out_Renamed)
		{
			out_Renamed.x = q.c * v.x - q.s * v.y;
			out_Renamed.y = q.s * v.x + q.c * v.y;
		}
		
		public static void  mulTrans(Rot q, Vec2 v, Vec2 out_Renamed)
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'tempy '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			float tempy = (- q.s) * v.x + q.c * v.y;
			out_Renamed.x = q.c * v.x + q.s * v.y;
			out_Renamed.y = tempy;
		}
		
		public static void  mulTransUnsafe(Rot q, Vec2 v, Vec2 out_Renamed)
		{
			out_Renamed.x = q.c * v.x + q.s * v.y;
			out_Renamed.y = (- q.s) * v.x + q.c * v.y;
		}
	}
}