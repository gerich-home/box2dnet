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
using Vec2 = org.jbox2d.common.Vec2;
namespace org.jbox2d.pooling.arrays
{
	
	/// <summary> not thread safe Vec2[] pool</summary>
	/// <author>  dmurph
	/// 
	/// </author>
	public class Vec2Array
	{
		
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		private final HashMap < Integer, Vec2 [] > map = new HashMap < Integer, Vec2 [] >();
		
		public virtual Vec2[] get_Renamed(int argLength)
		{
			assert(argLength > 0);
			
			if (!map.containsKey(argLength))
			{
				map.put(argLength, getInitializedArray(argLength));
			}
			
			//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
			assert(map.get(argLength).length == argLength): Array not built of correct length;
			return map.get_Renamed(argLength);
		}
		
		protected internal virtual Vec2[] getInitializedArray(int argLength)
		{
			//UPGRADE_NOTE: Final was removed from the declaration of 'ray '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
			Vec2[] ray = new Vec2[argLength];
			for (int i = 0; i < ray.Length; i++)
			{
				ray[i] = new Vec2();
			}
			return ray;
		}
	}
}