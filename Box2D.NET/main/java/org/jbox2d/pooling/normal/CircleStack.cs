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
//UPGRADE_TODO: The type 'org.jbox2d.pooling.IOrderedStack' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
using IOrderedStack = org.jbox2d.pooling.IOrderedStack;
//UPGRADE_TODO: The type 'org.slf4j.Logger' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
using Logger = org.slf4j.Logger;
//UPGRADE_TODO: The type 'org.slf4j.LoggerFactory' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
using LoggerFactory = org.slf4j.LoggerFactory;
namespace org.jbox2d.pooling.normal
{
	
	public class CircleStack
	{
		public CircleStack()
		{
			InitBlock();
		}
		private void  InitBlock()
		{
			size = argStackSize;
			pool = (E[]) Array.newInstance(argClass, argStackSize);
			for (int i = 0; i < argStackSize; i++)
			{
				try
				{
					pool[i] = argClass.newInstance();
				}
				//UPGRADE_NOTE: Exception 'java.lang.InstantiationException' was converted to 'System.Exception' which has different behavior. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1100'"
				catch (System.Exception e)
				{
					log.error("Error creating pooled object " + argClass.getSimpleName(), e);
					//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
					assert(false): Error creating pooled object  + argClass.getCanonicalName();
				}
				catch (System.UnauthorizedAccessException e)
				{
					log.error("Error creating pooled object " + argClass.getSimpleName(), e);
					//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
					assert(false): Error creating pooled object  + argClass.getCanonicalName();
				}
			}
			index = 0;
			container = (E[]) Array.newInstance(argClass, argContainerSize);
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		< E > implements IOrderedStack < E >
		//UPGRADE_NOTE: Final was removed from the declaration of 'log '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		//UPGRADE_NOTE: The initialization of  'log' was moved to static method 'org.jbox2d.pooling.normal.CircleStack'. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1005'"
		private static readonly Logger log;
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'pool '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private E[] pool;
		private int index;
		//UPGRADE_NOTE: Final was removed from the declaration of 'size '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private int size;
		//UPGRADE_NOTE: Final was removed from the declaration of 'container '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private E[] container;
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		SuppressWarnings(unchecked)
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		public CircleStack(Class < E > argClass, int argStackSize, int argContainerSize)
		
		public E pop()
		{
			index++;
			if (index >= size)
			{
				index = 0;
			}
			return pool[index];
		}
		
		public E[] pop(int argNum)
		{
			//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
			assert(argNum <= container.length): Container array is too small;
			if (index + argNum < size)
			{
				Array.Copy(pool, index, container, 0, argNum);
				index += argNum;
			}
			else
			{
				int overlap = (index + argNum) - size;
				Array.Copy(pool, index, container, 0, argNum - overlap);
				Array.Copy(pool, 0, container, argNum - overlap, overlap);
				index = overlap;
			}
			return container;
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		Override
		public virtual void  push(int argNum)
		{
		}
		static CircleStack()
		{
			log = LoggerFactory.getLogger(typeof(CircleStack));
		}
	}
}