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
//UPGRADE_TODO: The type 'org.jbox2d.pooling.IDynamicStack' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
using IDynamicStack = org.jbox2d.pooling.IDynamicStack;
//UPGRADE_TODO: The type 'org.slf4j.Logger' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
using Logger = org.slf4j.Logger;
//UPGRADE_TODO: The type 'org.slf4j.LoggerFactory' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
using LoggerFactory = org.slf4j.LoggerFactory;
namespace org.jbox2d.pooling.normal
{
	
	public class MutableStack
	{
		public MutableStack()
		{
			InitBlock();
		}
		private void  InitBlock()
		{
			this(argClass, argInitSize, null, null);
			index = 0;
			sClass = argClass;
			params_Renamed = argParam;
			args = argArgs;
			
			stack = null;
			index = 0;
			extendStack(argInitSize);
		}
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		< E, T extends E > implements IDynamicStack < E >
		
		//UPGRADE_NOTE: Final was removed from the declaration of 'log '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		//UPGRADE_NOTE: The initialization of  'log' was moved to static method 'org.jbox2d.pooling.normal.MutableStack'. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1005'"
		private static readonly Logger log;
		
		private T[] stack;
		private int index;
		private int size;
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		private final Class < T > sClass;
		
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		private final Class < ? > [] params;
		//UPGRADE_NOTE: Final was removed from the declaration of 'args '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
		private System.Object[] args;
		
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		public MutableStack(Class < T > argClass, int argInitSize)
		
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		public MutableStack(Class < T > argClass, int argInitSize, Class < ? > [] argParam, Object [] argArgs)
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		SuppressWarnings(unchecked)
		private void  extendStack(int argSize)
		{
			T[] newStack = (T[]) Array.newInstance(sClass, argSize);
			if (stack != null)
			{
				Array.Copy(stack, 0, newStack, 0, size);
			}
			for (int i = 0; i < newStack.length; i++)
			{
				try
				{
					if (params_Renamed != null)
					{
						newStack[i] = sClass.getConstructor(params_Renamed).newInstance(args);
					}
					else
					{
						newStack[i] = sClass.newInstance();
					}
				}
				//UPGRADE_NOTE: Exception 'java.lang.InstantiationException' was converted to 'System.Exception' which has different behavior. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1100'"
				catch (System.Exception e)
				{
					log.error("Error creating pooled object " + sClass.getSimpleName(), e);
					//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
					assert(false): Error creating pooled object  + sClass.getCanonicalName();
				}
				catch (System.UnauthorizedAccessException e)
				{
					log.error("Error creating pooled object " + sClass.getSimpleName(), e);
					//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
					assert(false): Error creating pooled object  + sClass.getCanonicalName();
				}
				catch (System.ArgumentException e)
				{
					log.error("Error creating pooled object " + sClass.getSimpleName(), e);
					//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
					assert(false): Error creating pooled object  + sClass.getCanonicalName();
				}
				catch (System.Security.SecurityException e)
				{
					log.error("Error creating pooled object " + sClass.getSimpleName(), e);
					//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
					assert(false): Error creating pooled object  + sClass.getCanonicalName();
				}
				catch (System.Reflection.TargetInvocationException e)
				{
					log.error("Error creating pooled object " + sClass.getSimpleName(), e);
					//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
					assert(false): Error creating pooled object  + sClass.getCanonicalName();
				}
				catch (System.MethodAccessException e)
				{
					log.error("Error creating pooled object " + sClass.getSimpleName(), e);
					//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
					assert(false): Error creating pooled object  + sClass.getCanonicalName();
				}
			}
			stack = newStack;
			size = newStack.length;
		}
		
		/* (non-Javadoc)
		* 
		* @see org.jbox2d.pooling.IDynamicStack#pop() */
		public E pop()
		{
			if (index >= size)
			{
				extendStack(size * 2);
			}
			return stack[index++];
		}
		
		/* (non-Javadoc)
		* 
		* @see org.jbox2d.pooling.IDynamicStack#push(E) */
		//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
		SuppressWarnings(unchecked)
		public void  push(E argObject)
		{
			assert(index > 0);
			stack[--index] = (T) argObject;
		}
		static MutableStack()
		{
			log = LoggerFactory.getLogger(typeof(MutableStack));
		}
	}
}