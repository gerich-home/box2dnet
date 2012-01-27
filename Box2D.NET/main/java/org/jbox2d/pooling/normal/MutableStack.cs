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
using org.jbox2d.pooling;
using System.Diagnostics;
//UPGRADE_TODO: The type 'org.slf4j.Logger' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
//using Logger = org.slf4j.Logger;
//UPGRADE_TODO: The type 'org.slf4j.LoggerFactory' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
//using LoggerFactory = org.slf4j.LoggerFactory;

namespace org.jbox2d.pooling.normal
{

    public class MutableStack<E, T> : IDynamicStack<E>
        where T : E
    {
        //UPGRADE_TODO: there is no logger class
        //private static readonly Logger log;
        //static OrderedStack()
        //{
        //	log = LoggerFactory.getLogger(typeof(MutableStack));
        //}

        private T[] stack;
        private int index;
        private int size;

        private readonly Type[] _params;
        private readonly Object[] args;

        public MutableStack(int argInitSize) :
            this(argInitSize, null, null)
        {

        }

        public MutableStack(int argInitSize, Type[] argParam, Object[] argArgs)
        {
            index = 0;
            _params = argParam;
            args = argArgs;

            stack = null;
            index = 0;
            extendStack(argInitSize);
        }

        private void extendStack(int argSize)
        {
            T[] newStack = new T[argSize];

            if (stack != null)
            {
                Array.Copy(stack, 0, newStack, 0, size);
            }

            for (int i = 0; i < newStack.Length; i++)
            {
                try
                {
                    if (_params != null)
                    {
                        newStack[i] = (T)typeof(T).GetConstructor(_params).Invoke(args);
                    }
                    else
                    {
                        newStack[i] = (T)typeof(T).GetConstructor(Type.EmptyTypes).Invoke(args);
                    }
                }
                catch (Exception e)
                {
                    Debug.Assert(false); //"Error creating pooled object " + sClass.getCanonicalName();
                }
            }

            stack = newStack;
            size = newStack.Length;
        }

        public readonly E pop()
        {
            if (index >= size)
            {
                extendStack(size * 2);
            }
            return stack[index++];
        }

        public readonly void push(E argObject)
        {
            Debug.Assert(index > 0);
            stack[--index] = (T)argObject;
        }
    }
}