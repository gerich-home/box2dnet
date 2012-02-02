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

//UPGRADE_TODO: The type 'org.slf4j.Logger' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
//using Logger = org.slf4j.Logger;
//UPGRADE_TODO: The type 'org.slf4j.LoggerFactory' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
//using LoggerFactory = org.slf4j.LoggerFactory;

namespace Box2D.Pooling.Normal
{

    public class CircleStack<E> : IOrderedStack<E>
        where E : new()
    {
        //UPGRADE_TODO: there is no logger class
        //private static readonly Logger log;
        //static CircleStack()
        //{
        //	log = LoggerFactory.getLogger(typeof(CircleStack));
        //}

        private readonly E[] pool;
        private int index;
        private readonly int size;
        private readonly E[] container;

        public CircleStack(int argStackSize, int argContainerSize)
        {
            size = argStackSize;
            pool = new E[argStackSize];

            for (int i = 0; i < argStackSize; i++)
            {
                try
                {
                    pool[i] = new E();
                }
                catch (Exception e)
                {
                    //log.error("Error creating pooled object " + argClass.getSimpleName(), e);
                    //UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
                    Debug.Assert(false); // Error creating pooled object  + argClass.getCanonicalName();
                }
                /*catch (System.UnauthorizedAccessException e)
                {
                    log.error("Error creating pooled object " + argClass.getSimpleName(), e);
                    //UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"
                    assert(false): Error creating pooled object  + argClass.getCanonicalName();
                }*/
            }
            index = 0;
            container = new E[argContainerSize];
        }

        public E Pop()
        {
            index++;
            if (index >= size)
            {
                index = 0;
            }
            return pool[index];
        }

        public E[] Pop(int argNum)
        {
            Debug.Assert(argNum <= container.Length); //Container array is too small
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

        public virtual void Push(int argNum)
        {
        }
    }
}