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

// Created at 4:14:34 AM Jul 17, 2010

using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace org.jbox2d.pooling.arrays
{

    /// <summary>
    /// Not thread safe int[] pooling
    /// </summary>
    /// <author>Daniel Murphy</author>
    public class IntArray
    {
        private readonly Dictionary<int, int[]> map = new Dictionary<int, int[]>();

        public virtual int[] get_Renamed(int argLength)
        {
            Debug.Assert(argLength > 0);

            if (!map.ContainsKey(argLength))
            {
                map.Add(argLength, getInitializedArray(argLength));
            }

            Debug.Assert(map[argLength].Length == argLength); // Array not built of correct length
            return map[argLength];
        }

        protected internal virtual int[] getInitializedArray(int argLength)
        {
            return new int[argLength];
        }
    }
}