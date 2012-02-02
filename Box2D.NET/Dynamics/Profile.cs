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
using System.Collections.Generic;

namespace Box2D.Dynamics
{

    public class Profile
    {
        public float Step;
        public float Collide;
        public float Solve;
        public float SolveInit;
        public float SolveVelocity;
        public float SolvePosition;
        public float Broadphase;
        public float SolveToi;

        public void ToDebugStrings(List<String> strings)
        {
            strings.Add("Profile:");
            strings.Add(string.Format(" step: {0}", Step));
            strings.Add(string.Format("  collide: {0}", Collide));
            strings.Add(string.Format("  solve: {0}", Solve));
            strings.Add(string.Format("   solveInit: {0}", SolveInit));
            strings.Add(string.Format("   solveVelocity: {0}", SolveVelocity));
            strings.Add(string.Format("   solvePosition: {0}", SolvePosition));
            strings.Add(string.Format("   broadphase: {0}", Broadphase));
            strings.Add(string.Format("  solveTOI: {0}", SolveToi));
        }
    }
}