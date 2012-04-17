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

/*
* JBox2D - A Java Port of Erin Catto's Box2D
* 
* JBox2D homepage: http://jbox2d.sourceforge.net/
* Box2D homepage: http://www.box2d.org
* 
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

using System;

namespace Box2D.Collision
{

    /// <summary> 
    /// Contact ids to facilitate warm starting. Note: the ContactFeatures class is just embedded in here
    /// </summary>
    public class ContactID : IComparable<ContactID>
    {
        public enum Type
        {
            Vertex,
            Face
        }

        public sbyte IndexA;
        public sbyte IndexB;
        public sbyte TypeA;
        public sbyte TypeB;

        public int Key
        {
            get
            {
                //UPGRADE_ISSUE: check following line carefully!
                return IndexA << 24 | IndexB << 16 | TypeA << 8 | TypeB;
            }
        }

        public bool IsEqual(ContactID cid)
        {
            return Key == cid.Key;
        }

        public ContactID()
        {
        }

        public ContactID(ContactID c)
        {
            Set(c);
        }

        public void Set(ContactID c)
        {
            IndexA = c.IndexA;
            IndexB = c.IndexB;
            TypeA = c.TypeA;
            TypeB = c.TypeB;
        }

        public void Flip()
        {
            sbyte tempA = IndexA;
            IndexA = IndexB;
            IndexB = tempA;
            tempA = TypeA;
            TypeA = TypeB;
            TypeB = tempA;
        }

        /// <summary>
        ///  zeros out the data
        /// </summary>
        public void Zero()
        {
            IndexA = 0;
            IndexB = 0;
            TypeA = 0;
            TypeB = 0;
        }

        public int CompareTo(ContactID o)
        {
            return Key - o.Key;
        }
    }
}