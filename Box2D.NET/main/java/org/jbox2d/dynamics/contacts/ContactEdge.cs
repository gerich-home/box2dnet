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
using Body = org.jbox2d.dynamics.Body;

namespace org.jbox2d.dynamics.contacts
{

    /// <summary>
    /// A contact edge is used to connect bodies and contacts together in a contact graph where each body
    /// is a node and each contact is an edge. A contact edge belongs to a doubly linked list maintained
    /// in each attached body. Each contact has two contact nodes, one for each attached body.
    /// </summary>
    /// <author>daniel</author>
    public class ContactEdge
    {
        /// <summary>
        /// provides quick access to the other body attached.
        /// </summary>
        public Body other = null;

        /// <summary>
        /// the contact
        /// </summary>
        public Contact contact = null;

        /// <summary>
        /// the previous contact edge in the body's contact list
        /// </summary>
        public ContactEdge prev = null;

        /// <summary>
        /// the next contact edge in the body's contact list
        /// </summary>
        public ContactEdge next = null;
    }
}