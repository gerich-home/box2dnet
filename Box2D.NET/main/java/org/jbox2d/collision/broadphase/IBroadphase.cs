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
using QueryCallback = org.jbox2d.callbacks.QueryCallback;
using RayCastCallback = org.jbox2d.callbacks.RayCastCallback;
using TreeCallback = org.jbox2d.callbacks.TreeCallback;
using AABB = org.jbox2d.collision.AABB;
using RayCastInput = org.jbox2d.collision.RayCastInput;

namespace org.jbox2d.collision.broadphase
{

    public interface IBroadphase
    {
        int ProxyCount
        {
            get;
        }

        int TreeHeight
        {
            get;
        }

        int TreeBalance
        {
            get;
        }

        float TreeQuality
        {
            get;
        }

        int createProxy(AABB aabb, object userData);

        void destroyProxy(int proxyId);

        void moveProxy(int proxyIdA, int proxyIdB);

        void touchProxy(int proxyId);

        AABB getFatAABB(int proxyId);

        object getUserData(int proxyId);

        bool testOverlap(int proxyIdA, int proxyIdB);

        void query(QueryCallback callback, AABB aabb);

        void raycast(RayCastCallback callback, RayCastInput input);
    }
}