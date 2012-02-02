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

// Created at 3:26:14 AM Jan 11, 2011

using System.Collections.Generic;
using System.Diagnostics;
using Box2D.Collision;
using Box2D.Common;
using Box2D.Dynamics.Contacts;
using Type = System.Type;

namespace Box2D.Pooling.Normal
{
    /// <summary>
    /// Provides object pooling for all objects used in the engine. Objects retrieved from here should
    /// only be used temporarily, and then pushed back (with the exception of arrays).
    /// </summary>
    /// <author>Daniel Murphy</author>
    public class DefaultWorldPool : IWorldPool
    {
        private readonly OrderedStack<Vec2> vecs;
        private readonly OrderedStack<Vec3> vec3s;
        private readonly OrderedStack<Mat22> mats;
        private readonly OrderedStack<Mat33> mat33s;
        private readonly OrderedStack<AABB> aabbs;
        private readonly OrderedStack<Rot> rots;

        private readonly Dictionary<int, float[]> afloats = new Dictionary<int, float[]>();
        private readonly Dictionary<int, int[]> aints = new Dictionary<int, int[]>();
        private readonly Dictionary<int, Vec2[]> avecs = new Dictionary<int, Vec2[]>();

        private readonly Type[] classes = new Type[] { typeof(IWorldPool) };
        private readonly object[] args;

        private readonly MutableStack<Contact, PolygonContact> pcstack;
        private readonly MutableStack<Contact, CircleContact> ccstack;
        private readonly MutableStack<Contact, PolygonAndCircleContact> cpstack;
        private readonly MutableStack<Contact, EdgeAndCircleContact> ecstack;
        private readonly MutableStack<Contact, EdgeAndPolygonContact> epstack;
        private readonly MutableStack<Contact, ChainAndCircleContact> chcstack;
        private readonly MutableStack<Contact, ChainAndPolygonContact> chpstack;

        private readonly Collision.Collision collision;
        private readonly TimeOfImpact toi;
        private readonly Distance dist;

        public DefaultWorldPool(int argSize, int argContainerSize)
        {
            args = new object[] { this };

            pcstack = new MutableStack<Contact, PolygonContact>(Settings.CONTACT_STACK_INIT_SIZE, classes, args);
            ccstack = new MutableStack<Contact, CircleContact>(Settings.CONTACT_STACK_INIT_SIZE, classes, args);
            cpstack = new MutableStack<Contact, PolygonAndCircleContact>(Settings.CONTACT_STACK_INIT_SIZE, classes, args);
            ecstack = new MutableStack<Contact, EdgeAndCircleContact>(Settings.CONTACT_STACK_INIT_SIZE, classes, args);
            epstack = new MutableStack<Contact, EdgeAndPolygonContact>(Settings.CONTACT_STACK_INIT_SIZE, classes, args);
            chcstack = new MutableStack<Contact, ChainAndCircleContact>(Settings.CONTACT_STACK_INIT_SIZE, classes, args);
            chpstack = new MutableStack<Contact, ChainAndPolygonContact>(Settings.CONTACT_STACK_INIT_SIZE, classes, args);

            vecs = new OrderedStack<Vec2>(argSize, argContainerSize);
            vec3s = new OrderedStack<Vec3>(argSize, argContainerSize);
            mats = new OrderedStack<Mat22>(argSize, argContainerSize);
            aabbs = new OrderedStack<AABB>(argSize, argContainerSize);
            rots = new OrderedStack<Rot>(argSize, argContainerSize);
            mat33s = new OrderedStack<Mat33>(argSize, argContainerSize);

            dist = new Distance();
            collision = new Collision.Collision(this);
            toi = new TimeOfImpact(this);
        }

        public IDynamicStack<Contact> GetPolyContactStack()
        {
            return pcstack;
        }

        public IDynamicStack<Contact> GetCircleContactStack()
        {
            return ccstack;
        }

        public IDynamicStack<Contact> GetPolyCircleContactStack()
        {
            return cpstack;
        }

        public IDynamicStack<Contact> GetEdgeCircleContactStack()
        {
            return ecstack;
        }

        public IDynamicStack<Contact> GetEdgePolyContactStack()
        {
            return epstack;
        }

        public IDynamicStack<Contact> GetChainCircleContactStack()
        {
            return chcstack;
        }

        public IDynamicStack<Contact> GetChainPolyContactStack()
        {
            return chpstack;
        }

        public Vec2 PopVec2()
        {
            return vecs.pop();
        }

        public Vec2[] PopVec2(int argNum)
        {
            return vecs.pop(argNum);
        }

        public void PushVec2(int argNum)
        {
            vecs.push(argNum);
        }

        public Vec3 PopVec3()
        {
            return vec3s.pop();
        }

        public Vec3[] PopVec3(int argNum)
        {
            return vec3s.pop(argNum);
        }

        public void PushVec3(int argNum)
        {
            vec3s.push(argNum);
        }

        public Mat22 PopMat22()
        {
            return mats.pop();
        }

        public Mat22[] PopMat22(int argNum)
        {
            return mats.pop(argNum);
        }

        public void PushMat22(int argNum)
        {
            mats.push(argNum);
        }

        public Mat33 PopMat33()
        {
            return mat33s.pop();
        }

        public void PushMat33(int argNum)
        {
            mat33s.push(argNum);
        }

        public AABB PopAABB()
        {
            return aabbs.pop();
        }

        public AABB[] PopAABB(int argNum)
        {
            return aabbs.pop(argNum);
        }

        public void PushAABB(int argNum)
        {
            aabbs.push(argNum);
        }

        public Rot PopRot()
        {
            return rots.pop();
        }

        public void PushRot(int num)
        {
            rots.push(num);
        }

        public Collision.Collision GetCollision()
        {
            return collision;
        }

        public TimeOfImpact GetTimeOfImpact()
        {
            return toi;
        }

        public Distance GetDistance()
        {
            return dist;
        }

        public float[] GetFloatArray(int argLength)
        {
            if (!afloats.ContainsKey(argLength))
            {
                afloats.Add(argLength, new float[argLength]);
            }

            Debug.Assert(afloats[argLength].Length == argLength); //Array not built with correct length
            return afloats[argLength];
        }

        public int[] GetIntArray(int argLength)
        {
            if (!aints.ContainsKey(argLength))
            {
                aints.Add(argLength, new int[argLength]);
            }

            Debug.Assert(aints[argLength].Length == argLength); //Array not built with correct length
            return aints[argLength];
        }

        public Vec2[] GetVec2Array(int argLength)
        {
            if (!avecs.ContainsKey(argLength))
            {
                Vec2[] ray = new Vec2[argLength];
                for (int i = 0; i < argLength; i++)
                {
                    ray[i] = new Vec2();
                }
                avecs.Add(argLength, ray);
            }

            Debug.Assert(avecs[argLength].Length == argLength); //Array not built with correct length
            return avecs[argLength];
        }
    }
}