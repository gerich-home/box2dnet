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

using System;
using AABB = org.jbox2d.collision.AABB;
using Collision = org.jbox2d.collision.Collision;
using Distance = org.jbox2d.collision.Distance;
using TimeOfImpact = org.jbox2d.collision.TimeOfImpact;
using Mat22 = org.jbox2d.common.Mat22;
using Mat33 = org.jbox2d.common.Mat33;
using Rot = org.jbox2d.common.Rot;
using Settings = org.jbox2d.common.Settings;
using Vec2 = org.jbox2d.common.Vec2;
using Vec3 = org.jbox2d.common.Vec3;
using CircleContact = org.jbox2d.dynamics.contacts.CircleContact;
using Contact = org.jbox2d.dynamics.contacts.Contact;
using PolygonAndCircleContact = org.jbox2d.dynamics.contacts.PolygonAndCircleContact;
using PolygonContact = org.jbox2d.dynamics.contacts.PolygonContact;
//UPGRADE_TODO: The type 'org.jbox2d.pooling.IDynamicStack' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
using org.jbox2d.pooling;
using IWorldPool = org.jbox2d.pooling.IWorldPool;
using System.Collections.Generic;
using System.Diagnostics;

namespace org.jbox2d.pooling.normal
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

        private readonly Collision collision;
        private readonly TimeOfImpact toi;
        private readonly Distance dist;

        public DefaultWorldPool(int argSize, int argContainerSize)
        {
            args = new object[] { this };

            pcstack = new MutableStack<Contact, PolygonContact>(Settings.CONTACT_STACK_INIT_SIZE, classes, args);

            ccstack = new MutableStack<Contact, CircleContact>(Settings.CONTACT_STACK_INIT_SIZE, classes, args);

            cpstack = new MutableStack<Contact, PolygonAndCircleContact>(Settings.CONTACT_STACK_INIT_SIZE, classes, args);

            vecs = new OrderedStack<Vec2>(argSize, argContainerSize);
            vec3s = new OrderedStack<Vec3>(argSize, argContainerSize);
            mats = new OrderedStack<Mat22>(argSize, argContainerSize);
            aabbs = new OrderedStack<AABB>(argSize, argContainerSize);
            rots = new OrderedStack<Rot>(argSize, argContainerSize);
            mat33s = new OrderedStack<Mat33>(argSize, argContainerSize);

            dist = new Distance();
            collision = new Collision(this);
            toi = new TimeOfImpact(this);
        }

        public IDynamicStack<Contact> getPolyContactStack()
        {
            return pcstack;
        }

        public IDynamicStack<Contact> getCircleContactStack()
        {
            return ccstack;
        }

        public IDynamicStack<Contact> getPolyCircleContactStack()
        {
            return cpstack;
        }

        public Vec2 popVec2()
        {
            return vecs.pop();
        }

        public Vec2[] popVec2(int argNum)
        {
            return vecs.pop(argNum);
        }

        public void pushVec2(int argNum)
        {
            vecs.push(argNum);
        }

        public Vec3 popVec3()
        {
            return vec3s.pop();
        }

        public Vec3[] popVec3(int argNum)
        {
            return vec3s.pop(argNum);
        }

        public void pushVec3(int argNum)
        {
            vec3s.push(argNum);
        }

        public Mat22 popMat22()
        {
            return mats.pop();
        }

        public Mat22[] popMat22(int argNum)
        {
            return mats.pop(argNum);
        }

        public void pushMat22(int argNum)
        {
            mats.push(argNum);
        }

        public Mat33 popMat33()
        {
            return mat33s.pop();
        }

        public void pushMat33(int argNum)
        {
            mat33s.push(argNum);
        }

        public AABB popAABB()
        {
            return aabbs.pop();
        }

        public AABB[] popAABB(int argNum)
        {
            return aabbs.pop(argNum);
        }

        public void pushAABB(int argNum)
        {
            aabbs.push(argNum);
        }

        public Rot popRot()
        {
            return rots.pop();
        }

        public void pushRot(int num)
        {
            rots.push(num);
        }

        public Collision getCollision()
        {
            return collision;
        }

        public TimeOfImpact getTimeOfImpact()
        {
            return toi;
        }

        public Distance getDistance()
        {
            return dist;
        }

        public float[] getFloatArray(int argLength)
        {
            if (!afloats.ContainsKey(argLength))
            {
                afloats.Add(argLength, new float[argLength]);
            }

            Debug.Assert(afloats[argLength].Length == argLength); //Array not built with correct length
            return afloats[argLength];
        }

        public int[] getIntArray(int argLength)
        {
            if (!aints.ContainsKey(argLength))
            {
                aints.Add(argLength, new int[argLength]);
            }

            Debug.Assert(aints[argLength].Length == argLength); //Array not built with correct length
            return aints[argLength];
        }

        public Vec2[] getVec2Array(int argLength)
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