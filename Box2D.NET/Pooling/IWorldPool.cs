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

using Box2D.Collision;
using Box2D.Common;
using Box2D.Dynamics.Contacts;

namespace Box2D.Pooling
{

    /// <summary>
    /// World pool interface
    /// </summary>
    /// <author>Daniel</author>
    public interface IWorldPool
    {
        IDynamicStack<Contact> GetPolyContactStack();
        IDynamicStack<Contact> GetCircleContactStack();
        IDynamicStack<Contact> GetPolyCircleContactStack();
        IDynamicStack<Contact> GetEdgeCircleContactStack();
        IDynamicStack<Contact> GetEdgePolyContactStack();
        IDynamicStack<Contact> GetChainCircleContactStack();
        IDynamicStack<Contact> GetChainPolyContactStack();

        Vec2 PopVec2();
        Vec2[] PopVec2(int num);
        void PushVec2(int num);

        Vec3 PopVec3();
        Vec3[] PopVec3(int num);
        void PushVec3(int num);

        Mat22 PopMat22();
        Mat22[] PopMat22(int num);
        void PushMat22(int num);

        Mat33 PopMat33();
        void PushMat33(int num);

        AABB PopAABB();
        AABB[] PopAABB(int num);
        void PushAABB(int num);

        Rot PopRot();
        void PushRot(int num);

        Collision.Collision GetCollision();

        TimeOfImpact GetTimeOfImpact();

        Distance GetDistance();

        float[] GetFloatArray(int argLength);

        int[] GetIntArray(int argLength);

        Vec2[] GetVec2Array(int argLength);
    }
}