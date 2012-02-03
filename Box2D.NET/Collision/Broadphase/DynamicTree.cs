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
using Box2D.Callbacks;
using Box2D.Common;
using Box2D.Pooling.Stacks;

namespace Box2D.Collision.Broadphase
{
    // updated to rev 100
    /// <summary>
    /// A dynamic tree arranges data in a binary tree to accelerate queries such as volume queries and
    /// ray casts. Leafs are proxies with an AABB. In the tree we expand the proxy AABB by _fatAABBFactor
    /// so that the proxy AABB is bigger than the client object. This allows the client object to move by
    /// small amounts without triggering a tree update.
    /// </summary>
    /// <author>daniel</author>
    public class DynamicTree
    {
        /// <summary>
        /// Compute the height of the binary tree in O(N) time. Should not be called often.
        /// </summary>
        /// <returns></returns>
        virtual public int Height
        {
            get
            {
                if (m_root == TreeNode.NULL_NODE)
                {
                    return 0;
                }
                return m_nodes[m_root].Height;
            }
        }

        /// <summary>
        /// Get the maximum balance of an node in the tree. The balance is the difference in height of the
        /// two children of a node.
        /// </summary>
        /// <returns></returns>
        virtual public int MaxBalance
        {
            get
            {
                int maxBalance = 0;
                for (int i = 0; i < m_nodeCapacity; ++i)
                {
                    TreeNode node = m_nodes[i];
                    if (node.Height <= 1)
                    {
                        continue;
                    }

                    Debug.Assert(node.Leaf == false);

                    int child1 = node.Child1;
                    int child2 = node.Child2;
                    int balance = MathUtils.Abs(m_nodes[child2].Height - m_nodes[child1].Height);
                    maxBalance = MathUtils.Max(maxBalance, balance);
                }

                return maxBalance;
            }
        }

        /// <summary>
        /// Get the ratio of the sum of the node areas to the root area.
        /// </summary>
        /// <returns></returns>
        virtual public float AreaRatio
        {
            get
            {
                if (m_root == TreeNode.NULL_NODE)
                {
                    return 0.0f;
                }

                TreeNode root = m_nodes[m_root];
                float rootArea = root.AABB.Perimeter;

                float totalArea = 0.0f;
                for (int i = 0; i < m_nodeCapacity; ++i)
                {
                    TreeNode node = m_nodes[i];
                    if (node.Height < 0)
                    {
                        // Free node in pool
                        continue;
                    }

                    totalArea += node.AABB.Perimeter;
                }

                return totalArea / rootArea;
            }
        }

        public virtual int InsertionCount { get; private set; }

        public const int MAX_STACK_SIZE = 64;

        private int m_root;
        private TreeNode[] m_nodes;
        private int m_nodeCount;
        private int m_nodeCapacity;

        private int m_freeList;

        private readonly Vec2[] drawVecs = new Vec2[4];
        private readonly DynamicIntStack intStack = new DynamicIntStack(10);

        public DynamicTree()
        {
            m_root = TreeNode.NULL_NODE;
            m_nodeCount = 0;
            m_nodeCapacity = 16;
            m_nodes = new TreeNode[16];

            // Build a linked list for the free list.
            for (int i = 0; i < m_nodeCapacity; i++)
            {
                m_nodes[i] = new TreeNode {Parent = i + 1, Height = -1};
            }
            m_nodes[m_nodeCapacity - 1].Parent = TreeNode.NULL_NODE;
            m_freeList = 0;

            InsertionCount = 0;

            for (int i = 0; i < drawVecs.Length; i++)
            {
                drawVecs[i] = new Vec2();
            }
        }

        /// <summary>
        /// Create a proxy. Provide a tight fitting AABB and a userData pointer.
        /// </summary>
        /// <param name="aabb"></param>
        /// <param name="userData"></param>
        /// <returns></returns>
        public int CreateProxy(AABB aabb, Object userData)
        {
            int proxyId = AllocateNode();

            // Fatten the aabb
            TreeNode node = m_nodes[proxyId];
            node.AABB.LowerBound.X = aabb.LowerBound.X - Settings.AABB_EXTENSION;
            node.AABB.LowerBound.Y = aabb.LowerBound.Y - Settings.AABB_EXTENSION;
            node.AABB.UpperBound.X = aabb.UpperBound.X + Settings.AABB_EXTENSION;
            node.AABB.UpperBound.Y = aabb.UpperBound.Y + Settings.AABB_EXTENSION;
            node.UserData = userData;

            InsertLeaf(proxyId);

            return proxyId;
        }

        /// <summary>
        /// Destroy a proxy
        /// </summary>
        /// <param name="proxyId"></param>
        public void DestroyProxy(int proxyId)
        {
            Debug.Assert(0 <= proxyId && proxyId < m_nodeCapacity);
            Debug.Assert(m_nodes[proxyId].Leaf);

            RemoveLeaf(proxyId);
            FreeNode(proxyId);
        }

        // djm pooling
        /// <summary>
        /// Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB, then the
        /// proxy is removed from the tree and re-inserted. Otherwise the function returns immediately.
        /// </summary>
        /// <returns>true if the proxy was re-inserted.</returns>
        public bool MoveProxy(int proxyId, AABB aabb, Vec2 displacement)
        {
            Debug.Assert(0 <= proxyId && proxyId < m_nodeCapacity);
            TreeNode node = m_nodes[proxyId];
            Debug.Assert(node.Leaf);

            if (node.AABB.Contains(aabb))
            {
                return false;
            }

            RemoveLeaf(proxyId);

            // Extend AABB
            Vec2 lowerBound = aabb.LowerBound;
            Vec2 upperBound = aabb.UpperBound;
            lowerBound.X -= Settings.AABB_EXTENSION;
            lowerBound.Y -= Settings.AABB_EXTENSION;
            upperBound.X += Settings.AABB_EXTENSION;
            upperBound.Y += Settings.AABB_EXTENSION;


            // Predict AABB displacement.
            float dx = displacement.X * Settings.AABB_MULTIPLIER;
            float dy = displacement.Y * Settings.AABB_MULTIPLIER;
            if (dx < 0.0f)
            {
                lowerBound.X += dx;
            }
            else
            {
                upperBound.X += dx;
            }

            if (dy < 0.0f)
            {
                lowerBound.Y += dy;
            }
            else
            {
                upperBound.Y += dy;
            }
            node.AABB.LowerBound.X = lowerBound.X;
            node.AABB.LowerBound.Y = lowerBound.Y;
            node.AABB.UpperBound.X = upperBound.X;
            node.AABB.UpperBound.Y = upperBound.Y;

            InsertLeaf(proxyId);
            return true;
        }

        public object GetUserData(int proxyId)
        {
            Debug.Assert(0 <= proxyId && proxyId < m_nodeCapacity);
            return m_nodes[proxyId].UserData;
        }

        public AABB GetFatAABB(int proxyId)
        {
            Debug.Assert(0 <= proxyId && proxyId < m_nodeCapacity);
            return m_nodes[proxyId].AABB;
        }

        /// <summary>
        /// Query an AABB for overlapping proxies. The callback class is called for each proxy that
        /// overlaps the supplied AABB.
        /// </summary>
        /// <param name="callback"></param>
        /// <param name="aabb"></param>
        public void Query(ITreeCallback callback, AABB aabb)
        {
            intStack.Reset();
            intStack.Push(m_root);

            while (intStack.Count > 0)
            {
                int nodeId = intStack.Pop();
                if (nodeId == TreeNode.NULL_NODE)
                {
                    continue;
                }

                TreeNode node = m_nodes[nodeId];

                if (AABB.TestOverlap(node.AABB, aabb))
                {
                    if (node.Leaf)
                    {
                        bool proceed = callback.TreeCallback(nodeId);
                        if (!proceed)
                        {
                            return;
                        }
                    }
                    else
                    {
                        intStack.Push(node.Child1);
                        intStack.Push(node.Child2);
                    }
                }
            }
        }

        private readonly Vec2 r = new Vec2();
        private readonly Vec2 v = new Vec2();
        private readonly Vec2 absV = new Vec2();
        private readonly Vec2 temp = new Vec2();
        private readonly Vec2 c = new Vec2();
        private readonly Vec2 h = new Vec2();
        private readonly Vec2 t = new Vec2();
        private readonly AABB aabb = new AABB();
        private readonly RayCastInput subInput = new RayCastInput();

        /// <summary>
        /// Ray-cast against the proxies in the tree. This relies on the callback to perform a exact
        /// ray-cast in the case were the proxy contains a shape. The callback also performs the any
        /// collision filtering. This has performance roughly equal to k * log(n), where k is the number of
        /// collisions and n is the number of proxies in the tree.
        /// </summary>
        /// <param name="input">the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).</param>
        /// <param name="callback">a callback class that is called for each proxy that is hit by the ray.</param>
        public virtual void Raycast(ITreeRayCastCallback callback, RayCastInput input)
        {
            Vec2 p1 = input.P1;
            Vec2 p2 = input.P2;
            r.Set(p2).SubLocal(p1);
            Debug.Assert(r.LengthSquared() > 0f);
            r.Normalize();

            // v is perpendicular to the segment.
            Vec2.CrossToOutUnsafe(1f, r, v);
            absV.Set(v).AbsLocal();

            // Separating axis for segment (Gino, p80).
            // |dot(v, p1 - c)| > dot(|v|, h)

            float maxFraction = input.MaxFraction;

            // Build a bounding box for the segment.
            AABB segAABB = aabb;
            // Vec2 t = p1 + maxFraction * (p2 - p1);
            temp.Set(p2).SubLocal(p1).MulLocal(maxFraction).AddLocal(p1);
            Vec2.MinToOut(p1, temp, segAABB.LowerBound);
            Vec2.MaxToOut(p1, temp, segAABB.UpperBound);

            intStack.Push(m_root);
            while (intStack.Count > 0)
            {
                int nodeId = intStack.Pop();
                if (nodeId == TreeNode.NULL_NODE)
                {
                    continue;
                }

                TreeNode node = m_nodes[nodeId];

                if (!AABB.TestOverlap(node.AABB, segAABB))
                {
                    continue;
                }

                // Separating axis for segment (Gino, p80).
                // |dot(v, p1 - c)| > dot(|v|, h)
                node.AABB.GetCenterToOut(c);
                node.AABB.GetExtentsToOut(h);
                temp.Set(p1).SubLocal(c);
                float separation = MathUtils.Abs(Vec2.Dot(v, temp)) - Vec2.Dot(absV, h);
                if (separation > 0.0f)
                {
                    continue;
                }

                if (node.Leaf)
                {
                    subInput.P1.Set(input.P1);
                    subInput.P2.Set(input.P2);
                    subInput.MaxFraction = maxFraction;

                    float value = callback.RaycastCallback(subInput, nodeId);

                    if (value == 0.0f)
                    {
                        // The client has terminated the ray cast.
                        return;
                    }

                    if (value > 0.0f)
                    {
                        // Update segment bounding box.
                        maxFraction = value;
                        t.Set(p2).SubLocal(p1).MulLocal(maxFraction).AddLocal(p1);
                        Vec2.MinToOut(p1, t, segAABB.LowerBound);
                        Vec2.MaxToOut(p1, t, segAABB.UpperBound);
                    }
                }
                else
                {
                    intStack.Push(node.Child1);
                    intStack.Push(node.Child2);
                }
            }
        }

        /// <summary>
        /// Compute the height of the tree.
        /// </summary>
        public int ComputeHeight()
        {
            return ComputeHeight(m_root);
        }

        private int ComputeHeight(int nodeId)
        {
            Debug.Assert(0 <= nodeId && nodeId < m_nodeCapacity);

            TreeNode node = m_nodes[nodeId];

            if (node.Leaf)
            {
                return 0;
            }
            int height1 = ComputeHeight(node.Child1);
            int height2 = ComputeHeight(node.Child2);
            return 1 + MathUtils.Max(height1, height2);
        }

        /// <summary>
        /// Validate this tree. For testing.
        /// </summary>
        public virtual void Validate()
        {
            ValidateStructure(m_root);
            ValidateMetrics(m_root);

            int freeCount = 0;
            int freeIndex = m_freeList;
            while (freeIndex != TreeNode.NULL_NODE)
            {
                Debug.Assert(0 <= freeIndex && freeIndex < m_nodeCapacity);
                freeIndex = m_nodes[freeIndex].Parent;
                ++freeCount;
            }

            Debug.Assert(Height == ComputeHeight());

            Debug.Assert(m_nodeCount + freeCount == m_nodeCapacity);
        }

        /// <summary>
        /// Build an optimal tree. Very expensive. For testing.
        /// </summary>
        public virtual void RebuildBottomUp()
        {
            var nodes = new int[m_nodeCount];
            int count = 0;

            // Build array of leaves. Free the rest.
            for (int i = 0; i < m_nodeCapacity; ++i)
            {
                if (m_nodes[i].Height < 0)
                {
                    // free node in pool
                    continue;
                }

                if (m_nodes[i].Leaf)
                {
                    m_nodes[i].Parent = TreeNode.NULL_NODE;
                    nodes[count] = i;
                    ++count;
                }
                else
                {
                    FreeNode(i);
                }
            }

            var b = new AABB();
            while (count > 1)
            {
                float minCost = Single.MaxValue;
                int iMin = -1, jMin = -1;
                for (int i = 0; i < count; ++i)
                {
                    AABB aabbi = m_nodes[nodes[i]].AABB;

                    for (int j = i + 1; j < count; ++j)
                    {
                        AABB aabbj = m_nodes[nodes[j]].AABB;
                        b.Combine(aabbi, aabbj);
                        float cost = b.Perimeter;
                        if (cost < minCost)
                        {
                            iMin = i;
                            jMin = j;
                            minCost = cost;
                        }
                    }
                }

                int index1 = nodes[iMin];
                int index2 = nodes[jMin];
                TreeNode child1 = m_nodes[index1];
                TreeNode child2 = m_nodes[index2];

                int parentIndex = AllocateNode();
                TreeNode parent = m_nodes[parentIndex];
                parent.Child1 = index1;
                parent.Child2 = index2;
                parent.Height = 1 + MathUtils.Max(child1.Height, child2.Height);
                parent.AABB.Combine(child1.AABB, child2.AABB);
                parent.Parent = TreeNode.NULL_NODE;

                child1.Parent = parentIndex;
                child2.Parent = parentIndex;

                nodes[jMin] = nodes[count - 1];
                nodes[iMin] = parentIndex;
                --count;
            }

            m_root = nodes[0];

            Validate();
        }

        private int AllocateNode()
        {
            if (m_freeList == TreeNode.NULL_NODE)
            {
                Debug.Assert(m_nodeCount == m_nodeCapacity);

                TreeNode[] old = m_nodes;
                m_nodeCapacity *= 2;
                m_nodes = new TreeNode[m_nodeCapacity];
                Array.Copy(old, 0, m_nodes, 0, old.Length);

                // Build a linked list for the free list.
                for (int i = m_nodeCount; i < m_nodeCapacity; i++)
                {
                    m_nodes[i] = new TreeNode {Parent = i + 1, Height = -1};
                }
                m_nodes[m_nodeCapacity - 1].Parent = TreeNode.NULL_NODE;
                m_freeList = m_nodeCount;
            }
            int nodeId = m_freeList;
            m_freeList = m_nodes[nodeId].Parent;

            m_nodes[nodeId].Parent = TreeNode.NULL_NODE;
            m_nodes[nodeId].Child1 = TreeNode.NULL_NODE;
            m_nodes[nodeId].Child2 = TreeNode.NULL_NODE;
            m_nodes[nodeId].Height = 0;
            m_nodes[nodeId].UserData = null;
            ++m_nodeCount;
            return nodeId;
        }

        /// <summary>
        /// returns a node to the pool
        /// </summary>
        /// <param name="nodeId"></param>
        private void FreeNode(int nodeId)
        {
            Debug.Assert(nodeId != TreeNode.NULL_NODE);
            Debug.Assert(0 < m_nodeCount);
            m_nodes[nodeId].Parent = m_freeList;
            m_nodes[nodeId].Height = -1;
            m_freeList = nodeId;
            m_nodeCount--;
        }

        private readonly Vec2 center = new Vec2();
        private readonly Vec2 delta1 = new Vec2();
        private readonly Vec2 delta2 = new Vec2();
        private readonly AABB combinedAABB = new AABB();

        private void InsertLeaf(int leaf)
        {
            InsertionCount++;

            if (m_root == TreeNode.NULL_NODE)
            {
                m_root = leaf;
                m_nodes[m_root].Parent = TreeNode.NULL_NODE;
                return;
            }

            // find the best sibling
            AABB leafAABB = m_nodes[leaf].AABB;
            int index = m_root;
            while (m_nodes[index].Leaf == false)
            {
                TreeNode node = m_nodes[index];
                int child1 = node.Child1;
                int child2 = node.Child2;

                float area = node.AABB.Perimeter;

                combinedAABB.Combine(node.AABB, leafAABB);
                float combinedArea = combinedAABB.Perimeter;

                // Cost of creating a new parent for this node and the new leaf
                float cost = 2.0f * combinedArea;

                // Minimum cost of pushing the leaf further down the tree
                float inheritanceCost = 2.0f * (combinedArea - area);

                // Cost of descending into child1
                float cost1;
                if (m_nodes[child1].Leaf)
                {
                    combinedAABB.Combine(leafAABB, m_nodes[child1].AABB);
                    cost1 = combinedAABB.Perimeter + inheritanceCost;
                }
                else
                {
                    combinedAABB.Combine(leafAABB, m_nodes[child1].AABB);
                    float oldArea = m_nodes[child1].AABB.Perimeter;
                    float newArea = combinedAABB.Perimeter;
                    cost1 = (newArea - oldArea) + inheritanceCost;
                }

                // Cost of descending into child2
                float cost2;
                if (m_nodes[child2].Leaf)
                {
                    combinedAABB.Combine(leafAABB, m_nodes[child2].AABB);
                    cost2 = combinedAABB.Perimeter + inheritanceCost;
                }
                else
                {
                    combinedAABB.Combine(leafAABB, m_nodes[child2].AABB);
                    float oldArea = m_nodes[child2].AABB.Perimeter;
                    float newArea = combinedAABB.Perimeter;
                    cost2 = newArea - oldArea + inheritanceCost;
                }

                // Descend according to the minimum cost.
                if (cost < cost1 && cost < cost2)
                {
                    break;
                }

                // Descend
                index = cost1 < cost2 ? child1 : child2;
            }

            int sibling = index;
            int oldParent = m_nodes[sibling].Parent;
            int newParentId = AllocateNode();
            TreeNode newParent = m_nodes[newParentId];
            newParent.Parent = oldParent;
            newParent.UserData = null;
            newParent.AABB.Combine(leafAABB, m_nodes[sibling].AABB);
            newParent.Height = m_nodes[sibling].Height + 1;

            if (oldParent != TreeNode.NULL_NODE)
            {
                // The sibling was not the root.
                if (m_nodes[oldParent].Child1 == sibling)
                {
                    m_nodes[oldParent].Child1 = newParentId;
                }
                else
                {
                    m_nodes[oldParent].Child2 = newParentId;
                }

                m_nodes[newParentId].Child1 = sibling;
                m_nodes[newParentId].Child2 = leaf;
                m_nodes[sibling].Parent = newParentId;
                m_nodes[leaf].Parent = newParentId;
            }
            else
            {
                // The sibling was the root.
                m_nodes[newParentId].Child1 = sibling;
                m_nodes[newParentId].Child2 = leaf;
                m_nodes[sibling].Parent = newParentId;
                m_nodes[leaf].Parent = newParentId;
                m_root = newParentId;
            }

            // Walk back up the tree fixing heights and AABBs
            index = m_nodes[leaf].Parent;
            while (index != TreeNode.NULL_NODE)
            {
                index = Balance(index);

                int child1 = m_nodes[index].Child1;
                int child2 = m_nodes[index].Child2;

                Debug.Assert(child1 != TreeNode.NULL_NODE);
                Debug.Assert(child2 != TreeNode.NULL_NODE);

                m_nodes[index].Height = 1 + MathUtils.Max(m_nodes[child1].Height, m_nodes[child2].Height);
                m_nodes[index].AABB.Combine(m_nodes[child1].AABB, m_nodes[child2].AABB);

                index = m_nodes[index].Parent;
            }

            // validate();
        }

        private void RemoveLeaf(int leaf)
        {
            if (leaf == m_root)
            {
                m_root = TreeNode.NULL_NODE;
                return;
            }

            int parent = m_nodes[leaf].Parent;
            int grandParent = m_nodes[parent].Parent;
            int sibling;
            if (m_nodes[parent].Child1 == leaf)
            {
                sibling = m_nodes[parent].Child2;
            }
            else
            {
                sibling = m_nodes[parent].Child1;
            }

            if (grandParent != TreeNode.NULL_NODE)
            {
                // Destroy parent and connect sibling to grandParent.
                if (m_nodes[grandParent].Child1 == parent)
                {
                    m_nodes[grandParent].Child1 = sibling;
                }
                else
                {
                    m_nodes[grandParent].Child2 = sibling;
                }
                m_nodes[sibling].Parent = grandParent;
                FreeNode(parent);

                // Adjust ancestor bounds.
                int index = grandParent;
                while (index != TreeNode.NULL_NODE)
                {
                    index = Balance(index);

                    int child1 = m_nodes[index].Child1;
                    int child2 = m_nodes[index].Child2;

                    m_nodes[index].AABB.Combine(m_nodes[child1].AABB, m_nodes[child2].AABB);
                    m_nodes[index].Height = 1 + MathUtils.Max(m_nodes[child1].Height, m_nodes[child2].Height);

                    index = m_nodes[index].Parent;
                }
            }
            else
            {
                m_root = sibling;
                m_nodes[sibling].Parent = TreeNode.NULL_NODE;
                FreeNode(parent);
            }

            // validate();
        }

        // Perform a left or right rotation if node A is imbalanced.
        // Returns the new root index.
        private int Balance(int iA)
        {
            Debug.Assert(iA != TreeNode.NULL_NODE);

            TreeNode A = m_nodes[iA];
            if (A.Leaf || A.Height < 2)
            {
                return iA;
            }

            int iB = A.Child1;
            int iC = A.Child2;
            Debug.Assert(0 <= iB && iB < m_nodeCapacity);
            Debug.Assert(0 <= iC && iC < m_nodeCapacity);

            TreeNode B = m_nodes[iB];
            TreeNode C = m_nodes[iC];

            int balance = C.Height - B.Height;

            // Rotate C up
            if (balance > 1)
            {
                int iF = C.Child1;
                int iG = C.Child2;
                TreeNode F = m_nodes[iF];
                TreeNode G = m_nodes[iG];
                Debug.Assert(0 <= iF && iF < m_nodeCapacity);
                Debug.Assert(0 <= iG && iG < m_nodeCapacity);

                // Swap A and C
                C.Child1 = iA;
                C.Parent = A.Parent;
                A.Parent = iC;

                // A's old parent should point to C
                if (C.Parent != TreeNode.NULL_NODE)
                {
                    if (m_nodes[C.Parent].Child1 == iA)
                    {
                        m_nodes[C.Parent].Child1 = iC;
                    }
                    else
                    {
                        Debug.Assert(m_nodes[C.Parent].Child2 == iA);
                        m_nodes[C.Parent].Child2 = iC;
                    }
                }
                else
                {
                    m_root = iC;
                }

                // Rotate
                if (F.Height > G.Height)
                {
                    C.Child2 = iF;
                    A.Child2 = iG;
                    G.Parent = iA;
                    A.AABB.Combine(B.AABB, G.AABB);
                    C.AABB.Combine(A.AABB, F.AABB);

                    A.Height = 1 + MathUtils.Max(B.Height, G.Height);
                    C.Height = 1 + MathUtils.Max(A.Height, F.Height);
                }
                else
                {
                    C.Child2 = iG;
                    A.Child2 = iF;
                    F.Parent = iA;
                    A.AABB.Combine(B.AABB, F.AABB);
                    C.AABB.Combine(A.AABB, G.AABB);

                    A.Height = 1 + MathUtils.Max(B.Height, F.Height);
                    C.Height = 1 + MathUtils.Max(A.Height, G.Height);
                }

                return iC;
            }

            // Rotate B up
            if (balance < -1)
            {
                int iD = B.Child1;
                int iE = B.Child2;
                TreeNode D = m_nodes[iD];
                TreeNode E = m_nodes[iE];
                Debug.Assert(0 <= iD && iD < m_nodeCapacity);
                Debug.Assert(0 <= iE && iE < m_nodeCapacity);

                // Swap A and B
                B.Child1 = iA;
                B.Parent = A.Parent;
                A.Parent = iB;

                // A's old parent should point to B
                if (B.Parent != TreeNode.NULL_NODE)
                {
                    if (m_nodes[B.Parent].Child1 == iA)
                    {
                        m_nodes[B.Parent].Child1 = iB;
                    }
                    else
                    {
                        Debug.Assert(m_nodes[B.Parent].Child2 == iA);
                        m_nodes[B.Parent].Child2 = iB;
                    }
                }
                else
                {
                    m_root = iB;
                }

                // Rotate
                if (D.Height > E.Height)
                {
                    B.Child2 = iD;
                    A.Child1 = iE;
                    E.Parent = iA;
                    A.AABB.Combine(C.AABB, E.AABB);
                    B.AABB.Combine(A.AABB, D.AABB);

                    A.Height = 1 + MathUtils.Max(C.Height, E.Height);
                    B.Height = 1 + MathUtils.Max(A.Height, D.Height);
                }
                else
                {
                    B.Child2 = iE;
                    A.Child1 = iD;
                    D.Parent = iA;
                    A.AABB.Combine(C.AABB, D.AABB);
                    B.AABB.Combine(A.AABB, E.AABB);

                    A.Height = 1 + MathUtils.Max(C.Height, D.Height);
                    B.Height = 1 + MathUtils.Max(A.Height, E.Height);
                }

                return iB;
            }

            return iA;
        }

        private void ValidateStructure(int index)
        {
            if (index == TreeNode.NULL_NODE)
            {
                return;
            }

            if (index == m_root)
            {
                Debug.Assert(m_nodes[index].Parent == TreeNode.NULL_NODE);
            }

            TreeNode node = m_nodes[index];

            int child1 = node.Child1;
            int child2 = node.Child2;

            if (node.Leaf)
            {
                Debug.Assert(child1 == TreeNode.NULL_NODE);
                Debug.Assert(child2 == TreeNode.NULL_NODE);
                Debug.Assert(node.Height == 0);
                return;
            }

            Debug.Assert(0 <= child1 && child1 < m_nodeCapacity);
            Debug.Assert(0 <= child2 && child2 < m_nodeCapacity);

            Debug.Assert(m_nodes[child1].Parent == index);
            Debug.Assert(m_nodes[child2].Parent == index);

            ValidateStructure(child1);
            ValidateStructure(child2);
        }

        private void ValidateMetrics(int index)
        {
            if (index == TreeNode.NULL_NODE)
            {
                return;
            }

            TreeNode node = m_nodes[index];

            int child1 = node.Child1;
            int child2 = node.Child2;

            if (node.Leaf)
            {
                Debug.Assert(child1 == TreeNode.NULL_NODE);
                Debug.Assert(child2 == TreeNode.NULL_NODE);
                Debug.Assert(node.Height == 0);
                return;
            }

            Debug.Assert(0 <= child1 && child1 < m_nodeCapacity);
            Debug.Assert(0 <= child2 && child2 < m_nodeCapacity);

            int height1 = m_nodes[child1].Height;
            int height2 = m_nodes[child2].Height;
            int height = 1 + MathUtils.Max(height1, height2);
            Debug.Assert(node.Height == height);

            var aabb = new AABB();
            aabb.Combine(m_nodes[child1].AABB, m_nodes[child2].AABB);

            Debug.Assert(aabb.LowerBound.Equals(node.AABB.LowerBound));
            Debug.Assert(aabb.UpperBound.Equals(node.AABB.UpperBound));

            ValidateMetrics(child1);
            ValidateMetrics(child2);
        }

        public virtual void DrawTree(DebugDraw argDraw)
        {
            if (m_root == TreeNode.NULL_NODE)
            {
                return;
            }
            int height = ComputeHeight();
            DrawTree(argDraw, m_root, 0, height);
        }

        private readonly Color3f color = new Color3f();
        private readonly Vec2 textVec = new Vec2();

        public virtual void DrawTree(DebugDraw argDraw, int nodeId, int spot, int height)
        {
            TreeNode node = m_nodes[nodeId];
            node.AABB.GetVertices(drawVecs);

            color.Set(1, (height - spot) * 1f / height, (height - spot) * 1f / height);
            argDraw.DrawPolygon(drawVecs, 4, color);

            argDraw.ViewportTranform.GetWorldToScreen(node.AABB.UpperBound, textVec);
            argDraw.DrawString(textVec.X, textVec.Y, nodeId + "-" + (spot + 1) + "/" + height, color);

            if (node.Child1 != TreeNode.NULL_NODE)
            {
                DrawTree(argDraw, node.Child1, spot + 1, height);
            }
            if (node.Child2 != TreeNode.NULL_NODE)
            {
                DrawTree(argDraw, node.Child2, spot + 1, height);
            }
        }
    }
}