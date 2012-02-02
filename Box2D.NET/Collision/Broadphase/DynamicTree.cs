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

//UPGRADE_ISSUE: The following fragment of code could not be parsed and was not converted. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1156'"

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
                return m_nodes[m_root].height;
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
                    //UPGRADE_NOTE: Final was removed from the declaration of 'node '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
                    TreeNode node = m_nodes[i];
                    if (node.height <= 1)
                    {
                        continue;
                    }

                    Debug.Assert(node.Leaf == false);

                    int child1 = node.child1;
                    int child2 = node.child2;
                    int balance = MathUtils.abs(m_nodes[child2].height - m_nodes[child1].height);
                    maxBalance = MathUtils.max(maxBalance, balance);
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
                float rootArea = root.aabb.Perimeter;

                float totalArea = 0.0f;
                for (int i = 0; i < m_nodeCapacity; ++i)
                {
                    TreeNode node = m_nodes[i];
                    if (node.height < 0)
                    {
                        // Free node in pool
                        continue;
                    }

                    totalArea += node.aabb.Perimeter;
                }

                return totalArea / rootArea;
            }
        }

        virtual public int InsertionCount
        {
            get
            {
                return m_insertionCount;
            }
        }

        public const int MAX_STACK_SIZE = 64;

        private int m_root;
        private TreeNode[] m_nodes;
        private int m_nodeCount;
        private int m_nodeCapacity;

        private int m_freeList;

        private int m_insertionCount;

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
                m_nodes[i] = new TreeNode();
                m_nodes[i].parent = i + 1;
                m_nodes[i].height = -1;
            }
            m_nodes[m_nodeCapacity - 1].parent = TreeNode.NULL_NODE;
            m_freeList = 0;

            m_insertionCount = 0;

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
        public int createProxy(AABB aabb, Object userData)
        {
            int proxyId = allocateNode();

            // Fatten the aabb
            TreeNode node = m_nodes[proxyId];
            node.aabb.lowerBound.x = aabb.lowerBound.x - Settings.aabbExtension;
            node.aabb.lowerBound.y = aabb.lowerBound.y - Settings.aabbExtension;
            node.aabb.upperBound.x = aabb.upperBound.x + Settings.aabbExtension;
            node.aabb.upperBound.y = aabb.upperBound.y + Settings.aabbExtension;
            node.userData = userData;

            insertLeaf(proxyId);

            return proxyId;
        }

        /// <summary>
        /// Destroy a proxy
        /// </summary>
        /// <param name="proxyId"></param>
        public void destroyProxy(int proxyId)
        {
            Debug.Assert(0 <= proxyId && proxyId < m_nodeCapacity);
            Debug.Assert(m_nodes[proxyId].Leaf);

            removeLeaf(proxyId);
            freeNode(proxyId);
        }

        // djm pooling
        /// <summary>
        /// Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB, then the
        /// proxy is removed from the tree and re-inserted. Otherwise the function returns immediately.
        /// </summary>
        /// <returns>true if the proxy was re-inserted.</returns>
        public bool moveProxy(int proxyId, AABB aabb, Vec2 displacement)
        {
            Debug.Assert(0 <= proxyId && proxyId < m_nodeCapacity);
            TreeNode node = m_nodes[proxyId];
            Debug.Assert(node.Leaf);

            if (node.aabb.contains(aabb))
            {
                return false;
            }

            removeLeaf(proxyId);

            // Extend AABB
            Vec2 lowerBound = aabb.lowerBound;
            Vec2 upperBound = aabb.upperBound;
            lowerBound.x -= Settings.aabbExtension;
            lowerBound.y -= Settings.aabbExtension;
            upperBound.x += Settings.aabbExtension;
            upperBound.y += Settings.aabbExtension;


            // Predict AABB displacement.
            float dx = displacement.x * Settings.aabbMultiplier;
            float dy = displacement.y * Settings.aabbMultiplier;
            if (dx < 0.0f)
            {
                lowerBound.x += dx;
            }
            else
            {
                upperBound.x += dx;
            }

            if (dy < 0.0f)
            {
                lowerBound.y += dy;
            }
            else
            {
                upperBound.y += dy;
            }
            node.aabb.lowerBound.x = lowerBound.x;
            node.aabb.lowerBound.y = lowerBound.y;
            node.aabb.upperBound.x = upperBound.x;
            node.aabb.upperBound.y = upperBound.y;

            insertLeaf(proxyId);
            return true;
        }

        public object getUserData(int proxyId)
        {
            Debug.Assert(0 <= proxyId && proxyId < m_nodeCapacity);
            return m_nodes[proxyId].userData;
        }

        public AABB getFatAABB(int proxyId)
        {
            Debug.Assert(0 <= proxyId && proxyId < m_nodeCapacity);
            return m_nodes[proxyId].aabb;
        }

        /// <summary>
        /// Query an AABB for overlapping proxies. The callback class is called for each proxy that
        /// overlaps the supplied AABB.
        /// </summary>
        /// <param name="callback"></param>
        /// <param name="araabbgAABB"></param>
        public void query(ITreeCallback callback, AABB aabb)
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

                if (AABB.testOverlap(node.aabb, aabb))
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
                        intStack.Push(node.child1);
                        intStack.Push(node.child2);
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
        public virtual void raycast(TreeRayCastCallback callback, RayCastInput input)
        {
            Vec2 p1 = input.p1;
            Vec2 p2 = input.p2;
            r.set_Renamed(p2).subLocal(p1);
            Debug.Assert(r.lengthSquared() > 0f);
            r.normalize();

            // v is perpendicular to the segment.
            Vec2.crossToOutUnsafe(1f, r, v);
            absV.set_Renamed(v).absLocal();

            // Separating axis for segment (Gino, p80).
            // |dot(v, p1 - c)| > dot(|v|, h)

            float maxFraction = input.maxFraction;

            // Build a bounding box for the segment.
            AABB segAABB = aabb;
            // Vec2 t = p1 + maxFraction * (p2 - p1);
            temp.set_Renamed(p2).subLocal(p1).mulLocal(maxFraction).addLocal(p1);
            Vec2.minToOut(p1, temp, segAABB.lowerBound);
            Vec2.maxToOut(p1, temp, segAABB.upperBound);

            intStack.Push(m_root);
            while (intStack.Count > 0)
            {
                int nodeId = intStack.Pop();
                if (nodeId == TreeNode.NULL_NODE)
                {
                    continue;
                }

                TreeNode node = m_nodes[nodeId];

                if (!AABB.testOverlap(node.aabb, segAABB))
                {
                    continue;
                }

                // Separating axis for segment (Gino, p80).
                // |dot(v, p1 - c)| > dot(|v|, h)
                node.aabb.getCenterToOut(c);
                node.aabb.getExtentsToOut(h);
                temp.set_Renamed(p1).subLocal(c);
                float separation = MathUtils.abs(Vec2.dot(v, temp)) - Vec2.dot(absV, h);
                if (separation > 0.0f)
                {
                    continue;
                }

                if (node.Leaf)
                {
                    subInput.p1.set_Renamed(input.p1);
                    subInput.p2.set_Renamed(input.p2);
                    subInput.maxFraction = maxFraction;

                    float value_Renamed = callback.raycastCallback(subInput, nodeId);

                    if (value_Renamed == 0.0f)
                    {
                        // The client has terminated the ray cast.
                        return;
                    }

                    if (value_Renamed > 0.0f)
                    {
                        // Update segment bounding box.
                        maxFraction = value_Renamed;
                        t.set_Renamed(p2).subLocal(p1).mulLocal(maxFraction).addLocal(p1);
                        Vec2.minToOut(p1, t, segAABB.lowerBound);
                        Vec2.maxToOut(p1, t, segAABB.upperBound);
                    }
                }
                else
                {
                    intStack.Push(node.child1);
                    intStack.Push(node.child2);
                }
            }
        }

        /// <summary>
        /// Compute the height of the tree.
        /// </summary>
        public int computeHeight()
        {
            return computeHeight(m_root);
        }

        private int computeHeight(int nodeId)
        {
            Debug.Assert(0 <= nodeId && nodeId < m_nodeCapacity);

            TreeNode node = m_nodes[nodeId];

            if (node.Leaf)
            {
                return 0;
            }
            int height1 = computeHeight(node.child1);
            int height2 = computeHeight(node.child2);
            return 1 + MathUtils.max(height1, height2);
        }

        /// <summary>
        /// Validate this tree. For testing.
        /// </summary>
        public virtual void validate()
        {
            validateStructure(m_root);
            validateMetrics(m_root);

            int freeCount = 0;
            int freeIndex = m_freeList;
            while (freeIndex != TreeNode.NULL_NODE)
            {
                Debug.Assert(0 <= freeIndex && freeIndex < m_nodeCapacity);
                freeIndex = m_nodes[freeIndex].parent;
                ++freeCount;
            }

            Debug.Assert(Height == computeHeight());

            Debug.Assert(m_nodeCount + freeCount == m_nodeCapacity);
        }

        /// <summary>
        /// Build an optimal tree. Very expensive. For testing.
        /// </summary>
        public virtual void rebuildBottomUp()
        {
            int[] nodes = new int[m_nodeCount];
            int count = 0;

            // Build array of leaves. Free the rest.
            for (int i = 0; i < m_nodeCapacity; ++i)
            {
                if (m_nodes[i].height < 0)
                {
                    // free node in pool
                    continue;
                }

                if (m_nodes[i].Leaf)
                {
                    m_nodes[i].parent = TreeNode.NULL_NODE;
                    nodes[count] = i;
                    ++count;
                }
                else
                {
                    freeNode(i);
                }
            }

            AABB b = new AABB();
            while (count > 1)
            {
                float minCost = Single.MaxValue;
                int iMin = -1, jMin = -1;
                for (int i = 0; i < count; ++i)
                {
                    AABB aabbi = m_nodes[nodes[i]].aabb;

                    for (int j = i + 1; j < count; ++j)
                    {
                        AABB aabbj = m_nodes[nodes[j]].aabb;
                        b.combine(aabbi, aabbj);
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

                int parentIndex = allocateNode();
                TreeNode parent = m_nodes[parentIndex];
                parent.child1 = index1;
                parent.child2 = index2;
                parent.height = 1 + MathUtils.max(child1.height, child2.height);
                parent.aabb.combine(child1.aabb, child2.aabb);
                parent.parent = TreeNode.NULL_NODE;

                child1.parent = parentIndex;
                child2.parent = parentIndex;

                nodes[jMin] = nodes[count - 1];
                nodes[iMin] = parentIndex;
                --count;
            }

            m_root = nodes[0];

            validate();
        }

        private int allocateNode()
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
                    m_nodes[i] = new TreeNode();
                    m_nodes[i].parent = i + 1;
                    m_nodes[i].height = -1;
                }
                m_nodes[m_nodeCapacity - 1].parent = TreeNode.NULL_NODE;
                m_freeList = m_nodeCount;
            }
            int nodeId = m_freeList;
            m_freeList = m_nodes[nodeId].parent;

            m_nodes[nodeId].parent = TreeNode.NULL_NODE;
            m_nodes[nodeId].child1 = TreeNode.NULL_NODE;
            m_nodes[nodeId].child2 = TreeNode.NULL_NODE;
            m_nodes[nodeId].height = 0;
            m_nodes[nodeId].userData = null;
            ++m_nodeCount;
            return nodeId;
        }

        /// <summary>
        /// returns a node to the pool
        /// </summary>
        /// <param name="argNode"></param>
        private void freeNode(int nodeId)
        {
            Debug.Assert(nodeId != TreeNode.NULL_NODE);
            Debug.Assert(0 < m_nodeCount);
            m_nodes[nodeId].parent = m_freeList;
            m_nodes[nodeId].height = -1;
            m_freeList = nodeId;
            m_nodeCount--;
        }

        private readonly Vec2 center = new Vec2();
        private readonly Vec2 delta1 = new Vec2();
        private readonly Vec2 delta2 = new Vec2();
        private readonly AABB combinedAABB = new AABB();

        private void insertLeaf(int leaf)
        {
            m_insertionCount++;

            if (m_root == TreeNode.NULL_NODE)
            {
                m_root = leaf;
                m_nodes[m_root].parent = TreeNode.NULL_NODE;
                return;
            }

            // find the best sibling
            AABB leafAABB = m_nodes[leaf].aabb;
            int index = m_root;
            while (m_nodes[index].Leaf == false)
            {
                TreeNode node = m_nodes[index];
                int child1 = node.child1;
                int child2 = node.child2;

                float area = node.aabb.Perimeter;

                combinedAABB.combine(node.aabb, leafAABB);
                float combinedArea = combinedAABB.Perimeter;

                // Cost of creating a new parent for this node and the new leaf
                float cost = 2.0f * combinedArea;

                // Minimum cost of pushing the leaf further down the tree
                float inheritanceCost = 2.0f * (combinedArea - area);

                // Cost of descending into child1
                float cost1;
                if (m_nodes[child1].Leaf)
                {
                    combinedAABB.combine(leafAABB, m_nodes[child1].aabb);
                    cost1 = combinedAABB.Perimeter + inheritanceCost;
                }
                else
                {
                    combinedAABB.combine(leafAABB, m_nodes[child1].aabb);
                    float oldArea = m_nodes[child1].aabb.Perimeter;
                    float newArea = combinedAABB.Perimeter;
                    cost1 = (newArea - oldArea) + inheritanceCost;
                }

                // Cost of descending into child2
                float cost2;
                if (m_nodes[child2].Leaf)
                {
                    combinedAABB.combine(leafAABB, m_nodes[child2].aabb);
                    cost2 = combinedAABB.Perimeter + inheritanceCost;
                }
                else
                {
                    combinedAABB.combine(leafAABB, m_nodes[child2].aabb);
                    float oldArea = m_nodes[child2].aabb.Perimeter;
                    float newArea = combinedAABB.Perimeter;
                    cost2 = newArea - oldArea + inheritanceCost;
                }

                // Descend according to the minimum cost.
                if (cost < cost1 && cost < cost2)
                {
                    break;
                }

                // Descend
                if (cost1 < cost2)
                {
                    index = child1;
                }
                else
                {
                    index = child2;
                }
            }

            int sibling = index;
            int oldParent = m_nodes[sibling].parent;
            int newParentId = allocateNode();
            TreeNode newParent = m_nodes[newParentId];
            newParent.parent = oldParent;
            newParent.userData = null;
            newParent.aabb.combine(leafAABB, m_nodes[sibling].aabb);
            newParent.height = m_nodes[sibling].height + 1;

            if (oldParent != TreeNode.NULL_NODE)
            {
                // The sibling was not the root.
                if (m_nodes[oldParent].child1 == sibling)
                {
                    m_nodes[oldParent].child1 = newParentId;
                }
                else
                {
                    m_nodes[oldParent].child2 = newParentId;
                }

                m_nodes[newParentId].child1 = sibling;
                m_nodes[newParentId].child2 = leaf;
                m_nodes[sibling].parent = newParentId;
                m_nodes[leaf].parent = newParentId;
            }
            else
            {
                // The sibling was the root.
                m_nodes[newParentId].child1 = sibling;
                m_nodes[newParentId].child2 = leaf;
                m_nodes[sibling].parent = newParentId;
                m_nodes[leaf].parent = newParentId;
                m_root = newParentId;
            }

            // Walk back up the tree fixing heights and AABBs
            index = m_nodes[leaf].parent;
            while (index != TreeNode.NULL_NODE)
            {
                index = balance(index);

                int child1 = m_nodes[index].child1;
                int child2 = m_nodes[index].child2;

                Debug.Assert(child1 != TreeNode.NULL_NODE);
                Debug.Assert(child2 != TreeNode.NULL_NODE);

                m_nodes[index].height = 1 + MathUtils.max(m_nodes[child1].height, m_nodes[child2].height);
                m_nodes[index].aabb.combine(m_nodes[child1].aabb, m_nodes[child2].aabb);

                index = m_nodes[index].parent;
            }

            // validate();
        }

        private void removeLeaf(int leaf)
        {
            if (leaf == m_root)
            {
                m_root = TreeNode.NULL_NODE;
                return;
            }

            int parent = m_nodes[leaf].parent;
            int grandParent = m_nodes[parent].parent;
            int sibling;
            if (m_nodes[parent].child1 == leaf)
            {
                sibling = m_nodes[parent].child2;
            }
            else
            {
                sibling = m_nodes[parent].child1;
            }

            if (grandParent != TreeNode.NULL_NODE)
            {
                // Destroy parent and connect sibling to grandParent.
                if (m_nodes[grandParent].child1 == parent)
                {
                    m_nodes[grandParent].child1 = sibling;
                }
                else
                {
                    m_nodes[grandParent].child2 = sibling;
                }
                m_nodes[sibling].parent = grandParent;
                freeNode(parent);

                // Adjust ancestor bounds.
                int index = grandParent;
                while (index != TreeNode.NULL_NODE)
                {
                    index = balance(index);

                    int child1 = m_nodes[index].child1;
                    int child2 = m_nodes[index].child2;

                    m_nodes[index].aabb.combine(m_nodes[child1].aabb, m_nodes[child2].aabb);
                    m_nodes[index].height = 1 + MathUtils.max(m_nodes[child1].height, m_nodes[child2].height);

                    index = m_nodes[index].parent;
                }
            }
            else
            {
                m_root = sibling;
                m_nodes[sibling].parent = TreeNode.NULL_NODE;
                freeNode(parent);
            }

            // validate();
        }

        // Perform a left or right rotation if node A is imbalanced.
        // Returns the new root index.
        private int balance(int iA)
        {
            Debug.Assert(iA != TreeNode.NULL_NODE);

            TreeNode A = m_nodes[iA];
            if (A.Leaf || A.height < 2)
            {
                return iA;
            }

            int iB = A.child1;
            int iC = A.child2;
            Debug.Assert(0 <= iB && iB < m_nodeCapacity);
            Debug.Assert(0 <= iC && iC < m_nodeCapacity);

            TreeNode B = m_nodes[iB];
            TreeNode C = m_nodes[iC];

            int balance = C.height - B.height;

            // Rotate C up
            if (balance > 1)
            {
                int iF = C.child1;
                int iG = C.child2;
                TreeNode F = m_nodes[iF];
                TreeNode G = m_nodes[iG];
                Debug.Assert(0 <= iF && iF < m_nodeCapacity);
                Debug.Assert(0 <= iG && iG < m_nodeCapacity);

                // Swap A and C
                C.child1 = iA;
                C.parent = A.parent;
                A.parent = iC;

                // A's old parent should point to C
                if (C.parent != TreeNode.NULL_NODE)
                {
                    if (m_nodes[C.parent].child1 == iA)
                    {
                        m_nodes[C.parent].child1 = iC;
                    }
                    else
                    {
                        Debug.Assert(m_nodes[C.parent].child2 == iA);
                        m_nodes[C.parent].child2 = iC;
                    }
                }
                else
                {
                    m_root = iC;
                }

                // Rotate
                if (F.height > G.height)
                {
                    C.child2 = iF;
                    A.child2 = iG;
                    G.parent = iA;
                    A.aabb.combine(B.aabb, G.aabb);
                    C.aabb.combine(A.aabb, F.aabb);

                    A.height = 1 + MathUtils.max(B.height, G.height);
                    C.height = 1 + MathUtils.max(A.height, F.height);
                }
                else
                {
                    C.child2 = iG;
                    A.child2 = iF;
                    F.parent = iA;
                    A.aabb.combine(B.aabb, F.aabb);
                    C.aabb.combine(A.aabb, G.aabb);

                    A.height = 1 + MathUtils.max(B.height, F.height);
                    C.height = 1 + MathUtils.max(A.height, G.height);
                }

                return iC;
            }

            // Rotate B up
            if (balance < -1)
            {
                int iD = B.child1;
                int iE = B.child2;
                TreeNode D = m_nodes[iD];
                TreeNode E = m_nodes[iE];
                Debug.Assert(0 <= iD && iD < m_nodeCapacity);
                Debug.Assert(0 <= iE && iE < m_nodeCapacity);

                // Swap A and B
                B.child1 = iA;
                B.parent = A.parent;
                A.parent = iB;

                // A's old parent should point to B
                if (B.parent != TreeNode.NULL_NODE)
                {
                    if (m_nodes[B.parent].child1 == iA)
                    {
                        m_nodes[B.parent].child1 = iB;
                    }
                    else
                    {
                        Debug.Assert(m_nodes[B.parent].child2 == iA);
                        m_nodes[B.parent].child2 = iB;
                    }
                }
                else
                {
                    m_root = iB;
                }

                // Rotate
                if (D.height > E.height)
                {
                    B.child2 = iD;
                    A.child1 = iE;
                    E.parent = iA;
                    A.aabb.combine(C.aabb, E.aabb);
                    B.aabb.combine(A.aabb, D.aabb);

                    A.height = 1 + MathUtils.max(C.height, E.height);
                    B.height = 1 + MathUtils.max(A.height, D.height);
                }
                else
                {
                    B.child2 = iE;
                    A.child1 = iD;
                    D.parent = iA;
                    A.aabb.combine(C.aabb, D.aabb);
                    B.aabb.combine(A.aabb, E.aabb);

                    A.height = 1 + MathUtils.max(C.height, D.height);
                    B.height = 1 + MathUtils.max(A.height, E.height);
                }

                return iB;
            }

            return iA;
        }

        private void validateStructure(int index)
        {
            if (index == TreeNode.NULL_NODE)
            {
                return;
            }

            if (index == m_root)
            {
                Debug.Assert(m_nodes[index].parent == TreeNode.NULL_NODE);
            }

            TreeNode node = m_nodes[index];

            int child1 = node.child1;
            int child2 = node.child2;

            if (node.Leaf)
            {
                Debug.Assert(child1 == TreeNode.NULL_NODE);
                Debug.Assert(child2 == TreeNode.NULL_NODE);
                Debug.Assert(node.height == 0);
                return;
            }

            Debug.Assert(0 <= child1 && child1 < m_nodeCapacity);
            Debug.Assert(0 <= child2 && child2 < m_nodeCapacity);

            Debug.Assert(m_nodes[child1].parent == index);
            Debug.Assert(m_nodes[child2].parent == index);

            validateStructure(child1);
            validateStructure(child2);
        }

        private void validateMetrics(int index)
        {
            if (index == TreeNode.NULL_NODE)
            {
                return;
            }

            TreeNode node = m_nodes[index];

            int child1 = node.child1;
            int child2 = node.child2;

            if (node.Leaf)
            {
                Debug.Assert(child1 == TreeNode.NULL_NODE);
                Debug.Assert(child2 == TreeNode.NULL_NODE);
                Debug.Assert(node.height == 0);
                return;
            }

            Debug.Assert(0 <= child1 && child1 < m_nodeCapacity);
            Debug.Assert(0 <= child2 && child2 < m_nodeCapacity);

            int height1 = m_nodes[child1].height;
            int height2 = m_nodes[child2].height;
            int height;
            height = 1 + MathUtils.max(height1, height2);
            Debug.Assert(node.height == height);

            AABB aabb = new AABB();
            aabb.combine(m_nodes[child1].aabb, m_nodes[child2].aabb);

            Debug.Assert(aabb.lowerBound.Equals(node.aabb.lowerBound));
            Debug.Assert(aabb.upperBound.Equals(node.aabb.upperBound));

            validateMetrics(child1);
            validateMetrics(child2);
        }

        public virtual void drawTree(DebugDraw argDraw)
        {
            if (m_root == TreeNode.NULL_NODE)
            {
                return;
            }
            int height = computeHeight();
            drawTree(argDraw, m_root, 0, height);
        }

        private readonly Color3f color = new Color3f();
        private readonly Vec2 textVec = new Vec2();

        public virtual void drawTree(DebugDraw argDraw, int nodeId, int spot, int height)
        {
            TreeNode node = m_nodes[nodeId];
            node.aabb.getVertices(drawVecs);

            color.set_Renamed(1, (height - spot) * 1f / height, (height - spot) * 1f / height);
            argDraw.DrawPolygon(drawVecs, 4, color);

            argDraw.ViewportTranform.getWorldToScreen(node.aabb.upperBound, textVec);
            argDraw.DrawString(textVec.x, textVec.y, nodeId + "-" + (spot + 1) + "/" + height, color);

            if (node.child1 != TreeNode.NULL_NODE)
            {
                drawTree(argDraw, node.child1, spot + 1, height);
            }
            if (node.child2 != TreeNode.NULL_NODE)
            {
                drawTree(argDraw, node.child2, spot + 1, height);
            }
        }
    }
}