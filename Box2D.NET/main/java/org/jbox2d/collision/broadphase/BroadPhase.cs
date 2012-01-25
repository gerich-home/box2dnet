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
using DebugDraw = org.jbox2d.callbacks.DebugDraw;
using PairCallback = org.jbox2d.callbacks.PairCallback;
using TreeCallback = org.jbox2d.callbacks.TreeCallback;
using TreeRayCastCallback = org.jbox2d.callbacks.TreeRayCastCallback;
using AABB = org.jbox2d.collision.AABB;
using RayCastInput = org.jbox2d.collision.RayCastInput;
using Vec2 = org.jbox2d.common.Vec2;
//UPGRADE_TODO: The type 'org.slf4j.Logger' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
//using Logger = org.slf4j.Logger;
//UPGRADE_TODO: The type 'org.slf4j.LoggerFactory' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
//using LoggerFactory = org.slf4j.LoggerFactory;

namespace org.jbox2d.collision.broadphase
{

	/// <summary>
	/// The broad-phase is used for computing pairs and performing volume queries and ray casts. This
	/// broad-phase does not persist pairs. Instead, this reports potentially new pairs. It is up to the
	/// client to consume the new pairs and to track subsequent overlap.
	/// </summary>
	/// <author>Daniel Murphy</author>
	public class BroadPhase : TreeCallback
	{
		/// <summary>
		/// Get the number of proxies.
		/// </summary>
		/// <returns></returns>
		virtual public int ProxyCount
		{
			get
			{
				return m_proxyCount;
			}
		}

		/// <summary>
		/// Get the height of the embedded tree.
		/// </summary>
		/// <returns></returns>
		virtual public int TreeHeight
		{
			get
			{
				return m_tree.computeHeight();
			}
		}

		virtual public int TreeBalance
		{
			get
			{
				return m_tree.MaxBalance;
			}
		}

		virtual public float TreeQuality
		{
			get
			{
				return m_tree.AreaRatio;
			}
		}

		//private static readonly Logger log = LoggerFactory.getLogger(typeof(BroadPhase));

		public const int NULL_PROXY = -1;

		private readonly DynamicTree m_tree;

		private int m_proxyCount;

		private int[] m_moveBuffer;
		private int m_moveCapacity;
		private int m_moveCount;

		private Pair[] m_pairBuffer;
		private int m_pairCapacity;
		private int m_pairCount;

		private int m_queryProxyId;

		public BroadPhase()
		{
			m_proxyCount = 0;

			m_pairCapacity = 16;
			m_pairCount = 0;
			m_pairBuffer = new Pair[m_pairCapacity];
			for (int i = 0; i < m_pairCapacity; i++)
			{
				m_pairBuffer[i] = new Pair();
			}

			m_moveCapacity = 16;
			m_moveCount = 0;
			m_moveBuffer = new int[m_moveCapacity];

			m_tree = new DynamicTree();
			m_queryProxyId = NULL_PROXY;
		}

		/// <summary>
		/// Create a proxy with an initial AABB. Pairs are not reported until updatePairs is called.
		/// </summary>
		/// <param name="aabb"></param>
		/// <param name="userData"></param>
		/// <returns></returns>
		public int createProxy(AABB aabb, object userData)
		{
			int proxyId = m_tree.createProxy(aabb, userData);
			++m_proxyCount;
			bufferMove(proxyId);
			return proxyId;
		}

		/// <summary>
		/// Destroy a proxy. It is up to the client to remove any pairs.
		/// </summary>
		/// <param name="proxyId"></param>
		public void destroyProxy(int proxyId)
		{
			unbufferMove(proxyId);
			--m_proxyCount;
			m_tree.destroyProxy(proxyId);
		}

		/// <summary>
		/// Call MoveProxy as many times as you like, then when you are done call UpdatePairs to finalized
		/// the proxy pairs (for your time step).
		/// </summary>
		public void moveProxy(int proxyId, AABB aabb, Vec2 displacement)
		{
			bool buffer = m_tree.moveProxy(proxyId, aabb, displacement);
			if (buffer)
			{
				bufferMove(proxyId);
			}
		}

		public virtual void touchProxy(int proxyId)
		{
			bufferMove(proxyId);
		}

		public virtual object getUserData(int proxyId)
		{
			return m_tree.getUserData(proxyId);
		}

		public virtual AABB getFatAABB(int proxyId)
		{
			return m_tree.getFatAABB(proxyId);
		}

		public virtual bool testOverlap(int proxyIdA, int proxyIdB)
		{
			// return AABB.testOverlap(proxyA.aabb, proxyB.aabb);
			AABB a = m_tree.getFatAABB(proxyIdA);
			AABB b = m_tree.getFatAABB(proxyIdB);
			if (b.lowerBound.x - a.upperBound.x > 0.0f || b.lowerBound.y - a.upperBound.y > 0.0f)
			{
				return false;
			}

			if (a.lowerBound.x - b.upperBound.x > 0.0f || a.lowerBound.y - b.upperBound.y > 0.0f)
			{
				return false;
			}

			return true;
		}

		public virtual void drawTree(DebugDraw argDraw)
		{
			m_tree.drawTree(argDraw);
		}

		/// <summary>
		/// Update the pairs. This results in pair callbacks. This can only add pairs.
		/// </summary>
		/// <param name="callback"></param>
		public void updatePairs(PairCallback callback)
		{
			// log.debug("beginning to update pairs");
			// Reset pair buffer
			m_pairCount = 0;

			// Perform tree queries for all moving proxies.
			for (int i = 0; i < m_moveCount; ++i)
			{
				m_queryProxyId = m_moveBuffer[i];
				if (m_queryProxyId == NULL_PROXY)
				{
					continue;
				}

				// We have to query the tree with the fat AABB so that
				// we don't fail to create a pair that may touch later.
				AABB fatAABB = m_tree.getFatAABB(m_queryProxyId);

				// Query tree, create pairs and add them pair buffer.
				// log.debug("quering aabb: "+m_queryProxy.aabb);
				m_tree.query(this, fatAABB);
			}
			// log.debug("Number of pairs found: "+m_pairCount);

			// Reset move buffer
			m_moveCount = 0;

			// Sort the pair buffer to expose duplicates.
			System.Array.Sort(m_pairBuffer, 0, m_pairCount - 0);

			// Send the pairs back to the client.
			int i2 = 0;
			while (i2 < m_pairCount)
			{
				Pair primaryPair = m_pairBuffer[i2];
				System.Object userDataA = m_tree.getUserData(primaryPair.proxyIdA);
				System.Object userDataB = m_tree.getUserData(primaryPair.proxyIdB);

				// log.debug("returning pair: "+userDataA+", "+userDataB);
				callback.addPair(userDataA, userDataB);
				++i2;

				// Skip any duplicate pairs.
				while (i2 < m_pairCount)
				{
					Pair pair = m_pairBuffer[i2];
					if (pair.proxyIdA != primaryPair.proxyIdA || pair.proxyIdB != primaryPair.proxyIdB)
					{
						break;
					}
					// log.debug("skipping duplicate");
					++i2;
				}
			}

			// Try to keep the tree balanced.
			// m_tree.rebalance(Settings.TREE_REBALANCE_STEPS);
		}

		/// <summary>
		/// Query an AABB for overlapping proxies. The callback class is called for each proxy that
		/// overlaps the supplied AABB.
		/// </summary>
		/// <param name="callback"></param>
		/// <param name="aabb"></param>
		public void query(TreeCallback callback, AABB aabb)
		{
			m_tree.query(callback, aabb);
		}

		/// <summary>
		/// Ray-cast against the proxies in the tree. This relies on the callback to perform a exact
		/// ray-cast in the case were the proxy contains a shape. The callback also performs the any
		/// collision filtering. This has performance roughly equal to k * log(n), where k is the number of
		/// collisions and n is the number of proxies in the tree.
		/// </summary>
		/// <param name="input">the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).</param>
		/// <param name="callback">a callback class that is called for each proxy that is hit by the ray.</param>
		public void raycast(TreeRayCastCallback callback, RayCastInput input)
		{
			m_tree.raycast(callback, input);
		}

		protected internal void bufferMove(int proxyId)
		{
			if (m_moveCount == m_moveCapacity)
			{
				int[] old = m_moveBuffer;
				m_moveCapacity *= 2;
				m_moveBuffer = new int[m_moveCapacity];
				Array.Copy(old, 0, m_moveBuffer, 0, old.Length);
			}

			m_moveBuffer[m_moveCount] = proxyId;
			++m_moveCount;
		}

		protected internal void unbufferMove(int proxyId)
		{
			for (int i = 0; i < m_moveCount; i++)
			{
				if (m_moveBuffer[i] == proxyId)
				{
					m_moveBuffer[i] = NULL_PROXY;
				}
			}
		}

		// private final PairStack pairStack = new PairStack();
		/// <summary>
		/// This is called from DynamicTree::query when we are gathering pairs.
		/// </summary>
		public bool treeCallback(int proxyId)
		{

			// log.debug("Got a proxy back: " + proxyId);
			// A proxy cannot form a pair with itself.
			if (proxyId == m_queryProxyId)
			{
				// log.debug("It was us...");
				return true;
			}

			// Grow the pair buffer as needed.
			if (m_pairCount == m_pairCapacity)
			{
				Pair[] oldBuffer = m_pairBuffer;
				m_pairCapacity *= 2;
				m_pairBuffer = new Pair[m_pairCapacity];
				Array.Copy(oldBuffer, 0, m_pairBuffer, 0, oldBuffer.Length);
				for (int i = oldBuffer.Length; i < m_pairCapacity; i++)
				{
					m_pairBuffer[i] = new Pair();
				}
			}

			if (proxyId < m_queryProxyId)
			{
				// log.debug("new proxy is first");
				m_pairBuffer[m_pairCount].proxyIdA = proxyId;
				m_pairBuffer[m_pairCount].proxyIdB = m_queryProxyId;
			}
			else
			{
				// log.debug("new proxy is second");
				m_pairBuffer[m_pairCount].proxyIdA = m_queryProxyId;
				m_pairBuffer[m_pairCount].proxyIdB = proxyId;
			}

			++m_pairCount;
			return true;
		}
	}
}