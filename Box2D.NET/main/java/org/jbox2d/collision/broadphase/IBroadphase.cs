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
		
		int createProxy(AABB aabb, System.Object userData);
		
		void  destroyProxy(int proxyId);
		
		void  moveProxy(int proxyIdA, int proxyIdB);
		
		void  touchProxy(int proxyId);
		
		AABB getFatAABB(int proxyId);
		
		System.Object getUserData(int proxyId);
		
		bool testOverlap(int proxyIdA, int proxyIdB);
		
		void  query(QueryCallback callback, AABB aabb);
		
		void  raycast(RayCastCallback callback, RayCastInput input);
	}
}