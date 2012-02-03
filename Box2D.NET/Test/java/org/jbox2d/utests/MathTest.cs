/// <summary>****************************************************************************
/// Copyright (c) 2011, Daniel Murphy All rights reserved.
/// 
/// Redistribution and use in source and binary forms, with or without modification, are permitted
/// provided that the following conditions are met: * Redistributions of source code must retain the
/// above copyright notice, this list of conditions and the following disclaimer. * Redistributions
/// in binary form must reproduce the above copyright notice, this list of conditions and the
/// following disclaimer in the documentation and/or other materials provided with the distribution.
/// 
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
/// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
/// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
/// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
/// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
/// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
/// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
/// WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
/// ****************************************************************************
/// </summary>
/// <summary> Created at 4:32:38 AM Jan 14, 2011</summary>
//using System;
//using Mat22 = org.jbox2d.common.Mat22;
//using MathUtils = org.jbox2d.common.MathUtils;
//using Vec2 = org.jbox2d.common.Vec2;
////UPGRADE_TODO: The type 'junit.framework.TestCase' could not be found. If it was not included in the conversion, there may be compiler issues. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1262'"
//using TestCase = junit.framework.TestCase;
//namespace org.jbox2d.utests
//{
	
//    /// <author>  Daniel Murphy
//    /// </author>
//    public class MathTest:TestCase
//    {
		
//        //UPGRADE_NOTE: Final was removed from the declaration of 'MAX '. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1003'"
//        //UPGRADE_WARNING: Data types in Visual C# might be different.  Verify the accuracy of narrowing conversions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1042'"
//        private static readonly int MAX = (int) (System.Single.MaxValue / 1000);
//        private const int RAND_ITERS = 100;
		
//        public void  testFastMath()
//        {
//            System.Random r = new System.Random();
//            for (int i = 0; i < RAND_ITERS; i++)
//            {
//                float a = (float) r.NextDouble() * MAX - MAX / 2;
//                //UPGRADE_WARNING: Data types in Visual C# might be different.  Verify the accuracy of narrowing conversions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1042'"
//                assertEquals((int) System.Math.Floor(a), MathUtils.floor(a));
//            }
			
//            for (int i = 0; i < RAND_ITERS; i++)
//            {
//                float a = (float) r.NextDouble() * MAX - MAX / 2;
//                //UPGRADE_WARNING: Data types in Visual C# might be different.  Verify the accuracy of narrowing conversions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1042'"
//                assertEquals((int) System.Math.Ceiling(a), MathUtils.ceil(a));
//            }
			
//            for (int i = 0; i < RAND_ITERS; i++)
//            {
//                float a = (float) r.NextDouble() * MAX - MAX / 2;
//                float b = (float) r.NextDouble() * MAX - MAX / 2;
//                assertEquals(System.Math.Max(a, b), MathUtils.max(a, b));
//            }
			
//            for (int i = 0; i < RAND_ITERS; i++)
//            {
//                float a = (float) r.NextDouble() * MAX - MAX / 2;
//                float b = (float) r.NextDouble() * MAX - MAX / 2;
//                assertEquals(System.Math.Min(a, b), MathUtils.min(a, b));
//            }
			
//            for (int i = 0; i < RAND_ITERS; i++)
//            {
//                float a = (float) r.NextDouble() * MAX - MAX / 2;
//                //UPGRADE_TODO: Method 'java.lang.Math.round' was converted to 'System.Math.Round' which has a different behavior. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1073_javalangMathround_float'"
//                assertEquals((int) System.Math.Round((double) a), MathUtils.round(a));
//            }
			
//            for (int i = 0; i < RAND_ITERS; i++)
//            {
//                float a = (float) r.NextDouble() * MAX - MAX / 2;
//                assertEquals(System.Math.Abs(a), MathUtils.abs(a));
//            }
//        }
		
//        public void  testVec2()
//        {
//            Vec2 v = new Vec2();
//            v.x = 0;
//            v.y = 1;
//            v.subLocal(new Vec2(10, 10));
//            assertEquals(- 10f, v.x);
//            assertEquals(- 9f, v.y);
			
//            Vec2 v2 = v.add(new Vec2(1, 1));
//            assertEquals(- 9f, v2.x);
//            assertEquals(- 8f, v2.y);
//            assertFalse(v.Equals(v2));
			
//            // TODO write tests for the rest of common lib
//        }
		
//        public void  testMat22Unsafes()
//        {
//            Vec2 v1 = new Vec2(10, - 1.3f);
//            Mat22 m1 = new Mat22(1, 34, - 3, 3);
//            Mat22 m2 = new Mat22(2, - 1, 4.1f, - 4);
//            Vec2 vo = new Vec2();
//            Mat22 mo = new Mat22();
			
//            Mat22.mulToOutUnsafe(m1, m2, mo);
//            assertEquals(Mat22.mul(m1, m2), mo);
			
//            Mat22.mulToOutUnsafe(m1, v1, vo);
//            assertEquals(Mat22.mul(m1, v1), vo);
			
//            Mat22.mulTransToOutUnsafe(m1, m2, mo);
//            assertEquals(Mat22.mulTrans(m1, m2), mo);
			
//            Mat22.mulTransToOutUnsafe(m1, v1, vo);
//            assertEquals(Mat22.mulTrans(m1, v1), vo);
//        }
//    }
//}