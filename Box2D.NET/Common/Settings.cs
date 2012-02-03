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

namespace Box2D.Common
{

    /// <summary>
    /// Global tuning constants based on MKS units and various integer maximums (vertices per shape,
    /// pairs, etc.).
    /// </summary>
    public class Settings
    {
        /// <summary>
        /// A "close to zero" float epsilon value for use
        /// </summary>
        public const float EPSILON = 1.1920928955078125e-7f;

        /// <summary>
        /// Pi
        /// </summary>
        //UPGRADE_WARNING: Data types in Visual C# might be different.  Verify the accuracy of narrowing conversions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1042'"
        public static readonly float PI = (float)Math.PI;

        // JBox2D specific settings
        /// <summary>
        /// needs to be final, or will slow down math methods
        /// </summary>
        public const bool FAST_MATH = true;
        public const int CONTACT_STACK_INIT_SIZE = 10;
        public const bool SINCOS_LUT_ENABLED = true;

        /// <summary>
        /// smaller the precision, the larger the table. If a small table is used (eg, precision is .006 or
        /// greater), make sure you set the table to lerp it's results. Accuracy chart is in the MathUtils
        /// source. Or, run the tests yourself in {@link SinCosTest}.</br>
        /// </br>
        /// Good lerp precision
        /// values:
        /// <ul>
        /// <li>.0092</li>
        /// <li>.008201</li>
        /// <li>.005904</li>
        /// <li>.005204</li>
        /// <li>.004305</li>
        /// <li>.002807</li>
        /// <li>.001508</li>
        /// <li>9.32500E-4</li>
        /// <li>7.48000E-4</li>
        /// <li>8.47000E-4</li>
        /// <li>.0005095</li>
        /// <li>.0001098</li>
        /// <li>9.50499E-5</li>
        /// <li>6.08500E-5</li>
        /// <li>3.07000E-5</li>
        /// <li>1.53999E-5</li>
        /// </ul>
        /// </summary>
        public const float SINCOS_LUT_PRECISION = .00011f;

        //UPGRADE_WARNING: Data types in Visual C# might be different.  Verify the accuracy of narrowing conversions. "ms-help://MS.VSCC.v80/dv_commoner/local/redirect.htm?index='!DefaultContextWindowIndex'&keyword='jlca1042'"
        public static readonly int SINCOS_LUT_LENGTH = (int)Math.Ceiling(Math.PI * 2 / SINCOS_LUT_PRECISION);

        /// <summary>
        /// Use if the table's precision is large (eg .006 or greater). Although it is more expensive, it
        /// greatly increases accuracy. Look in the MathUtils source for some test results on the accuracy
        /// and speed of lerp vs non lerp. Or, run the tests yourself in {@link SinCosTest}.
        /// </summary>
        public const bool SINCOS_LUT_LERP = false;


        // Collision

        /// <summary>
        /// The maximum number of contact points between two convex shapes.
        /// </summary>
        public const int MAX_MANIFOLD_POINTS = 2;

        /// <summary>
        /// The maximum number of vertices on a convex polygon.
        /// </summary>
        public const int MAX_POLYGON_VERTICES = 8;

        /// <summary>
        /// This is used to fatten AABBs in the dynamic tree. This allows proxies to move by a small amount
        /// without triggering a tree adjustment. This is in meters.
        /// </summary>
        public const float AABB_EXTENSION = 0.1f;

        /// <summary>
        /// This is used to fatten AABBs in the dynamic tree. This is used to predict the future position
        /// based on the current displacement. This is a dimensionless multiplier.
        /// </summary>
        public const float AABB_MULTIPLIER = 2.0f;

        /// <summary>
        /// A small length used as a collision and constraint tolerance. Usually it is chosen to be
        /// numerically significant, but visually insignificant.
        /// </summary>
        public const float LINEAR_SLOP = 0.005f;

        /// <summary>
        /// A small angle used as a collision and constraint tolerance. Usually it is chosen to be
        /// numerically significant, but visually insignificant.
        /// </summary>
        public static readonly float ANGULAR_SLOP = (2.0f / 180.0f * PI);

        /// <summary>
        /// The radius of the polygon/edge shape skin. This should not be modified. Making this smaller
        /// means polygons will have and insufficient for continuous collision. Making it larger may create
        /// artifacts for vertex collision.
        /// </summary>
        public static readonly float POLYGON_RADIUS = (2.0f * LINEAR_SLOP);

        /// <summary>
        /// Maximum number of sub-steps per contact in continuous physics simulation.
        /// </summary>
        public const int MAX_SUB_STEPS = 8;

        // Dynamics

        /// <summary>
        /// Maximum number of contacts to be handled to solve a TOI island.
        /// </summary>
        public const int MAX_TOI_CONTACTS = 32;

        /// <summary>
        /// A velocity threshold for elastic collisions. Any collision with a relative linear velocity
        /// below this threshold will be treated as inelastic.
        /// </summary>
        public const float VELOCITY_THRESHOLD = 1.0f;

        /// <summary>
        /// The maximum linear position correction used when solving constraints. This helps to prevent
        /// overshoot.
        /// </summary>
        public const float MAX_LINEAR_CORRECTION = 0.2f;

        /// <summary>
        /// The maximum angular position correction used when solving constraints. This helps to prevent
        /// overshoot.
        /// </summary>
        public static readonly float MAX_ANGULAR_CORRECTION = (8.0f / 180.0f * PI);

        /// <summary>
        /// The maximum linear velocity of a body. This limit is very large and is used to prevent
        /// numerical problems. You shouldn't need to adjust this.
        /// </summary>
        public const float MAX_TRANSLATION = 2.0f;

        public static readonly float MAX_TRANSLATION_SQUARED = (MAX_TRANSLATION * MAX_TRANSLATION);

        /// <summary>
        /// The maximum angular velocity of a body. This limit is very large and is used to prevent
        /// numerical problems. You shouldn't need to adjust this.
        /// </summary>
        public static readonly float MAX_ROTATION = (0.5f * PI);
        public static float MaxRotationSquared = (MAX_ROTATION * MAX_ROTATION);

        /// <summary>
        /// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so that
        /// overlap is removed in one time step. However using values close to 1 often lead to overshoot.
        /// </summary>
        public const float BAUMGARTE = 0.2f;
        public const float TOI_BAUGARTE = 0.75f;


        // Sleep

        /// <summary>
        /// The time that a body must be still before it will go to sleep.
        /// </summary>
        public const float TIME_TO_SLEEP = 0.5f;

        /// <summary>
        /// A body cannot sleep if its linear velocity is above this tolerance.
        /// </summary>
        public const float LINEAR_SLEEP_TOLERANCE = 0.01f;

        /// <summary>
        /// A body cannot sleep if its angular velocity is above this tolerance.
        /// </summary>
        public static readonly float ANGULAR_SLEEP_TOLERANCE = (2.0f / 180.0f * PI);

        /// <summary>
        /// Friction mixing law. Feel free to customize this. TODO djm: add customization
        /// </summary>
        /// <param name="friction1"></param>
        /// <param name="friction2"></param>
        /// <returns></returns>
        public static float MixFriction(float friction1, float friction2)
        {
            return MathUtils.Sqrt(friction1 * friction2);
        }

        /// <summary>
        /// Restitution mixing law. Feel free to customize this. TODO djm: add customization
        /// </summary>
        /// <param name="restitution1"></param>
        /// <param name="restitution2"></param>
        /// <returns></returns>
        public static float MixRestitution(float restitution1, float restitution2)
        {
            return restitution1 > restitution2 ? restitution1 : restitution2;
        }
    }
}