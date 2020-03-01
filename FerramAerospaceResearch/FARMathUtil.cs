/*
Ferram Aerospace Research v0.15.10.1 "Lundgren"
=========================
Aerodynamics model for Kerbal Space Program

Copyright 2019, Michael Ferrara, aka Ferram4

   This file is part of Ferram Aerospace Research.

   Ferram Aerospace Research is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Ferram Aerospace Research is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Ferram Aerospace Research.  If not, see <http://www.gnu.org/licenses/>.

   Serious thanks:		a.g., for tons of bugfixes and code-refactorings
				stupid_chris, for the RealChuteLite implementation
            			Taverius, for correcting a ton of incorrect values
				Tetryds, for finding lots of bugs and issues and not letting me get away with them, and work on example crafts
            			sarbian, for refactoring code for working with MechJeb, and the Module Manager updates
            			ialdabaoth (who is awesome), who originally created Module Manager
                        	Regex, for adding RPM support
				DaMichel, for some ferramGraph updates and some control surface-related features
            			Duxwing, for copy editing the readme

   CompatibilityChecker by Majiir, BSD 2-clause http://opensource.org/licenses/BSD-2-Clause

   Part.cfg changes powered by sarbian & ialdabaoth's ModuleManager plugin; used with permission
	http://forum.kerbalspaceprogram.com/threads/55219

   ModularFLightIntegrator by Sarbian, Starwaster and Ferram4, MIT: http://opensource.org/licenses/MIT
	http://forum.kerbalspaceprogram.com/threads/118088

   Toolbar integration powered by blizzy78's Toolbar plugin; used with permission
	http://forum.kerbalspaceprogram.com/threads/60863
 */

using System;
using UnityEngine;
using FerramAerospaceResearch.FARUtils;

namespace FerramAerospaceResearch
{
    public static class FARMathUtil
    {
        public const double rad2deg = 180d / Math.PI;
        public const double deg2rad = Math.PI / 180d;

        // ReSharper disable CompareOfFloatsByEqualityOperator
        public static bool NearlyEqual(this double a, double b, double epsilon = 1e-14)
        {
            if (a.Equals(b))
            {
                // shortcut, handles infinities
                return true;
            }

            double diff = Math.Abs(a - b);
            if (a == 0 || b == 0 || diff < double.Epsilon)
            {
                // a or b is zero or both are extremely close to it
                // relative error is less meaningful here
                return diff < epsilon * double.Epsilon;
            }

            // use relative error
            return diff / (Math.Abs(a) + Math.Abs(b)) < epsilon;
        }

        public static bool NearlyEqual(this float a, float b, float epsilon = 1e-6f)
        {
            if (a.Equals(b))
            {
                // shortcut, handles infinities
                return true;
            }

            float diff = Math.Abs(a - b);
            if (a == 0 || b == 0 || diff < float.Epsilon)
            {
                // a or b is zero or both are extremely close to it
                // relative error is less meaningful here
                return diff < epsilon * float.Epsilon;
            }

            // use relative error
            return diff / (Math.Abs(a) + Math.Abs(b)) < epsilon;
        }
        // ReSharper restore CompareOfFloatsByEqualityOperator

        public static double Lerp(double x1, double x2, double y1, double y2, double x)
        {
            double y = (y2 - y1) / (x2 - x1);
            y *= (x - x1);
            y += y1;
            return y;
        }

        public static T Clamp<T>(this T val, T min, T max) where T : IComparable<T>
        {
            if (val.CompareTo(min) < 0) return min;
            else if (val.CompareTo(max) > 0) return max;
            else return val;
        }

        public static bool Approximately(double p, double q)
        {
            if (Math.Abs(p - q) < double.Epsilon)
                return true;
            return false;
        }

        public static bool Approximately(double p, double q, double error)
        {
            if (Math.Abs(p - q) < error)
                return true;
            return false;
        }

        public static double ArithmeticGeometricMean(double a, double b, double error)
        {
            while (!Approximately(a, b, error))
            {
                double tmpA = 0.5 * (a + b);
                b = Math.Sqrt(a * b);
                a = tmpA;
            }
            return (a + b) * 0.5;
        }

        public static double ModifiedArithmeticGeometricMean(double a, double b, double error)
        {
            double c = 0;
            while (!Approximately(a, b, error))
            {
                double tmpA = 0.5 * (a + b);
                double tmpSqrt = Math.Sqrt((a - c) * (b - c));
                b = c + tmpSqrt;
                c = c - tmpSqrt;
                a = tmpA;
            }
            return (a + b) * 0.5;
        }

        public static double CompleteEllipticIntegralSecondKind(double k, double error)
        {
            double value = 2 * ArithmeticGeometricMean(1, k, error);
            value = Math.PI * ModifiedArithmeticGeometricMean(1, k * k, error) / value;

            return value;
        }

        //Approximation of Math.Pow(), as implemented here: http://martin.ankerl.com/2007/10/04/optimized-pow-approximation-for-java-and-c-c/
        public static double PowApprox(double a, double b)
        {
            int tmp = (int)(BitConverter.DoubleToInt64Bits(a) >> 32);
            int tmp2 = (int)(b * (tmp - 1072632447) + 1072632447);
            return BitConverter.Int64BitsToDouble(((long)tmp2) << 32);
        }

        public static double BrentsMethod(Func<double, double> function, double a, double b, double epsilon, int maxIter)
        {
            double delta = epsilon * 100;
            double fa, fb;
            fa = function(a);
            fb = function(b);

            if (fa * fb >= 0)
                return 0;

            if (Math.Abs(fa) < Math.Abs(fb))
            {
                double tmp = fa;
                fa = fb;
                fb = tmp;

                tmp = a;
                a = b;
                b = tmp;
            }

            double c = a, d = a, fc = function(c);

            double s = b, fs = fb;

            bool flag = true;
            int iter = 0;
            while (!fs.NearlyEqual(0) && Math.Abs(a - b) > epsilon && iter < maxIter)
            {
                if ((fa - fc) > double.Epsilon && (fb - fc) > double.Epsilon)    //inverse quadratic interpolation
                {
                    s = a * fc * fb / ((fa - fb) * (fa - fc));
                    s += b * fc * fa / ((fb - fa) * (fb - fc));
                    s += c * fc * fb / ((fc - fa) * (fc - fb));
                }
                else
                {
                    s = (b - a) / (fb - fa);    //secant method
                    s *= fb;
                    s = b - s;
                }

                double b_s = Math.Abs(b - s), b_c = Math.Abs(b - c), c_d = Math.Abs(c - d);

                //Conditions for bisection method
                bool condition1;
                double a3pb_over4 = (3 * a + b) * 0.25;

                if (a3pb_over4 > b)
                    if (s < a3pb_over4 && s > b)
                        condition1 = false;
                    else
                        condition1 = true;
                else
                    if (s > a3pb_over4 && s < b)
                        condition1 = false;
                    else
                        condition1 = true;

                bool condition2;

                if (flag && b_s >= b_c * 0.5)
                    condition2 = true;
                else
                    condition2 = false;

                bool condition3;

                if (!flag && b_s >= c_d * 0.5)
                    condition3 = true;
                else
                    condition3 = false;

                bool condition4;

                if (flag && b_c <= delta)
                    condition4 = true;
                else
                    condition4 = false;

                bool conditon5;

                if (!flag && c_d <= delta)
                    conditon5 = true;
                else
                    conditon5 = false;

                if (condition1 || condition2 || condition3 || condition4 || conditon5)
                {
                    s = a + b;
                    s *= 0.5;
                    flag = true;
                }
                else
                    flag = false;

                fs = function(s);
                d = c;
                c = b;

                if (fa * fs < 0)
                {
                    b = s;
                    fb = fs;
                }
                else
                {
                    a = s;
                    fa = fs;
                }

                if (Math.Abs(fa) < Math.Abs(fb))
                {
                    double tmp = fa;
                    fa = fb;
                    fb = tmp;

                    tmp = a;
                    a = b;
                    b = tmp;
                }
                iter++;
            }
            return s;
        }

        #region Segment Search Method
        public class IterationTolerances
        {
            /// <summary>
            /// By convention the argument search extent is named 'rightedge'.
            /// If the function is negative at zero the segment search method
            /// will check the interval ]0; rightedge].
            /// If the function is positive at zero the segment search method
            /// will check the interval [-rightedge; 0[.
            /// If the search must revert to Brent's method the search interval
            /// is [leftedge; rightedge].
            /// </summary>
            public double rightedge;

            /// <summary>
            /// The left edge of the argument interval used in case the search
            /// must revert to Brent's method.
            /// </summary>
            public double leftedge;

            /// <summary>
            /// The size of the first segment searched, e.g. [0deg; 5deg].
            /// </summary>
            public double xstepinitial;

            /// <summary>
            /// The size of subsequent segments searched.
            /// </summary>
            public double xstepsize;

            /// <summary>
            /// Parameters <paramref name="minpart"/> and
            /// <paramref name="maxpart"/> are used in the linear interpolation
            /// loop to avoid 'getting stuck'.
            /// </summary>
            public double minpart;

            /// <summary>
            /// Parameters <paramref name="minpart"/> and
            /// <paramref name="maxpart"/> are used in the linear interpolation
            /// loop to avoid 'getting stuck'.
            /// </summary>
            public double maxpart;

            /// <summary>
            /// When to give up the search for a (positive) maximum.
            /// </summary>
            public double tol_triangle;

            /// <summary>
            /// The argument precision used in linear interpolation.
            /// </summary>
            public double tol_linear;

            /// <summary>
            /// The iteration tolerance used in case the search must revert
            /// to Brent's method.
            /// </summary>
            public double tol_brent;

            /// <summary>
            /// Determines what to do if the segment search method fails.
            /// If <paramref name="allowbrent"/> is true the search reverts to
            /// FAR's Brent's method, meaning the search will start over using
            /// Brent's method, whereas if <paramref name="allowbrent"/> is
            /// false the search is simply abandoned and will return NaN.
            /// When <paramref name="allowbrent"/> is false, the other Brent
            /// parameters, i.e. 'leftedge', 'iterlim' and 'tol_brent', are not
            /// used.
            /// </summary>
            public bool allowbrent;

            /// <summary>
            /// The iteration limit used in case the search must revert to
            /// Brent's method.
            /// </summary>
            public int iterlim;

            /// <summary>
            /// The default iteration tolerances (from Rodhern's update 2).
            /// </summary>
            public IterationTolerances()
            {
                this.rightedge = 30d;
                this.leftedge = -rightedge;
                this.xstepinitial = 5d;
                this.xstepsize = 10d;
                this.minpart = 1d / 8d;
                this.maxpart = 7d / 8d;
                this.tol_triangle = 1E-3;
                this.tol_linear = 3E-4;
                this.tol_brent = 1E-3;
                this.iterlim = 500;
                this.allowbrent = true;
            }
        }

        public static double SegmentSearchMethod(Func<double, double> function, IterationTolerances tols)
        {
            double x0 = 0d;
            double f0 = function(x0);
            MirroredFunction mfobj = new MirroredFunction(function, f0 > 0d, tols);
            if (mfobj.IsMirrored) f0 = -f0;
            Func<double, double> f = mfobj.Delegate;
            double x1 = tols.xstepinitial;
            double f1 = f(x1);
            if (f1 < f0) return mfobj.BrentSolve("Negative initial gradient.");

        LblSkipRight:
            if (f1 > 0) return mfobj.LinearSolution(x0, f0, x1, f1);
            double x2 = Clamp<double>(x1 + tols.xstepsize, 0d, tols.rightedge);
            if (x2 == x1) return mfobj.BrentSolve("Reached far right edge.");
            double f2 = f(x2);
            if (f2 > f1)
            { // skip right
                x0 = x1; f0 = f1;
                x1 = x2; f1 = f2;
                goto LblSkipRight;
            }

        LblTriangle:
            if (f1 > 0) return mfobj.LinearSolution(x0, f0, x1, f1);
            if (x2 - x0 < tols.tol_triangle) return mfobj.BrentSolve("Local maximum is negative (search point x= " + x0 + ").");
            double x01 = (x0 + x1) / 2d;
            double x12 = (x1 + x2) / 2d;
            double f01 = f(x01);
            double f12 = f(x12);
            if (f01 >= f1 && f01 >= f12)
            { // maximum at x01
                x1 = x01; f1 = f01;
                x2 = x1; f2 = f1;
                goto LblTriangle;
            }
            else if (f12 > f1 && f12 > f01)
            { // maximum at x12
                x0 = x1; f0 = f1;
                x1 = x12; f1 = f12;
                goto LblTriangle;
            }
            else
            { // shrink around x1
                x0 = x01; f0 = f01;
                x2 = x12; f2 = f12;
                goto LblTriangle;
            }
        }

        public class MirroredFunction
        {
            private Func<double, double> F;
            private bool mirror;
            private IterationTolerances tols; // reference to iteration tolerances

            public MirroredFunction(Func<double, double> original, bool mirrored, IterationTolerances tolerances)
            {
                F = original;
                mirror = mirrored;
                tols = tolerances;
            }

            public Func<double, double> Delegate
            {
                get
                {
                    if (mirror)
                        return this.InvokeMirrored;
                    else
                        return this.F;
                }
            }

            public bool IsMirrored { get { return this.mirror; } }

            private double InvokeMirrored(double x)
            {
                return -F.Invoke(-x);
            }

            public double LinearSolution(double x0, double f0, double x1, double f1)
            {
                if (this.IsMirrored)
                {
                    double oldx0 = x0;
                    double oldf0 = f0;
                    x0 = -x1; f0 = -f1;
                    x1 = -oldx0; f1 = -oldf0;
                }

            LblLoop:
                double x = x0 + (x0 - x1) * f0 / (f1 - f0);
                if (x1 - x0 < tols.tol_linear) return x;
                x = Clamp<double>(x, tols.maxpart * x0 + tols.minpart * x1, tols.minpart * x0 + tols.maxpart * x1);
                double fx = F(x);
                if (fx < 0d)
                {
                    x0 = x; f0 = fx;
                    goto LblLoop;
                }
                else if (fx > 0d)
                {
                    x1 = x; f1 = fx;
                    goto LblLoop;
                }
                else
                    return x;
            }

            public double BrentSolve(string dbgmsg)
            {
                if (tols.allowbrent)
                {
                    Debug.Log("[Rodhern][FAR] MirroredFunction (mirrored= " + mirror + ") reverting to BrentsMethod: " + dbgmsg);
                    return FARMathUtil.BrentsMethod(this.F, tols.leftedge, tols.rightedge, tols.tol_brent, tols.iterlim);
                }
                else
                {
                    Debug.Log("[Rodhern][FAR] MirroredFunction (mirrored= " + mirror + ") abandoned search: " + dbgmsg);
                    return Double.NaN;
                }
            }
        }
        #endregion
    }
}
