using System;
using System.Collections.Generic;
using UnityEngine;
using ferram4;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation
{
    class InstantConditionSimVars
    {
        private InstantConditionSim parent;
        private InstantConditionSimInput iterationInput;
        private InstantConditionSimOutput iterationOutput;
        private FARMathUtil.IterationTolerances alphatol = AlphaTolerance();
        private FARMathUtil.IterationTolerances pitchtol = PitchTolerance();
        private double neededCl; // this is the target for level flight
        private Vector3d CoM;

        public InstantConditionSimVars(InstantConditionSim parent, CelestialBody body, double altitude, double machNumber, double neededCl, double beta, double phi, int flap, bool spoilers)
        {
            this.parent = parent;
            this.neededCl = neededCl;
            this.CoM = parent.GetCoM();
            FlightEnv fltenv = FlightEnv.NewSim(body, altitude, machNumber);
            iterationInput = new InstantConditionSimInput(0,beta,phi, 0,0,0, fltenv, 0, flap, spoilers);
            iterationOutput = new InstantConditionSimOutput();
        }

        public void ResetAndGetClCdCmSteady(InstantConditionSimInput input, out InstantConditionSimOutput output)
        {
            parent.ResetClCdCmSteady(CoM, input);
            parent.GetClCdCmSteady(input, out output, true, false);
        }

        public void IterateOnce(double alpha0, double pitch0, double alpha1, double pitch1, out double alpha2, out double pitch2)
        {
            iterationInput.alpha = alpha1;
            iterationInput.pitchValue = pitch0;
            parent.GetClCdCmSteady(iterationInput, out iterationOutput, true, true);
            double Cl10 = iterationOutput.Cl;
            double Cm10 = iterationOutput.Cm;

            iterationInput.alpha = alpha0;
            iterationInput.pitchValue = pitch1;
            parent.GetClCdCmSteady(iterationInput, out iterationOutput, true, true);
            double Cl01 = iterationOutput.Cl;
            double Cm01 = iterationOutput.Cm;

            iterationInput.alpha = alpha1;
            parent.GetClCdCmSteady(iterationInput, out iterationOutput, true, true);
            double Cl11 = iterationOutput.Cl;
            double Cm11 = iterationOutput.Cm;

            // Assume the solution did not yet converge

            double clgrad = (Cl11 - Cl01) / (alpha1 - alpha0);
            double cmcrss = (Cm11 - Cm01) / (alpha1 - alpha0);
            double cmgrad = (Cm11 - Cm10) / (pitch1 - pitch0);
            double clcrss = (Cl11 - Cl10) / (pitch1 - pitch0);
            double gradientscale = clgrad * cmgrad - clcrss * cmcrss;

            double cl = neededCl - Cl11; // for improved readability
            double cm = -Cm11;

            const double gradeps = 2E-5; // a gradient close to zero can be ignored
            double deltaalpha, deltapitch;
            if (Math.Abs(clgrad) > gradeps && Math.Abs(cmgrad) > gradeps
                && Math.Abs(gradientscale) > gradeps*gradeps)
            {
                deltaalpha = (cmgrad * cl - clcrss * cm) / gradientscale;
                deltapitch = (clgrad * cm - cmcrss * cl) / gradientscale;
            }
            else // update just the variable that have a non-zero gradient
            {
                if (Math.Abs(clgrad) > gradeps)
                    deltaalpha = cl / clgrad;
                else
                    deltaalpha = 0;

                if (Math.Abs(cmgrad) > gradeps)
                    deltapitch = cm / cmgrad;
                else
                    deltapitch = 0;
            }
            double maxdeltaalpha = Math.Abs(alpha1 - alpha0);
            double maxdeltapitch = Math.Abs(pitch1 - pitch0);
            deltaalpha = FARMathUtil.Clamp(deltaalpha, -maxdeltaalpha, +maxdeltaalpha);
            deltapitch = FARMathUtil.Clamp(deltapitch, -maxdeltapitch, +maxdeltapitch);
            alpha2 = FARMathUtil.Clamp(alpha1 + deltaalpha, alphatol.leftedge, alphatol.rightedge);
            pitch2 = FARMathUtil.Clamp(pitch1 + deltapitch, pitchtol.leftedge, pitchtol.rightedge);
        }

        public InstantConditionSimIterationResult IterateForAlphaAndPitch(out InstantConditionSimInput resultinput, out InstantConditionSimOutput resultoutput)
        {
            // reset 'old' calculations before first iteration
            parent.ResetClCdCmSteady(CoM, iterationInput);

            // level flight with yoke at neutral
            double alpha0 = FindAlphaForPitch(0);

            // stable attitude flight (at alpha0)
            double pitch0 = FindPitchForAlpha(alpha0);

            // level flight with deflected control surfaces
            double alpha1 = FindAlphaForPitch(pitch0);

            // updated stable attitude deflection
            double pitch1 = FindPitchForAlpha(alpha1);

            // iteration
            const int iterlim = 50;
            const double tolscale = 1 / 8; // scaled tolerance is stricter on the variable (alpha or pitch) to converge first
            int iterstep = 0;              //  in relation to when to exit two-dimensional iteration; ideally both variables
            while (iterstep < iterlim      //  have converged within tol_linear at that point.
                && Math.Abs(pitch1 - pitch0) > pitchtol.tol_linear * tolscale
                && Math.Abs(alpha1 - alpha0) > alphatol.tol_linear * tolscale)
            {
                double alpha2;
                double pitch2;
                IterateOnce(alpha0, pitch0, alpha1, pitch1, out alpha2, out pitch2);
                ++iterstep;
                alpha0 = alpha1; pitch0 = pitch1;
                alpha1 = alpha2; pitch1 = pitch2;
            }

            if (Math.Abs(pitch1 - pitch0) < pitchtol.tol_linear
                && Math.Abs(alpha1 - alpha0) < alphatol.tol_linear)
            { // ok, use alpha1 and pitch1 (i.e. the result of the last iteration)
            }
            else if (Math.Abs(pitch1 - pitch0) < pitchtol.tol_linear)
            { // we think we roughly know the stable attitude yoke position
                Debug.Log("[Rodhern][FAR] pitch determined (solve for alpha).");
                alpha1 = FindAlphaForPitch(pitch1);
            }
            else if (Math.Abs(alpha1 - alpha0) < alphatol.tol_linear)
            { // accept partial optimization
                Debug.Log("[Rodhern][FAR] partial solution (alpha ~= " + alpha1 + ").");
                pitch1 = FindPitchForAlpha(alpha1);
                alpha1 = FindAlphaForPitch(pitch1);
            }
            else
            { // level (but unstable) flight with yoke at neutral
                Debug.Log("[Rodhern][FAR] fix pitch at zero (last alpha was " + alpha1 + ").");
                pitch1 = 0;
                alpha1 = FindAlphaForPitch(pitch1);
            }

            if (Double.IsNaN(alpha1) || Double.IsNaN(pitch1))
            {
                alpha1 = 0;
                pitch1 = 0;
            }

            iterationInput.alpha = alpha1;
            iterationInput.pitchValue = pitch1;
            ResetAndGetClCdCmSteady(iterationInput, out iterationOutput);

            string AoAState;
            if (Math.Abs((iterationOutput.Cl - neededCl) / neededCl) < 0.01)
            {
                if (Math.Abs(iterationOutput.Cm) < 0.01)
                    AoAState = "";
                else
                    AoAState = (iterationOutput.Cm > 0) ? "\\" : "/";
            }
            else
            {
                AoAState = (iterationOutput.Cl > neededCl) ? "<" : ">";
            }
            Debug.Log("[Rodhern][FAR] Cl needed: " + neededCl + ", AoAState: '" + AoAState + "'," + " AoA: " + alpha1 + ", pitch: " + pitch1
                      + ", Cl: " + iterationOutput.Cl + ", Cd: " + iterationOutput.Cd + ", Cm: " + iterationOutput.Cm + ".");
            
            resultinput = iterationInput.Clone(); // clone so that we do not give away our private variable reference
            resultoutput = iterationOutput; // new iteration output values are made at each calculation, so we can do a simple struct assignment copy
            return new InstantConditionSimIterationResult(iterationOutput.Cl, iterationOutput.Cd, iterationOutput.Cm, pitch1, alpha1, AoAState);
        }

        private static FARMathUtil.IterationTolerances AlphaTolerance()
        {
            var tols = new FARMathUtil.IterationTolerances();
            tols.allowbrent = false;
            return tols;
        }

        private static FARMathUtil.IterationTolerances PitchTolerance()
        {
            var tols = new FARMathUtil.IterationTolerances();
            tols.leftedge = -1; tols.rightedge = 1;
            tols.minpart = 0.25; tols.maxpart = 0.75;
            tols.tol_triangle = 1E-2; tols.tol_linear = 2E-3;
            tols.xstepinitial = 0.15; tols.xstepsize = 0.35;
            tols.allowbrent = false;
            return tols;
        }

        public double FindAlphaForPitch(double pitch)
        {
            if (Double.IsNaN(pitch))
                return Double.NaN;
            else
            {
                iterationInput.pitchValue = pitch;
                return FARMathUtil.SegmentSearchMethod(this.FunctionIterateForAlpha, alphatol);
            }
        }

        public double FindPitchForAlpha(double alpha)
        {
            if (Double.IsNaN(alpha))
                return Double.NaN;
            else
            {
                iterationInput.alpha = alpha;
                return FARMathUtil.SegmentSearchMethod(this.FunctionIterateForPitch, pitchtol);
            }
        }

        private double FunctionIterateForAlpha(double alpha)
        {
            iterationInput.alpha = alpha;
            parent.GetClCdCmSteady(iterationInput, out iterationOutput, true, true);
            return iterationOutput.Cl - neededCl;
        }

        private double FunctionIterateForPitch(double pitch)
        {
            iterationInput.pitchValue = pitch;
            parent.GetClCdCmSteady(iterationInput, out iterationOutput, true, true);
            return iterationOutput.Cm;
        }

    }
}
