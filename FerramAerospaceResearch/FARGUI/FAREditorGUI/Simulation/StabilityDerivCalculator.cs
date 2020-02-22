/*
Ferram Aerospace Research v0.15.9.5 "Lighthill"
=========================
Aerodynamics model for Kerbal Space Program

Copyright 2017, Michael Ferrara, aka Ferram4

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
using System.Collections.Generic;
using ferram4;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation
{
    class StabilityDerivCalculator
    {
        InstantConditionSim _instantCondition;

        public StabilityDerivCalculator(InstantConditionSim instantConditionSim)
        {
            _instantCondition = instantConditionSim;
        }

        public StabilityDerivExportOutput CalculateStabilityDerivs(CelestialBody body, double alt, double machNumber, int flapSetting, bool spoilers)
        {
            if (body.GetPressure(alt) > 0)
                return CalculateStabilityDerivs(body, alt, machNumber, flapSetting, spoilers, 0, 0);
            else
                return null;
        }

        public StabilityDerivExportOutput CalculateStabilityDerivs(CelestialBody body, double alt, double machNumber, int flapSetting, bool spoilers, double beta, double phi)
        {
            double pressure = body.GetPressure(alt);
            double temperature = body.GetTemperature(alt);
            double density = body.GetDensity(pressure, temperature);
            double sspeed = body.GetSpeedOfSound(pressure, density);
            double u0 = sspeed * machNumber;
            double q = u0 * u0 * density * 0.5f;

            Vector3d CoM;
            double mass, area, MAC, b;
            _instantCondition.GetCoMAndSize(out CoM, out mass, out area, out MAC, out b);

            double effectiveG = _instantCondition.CalculateEffectiveGravity(body, alt, u0);
            double neededCl = effectiveG * mass / (q * area);

            InstantConditionSimVars iterationSimVars =
                new InstantConditionSimVars(_instantCondition, body, alt, machNumber, neededCl, beta, phi, flapSetting, spoilers);
            InstantConditionSimInput nominalInput;
            InstantConditionSimOutput nominalOutput;
            InstantConditionSimIterationResult stableCondition =
                iterationSimVars.IterateForAlphaAndPitch(out nominalInput, out nominalOutput);

            InstantConditionSimInput input = nominalInput.Clone();
            InstantConditionSimOutput pertOutput;
            double[] derivatives = new double[27];

            // update size (in practice MAC and b) to match stableCondition
            _instantCondition.GetCoMAndSize(out CoM, out mass, out area, out MAC, out b);

            double Ix, Iy, Iz;
            double Ixy, Iyz, Ixz;
            _instantCondition.GetInertia(CoM, out Ix, out Iy, out Iz, out Ixy, out Iyz, out Ixz);


            input.alpha = stableCondition.stableAoA + 2;
            iterationSimVars.ResetAndGetClCdCmSteady(input, out pertOutput);

            pertOutput.Cl = (pertOutput.Cl - nominalOutput.Cl) / (2 * FARMathUtil.deg2rad);                   //vert vel derivs
            pertOutput.Cd = (pertOutput.Cd - nominalOutput.Cd) / (2 * FARMathUtil.deg2rad);
            pertOutput.Cm = (pertOutput.Cm - nominalOutput.Cm) / (2 * FARMathUtil.deg2rad);

            pertOutput.Cl += nominalOutput.Cd;
            pertOutput.Cd -= nominalOutput.Cl;

            pertOutput.Cl *= -q * area / (mass * u0);
            pertOutput.Cd *= -q * area / (mass * u0);
            pertOutput.Cm *= q * area * MAC / (Iy * u0);

            derivatives[3] = pertOutput.Cl;  //Zw
            derivatives[4] = pertOutput.Cd;  //Xw
            derivatives[5] = pertOutput.Cm;  //Mw


            input.alpha = stableCondition.stableAoA;
            input.fltenv.MachNumber = machNumber + 0.05;
            iterationSimVars.ResetAndGetClCdCmSteady(input, out pertOutput);

            pertOutput.Cl = (pertOutput.Cl - nominalOutput.Cl) / 0.05 * machNumber;                   //fwd vel derivs
            pertOutput.Cd = (pertOutput.Cd - nominalOutput.Cd) / 0.05 * machNumber;
            pertOutput.Cm = (pertOutput.Cm - nominalOutput.Cm) / 0.05 * machNumber;

            pertOutput.Cl += 2 * nominalOutput.Cl;
            pertOutput.Cd += 2 * nominalOutput.Cd;
            pertOutput.Cm += 2 * nominalOutput.Cm;

            pertOutput.Cl *= -q * area / (mass * u0);
            pertOutput.Cd *= -q * area / (mass * u0);
            pertOutput.Cm *= q * area * MAC / (Iy * u0);

            derivatives[6] = pertOutput.Cl;  //Zu
            derivatives[7] = pertOutput.Cd;  //Xu
            derivatives[8] = pertOutput.Cm;  //Mu


            input.fltenv.MachNumber = machNumber;
            input.alphaDot = -3;
            iterationSimVars.ResetAndGetClCdCmSteady(input, out pertOutput);

            pertOutput.Cl = (pertOutput.Cl - nominalOutput.Cl) / (3 * FARMathUtil.deg2rad);                   //pitch rate derivs
            pertOutput.Cd = (pertOutput.Cd - nominalOutput.Cd) / (3 * FARMathUtil.deg2rad);
            pertOutput.Cm = (pertOutput.Cm - nominalOutput.Cm) / (3 * FARMathUtil.deg2rad);

            pertOutput.Cl *= -q * area / mass;
            pertOutput.Cd *= -q * area / mass;
            pertOutput.Cm *= q * area * MAC / Iy;

            derivatives[9] = pertOutput.Cl;  //Zq
            derivatives[10] = pertOutput.Cd; //Xq
            derivatives[11] = pertOutput.Cm; //Mq


            input.alphaDot = 0;
            double pitchDelta = (stableCondition.stablePitchValue > 0) ? -0.1 : 0.1;
            input.pitchValue = stableCondition.stablePitchValue + pitchDelta;
            iterationSimVars.ResetAndGetClCdCmSteady(input, out pertOutput);

            pertOutput.Cl = (pertOutput.Cl - nominalOutput.Cl) / pitchDelta;                   //elevator derivs
            pertOutput.Cd = (pertOutput.Cd - nominalOutput.Cd) / pitchDelta;
            pertOutput.Cm = (pertOutput.Cm - nominalOutput.Cm) / pitchDelta;

            pertOutput.Cl *= -q * area / mass; // Rodhern: Replaced 'q' by '-q', so that formulas
            pertOutput.Cd *= -q * area / mass; //  for Ze and Xe match those for Zu and Xu.
            pertOutput.Cm *= q * area * MAC / Iy;

            derivatives[12] = pertOutput.Cl; //Ze
            derivatives[13] = pertOutput.Cd; //Xe
            derivatives[14] = pertOutput.Cm; //Me


            input.pitchValue = stableCondition.stablePitchValue;
            input.beta = (beta + 2);
            iterationSimVars.ResetAndGetClCdCmSteady(input, out pertOutput);

            pertOutput.Cy = (pertOutput.Cy - nominalOutput.Cy) / (2 * FARMathUtil.deg2rad);                   //sideslip angle derivs
            pertOutput.Cn = (pertOutput.Cn - nominalOutput.Cn) / (2 * FARMathUtil.deg2rad);
            pertOutput.C_roll = (pertOutput.C_roll - nominalOutput.C_roll) / (2 * FARMathUtil.deg2rad);

            pertOutput.Cy *= q * area / mass;
            pertOutput.Cn *= q * area * b / Iz;
            pertOutput.C_roll *= q * area * b / Ix;

            derivatives[15] = pertOutput.Cy;     //Yb
            derivatives[17] = pertOutput.Cn;     //Nb
            derivatives[16] = pertOutput.C_roll; //Lb


            input.beta = beta;
            input.phiDot = -3;
            iterationSimVars.ResetAndGetClCdCmSteady(input, out pertOutput);

            pertOutput.Cy = (pertOutput.Cy - nominalOutput.Cy) / (3 * FARMathUtil.deg2rad);                   //roll rate derivs
            pertOutput.Cn = (pertOutput.Cn - nominalOutput.Cn) / (3 * FARMathUtil.deg2rad);
            pertOutput.C_roll = (pertOutput.C_roll - nominalOutput.C_roll) / (3 * FARMathUtil.deg2rad);

            pertOutput.Cy *= q * area / mass;
            pertOutput.Cn *= q * area * b / Iz;
            pertOutput.C_roll *= q * area * b / Ix;

            derivatives[18] = pertOutput.Cy;     //Yp
            derivatives[20] = pertOutput.Cn;     //Np
            derivatives[19] = pertOutput.C_roll; //Lp


            input.phiDot = 0;
            input.betaDot = -3;
            iterationSimVars.ResetAndGetClCdCmSteady(input, out pertOutput);

            pertOutput.Cy = (pertOutput.Cy - nominalOutput.Cy) / (3 * FARMathUtil.deg2rad);                   //yaw rate derivs
            pertOutput.Cn = (pertOutput.Cn - nominalOutput.Cn) / (3 * FARMathUtil.deg2rad);
            pertOutput.C_roll = (pertOutput.C_roll - nominalOutput.C_roll) / (3 * FARMathUtil.deg2rad);

            pertOutput.Cy *= q * area / mass;
            pertOutput.Cn *= q * area * b / Iz;
            pertOutput.C_roll *= q * area * b / Ix;

            derivatives[21] = pertOutput.Cy;     //Yr
            derivatives[23] = pertOutput.Cn;     //Nr
            derivatives[22] = pertOutput.C_roll; //Lr


            input = new InstantConditionSimInput(body); // Reset to (an artificial) default condition
            _instantCondition.ResetClCdCmSteady(CoM, input);

            // Assign values to output variables
            StabilityDerivOutput stabDerivOutput = new StabilityDerivOutput(stableCondition, derivatives);
            stabDerivOutput.nominalVelocity = u0;
            stabDerivOutput.altitude = alt;
            stabDerivOutput.body = body;
            stabDerivOutput.b = b;
            stabDerivOutput.MAC = MAC;
            stabDerivOutput.area = area;
            stabDerivOutput.stabDerivs[0] = Ix;
            stabDerivOutput.stabDerivs[1] = Iy;
            stabDerivOutput.stabDerivs[2] = Iz;
            stabDerivOutput.stabDerivs[24] = Ixy;
            stabDerivOutput.stabDerivs[25] = Iyz;
            stabDerivOutput.stabDerivs[26] = Ixz;

            // Assign values to export variables
            StabilityDerivExportVariables stabDerivExport = new StabilityDerivExportVariables();
            stabDerivExport.craftmass = mass;
            stabDerivExport.envpressure = pressure;
            stabDerivExport.envtemperature = temperature;
            stabDerivExport.envdensity = density;
            stabDerivExport.envsoundspeed = sspeed;
            stabDerivExport.envg = _instantCondition.CalculateAccelerationDueToGravity(body, alt);
            stabDerivExport.sitmach = machNumber;
            stabDerivExport.sitdynpres = q;
            stabDerivExport.siteffg = _instantCondition.CalculateEffectiveGravity(body, alt, u0);

            return new StabilityDerivExportOutput(stabDerivOutput, stabDerivExport);
        }

    }
}
