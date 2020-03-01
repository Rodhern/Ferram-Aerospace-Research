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
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation
{
    class InstantConditionSimIterationResult
    {
        public double stableCl;
        public double stableCd;
        public double stableCm;
        public double stablePitchValue;
        public double stableAoA;
        public string stableAoAState;

        public InstantConditionSimIterationResult(double Cl, double Cd, double Cm, double pitch, double AoA, string AoAState)
        {
            stableCl = Cl;
            stableCd = Cd;
            stableCm = Cm;
            stablePitchValue = pitch;
            stableAoA = AoA;
            stableAoAState = AoAState;
        }
    }


    class StabilityDerivOutput
    {
        public CelestialBody body;
        public double altitude;
        public double nominalVelocity;

        public double b;
        public double MAC;
        public double area;

        public InstantConditionSimIterationResult stableCondition;
        public double[] stabDerivs;

        public StabilityDerivOutput()
        {
            stableCondition = new InstantConditionSimIterationResult(0, 0, 0, 0, 0, "");
            stabDerivs = new double[27];
        }

        /// <summary>
        /// Creates a partially initialized StabilityDerivOutput object.
        /// Notice that the parameters are not cloned; the references are used as is.
        /// </summary>
        public StabilityDerivOutput(InstantConditionSimIterationResult stableCondition, double[] derivatives)
        {
            this.stableCondition = stableCondition;
            this.stabDerivs = derivatives;
        }
    }
}
