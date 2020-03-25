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
using UnityEngine;
using FerramAerospaceResearch.FARUtils;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation
{
    class StabilityDerivLinearSim
    {
        InstantConditionSim _instantCondition;

        public StabilityDerivLinearSim(InstantConditionSim instantConditionSim)
        {
            _instantCondition = instantConditionSim;
        }

        public GraphData RunTransientSimLateral(StabilityDerivOutput vehicleData, double endTime, double initDt, double[] InitCond)
        {
            SimMatrix A = new SimMatrix(4, 4);

            int i = 0;
            int j = 0;
            double[] Derivs = new double[27];

            vehicleData.stabDerivs.CopyTo(Derivs, 0);

            double u0 = vehicleData.nominalVelocity;
            double b2u = vehicleData.b / (2 * u0);
            double effg = _instantCondition.CalculateEffectiveGravity(vehicleData.body, vehicleData.altitude, u0) * Math.Cos(vehicleData.stableCondition.stableAoA * Math.PI / 180);
            double factor_xz_x = Derivs[26] / Derivs[0];
            double factor_xz_z = Derivs[26] / Derivs[2];
            double factor_invxz = 1 / (1 - factor_xz_x * factor_xz_z);

            FARLogger.Info("u0= " + u0);
            FARLogger.Info("b/(2u)= " + b2u + " IGNORED!");
            FARLogger.Info("effg= " + effg + ", after multiplication with cos(AoA).");
            FARLogger.Info("Ixz/Ix= " + factor_xz_x + ", used to add yaw to roll-deriv.");
            FARLogger.Info("Ixz/Iz= " + factor_xz_z + ", used to add roll to yaw-deriv.");
            FARLogger.Info("(1 - Ixz^2/(IxIz))^-1= " + factor_invxz);

            // Rodhern: For possible backward compability the rotation (moment) derivatives can be
            //  scaled by "b/(2u)" (for pitch rate "mac/(2u)").
            //for (int h = 18; h <= 23; h++)
            //    Derivs[h] = Derivs[h] * b2u;

            Derivs[15] = Derivs[15] / u0;
            Derivs[18] = Derivs[18] / u0;
            Derivs[21] = Derivs[21] / u0 - 1;

            double Lb = Derivs[16] * factor_invxz;
            double Nb = Derivs[17] * factor_invxz;

            double Lp = Derivs[19] * factor_invxz;
            double Np = Derivs[20] * factor_invxz;

            double Lr = Derivs[22] * factor_invxz;
            double Nr = Derivs[23] * factor_invxz;

            Derivs[16] = Lb + factor_xz_x * Nb;
            Derivs[17] = Nb + factor_xz_z * Lb;

            Derivs[19] = Lp + factor_xz_x * Np;
            Derivs[20] = Np + factor_xz_z * Lp;

            Derivs[22] = Lr + factor_xz_x * Nr;
            Derivs[23] = Nr + factor_xz_z * Lr;

            for (int k = 15; k < Derivs.Length; k++)
            {
                double f = Derivs[k];

                if (i <= 2)
                {
                    FARLogger.Info("A[" + i + "," + j + "]= f_" + k + " = " + f + ", after manipulation.");
                    A.Add(f, i, j);
                }

                if (j < 2)
                    j++;
                else
                {
                    j = 0;
                    i++;
                }
            }
            A.Add(effg / u0, 3, 0);
            A.Add(1, 1, 3);

            A.PrintToConsole();                //We should have an array that looks like this:

            /*             i --------------->
             *       j  [ Yb / u0 , Yp / u0 , -(1 - Yr/ u0) ,  g Cos(θ0) / u0 ]
             *       |  [   Lb    ,    Lp   ,      Lr       ,          0          ]
             *       |  [   Nb    ,    Np   ,      Nr       ,          0          ]
             *      \ / [    0    ,    1    ,      0        ,          0          ]
             *       V  
             */

            RungeKutta4 transSolve = new RungeKutta4(endTime, initDt, A, InitCond);
            transSolve.Solve();

            GraphData lines = new GraphData();
            lines.xValues = transSolve.time;

            double[] yVal = transSolve.GetSolution(0);
            ScaleAndClampValues(yVal, 180 / Math.PI, 50);
            lines.AddData(yVal, GUIColors.GetColor(3), "β", true);

            yVal = transSolve.GetSolution(1);
            ScaleAndClampValues(yVal, 180 / Math.PI, 50);
            lines.AddData(yVal, GUIColors.GetColor(2), "p", true);

            yVal = transSolve.GetSolution(2);
            ScaleAndClampValues(yVal, 180 / Math.PI, 50);
            lines.AddData(yVal, GUIColors.GetColor(1), "r", true);

            yVal = transSolve.GetSolution(3);
            ScaleAndClampValues(yVal, 180 / Math.PI, 50);
            lines.AddData(yVal, GUIColors.GetColor(0), "φ", true);

            /*graph.SetBoundaries(0, endTime, -10, 10);
            graph.SetGridScaleUsingValues(1, 5);
            graph.horizontalLabel = "time";
            graph.verticalLabel = "value";
            graph.Update();*/

            return lines;
        }

        public GraphData RunTransientSimLongitudinal(StabilityDerivOutput vehicleData, double endTime, double initDt, double[] InitCond)
        {
            SimMatrix A = new SimMatrix(4, 4);

            int i = 0;
            int j = 0;
            double[] Derivs = new double[27];

            vehicleData.stabDerivs.CopyTo(Derivs, 0);

            double MAC2u = vehicleData.MAC / (2 * vehicleData.nominalVelocity);
            double effg = _instantCondition.CalculateEffectiveGravity(vehicleData.body, vehicleData.altitude, vehicleData.nominalVelocity);

            FARLogger.Info("MAC/(2u)= " + MAC2u + " IGNORED!");
            FARLogger.Info("effg= " + effg);

            // Rodhern: For possible backward compability the rotation (moment) derivatives can be
            //  scaled by "mac/(2u)" (pitch) and "b/(2u)" (roll and yaw).
            //for (int h = 9; h <= 11; h++)
            //    Derivs[h] = Derivs[h] * MAC2u;

            Derivs[9] = Derivs[9] + vehicleData.nominalVelocity;

            for (int k = 3; k < 15 && k < Derivs.Length; k++)
            {
                double f = Derivs[k];

                if (i <= 2)
                {
                    FARLogger.Info("A[" + i + "," + j + "]= f_" + k + " = " + f);
                    A.Add(f, i, j);
                }
                else
                    FARLogger.Debug("Ignore B[0," + j + "]= " + f);

                if (j < 2)
                    j++;
                else
                {
                    j = 0;
                    i++;
                }
            }
            A.Add(-effg, 3, 1);
            A.Add(1, 2, 3);

            A.PrintToConsole();                //We should have an array that looks like this:

            /*            i --------------->
             *       j  [ Z w , Z u , Z q  + u,  0 ]
             *       |  [ X w , X u , X q     , -g ]
             *       |  [ M w , M u , M q     ,  0 ]
             *      \ / [  0  ,  0  ,  1      ,  0 ]
             *       V  
             */
                                               //And one that looks like this: (Unused)
            /*
             *          [ Z e ]
             *          [ X e ]
             *          [ M e ]
             *          [  0  ]
             *
             */

            RungeKutta4 transSolve = new RungeKutta4(endTime, initDt, A, InitCond);
            transSolve.Solve();

            GraphData lines = new GraphData();
            lines.xValues = transSolve.time;

            double[] yVal = transSolve.GetSolution(0);
            ScaleAndClampValues(yVal, 1, 50);
            lines.AddData(yVal, GUIColors.GetColor(3), "w", true);

            yVal = transSolve.GetSolution(1);
            ScaleAndClampValues(yVal, 1, 50);
            lines.AddData(yVal, GUIColors.GetColor(2), "u", true);

            yVal = transSolve.GetSolution(2);
            ScaleAndClampValues(yVal, 180 / Math.PI, 50);
            lines.AddData(yVal, GUIColors.GetColor(1), "q", true);

            yVal = transSolve.GetSolution(3);
            ScaleAndClampValues(yVal, 180 / Math.PI, 50);
            lines.AddData(yVal, GUIColors.GetColor(0), "θ", true);

            /*graph.SetBoundaries(0, endTime, -10, 10);
            graph.SetGridScaleUsingValues(1, 5);
            graph.horizontalLabel = "time";
            graph.verticalLabel = "value";
            graph.Update();*/

            return lines;
        }

        private void ScaleAndClampValues(double[] yVal, double scalingFactor, double clampValue)
        {
            for (int k = 0; k < yVal.Length; k++)
            {
                yVal[k] = yVal[k] * scalingFactor;
                if (yVal[k] > clampValue)
                    yVal[k] = clampValue;
                else if (yVal[k] < -clampValue)
                    yVal[k] = -clampValue;
            }
        }
    }
}
