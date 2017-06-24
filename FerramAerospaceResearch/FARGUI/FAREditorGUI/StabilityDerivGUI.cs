﻿/*
Ferram Aerospace Research v0.15.8.1 "Lewis"
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
using System.Text.RegularExpressions;
using UnityEngine;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using ferram4;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI
{
    class StabilityDerivGUI
    {

        GUIDropDown<int> _flapSettingDropdown;
        GUIDropDown<CelestialBody> _bodySettingDropdown;

        StabilityDerivOutput stabDerivOutput;

        string altitude = "0";
        string machNumber = "0.35";
        bool spoilersDeployed = false;

        EditorSimManager simManager;

        Vector3 aoAVec;

        public StabilityDerivGUI(EditorSimManager simManager, GUIDropDown<int> flapSettingDropDown, GUIDropDown<CelestialBody> bodySettingDropdown)
        {
            this.simManager = simManager;
            _flapSettingDropdown = flapSettingDropDown;
            _bodySettingDropdown = bodySettingDropdown;

            stabDerivOutput = new StabilityDerivOutput();
        }

        public void ArrowAnim(ArrowPointer velArrow)
        {
            velArrow.Direction = -aoAVec;
            //Debug.Log(velArrow.Direction);
        }

        void SetAngleVectors(double aoA)
        {
            aoA *= FARMathUtil.deg2rad;

            if (EditorDriver.editorFacility == EditorFacility.SPH)
            {
                aoAVec = new Vector3d(0, -Math.Sin(aoA), Math.Cos(aoA));
            }
            else
            {
                aoAVec = new Vector3d(0, Math.Cos(aoA), Math.Sin(aoA));
            }
        }

        public void Display()
        {
            const int W160 = 160 + 20; // Rodhern: A width originally designed to be of size 160.
            //stabDerivHelp = GUILayout.Toggle(stabDerivHelp, "?", ButtonStyle, GUILayout.Width(200));

            GUILayout.Label("Flight Condition:");
            GUILayout.BeginHorizontal();
            GUILayout.Label("Planet:");
            _bodySettingDropdown.GUIDropDownDisplay();

            GUILayout.Label("Altitude (km):");
            altitude = GUILayout.TextField(altitude, GUILayout.ExpandWidth(true));

            GUILayout.Label("Mach Number: ");
            machNumber = GUILayout.TextField(machNumber, GUILayout.ExpandWidth(true));

            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Flap Setting: ");
            _flapSettingDropdown.GUIDropDownDisplay();
            GUILayout.Label("Spoilers:");
            spoilersDeployed = GUILayout.Toggle(spoilersDeployed, spoilersDeployed ? "Deployed" : "Retracted", GUILayout.Width(100));
            GUILayout.EndHorizontal();

            if (GUILayout.Button("Calculate Stability Derivatives", GUILayout.Width(250.0F), GUILayout.Height(25.0F)))
            {
                CelestialBody body = _bodySettingDropdown.ActiveSelection;
                FARAeroUtil.UpdateCurrentActiveBody(body);
                //atm_temp_str = Regex.Replace(atm_temp_str, @"[^-?[0-9]*(\.[0-9]*)?]", "");
                //rho_str = Regex.Replace(rho_str, @"[^-?[0-9]*(\.[0-9]*)?]", "");
                machNumber = Regex.Replace(machNumber, @"[^-?[0-9]*(\.[0-9]*)?]", "");

                altitude = Regex.Replace(altitude, @"[^-?[0-9]*(\.[0-9]*)?]", "");
                double altitudeDouble = Convert.ToDouble(altitude);
                altitudeDouble *= 1000;


                double temp = body.GetTemperature(altitudeDouble);
                double pressure = body.GetPressure(altitudeDouble);
                if (pressure > 0)
                {
                    //double temp = Convert.ToSingle(atm_temp_str);
                    double machDouble = Convert.ToSingle(machNumber);
                    machDouble = FARMathUtil.Clamp(machDouble, 0.001, float.PositiveInfinity);

                    double density = body.GetDensity(pressure, temp);

                    double sspeed = body.GetSpeedOfSound(pressure, density);
                    double vel = sspeed * machDouble;

                    //UpdateControlSettings();

                    double q = vel * vel * density * 0.5f;

                    stabDerivOutput = simManager.StabDerivCalculator.CalculateStabilityDerivs(vel, q, machDouble, 0, 0, 0, _flapSettingDropdown.ActiveSelection, spoilersDeployed, body, altitudeDouble);
                    simManager.vehicleData = stabDerivOutput;
                    SetAngleVectors(stabDerivOutput.stableAoA);
                }
                else
                {
                    PopupDialog.SpawnPopupDialog(new Vector2(0, 0), new Vector2(0, 0), "Error!", "Altitude was above atmosphere", "OK", true, HighLogic.UISkin);
                }
            }

            GUILayout.BeginHorizontal();
            GUILayout.Label("Aircraft Properties", GUILayout.Width(180));
            GUILayout.Label("Moments of Inertia", GUILayout.Width(W160));
            GUILayout.Label("Products of Inertia", GUILayout.Width(W160));
            GUILayout.Label("Level Flight", GUILayout.Width(140));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.BeginVertical(GUILayout.Width(180));
            GUILayout.Label("Ref Area: " + stabDerivOutput.area.ToString("G3") + " m²");
            GUILayout.Label("Scaled Chord: " + stabDerivOutput.MAC.ToString("G3") + " m");
            GUILayout.Label("Scaled Span: " + stabDerivOutput.b.ToString("G3") + " m");
            GUILayout.EndVertical();


            GUILayout.BeginVertical(GUILayout.Width(W160));
            GUILayout.Label(new GUIContent("Ixx: " + stabDerivOutput.stabDerivs[0].ToString("G6") + " kg * m²", "Inertia about X-axis due to rotation about X-axis"));
            GUILayout.Label(new GUIContent("Iyy: " + stabDerivOutput.stabDerivs[1].ToString("G6") + " kg * m²", "Inertia about Y-axis due to rotation about Y-axis"));
            GUILayout.Label(new GUIContent("Izz: " + stabDerivOutput.stabDerivs[2].ToString("G6") + " kg * m²", "Inertia about Z-axis due to rotation about Z-axis"));
            GUILayout.EndVertical();

            GUILayout.BeginVertical(GUILayout.Width(W160));
            GUILayout.Label(new GUIContent("Ixy: " + stabDerivOutput.stabDerivs[24].ToString("G6") + " kg * m²", "Inertia about X-axis due to rotation about Y-axis; is equal to inertia about Y-axis due to rotation about X-axis"));
            GUILayout.Label(new GUIContent("Iyz: " + stabDerivOutput.stabDerivs[25].ToString("G6") + " kg * m²", "Inertia about Y-axis due to rotation about Z-axis; is equal to inertia about Z-axis due to rotation about Y-axis"));
            GUILayout.Label(new GUIContent("Ixz: " + stabDerivOutput.stabDerivs[26].ToString("G6") + " kg * m²", "Inertia about X-axis due to rotation about Z-axis; is equal to inertia about Z-axis due to rotation about X-axis"));
            GUILayout.EndVertical();

            GUILayout.BeginVertical(GUILayout.Width(140));
            GUILayout.Label(new GUIContent("u0: " + stabDerivOutput.nominalVelocity.ToString("G6") + " m/s", "Air speed based on this mach number and temperature."));
            GUILayout.BeginHorizontal();
            GUILayout.Label(new GUIContent("Cl: " + stabDerivOutput.stableCl.ToString("G3"), "Required lift coefficient at this mass, speed and air density."));
            GUILayout.Label(new GUIContent("Cd: " + stabDerivOutput.stableCd.ToString("G3"), "Resulting drag coefficient at this mass, speed and air density."));
            GUILayout.EndHorizontal();
            GUILayout.Label(new GUIContent("AoA: " + stabDerivOutput.stableAoAState + stabDerivOutput.stableAoA.ToString("G6") + " deg", "Angle of attack required to achieve the necessary lift force."));
            GUILayout.EndVertical();

            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Longitudinal Derivatives", GUILayout.Width(W160));
            GUILayout.EndHorizontal();

            GUIStyle BackgroundStyle = new GUIStyle(GUI.skin.box);
            BackgroundStyle.hover = BackgroundStyle.active = BackgroundStyle.normal;

            GUILayout.BeginVertical(BackgroundStyle);
            GUILayout.BeginHorizontal();
            GUILayout.Label("Down Vel Derivatives", GUILayout.Width(W160));
            GUILayout.Label("Fwd Vel Derivatives", GUILayout.Width(W160));
            GUILayout.Label("Pitch Rate Derivatives", GUILayout.Width(W160));
            GUILayout.Label("Pitch Ctrl Derivatives", GUILayout.Width(W160));
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            StabilityLabel("Zw: ", stabDerivOutput.stabDerivs[3], " s⁻¹", "Change in Z-direction acceleration with respect to Z-direction velocity; should be negative", W160, -1);
            StabilityLabel("Zu: ", stabDerivOutput.stabDerivs[6], " s⁻¹", "Change in Z-direction acceleration with respect to X-direction velocity; should be negative", W160, -1);
            StabilityLabel("Zq: ", stabDerivOutput.stabDerivs[9], " m/s", "Change in Z-direction acceleration with respect to pitch-up rate; sign unimportant", W160, 0);
            StabilityLabel("Zδe: ", stabDerivOutput.stabDerivs[12], " m/s²", "Change in Z-direction acceleration with respect to pitch control input; should be negative", W160, 0);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            StabilityLabel("Xw: ", stabDerivOutput.stabDerivs[4], " s⁻¹", "Change in X-direction acceleration with respect to Z-direction velocity; sign unimportant", W160, 0);
            StabilityLabel("Xu: ", stabDerivOutput.stabDerivs[7], " s⁻¹", "Change in X-direction acceleration with respect to X-direction velocity; should be negative", W160, -1);
            StabilityLabel("Xq: ", stabDerivOutput.stabDerivs[10], " m/s", "Change in X-direction acceleration with respect to pitch-up rate; sign unimportant", W160, 0);
            StabilityLabel("Xδe: ", stabDerivOutput.stabDerivs[13], " m/s²", "Change in X-direction acceleration with respect to pitch control input; sign unimportant", W160, 0);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            StabilityLabel("Mw: ", stabDerivOutput.stabDerivs[5], " (m * s)⁻¹", "Change in pitch-up angular acceleration with respect to Z-direction velocity; should be negative", W160, -1);
            StabilityLabel("Mu: ", stabDerivOutput.stabDerivs[8], " (m * s)⁻¹", "Change in pitch-up angular acceleration acceleration with respect to X-direction velocity; sign unimportant", W160, 0);
            StabilityLabel("Mq: ", stabDerivOutput.stabDerivs[11], " s⁻¹", "Change in pitch-up angular acceleration acceleration with respect to pitch-up rate; should be negative", W160, -1);
            StabilityLabel("Mδe: ", stabDerivOutput.stabDerivs[14], " s⁻²", "Change in pitch-up angular acceleration acceleration with respect to pitch control input; should be positive", W160, 1);
            GUILayout.EndHorizontal();
            GUILayout.EndVertical();

            GUILayout.BeginHorizontal();
            GUILayout.Label("Lateral Derivatives", GUILayout.Width(W160));
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            GUILayout.Label("Sideslip Derivatives", GUILayout.Width(W160));
            GUILayout.Label("Roll Rate Derivatives", GUILayout.Width(W160));
            GUILayout.Label("Yaw Rate Derivatives", GUILayout.Width(W160));
            GUILayout.EndHorizontal();
            GUILayout.BeginVertical(BackgroundStyle);
            GUILayout.BeginHorizontal();
            StabilityLabel("Yβ: ", stabDerivOutput.stabDerivs[15], " m/s²", "Change in Y-direction acceleration with respect to sideslip angle β; should be negative", W160, -1);
            StabilityLabel("Yp: ", stabDerivOutput.stabDerivs[18], " m/s", "Change in Y-direction acceleration with respect to roll-right rate; sign unimportant", W160, 0);
            StabilityLabel("Yr: ", stabDerivOutput.stabDerivs[21], " m/s", "Change in Y-direction acceleration with respect to yaw-right rate; should be positive", W160, 1);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            StabilityLabel("Lβ: ", stabDerivOutput.stabDerivs[16], " s⁻²", "Change in roll-right angular acceleration with respect to sideslip angle β; should be negative", W160, -1);
            StabilityLabel("Lp: ", stabDerivOutput.stabDerivs[19], " s⁻¹", "Change in roll-right angular acceleration with respect to roll-right rate; should be negative", W160, -1);
            StabilityLabel("Lr: ", stabDerivOutput.stabDerivs[22], " s⁻¹", "Change in roll-right angular acceleration with respect to yaw-right rate; should be positive", W160, 1);
            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();
            StabilityLabel("Nβ: ", stabDerivOutput.stabDerivs[17], " s⁻²", "Change in yaw-right angular acceleration with respect to sideslip angle β; should be positive", W160, 1);
            StabilityLabel("Np: ", stabDerivOutput.stabDerivs[20], " s⁻¹", "Change in yaw-right angular acceleration with respect to roll-right rate; sign unimportant", W160, 0);
            StabilityLabel("Nr: ", stabDerivOutput.stabDerivs[23], " s⁻¹", "Change in yaw-right angular acceleration with respect to yaw-right rate; should be negative", W160, -1);
            GUILayout.EndHorizontal();
            GUILayout.EndVertical();

            DrawTooltip();
        }

        private void StabilityLabel(String text1, double val, String text2, String tooltip, int width, int sign)
        {
            Color color = Color.white;
            if (sign != 0)
                color = (Math.Sign(val) == sign) ? Color.green : Color.red;

            GUIStyle style = new GUIStyle(GUI.skin.label);
            style.normal.textColor = style.hover.textColor = color;

            GUILayout.Label(new GUIContent(text1 + val.ToString("G6") + text2, tooltip), style, GUILayout.Width(width));
        }

        private void DrawTooltip()
        {
            if (GUI.tooltip == "")
                return;

            GUIStyle BackgroundStyle = new GUIStyle(GUI.skin.box);
            BackgroundStyle.hover = BackgroundStyle.active = BackgroundStyle.normal;

            Vector3 mousePos = GUIUtils.GetMousePos();
            Rect windowRect = EditorGUI.GUIRect;

            Rect tooltipRect = new Rect(Mathf.Clamp(mousePos.x - windowRect.x, 0, windowRect.width - 300), Mathf.Clamp(mousePos.y - windowRect.y, 0, windowRect.height - 80), 300, 80);

            GUIStyle toolTipStyle = BackgroundStyle;
            toolTipStyle.normal.textColor = toolTipStyle.active.textColor = toolTipStyle.hover.textColor = toolTipStyle.focused.textColor = toolTipStyle.onNormal.textColor = toolTipStyle.onHover.textColor = toolTipStyle.onActive.textColor = toolTipStyle.onFocused.textColor = new Color(1, 0.75f, 0);

            GUI.Box(tooltipRect, GUI.tooltip, toolTipStyle);
        }
    }
}
