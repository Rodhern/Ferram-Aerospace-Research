/*
Ferram Aerospace Research v0.15.9.6 "Lin"
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
using UnityEngine;
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation
{
    class InstantConditionSim
    {
        List<FARAeroSection> _currentAeroSections;
        List<FARAeroPartModule> _currentAeroModules;
        List<FARWingAerodynamicModel> _wingAerodynamicModel;

        public double _maxCrossSectionFromBody;
        public double _bodyLength;

        public bool Ready
        {
            get { return _currentAeroSections != null && _currentAeroModules != null && _wingAerodynamicModel != null; }
        }

        public void UpdateAeroData(List<FARAeroPartModule> aeroModules, List<FARAeroSection> aeroSections, VehicleAerodynamics vehicleAero, List<FARWingAerodynamicModel> wingAerodynamicModel)
        {
            _currentAeroModules = aeroModules;
            _currentAeroSections = aeroSections;
            _wingAerodynamicModel = wingAerodynamicModel;
            _maxCrossSectionFromBody = vehicleAero.MaxCrossSectionArea;
            _bodyLength = vehicleAero.Length;
        }

        public Vector3d GetCoM()
        {
            Vector3d CoM;
            double mass, area, MAC, b;
            GetCoMAndSize(out CoM, out mass, out area, out MAC, out b);
            return CoM;
        }

        public void GetCoMAndSize(out Vector3d CoM, out double mass, out double area, out double MAC, out double b)
        {
            CoM = Vector3d.zero;
            mass = 0; area = 0; MAC = 0; b = 0;

            List<Part> partsList = EditorLogic.SortedShipList;
            for (int i = 0; i < partsList.Count; i++)
            {
                Part p = partsList[i];
                if (FARAeroUtil.IsNonphysical(p))
                    continue;

                double partMass = p.mass;
                if (p.Resources.Count > 0)
                    partMass += p.GetResourceMass();
                //partMass += p.GetModuleMass(p.mass); // If you want to use GetModuleMass, you need to start from p.partInfo.mass, not p.mass

                CoM += partMass * (Vector3d)p.transform.TransformPoint(p.CoMOffset);
                mass += partMass;

                FARWingAerodynamicModel w = p.GetComponent<FARWingAerodynamicModel>();
                if (w != null && !w.isShielded)
                {
                    area += w.S;
                    MAC += w.GetMAC() * w.S;
                    b += w.Getb_2() * w.S;
                }
            }
            if (area > 0)
            {
                MAC /= area;
                b /= area;
            }
            else
            {
                area = _maxCrossSectionFromBody;
                MAC = _bodyLength;
                b = 1;
            }
            CoM /= mass;
            mass *= 1000;
        }

        public void GetInertia(Vector3d CoM, out double Ix, out double Iy, out double Iz, out double Ixy, out double Iyz, out double Ixz)
        {
            Ix = 0; Iy = 0; Iz = 0; Ixy = 0; Iyz = 0; Ixz = 0;

            List<Part> partsList = EditorLogic.SortedShipList;
            for (int i = 0; i < partsList.Count; i++)
            {
                Part p = partsList[i];
                if (p == null || FARAeroUtil.IsNonphysical(p))
                    continue;

                //This section handles the parallel axis theorem
                Vector3 relPos = p.transform.TransformPoint(p.CoMOffset) - CoM;
                double x2, y2, z2, x, y, z;
                x2 = relPos.z * relPos.z;
                y2 = relPos.x * relPos.x;
                z2 = relPos.y * relPos.y;
                x = relPos.z;
                y = relPos.x;
                z = relPos.y;

                double partMass = p.mass;
                if (p.Resources.Count > 0)
                    partMass += p.GetResourceMass();
                //partMass += p.GetModuleMass(p.mass); // If you want to use GetModuleMass, you need to start from p.partInfo.mass, not p.mass

                Ix += (y2 + z2) * partMass;
                Iy += (x2 + z2) * partMass;
                Iz += (x2 + y2) * partMass;

                Ixy += -x * y * partMass;
                Iyz += -z * y * partMass;
                Ixz += -x * z * partMass;

                //And this handles the part's own moment of inertia
                Vector3 principalInertia = p.Rigidbody.inertiaTensor;
                Quaternion prncInertRot = p.Rigidbody.inertiaTensorRotation;

                //The rows of the direction cosine matrix for a quaternion
                Vector3 Row1 = new Vector3(prncInertRot.x * prncInertRot.x - prncInertRot.y * prncInertRot.y - prncInertRot.z * prncInertRot.z + prncInertRot.w * prncInertRot.w,
                    2 * (prncInertRot.x * prncInertRot.y + prncInertRot.z * prncInertRot.w),
                    2 * (prncInertRot.x * prncInertRot.z - prncInertRot.y * prncInertRot.w));

                Vector3 Row2 = new Vector3(2 * (prncInertRot.x * prncInertRot.y - prncInertRot.z * prncInertRot.w),
                    -prncInertRot.x * prncInertRot.x + prncInertRot.y * prncInertRot.y - prncInertRot.z * prncInertRot.z + prncInertRot.w * prncInertRot.w,
                    2 * (prncInertRot.y * prncInertRot.z + prncInertRot.x * prncInertRot.w));

                Vector3 Row3 = new Vector3(2 * (prncInertRot.x * prncInertRot.z + prncInertRot.y * prncInertRot.w),
                    2 * (prncInertRot.y * prncInertRot.z - prncInertRot.x * prncInertRot.w),
                    -prncInertRot.x * prncInertRot.x - prncInertRot.y * prncInertRot.y + prncInertRot.z * prncInertRot.z + prncInertRot.w * prncInertRot.w);

                //And converting the principal moments of inertia into the coordinate system used by the system
                Ix += principalInertia.x * Row1.x * Row1.x + principalInertia.y * Row1.y * Row1.y + principalInertia.z * Row1.z * Row1.z;
                Iy += principalInertia.x * Row2.x * Row2.x + principalInertia.y * Row2.y * Row2.y + principalInertia.z * Row2.z * Row2.z;
                Iz += principalInertia.x * Row3.x * Row3.x + principalInertia.y * Row3.y * Row3.y + principalInertia.z * Row3.z * Row3.z;

                Ixy += principalInertia.x * Row1.x * Row2.x + principalInertia.y * Row1.y * Row2.y + principalInertia.z * Row1.z * Row2.z;
                Ixz += principalInertia.x * Row1.x * Row3.x + principalInertia.y * Row1.y * Row3.y + principalInertia.z * Row1.z * Row3.z;
                Iyz += principalInertia.x * Row2.x * Row3.x + principalInertia.y * Row2.y * Row3.y + principalInertia.z * Row2.z * Row3.z;
            }
            Ix *= 1000;
            Iy *= 1000;
            Iz *= 1000;
        }


        public void GetAxisVectors(Vector3d CoM, InstantConditionSimInput input, out Vector3d velocity, out Vector3d liftDown, out Vector3d sideways, out Vector3d angVel)
        {
            Vector3d forward = Vector3.forward;
            Vector3d up = Vector3.up;
            Vector3d right = Vector3.right;

            if (EditorDriver.editorFacility == EditorFacility.VAB)
            {
                forward = Vector3.up;
                up = -Vector3.forward;
            }

            double sinAlpha = Math.Sin(input.alpha * Math.PI / 180);
            double cosAlpha = Math.Sqrt(Math.Max(1 - sinAlpha * sinAlpha, 0));

            double sinBeta = Math.Sin(input.beta * Math.PI / 180);
            double cosBeta = Math.Sqrt(Math.Max(1 - sinBeta * sinBeta, 0));

            double sinPhi = Math.Sin(input.phi * Math.PI / 180);
            double cosPhi = Math.Sqrt(Math.Max(1 - sinPhi * sinPhi, 0));

            double alphaDot = input.alphaDot * Math.PI / 180;
            double betaDot = input.betaDot * Math.PI / 180;
            double phiDot = input.phiDot * Math.PI / 180;

            velocity = forward * cosAlpha * cosBeta;
            velocity += right * (sinPhi * sinAlpha * cosBeta + cosPhi * sinBeta);
            velocity += -up * (cosPhi * sinAlpha * cosBeta - sinPhi * sinBeta);

            liftDown = -forward * sinAlpha;
            liftDown += right * sinPhi * cosAlpha;
            liftDown += -up * cosPhi * cosAlpha;

            sideways = -forward * cosAlpha * sinBeta;
            sideways += right * (cosPhi * cosBeta - sinPhi * sinAlpha * sinBeta);
            sideways += up * (cosPhi * sinAlpha * sinBeta + sinPhi * cosBeta);

            angVel = forward * (phiDot - sinAlpha * betaDot);
            angVel += right * (cosPhi * alphaDot + cosAlpha * sinPhi * betaDot);
            angVel += up * (sinPhi * alphaDot - cosAlpha * cosPhi * betaDot);
        }

        public double CalculateAccelerationDueToGravity(CelestialBody body, double alt)
        {
            return CalculateEffectiveGravity(body, alt, 0);
        }

        public double CalculateEffectiveGravity(CelestialBody body, double altitude, double speed)
        {
            double radius = body.Radius + altitude;
            double mu = body.gravParameter;
            return mu / (radius * radius) - (speed * speed) / radius;
        }

        public void GetClCdCmSteady(InstantConditionSimInput input, out InstantConditionSimOutput output, bool clear, bool reset_stall)
        {
            Vector3d CoM;
            double mass, area, MAC, b; // mass not actually used in these calculations
            GetCoMAndSize(out CoM, out mass, out area, out MAC, out b);

            FARCenterQuery center;
            ResetClCdCmSteady(CoM, input, out center, false, clear, reset_stall);

            Vector3d velocity, liftDown, sideways, angVel, velVector;
            GetAxisVectors(CoM, input, out velocity, out liftDown, out sideways, out angVel);
            velVector = input.fltenv.VelocityVector(velocity);

            output = new InstantConditionSimOutput();

            for (int i = 0; i < _wingAerodynamicModel.Count; i++)
            {
                FARWingAerodynamicModel w = _wingAerodynamicModel[i];
                if (!(w && w.part) || w.isShielded)
                    continue;

                Vector3d relPos = w.GetAerodynamicCenter() - CoM;
                Vector3d vel = velVector + Vector3d.Cross(angVel, relPos);
                Vector3d force = w.ComputeForceEditor(vel, input.fltenv);

                output.Cl += -Vector3d.Dot(force, liftDown);
                output.Cd += -Vector3d.Dot(force, velocity);
                output.Cy += Vector3d.Dot(force, sideways);

                Vector3d moment = -Vector3d.Cross(relPos, force);

                output.Cm += Vector3d.Dot(moment, sideways);
                output.Cn += Vector3d.Dot(moment, liftDown);
                output.C_roll += Vector3d.Dot(moment, velocity);
            }

            Vector3d centerForce = center.force;

            output.Cl += -Vector3d.Dot(centerForce, liftDown);
            output.Cd += -Vector3d.Dot(centerForce, velocity);
            output.Cy += Vector3d.Dot(centerForce, sideways);

            Vector3d centerMoment = -center.TorqueAt(CoM);

            output.Cm += Vector3d.Dot(centerMoment, sideways);
            output.Cn += Vector3d.Dot(centerMoment, liftDown);
            output.C_roll += Vector3d.Dot(centerMoment, velocity);

            double q = input.fltenv.DynamicPressure();
            double recip = 1 / (q * area); // reciprocal value to area and dynamic pressure
            output.Cl *= recip;
            output.Cd *= recip;
            output.Cy *= recip;
            output.Cm *= recip / MAC;
            output.Cn *= recip / b;
            output.C_roll *= recip / b;
        }

        /// <summary>
        /// Explicit reset, incl. SetControlState, but discard the FARCenterQuery result.
        /// </summary>
        public void ResetClCdCmSteady(Vector3d CoM, InstantConditionSimInput input)
        {
            FARCenterQuery center;
            ResetClCdCmSteady(CoM, input, out center, true, true, true);
        }

        public void ResetClCdCmSteady(Vector3d CoM, InstantConditionSimInput input, out FARCenterQuery center, bool reset_cossweep, bool clear_clcd, bool reset_stall)
        {
            Vector3d velocity, liftDown, sideways, angVel, velVector;
            GetAxisVectors(CoM, input, out velocity, out liftDown, out sideways, out angVel);
            velVector = input.fltenv.VelocityVector(velocity);

            if (reset_cossweep)
            for (int i = 0; i < _wingAerodynamicModel.Count; i++)
            {
                FARWingAerodynamicModel w = _wingAerodynamicModel[i];
                if (w == null)
                    continue;

                w.ResetCosSweepAngle();
            }

            for (int i = 0; i < _wingAerodynamicModel.Count; i++)
            {
                FARWingAerodynamicModel w = _wingAerodynamicModel[i];
                if (!(w && w.part))
                    continue;

                w.ComputeForceEditor(velVector, input.fltenv);
            }

            if (clear_clcd)
            for (int i = 0; i < _wingAerodynamicModel.Count; i++)
            {
                FARWingAerodynamicModel w = _wingAerodynamicModel[i];
                if (!(w && w.part))
                    continue;

                w.EditorClClear(reset_stall);
            }

            for (int i = 0; i < _wingAerodynamicModel.Count; i++)
            {
                FARWingAerodynamicModel w = _wingAerodynamicModel[i];
                if (!(w && w.part))
                    continue;

                Vector3d relPos = w.GetAerodynamicCenter() - CoM;
                Vector3d vel = velVector + Vector3d.Cross(angVel, relPos);

                if (w is FARControllableSurface)
                    (w as FARControllableSurface).SetControlStateEditor(CoM, vel, (float)input.pitchValue, 0, 0, input.flaps, input.spoilers);
                else if (w.isShielded)
                    continue;

                w.ComputeForceEditor(vel, input.fltenv);
            }

            center = new FARCenterQuery();
            for (int i = 0; i < _currentAeroSections.Count; i++)
            {
                // Rodhern: The hardcoded 2 for density is replaced, but some other parameters are still rough editor approximations.
                float reynoldsPerUnitLength = 10000;
                float pseudoKnudsenNumber = 0;
                double pseudoLengthScale = 1;
                float skinFrictionDrag = (float)input.fltenv.SkinFrictionDrag(pseudoLengthScale);
                _currentAeroSections[i].PredictionCalculateAeroForces((float)input.fltenv.Rho, (float)input.fltenv.MachNumber,
                                                                      reynoldsPerUnitLength, pseudoKnudsenNumber, skinFrictionDrag,
                                                                      velVector, center);
            }
        }

    }
}
