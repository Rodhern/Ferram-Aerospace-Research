﻿/*
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
using System.Reflection;
using System.Diagnostics;
using UnityEngine;
using KSP.UI.Screens;
using KSP.Localization;
using ModuleWheels;
using PreFlightTests;
using FerramAerospaceResearch.FARUtils;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARPartGeometry;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.DesignConcerns;
using ferram4;
using Debug = UnityEngine.Debug;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI
{
    [KSPAddon(KSPAddon.Startup.EditorAny, false)]
    public class EditorGUI : MonoBehaviour
    {
        static EditorGUI instance;
        public static EditorGUI Instance
        {
            get { return instance; }
        }

        int _updateRateLimiter = 0;
        bool _updateQueued = true;

        static bool showGUI = false;
        bool useKSPSkin = true;
        Rect guiRect;
        public static Rect GUIRect
        {
            get { return instance.guiRect; }
        }
        static IButton blizzyEditorGUIButton;

        VehicleAerodynamics _vehicleAero;
        List<GeometryPartModule> _currentGeometryModules = new List<GeometryPartModule>();
        List<FARWingAerodynamicModel> _wingAerodynamicModel = new List<FARWingAerodynamicModel>();
        Stopwatch voxelWatch = new Stopwatch();

        int prevPartCount = 0;
        bool partMovement = false;

        EditorSimManager _simManager;

        InstantConditionSim _instantSim;
        EditorAreaRulingOverlay _areaRulingOverlay;
        StaticAnalysisGraphGUI _editorGraph;
        StabilityDerivGUI _stabDeriv;
        StabilityDerivSimulationGUI _stabDerivLinSim;

        List<IDesignConcern> _customDesignConcerns = new List<IDesignConcern>();

        MethodInfo editorReportUpdate;

        bool gearToggle = false;
        bool showAoAArrow = true;

        ArrowPointer velocityArrow = null;
        Transform arrowTransform = null;

        GUIDropDown<FAREditorMode> modeDropdown;
        FAREditorMode currentMode = FAREditorMode.STATIC;
        private enum FAREditorMode
        {
            STATIC,
            STABILITY,
            SIMULATION,
            AREA_RULING
        }

        private static string[] FAReditorMode_str =
        {
            Localizer.Format("FAREditorModeStatic"),
            Localizer.Format("FAREditorModeDataStab"),
            Localizer.Format("FAREditorModeDerivSim"),
            Localizer.Format("FAREditorModeTrans")
        };

        void Start()
        {
            if (CompatibilityChecker.IsAllCompatible() && instance == null)
                instance = this;
            else
            {
                GameObject.Destroy(this);
                return;
            }

            showGUI = false;
            if (FARDebugAndSettings.FARDebugButtonStock)
                    FARDebugAndSettings.FARDebugButtonStock.SetFalse(false);

            _vehicleAero = new VehicleAerodynamics();

            guiRect = new Rect(Screen.width / 4, Screen.height / 6, 10, 10);

            _instantSim = new InstantConditionSim();
            GUIDropDown<int> flapSettingDropDown = new GUIDropDown<int>(new string[] { Localizer.Format("FARFlapSetting0"), Localizer.Format("FARFlapSetting1"), Localizer.Format("FARFlapSetting2"), Localizer.Format("FARFlapSetting3") }, new int[] { 0, 1, 2, 3 }, 0);
            GUIDropDown<CelestialBody> celestialBodyDropdown = CreateBodyDropdown();

            modeDropdown = new GUIDropDown<FAREditorMode>(FAReditorMode_str, new FAREditorMode[] {FAREditorMode.STATIC, FAREditorMode.STABILITY, FAREditorMode.SIMULATION, FAREditorMode.AREA_RULING});

            _simManager = new EditorSimManager(_instantSim);

            _editorGraph = new StaticAnalysisGraphGUI(_simManager, flapSettingDropDown, celestialBodyDropdown);
            _stabDeriv = new StabilityDerivGUI(_simManager, flapSettingDropDown, celestialBodyDropdown);
            _stabDerivLinSim = new StabilityDerivSimulationGUI(_simManager);

            Color crossSection = GUIColors.GetColor(3);
            crossSection.a = 0.8f;

            Color crossSectionDeriv = GUIColors.GetColor(2);
            crossSectionDeriv.a = 0.8f;

            _areaRulingOverlay = EditorAreaRulingOverlay.CreateNewAreaRulingOverlay(new Color(0.05f, 0.05f, 0.05f, 0.7f), crossSection, crossSectionDeriv, 10, 5);
            guiRect.height = 500;
            guiRect.width = 650;


            GameEvents.onEditorPartEvent.Add(UpdateGeometryEvent);
            GameEvents.onEditorUndo.Add(ResetEditorEvent);
            GameEvents.onEditorRedo.Add(ResetEditorEvent);
            GameEvents.onEditorShipModified.Add(ResetEditorEvent);
            GameEvents.onEditorLoad.Add(ResetEditorEvent);

            GameEvents.onGUIEngineersReportReady.Add(AddDesignConcerns);
            GameEvents.onGUIEngineersReportDestroy.Add(RemoveDesignConcerns);

            RequestUpdateVoxel();
        }

        void AddDesignConcerns()
        {
            editorReportUpdate = EngineersReport.Instance.GetType().GetMethod("OnCraftModified", BindingFlags.Instance | BindingFlags.NonPublic, null, Type.EmptyTypes, null);
            _customDesignConcerns.Add(new AreaRulingConcern(_vehicleAero));
            //_customDesignConcerns.Add(new AeroStabilityConcern(_instantSim, EditorDriver.editorFacility == EditorFacility.SPH ? EditorFacilities.SPH : EditorFacilities.VAB));
            for (int i = 0; i < _customDesignConcerns.Count; i++)
                EngineersReport.Instance.AddTest(_customDesignConcerns[i]);
        }

        void RemoveDesignConcerns()
        {
            for (int i = 0; i < _customDesignConcerns.Count; i++)
                EngineersReport.Instance.RemoveTest(_customDesignConcerns[i]);
        }

        void OnDestroy()
        {
            GameEvents.onEditorPartEvent.Remove(UpdateGeometryEvent);
            GameEvents.onEditorUndo.Remove(ResetEditorEvent);
            GameEvents.onEditorRedo.Remove(ResetEditorEvent);
            GameEvents.onEditorShipModified.Remove(ResetEditorEvent);
            GameEvents.onEditorLoad.Remove(ResetEditorEvent);

            GameEvents.onGUIEngineersReportReady.Remove(AddDesignConcerns);
            GameEvents.onGUIEngineersReportDestroy.Remove(AddDesignConcerns);

            //EditorLogic.fetch.Unlock("FAREdLock");

            if (blizzyEditorGUIButton != null)
            {
                blizzyEditorGUIButton.Destroy();
                blizzyEditorGUIButton = null;
            }

            _stabDerivLinSim = null;
            _instantSim = null;
            _areaRulingOverlay.Cleanup();
            _areaRulingOverlay = null;
            _editorGraph = null;
            _stabDeriv = null;

            if (_vehicleAero != null)
                _vehicleAero.ForceCleanup();
            _vehicleAero = null;
        }

        #region EditorEvents
        private void ResetEditorEvent(ShipConstruct construct)
        {
            FARAeroUtil.ResetEditorParts();

            if (EditorLogic.RootPart != null)
            {
                List<Part> partsList = EditorLogic.SortedShipList;
                for (int i = 0; i < partsList.Count; i++)
                    UpdateGeometryModule(partsList[i]);
            }

            RequestUpdateVoxel();
        }
        private void ResetEditorEvent(ShipConstruct construct, CraftBrowserDialog.LoadType type)
        {
            ResetEditor();
        }

        public static void ResetEditor()
        {
            Color crossSection = GUIColors.GetColor(3);
            crossSection.a = 0.8f;

            Color crossSectionDeriv = GUIColors.GetColor(2);
            crossSectionDeriv.a = 0.8f;
            instance._areaRulingOverlay.RestartOverlay();
            //instance._areaRulingOverlay = new EditorAreaRulingOverlay(new Color(0.05f, 0.05f, 0.05f, 0.7f), crossSection, crossSectionDeriv, 10, 5);
            RequestUpdateVoxel();
        }

        private void UpdateGeometryEvent(ConstructionEventType type, Part pEvent)
        {
            if (type == ConstructionEventType.PartRotated ||
            type == ConstructionEventType.PartOffset ||
            type == ConstructionEventType.PartAttached ||
            type == ConstructionEventType.PartDetached ||
            type == ConstructionEventType.PartRootSelected ||
                type == ConstructionEventType.Unknown)
            {
                if (EditorLogic.SortedShipList.Count > 0)
                    UpdateGeometryModule(type, pEvent);
                RequestUpdateVoxel();

                if (type != ConstructionEventType.Unknown)
                    partMovement = true;
            }
        }

        private void UpdateGeometryModule(ConstructionEventType type, Part p)
        {
            GeometryPartModule g = p.GetComponent<GeometryPartModule>();
            if (g != null && g.Ready)
            {
                if (type == ConstructionEventType.Unknown)
                    g.RebuildAllMeshData();
                else
                    g.EditorUpdate();
            }
        }

        private void UpdateGeometryModule(Part p)
        {
            GeometryPartModule g = p.GetComponent<GeometryPartModule>();
            if (g != null && g.Ready)
            {
                g.EditorUpdate();
            }
        }


        private void LEGACY_UpdateWingAeroModels(bool updateWingInteractions)
        {
            List<Part> partsList = EditorLogic.SortedShipList;
            _wingAerodynamicModel.Clear();
            for (int i = 0; i < partsList.Count; i++)
            {
                Part p = partsList[i];
                if(p != null)
                    if (p.Modules.Contains<FARWingAerodynamicModel>())
                    {
                        FARWingAerodynamicModel w = p.Modules.GetModule<FARWingAerodynamicModel>();
                        if(updateWingInteractions)
                            w.EditorUpdateWingInteractions();
                        _wingAerodynamicModel.Add(w);
                    }
                    else if (p.Modules.Contains<FARControllableSurface>())
                    {
                        FARControllableSurface c = p.Modules.GetModule<FARControllableSurface>();
                        if (updateWingInteractions)
                            c.EditorUpdateWingInteractions();
                        _wingAerodynamicModel.Add(c);
                    }
            }

        }
        #endregion
        void Awake()
        {
            FARThreading.VoxelizationThreadpool.RunInMainThread = Debug.isDebugBuild;
            if (FARDebugValues.useBlizzyToolbar)
                GenerateBlizzyToolbarButton();
        }
        void Update()
        {
            FARThreading.VoxelizationThreadpool.Instance.ExecuteMainThreadTasks();
        }

        void FixedUpdate()
        {
            if (EditorLogic.RootPart != null)
            {
                if (_vehicleAero.CalculationCompleted)
                {
                    _vehicleAero.UpdateSonicDragArea();
                    LEGACY_UpdateWingAeroModels(EditorLogic.SortedShipList.Count != prevPartCount || partMovement);
                    prevPartCount = EditorLogic.SortedShipList.Count;

                    voxelWatch.Stop();
                    FARLogger.Info("Voxelization Time (ms): " + voxelWatch.ElapsedMilliseconds);

                    voxelWatch.Reset();

                    _simManager.UpdateAeroData(_vehicleAero, _wingAerodynamicModel);
                    UpdateCrossSections();
                    editorReportUpdate.Invoke(EngineersReport.Instance, null);
                }

                if (_updateRateLimiter < FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate)
                {
                    _updateRateLimiter++;
                }
                else if (_updateQueued)
                {
                    var shipname = EditorLogic.fetch.ship.shipName ?? "unknown ship";
                    FARLogger.Info("Updating " + shipname);
                    RecalculateVoxel();
                }
            }
            else
            {
                _updateQueued = true;
                _updateRateLimiter = FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate - 2;
            }
        }

        #region voxel
        public static void RequestUpdateVoxel()
        {
            if (instance._updateRateLimiter > FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate)
                instance._updateRateLimiter = FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate - 2;
            instance._updateQueued = true;
            //instance._areaRulingOverlay.SetVisibility(false);

        }

        void RecalculateVoxel()
        {
            if (_updateRateLimiter < FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate)        //this has been updated recently in the past; queue an update and return
            {
                _updateQueued = true;
                return;
            }
            else                                //last update was far enough in the past to run; reset rate limit counter and clear the queued flag
            {
                _updateRateLimiter = 0;
                _updateQueued = false;
            }
            List<Part> partList = EditorLogic.SortedShipList;

            _currentGeometryModules.Clear();

            for (int i = 0; i < partList.Count; i++)
            {
                Part p = partList[i];
                if (p.Modules.Contains<GeometryPartModule>())
                {
                    GeometryPartModule g = p.Modules.GetModule<GeometryPartModule>();
                    if (g != null)
                    {
                        if (g.Ready)
                            _currentGeometryModules.Add(g);
                        else
                        {
                            _updateRateLimiter = FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate - 2;
                            _updateQueued = true;
                            //FARLogger.Info("We're not ready!");
                            return;
                        }
                    }
                }
            }
            TriggerIGeometryUpdaters();


            if (_currentGeometryModules.Count > 0)
            {
                voxelWatch.Start();
                if (!_vehicleAero.TryVoxelUpdate(EditorLogic.RootPart.partTransform.worldToLocalMatrix, EditorLogic.RootPart.partTransform.localToWorldMatrix, FARSettingsScenarioModule.VoxelSettings.numVoxelsControllableVessel, partList, _currentGeometryModules, true))
                {
                    voxelWatch.Stop();
                    voxelWatch.Reset();
                    _updateRateLimiter = FARSettingsScenarioModule.VoxelSettings.minPhysTicksPerUpdate - 2;
                    _updateQueued = true;
                }
            }
        }

        private void TriggerIGeometryUpdaters()
        {
            for (int i = 0; i < _currentGeometryModules.Count; i++)
                _currentGeometryModules[i].RunIGeometryUpdaters();
        }

        void UpdateCrossSections()
        {
            double[] areas = _vehicleAero.GetCrossSectionAreas();
            double[] secondDerivAreas = _vehicleAero.GetCrossSection2ndAreaDerivs();
            double[] pressureCoeff = _vehicleAero.GetPressureCoeffs();

            double sectionThickness = _vehicleAero.SectionThickness;
            double offset = _vehicleAero.FirstSectionXOffset();

            double[] xAxis = new double[areas.Length];

            double maxValue = 0;
            for (int i = 0; i < areas.Length; i++)
            {
                maxValue = Math.Max(maxValue, areas[i]);
            }

            for (int i = 0; i < xAxis.Length; i++)
            {
                xAxis[i] = (xAxis.Length - i - 1) * sectionThickness + offset;
            }

            _areaRulingOverlay.UpdateAeroData(_vehicleAero.VoxelAxisToLocalCoordMatrix(), xAxis, areas, secondDerivAreas, pressureCoeff, maxValue);
        }
        #endregion

        #region GUIFunctions

        void OnGUI()
        {
            //Make this an option
            if (useKSPSkin)
                GUI.skin = HighLogic.Skin;

            PreventClickThrough();
        }

        /// <summary> Lock the model if our own window is shown and has cursor focus to prevent click-through. </summary>
        private void PreventClickThrough()
        {
            bool cursorInGUI = false;
            EditorLogic EdLogInstance = EditorLogic.fetch;
            if (!EdLogInstance)
            {
                return;
            }
            if (showGUI)
            {
                guiRect = GUILayout.Window(this.GetHashCode(), guiRect, OverallSelectionGUI, Localizer.Format("FAREditorTitle"));
                guiRect = GUIUtils.ClampToScreen(guiRect);
                cursorInGUI = guiRect.Contains(GUIUtils.GetMousePos());
            }
            if (cursorInGUI)
            {
                // TODO 1.2: verify what EditorTooltip is/was, it cannot be found
                //if (EditorTooltip.Instance)
                //    EditorTooltip.Instance.HideToolTip();

                if (!CameraMouseLook.GetMouseLook())
                    EdLogInstance.Lock(false, false, false, "FAREdLock");
                else
                    EdLogInstance.Unlock("FAREdLock");
            }
            else if (!cursorInGUI)
            {
                EdLogInstance.Unlock("FAREdLock");
            }
        }

        void OverallSelectionGUI(int windowId)
        {
            GUILayout.BeginHorizontal(GUILayout.Width(800));
            modeDropdown.GUIDropDownDisplay();
            currentMode = modeDropdown.ActiveSelection;

            GUILayout.BeginVertical();
            if (GUILayout.Button(gearToggle ? Localizer.Format("FARGearToggleLower") : Localizer.Format("FARGearToggleRaise")))
                ToggleGear();
            GUILayout.EndVertical();

            GUILayout.BeginVertical();
            if (GUILayout.Button(showAoAArrow ? Localizer.Format("FARVelIndHide") : Localizer.Format("FARVelIndShow")))
                showAoAArrow = !showAoAArrow;
            GUILayout.EndVertical();

            GUILayout.EndHorizontal();
            //GUILayout.EndHorizontal();
            if (currentMode == FAREditorMode.STATIC)
            {
                _editorGraph.Display();
                guiRect.height = useKSPSkin ? 570 : 450;
            }
            else if (currentMode == FAREditorMode.STABILITY)
            {
                _stabDeriv.Display();
                guiRect.height = useKSPSkin ? 680 : 450;
            }
            else if (currentMode == FAREditorMode.SIMULATION)
            {
                _stabDerivLinSim.Display();
                guiRect.height = useKSPSkin ? 570 : 450;
            }
            else if (currentMode == FAREditorMode.AREA_RULING)
            {
                CrossSectionAnalysisGUI();
                DebugVisualizationGUI();
                guiRect.height = useKSPSkin ? 350 : 220;
            }

            GUI.DragWindow();
        }

        void DebugVisualizationGUI()
        {
            GUILayout.BeginHorizontal();
            if (GUILayout.Button(Localizer.Format("FARDebugVoxels")))
                _vehicleAero.DebugVisualizeVoxels(EditorLogic.RootPart.partTransform.localToWorldMatrix);
            GUILayout.EndHorizontal();
        }

        void CrossSectionAnalysisGUI()
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("FAREditorTitleTransonic"), GUILayout.Width(350));
            GUILayout.EndHorizontal();

            GUIStyle BackgroundStyle = new GUIStyle(GUI.skin.box);
            BackgroundStyle.hover = BackgroundStyle.active = BackgroundStyle.normal;

            GUILayout.BeginHorizontal();
            GUILayout.BeginVertical(BackgroundStyle, GUILayout.Width(350), GUILayout.ExpandHeight(true));
            GUILayout.Label(Localizer.Format("FAREditorTransMaxArea") + _vehicleAero.MaxCrossSectionArea.ToString("G6") + " " + Localizer.Format("FARUnitMSq"));
            GUILayout.Label(Localizer.Format("FAREditorTransMach1DragArea") + _vehicleAero.SonicDragArea.ToString("G6") + " " + Localizer.Format("FARUnitMSq"));
            GUILayout.Label(Localizer.Format("FAREditorTransCritMach") + _vehicleAero.CriticalMach.ToString("G6"));
            GUILayout.EndVertical();

            GUILayout.BeginVertical(BackgroundStyle, GUILayout.ExpandHeight(true));
            GUILayout.Label(Localizer.Format("FAREditorTransMinDragExp1"));
            bool areaVisible  = _areaRulingOverlay.IsVisible(EditorAreaRulingOverlay.OverlayType.AREA);
            bool derivVisible = _areaRulingOverlay.IsVisible(EditorAreaRulingOverlay.OverlayType.DERIV);
            bool coeffVisible = _areaRulingOverlay.IsVisible(EditorAreaRulingOverlay.OverlayType.COEFF);

            if (GUILayout.Toggle(areaVisible, Localizer.Format("FAREditorTransAreaCurve")) != areaVisible)
                _areaRulingOverlay.SetVisibility(EditorAreaRulingOverlay.OverlayType.AREA, !areaVisible);

            if (GUILayout.Toggle(derivVisible, Localizer.Format("FAREditorTransCurvCurve")) != derivVisible)
                _areaRulingOverlay.SetVisibility(EditorAreaRulingOverlay.OverlayType.DERIV, !derivVisible);

            if (GUILayout.Toggle(coeffVisible, Localizer.Format("FAREditorTransPresCurve")) != coeffVisible)
                _areaRulingOverlay.SetVisibility(EditorAreaRulingOverlay.OverlayType.COEFF, !coeffVisible);

            GUILayout.Label(Localizer.Format("FAREditorTransMinDragExp2"));
            GUILayout.EndVertical();
            GUILayout.EndHorizontal();
        }
        #endregion

        #region AoAArrow
        void LateUpdate()
        {
            if (arrowTransform == null)
            {
                if (velocityArrow != null)
                    UnityEngine.Object.Destroy(velocityArrow);

                if (EditorLogic.RootPart != null)
                    arrowTransform = EditorLogic.RootPart.partTransform;
                else
                    return;
            }
            if (velocityArrow == null)
                velocityArrow = ArrowPointer.Create(arrowTransform, Vector3.zero, Vector3.forward, 15, Color.white, true);

            if (showGUI && showAoAArrow)
            {
                velocityArrow.gameObject.SetActive(true);
                ArrowDisplay();
            }
            else
                velocityArrow.gameObject.SetActive(false);
        }

        void ArrowDisplay()
        {
            if (currentMode == FAREditorMode.STATIC)
                _editorGraph.ArrowAnim(velocityArrow);
            else if (currentMode == FAREditorMode.STABILITY || currentMode == FAREditorMode.SIMULATION)
                _stabDeriv.ArrowAnim(velocityArrow);
            else
                velocityArrow.Direction = Vector3.zero;
        }
        #endregion

        #region AppLauncher

        private void GenerateBlizzyToolbarButton()
        {
            if (blizzyEditorGUIButton == null)
            {
                blizzyEditorGUIButton = ToolbarManager.Instance.add("FerramAerospaceResearch", "FAREditorButtonBlizzy");
                blizzyEditorGUIButton.TexturePath = "FerramAerospaceResearch/Textures/icon_button_blizzy";
                blizzyEditorGUIButton.ToolTip = "FAR Editor";
                blizzyEditorGUIButton.OnClick += (e) => showGUI = !showGUI;
            }
        }

        public static void onAppLaunchToggle()
        {
            showGUI = !showGUI;
        }

        #endregion

        #region UtilFuncs
        GUIDropDown<CelestialBody> CreateBodyDropdown()
        {
            CelestialBody[] bodies = FlightGlobals.Bodies.ToArray();
            string[] bodyNames = new string[bodies.Length];
            for (int i = 0; i < bodyNames.Length; i++)
                bodyNames[i] = bodies[i].bodyName;

            int kerbinIndex = 1;
            GUIDropDown<CelestialBody> celestialBodyDropdown = new GUIDropDown<CelestialBody>(bodyNames, bodies, kerbinIndex);
            return celestialBodyDropdown;
        }

        void ToggleGear()
        {
            List<Part> partsList = EditorLogic.SortedShipList;
            for(int i = 0; i < partsList.Count; i++)
            {
                Part p = partsList[i];
                if (p.Modules.Contains<ModuleWheelDeployment>())
                {
                    ModuleWheelDeployment l = p.Modules.GetModule<ModuleWheelDeployment>();
                    l.ActionToggle(new KSPActionParam(KSPActionGroup.Gear, gearToggle ? KSPActionType.Activate : KSPActionType.Deactivate));
                }
                if(p.Modules.Contains("FSwheel"))
                {
                    PartModule m = p.Modules["FSwheel"];
                    MethodInfo method = m.GetType().GetMethod("animate", BindingFlags.Instance | BindingFlags.NonPublic);
                    method.Invoke(m, gearToggle ? new object[] { "Deploy" } : new object[] { "Retract" });
                }
                if (p.Modules.Contains("FSBDwheel"))
                {
                    PartModule m = p.Modules["FSBDwheel"];
                    MethodInfo method = m.GetType().GetMethod("animate", BindingFlags.Instance | BindingFlags.NonPublic);
                    method.Invoke(m, gearToggle ? new object[] { "Deploy" } : new object[] { "Retract" });
                }
                if (p.Modules.Contains("KSPWheelAdjustableGear"))
                {
                    PartModule m = p.Modules["KSPWheelAdjustableGear"];
                    MethodInfo method = m.GetType().GetMethod("deploy", BindingFlags.Instance | BindingFlags.Public);
                    try
                    {
                        method.Invoke(m, null);
                    }
                    catch(Exception e)
                    {
                        FARLogger.Exception(e);      //we just catch and print this ourselves to allow things to continue working, since there seems to be a bug in KSPWheels as of this writing
                    }
                }
            }
            gearToggle = !gearToggle;
        }

        #endregion
    }
}
