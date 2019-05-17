﻿/*
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

using System.Collections.Generic;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARGUI;
using FerramAerospaceResearch.FARGUI.FARFlightGUI;
using FerramAerospaceResearch.FARPartGeometry;
using FerramAerospaceResearch.FARUtils;
using UnityEngine;

namespace FerramAerospaceResearch
{
    [KSPScenario(ScenarioCreationOptions.AddToAllGames, GameScenes.SPACECENTER, GameScenes.EDITOR, GameScenes.FLIGHT)]
    public class FARSettingsScenarioModule : ScenarioModule
    {
        public bool newGame;
        public FARDifficultyAndExactnessSettings settings;
        public static FARDifficultyAndExactnessSettings Settings
        {
            get
            {
                return Instance.settings;
            }
        }

        public FARDifficultyAndExactnessSettings customSettings;
        public List<FARDifficultyAndExactnessSettings> presets;
        public FARVoxelSettings voxelSettings;
        public static FARVoxelSettings VoxelSettings
        {
            get
            {
                return Instance.voxelSettings;
            }
        }

        public List<ConfigNode> flightGUISettings;
        public static List<ConfigNode> FlightGUISettings
        {
            get
            {
                return Instance.flightGUISettings;
            }
        }

        private static List<string> presetNames;

        public int currentIndex;

        private static FARSettingsScenarioModule instance;
        public static FARSettingsScenarioModule Instance
        {
            get
            {
                return instance;
            }
        }

        private GUIDropDown<FARDifficultyAndExactnessSettings> dropdown;

        public static void MainMenuBuildDefaultScenarioModule()
        {
            if (instance != null) return;
            instance = new GameObject().AddComponent<FARSettingsScenarioModule>();
            FARLogger.Info("Creating new setting module for tutorial/scenario");
            instance.OnLoad(new ConfigNode());
            instance.Start();
        }


        private FARSettingsScenarioModule()
        {
            instance = this;
        }

        private void Start()
        {
            if (!CompatibilityChecker.IsAllCompatible())
            {
                enabled = false;
                return;
            }
            instance = this;

            //if (newGame)
            //    PopupDialog.SpawnPopupDialog("Ferram Aerospace Research", "Welcome to KSP with FAR!\n\r\n\rThings will be much harder from here on out; the FAR button in the top-right corner will bring you to difficulty settings if you ever decide to change them.  Have fun!", "OK", false, HighLogic.Skin);

            FARLogger.Info("Vehicle Voxel Setup started");
            FARAeroSection.GenerateCrossFlowDragCurve();
            VehicleVoxel.VoxelSetup();
            PhysicsGlobals.DragCubeMultiplier = 0;
            FARLogger.Info("Vehicle Voxel Setup complete");

            //GameEvents.onGameStateSave.Add(OnSave);
            newGame = false;
        }

        public override void OnSave(ConfigNode node)
        {
            FARLogger.Info("saved");
            node.AddValue("newGame", newGame);
            node.AddValue("fractionTransonicDrag", settings.fractionTransonicDrag);
            node.AddValue("gaussianVehicleLengthFractionForSmoothing", settings.gaussianVehicleLengthFractionForSmoothing);
            node.AddValue("numAreaSmoothingPasses", settings.numAreaSmoothingPasses);
            node.AddValue("numDerivSmoothingPasses", settings.numDerivSmoothingPasses);
            node.AddValue("numVoxelsControllableVessel", voxelSettings.numVoxelsControllableVessel);
            node.AddValue("numVoxelsDebrisVessel", voxelSettings.numVoxelsDebrisVessel);
            node.AddValue("minPhysTicksPerUpdate", voxelSettings.minPhysTicksPerUpdate);
            node.AddValue("useHigherResVoxelPoints", voxelSettings.useHigherResVoxelPoints);
            node.AddValue("index", settings.index);

            FlightGUI.SaveActiveData();
            ConfigNode flightGUINode = new ConfigNode("FlightGUISettings");
            FARLogger.Info("Saving FAR Data");
            foreach (ConfigNode configNode in flightGUISettings)
            {
                flightGUINode.AddNode(configNode);
            }
            node.AddNode(flightGUINode);

            FARDebugAndSettings.SaveConfigs(node);
        }

        public override void OnLoad(ConfigNode node)
        {
            instance = this;
            GeneratePresets();
            int index = 4;
            if (node.HasValue("newGame"))
                newGame = bool.Parse(node.GetValue("newGame"));

            if (node.HasValue("index"))
                index = int.Parse(node.GetValue("index"));

            dropdown = new GUIDropDown<FARDifficultyAndExactnessSettings>(presetNames.ToArray(), presets.ToArray(), index < 0 ? 2 : index);
            voxelSettings = new FARVoxelSettings();

            if (node.HasValue("numVoxelsControllableVessel"))
                voxelSettings.numVoxelsControllableVessel = int.Parse(node.GetValue("numVoxelsControllableVessel"));
            if (node.HasValue("numVoxelsDebrisVessel"))
                voxelSettings.numVoxelsDebrisVessel = int.Parse(node.GetValue("numVoxelsDebrisVessel"));
            if (node.HasValue("minPhysTicksPerUpdate"))
                voxelSettings.minPhysTicksPerUpdate = int.Parse(node.GetValue("minPhysTicksPerUpdate"));
            if (node.HasValue("useHigherResVoxelPoints"))
                voxelSettings.useHigherResVoxelPoints = bool.Parse(node.GetValue("useHigherResVoxelPoints"));

            if (index == -1)
            {
                settings = new FARDifficultyAndExactnessSettings(index);

                if (node.HasValue("fractionTransonicDrag"))
                    settings.fractionTransonicDrag = double.Parse(node.GetValue("fractionTransonicDrag"));
                if (node.HasValue("gaussianVehicleLengthFractionForSmoothing"))
                    settings.gaussianVehicleLengthFractionForSmoothing = double.Parse(node.GetValue("gaussianVehicleLengthFractionForSmoothing"));

                if (node.HasValue("numAreaSmoothingPasses"))
                    settings.numAreaSmoothingPasses = int.Parse(node.GetValue("numAreaSmoothingPasses"));
                if (node.HasValue("numDerivSmoothingPasses"))
                    settings.numDerivSmoothingPasses = int.Parse(node.GetValue("numDerivSmoothingPasses"));


                customSettings = settings;
            }
            else
            {
                settings = presets[index];
                customSettings = new FARDifficultyAndExactnessSettings(-1);
            }
            currentIndex = index;


            FARLogger.Info("Loading FAR Data");
            flightGUISettings = new List<ConfigNode>();
            if(node.HasNode("FlightGUISettings"))
            {
                foreach (ConfigNode flightGUINode in node.GetNode("FlightGUISettings").nodes)
                    flightGUISettings.Add(flightGUINode);
            }

            FARDebugAndSettings.LoadConfigs(node);
        }

        private void GeneratePresets()
        {
            presets = new List<FARDifficultyAndExactnessSettings>();
            presetNames = new List<string>();

            FARDifficultyAndExactnessSettings tmp = new FARDifficultyAndExactnessSettings(0.6, 0.03, 2, 2, 0);
            presets.Add(tmp);
            presetNames.Add("Low Drag, Lenient Area Ruling");

            tmp = new FARDifficultyAndExactnessSettings(0.7, 0.03, 2, 2, 1);
            presets.Add(tmp);
            presetNames.Add("Moderate Drag, Lenient Area Ruling");

            tmp = new FARDifficultyAndExactnessSettings(0.7, 0.015, 2, 1, 2);
            presets.Add(tmp);
            presetNames.Add("Moderate Drag, Moderate Area Ruling");

            tmp = new FARDifficultyAndExactnessSettings(0.85, 0.015, 2, 1, 3);
            presets.Add(tmp);
            presetNames.Add("High Drag, Moderate Area Ruling");

            tmp = new FARDifficultyAndExactnessSettings(1, 0.015, 2, 1, 4);
            presets.Add(tmp);
            presetNames.Add("Full Drag, Moderate Area Ruling");

            tmp = new FARDifficultyAndExactnessSettings(1, 0.010, 1, 1, 5);
            presets.Add(tmp);
            presetNames.Add("Full Drag, Strict Area Ruling");
        }

        public void DisplaySelection()
        {
            GUILayout.BeginVertical();
            GUILayout.Label("Transonic Drag Settings");
            GUILayout.Label("Absolute magnitude of drag can be scaled, as can how lenient FAR is about enforcing proper area ruling.");

            GUILayout.BeginHorizontal();
            if (currentIndex >= 0)
            {
                dropdown.GUIDropDownDisplay(GUILayout.Width(300));
                settings = dropdown.ActiveSelection;
                currentIndex = settings.index;
            }
            else
            {
                GUILayout.BeginVertical();
                settings = customSettings;
                settings.fractionTransonicDrag = GUIUtils.TextEntryForDouble("Frac Mach 1 Drag: ", 150, settings.fractionTransonicDrag);
                GUILayout.Label("The below are used in controlling leniency of design.  Higher values for all will result in more leniency");
                settings.gaussianVehicleLengthFractionForSmoothing = GUIUtils.TextEntryForDouble("% Vehicle Length for Smoothing", 250, settings.gaussianVehicleLengthFractionForSmoothing);
                settings.numAreaSmoothingPasses = GUIUtils.TextEntryForInt("Smoothing Passes, Cross-Sectional Area", 250, settings.numAreaSmoothingPasses);
                if (settings.numAreaSmoothingPasses < 0)
                    settings.numAreaSmoothingPasses = 0;
                settings.numDerivSmoothingPasses = GUIUtils.TextEntryForInt("Smoothing Passes, area 2nd deriv", 250, settings.numDerivSmoothingPasses);
                if (settings.numDerivSmoothingPasses < 0)
                    settings.numDerivSmoothingPasses = 0;

                customSettings = settings;
                GUILayout.EndVertical();
            }
            if (GUILayout.Button(currentIndex < 0 ? "Switch Back To Presets" : "Choose Custom Settings"))
            {
                if (currentIndex >= 0)
                    currentIndex = -1;
                else
                    currentIndex = 4;
            }
            GUILayout.EndHorizontal();
            GUILayout.Label("Voxel Detail Settings; increasing these will improve accuracy at the cost of performance");

            voxelSettings.numVoxelsControllableVessel = GUIUtils.TextEntryForInt("Voxels Controllable Vessel: ", 200, voxelSettings.numVoxelsControllableVessel);
            if (voxelSettings.numVoxelsControllableVessel < 0)
                voxelSettings.numVoxelsControllableVessel = 100000;

            voxelSettings.numVoxelsDebrisVessel = GUIUtils.TextEntryForInt("Voxels Debris: ", 200, voxelSettings.numVoxelsDebrisVessel);
            if (voxelSettings.numVoxelsDebrisVessel < 0)
                voxelSettings.numVoxelsDebrisVessel = 5000;

            voxelSettings.minPhysTicksPerUpdate = GUIUtils.TextEntryForInt("Min Phys Ticks per Voxel Update: ", 200, voxelSettings.minPhysTicksPerUpdate);
            if (voxelSettings.minPhysTicksPerUpdate < 0)
                voxelSettings.minPhysTicksPerUpdate = 80;

            GUILayout.BeginHorizontal();
            GUILayout.Label("Use Higher Res SubVoxels: ");
            voxelSettings.useHigherResVoxelPoints = GUILayout.Toggle(voxelSettings.useHigherResVoxelPoints, voxelSettings.useHigherResVoxelPoints ? "High Res SubVoxels" : "Low Res SubVoxels");
            GUILayout.EndHorizontal();

            GUILayout.EndVertical();
        }
    }

    public class FARDifficultyAndExactnessSettings
    {
        public double fractionTransonicDrag = 0.7;
        public double gaussianVehicleLengthFractionForSmoothing = 0.015;
        public int numAreaSmoothingPasses = 2;
        public int numDerivSmoothingPasses = 1;
        public int index;


        public FARDifficultyAndExactnessSettings(int index)
        {
            this.index = index;
        }

        public FARDifficultyAndExactnessSettings(double transDrag, double gaussianLength, int areaPass, int derivPass, int index)
        {
            this.index = index;
            fractionTransonicDrag = transDrag;
            gaussianVehicleLengthFractionForSmoothing = gaussianLength;
            numAreaSmoothingPasses = areaPass;
            numDerivSmoothingPasses = derivPass;
        }
    }

    public class FARVoxelSettings
    {
        public int numVoxelsControllableVessel;
        public int numVoxelsDebrisVessel;
        public int minPhysTicksPerUpdate;
        public bool useHigherResVoxelPoints;

        public FARVoxelSettings() : this(250000, 20000, 80, true) { }

        public FARVoxelSettings(int vesselCount, int debrisCount, int minPhysTicks, bool higherResVoxPoints)
        {
            numVoxelsControllableVessel = vesselCount;
            numVoxelsDebrisVessel = debrisCount;
            minPhysTicksPerUpdate = minPhysTicks;
            useHigherResVoxelPoints = higherResVoxPoints;
        }
    }
}

