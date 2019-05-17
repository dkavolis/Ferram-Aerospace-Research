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

using System.Collections.Generic;
using System.Text;
using ferram4;
using FerramAerospaceResearch.FARAeroComponents;
using FerramAerospaceResearch.FARUtils;
using KSP.IO;
using KSP.Localization;
using StringLeakTest;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FARFlightGUI
{
    public class FlightGUI : VesselModule
    {
        private Vessel _vessel;
        private FARVesselAero _vesselAero;

        private static bool showGUI;
        public static bool showAllGUI = true;
        public static bool savedShowGUI = true;
        private static Rect mainGuiRect;
        private static Rect dataGuiRect;
        private static Rect settingsGuiRect;
        private static IButton blizzyFlightGUIButton;
        private static int activeFlightGUICount;
        private static int frameCountForSaving;
        public static Dictionary<Vessel, FlightGUI> vesselFlightGUI;

        private StringBuilder _strBuilder = new StringBuilder();

        private PhysicsCalcs _physicsCalcs;
        private VesselFlightInfo infoParameters;
        public VesselFlightInfo InfoParameters
        {
            get { return infoParameters; }
        }

        private FlightStatusGUI _flightStatusGUI;
        private StabilityAugmentation _stabilityAugmentation;
        private FlightDataGUI _flightDataGUI;
        private AeroVisualizationGUI _aeroVizGUI;
        public AeroVisualizationGUI AeroVizGUI
        {
            get { return _aeroVizGUI; }
        }

        private AirspeedSettingsGUI _airSpeedGUI;
        public AirspeedSettingsGUI airSpeedGUI
        {
            get { return _airSpeedGUI; }
        }

        private bool showFlightDataWindow;
        private bool showSettingsWindow;

        internal static GUIStyle boxStyle;
        internal static GUIStyle buttonStyle;

        private GUIDropDown<int> settingsWindow;

        protected override void OnAwake()
        {
            if (vesselFlightGUI == null)
            {
                vesselFlightGUI = new Dictionary<Vessel, FlightGUI>();
            }
        }

        protected override void OnStart()
        {
            base.OnStart();

            if (!CompatibilityChecker.IsAllCompatible())
            {
                enabled = false;
                return;
            }

            showGUI = savedShowGUI;
            //since we're sharing the button, we need these shenanigans now
            if (FARDebugAndSettings.FARDebugButtonStock && HighLogic.LoadedSceneIsFlight)
                if (showGUI)
                    FARDebugAndSettings.FARDebugButtonStock.SetTrue(false);
                else
                    FARDebugAndSettings.FARDebugButtonStock.SetFalse(false);


            _vessel = GetComponent<Vessel>();
            _vesselAero = GetComponent<FARVesselAero>();
            _physicsCalcs = new PhysicsCalcs(_vessel, _vesselAero);
            _flightStatusGUI = new FlightStatusGUI();
            _stabilityAugmentation = new StabilityAugmentation(_vessel);
            _flightDataGUI = new FlightDataGUI();
            _aeroVizGUI = new AeroVisualizationGUI();

            settingsWindow = new GUIDropDown<int>(new[] { Localizer.Format("FARFlightGUIWindowSelect0"), Localizer.Format("FARFlightGUIWindowSelect1"), Localizer.Format("FARFlightGUIWindowSelect2"), Localizer.Format("FARFlightGUIWindowSelect3") }, new[] { 0, 1, 2, 3 });
            //boxStyle.padding = new RectOffset(4, 4, 4, 4);

            if (vesselFlightGUI.ContainsKey(_vessel))
                vesselFlightGUI[_vessel] = this;
            else
                vesselFlightGUI.Add(_vessel, this);

            enabled = true;

            if(FARDebugValues.useBlizzyToolbar)
                GenerateBlizzyToolbarButton();

            activeFlightGUICount++;

            if(_vessel == FlightGlobals.ActiveVessel || FlightGlobals.ActiveVessel == null)
                LoadConfigs();

            GameEvents.onShowUI.Add(ShowUI);
            GameEvents.onHideUI.Add(HideUI);
        }

        private void OnDestroy()
        {
            FlightGUIDrawer.SetGUIActive(this,false);
            GameEvents.onShowUI.Remove(ShowUI);
            GameEvents.onHideUI.Remove(HideUI);
            SaveConfigs();
            if (_vessel)
            {
                vesselFlightGUI.Remove(_vessel);
            }
            _physicsCalcs = null;

            _flightDataGUI?.SaveSettings();
            _flightDataGUI = null;

            _stabilityAugmentation?.SaveAndDestroy();
            _stabilityAugmentation = null;

            _airSpeedGUI?.SaveSettings();
            _airSpeedGUI = null;

            _aeroVizGUI?.SaveSettings();

            _flightStatusGUI = null;
            settingsWindow = null;

            activeFlightGUICount--;

            if (activeFlightGUICount <= 0)
            {
                activeFlightGUICount = 0;
                if (blizzyFlightGUIButton != null)
                    ClearBlizzyToolbarButton();
            }

            savedShowGUI = showGUI;
        }

        public void SaveData()
        {
            if (_vessel == FlightGlobals.ActiveVessel)
            {
                SaveConfigs();
                _airSpeedGUI?.SaveSettings();
                _stabilityAugmentation?.SaveSettings();
                _flightDataGUI?.SaveSettings();
                _aeroVizGUI?.SaveSettings();
            }
        }
        public static void SaveActiveData()
        {
            FlightGUI gui;
            if (FlightGlobals.ready && FlightGlobals.ActiveVessel != null && vesselFlightGUI != null && vesselFlightGUI.TryGetValue(FlightGlobals.ActiveVessel, out gui))
            {
                if(gui != null)
                    gui.SaveData();
            }
        }

        //Receives message from FARVesselAero through _vessel on the recalc being completed
        public void UpdateAeroModules(List<FARAeroPartModule> newAeroModules, List<FARWingAerodynamicModel> legacyWingModels)
        {
            _physicsCalcs.UpdateAeroModules(newAeroModules, legacyWingModels);
        }

        //Receives a message from any FARWingAerodynamicModel or FARAeroPartModule that has failed to update the GUI
        // ReSharper disable once UnusedMember.Local
        private void AerodynamicFailureStatus()
        {
            _flightStatusGUI?.AerodynamicFailureStatus();
        }

        #region PhysicsAndOrientationBlock

        private void FixedUpdate()
        {
            if (_physicsCalcs == null)
                return;

            infoParameters = _physicsCalcs.UpdatePhysicsParameters();

            _stabilityAugmentation.UpdatePhysicsInfo(infoParameters);
            _flightStatusGUI.UpdateInfoParameters(infoParameters);
            _flightDataGUI.UpdateInfoParameters(infoParameters);
        }

        private void Update()
        {
            FlightGUIDrawer.SetGUIActive(this,(_vessel == FlightGlobals.ActiveVessel && showGUI && showAllGUI));
            if (frameCountForSaving >= 120)
            {
                SaveActiveData();
                frameCountForSaving = 0;
            }
            else
                frameCountForSaving++;
        }

        #endregion

        private void LateUpdate()
        {
            //OnGUIAppLauncherReady();
            if (_airSpeedGUI != null)
                _airSpeedGUI.ChangeSurfVelocity();
            else if (_vessel != null)
                _airSpeedGUI = new AirspeedSettingsGUI(_vessel);
        }

        #region GUI Functions

        public void DrawGUI()
        {
            GUI.skin = HighLogic.Skin;
            if(boxStyle == null)
            {
                boxStyle = new GUIStyle(GUI.skin.box);
                boxStyle.normal.textColor = boxStyle.focused.textColor = Color.white;
                boxStyle.hover.textColor = boxStyle.active.textColor = Color.yellow;
                boxStyle.onNormal.textColor = boxStyle.onFocused.textColor = boxStyle.onHover.textColor = boxStyle.onActive.textColor = Color.green;
                boxStyle.padding = new RectOffset(2, 2, 2, 2);
            }
            if (buttonStyle == null)
            {
                buttonStyle = new GUIStyle(GUI.skin.button);
                buttonStyle.normal.textColor = buttonStyle.focused.textColor = Color.white;
                buttonStyle.hover.textColor = buttonStyle.active.textColor = buttonStyle.onActive.textColor = Color.yellow;
                buttonStyle.onNormal.textColor = buttonStyle.onFocused.textColor = buttonStyle.onHover.textColor = Color.green;
                buttonStyle.padding = new RectOffset(2, 2, 2, 2);

            }
            if (_vessel == FlightGlobals.ActiveVessel && showGUI && showAllGUI)
            {
                mainGuiRect = GUILayout.Window(GetHashCode(), mainGuiRect, MainFlightGUIWindow, "FAR, " + FARVersion.VersionString, GUILayout.MinWidth(230));
                GUIUtils.ClampToScreen(mainGuiRect);

                if (showFlightDataWindow)
                {
                    dataGuiRect = GUILayout.Window(GetHashCode() + 1, dataGuiRect, FlightDataWindow, Localizer.Format("FARFlightDataTitle"), GUILayout.MinWidth(150));
                    GUIUtils.ClampToScreen(dataGuiRect);
                }

                if (showSettingsWindow)
                {
                    settingsGuiRect = GUILayout.Window(GetHashCode() + 2, settingsGuiRect, SettingsWindow, Localizer.Format("FARFlightSettings"), GUILayout.MinWidth(200));
                    GUIUtils.ClampToScreen(settingsGuiRect);
                }
            }
        }

        private void MainFlightGUIWindow(int windowId)
        {
            GUILayout.BeginVertical(GUILayout.Height(100));
            GUILayout.BeginHorizontal();
            _strBuilder.Length = 0;
            _strBuilder.Append(Localizer.Format("FARAbbrevMach"));
            _strBuilder.Append(": ");
            _strBuilder.Concat((float)(_vesselAero.MachNumber),3).AppendLine();
            _strBuilder.AppendFormat(Localizer.Format("FARFlightGUIReynolds"),_vesselAero.ReynoldsNumber);
            GUILayout.Box(_strBuilder.ToString(), boxStyle, GUILayout.ExpandWidth(true));
            GUILayout.EndHorizontal();

            _strBuilder.Length = 0;
            _strBuilder.Append(Localizer.Format("FARFlightGUIAtmDens"));
            _strBuilder.Concat((float)(vessel.atmDensity),3);

            GUILayout.Box(_strBuilder.ToString(), boxStyle, GUILayout.ExpandWidth(true));

            _flightStatusGUI.Display();
            showFlightDataWindow = GUILayout.Toggle(showFlightDataWindow, Localizer.Format("FARFlightGUIFltDataBtn"), buttonStyle, GUILayout.ExpandWidth(true));
            showSettingsWindow = GUILayout.Toggle(showSettingsWindow, Localizer.Format("FARFlightGUIFltSettings"), buttonStyle, GUILayout.ExpandWidth(true));

            GUILayout.Label(Localizer.Format("FARFlightGUIFltAssistance"));

            _stabilityAugmentation.Display();

            GUILayout.EndVertical();
            GUI.DragWindow();
        }

        private void FlightDataWindow(int windowId)
        {
            _flightDataGUI.DataDisplay();
            GUI.DragWindow();
        }

        private void SettingsWindow(int windowId)
        {
            GUILayout.Label(Localizer.Format("FARFlightSettingsLabel"));
            settingsWindow.GUIDropDownDisplay();
            int selection = settingsWindow.ActiveSelection;
            switch (selection)
            {
                case 0:
                    if (_flightDataGUI.SettingsDisplay())
                        dataGuiRect.height = 0;
                    break;
                case 1:
                    _stabilityAugmentation.SettingsDisplay();
                    break;
                case 2:
                    _airSpeedGUI.AirSpeedSettings();
                    break;
                case 3:
                    _aeroVizGUI.SettingsDisplay();
                    break;
            }
            GUI.DragWindow();
        }
        #endregion

        #region AppLauncher

        private static void ClearBlizzyToolbarButton()
        {
            blizzyFlightGUIButton.Destroy();
            blizzyFlightGUIButton = null;
        }

        private void GenerateBlizzyToolbarButton()
        {
            if (blizzyFlightGUIButton == null)
            {
                blizzyFlightGUIButton = ToolbarManager.Instance.add("FerramAerospaceResearch", "FARFlightButtonBlizzy");
                blizzyFlightGUIButton.TexturePath = "FerramAerospaceResearch/Textures/icon_button_blizzy";
                blizzyFlightGUIButton.ToolTip = "FAR Flight Sys";
                blizzyFlightGUIButton.OnClick += e => showGUI = !showGUI;
            }
        }

        public static void onAppLaunchToggle()
        {
            showGUI = !showGUI;
        }

        private void HideUI()
        {
            showAllGUI = false;
        }

        private void ShowUI()
        {
            showAllGUI = true;
        }
        #endregion

        private void SaveConfigs()
        {
            if (FARDebugAndSettings.config != null)
            {
                PluginConfiguration config = FARDebugAndSettings.config;
                config.SetValue("flight_mainGuiRect", mainGuiRect);
                config.SetValue("flight_dataGuiRect", dataGuiRect);
                config.SetValue("flight_settingsGuiRect", settingsGuiRect);
            }
        }

        private void LoadConfigs()
        {
            PluginConfiguration config = FARDebugAndSettings.config;
            mainGuiRect = config.GetValue("flight_mainGuiRect", new Rect());
            dataGuiRect = config.GetValue("flight_dataGuiRect", new Rect());
            settingsGuiRect = config.GetValue("flight_settingsGuiRect", new Rect());
        }
    }
}
