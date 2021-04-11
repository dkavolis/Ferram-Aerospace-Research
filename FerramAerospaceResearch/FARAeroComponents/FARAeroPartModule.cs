/*
Ferram Aerospace Research v0.16.0.2 "Mader"
=========================
Aerodynamics model for Kerbal Space Program

Copyright 2020, Michael Ferrara, aka Ferram4

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
using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using FerramAerospaceResearch.FARGUI.FARFlightGUI;
using FerramAerospaceResearch.FARPartGeometry;
using FerramAerospaceResearch.RealChuteLite;
using FerramAerospaceResearch.Settings;
using KSP.Localization;
using UnityEngine;

namespace FerramAerospaceResearch.FARAeroComponents
{
    //Used to hold relevant aero data for each part before applying it
    public class FARAeroPartModule : PartModule, ILiftProvider
    {
        public static float waterSlowDragNew = -1;
        public static float minVelVesselMultNew;

        // ReSharper disable once ConvertToConstant.Local
        private readonly bool partStressOverride = false; // currently not changed
        public Vector3 partLocalVel;
        public Vector3 partLocalVelNorm;
        public Vector3 partLocalAngVel;

        private Vector3 worldSpaceVelNorm;
        public Vector3 worldSpaceAeroForce;
        public Vector3 worldSpaceTorque;

        public Vector3 totalWorldSpaceAeroForce;

        private Vector3 partLocalForce;
        private Vector3 partLocalTorque;

        public float hackWaterDragVal;

        private ProjectedArea projectedArea;
        private double partStressMaxY = double.MaxValue;
        private double partStressMaxXZ = double.MaxValue;
        private double partForceMaxY = double.MaxValue;
        private double partForceMaxXZ = double.MaxValue;

        private ArrowPointer liftArrow;
        private ArrowPointer dragArrow;
        private ArrowPointer momentArrow;

        private DummyAirstreamShield shield;

        public const string GroupName = "FARAeroGroup";
        public const string GroupDisplayName = "FAR Debug";
        private bool fieldsVisible;
        private bool thermalFieldsVisible;
        public readonly List<BaseField> ThermalFields = new List<BaseField>();

        // ReSharper disable once NotAccessedField.Global -> unity
        [KSPField(isPersistant = false,
                  guiActive = false,
                  guiActiveEditor = false,
                  guiFormat = "F3",
                  guiUnits = "FARUnitKN",
                  groupName = GroupName,
                  groupDisplayName = GroupDisplayName)]
        public float dragForce;

        // ReSharper disable once NotAccessedField.Global -> unity
        [KSPField(isPersistant = false,
                  guiActive = false,
                  guiActiveEditor = false,
                  guiFormat = "F3",
                  guiUnits = "FARUnitKN",
                  groupName = GroupName)]
        public float liftForce;

        [KSPField(guiFormat = "F3", guiUnits = "FARUnitMSq", groupName = GroupName, groupDisplayName = GroupDisplayName)]
        public double radiativeArea;
        [KSPField(guiFormat = "F3", guiUnits = "FARUnitMSq", groupName = GroupName)]
        public double convectionArea;
        [KSPField(guiFormat = "F1", guiUnits = "K", groupName = GroupName)]
        public double exposedSkinTemp;
        [KSPField(guiFormat = "F1", guiUnits = "K", groupName = GroupName)]
        public double unexposedSkinTemp;
        [KSPField(guiFormat = "F1", guiUnits = "K", groupName = GroupName)]
        public double partTemp;
        [KSPField(guiFormat = "F1", guiUnits = "K", groupName = GroupName)]
        public double atmosphereTemp;
        [KSPField(guiFormat = "F1", guiUnits = "K", groupName = GroupName)]
        public double externalTemp;
        [KSPField(guiFormat = "F1", guiUnits = "K", groupName = GroupName)]
        public double exposedBackgroundTemp;

        [KSPField(guiFormat = "F3", groupName = GroupName)]
        public double convergenceFactor;
        [KSPField(guiFormat = "F3", groupName = GroupName)]
        public double skinSkinConductionMult;
        [KSPField(guiFormat = "F3", groupName = GroupName)]
        public double skinSkinConductionFactor;
        [KSPField(guiFormat = "F1", guiUnits = "kW", groupName = GroupName)]
        public double convectionFlux;
        [KSPField(guiFormat = "F3", groupName = GroupName)]
        public double finalConvCoeff;
        [KSPField(guiFormat = "F1", guiUnits = "kW", groupName = GroupName)]
        public double intConductionFlux;
        [KSPField(guiFormat = "F1", guiUnits = "kW", groupName = GroupName)]
        public double skinConductionFlux;
        [KSPField(guiFormat = "F1", guiUnits = "kW", groupName = GroupName)]
        public double skinInternalConductionFlux;
        [KSPField(guiFormat = "F1", guiUnits = "kW", groupName = GroupName)]
        public double unexpSkinInternalConductionFlux;
        [KSPField(guiFormat = "P1", groupName = GroupName)]
        public double skinExposedAreaFrac;
        [KSPField(guiFormat = "F1", guiUnits = "kW", groupName = GroupName)]
        public double radiationFlux;
        [KSPField(guiFormat = "F1", guiUnits = "kW", groupName = GroupName)]
        public double unexpRadiationFlux;
        [KSPField(guiFormat = "F1", guiUnits = "kW", groupName = GroupName)]
        public double skinSkinConductionFlux;

        private Transform partTransform;

        private MaterialColorUpdater materialColorUpdater;
        private ModuleLiftingSurface stockAeroSurfaceModule;
        private bool updateVisualization;
        public FARWingAerodynamicModel LegacyWingModel { get; private set; }

        public ProjectedArea ProjectedAreas
        {
            get { return projectedArea; }
        }

        // TODO 1.2: provide actual implementation of new 1.2 methods

        public bool DisableBodyLift
        {
            get { return false; }
        }

        public bool IsLifting
        {
            get { return true; }
        }

        public void OnCenterOfLiftQuery(CenterOfLiftQuery CoLMarker)
        {
            // Compute the actual center ourselves once per frame
            // Feed the precomputed values to the vanilla indicator
            CoLMarker.pos = EditorAeroCenter.VesselRootLocalAeroCenter; //hacking the old stuff to work with the new
            CoLMarker.pos = EditorLogic.RootPart.partTransform.localToWorldMatrix.MultiplyPoint3x4(CoLMarker.pos);
            CoLMarker.dir = Vector3.zero;
            CoLMarker.lift = 1;
        }

        public void SetShielded(bool value)
        {
            part.ShieldedFromAirstream = value;
            if (!value)
            {
                part.RemoveShield(shield);
                return;
            }

            part.AddShield(shield);
            worldSpaceAeroForce = Vector3.zero;
            worldSpaceTorque = Vector3.zero;

            totalWorldSpaceAeroForce = Vector3.zero;

            partLocalForce = Vector3.zero;
            partLocalTorque = Vector3.zero;

            partLocalAngVel = Vector3.zero;
            partLocalVel = Vector3.zero;
            partLocalVelNorm = Vector3.zero;

            part.dragVectorMag = 0f;
            part.dragVectorDir = Vector3.zero;
            part.dragVectorDirLocal = Vector3.zero;
            part.dragScalar = 0f;

            UpdateAeroDisplay();
        }

        public void ForceLegacyAeroUpdates()
        {
            if (LegacyWingModel != null)
                LegacyWingModel.ForceOnVesselPartsChange();
        }


        public void SetProjectedArea(ProjectedArea areas, Matrix4x4 vesselToWorldMatrix)
        {
            var transformedArea = new ProjectedArea();
            if (!part)
                return;

            Matrix4x4 transformMatrix = part.partTransform.worldToLocalMatrix * vesselToWorldMatrix;

            for (int i = 0; i < 6; i++)
                IncrementAreas(ref transformedArea, (float)areas[i] * ProjectedArea.FaceDirections[i], transformMatrix);

            projectedArea = transformedArea;
            projectedArea.totalArea = projectedArea.iN +
                                      projectedArea.iP +
                                      projectedArea.jN +
                                      projectedArea.jP +
                                      projectedArea.kN +
                                      projectedArea.kP;

            if (projectedArea.totalArea <= 0)
            {
                part.ShieldedFromAirstream = true;
                part.AddShield(shield);

                if (!(liftArrow is null))
                {
                    Destroy(liftArrow);
                    liftArrow = null;
                }

                if (!(dragArrow is null))
                {
                    Destroy(dragArrow);
                    dragArrow = null;
                }

                if (!(momentArrow is null))
                {
                    Destroy(momentArrow);
                    momentArrow = null;
                }
            }
            else
            {
                part.ShieldedFromAirstream = false;
                part.RemoveShield(shield);
            }

            double areaForStress = projectedArea.totalArea / 6;
            if (!FARDebugValues.allowStructuralFailures ||
                areaForStress <= 0.1 ||
                part.Modules.Contains<RealChuteFAR>() ||
                part.Modules.Contains<ModuleAblator>())
            {
                partForceMaxY = double.MaxValue;
                partForceMaxXZ = double.MaxValue;
                return;
            }

            partForceMaxY = areaForStress * partStressMaxY;
            partForceMaxXZ = areaForStress * partStressMaxXZ;
        }

        private static void IncrementAreas(ref ProjectedArea data, Vector3 vector, Matrix4x4 transformMatrix)
        {
            vector = transformMatrix.MultiplyVector(vector);

            if (vector.x >= 0)
                data.iP += vector.x;
            else
                data.iN -= vector.x;

            if (vector.y >= 0)
                data.jP += vector.y;
            else
                data.jN -= vector.y;

            if (vector.z >= 0)
                data.kP += vector.z;
            else
                data.kN -= vector.z;
        }

        private void Start()
        {
            shield = new DummyAirstreamShield {part = part};

            if (waterSlowDragNew < 0)
            {
                waterSlowDragNew = PhysicsGlobals.BuoyancyWaterDragSlow;
                minVelVesselMultNew = (float)PhysicsGlobals.BuoyancyWaterDragPartVelGreaterVesselMult;
                PhysicsGlobals.BuoyancyWaterDragPartVelGreaterVesselMult = 0;
            }

            part.maximum_drag = 0;
            part.minimum_drag = 0;
            part.angularDrag = 0;
            if (HighLogic.LoadedSceneIsFlight)
                enabled = true;
            else if (HighLogic.LoadedSceneIsEditor)
                enabled = false;

            partLocalVel = Vector3.zero;
            partLocalForce = Vector3.zero;
            partLocalTorque = Vector3.zero;

            if (FARDebugValues.allowStructuralFailures && !partStressOverride)
            {
                FARPartStressTemplate template = FARAeroStress.DetermineStressTemplate(part);
                partStressMaxY = template.YMaxStress;
                partStressMaxXZ = template.XZMaxStress;
            }

            partTransform = part.partTransform;

            materialColorUpdater = new MaterialColorUpdater(partTransform, PhysicsGlobals.TemperaturePropertyID);
            if (part.Modules.Contains<FARWingAerodynamicModel>())
                LegacyWingModel = part.Modules.GetModule<FARWingAerodynamicModel>();
            else if (part.Modules.Contains<FARControllableSurface>())
                LegacyWingModel = part.Modules.GetModule<FARControllableSurface>();
            else
                LegacyWingModel = null;

            // For handling airbrakes aero visualization
            stockAeroSurfaceModule = part.Modules.Contains<ModuleAeroSurface>()
                                         ? part.Modules.GetModule<ModuleAeroSurface>()
                                         : null;
            AddThermalDebugFields();
        }

        private void AddThermalDebugFields()
        {
            ThermalFields.Clear();
            ThermalFields.Add(Fields[nameof(convergenceFactor)]);
            ThermalFields.Add(Fields[nameof(skinSkinConductionMult)]);
            ThermalFields.Add(Fields[nameof(skinSkinConductionFactor)]);
            ThermalFields.Add(Fields[nameof(convectionArea)]);
            ThermalFields.Add(Fields[nameof(radiativeArea)]);
            ThermalFields.Add(Fields[nameof(skinExposedAreaFrac)]);
            ThermalFields.Add(Fields[nameof(exposedSkinTemp)]);
            ThermalFields.Add(Fields[nameof(unexposedSkinTemp)]);
            ThermalFields.Add(Fields[nameof(partTemp)]);
            ThermalFields.Add(Fields[nameof(externalTemp)]);
            ThermalFields.Add(Fields[nameof(atmosphereTemp)]);
            ThermalFields.Add(Fields[nameof(exposedBackgroundTemp)]);
            ThermalFields.Add(Fields[nameof(convectionFlux)]);
            ThermalFields.Add(Fields[nameof(finalConvCoeff)]);
            ThermalFields.Add(Fields[nameof(intConductionFlux)]);
            ThermalFields.Add(Fields[nameof(skinConductionFlux)]);
            ThermalFields.Add(Fields[nameof(skinInternalConductionFlux)]);
            ThermalFields.Add(Fields[nameof(skinSkinConductionFlux)]);
            ThermalFields.Add(Fields[nameof(unexpSkinInternalConductionFlux)]);
            ThermalFields.Add(Fields[nameof(radiationFlux)]);
            ThermalFields.Add(Fields[nameof(unexpRadiationFlux)]);
        }
        private void SetThermalFieldsVisibility(bool enabled)
        {
            thermalFieldsVisible = enabled;
            foreach (BaseField f in ThermalFields)
                f.guiActive = enabled;
        }
        private void UpdateThermalDebugFields()
        {
            convergenceFactor = PhysicsGlobals.ThermalConvergenceFactor;
            skinSkinConductionMult = part.skinSkinConductionMult;
            skinSkinConductionFactor = PhysicsGlobals.SkinSkinConductionFactor;
            convectionArea = part.ptd.convectionArea;
            radiativeArea = 1f / part.ptd.radAreaRecip;
            skinExposedAreaFrac = part.skinExposedAreaFrac;
            exposedSkinTemp = part.skinTemperature;
            unexposedSkinTemp = part.skinUnexposedTemperature;
            partTemp = part.temperature;
            externalTemp = FlightIntegrator.ActiveVesselFI.externalTemperature;
            atmosphereTemp = FlightIntegrator.ActiveVesselFI.atmosphericTemperature;
            exposedBackgroundTemp = part.ptd.brtExposed;
            //bodyArea = FlightIntegrator.ActiveVesselFI.GetBodyArea(part.ptd);
            //sunArea = FlightIntegrator.ActiveVesselFI.GetSunArea(part.ptd);
            convectionFlux = part.ptd.convectionFlux;
            finalConvCoeff = part.ptd.finalCoeff;
            intConductionFlux = part.ptd.intConductionFlux;
            skinConductionFlux = part.ptd.skinConductionFlux;
            skinInternalConductionFlux = part.ptd.skinInteralConductionFlux;
            skinSkinConductionFlux = part.ptd.skinSkinConductionFlux;
            unexpSkinInternalConductionFlux = part.ptd.unexpSkinInternalConductionFlux;
            //expFlux = part.ptd.expFlux;
            //unexpFlux = part.ptd.unexpFlux;
            radiationFlux = part.ptd.radiationFlux;
            unexpRadiationFlux = part.ptd.unexpRadiationFlux;
        }

        public double ProjectedAreaWorld(Vector3 normalizedDirectionVector)
        {
            return ProjectedAreaLocal(partTransform.worldToLocalMatrix.MultiplyVector(normalizedDirectionVector));
        }

        public double ProjectedAreaLocal(Vector3 normalizedDirectionVector)
        {
            double area = 0;
            if (normalizedDirectionVector.x > 0)
                area += normalizedDirectionVector.x * projectedArea.iP;
            else
                area -= normalizedDirectionVector.x * projectedArea.iN;

            if (normalizedDirectionVector.y > 0)
                area += normalizedDirectionVector.y * projectedArea.jP;
            else
                area -= normalizedDirectionVector.y * projectedArea.jN;

            if (normalizedDirectionVector.z > 0)
                area += normalizedDirectionVector.z * projectedArea.kP;
            else
                area -= normalizedDirectionVector.z * projectedArea.kN;

            return area;
        }

        public void Update()
        {
            CalculateTotalAeroForce();

            AeroVisualizationGUI aeroVizGUI = null;

            if (FlightGUI.vesselFlightGUI != null &&
                vessel != null &&
                FlightGUI.vesselFlightGUI.TryGetValue(vessel, out FlightGUI flightGUI))
                aeroVizGUI = flightGUI.AeroVizGUI;

            if (aeroVizGUI == null)
                return;

            bool anyActive = aeroVizGUI.AnyVisualizationActive;
            if (!anyActive && !updateVisualization ||
                !HighLogic.LoadedSceneIsFlight ||
                PhysicsGlobals.ThermalColorsDebug)
                return;
            Color tintColor = AeroVisualizationTintingCalculation(aeroVizGUI);
            materialColorUpdater.Update(tintColor);

            // this will disable visualization if none are active delayed by 1 frame to clean up any tint
            updateVisualization = anyActive;
        }

        //Do this so FlightGUI can read off of the numbers from this
        private void CalculateTotalAeroForce()
        {
            if (projectedArea.totalArea <= 0.0)
                return;
            totalWorldSpaceAeroForce = worldSpaceAeroForce;

            // Combine forces from legacy wing model
            if (LegacyWingModel != null)
                totalWorldSpaceAeroForce += LegacyWingModel.worldSpaceForce;

            // Handle airbrakes
            if (stockAeroSurfaceModule != null)
                totalWorldSpaceAeroForce += stockAeroSurfaceModule.dragForce + stockAeroSurfaceModule.liftForce;
        }

        //Returns the tinted color if active; else it returns an alpha 0 color
        private Color AeroVisualizationTintingCalculation(AeroVisualizationGUI aeroVizGUI)
        {
            // Disable tinting for low dynamic pressure to prevent flicker
            if (vessel.dynamicPressurekPa <= 0.00001)
                return new Color(0, 0, 0, 0);

            // Stall tinting overrides Cl / Cd tinting
            if (LegacyWingModel != null && aeroVizGUI.TintForStall)
                return new Color((float)(LegacyWingModel.GetStall() * 100.0 / aeroVizGUI.FullySaturatedStall),
                                 0f,
                                 0f,
                                 0.5f);

            if (!aeroVizGUI.TintForCl && !aeroVizGUI.TintForCd)
                return new Color(0, 0, 0, 0);

            double visualizationCl = 0, visualizationCd = 0;

            if (projectedArea.totalArea > 0.0)
            {
                Vector3 worldVelNorm = partTransform.localToWorldMatrix.MultiplyVector(partLocalVelNorm);
                Vector3 worldDragArrow = Vector3.Dot(totalWorldSpaceAeroForce, worldVelNorm) * worldVelNorm;
                Vector3 worldLiftArrow = totalWorldSpaceAeroForce - worldDragArrow;

                double invAndDynPresArea = LegacyWingModel != null ? LegacyWingModel.S : projectedArea.totalArea;
                invAndDynPresArea *= vessel.dynamicPressurekPa;
                invAndDynPresArea = 1 / invAndDynPresArea;
                visualizationCl = worldLiftArrow.magnitude * invAndDynPresArea;
                visualizationCd = worldDragArrow.magnitude * invAndDynPresArea;
            }

            double fullSatCl, satCl = 0, fullSatCd, satCd = 0;

            if (LegacyWingModel != null)
            {
                fullSatCl = aeroVizGUI.FullySaturatedCl;
                fullSatCd = aeroVizGUI.FullySaturatedCd;
            }
            else
            {
                fullSatCl = aeroVizGUI.FullySaturatedClBody;
                fullSatCd = aeroVizGUI.FullySaturatedCdBody;
            }

            if (aeroVizGUI.TintForCl)
                satCl = Math.Abs(visualizationCl / fullSatCl);
            if (aeroVizGUI.TintForCd)
                satCd = Math.Abs(visualizationCd / fullSatCd);

            return new Color((float)satCd, 0.5f * (float)(satCl + satCd), (float)satCl, 0.5f);
        }

        public void ApplyForces()
        {
            if (!part)
                return;

            if (float.IsNaN(partLocalForce.sqrMagnitude))
                partLocalForce = Vector3.zero;
            if (float.IsNaN(partLocalTorque.sqrMagnitude))
                partLocalTorque = Vector3.zero;

            Vector3 localForceTemp = Vector3.Dot(partLocalVelNorm, partLocalForce) * partLocalVelNorm;

            partLocalForce = localForceTemp * part.dragScalar + (partLocalForce - localForceTemp) * part.bodyLiftScalar;
            partLocalTorque *= part.dragScalar;

            part.dragScalar = 0;
            part.bodyLiftScalar = 0;

            if (!vessel.packed)
                CheckAeroStressFailure();

            Rigidbody rb = part.Rigidbody;

            if (!rb)
                return;

            worldSpaceAeroForce = partTransform.TransformDirection(partLocalForce);
            worldSpaceTorque = partTransform.TransformDirection(partLocalTorque);
            UpdateAeroDisplay();

            if (part.submergedPortion <= 0)
            {
                part.AddForce(worldSpaceAeroForce);
            }
            else
            {
                Vector3 worldSpaceDragForce = Vector3.Dot(worldSpaceVelNorm, worldSpaceAeroForce) * worldSpaceVelNorm;
                Vector3 worldSpaceLiftForce = worldSpaceAeroForce - worldSpaceDragForce;

                Vector3 waterDragForce, waterLiftForce;
                if (part.submergedPortion < 1)
                {
                    float waterFraction = (float)(part.submergedDynamicPressurekPa * part.submergedPortion +
                                                  part.dynamicPressurekPa * (1 - part.submergedPortion));
                    waterFraction = (float)(part.submergedDynamicPressurekPa * part.submergedPortion) / waterFraction;

                    waterDragForce = worldSpaceDragForce * waterFraction; //calculate areaDrag vector
                    waterLiftForce = worldSpaceLiftForce * waterFraction;

                    worldSpaceDragForce -= waterDragForce; //remove water drag from this
                    worldSpaceLiftForce -= waterLiftForce;

                    waterDragForce *= (float)Math.Min(part.submergedDragScalar, 1);
                    waterLiftForce *= (float)part.submergedLiftScalar;
                }
                else
                {
                    waterDragForce = worldSpaceDragForce * (float)Math.Min(part.submergedDragScalar, 1);
                    waterLiftForce = worldSpaceLiftForce * (float)part.submergedLiftScalar;
                    worldSpaceDragForce = Vector3.zero;
                }

                hackWaterDragVal += Math.Abs(waterDragForce.magnitude / (rb.mass * rb.velocity.magnitude));

                if (!float.IsNaN(worldSpaceDragForce.x))
                    part.AddForce(worldSpaceDragForce + worldSpaceLiftForce + waterLiftForce);

                worldSpaceAeroForce = worldSpaceDragForce + worldSpaceLiftForce + waterDragForce + waterLiftForce;
            }

            part.AddTorque(worldSpaceTorque);

            partLocalForce = Vector3.zero;
            partLocalTorque = Vector3.zero;
        }

        //just to make water drag work in some possibly sane way
        public void FixedUpdate()
        {
            if (waterSlowDragNew <= 0 || !vessel || part.submergedPortion <= 0)
                return;
            PhysicsGlobals.BuoyancyWaterDragSlow = Math.Max(hackWaterDragVal, 0f);
            hackWaterDragVal = 0;

            float vel = partLocalVel.magnitude;

            if (vel < PhysicsGlobals.BuoyancyWaterDragMinVel || vel > vessel.srfSpeed * minVelVesselMultNew)
                PhysicsGlobals.BuoyancyWaterDragSlow += waterSlowDragNew;
        }

        public void AddLocalForce(Vector3 localForce, Vector3 localLocation)
        {
            partLocalForce += localForce;
            partLocalTorque += Vector3.Cross(localLocation - part.CoMOffset, localForce);
        }

        public void AddLocalForceAndTorque(Vector3 localForce, Vector3 localTorque, Vector3 localLocation)
        {
            Vector3 localRadVector = localLocation - part.CoMOffset;
            partLocalForce += localForce;
            partLocalTorque += Vector3.Cross(localRadVector, localForce);

            partLocalTorque += localTorque;
        }

        public void UpdateVelocityAndAngVelocity(Vector3 frameVel)
        {
            if (partTransform is null)
                if (part != null)
                    partTransform = part.partTransform;
                else
                    return;

            if (part == null)
                return;

            Rigidbody rb = part.Rigidbody;

            if (rb == null)
                return;

            //world velocity
            partLocalVel = rb.velocity + frameVel - FARAtmosphere.GetWind(FARAeroUtil.CurrentBody, part, rb.position);

            worldSpaceVelNorm = partLocalVel.normalized;
            partLocalVel = partTransform.InverseTransformDirection(partLocalVel);

            partLocalVelNorm = partLocalVel.normalized;

            partLocalAngVel = rb.angularVelocity;
            partLocalAngVel = partTransform.InverseTransformDirection(partLocalAngVel);
        }

        private void CheckAeroStressFailure()
        {
            if (partForceMaxY * (1 + part.submergedPortion * 1000) < partLocalForce.y ||
                Vector3.ProjectOnPlane(partLocalForce, Vector3.up).magnitude >
                partForceMaxXZ * (1 + part.submergedPortion * 1000))
                ApplyAeroStressFailure();
        }

        private void ApplyAeroStressFailure()
        {
            bool failureOccured = false;
            if (part.Modules.Contains<ModuleProceduralFairing>())
            {
                ModuleProceduralFairing fairing = part.Modules.GetModule<ModuleProceduralFairing>();
                fairing.ejectionForce = 0.5f;

                fairing.DeployFairing();
                failureOccured = true;
            }

            List<Part> children = part.children;
            // ReSharper disable once ForCanBeConvertedToForeach -> decouple modifies collection
            for (int i = 0; i < children.Count; i++)
            {
                Part child = children[i];
                child.decouple(25);

                failureOccured = true;
            }

            if (part.parent)
            {
                part.decouple(25);
                failureOccured = true;
            }

            if (!failureOccured || !vessel)
                return;
            vessel.SendMessage("AerodynamicFailureStatus");
            string msg = Localizer.Format("FARFlightLogAeroFailure",
                                          KSPUtil.PrintTimeStamp(FlightLogger.met),
                                          part.partInfo.title);
            FlightLogger.eventLog.Add(msg);
            if (FARDebugValues.aeroFailureExplosions)
                FXMonger.Explode(part, partTransform.position, (float)projectedArea.totalArea * 0.0005f);
        }

        private void UpdateAeroDisplay()
        {
            Vector3 worldDragArrow = Vector3.zero;
            Vector3 worldLiftArrow = Vector3.zero;

            if ((PhysicsGlobals.AeroForceDisplay || PhysicsGlobals.AeroDataDisplay) && !part.ShieldedFromAirstream)
            {
                Vector3 worldVelNorm = partTransform.localToWorldMatrix.MultiplyVector(partLocalVelNorm);
                worldDragArrow = Vector3.Dot(worldSpaceAeroForce, worldVelNorm) * worldVelNorm;
                worldLiftArrow = worldSpaceAeroForce - worldDragArrow;
            }

            if (PhysicsGlobals.AeroForceDisplay && !part.ShieldedFromAirstream)
            {
                if (liftArrow == null)
                {
                    liftArrow = ArrowPointer.Create(partTransform,
                                                    Vector3.zero,
                                                    worldLiftArrow,
                                                    worldLiftArrow.magnitude *
                                                    FARKSPAddonFlightScene.FARAeroForceDisplayScale,
                                                    FARConfig.GUIColors.ClColor,
                                                    true);
                }
                else
                {
                    liftArrow.Direction = worldLiftArrow;
                    liftArrow.Length = worldLiftArrow.magnitude * FARKSPAddonFlightScene.FARAeroForceDisplayScale;
                }

                if (dragArrow == null)
                {
                    dragArrow = ArrowPointer.Create(partTransform,
                                                    Vector3.zero,
                                                    worldDragArrow,
                                                    worldDragArrow.magnitude *
                                                    FARKSPAddonFlightScene.FARAeroForceDisplayScale,
                                                    FARConfig.GUIColors.CdColor,
                                                    true);
                }
                else
                {
                    dragArrow.Direction = worldDragArrow;
                    dragArrow.Length = worldDragArrow.magnitude * FARKSPAddonFlightScene.FARAeroForceDisplayScale;
                }

                if (FARDebugValues.showMomentArrows)
                {
                    if (momentArrow == null)
                    {
                        momentArrow = ArrowPointer.Create(partTransform,
                                                          Vector3.zero,
                                                          worldSpaceTorque,
                                                          worldSpaceTorque.magnitude *
                                                          FARKSPAddonFlightScene.FARAeroForceDisplayScale,
                                                          FARConfig.GUIColors.CmColor,
                                                          true);
                    }
                    else
                    {
                        momentArrow.Direction = -worldSpaceTorque;
                        momentArrow.Length =
                            worldSpaceTorque.magnitude * FARKSPAddonFlightScene.FARAeroForceDisplayScale;
                    }
                }
            }
            else
            {
                if (!(liftArrow is null))
                {
                    Destroy(liftArrow);
                    liftArrow = null;
                }

                if (!(dragArrow is null))
                {
                    Destroy(dragArrow);
                    dragArrow = null;
                }

                if (!(momentArrow is null))
                {
                    Destroy(momentArrow);
                    momentArrow = null;
                }
            }
            if (PhysicsGlobals.AeroDataDisplay && !part.ShieldedFromAirstream)
            {
                if (!fieldsVisible)
                {
                    Fields[nameof(dragForce)].guiActive = true;
                    Fields[nameof(liftForce)].guiActive = true;
                    fieldsVisible = true;
                }

                dragForce = worldDragArrow.magnitude;
                liftForce = worldLiftArrow.magnitude;
            }
            else if (fieldsVisible)
            {
                Fields[nameof(dragForce)].guiActive = false;
                Fields[nameof(liftForce)].guiActive = false;
                fieldsVisible = false;
            }
        }

        public override void OnUpdate()
        {
            if (PhysicsGlobals.ThermalDataDisplay != thermalFieldsVisible)
                SetThermalFieldsVisibility(PhysicsGlobals.ThermalDataDisplay);
            if (PhysicsGlobals.ThermalDataDisplay)
                UpdateThermalDebugFields();
        }

        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);

            if (!FARDebugValues.allowStructuralFailures || !node.HasNode("FARPartStressTemplate"))
                return;
            ConfigNode stressTemplate = node.GetNode("FARPartStressTemplate");
            FARPartStressTemplate defaultTemplate = FARAeroStress.DetermineStressTemplate(part);
            if (stressTemplate.HasValue("YmaxStress"))
                if (!double.TryParse(stressTemplate.GetValue("YmaxStress"), out partStressMaxY))
                    partStressMaxY = defaultTemplate.YMaxStress;
            // ReSharper disable once InvertIf
            if (stressTemplate.HasValue("XZmaxStress"))
                if (!double.TryParse(stressTemplate.GetValue("XZmaxStress"), out partStressMaxXZ))
                    partStressMaxXZ = defaultTemplate.XZMaxStress;
        }

        private void OnDestroy()
        {
            if (liftArrow != null)
            {
                Destroy(liftArrow);
                liftArrow = null;
            }

            if (dragArrow != null)
            {
                Destroy(dragArrow);
                dragArrow = null;
            }

            if (momentArrow != null)
            {
                Destroy(momentArrow);
                momentArrow = null;
            }

            LegacyWingModel = null;
            stockAeroSurfaceModule = null;
        }

        public struct ProjectedArea
        {
            public double iP, iN; //area in x direction
            public double jP, jN; //area in y direction
            public double kP, kN; //area in z direction
            public double totalArea;

            public static readonly Vector3d[] FaceDirections =
            {
                Vector3d.right, Vector3d.left, Vector3d.up, Vector3d.down, Vector3d.forward, Vector3d.back
            };

            // a map onto stock cube faces, adds a bit of robustness when mapping from ProjectedAreas to drag cubes faces arrays
            public static readonly int[] FaceMap =
            {
                (int)DragCube.DragFace.XP,
                (int)DragCube.DragFace.XN,
                (int)DragCube.DragFace.YP,
                (int)DragCube.DragFace.YN,
                (int)DragCube.DragFace.ZP,
                (int)DragCube.DragFace.ZN
            };

            // since this is POD struct, use pointer casting for quick indexed access
            public unsafe double this[int index]
            {
                get
                {
                    FARLogger.Assert(index < 7, "Index out of bounds");
                    fixed (ProjectedArea* areas = &this)
                    {
                        return ((double*)areas)[index];
                    }
                }
                set
                {
                    FARLogger.Assert(index < 7, "Index out of bounds");
                    fixed (ProjectedArea* areas = &this)
                    {
                        ((double*)areas)[index] = value;
                    }
                }
            }

            public static ProjectedArea operator +(ProjectedArea a, ProjectedArea b)
            {
                a.iN += b.iN;
                a.iP += b.iP;
                a.jN += b.jN;
                a.jP += b.jP;
                a.kN += b.kN;
                a.kP += b.kP;
                return a;
            }

            public static ProjectedArea operator +(ProjectedArea a, VoxelCrossSection.SideAreaValues b)
            {
                a.iN += b.iN;
                a.iP += b.iP;
                a.jN += b.jN;
                a.jP += b.jP;
                a.kN += b.kN;
                a.kP += b.kP;
                return a;
            }

            public static implicit operator ProjectedArea(VoxelCrossSection.SideAreaValues b)
            {
                var a = new ProjectedArea
                {
                    iN = b.iN,
                    iP = b.iP,
                    jN = b.jN,
                    jP = b.jP,
                    kN = b.kN,
                    kP = b.kP
                };
                return a;
            }
        }
    }
}
