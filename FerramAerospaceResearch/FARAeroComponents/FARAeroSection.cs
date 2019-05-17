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

using System;
using System.Collections.Generic;
using ferram4;
using FerramAerospaceResearch.FARPartGeometry;
using UnityEngine;

namespace FerramAerospaceResearch.FARAeroComponents
{
    internal class FARAeroSection
    {
        private static FloatCurve crossFlowDragMachCurve;
        private static FloatCurve crossFlowDragReynoldsCurve;

        public FARFloatCurve xForcePressureAoA0;
        public FARFloatCurve xForcePressureAoA180;
        public FARFloatCurve xForceSkinFriction;
        private float potentialFlowNormalForce;
        private float viscCrossflowDrag;
        private float flatnessRatio;
        private float invFlatnessRatio;
        private float hypersonicMomentForward;
        private float hypersonicMomentBackward;
        private float diameter;

        private float mergeFactor;
        private Vector3 worldNormalVector;

        private List<PartData> partData;
        private Dictionary<FARAeroPartModule, int> handledAeroModulesIndexDict;

        public struct PartData
        {
            public FARAeroPartModule aeroModule;
            public Vector3 centroidPartSpace;
            public Vector3 xRefVectorPartSpace;
            public Vector3 nRefVectorPartSpace;
            public float dragFactor;    //sum of these should add up to 1
        }

        public static FARAeroSection CreateNewAeroSection()
        {
            FARAeroSection section = new FARAeroSection
            {
                xForcePressureAoA0   = new FARFloatCurve(6),
                xForcePressureAoA180 = new FARFloatCurve(6),
                xForceSkinFriction   = new FARFloatCurve(3),
                partData             = new List<PartData>(),
                handledAeroModulesIndexDict =
                    new Dictionary<FARAeroPartModule, int>(ObjectReferenceEqualityComparer<FARAeroPartModule>.Default)
            };


            GenerateCrossFlowDragCurve();

            return section;
        }

        private FARAeroSection() { }

        // ReSharper disable ParameterHidesMember -> updating member values
        public void UpdateAeroSection(float potentialFlowNormalForce, float viscCrossflowDrag, float diameter, float flatnessRatio, float hypersonicMomentForward, float hypersonicMomentBackward,
                                      Vector3 centroidWorldSpace, Vector3 xRefVectorWorldSpace, Vector3 nRefVectorWorldSpace, Matrix4x4 vesselToWorldMatrix, Vector3 vehicleMainAxis, List<FARAeroPartModule> moduleList,
            List<float> dragFactor, Dictionary<Part, PartTransformInfo> partWorldToLocalMatrixDict)
        // ReSharper restore ParameterHidesMember
        {
            mergeFactor = 0;

            this.potentialFlowNormalForce = potentialFlowNormalForce;                   //copy lifting body info over
            this.viscCrossflowDrag = viscCrossflowDrag;
            this.flatnessRatio = flatnessRatio;
            invFlatnessRatio = 1 / flatnessRatio;
            this.hypersonicMomentForward = hypersonicMomentForward;
            this.hypersonicMomentBackward = hypersonicMomentBackward;
            this.diameter = diameter;

            partData.Clear();
            handledAeroModulesIndexDict.Clear();
            if (partData.Capacity < moduleList.Capacity)
                partData.Capacity = moduleList.Capacity;

            Vector3 worldVehicleAxis = vesselToWorldMatrix.MultiplyVector(vehicleMainAxis);

            Vector3 centroidLocationAlongxRef = Vector3.Project(centroidWorldSpace, worldVehicleAxis);
            Vector3 centroidSansxRef = Vector3.ProjectOnPlane(centroidWorldSpace, worldVehicleAxis);

            Vector3 worldSpaceAvgPos = Vector3.zero;
            float totalDragFactor = 0;
            for (int i = 0; i < moduleList.Count; i++)
            {
                Part p = moduleList[i].part;
                if (partWorldToLocalMatrixDict.ContainsKey(p))
                {
                    worldSpaceAvgPos += partWorldToLocalMatrixDict[p].worldPosition * dragFactor[i];
                    totalDragFactor += dragFactor[i];
                }
            }

            worldSpaceAvgPos /= totalDragFactor;

            worldSpaceAvgPos = Vector3.ProjectOnPlane(worldSpaceAvgPos, worldVehicleAxis);

            Vector3 avgPosDiffFromCentroid = centroidSansxRef - worldSpaceAvgPos;

            for (int i = 0; i < moduleList.Count; i++)
            {
                PartData data = new PartData {aeroModule = moduleList[i]};
                Matrix4x4 transformMatrix = partWorldToLocalMatrixDict[data.aeroModule.part].worldToLocalMatrix;

                Vector3 forceCenterWorldSpace = centroidLocationAlongxRef + Vector3.ProjectOnPlane(partWorldToLocalMatrixDict[data.aeroModule.part].worldPosition, worldVehicleAxis) + avgPosDiffFromCentroid;

                data.centroidPartSpace = transformMatrix.MultiplyPoint3x4(forceCenterWorldSpace);
                data.xRefVectorPartSpace = transformMatrix.MultiplyVector(xRefVectorWorldSpace);
                data.nRefVectorPartSpace = transformMatrix.MultiplyVector(nRefVectorWorldSpace);
                data.dragFactor = dragFactor[i];

                if (i < partData.Count)
                    partData[i] = data;
                else
                    partData.Add(data);

                handledAeroModulesIndexDict.Add(data.aeroModule, i);
            }

            xForcePressureAoA0.BakeCurve();
            xForcePressureAoA180.BakeCurve();
            xForceSkinFriction.BakeCurve();

            worldNormalVector = nRefVectorWorldSpace;
        }

        public bool CanMerge(FARAeroSection otherSection)
        {
            if (mergeFactor >= 10)
                return false;       //only merge up to 10 sections

            bool merge = true;

            float flatnessRelDiff = flatnessRatio - otherSection.flatnessRatio;
            flatnessRelDiff *= invFlatnessRatio;

            if (flatnessRelDiff < 0.05)  //allow for 5% rel difference for merging
                if ((flatnessRatio - 1) < 0.05)  //if it's within 5% of 1, it's good
                    merge &= true;
                else if (Math.Abs(Vector3.Dot(worldNormalVector, otherSection.worldNormalVector)) > 0.999)    //allow 5 degrees error for flatnessRatio
                    merge &= true;
                else
                    merge &= false;         //too different in out-of-roundness, don't merge

            float diameterRelDiff = diameter - otherSection.diameter;
            diameterRelDiff /= diameter;

            if (diameterRelDiff < 0.05)
                merge &= true;
            else
                merge &= false;

            return merge;
        }

        public void MergeAeroSection(FARAeroSection otherSection)
        {
            //increase merge factor each time we merge to maintain relative strength of sections
            mergeFactor += 1;

            float invMergeFactorP1 = 1 / (mergeFactor + 1);

            //add simple factors
            potentialFlowNormalForce += otherSection.potentialFlowNormalForce;
            viscCrossflowDrag += otherSection.viscCrossflowDrag;
            hypersonicMomentForward += otherSection.hypersonicMomentForward;
            hypersonicMomentBackward += otherSection.hypersonicMomentBackward;

            flatnessRatio = invMergeFactorP1 * (flatnessRatio * mergeFactor + otherSection.flatnessRatio);
            invFlatnessRatio = invMergeFactorP1 * (invFlatnessRatio * mergeFactor + otherSection.invFlatnessRatio);
            diameter = invMergeFactorP1 * (diameter * mergeFactor + otherSection.diameter);

            //merge the curves; don't scale because these are actual drag values
            xForcePressureAoA0.AddCurve(otherSection.xForcePressureAoA0);
            xForcePressureAoA180.AddCurve(otherSection.xForcePressureAoA180);
            xForceSkinFriction.AddCurve(otherSection.xForceSkinFriction);

            //prep old part data for merging
            for (int i = 0; i < partData.Count; ++i)
            {
                PartData oldData = partData[i];
                oldData.dragFactor *= mergeFactor;      //scale all of these up for the incoming data
                oldData.centroidPartSpace *= mergeFactor;
                oldData.xRefVectorPartSpace *= mergeFactor;
                oldData.nRefVectorPartSpace *= mergeFactor;
                partData[i] = oldData;
            }

            //merge PartData
            float mergeFactorP1 = mergeFactor + 1;
            for (int i = 0; i < otherSection.partData.Count; ++i)
            {
                PartData tmpOtherData = otherSection.partData[i];
                if (handledAeroModulesIndexDict.TryGetValue(tmpOtherData.aeroModule, out int index))
                {
                    PartData tmpData = partData[index];
                    tmpData.centroidPartSpace += tmpOtherData.centroidPartSpace;         //prep'd for averaging
                    tmpData.xRefVectorPartSpace += tmpOtherData.xRefVectorPartSpace;
                    tmpData.nRefVectorPartSpace += tmpOtherData.nRefVectorPartSpace;
                    tmpData.dragFactor += tmpOtherData.dragFactor;

                    partData[index] = tmpData;
                }
                else
                {
                    /*tmpOtherData.centroidPartSpace = mergeFactorP1 * (tmpOtherData.centroidPartSpace);     //these must be scaled to completely counter the effect of the downscale at the end
                    tmpOtherData.xRefVectorPartSpace = (tmpOtherData.xRefVectorPartSpace);
                    tmpOtherData.nRefVectorPartSpace = (tmpOtherData.nRefVectorPartSpace);
                    tmpOtherData.dragFactor = invMergeFactor * (tmpOtherData.dragFactor);       //this will already be scaled down at the end
                    */
                    tmpOtherData.centroidPartSpace *= mergeFactorP1;      //needs to be scaled this way to place the centroid in the correct location
                    partData.Add(tmpOtherData);

                    handledAeroModulesIndexDict.Add(tmpOtherData.aeroModule, partData.Count - 1);
                }
            }

            for (int i = 0; i < partData.Count; ++i)
            {
                PartData newData = partData[i];
                newData.dragFactor *= invMergeFactorP1;      //now scale everything back down to sane levels
                newData.centroidPartSpace *= invMergeFactorP1;
                newData.xRefVectorPartSpace.Normalize();
                newData.nRefVectorPartSpace.Normalize();
                partData[i] = newData;
            }

        }

        public void ClearAeroSection()
        {
            //xForcePressureAoA0 = null;
            //xForcePressureAoA180 = null;
            //xForceSkinFriction = null;
            partData.Clear();
            handledAeroModulesIndexDict.Clear();
        }

        #region Force application contexts
        private interface IForceContext
        {
            /// <summary>
            /// The part-relative velocity of the part whose force is being computed
            /// </summary>
            /// <param name="pd">The part data for which to compute the local velocity</param>
            Vector3 LocalVelocity(PartData pd);
            /// <summary>
            /// Apply a calculated force to a part.
            /// </summary>
            /// <param name="pd">The part data of the part that the force should be applied to</param>
            /// <param name="localVel">The local velocity of the part</param>
            /// <param name="forceVector">The calculated force vector to be applied to the part</param>
            /// <param name="torqueVector">The calculated torque vector to be applied to the part</param>
            void ApplyForce(PartData pd, Vector3 localVel, Vector3 forceVector, Vector3 torqueVector);
        }

        private class SimulatedForceContext : IForceContext
        {
            /// <summary>
            /// The world-space velocity of the part whose force is being simulated
            /// </summary>
            private Vector3 worldVel;

            /// <summary>
            /// The center with which force should be accumulated
            /// </summary>
            private FARCenterQuery center;

            /// <summary>
            /// The atmospheric density that the force is being simulated at
            /// </summary>
            private float atmDensity;

            public SimulatedForceContext(Vector3 worldVel, FARCenterQuery center, float atmDensity)
            {
                this.worldVel = worldVel;
                this.center = center;
                this.atmDensity = atmDensity;
            }

            // ReSharper disable ParameterHidesMember -> updating member values
            public void UpdateSimulationContext(Vector4 worldVel, FARCenterQuery center, float atmDensity)
            {
                this.worldVel = worldVel;
                this.center = center;
                this.atmDensity = atmDensity;
            }
            // ReSharper restore ParameterHidesMember

            public Vector3 LocalVelocity(PartData pd)
            {
                if (pd.aeroModule.part == null || pd.aeroModule.part.partTransform == null)
                    return Vector3.zero;
                return pd.aeroModule.part.partTransform.InverseTransformVector(worldVel);
            }

            public void ApplyForce(PartData pd, Vector3 localVel, Vector3 forceVector, Vector3 torqueVector)
            {
                var tmp = 0.0005 * Vector3.SqrMagnitude(localVel);
                var dynamicPressurekPa = tmp * atmDensity;
                var dragFactor = dynamicPressurekPa * Mathf.Max(PhysicsGlobals.DragCurvePseudoReynolds.Evaluate(atmDensity * Vector3.Magnitude(localVel)), 1.0f);
                var liftFactor = dynamicPressurekPa;

                var localVelNorm = Vector3.Normalize(localVel);
                Vector3 localForceTemp = Vector3.Dot(localVelNorm, forceVector) * localVelNorm;
                var partLocalForce = (localForceTemp * (float)dragFactor + (forceVector - localForceTemp) * (float)liftFactor);
                forceVector = pd.aeroModule.part.transform.TransformDirection(partLocalForce);
                torqueVector = pd.aeroModule.part.transform.TransformDirection(torqueVector * (float)dynamicPressurekPa);
                if (!float.IsNaN(forceVector.x) && !float.IsNaN(torqueVector.x))
                {
                    Vector3 centroid = pd.aeroModule.part.transform.TransformPoint(pd.centroidPartSpace - pd.aeroModule.part.CoMOffset);
                    center.AddForce(centroid, forceVector);
                    center.AddTorque(torqueVector);
                }
            }
        }

        private class FlightForceContext : IForceContext
        {
            public Vector3 LocalVelocity(PartData pd)
            {
                return pd.aeroModule.partLocalVel;
            }

            public void ApplyForce(PartData pd, Vector3 localVel, Vector3 forceVector, Vector3 torqueVector)
            {
                pd.aeroModule.AddLocalForceAndTorque(forceVector, torqueVector, pd.centroidPartSpace);
            }
        }
        #endregion

        private void CalculateAeroForces(
            float machNumber, float reynoldsPerUnitLength, float pseudoKnudsenNumber, float skinFrictionDrag,
            IForceContext forceContext)
        {
            double skinFrictionForce = skinFrictionDrag * xForceSkinFriction.Evaluate(machNumber);      //this will be the same for each part, so why recalc it multiple times?
            double xForceAoA0 = xForcePressureAoA0.Evaluate(machNumber);
            double xForceAoA180 = xForcePressureAoA180.Evaluate(machNumber);

            foreach (PartData data in partData)
            {
                FARAeroPartModule aeroModule = data.aeroModule;
                if (aeroModule is null)
                {
                    continue;
                }

                Vector3 xRefVector = data.xRefVectorPartSpace;
                Vector3 nRefVector = data.nRefVectorPartSpace;

                Vector3 velLocal = forceContext.LocalVelocity(data);
                // Rejects both negligable speed and invalid simulation cases
                if (velLocal.sqrMagnitude.NearlyEqual(0.0f))
                {
                    continue;
                }

                Vector3 angVelLocal = aeroModule.partLocalAngVel;

                velLocal += Vector3.Cross(data.centroidPartSpace, angVelLocal); //some transform issue here, needs investigation
                Vector3 velLocalNorm = velLocal.normalized;

                Vector3 localNormalForceVec = Vector3.ProjectOnPlane(-velLocalNorm, xRefVector).normalized;

                double cosAoA     = Vector3.Dot(xRefVector, velLocalNorm);
                double cosSqrAoA  = cosAoA * cosAoA;
                double sinSqrAoA  = Math.Max(1 - cosSqrAoA, 0);
                double sinAoA     = Math.Sqrt(sinSqrAoA);
                double sin2AoA    = 2 * sinAoA * Math.Abs(cosAoA);
                double cosHalfAoA = Math.Sqrt(0.5 + 0.5 * Math.Abs(cosAoA));


                double nForce;
                nForce = potentialFlowNormalForce * Math.Sign(cosAoA) * cosHalfAoA * sin2AoA; //potential flow normal force
                if (nForce < 0)                                                               //potential flow is not significant over the rear face of things
                    nForce = 0;

                //if (machNumber > 3)
                //    nForce *= 2d - machNumber * 0.3333333333333333d;

                float normalForceFactor = Math.Abs(Vector3.Dot(localNormalForceVec, nRefVector));
                normalForceFactor *= normalForceFactor;

                normalForceFactor = invFlatnessRatio * (1 - normalForceFactor) + flatnessRatio * normalForceFactor; //accounts for changes in relative flatness of shape


                float crossFlowMach, crossFlowReynolds;
                crossFlowMach     = machNumber * (float)sinAoA;
                crossFlowReynolds = reynoldsPerUnitLength * diameter * (float)sinAoA / normalForceFactor;

                nForce += viscCrossflowDrag * sinSqrAoA * CalculateCrossFlowDrag(crossFlowMach, crossFlowReynolds); //viscous crossflow normal force

                nForce *= normalForceFactor;

                double xForce        = -skinFrictionForce * Math.Sign(cosAoA) * cosSqrAoA;
                double localVelForce = xForce * pseudoKnudsenNumber;
                xForce -= localVelForce;

                localVelForce = Math.Abs(localVelForce);

                float moment        = (float)(cosAoA * sinAoA);
                float dampingMoment = 4f * moment;

                if (cosAoA > 0)
                {
                    xForce += cosSqrAoA * xForceAoA0;
                    float momentFactor;
                    if (machNumber > 6)
                        momentFactor = hypersonicMomentForward;
                    else if (machNumber < 0.6)
                        momentFactor = 0.6f * hypersonicMomentBackward;
                    else
                    {
                        float tmp = (-0.185185185f * machNumber + 1.11111111111f);
                        momentFactor = tmp * hypersonicMomentBackward * 0.6f + (1 - tmp) * hypersonicMomentForward;
                    }
                    //if (machNumber < 1.5)
                    //    momentFactor += hypersonicMomentBackward * (0.5f - machNumber * 0.33333333333333333333333333333333f) * 0.2f;

                    moment        *= momentFactor;
                    dampingMoment *= momentFactor;
                }
                else
                {
                    xForce += cosSqrAoA * xForceAoA180;
                    float momentFactor; //negative to deal with the ref vector facing the opposite direction, causing the moment vector to point in the opposite direction
                    if (machNumber > 6)
                        momentFactor = hypersonicMomentBackward;
                    else if (machNumber < 0.6)
                        momentFactor = 0.6f * hypersonicMomentForward;
                    else
                    {
                        float tmp = (-0.185185185f * machNumber + 1.11111111111f);
                        momentFactor = tmp * hypersonicMomentForward * 0.6f + (1 - tmp) * hypersonicMomentBackward;
                    }
                    //if (machNumber < 1.5)
                    //    momentFactor += hypersonicMomentForward * (0.5f - machNumber * 0.33333333333333333333333333333333f) * 0.2f;

                    moment        *= momentFactor;
                    dampingMoment *= momentFactor;
                }
                moment        /= normalForceFactor;
                dampingMoment =  Math.Abs(dampingMoment) * 0.1f;
                //dampingMoment += (float)Math.Abs(skinFrictionForce) * 0.1f;
                float rollDampingMoment = (float)(skinFrictionForce * 0.5 * diameter); //skin friction force times avg moment arm for vehicle
                rollDampingMoment *= (0.75f + flatnessRatio * 0.25f);                  //this is just an approximation for now

                Vector3 forceVector = (float)xForce * xRefVector + (float)nForce * localNormalForceVec;
                forceVector -= (float)localVelForce * velLocalNorm;

                Vector3 torqueVector = Vector3.Cross(xRefVector, localNormalForceVec) * moment;

                Vector3 axialAngLocalVel    = Vector3.Dot(xRefVector, angVelLocal) * xRefVector;
                Vector3 nonAxialAngLocalVel = angVelLocal - axialAngLocalVel;

                if (velLocal.sqrMagnitude > 0.001f)
                    torqueVector -= (dampingMoment * nonAxialAngLocalVel) + (rollDampingMoment * axialAngLocalVel * axialAngLocalVel.magnitude) / velLocal.sqrMagnitude;
                else
                    torqueVector -= (dampingMoment * nonAxialAngLocalVel) + (rollDampingMoment * axialAngLocalVel * axialAngLocalVel.magnitude) / 0.001f;

                //float dynPresAndScaling = 0.0005f * atmDensity * velLocal.sqrMagnitude * data.dragFactor;        //dyn pres and N -> kN conversion

                //forceVector *= dynPresAndScaling;
                //torqueVector *= dynPresAndScaling;

                forceVector  *= data.dragFactor;
                torqueVector *= data.dragFactor;

                forceContext.ApplyForce(data, velLocal, forceVector, torqueVector);
            }
        }

        private SimulatedForceContext simContext = new SimulatedForceContext(Vector3.zero, new FARCenterQuery(), 0.0f);
        public void PredictionCalculateAeroForces(float atmDensity, float machNumber, float reynoldsPerUnitLength, float pseudoKnudsenNumber, float skinFrictionDrag, Vector3 vel, FARCenterQuery center)
        {
            simContext.UpdateSimulationContext(vel, center, atmDensity);
            CalculateAeroForces(machNumber, reynoldsPerUnitLength, pseudoKnudsenNumber, skinFrictionDrag, simContext);
        }

        private FlightForceContext flightContext = new FlightForceContext();
        public void FlightCalculateAeroForces(float machNumber, float reynoldsPerUnitLength, float pseudoKnudsenNumber, float skinFrictionDrag)
        {
            CalculateAeroForces(machNumber, reynoldsPerUnitLength, pseudoKnudsenNumber, skinFrictionDrag, flightContext);

        }

        public static void GenerateCrossFlowDragCurve()
        {
            if (crossFlowDragMachCurve != null)
                return;

            crossFlowDragMachCurve = new FloatCurve();
            crossFlowDragMachCurve.Add(0, 1.2f, 0, 0);
            crossFlowDragMachCurve.Add(0.3f, 1.2f, 0, 0);
            crossFlowDragMachCurve.Add(0.7f, 1.5f, 0, 0);
            crossFlowDragMachCurve.Add(0.85f, 1.41f, 0, 0);
            crossFlowDragMachCurve.Add(0.95f, 2.1f, 0, 0);
            crossFlowDragMachCurve.Add(1f, 2f, -2f, -2f);
            crossFlowDragMachCurve.Add(1.3f, 1.6f, -0.5f, -0.5f);
            crossFlowDragMachCurve.Add(2f, 1.4f, -0.1f, -0.1f);
            crossFlowDragMachCurve.Add(5f, 1.25f, -0.02f, -0.02f);
            crossFlowDragMachCurve.Add(10f, 1.2f, 0, 0);

            crossFlowDragReynoldsCurve = new FloatCurve();
            crossFlowDragReynoldsCurve.Add(10000, 1f, 0, 0);
            crossFlowDragReynoldsCurve.Add(100000, 1.0083333333333333333333333333333f, 0, 0);
            crossFlowDragReynoldsCurve.Add(180000, 1.0083333333333333333333333333333f, 0, 0);
            crossFlowDragReynoldsCurve.Add(250000, 0.66666666666666666666666666666667f);
            crossFlowDragReynoldsCurve.Add(300000, 0.25f, -5E-07f, -5E-07f);
            crossFlowDragReynoldsCurve.Add(500000, 0.20833333333333333333333333333333f, 0, 0);
            crossFlowDragReynoldsCurve.Add(1000000, 0.33333333333333333333333333333333f, 7E-8f, 7E-8f);
            crossFlowDragReynoldsCurve.Add(10000000, 0.58333333333333333333333333333333f, 0, 0);
        }

        private float CalculateCrossFlowDrag(float crossFlowMach, float crossFlowReynolds)
        {
            if (crossFlowMach > 0.5f)
                return crossFlowDragMachCurve.Evaluate(crossFlowMach);
            float reynoldsInfluenceFactor = 1;
            if (crossFlowMach > 0.4f)
                reynoldsInfluenceFactor -= (crossFlowMach - 0.4f) * 10;

            float crossFlowDrag = crossFlowDragReynoldsCurve.Evaluate(crossFlowReynolds);
            crossFlowDrag = (crossFlowDrag - 1) * reynoldsInfluenceFactor + 1;
            crossFlowDrag *= crossFlowDragMachCurve.Evaluate(crossFlowMach);

            return crossFlowDrag;
        }
    }
}
