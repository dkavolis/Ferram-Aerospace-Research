/*
Ferram Aerospace Research v0.15.9.5 "Lighthill"
=========================
Copyright 2018, Daumantas Kavolis, aka dkavolis

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
   along with Ferram Aerospace Research.  If not, see <http: //www.gnu.org/licenses/>.

   Serious thanks:        a.g., for tons of bugfixes and code-refactorings
                stupid_chris, for the RealChuteLite implementation
                        Taverius, for correcting a ton of incorrect values
                Tetryds, for finding lots of bugs and issues and not letting me get away with them, and work on example crafts
                        sarbian, for refactoring code for working with MechJeb, and the Module Manager updates
                        ialdabaoth (who is awesome), who originally created Module Manager
                            Regex, for adding RPM support
                DaMichel, for some ferramGraph updates and some control surface-related features
                        Duxwing, for copy editing the readme

   CompatibilityChecker by Majiir, BSD 2-clause http: //opensource.org/licenses/BSD-2-Clause

   Part.cfg changes powered by sarbian & ialdabaoth's ModuleManager plugin; used with permission
    http: //forum.kerbalspaceprogram.com/threads/55219

   ModularFLightIntegrator by Sarbian, Starwaster and Ferram4, MIT: http: //opensource.org/licenses/MIT
    http: //forum.kerbalspaceprogram.com/threads/118088

   Toolbar integration powered by blizzy78's Toolbar plugin; used with permission
    http: //forum.kerbalspaceprogram.com/threads/60863
 */

using FerramAerospaceResearch.FARGUI.FAREditorGUI;
using FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation;

namespace FerramAerospaceResearch.FARAPIModules
{
    public class InstantConditionSimulation
    {
        private InstantConditionSim _instantCondition;
        public InstantConditionSimOutput iterationOutput
        {
            get { return _instantCondition.iterationOutput; }
        }

        public InstantConditionSimulation()
        {
            _instantCondition = EditorGUI.Instance.SimManager.InstantCondition.Clone<InstantConditionSim>();
        }

        public bool Ready
        {
            get { return _instantCondition.Ready; }
        }

        public double CalculateAccelerationDueToGravity(CelestialBody body, double alt)
        {
            return _instantCondition.CalculateAccelerationDueToGravity(body, alt);
        }

        public InstantConditionSimOutput ComputeNonDimensionalForces(InstantConditionSimInput input, bool clear, bool reset_stall = false)
        {
            InstantConditionSimOutput output = new InstantConditionSimOutput();
            _instantCondition.GetClCdCmSteady(input, out output, clear, reset_stall);
            return output;
        }

        public InstantConditionSimOutput ComputeNonDimensionalForces(double alpha, double beta, double phi, double alphaDot, double betaDot, double phiDot, double machNumber, double pitchValue, bool clear, bool reset_stall = false)
        {
            InstantConditionSimInput input = new InstantConditionSimInput(alpha, beta, phi, alphaDot, betaDot, phiDot, machNumber, pitchValue);
            return ComputeNonDimensionalForces(input, clear, reset_stall);
        }

        public InstantConditionSimOutput ComputeNonDimensionalForces(double alpha, double beta, double phi, double alphaDot, double betaDot, double phiDot, double machNumber, double pitchValue, int flaps, bool spoilers, bool clear, bool reset_stall = false)
        {
            InstantConditionSimInput input = new InstantConditionSimInput(alpha, beta, phi, alphaDot, betaDot, phiDot, machNumber, pitchValue, flaps, spoilers);
            return ComputeNonDimensionalForces(input, clear, reset_stall);
        }

        public void SetState(double machNumber, double Cl, Vector3d CoM, double pitch, int flapSetting, bool spoilers)
        {
            _instantCondition.SetState(machNumber, Cl, CoM, pitch, flapSetting, spoilers);
        }

        public double FunctionIterateForAlpha(double alpha)
        {
            return _instantCondition.FunctionIterateForAlpha(alpha);
        }

        public double ComputeRequiredAoA(double machNumber, double Cl, Vector3d CoM, double pitch, int flapSetting, bool spoilers)
        {
            _instantCondition.SetState(machNumber, Cl, CoM, pitch, flapSetting, spoilers);
            return FARMathUtil.BrentsMethod(_instantCondition.FunctionIterateForAlpha, -30d, 30d, 0.001, 500);
        }

    }
}
