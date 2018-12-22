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

        /// <summary>
        /// The last iteration output.
        /// </summary>
        /// <value>The last iteration output when either FunctionIterateForAlpha or ComputeRequiredAoA was called.</value>
        public InstantConditionSimOutput iterationOutput
        {
            get { return _instantCondition.iterationOutput; }
        }

        public double maxCrossSectionFromBody { get => _instantCondition._maxCrossSectionFromBody; }
        public double bodyLength { get => _instantCondition._bodyLength; }
        public double area { get => _instantCondition.area; }
        /// <summary>
        /// Mean aerodynamic chord
        /// </summary>
        public double MAC { get => _instantCondition.MAC; }
        /// <summary>
        /// Wing span
        /// </summary>
        public double b_2 { get => _instantCondition.b_2; }
        /// <summary>
        /// Center of mass
        /// </summary>
        public Vector3d CoM { get => _instantCondition.CoM; }
        public double mass { get => _instantCondition.mass; }

        /// <summary>
        /// Default constructor that encapsulates a clone of instant condition simulation used in FAR. <see cref="InstantConditionSimulation.Update" /> should be called if the editor state changes.
        /// </summary>
        public InstantConditionSimulation()
        {
            _instantCondition = EditorGUI.Instance.SimManager.InstantCondition.Clone<InstantConditionSim>();
        }

        /// <summary>
        /// Whether simulation can be run.
        /// </summary>
        /// <value>Is simulation ready to be run.</value>
        public bool Ready
        {
            get { return _instantCondition.Ready; }
        }

        /// <summary>
        /// Compute the gravitational acceleration.
        /// </summary>
        /// <param name="body">Celestial body to compute acceleration for</param>
        /// <param name="alt">Altitude above mean radius</param>
        /// <returns>Gravitational acceleration.</returns>
        public double CalculateAccelerationDueToGravity(CelestialBody body, double alt)
        {
            return _instantCondition.CalculateAccelerationDueToGravity(body, alt);
        }

        /// <summary>
        /// Compute non-dimensional forces for the current body and vehicle.
        /// <see cref="FARAPI.Simulation.UpdateCurrentBody" /> to set the celestial body to be used for simulation.
        /// </summary>
        /// <param name="input">Simulation input values InstantConditionSimInput</param>
        /// <param name="clear">Whether to clear Cl and Cd from the wing aerodynamic model, only needed on the first simulation or for clearing stall.</param>
        /// <param name="reset_stall">Whether to reset the stall, <paramref name="clear"/> needs to be true. Set to true if input conditions are changed that would result in stall</param>
        /// <returns>InstantConditionSimOutput of non-dimensional forces</returns>
        public InstantConditionSimOutput ComputeNonDimensionalForces(InstantConditionSimInput input, bool clear, bool reset_stall = false)
        {
            InstantConditionSimOutput output = new InstantConditionSimOutput();
            _instantCondition.GetClCdCmSteady(input, out output, clear, reset_stall);
            return output;
        }

        /// <summary>
        /// Overload of <see cref="ComputeNonDimensionalForces" /> using only native types. See <see cref="FARAPI.Simulation.SimulationInput(double, double, double, double, double, double, double, double)"/> for arguments.
        /// </summary>
        public InstantConditionSimOutput ComputeNonDimensionalForces(double alpha, double beta, double phi, double alphaDot, double betaDot, double phiDot, double machNumber, double pitchValue, bool clear, bool reset_stall = false)
        {
            InstantConditionSimInput input = new InstantConditionSimInput(alpha, beta, phi, alphaDot, betaDot, phiDot, machNumber, pitchValue);
            return ComputeNonDimensionalForces(input, clear, reset_stall);
        }

        /// <summary>
        /// Overload of <see cref="ComputeNonDimensionalForces" /> using only native types. See <see cref="FARAPI.Simulation.SimulationInput(double, double, double, double, double, double, double, double, int, bool)"/> for arguments.
        /// </summary>
        public InstantConditionSimOutput ComputeNonDimensionalForces(double alpha, double beta, double phi, double alphaDot, double betaDot, double phiDot, double machNumber, double pitchValue, int flaps, bool spoilers, bool clear, bool reset_stall = false)
        {
            InstantConditionSimInput input = new InstantConditionSimInput(alpha, beta, phi, alphaDot, betaDot, phiDot, machNumber, pitchValue, flaps, spoilers);
            return ComputeNonDimensionalForces(input, clear, reset_stall);
        }

        /// <summary>
        /// Set the state that will be used for computing difference between required and current lift coefficients.
        /// <seealso cref="FunctionIterateForAlpha" />, <seealso cref="InstantConditionSimulation.ComputeRequiredAoA(double, double, Vector3d, double, int, bool)" />
        /// </summary>
        /// <param name="machNumber">Mach number</param>
        /// <param name="Cl">Required lift coefficient</param>
        /// <param name="CoM">Center of mass</param>
        /// <param name="pitch">Pitch angle</param>
        /// <param name="flapSetting">Flap deflection level 0 (no deflection) to 3 (full deflection)</param>
        /// <param name="spoilers">Whether spoilers are extended</param>
        public void SetState(double machNumber, double Cl, Vector3d CoM, double pitch, int flapSetting, bool spoilers)
        {
            _instantCondition.SetState(machNumber, Cl, CoM, pitch, flapSetting, spoilers);
        }

        /// <summary>
        /// Compute the difference between required and current lift coefficients at angle of attack <paramref name="alpha"/>. <see cref="SetState"/> to
        /// </summary>
        /// <param name="alpha">Current angle of attack</param>
        /// <returns>Difference between current and required lift coefficients.</returns>
        public double FunctionIterateForAlpha(double alpha)
        {
            return _instantCondition.FunctionIterateForAlpha(alpha);
        }

        /// <summary>
        /// Computes required angle of attack to achieve lift coefficient <paramref name="Cl"/> at a specified Mach number <paramref name="machNumber"/>.null Uses Brent's method internally.
        /// See <see cref="SetState" /> for arguments.
        /// </summary>
        /// <returns>Angle of attack for steady flight</returns>
        public double ComputeRequiredAoA(double machNumber, double Cl, Vector3d CoM, double pitch, int flapSetting, bool spoilers)
        {
            _instantCondition.SetState(machNumber, Cl, CoM, pitch, flapSetting, spoilers);
            return FARMathUtil.BrentsMethod(_instantCondition.FunctionIterateForAlpha, -30d, 30d, 0.001, 500);
        }

        /// <summary>
        /// Updates this <see cref="InstantConditionSimulation" /> to match the current state of the editor.
        /// </summary>
        public void Update()
        {
            _instantCondition = EditorGUI.Instance.SimManager.InstantCondition.Clone<InstantConditionSim>();
        }

    }
}
