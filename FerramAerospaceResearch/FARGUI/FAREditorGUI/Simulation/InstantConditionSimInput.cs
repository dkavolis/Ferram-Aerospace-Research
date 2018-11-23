/*
Ferram Aerospace Research v0.15.9.5 "Lighthill"
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
using FerramAerospaceResearch.FARUtils;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI.Simulation
{
    public class InstantConditionSimInput : IFARCloneable
    {
        private FARCloneHelper _cloneHelper;
        public double alpha;
        public double beta;
        public double phi;
        public double alphaDot;
        public double betaDot;
        public double phiDot;
        public double machNumber;
        public double pitchValue;
        public int flaps;
        public bool spoilers;

        public InstantConditionSimInput() => _cloneHelper = new FARCloneHelper();

        public InstantConditionSimInput(double alpha, double beta, double phi, double alphaDot, double betaDot, double phiDot, double machNumber, double pitchValue) : this()
        {
            this.alpha = alpha;
            this.beta = beta;
            this.phi = phi;
            this.alphaDot = alphaDot;
            this.betaDot = betaDot;
            this.phiDot = phiDot;
            this.machNumber = machNumber;
            this.pitchValue = pitchValue;

            flaps = 0;
            spoilers = false;
        }

        public InstantConditionSimInput(double alpha, double beta, double phi, double alphaDot, double betaDot, double phiDot, double machNumber, double pitchValue, int flaps, bool spoilers) : this()
        {
            this.alpha = alpha;
            this.beta = beta;
            this.phi = phi;
            this.alphaDot = alphaDot;
            this.betaDot = betaDot;
            this.phiDot = phiDot;
            this.machNumber = machNumber;
            this.pitchValue = pitchValue;
            this.flaps = flaps;
            this.spoilers = spoilers;
        }

        protected FARCloneHelper cloneHelper
        {
            get
            {
                if (_cloneHelper == null)
                    _cloneHelper = new FARCloneHelper();
                return _cloneHelper;
            }
        }

        public Guid GUID
        {
            get
            {
                return cloneHelper.GUID;
            }
        }

        public bool isClone
        {
            get
            {
                return cloneHelper.isClone;
            }
        }

        protected InstantConditionSimInput(InstantConditionSimInput other, Dictionary<Guid, object> cache)
        {
            _cloneHelper = new FARCloneHelper(other, this, cache);

            alpha = other.alpha;
            beta = other.beta;
            phi = other.phi;
            alphaDot = other.alphaDot;
            betaDot = other.betaDot;
            phiDot = other.phiDot;
            machNumber = other.machNumber;
            pitchValue = other.pitchValue;
            flaps = other.flaps;
            spoilers = other.spoilers;
        }

        public T Clone<T>() where T : IFARCloneable => FARCloneHelper.Clone<T>(this);
        public T Clone<T>(Dictionary<Guid, object> cache) where T : IFARCloneable => FARCloneHelper.Clone<T>(this, cache);
        public virtual object Clone(IFARCloneable other, Dictionary<Guid, object> cache) => new InstantConditionSimInput((InstantConditionSimInput)other, cache);
    }
}
