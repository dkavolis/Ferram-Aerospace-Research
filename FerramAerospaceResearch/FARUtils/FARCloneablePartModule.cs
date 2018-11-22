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

using System;
using System.Collections.Generic;
using UnityEngine;

namespace FerramAerospaceResearch.FARUtils
{
    public class FARCloneablePartModule : PartModule, IFARCloneable
    {
        private FARCloneHelper _cloneHelper;

        // Override Component.tag and Component.transform since calling plain constructor won't create proper Unity objects
        private Transform _transform;
        private string _tag;

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

        public new Transform transform
        {
            get
            {
                if (_transform == null)
                    return base.transform;
                return _transform;
            }
        }

        public new string tag
        {
            get
            {
                if (_tag == null)
                    return base.tag;
                return _tag;
            }
            set
            {
                if (_tag == null)
                    base.tag = value;
                else
                    _tag = value;
            }
        }

        public FARCloneablePartModule() : base()
        {
            _cloneHelper = new FARCloneHelper();
        }

        protected FARCloneablePartModule(FARCloneablePartModule other, Dictionary<Guid, object> cache) : base()
        {
            _cloneHelper = new FARCloneHelper(other, this, cache);

            _transform = other.transform;
            _tag = other.tag;

            // Only do a shallow copy of KSP objects since they are not mutated here
            part = other.part;
            resHandler = other.resHandler;
            upgradesApplied = FARCloneHelper.ShallowCopy(other.upgradesApplied);
            showUpgradesInModuleInfo = other.showUpgradesInModuleInfo;
            upgradesAsk = other.upgradesAsk;
            moduleName = other.moduleName;
            upgrades = other.upgrades;
            upgradesApply = other.upgradesApply;
            isEnabled = other.isEnabled;
            snapshot = other.snapshot;
            stagingToggleEnabledEditor = other.stagingToggleEnabledEditor;
            stagingEnabled = other.stagingEnabled;
            stagingEnableText = other.stagingEnableText;
            stagingDisableText = other.stagingDisableText;
            overrideStagingIconIfBlank = other.overrideStagingIconIfBlank;
            moduleIsEnabled = other.moduleIsEnabled;
            stagingToggleEnabledFlight = other.stagingToggleEnabledFlight;
        }

        public T Clone<T>() where T : IFARCloneable => FARCloneHelper.Clone<T>(this);

        public T Clone<T>(Dictionary<Guid, object> cache) where T : IFARCloneable => FARCloneHelper.Clone<T>(this, cache);

        public virtual object Clone(IFARCloneable other, Dictionary<Guid, object> cache) => new FARCloneablePartModule((FARCloneablePartModule)other, cache);
    }
}
