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

using System;
using System.Collections.Generic;
using FerramAerospaceResearch;
using FerramAerospaceResearch.FARUtils;
using UnityEngine;

// ReSharper disable once CheckNamespace
namespace ferram4
{
    public class FARPartModule : PartModule
    {
        protected Callback OnVesselPartsChange;
        public List<Part> VesselPartList;
        private Collider[] partColliders;

        public Collider[] PartColliders { get { if(partColliders == null) TriggerPartColliderUpdate(); return partColliders; } }

        public void ForceOnVesselPartsChange()
        {
            OnVesselPartsChange?.Invoke();
        }

        public void Start()
        {
            Initialization();
        }

        public virtual void Initialization()
        {
            if (!CompatibilityChecker.IsAllCompatible())
            {
                enabled = false;
                return;
            }

            OnVesselPartsChange = UpdateShipPartsList;
            UpdateShipPartsList();
        }

        public void TriggerPartColliderUpdate()
        {
            //Set up part collider list to easy runtime overhead with memory churning
            foreach (PartModule m in part.Modules)
            {
                if (m is FARPartModule farModule)
                {
                    if (farModule.partColliders != null)
                    {
                        partColliders = farModule.partColliders;
                        break;
                    }
                }
            }

            // For some reason fuelLine throws NRE when trying to get colliders
            if (partColliders == null)
            {
                try
                {
                    partColliders = part.GetPartColliders();
                }
                catch (NullReferenceException)
                {
                    FARLogger.Info("NullReferenceException trying to get part colliders from " + part + ", defaulting to no colliders");
                    partColliders = new Collider[0];
                }
            }
        }
        protected void UpdateShipPartsList()
        {
            VesselPartList = GetShipPartList();
        }

        public List<Part> GetShipPartList()
        {
            List<Part> list;
            if (HighLogic.LoadedSceneIsEditor)
                list = FARAeroUtil.AllEditorParts;
            else if (vessel)
                list = vessel.parts;
            else
            {
                list = new List<Part>();
                if (part)
                    list.Add(part);
            }

            return list;
        }

        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);
            if(HighLogic.LoadedSceneIsFlight || HighLogic.LoadedSceneIsEditor)
                if(!(part is CompoundPart))
                    TriggerPartColliderUpdate();
        }

        //public override void OnSave(ConfigNode node)
        //{
        //    //By blanking this nothing should be saved to the craft file or the persistance file
        //}
        protected virtual void OnDestroy()
        {
            OnVesselPartsChange = null;
            VesselPartList = null;
            partColliders = null;
        }
    }
}
