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
using UnityEngine;
using FerramAerospaceResearch.PartExtensions;
using FerramAerospaceResearch;
using FerramAerospaceResearch.FARUtils;

namespace ferram4
{
    public class FARPartModule : FARCloneablePartModule
    {
        protected Callback OnVesselPartsChange;
        public List<Part> VesselPartList = null;
        int VesselPartListCount = 0;
        private Collider[] partColliders = null;

        public Collider[] PartColliders { get { if(partColliders == null) TriggerPartColliderUpdate(); return partColliders; } protected set { partColliders = value; } }

        public FARPartModule() : base() { }

        public void ForceOnVesselPartsChange()
        {
            if(OnVesselPartsChange != null)
                OnVesselPartsChange();
        }

        public void Start()
        {
            Initialization();
        }

        public virtual void Initialization()
        {
            if (!CompatibilityChecker.IsAllCompatible())
            {
                this.enabled = false;
                return;
            }

            OnVesselPartsChange = UpdateShipPartsList;
            UpdateShipPartsList();
        }

        public void TriggerPartColliderUpdate()
        {
            //Set up part collider list to easy runtime overhead with memory churning
            for (int i = 0; i < part.Modules.Count; i++)
            {
                PartModule m = part.Modules[i];
                if (m is FARPartModule)
                {
                    FARPartModule farModule = (m as FARPartModule);
                    if (farModule.partColliders != null)
                    {
                        this.partColliders = farModule.partColliders;
                        break;
                    }
                }
            }
            // For some reason fuelLine throws NRE when trying to get colliders
            if (this.partColliders == null)
            {
                try
                {
                    this.partColliders = part.GetPartColliders();
                }
                catch (NullReferenceException)
                {
                    FARLogger.Info("NullReferenceException trying to get part colliders from " + part + ", defaulting to no colliders");
                    this.partColliders = new Collider[0];
                }
            }
        }
        protected void UpdateShipPartsList()
        {
            VesselPartList = GetShipPartList();
        }

        public List<Part> GetShipPartList()
        {
            List<Part> list = null;
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
            VesselPartListCount = list.Count;

//            print("Updated Vessel Part List...");

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

        protected FARPartModule(FARPartModule other, Dictionary<Guid, object> cache) : base(other, cache)
        {
            // Only do a shallow copy of KSP objects since they are not mutated here
            VesselPartList = FARCloneHelper.ShallowCopy(other.VesselPartList);
            VesselPartListCount = other.VesselPartListCount;
            partColliders = FARCloneHelper.ShallowCopy(other.partColliders);

            // For now keeping the callback null, no need to update vessel part list on a clone
        }

        public override object Clone(IFARCloneable other, Dictionary<Guid, object> cache) => new FARPartModule((FARPartModule)other, cache);
    }
}
