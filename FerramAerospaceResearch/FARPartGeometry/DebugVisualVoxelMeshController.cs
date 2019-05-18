/*
Ferram Aerospace Research v0.15.10.1 "Lundgren"
=========================
Copyright 2019, Daumantas Kavolis, aka dkavolis

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
using FerramAerospaceResearch.FARThreading;
using FerramAerospaceResearch.FARUtils;
using UnityEngine;
using Object = UnityEngine.Object;

namespace FerramAerospaceResearch.FARPartGeometry
{
    public class DebugVisualVoxelMeshController : IDisposable
    {
        // limit of vertices in each mesh imposed by Unity
        public const int MaxVerticesPerSubmesh = 65000;
        public const int MaxVoxelsPerSubmesh = MaxVerticesPerSubmesh / 4;
        private readonly List<DebugVisualVoxel> debugVoxels = new List<DebugVisualVoxel>();
        private readonly List<DebugVisualVoxelSubmesh> submeshes = new List<DebugVisualVoxelSubmesh>();
        private bool active;

        public DebugVisualVoxelMeshController(Transform parent = null)
        {
            Parent = parent;
        }

        public bool Active
        {
            // ReSharper disable once UnusedMember.Global
            get { return active; }
            set
            {
                if (active != value)
                {
                    active = value;
                    QueueMainThreadTask(UpdateActive);
                }
            }
        }

        internal List<DebugVisualVoxel> DebugVoxels
        {
            get { return debugVoxels; }
        }

        public Transform Parent { get; }

        private void UpdateActive()
        {
            foreach (DebugVisualVoxelSubmesh submesh in submeshes)
            {
                if (submesh != null)
                    submesh.Active = active;
            }
        }

        public void Rebuild()
        {
            FARLogger.Info("Rebuilding visual voxel mesh...");
            Clear();
            int meshes = debugVoxels.Count / MaxVoxelsPerSubmesh + 1;
            FARLogger.Info("Voxel mesh contains " + debugVoxels.Count + " voxels in " + meshes + " submeshes");
            SetupSubmeshes(meshes);
            for (int i = 0; i < meshes; i++)
            {
                for (int j = i * MaxVoxelsPerSubmesh; j < Math.Min((i + 1) * MaxVoxelsPerSubmesh, debugVoxels.Count); j++)
                {
                    debugVoxels[j].AddToMesh(submeshes[i].Vertices, submeshes[i].Uvs, submeshes[i].Triangles);
                }
                submeshes[i].Rebuild();
            }
            FARLogger.Info("Finished rebuilding visual voxel mesh.");
        }

        public void RebuildSafe()
        {
            QueueMainThreadTask(Rebuild);
        }

        public void Clear(bool clearVoxels = false)
        {
            foreach (DebugVisualVoxelSubmesh submesh in submeshes)
            {
                submesh.Clear();
            }
            if (clearVoxels)
            {
                debugVoxels.Clear();
            }
        }

        private void SetupSubmeshes(int meshes)
        {
            for (int i = submeshes.Count; i < meshes; i++)
            {
                submeshes.Add(DebugVisualVoxelSubmesh.Create(Parent, active));
            }
        }

        public void Dispose()
        {
            QueueMainThreadTask(DisposeSafe);
        }

        private void DisposeSafe()
        {
            foreach (DebugVisualVoxelSubmesh submesh in submeshes)
            {
                Object.Destroy(submesh);
            }
        }

        private void QueueMainThreadTask(Action action)
        {
            if (VoxelizationThreadpool.Instance.inMainThread)
            {
                FARLogger.Debug("In main thread, not queueing " + action.Method.Name);
                action();
            }
            else
            {
                ThreadSafeDebugLogger.Instance.RegisterDebugMessage("Running " + action.Method.Name + " in main thread");
                VoxelizationThreadpool.Instance.RunOnMainThread(action);
            }
        }

    }
}
