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
using UnityEngine;

namespace FerramAerospaceResearch.FARPartGeometry
{
    internal class VoxelChunk
    {
        //private Part[] voxelPoints = null;
        //private float[] voxelSize = null;
        private readonly PartSizePair[] voxelPoints;
        private DebugVisualVoxel[, ,] visualVoxels;
        private HashSet<Part> overridingParts;

        private double _size;

        private Vector3d lowerCorner;
        //int iOffset, jOffset, kOffset;
        private int offset;

        public VoxelChunk(double size, Vector3d lowerCorner, int iOffset, int jOffset, int kOffset, HashSet<Part> overridingParts, bool usePartSize256)
        {
            _size = size;
            offset = iOffset + 8 * jOffset + 64 * kOffset;
            //voxelPoints = new Part[512];
            //voxelSize = new float[512];
            voxelPoints = new PartSizePair[512];
            if (usePartSize256)
            {
                for (int i = 0; i < voxelPoints.Length; i++)
                    voxelPoints[i] = new PartSizePair8Bit();
            }
            else
            {
                for (int i = 0; i < voxelPoints.Length; i++)
                    voxelPoints[i] = new PartSizePair4Bit();
            }


            this.lowerCorner = lowerCorner;
            this.overridingParts = overridingParts;
        }

        // ReSharper disable ParameterHidesMember -> update member values
        public void SetChunk(double size, Vector3d lowerCorner, int iOffset, int jOffset, int kOffset, HashSet<Part> overridingParts)
        {
            _size = size;
            offset = iOffset + 8 * jOffset + 64 * kOffset;
            this.lowerCorner = lowerCorner;
            this.overridingParts = overridingParts;
        }
        // ReSharper restore ParameterHidesMember

        public void ClearChunk()
        {
            //_size = 0;
            offset = 0;
            lowerCorner = Vector3d.zero;
            foreach (PartSizePair voxelPoint in voxelPoints)
                voxelPoint.Clear();

            overridingParts = null;
        }

        //Use when locking is unnecessary and only to change size, not part
        public void SetVoxelPointGlobalIndexNoLock(int zeroBaseIndex, byte location, VoxelOrientationPlane plane = VoxelOrientationPlane.FILL_VOXEL)
        {
            zeroBaseIndex -= offset;
            //voxelPoints[zeroBaseIndex].part = p;
            voxelPoints[zeroBaseIndex].SetPlaneLocation(plane, location);
        }

        //Use when certain that locking is unnecessary and need to fill the location
        public void SetVoxelPointPartOnlyGlobalIndexNoLock(int zeroBaseIndex, Part p)
        {
            zeroBaseIndex -= offset;
            SetPart(p, zeroBaseIndex, VoxelOrientationPlane.NONE, 0);
        }

        // ReSharper disable once UnusedMember.Global
        public void SetVoxelPointPartOnlyGlobalIndexNoLock(int i, int j, int k, Part p)
        {
            int index = i + 8 * j + 64 * k - offset;
            SetPart(p, index, VoxelOrientationPlane.NONE, 0);
        }
        //Use when certain that locking is unnecessary and need to fill the location
        public void SetVoxelPointGlobalIndexNoLock(int zeroBaseIndex, Part p, byte location, VoxelOrientationPlane plane = VoxelOrientationPlane.FILL_VOXEL)
        {
            zeroBaseIndex -= offset;
            SetPart(p, zeroBaseIndex, plane, location);
        }

        // ReSharper disable once UnusedMember.Global
        public void SetVoxelPointGlobalIndexNoLock(int i, int j, int k, Part p, byte location, VoxelOrientationPlane plane = VoxelOrientationPlane.FILL_VOXEL)
        {
            int index = i + 8 * j + 64 * k - offset;
            SetPart(p, index, plane, location);
        }
        //Sets point and ensures that includedParts includes p
        public void SetVoxelPointGlobalIndex(int zeroBaseIndex, Part p, byte location, VoxelOrientationPlane plane = VoxelOrientationPlane.FILL_VOXEL)
        {
            zeroBaseIndex -= offset;

            lock (voxelPoints)
            {
                SetPart(p, zeroBaseIndex, plane, location);
            }
        }

        // ReSharper disable once UnusedMember.Global
        public void SetVoxelPointGlobalIndex(int i, int j, int k, Part p, byte location, VoxelOrientationPlane plane = VoxelOrientationPlane.FILL_VOXEL)
        {
            int index = i + 8 * j + 64 * k - offset;

            lock (voxelPoints)
            {
                SetPart(p, index, plane, location);
            }
        }

        private void SetPart(Part p, int index, VoxelOrientationPlane plane, byte location)
        {
            PartSizePair pair = voxelPoints[index];

            Part currentPart = pair.part;
            //if we update the plane location with this, then we can consider replacing the part here.  Otherwise, we don't
            bool largerThanLast = pair.SetPlaneLocation(plane, location);
            if (currentPart is null || overridingParts.Contains(p) || (largerThanLast && !overridingParts.Contains(currentPart)))
                pair.part = p;

        }

        // ReSharper disable once UnusedMember.Global
        public bool VoxelPointExistsLocalIndex(int zeroBaseIndex)
        {
            return (voxelPoints[zeroBaseIndex].GetSize() > 0);
        }

        // ReSharper disable once UnusedMember.Global
        public bool VoxelPointExistsLocalIndex(int i, int j, int k)
        {
            int index = i + 8 * j + 64 * k;
            return (voxelPoints[index].GetSize() > 0);
        }

        public bool VoxelPointExistsGlobalIndex(int zeroBaseIndex)
        {
            return (voxelPoints[zeroBaseIndex - offset].GetSize() > 0);
        }

        // ReSharper disable once UnusedMember.Global
        public bool VoxelPointExistsGlobalIndex(int i, int j, int k)
        {
            int index = i + 8 * j + 64 * k - offset;
            return (voxelPoints[index].GetSize() > 0);
        }


        public Part GetVoxelPartGlobalIndex(int zeroBaseIndex)
        {
            int index = zeroBaseIndex - offset;
            Part p = voxelPoints[index].part;
            return p;
        }

        // ReSharper disable once UnusedMember.Global
        public Part GetVoxelPartGlobalIndex(int i, int j, int k)
        {
            int index = i + 8 * j + 64 * k - offset;
            Part p = voxelPoints[index].part;

            return p;
        }

        public PartSizePair GetVoxelPartSizePairGlobalIndex(int zeroBaseIndex)
        {
            int index = zeroBaseIndex - offset;
            return voxelPoints[index];
        }

        // ReSharper disable once UnusedMember.Global
        public PartSizePair GetVoxelPartSizePairGlobalIndex(int i, int j, int k)
        {
            int index = i + 8 * j + 64 * k - offset;

            return voxelPoints[index];
        }

        public void VisualizeVoxels(Matrix4x4 vesselLocalToWorldMatrix, DebugVisualVoxelMeshController voxelMesh)
        {
            ClearVisualVoxels();
            visualVoxels = new DebugVisualVoxel[8, 8, 8];
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    for (int k = 0; k < 8; k++)
                    {
                        //if(voxelPoints[i,j,k] != null)
                        PartSizePair pair = voxelPoints[i + 8 * j + 64 * k];
                        if ((object) pair.part != null)
                        {
                            double elementSize = pair.GetSize();
                            if (elementSize > 1)
                                elementSize = 1;

                            elementSize *= _size * 0.5f;
                            var vx = new DebugVisualVoxel(vesselLocalToWorldMatrix.MultiplyPoint3x4(lowerCorner + new Vector3d(i, j, k) * _size), elementSize);
                            voxelMesh.DebugVoxels.Add(vx);
                            visualVoxels[i, j, k] = vx;
                        }
                    }
        }

        public void ClearVisualVoxels()
        {
            if (visualVoxels != null)
                for (int i = 0; i < 8; i++)
                    for (int j = 0; j < 8; j++)
                        for (int k = 0; k < 8; k++)
                        {
                            visualVoxels[i, j, k] = null;
                        }
        }

        //~VoxelChunk()
        //{
        //    ClearVisualVoxels();
        //}
    }

    //[Flags]
    public enum VoxelOrientationPlane
    {
        NONE = 0,
        X_UP = 1,
        X_DOWN = 2,
        Y_UP = 4,
        Y_DOWN = 8,
        Z_UP = 16,
        Z_DOWN = 32,
        FILL_VOXEL = 64
    }
}
