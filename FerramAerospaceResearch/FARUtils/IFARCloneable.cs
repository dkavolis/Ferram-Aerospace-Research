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

namespace FerramAerospaceResearch.FARUtils
{

    /// <summary>
    /// An interface to uniquely indetify objects.
    /// </summary>
    public interface IFARIdentifiable
    {
        /// <summary>
        /// Unique object identifier. GetHashCode is not guaranteed to be unique, see
        /// <a href="https://docs.microsoft.com/en-us/dotnet/api/system.object.gethashcode?redirectedfrom=MSDN&view=netframework-4.7.2#System_Object_GetHashCode">here</a>
        /// </summary>
        Guid GUID { get; }

        /// <summary>
        /// Whether this object is a clone.
        /// </summary>
        bool isClone { get; }
    }

    /// <summary>
    /// An interface that should clone the object by deep copying.
    /// </summary>
    public interface IFARCloneable : IFARIdentifiable
    {
        /// <summary>
        /// Abstract method that should perform deep copy of the object.
        /// </summary>
        /// <returns>Clone of the implementing object.</returns>
        T Clone<T>() where T : IFARCloneable;

        /// <summary>
        /// Abstract method that should perform deep copy of the object using the cloned object references in
        /// <paramref name="cache"/>.
        /// </summary>
        /// <param name="cache">Dictionary of the cloned objects. Keys are the original objects GUIDs.</param>
        /// <returns>Clone of the implementing object.</returns>
        T Clone<T>(Dictionary<Guid, object> cache) where T : IFARCloneable;

        /// <summary>
        /// Abstract method that should perform deep copy of <paramref name="other"/> using the cloned object references.
        /// Keeping the return and input types broad to allow for virtual implementation. Ideally, this method should
        /// always be called from the other two - the first one initializing the cache and calling the second one,
        /// and the second one calling this method with the current object as an argument.
        /// in <paramref name="cache"/>.
        /// </summary>
        /// <param name="other">Object to copy.</param>
        /// <param name="cache">Dictionary of the cloned objects. Keys are the original objects GUIDs.</param>
        /// <returns>Clone of <paramref name="other"/>.</returns>
        object Clone(IFARCloneable other, Dictionary<Guid, object> cache);
    }
}
