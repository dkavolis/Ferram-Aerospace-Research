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
    /// A helper class for IFARCloneable&lt;T&gt; since C# 7.0 does not support multiple inheritance (it is allowed in
    /// C# 8.0+). This should be used via object aggregation. The IFARCloneable implementing class should call
    /// corresponding constructor and pass GUID getter, Clone() and Clone(cache) methods to an instance of
    /// FARCloneHelper. Clone(other, cache) should redirect to a copy constructor with the same signature.
    /// </summary>
    public class FARCloneHelper
    {
        // Using guid for unique identifiers since it is thread safe. We could use a counter and an int ID
        // but then counter increment and id setter would have to be manually locked for thread safety.
        private Guid guid;
        private bool cloned;

        public Guid GUID
        {
            get
            {
                return guid;
            }
        }

        public bool isClone
        {
            get
            {
                return cloned;
            }
        }

        public FARCloneHelper() => guid = Guid.NewGuid();

        /// <summary>
        /// Constructor that additionally adds clone <paramref name="clone"/> of <paramref name="original"/> to <paramref name="cache"/>.
        /// </summary>
        /// <param name="original">Object being cloned.</param>
        /// <param name="clone">Clone of <paramref name="original"/>.</param>
        /// <param name="cache">Cache of cloned objects, keys are GUIDs of parents</param>
        /// <example>
        /// In copy constructor <c>T(IFARCloneable&lt;T> other, Dictionary&lt;Guid, object> cache)</c>:
        /// <code>
        /// this.cloner = new FARCloneHelper(other, this, cache);
        /// </code>
        /// </example>
        public FARCloneHelper(IFARIdentifiable original, IFARIdentifiable clone, Dictionary<Guid, object> cache) : this()
        {
            cache.Add(original.GUID, clone);
            cloned = true;
        }

        /// <summary>
        /// Generic clone method. Instantiates an empty cache used to store references to cloned objects.
        /// </summary>
        /// <param name="original">Object to clone.</param>
        /// <returns>Clone of the current object.</returns>
        public static T Clone<T>(IFARCloneable original) where T : IFARCloneable
        {
            Dictionary<Guid, object> cache = new Dictionary<Guid, object>();
            return Clone<T>(original, cache);
        }

        /// <summary>
        /// Generic clone method that first searches the cache for the cloned object. Should be called from copy constructors.
        /// </summary>
        /// <param name="original">Object to clone.</param>
        /// <param name="cache">Cache of cloned objects.</param>
        /// <returns>Clone of <paramref name="original"/> of type <typeparamref name="T"/>.</returns>
        public static T Clone<T>(IFARCloneable original, Dictionary<Guid, object> cache) where T : IFARCloneable
        {
            if (original == null)
                return default(T);

            object clone;
            if (cache.TryGetValue(original.GUID, out clone))
                return (T)clone;

            if (original.isClone)
                return (T)original;

            if (!(original is T))
                throw new ArgumentException($"Invalid argument type {original.GetType().FullName}, expected {typeof(T).FullName}.");

            return (T)original.Clone(original, cache);
        }

        public static List<T> ShallowCopy<T>(List<T> other)
        {
            if (other == null)
                return null;
            return other.ShallowCopy();
        }

        public static T[] ShallowCopy<T>(T[] other)
        {
            if (other == null)
                return null;
            return other.ShallowCopy();
        }

        public static List<T> DeepCopy<T>(List<T> other, Dictionary<Guid, object> cache) where T : IFARCloneable
        {
            if (other == null)
                return null;
            return other.DeepCopy(cache);
        }
    }
}
