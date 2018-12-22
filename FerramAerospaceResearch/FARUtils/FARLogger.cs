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

// defining symbols here doesn't work, see https://forum.unity.com/threads/conditionalattribute-not-working.469720/
// #if !DEBUG && !INFO && !WARNING && !ERROR
// #define INFO
// #endif

using System;
using System.Diagnostics;
using System.Collections.Generic;
using UnityEngine;

namespace FerramAerospaceResearch.FARUtils
{
    public static class FARLogger
    {

        private static Dictionary<string, Stopwatch> alltimers = new Dictionary<string, Stopwatch>();
        private static Stopwatch myWatch = null;


        public static string defaultTag = $"[FAR {FARVersion.String}]";

        private static string _tag = defaultTag;

        public static string Tag
        {
            get
            {
                return _tag;
            }
            set
            {
                SetTag(value);
            }
        }

        public static string GetTag()
        {
            return _tag;
        }

        public static string GetTag(string tag)
        {
            return "[" + tag + "]";
        }

        public static string GetTag(string[] tags)
        {
            return GetTag(String.Join("] [", tags));
        }

        public static void SetTag()
        {
            _tag = defaultTag;
        }

        public static void SetTag(string tag)
        {
            _tag = GetTag(tag);
        }

        public static void SetTag(string[] tags)
        {
            _tag = GetTag(tags);
        }

        #region Info
        [Conditional("DEBUG"), Conditional("INFO")]
        public static void Info(object message)
        {
            UnityEngine.Debug.Log(Tag + " " + message);
        }

        [Conditional("DEBUG"), Conditional("INFO")]
        public static void Info(object message, UnityEngine.Object context)
        {
            UnityEngine.Debug.Log(Tag + " " + message, context);
        }

        [Conditional("DEBUG"), Conditional("INFO")]
        public static void InfoFormat(string format, params object[] args)
        {
            UnityEngine.Debug.LogFormat(Tag + " " + format, args);
        }

        [Conditional("DEBUG"), Conditional("INFO")]
        public static void InfoFormat(UnityEngine.Object context, string format, params object[] args)
        {
            UnityEngine.Debug.LogFormat(context, Tag + " " + format, args);
        }

        [Conditional("DEBUG"), Conditional("INFO")]
        public static void InfoWithCaller(object message)
        {
            UnityEngine.Debug.Log(Tag + " " + GetCallerInfo() + " - " + message);
        }

        [Conditional("DEBUG"), Conditional("INFO")]
        public static void InfoWithCaller(object message, UnityEngine.Object context)
        {
            UnityEngine.Debug.Log(Tag + " " + GetCallerInfo() + " - " + message, context);
        }

        [Conditional("DEBUG"), Conditional("INFO")]
        public static void InfoFormatWithCaller(string format, params object[] args)
        {
            UnityEngine.Debug.LogFormat(Tag + " " + GetCallerInfo() + " - " + format, args);
        }

        [Conditional("DEBUG"), Conditional("INFO")]
        public static void InfoFormatWithCaller(UnityEngine.Object context, string format, params object[] args)
        {
            UnityEngine.Debug.LogFormat(context, Tag + " " + GetCallerInfo() + " - " + format, args);
        }
        #endregion // Info

        #region Debug
        [Conditional("DEBUG")]
        public static void Debug(object message) => UnityEngine.Debug.Log(Tag + " " + message);

        [Conditional("DEBUG")]
        public static void Debug(object message, UnityEngine.Object context) => UnityEngine.Debug.Log(Tag + " " + message, context);

        [Conditional("DEBUG")]
        public static void DebugFormat(string format, params object[] args) => UnityEngine.Debug.LogFormat(Tag + " " + format, args);

        [Conditional("DEBUG")]
        public static void DebugFormat(UnityEngine.Object context, string format, params object[] args) => UnityEngine.Debug.LogFormat(context, Tag + " " + format, args);

        [Conditional("DEBUG")]
        public static void DebugWithCaller(object message) => UnityEngine.Debug.Log(Tag + " " + GetCallerInfo() + " - " + message);

        [Conditional("DEBUG")]
        public static void DebugWithCaller(object message, UnityEngine.Object context) => UnityEngine.Debug.Log(Tag + " " + GetCallerInfo() + " - " + message, context);

        [Conditional("DEBUG")]
        public static void DebugFormatWithCaller(string format, params object[] args) => UnityEngine.Debug.LogFormat(Tag + " " + GetCallerInfo() + " - " + format, args);

        [Conditional("DEBUG")]
        public static void DebugFormatWithCaller(UnityEngine.Object context, string format, params object[] args) => UnityEngine.Debug.LogFormat(context, Tag + " " + GetCallerInfo() + " - " + format, args);
        #endregion // Debug

        #region Warning
        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING")]
        public static void Warning(object message) => UnityEngine.Debug.LogWarning(Tag + " " + message);

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING")]
        public static void Warning(object message, UnityEngine.Object context) => UnityEngine.Debug.LogWarning(Tag + " " + message, context);

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING")]
        public static void WarningFormat(string format, params object[] args) => UnityEngine.Debug.LogWarningFormat(Tag + " " + format, args);

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING")]
        public static void WarningFormat(UnityEngine.Object context, string format, params object[] args) => UnityEngine.Debug.LogWarningFormat(context, Tag + " " + format, args);

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING")]
        public static void WarningWithCaller(object message) => UnityEngine.Debug.LogWarning(Tag + " " + GetCallerInfo() + " - " + message);

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING")]
        public static void WarningWithCaller(object message, UnityEngine.Object context) => UnityEngine.Debug.LogWarning(Tag + " " + GetCallerInfo() + " - " + message, context);

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING")]
        public static void WarningFormatWithCaller(string format, params object[] args) => UnityEngine.Debug.LogWarningFormat(Tag + " " + GetCallerInfo() + " - " + format, args);

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING")]
        public static void WarningFormatWithCaller(UnityEngine.Object context, string format, params object[] args) => UnityEngine.Debug.LogWarningFormat(context, Tag + " " + GetCallerInfo() + " - " + format, args);
        #endregion // Warning

        #region Error
        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING"), Conditional("ERROR")]
        public static void Error(object message) => UnityEngine.Debug.LogError(Tag + " " + message);

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING"), Conditional("ERROR")]
        public static void Error(object message, UnityEngine.Object context) => UnityEngine.Debug.LogError(Tag + " " + message, context);

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING"), Conditional("ERROR")]
        public static void ErrorFormat(string format, params object[] args) => UnityEngine.Debug.LogErrorFormat(Tag + " " + format, args);

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING"), Conditional("ERROR")]
        public static void ErrorFormat(UnityEngine.Object context, string format, params object[] args) => UnityEngine.Debug.LogErrorFormat(context, Tag + " " + format, args);

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING"), Conditional("ERROR")]
        public static void ErrorWithCaller(object message) => UnityEngine.Debug.LogError(Tag + " " + GetCallerInfo() + " - " + message);

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING"), Conditional("ERROR")]
        public static void ErrorWithCaller(object message, UnityEngine.Object context) => UnityEngine.Debug.LogError(Tag + " " + GetCallerInfo() + " - " + message, context);

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING"), Conditional("ERROR")]
        public static void ErrorFormatWithCaller(string format, params object[] args) => UnityEngine.Debug.LogErrorFormat(Tag + " " + GetCallerInfo() + " - " + format, args);

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING"), Conditional("ERROR")]
        public static void ErrorFormatWithCaller(UnityEngine.Object context, string format, params object[] args) => UnityEngine.Debug.LogErrorFormat(context, Tag + " " + GetCallerInfo() + " - " + format, args);
        #endregion // Error

        #region Assertion
        public static void Assertion(object message) => UnityEngine.Debug.LogAssertion(Tag + " " + message);

        public static void Assertion(object message, UnityEngine.Object context) => UnityEngine.Debug.LogAssertion(Tag + " " + message, context);

        public static void AssertionFormat(string format, params object[] args) => UnityEngine.Debug.LogAssertionFormat(Tag + " " + format, args);

        public static void AssertionFormat(UnityEngine.Object context, string format, params object[] args) => UnityEngine.Debug.LogAssertionFormat(context, Tag + " " + format, args);

        public static void AssertionWithCaller(object message) => UnityEngine.Debug.LogAssertion(Tag + " " + GetCallerInfo() + " - " + message);

        public static void AssertionWithCaller(object message, UnityEngine.Object context) => UnityEngine.Debug.LogAssertion(Tag + " " + GetCallerInfo() + " - " + message, context);

        public static void AssertionFormatWithCaller(string format, params object[] args) => UnityEngine.Debug.LogAssertionFormat(Tag + " " + GetCallerInfo() + " - " + format, args);

        public static void AssertionFormatWithCaller(UnityEngine.Object context, string format, params object[] args) => UnityEngine.Debug.LogAssertionFormat(context, Tag + " " + GetCallerInfo() + " - " + format, args);
        #endregion // Assertion

        #region Exception
        public static void Exception(Exception exception)
        {
            Error("Logged exception:");
            UnityEngine.Debug.LogException(exception);
        }

        public static void Exception(Exception exception, UnityEngine.Object context)
        {
            Error("Logged exception:");
            UnityEngine.Debug.LogException(exception, context);
        }

        public static void ExceptionWithCaller(Exception exception)
        {
            Error(GetCallerInfo() + " - Logged exception:");
            UnityEngine.Debug.LogException(exception);
        }

        public static void ExceptionWithCaller(Exception exception, UnityEngine.Object context)
        {
            Error(GetCallerInfo() + " - Logged exception:");
            UnityEngine.Debug.LogException(exception, context);
        }
        #endregion // Exception

        #region Performace
        internal static void PerfStart(string id = "default")
        {
            myWatch = new Stopwatch();

            if (alltimers.ContainsKey(id))
            {
                alltimers.Remove(id);
            }
            alltimers.Add(id, myWatch);
            myWatch.Start();
        }

        /// <summary>
        /// resets the Stopwatchtimer with the (string)id and prints the result.
        /// </summary>
        /// <param name="id"></param>
        internal static void PerfStop(string id = "default")
        {
            if (alltimers.TryGetValue(id, out myWatch))
            {
                myWatch.Stop();
                FARLogger.Info("Stopwatch: \"" + id + "\" elapsed time: " + myWatch.Elapsed);
                myWatch.Reset();
                alltimers.Remove(id);
            }
        }

        /// <summary>
        /// Pauses the timer with the id
        /// </summary>
        /// <param name="id"></param>
        internal static void PerfPause(string id = "default")
        {
            if (alltimers.TryGetValue(id, out myWatch))
            {
                myWatch.Stop();
            }
        }

        /// <summary>
        /// resumes the timer
        /// </summary>
        /// <param name="id"></param>
        internal static void PerfContinue(string id = "default")
        {
            if (alltimers.TryGetValue(id, out myWatch))
            {
                myWatch.Start();
            }
        }

        #endregion


        //http://geekswithblogs.net/BlackRabbitCoder/archive/2013/07/25/c.net-little-wonders-getting-caller-information.aspx
        private static string GetCallerInfo()
        {
            // frame 0 - GetCallerInfo
            // frame 1 - one of the log methods
            // frame 2 - source of the log method caller
            //             StackFrame frame = new StackFrame(2, true);
            //             string method = frame.GetMethod().Name;
            // #if DEBUG
            //             // release mode doesn't have debug symbols
            //             string fileName = frame.GetFileName();
            //             int lineNumber = frame.GetFileLineNumber();

            //             return $"{fileName}({lineNumber}):{method}";
            // #else
            //             return method
            // #endif
            string trace = System.Environment.StackTrace;
            if (String.IsNullOrEmpty(trace))
            {
                return "";
            }
            string[] lines = trace.Split(new[] { "\r\n", "\r", "\n" }, StringSplitOptions.None);
            string caller = lines[3].Trim();
            return caller.Substring(0, caller.IndexOf("("));
        }

#if NET_NEWER_4_5
        // For reference if KSP .NET version is updated
        // there's a much faster method to obtain caller info in .NET >= 4.5
        public static string MessageWithCallerInfo(object message,
            [CallerMemberName] string memberName = "",
            [CallerFilePath] string fileName = "",
            [CallerLineNumber] int lineNumber = 0)
        {
            return fileName + "(" + lineNumber + "):" + memberName + " - " + message;
        }
#endif
    }
}
