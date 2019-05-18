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

// defining symbols here doesn't work, see https://forum.unity.com/threads/conditionalattribute-not-working.469720/
// #if !DEBUG && !INFO && !WARNING && !ERROR
// #define INFO
// #endif

using System;
using System.Diagnostics;
using Object = UnityEngine.Object;

// ReSharper disable InvocationIsSkipped UnusedMember.Global

namespace FerramAerospaceResearch.FARUtils
{
    public static class FARLogger
    {

        public static readonly string defaultTag = $"[FAR {FARVersion.String}] ";
        private static readonly string[] separators = {"\r\n", "\r", "\n"};

        public static string Tag { get; } = defaultTag;

        #region Info
        [Conditional("DEBUG"), Conditional("INFO")]
        public static void Info(object message)
        {
            UnityEngine.Debug.Log(Tag + message);
        }

        [Conditional("DEBUG"), Conditional("INFO")]
        public static void Info(object message, Object context)
        {
            UnityEngine.Debug.Log(Tag + message, context);
        }

        [Conditional("DEBUG"), Conditional("INFO")]
        public static void InfoFormat(string format, params object[] args)
        {
            UnityEngine.Debug.LogFormat(Tag + format, args);
        }

        [Conditional("DEBUG"), Conditional("INFO")]
        public static void InfoFormat(Object context, string format, params object[] args)
        {
            UnityEngine.Debug.LogFormat(context, Tag + format, args);
        }

        [Conditional("DEBUG"), Conditional("INFO")]
        public static void InfoWithCaller(object message)
        {
            UnityEngine.Debug.Log(Tag + GetCallerInfo() + " - " + message);
        }

        [Conditional("DEBUG"), Conditional("INFO")]
        public static void InfoWithCaller(object message, Object context)
        {
            UnityEngine.Debug.Log(Tag + GetCallerInfo() + " - " + message, context);
        }

        [Conditional("DEBUG"), Conditional("INFO")]
        public static void InfoFormatWithCaller(string format, params object[] args)
        {
            UnityEngine.Debug.LogFormat(Tag + GetCallerInfo() + " - " + format, args);
        }

        [Conditional("DEBUG"), Conditional("INFO")]
        public static void InfoFormatWithCaller(Object context, string format, params object[] args)
        {
            UnityEngine.Debug.LogFormat(context, Tag + GetCallerInfo() + " - " + format, args);
        }

        #endregion // Info

        #region Debug
        [Conditional("DEBUG")]
        public static void Debug(object message)
        {
            UnityEngine.Debug.Log(Tag + message);
        }

        [Conditional("DEBUG")]
        public static void Debug(object message, Object context)
        {
            UnityEngine.Debug.Log(Tag + message, context);
        }

        [Conditional("DEBUG")]
        public static void DebugFormat(string format, params object[] args)
        {
            UnityEngine.Debug.LogFormat(Tag + format, args);
        }

        [Conditional("DEBUG")]
        public static void DebugFormat(Object context, string format, params object[] args)
        {
            UnityEngine.Debug.LogFormat(context, Tag + format, args);
        }

        [Conditional("DEBUG")]
        public static void DebugWithCaller(object message)
        {
            UnityEngine.Debug.Log(Tag + GetCallerInfo() + " - " + message);
        }

        [Conditional("DEBUG")]
        public static void DebugWithCaller(object message, Object context)
        {
            UnityEngine.Debug.Log(Tag + GetCallerInfo() + " - " + message, context);
        }

        [Conditional("DEBUG")]
        public static void DebugFormatWithCaller(string format, params object[] args)
        {
            UnityEngine.Debug.LogFormat(Tag + GetCallerInfo() + " - " + format, args);
        }

        [Conditional("DEBUG")]
        public static void DebugFormatWithCaller(Object context, string format, params object[] args)
        {
            UnityEngine.Debug.LogFormat(context, Tag + GetCallerInfo() + " - " + format, args);
        }

        #endregion // Debug

        #region Warning
        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING")]
        public static void Warning(object message)
        {
            UnityEngine.Debug.LogWarning(Tag + message);
        }

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING")]
        public static void Warning(object message, Object context)
        {
            UnityEngine.Debug.LogWarning(Tag + message, context);
        }

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING")]
        public static void WarningFormat(string format, params object[] args)
        {
            UnityEngine.Debug.LogWarningFormat(Tag + format, args);
        }

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING")]
        public static void WarningFormat(Object context, string format, params object[] args)
        {
            UnityEngine.Debug.LogWarningFormat(context, Tag + format, args);
        }

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING")]
        public static void WarningWithCaller(object message)
        {
            UnityEngine.Debug.LogWarning(Tag + GetCallerInfo() + " - " + message);
        }

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING")]
        public static void WarningWithCaller(object message, Object context)
        {
            UnityEngine.Debug.LogWarning(Tag + GetCallerInfo() + " - " + message, context);
        }

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING")]
        public static void WarningFormatWithCaller(string format, params object[] args)
        {
            UnityEngine.Debug.LogWarningFormat(Tag + GetCallerInfo() + " - " + format, args);
        }

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING")]
        public static void WarningFormatWithCaller(Object context, string format, params object[] args)
        {
            UnityEngine.Debug.LogWarningFormat(context, Tag + GetCallerInfo() + " - " + format, args);
        }

        #endregion // Warning

        #region Error
        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING"), Conditional("ERROR")]
        public static void Error(object message)
        {
            UnityEngine.Debug.LogError(Tag + message);
        }

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING"), Conditional("ERROR")]
        public static void Error(object message, Object context)
        {
            UnityEngine.Debug.LogError(Tag + message, context);
        }

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING"), Conditional("ERROR")]
        public static void ErrorFormat(string format, params object[] args)
        {
            UnityEngine.Debug.LogErrorFormat(Tag + format, args);
        }

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING"), Conditional("ERROR")]
        public static void ErrorFormat(Object context, string format, params object[] args)
        {
            UnityEngine.Debug.LogErrorFormat(context, Tag + format, args);
        }

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING"), Conditional("ERROR")]
        public static void ErrorWithCaller(object message)
        {
            UnityEngine.Debug.LogError(Tag + GetCallerInfo() + " - " + message);
        }

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING"), Conditional("ERROR")]
        public static void ErrorWithCaller(object message, Object context)
        {
            UnityEngine.Debug.LogError(Tag + GetCallerInfo() + " - " + message, context);
        }

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING"), Conditional("ERROR")]
        public static void ErrorFormatWithCaller(string format, params object[] args)
        {
            UnityEngine.Debug.LogErrorFormat(Tag + GetCallerInfo() + " - " + format, args);
        }

        [Conditional("DEBUG"), Conditional("INFO"), Conditional("WARNING"), Conditional("ERROR")]
        public static void ErrorFormatWithCaller(Object context, string format, params object[] args)
        {
            UnityEngine.Debug.LogErrorFormat(context, Tag + GetCallerInfo() + " - " + format, args);
        }

        #endregion // Error

        #region Assertion
        [Conditional("UNITY_ASSERTIONS")]
        public static void Assertion(object message)
        {
            UnityEngine.Debug.LogAssertion(Tag + message);
        }

        [Conditional("UNITY_ASSERTIONS")]
        public static void Assertion(object message, Object context)
        {
            UnityEngine.Debug.LogAssertion(Tag + message, context);
        }

        [Conditional("UNITY_ASSERTIONS")]
        public static void AssertionFormat(string format, params object[] args)
        {
            UnityEngine.Debug.LogAssertionFormat(Tag + format, args);
        }

        [Conditional("UNITY_ASSERTIONS")]
        public static void AssertionFormat(Object context, string format, params object[] args)
        {
            UnityEngine.Debug.LogAssertionFormat(context, Tag + format, args);
        }

        [Conditional("UNITY_ASSERTIONS")]
        public static void AssertionWithCaller(object message)
        {
            UnityEngine.Debug.LogAssertion(Tag + GetCallerInfo() + " - " + message);
        }

        [Conditional("UNITY_ASSERTIONS")]
        public static void AssertionWithCaller(object message, Object context)
        {
            UnityEngine.Debug.LogAssertion(Tag + GetCallerInfo() + " - " + message, context);
        }

        [Conditional("UNITY_ASSERTIONS")]
        public static void AssertionFormatWithCaller(string format, params object[] args)
        {
            UnityEngine.Debug.LogAssertionFormat(Tag + GetCallerInfo() + " - " + format, args);
        }

        [Conditional("UNITY_ASSERTIONS")]
        public static void AssertionFormatWithCaller(Object context, string format, params object[] args)
        {
            UnityEngine.Debug.LogAssertionFormat(context, Tag + GetCallerInfo() + " - " + format, args);
        }

        #endregion // Assertion

        #region Exception
        public static void Exception(Exception exception)
        {
            Error("Logged exception:");
            UnityEngine.Debug.LogException(exception);
        }

        public static void Exception(Exception exception, Object context)
        {
            Error("Logged exception:");
            UnityEngine.Debug.LogException(exception, context);
        }

        public static void ExceptionWithCaller(Exception exception)
        {
            Error(GetCallerInfo() + " - Logged exception:");
            UnityEngine.Debug.LogException(exception);
        }

        public static void ExceptionWithCaller(Exception exception, Object context)
        {
            Error(GetCallerInfo() + " - Logged exception:");
            UnityEngine.Debug.LogException(exception, context);
        }
        #endregion // Exception

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
            string trace = Environment.StackTrace;
            if (String.IsNullOrEmpty(trace))
            {
                return "";
            }
            string[] lines = trace.Split(separators, StringSplitOptions.None);
            string caller = lines[3].Trim();
            return caller.Substring(0, caller.IndexOf("(", StringComparison.Ordinal));
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
