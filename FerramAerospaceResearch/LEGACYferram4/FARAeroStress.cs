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

namespace FerramAerospaceResearch
{
    public static class FARAeroStress
    {
        public static readonly List<FARPartStressTemplate> StressTemplates = new List<FARPartStressTemplate>();
        public static bool loaded;



        public static void SaveCustomStressTemplates()
        {
            ConfigNode node = new ConfigNode("@FARAeroStress[default]:FOR[FerramAerospaceResearch]");
            node.AddNode(new ConfigNode("!FARPartStressTemplate,*"));

            foreach (FARPartStressTemplate template in StressTemplates)
            {
                node.AddNode(CreateAeroStressConfigNode(template));
            }

            ConfigNode saveNode = new ConfigNode();
            saveNode.AddNode(node);
            saveNode.Save(KSPUtil.ApplicationRootPath.Replace("\\", "/") + "GameData/FerramAerospaceResearch/CustomFARAeroStress.cfg");
        }

        private static ConfigNode CreateAeroStressConfigNode(FARPartStressTemplate template)
        {
            ConfigNode node = new ConfigNode("FARPartStressTemplate");
            node.AddValue("name", template.name);
            node.AddValue("YmaxStress", template.YmaxStress);
            node.AddValue("XZmaxStress", template.XZmaxStress);
            node.AddValue("requiresCrew", template.crewed.ToString());
            node.AddValue("isSpecialTemplate", template.isSpecialTemplate.ToString());

            ConfigNode res = new ConfigNode("Resources");

            res.AddValue("numReq", template.minNumResources);
            res.AddValue("rejectUnlistedResources", template.rejectUnlistedResources);

            //Make sure to update this whenever MM fixes how it goes through nodes and values
            foreach (string s in template.resources)
            {
                res.AddValue("res", s);
            }
            foreach (string s in template.excludeResources)
            {
                res.AddValue("excludeRes", s);
            }

            res.AddValue("flowMode",
                         template.flowModeNeeded
                             ? FARDebugAndSettings.FlowMode_str[(int) template.flowMode]
                             : "unneeded");

            node.AddNode(res);

            return node;
        }


        public static void LoadStressTemplates()
        {
            if (loaded)
                return;
            StressTemplates.Clear();
            foreach (ConfigNode node in GameDatabase.Instance.GetConfigNodes("FARAeroStress"))
                if(node != null)
                    foreach(ConfigNode template in node.GetNodes("FARPartStressTemplate"))
                        StressTemplates.Add(CreateFARPartStressTemplate(template));

            loaded = true;
        }

        private static FARPartStressTemplate CreateFARPartStressTemplate(ConfigNode template)
        {
            FARPartStressTemplate parsedTemplate = new FARPartStressTemplate
            {
                XZmaxStress             = 500,
                YmaxStress              = 500,
                name                    = "default",
                isSpecialTemplate       = false,
                minNumResources         = 0,
                resources               = new List<string>(),
                excludeResources        = new List<string>(),
                rejectUnlistedResources = false,
                crewed                  = false,
                flowModeNeeded          = false,
                flowMode                = ResourceFlowMode.NO_FLOW
            };

            if (template.HasValue("name"))
                parsedTemplate.name = template.GetValue("name");
            if (template.HasValue("isSpecialTemplate"))
                bool.TryParse(template.GetValue("isSpecialTemplate"), out parsedTemplate.isSpecialTemplate);
            if (template.HasValue("YmaxStress"))
                double.TryParse(template.GetValue("YmaxStress"), out parsedTemplate.YmaxStress);
            if (template.HasValue("XZmaxStress"))
                double.TryParse(template.GetValue("XZmaxStress"), out parsedTemplate.XZmaxStress);
            if (template.HasValue("requiresCrew"))
                bool.TryParse(template.GetValue("requiresCrew"), out parsedTemplate.crewed);

            if (template.HasNode("Resources"))
            {
                ConfigNode resources = template.GetNode("Resources");
                if(resources.HasValue("numReq"))
                    int.TryParse(resources.GetValue("numReq"), out parsedTemplate.minNumResources);

                if (resources.HasValue("rejectUnlistedResources"))
                    bool.TryParse(resources.GetValue("rejectUnlistedResources"), out parsedTemplate.rejectUnlistedResources);

                if (resources.HasValue("flowMode"))
                {
                    parsedTemplate.flowModeNeeded = true;
                    string flowString = resources.GetValue("flowMode").ToLowerInvariant();

                    if (flowString == "all_vessel")
                        parsedTemplate.flowMode = ResourceFlowMode.ALL_VESSEL;
                    else if (flowString == "stack_priority_search")
                        parsedTemplate.flowMode = ResourceFlowMode.STACK_PRIORITY_SEARCH;
                    else if (flowString == "stage_priority_flow")
                        parsedTemplate.flowMode = ResourceFlowMode.STAGE_PRIORITY_FLOW;
                    else if (flowString == "no_flow")
                        parsedTemplate.flowMode = ResourceFlowMode.NO_FLOW;
                    else
                        parsedTemplate.flowModeNeeded = false;
                }

                PartResourceLibrary l = PartResourceLibrary.Instance;
                foreach (string resString in resources.GetValues("res"))
                {
                    if (l.resourceDefinitions.Contains(resString))
                        parsedTemplate.resources.Add(resString);
                }
                foreach (string resString in resources.GetValues("excludeRes"))
                {
                    if (l.resourceDefinitions.Contains(resString))
                        parsedTemplate.excludeResources.Add(resString);
                }
            }

            return parsedTemplate;
        }

        public static FARPartStressTemplate DetermineStressTemplate(Part p)
        {
            FARPartStressTemplate template = StressTemplates[0];

            int resCount = p.Resources.Count;
            bool crewed = p.CrewCapacity > 0;
            if ( p.Modules.Contains<ModuleAblator>()|| p.Resources.Contains("Ablator"))
                return template;

            foreach (FARPartStressTemplate candidate in StressTemplates)
            {
                if (candidate.isSpecialTemplate)
                    continue;
                if (candidate.crewed != crewed)
                    continue;

                if (resCount < candidate.minNumResources)
                    continue;

                if (candidate.rejectUnlistedResources)
                {
                    bool cont = true;
                    int numRes = 0;
                    foreach (PartResource res in p.Resources)
                    {
                        if (candidate.resources.Contains(res.info.name))
                        {
                            numRes++;
                            cont = false;
                        }
                        else
                        {
                            cont = true;
                            break;
                        }
                    }

                    if (cont || numRes < candidate.minNumResources)
                        continue;
                }
                else
                {
                    int numRes = 0;
                    foreach (PartResource res in p.Resources)
                        if (!candidate.excludeResources.Contains(res.info.name))
                            if(!candidate.flowModeNeeded || res.info.resourceFlowMode == candidate.flowMode)
                                numRes++;



                    if (numRes < candidate.minNumResources)
                        continue;
                }

                template = candidate;
            }


            return template;
        }
    }

    public struct FARPartStressTemplate
    {
        public string name;
        public bool isSpecialTemplate;
        public double YmaxStress;
        public double XZmaxStress;
        public List<string> resources;
        public List<string> excludeResources;
        public ResourceFlowMode flowMode;
        public bool flowModeNeeded;
        public int minNumResources;
        public bool rejectUnlistedResources;
        public bool crewed;
    }
}
