using System;
using System.Collections.Generic;
using UnityEngine;
using KSPAssets;
using KSPAssets.Loaders;
using FerramAerospaceResearch.FARUtils;

namespace FerramAerospaceResearch
{
    [KSPAddon(KSPAddon.Startup.MainMenu, true)]
    class FARAssets : MonoBehaviour
    {

        public static Dictionary<string, Material> materialDict;
        public static Dictionary<string, Shader> shaderDict;

        void Start()
        {
            materialDict = new Dictionary<string, Material>();
            shaderDict = new Dictionary<string, Shader>();

            /* FARLogger.Info("Asset bundles");
            FARLogger.Info("" + AssetLoader.BundleDefinitions.Count);
            foreach (BundleDefinition b in AssetLoader.BundleDefinitions)
            {
                FARLogger.Info("" + b.name + " " + b.createdTime + " " + b.path + " " + b.info + " " + b.urlName);
            }
            FARLogger.Info("" + AssetLoader.AssetDefinitions.Count);
            foreach (AssetDefinition a in AssetLoader.AssetDefinitions)
            {
                FARLogger.Info("" + a.name + " " + a.type + " " + a.path);
            }*/

            // AssetLoader.LoadAssets(LoadAssets,
            // AssetLoader.GetAssetDefinitionWithName("FerramAerospaceResearch/Shaders/farshaders",
            // "FARCrossSectionGraph"));
            AssetLoader.LoadAssets(LoadAssets, AssetLoader.GetAssetDefinitionsWithType("FerramAerospaceResearch/Assets/farassets", typeof(Material)));
            AssetLoader.LoadAssets(LoadAssets, AssetLoader.GetAssetDefinitionsWithType("FerramAerospaceResearch/Assets/farassets", typeof(Shader)));
            FARLogger.Info("Finished loading FAR Assets.");
        }

        void LoadAssets(AssetLoader.Loader loader)
        {
            for (int i = 0; i < loader.definitions.Length; i++)
            {
                UnityEngine.Object o = loader.objects[i];
                if (o == null)
                    continue;

                Type oType = o.GetType();
                if (oType == typeof(Material))
                {
                    FARLogger.Info("Adding material " + loader.definitions[i].name + " to dictionary");
                    materialDict.Add(loader.definitions[i].name, o as Material);
                }
                else if (oType == typeof(Shader))
                {
                    FARLogger.Info("Adding shader " + loader.definitions[i].name + " to dictionary");
                    shaderDict.Add(loader.definitions[i].name, o as Shader);
                }
            }
        }
    }
}
