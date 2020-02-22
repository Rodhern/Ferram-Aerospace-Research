using System;
using System.Collections.Generic;
using UnityEngine;
using KSPAssets;
using KSPAssets.Loaders;

namespace FerramAerospaceResearch
{
    [KSPAddon(KSPAddon.Startup.MainMenu, true)]
    class FARAssets : MonoBehaviour
    {

        public static Dictionary<string, Material> materialDict;

        void Start()
        {
            materialDict = new Dictionary<string, Material>();

            /* Debug.Log("[FAR] Asset bundles");
            Debug.Log("[FAR] " + AssetLoader.BundleDefinitions.Count);
            foreach (BundleDefinition b in AssetLoader.BundleDefinitions)
            {
                Debug.Log("[FAR] " + b.name + " " + b.createdTime + " " + b.path + " " + b.info + " " + b.urlName);
            }
            Debug.Log("[FAR] " + AssetLoader.AssetDefinitions.Count);
            foreach (AssetDefinition a in AssetLoader.AssetDefinitions)
            {
                Debug.Log("[FAR] " + a.name + " " + a.type + " " + a.path);
            }*/

            // AssetLoader.LoadAssets(LoadAssets,
            // AssetLoader.GetAssetDefinitionWithName("FerramAerospaceResearch/Shaders/farshaders",
            // "FARCrossSectionGraph"));
            AssetLoader.LoadAssets(LoadAssets, AssetLoader.GetAssetDefinitionWithName("FerramAerospaceResearch/Assets/farassets", "FARGraphMaterial"));
        }

        void LoadAssets(AssetLoader.Loader loader)
        {
            for (int i = 0; i < loader.definitions.Length; i++ )
            {
                UnityEngine.Object o = loader.objects[i];
                if (o == null)
                    continue;

                Type oType = o.GetType();
                if (oType == typeof(Material))
                {
                    Debug.Log("[FAR] Adding material " + loader.definitions[i].name + " to dictionary");
                    materialDict.Add(loader.definitions[i].name, o as Material);
                }
            }
        }
    }
}
