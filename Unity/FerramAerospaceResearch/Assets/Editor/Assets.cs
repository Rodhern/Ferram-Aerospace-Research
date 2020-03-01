using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Editor
{
    public static class Assets
    {
        // name of the asset bundle
        public const string AssetBundle = "farassets";

        [MenuItem("FAR/Print Assets")]
        public static void PrintAssets()
        {
            foreach (var asset in GetAllAssets())
                Debug.Log(asset);
        }

        public static List<Asset> GetAllAssets()
        {
            var paths  = AssetDatabase.GetAssetPathsFromAssetBundle(AssetBundle);
            var assets = new List<Asset>(paths.Length);
            // ReSharper disable once LoopCanBeConvertedToQuery
            foreach (string path in paths)
                assets.Add(new Asset
                {
                    path  = path,
                    asset = AssetDatabase.LoadMainAssetAtPath(path)
                });
            return assets;
        }

        // ReSharper disable once UnusedMember.Global
        public static List<Asset> GetSelectedAssets()
        {
            var guids  = Selection.assetGUIDs;
            var assets = new List<Asset>(guids.Length);
            // ReSharper disable once LoopCanBeConvertedToQuery
            foreach (string guid in guids)
            {
                string path = AssetDatabase.GUIDToAssetPath(guid);
                assets.Add(new Asset
                {
                    path  = path,
                    asset = AssetDatabase.LoadMainAssetAtPath(path)
                });
            }

            return assets;
        }

        public struct Asset
        {
            public string path;
            public Object asset;

            public override string ToString()
            {
                return $"{asset} ({path})";
            }
        }

        public struct AssetDefinition
        {
            public readonly string path;
            public readonly string name;

            public AssetDefinition(Asset asset)
            {
                path = asset.path;
                name = asset.asset.name;
            }
        }
    }
}
