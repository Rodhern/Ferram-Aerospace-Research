using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;
using Object = UnityEngine.Object;

namespace Editor
{
    public static class Bundler
    {
        private const string Dir              = "AssetBundles";
        private const string Extension        = ".far";
        private const string PrefabBundle     = "farprefabs";
        private const string ShaderBundle     = "farshaders";
        private const string ScriptableBundle = "farassets";

        private static readonly ShaderTarget[] platforms =
        {
            new ShaderTarget
            {
                suffix   = "_windows",
                platform = BuildTarget.StandaloneWindows64
            },
            new ShaderTarget
            {
                suffix   = "_linux",
                platform = BuildTarget.StandaloneLinux64
            },
            new ShaderTarget
            {
                suffix   = "_macosx",
                platform = BuildTarget.StandaloneOSXUniversal
            }
        };

        [MenuItem("FAR/Build Bundles")]
        public static void BuildAllAssetBundles()
        {
            var builds = BuildMap();

            BuildBundle(builds[BundleType.Assets]);
            BuildShaders(builds[BundleType.Shaders]);
            BuildBundle(builds[BundleType.Prefabs]);
        }

        [MenuItem("FAR/Build Shaders")]
        public static void BuildAllShaders()
        {
            BuildShaders(BuildMap()[BundleType.Shaders]);
        }

        [MenuItem("FAR/Build Prefabs")]
        public static void BuildPrefabs()
        {
            BuildBundle(BuildMap()[BundleType.Prefabs]);
        }

        [MenuItem("FAR/Build Assets")]
        public static void BuildAssets()
        {
            BuildBundle(BuildMap()[BundleType.Assets]);
        }

        private static void BuildShaders(AssetBundleBuild build)
        {
            string name = build.assetBundleName;
            foreach (var target in platforms)
            {
                build.assetBundleName = name + target.suffix;
                BuildBundle(build, target.platform);
            }
        }

        private static void BuildBundle(AssetBundleBuild build, BuildTarget target = BuildTarget.StandaloneWindows64)
        {
            string name = build.assetBundleName;
            BuildPipeline.BuildAssetBundles(Dir,
                                            new[] {build},
                                            BuildAssetBundleOptions.ForceRebuildAssetBundle,
                                            target);
            FileUtil.ReplaceFile($"{Dir}/{name}", $"{Dir}/{name}{Extension}");
            FileUtil.DeleteFileOrDirectory($"{Dir}/{name}");

            FileUtil.DeleteFileOrDirectory($"{Dir}/AssetBundles");
            FileUtil.DeleteFileOrDirectory($"{Dir}/AssetBundles.manifest");
        }

        private static Dictionary<BundleType, AssetBundleBuild> BuildMap()
        {
            var assets     = Assets.GetAllAssets();
            var materials  = new List<Assets.AssetDefinition>();
            var shaders    = new List<Assets.AssetDefinition>();
            var prefabs    = new List<Assets.AssetDefinition>();
            var scriptable = new List<Assets.AssetDefinition>();

            foreach (var asset in assets)
            {
                var type = asset.asset.GetType();
                if (type.IsAssignableFrom(typeof(Shader)))
                    shaders.Add(new Assets.AssetDefinition(asset));
                else if (type.IsAssignableFrom(typeof(Material)))
                    materials.Add(new Assets.AssetDefinition(asset));
                else if (type.IsAssignableFrom(typeof(GameObject)))
                    prefabs.Add(new Assets.AssetDefinition(asset));
                else
                    scriptable.Add(new Assets.AssetDefinition(asset));
            }

            shaders.AddRange(materials);

            return new Dictionary<BundleType, AssetBundleBuild>
            {
                {
                    BundleType.Prefabs, new AssetBundleBuild
                    {
                        assetBundleName  = PrefabBundle,
                        assetNames       = prefabs.Select(def => def.path).ToArray(),
                        addressableNames = prefabs.Select(def => def.name).ToArray()
                    }
                },
                {
                    BundleType.Assets, new AssetBundleBuild
                    {
                        assetBundleName  = ScriptableBundle,
                        assetNames       = scriptable.Select(def => def.path).ToArray(),
                        addressableNames = scriptable.Select(def => def.name).ToArray()
                    }
                },
                {
                    BundleType.Shaders, new AssetBundleBuild
                    {
                        assetBundleName  = ShaderBundle,
                        assetNames       = shaders.Select(def => def.path).ToArray(),
                        addressableNames = shaders.Select(def => def.name).ToArray()
                    }
                }
            };
        }

        [MenuItem("FAR/Resolve Dependencies")]
        public static void UpdateDependencies()
        {
            Debug.Log($"Updating dependencies for {Assets.AssetBundle}");

            var objects = new List<Object>();
            var paths   = new HashSet<string>();
            foreach (var asset in Assets.GetAllAssets())
            {
                string assetPath = asset.path;
                string fullPath  = new FileInfo(assetPath).FullName;
                objects.Add(AssetDatabase.LoadMainAssetAtPath(assetPath));
                paths.Add(fullPath);
            }

            var dependencies = EditorUtility.CollectDependencies(objects.ToArray());

            foreach (var obj in dependencies)
            {
                string assetPath = AssetDatabase.GetAssetPath(obj);
                if (string.IsNullOrEmpty(assetPath))
                    continue;
                string fullPath = new FileInfo(assetPath).FullName;
                if (paths.Contains(fullPath) || !assetPath.StartsWith("Assets", StringComparison.Ordinal))
                    continue;
                string validType = GetValidType(ref assetPath, obj);
                if (string.IsNullOrEmpty(validType) || validType == "dll")
                    continue;
                paths.Add(fullPath);
                AssetImporter.GetAtPath(assetPath).SetAssetBundleNameAndVariant(Assets.AssetBundle, "");
                Debug.Log($"Adding asset at {assetPath} to {Assets.AssetBundle}");
            }

            AssetDatabase.Refresh();
        }

        private static string GetValidType(ref string assetPath, Object asset)
        {
            if (assetPath.EndsWith(".cs", StringComparison.Ordinal))
                return null;
            if (assetPath.EndsWith(".dll", StringComparison.Ordinal) ||
                assetPath.EndsWith(".DLL", StringComparison.Ordinal))
                return "dll";
            if (assetPath.EndsWith(".cfg.txt", StringComparison.Ordinal))
                return "cfg";

            if (!assetPath.EndsWith(".cfg", StringComparison.Ordinal))
                return asset.GetType().Name;
            Debug.Log("Renaming cfg asset from '" + assetPath + "' to '" + assetPath + ".txt'");
            string text = assetPath + ".txt";
            File.Move(assetPath, text);
            assetPath = text;
            return "cfg";
        }

        private enum BundleType
        {
            Prefabs,
            Shaders,
            Assets
        }

        private struct ShaderTarget
        {
            public string      suffix;
            public BuildTarget platform;
        }
    }
}
