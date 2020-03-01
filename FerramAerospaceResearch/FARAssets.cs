using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using UnityEngine;
using FerramAerospaceResearch.FARUtils;
using Object = UnityEngine.Object;

namespace FerramAerospaceResearch
{
    [KSPAddon(KSPAddon.Startup.MainMenu, true)]
    class FARAssets : MonoBehaviour
    {
        private static readonly string assetBundleRootPath = Path.Combine(Assembly.GetExecutingAssembly().Location, "../../Assets");
        private const string AssetBundleExtension = ".far";

        public class FARAssetDictionary<T> : Dictionary<string, T> where T : Object
        {
            private AssetBundle assetBundle;
            private string bundleName;

            public string BundleName
            {
                get { return bundleName; }
                private set
                {
                    bundleName = value;
                    SetBundlePath(value);
                }
            }

            public string BundlePath { get; private set; }
            public bool AssetsLoaded { get; private set; }

            public FARAssetDictionary(string bundleName)
            {
                BundleName = bundleName;
            }

            public IEnumerator LoadAsync()
            {
                FARLogger.Debug(string.Format("Loading asset bundle {0}", BundlePath));
                var createRequest = AssetBundle.LoadFromFileAsync(BundlePath);
                yield return createRequest;

                assetBundle = createRequest.assetBundle;
                if (assetBundle == null)
                {
                    FARLogger.Error(string.Format("Could not load asset bundle from {0}", BundlePath));
                    yield break;
                }

                var loadRequest = assetBundle.LoadAllAssetsAsync(typeof(T));
                yield return loadRequest;

                foreach (var asset in loadRequest.allAssets)
                {
                    FARLogger.Debug(string.Format("Adding {0} to dictionary", asset));
                    Add(asset.name, (T) asset);
                }

                FARLogger.Debug(string.Format("Finished loading {0} assets from {1}", typeof(T), BundlePath));
                AssetsLoaded = true;

                OnLoad();
            }

            protected virtual void OnLoad()
            {
            }

            protected virtual void SetBundlePath(string name)
            {
                BundlePath = Path.GetFullPath(Path.Combine(assetBundleRootPath, name) + AssetBundleExtension);
            }
        }

        public class FARShaderCache : FARAssetDictionary<Shader>
        {
            public class ShaderMaterialPair
            {
                private Shader _shader;
                private Material _material;
                public Shader Shader { get { return _shader; } }
                public Material Material { get { return _material; } }

                public ShaderMaterialPair(Shader shader) : this(shader, new Material(shader))
                {
                }

                public ShaderMaterialPair(Shader shader, Material material)
                {
                    _shader = shader;
                    _material = material;
                }
            }

            public ShaderMaterialPair LineRenderer { get; private set; }
            public ShaderMaterialPair DebugVoxels { get; private set; }

            public FARShaderCache(string bundleName) : base(bundleName)
            {
            }

            protected override void OnLoad()
            {
                LineRenderer = new ShaderMaterialPair(Shader.Find("Hidden/Internal-Colored"));
                Shader voxelShader;
                if (TryGetValue("FerramAerospaceResearch/Debug Voxel Mesh", out voxelShader))
                {
                    DebugVoxels = new ShaderMaterialPair(voxelShader);
                    DebugVoxels.Material.SetFloat("_Cutoff", 0.45f);
                }
                else
                {
                    FARLogger.Warning("Could not find voxel mesh shader. Using Sprites/Default for rendering, you WILL see depth artifacts");
                    DebugVoxels = new ShaderMaterialPair(Shader.Find("Sprites/Default"));
                }
            }

            protected override void SetBundlePath(string name)
            {
                // ReSharper disable once SwitchStatementMissingSomeCases
                switch (Application.platform)
                {
                    case RuntimePlatform.LinuxPlayer:
                        FARLogger.Info("Loading shaders from Linux bundle");
                        name += "_linux";
                        break;
                    case RuntimePlatform.WindowsPlayer:
                        if (SystemInfo.graphicsDeviceVersion.StartsWith("OpenGL", StringComparison.Ordinal))
                        {
                            FARLogger.Info("Loading shaders from Linux/OpenGL bundle");
                            name += "_linux"; //For OpenGL users on Windows we load the Linux shaders to fix OpenGL issues
                        }
                        else
                        {
                            FARLogger.Info("Loading shaders from Windows bundle");
                            name += "_windows";
                        }
                        break;
                    case RuntimePlatform.OSXPlayer:
                        FARLogger.Info("Loading shaders from MacOSX bundle");
                        name += "_macosx";
                        break;
                    default:
                        // Should never reach this
                        FARLogger.Error(string.Format("Invalid runtime platform {0}", Application.platform));
                        break;
                }

                base.SetBundlePath(name);
            }
        }

        public class FARTextureCache : Dictionary<string, Texture2D>
        {
            public Texture2D IconLarge { get; private set; }
            public Texture2D IconSmall { get; private set; }
            public Texture2D VoxelTexture { get; private set; }

            public void Initialize()
            {
                Add("icon_button_stock", GameDatabase.Instance.GetTexture("FerramAerospaceResearch/Textures/icon_button_stock", false));
                Add("icon_button_blizzy", GameDatabase.Instance.GetTexture("FerramAerospaceResearch/Textures/icon_button_blizzy", false));
                Add("sprite_debug_voxel", GameDatabase.Instance.GetTexture("FerramAerospaceResearch/Textures/sprite_debug_voxel", false));
                IconLarge = this["icon_button_stock"];
                IconSmall = this["icon_button_blizzy"];
                VoxelTexture = this["sprite_debug_voxel"];
            }
        }

        public static FARShaderCache ShaderCache { get; private set; }
        public static FARTextureCache TextureCache { get; private set; }

        void Start()
        {
            ShaderCache = new FARShaderCache("farshaders");
            TextureCache = new FARTextureCache();
            TextureCache.Initialize();
            StartCoroutine(LoadAssetsAsync());
        }

        private static IEnumerator LoadAssetsAsync()
        {
            // using a separate method to chain asset loading in the future
            yield return ShaderCache.LoadAsync();
        }
    }
}
