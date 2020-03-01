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

using System.Collections.Generic;
using UnityEngine;
using FerramAerospaceResearch.FARUtils;

namespace FerramAerospaceResearch.FARPartGeometry
{
    [RequireComponent(typeof(MeshFilter)), RequireComponent(typeof(MeshRenderer))]
    public class DebugVisualVoxelSubmesh : MonoBehaviour
    {
        private Mesh m_mesh;
        private MeshFilter m_meshFilter;
        private MeshRenderer m_meshRenderer;

        private List<Vector3> m_vertices = new List<Vector3>();
        private List<Vector2> m_uvs = new List<Vector2>();
        private List<int> m_triangles = new List<int>();

        public static DebugVisualVoxelSubmesh Create(Transform parent = null, bool active = true)
        {
            GameObject go = new GameObject();
            if (parent != null)
                go.transform.SetParent(parent);
            DebugVisualVoxelSubmesh component = go.AddComponent<DebugVisualVoxelSubmesh>();
            go.SetActive(active);
            return component;
        }

        public bool active
        {
            get { return gameObject.activeSelf; }
            set { gameObject.SetActive(value); }
        }

        public Mesh Mesh
        {
            get { return m_mesh; }
        }
        public MeshRenderer MeshRenderer
        {
            get { return m_meshRenderer; }
        }
        public List<Vector3> Vertices
        {
            get { return m_vertices; }
        }
        public List<Vector2> Uvs
        {
            get { return m_uvs; }
        }
        public List<int> Triangles
        {
            get { return m_triangles; }
        }

        protected virtual void Awake()
        {
            FARLogger.Debug("Setting up debug voxel submesh");
            m_mesh = new Mesh();
            m_meshFilter = GetComponent<MeshFilter>();
            m_meshRenderer = GetComponent<MeshRenderer>();

            m_meshFilter.mesh = m_mesh;
            m_meshRenderer.material = FARAssets.ShaderCache.DebugVoxels.Material;
            m_meshRenderer.material.mainTexture = FARAssets.TextureCache.VoxelTexture;
            setupMeshRenderer();
        }

        private void setupMeshRenderer()
        {
            m_meshRenderer.lightProbeUsage = UnityEngine.Rendering.LightProbeUsage.Off;
            m_meshRenderer.reflectionProbeUsage = UnityEngine.Rendering.ReflectionProbeUsage.Off;
            m_meshRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            m_meshRenderer.receiveShadows = false;
            // m_meshRenderer.motionVectorGenerationMode = MotionVectorGenerationMode.ForceNoMotion; // Rodhern: Downgrade to KSP 1.3.1 (Unity 5.4)
        }

        public void Rebuild()
        {
            m_mesh.SetVertices(m_vertices);
            m_mesh.SetUVs(0, m_uvs);
            m_mesh.SetTriangles(m_triangles, 0);
        }

        public void Clear()
        {
            m_mesh.Clear();
            m_uvs.Clear();
            m_vertices.Clear();
            m_triangles.Clear();
        }

        protected virtual void OnDestroy()
        {
            Clear();

            // have to clean up renderer material
            Destroy(m_meshRenderer.material);
        }
    }
}
