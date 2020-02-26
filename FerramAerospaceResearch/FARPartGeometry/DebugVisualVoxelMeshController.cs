/*
Ferram Aerospace Research v0.15.9.7 "Lumley"
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
using UnityEngine;
using FerramAerospaceResearch.FARThreading;
using FerramAerospaceResearch.FARUtils;

namespace FerramAerospaceResearch.FARPartGeometry
{
    public class DebugVisualVoxelMeshController : IDisposable
    {
        // limit of verts in each mesh imposed by Unity
        public const int MAX_VERTS_PER_SUBMESH = 65000;
        public const int MAX_VOXELS_PER_SUBMESH = MAX_VERTS_PER_SUBMESH / 4;
        private List<DebugVisualVoxel> m_debugVoxels = new List<DebugVisualVoxel>();
        private List<DebugVisualVoxelSubmesh> m_submeshes = new List<DebugVisualVoxelSubmesh>();
        private bool m_active = false;

        private Transform m_parent;

        public DebugVisualVoxelMeshController(Transform parent = null)
        {
            m_parent = parent;
        }

        public bool active
        {
            get { return m_active; }
            set
            {
                if (m_active != value)
                {
                    m_active = value;
                    QueueMainThreadTask(UpdateActive);
                }
            }
        }

        internal List<DebugVisualVoxel> DebugVoxels
        {
            get { return m_debugVoxels; }
        }
        public Transform Parent
        {
            get { return m_parent; }
            set { m_parent = value; }
        }

        private void UpdateActive()
        {
            foreach (DebugVisualVoxelSubmesh submesh in m_submeshes)
            {
                submesh.active = m_active;
            }
        }

        public void Rebuild()
        {
            FARLogger.Info("Rebuilding visual voxel mesh...");
            Clear(false);
            int submeshes = m_debugVoxels.Count / MAX_VOXELS_PER_SUBMESH + 1;
            FARLogger.Info("Voxel mesh contains " + m_debugVoxels.Count + " voxels in " + submeshes + " submeshes");
            SetupSubmeshes(submeshes);
            int indexOffset = 0;
            for (int i = 0; i < submeshes; i++)
            {
                for (int j = i * MAX_VOXELS_PER_SUBMESH; j < Math.Min((i + 1) * MAX_VOXELS_PER_SUBMESH, m_debugVoxels.Count); j++)
                {
                    m_debugVoxels[j].AddToMesh(m_submeshes[i].Vertices, m_submeshes[i].Uvs, m_submeshes[i].Triangles, indexOffset);
                }
                m_submeshes[i].Rebuild();
            }
            FARLogger.Info("Finished rebuilding visual voxel mesh.");
        }

        public void RebuildSafe()
        {
            QueueMainThreadTask(Rebuild);
        }

        public void Clear(bool clearVoxels = false)
        {
            foreach (DebugVisualVoxelSubmesh submesh in m_submeshes)
            {
                submesh.Clear();
            }
            if (clearVoxels)
            {
                m_debugVoxels.Clear();
            }
        }

        public void ClearSafe(bool clearVoxels = false)
        {
            QueueMainThreadTask(() => Clear(clearVoxels));
        }

        private void SetupSubmeshes(int submeshes)
        {
            for (int i = m_submeshes.Count; i < submeshes; i++)
            {
                m_submeshes.Add(DebugVisualVoxelSubmesh.Create(m_parent, m_active));
            }
        }

        public void Dispose()
        {
            QueueMainThreadTask(DisposeSafe);
        }

        private void DisposeSafe()
        {
            foreach (DebugVisualVoxelSubmesh submesh in m_submeshes)
            {
                MonoBehaviour.Destroy(submesh);
            }
        }

        private void QueueMainThreadTask(Action action)
        {
            if (VoxelizationThreadpool.Instance.inMainThread)
            {
                FARLogger.Debug("In main thread, not queueing " + action.Method.Name);
                action();
            }
            else
            {
                ThreadSafeDebugLogger.Instance.RegisterDebugMessage("Running " + action.Method.Name + " in main thread");
                VoxelizationThreadpool.Instance.RunOnMainThread(action);
            }
        }

    }
}
