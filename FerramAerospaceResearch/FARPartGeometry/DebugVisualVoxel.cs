/*
Ferram Aerospace Research v0.15.9.7 "Lumley"
=========================
Aerodynamics model for Kerbal Space Program

Copyright 2017, Michael Ferrara, aka Ferram4

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

using System;
using System.Collections.Generic;
using UnityEngine;

namespace FerramAerospaceResearch.FARPartGeometry
{
    class DebugVisualVoxel
    {
        public static float globalScale = 0.9f;
        float m_scale;
        float m_extent;
        Vector3 m_position;

        public DebugVisualVoxel(Vector3 pos, double elementScale)
        {
            Scale = (float) elementScale;
            m_position = pos;
        }

        public float Scale
        {
            get { return m_scale; }
            set
            {
                m_scale = value;
                m_extent = value * globalScale;
            }
        }
        public float Extent
        {
            get { return m_extent; }
            set { Scale = value / globalScale; }
        }
        public Vector3 Position
        {
            get { return m_position; }
            set { m_position = value; }
        }
        public Vector3 BottomLeft
        {
            get { return new Vector3(Position.x - Extent, Position.y - Extent, Position.z); }
        }
        public Vector3 BottomRight
        {
            get { return new Vector3(Position.x + Extent, Position.y - Extent, Position.z); }
        }
        public Vector3 TopRight
        {
            get { return new Vector3(Position.x + Extent, Position.y + Extent, Position.z); }
        }
        public Vector3 TopLeft
        {
            get { return new Vector3(Position.x - Extent, Position.y + Extent, Position.z); }
        }

        public void AddToMesh(List<Vector3> vertices, List<Vector2> uvs, List<int> triangles, int indexOffset = 0)
        {
            // using offset for when KSP updates to Unity 2017.4+ -> 1 mesh can contain more than 65k vertices
            int counter = vertices.Count - indexOffset;

            // update extent in case globalScale changed
            m_extent = m_scale * globalScale;

            vertices.Add(BottomLeft);
            uvs.Add(new Vector2(0, 0));

            vertices.Add(TopLeft);
            uvs.Add(new Vector2(0, 1));

            vertices.Add(TopRight);
            uvs.Add(new Vector2(1, 1));

            vertices.Add(BottomRight);
            uvs.Add(new Vector2(1, 0));

            // left-handed triangles
            triangles.Add(counter);
            triangles.Add(counter + 1);
            triangles.Add(counter + 2);

            triangles.Add(counter);
            triangles.Add(counter + 2);
            triangles.Add(counter + 3);
        }

    }
}
