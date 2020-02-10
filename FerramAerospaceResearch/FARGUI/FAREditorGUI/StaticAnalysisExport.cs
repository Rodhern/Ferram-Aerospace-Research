using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace FerramAerospaceResearch.FARGUI.FAREditorGUI
{
    class StaticAnalysisExportFile
    {
        private System.Globalization.CultureInfo enus;
        private float[] machlist;
        private float[] aoadeglist;
        private List<double[]> datarows;

        private const string floatstyleculture = "en-US";
        private const string floatstyleformat = "E8";

        public StaticAnalysisExportFile()
        {
            enus = System.Globalization.CultureInfo.CreateSpecificCulture(floatstyleculture);
            LoadConfigLists(out machlist, out aoadeglist, enus);
            datarows = new List<Double[]>();
        }

        static public string ConfigFilePath
        {
            get
            {
                string path = KSPUtil.ApplicationRootPath;
                path += "GameData/FerramAerospaceResearch/Plugins/PluginData/FerramAerospaceResearch/";
                path += "saexpcfg.txt";
                return path;
            }
        }

        static public string TextFilePath
        {
            get
            {
                string path = KSPUtil.ApplicationRootPath;
                path += "GameData/FerramAerospaceResearch/Plugins/PluginData/";
                path += "StaticAnalysisSweepDataExport.txt";
                return path;
            }
        }

        public List<float> MachNumberList
        { get { return new List<float>(machlist); } }

        public List<float> AoADegreeList
        { get { return new List<float>(aoadeglist); } }

        public int DataCount
        { get { return datarows.Count; } }

        public void AddDatapoint(double mach, double aoadeg, double Cl, double Cd, double Cm)
        {
            datarows.Add(new double[] { mach, aoadeg, Cl, Cd, Cm });
        }

        static public bool ReadWhiteSpace(string[] lines, int lineoffset, out int blanklinecount)
        {
            blanklinecount = 0;
            while (lines.Length > lineoffset + blanklinecount)
            {
                if (lines[lineoffset + blanklinecount].Trim() == "")
                    blanklinecount++;
                else
                    break;
            }
            return blanklinecount > 0;
        }

        static public bool ReadMatrix(string[] lines, int lineoffset, out string name, out float[,] matrix, System.Globalization.CultureInfo enus)
        {
            name = null; matrix = null;
            if (lineoffset < 0)
                return false;
            if (lines.Length < lineoffset + 4)
                return false;

            bool b0 = lines[lineoffset + 0].StartsWith("# name: ");
            bool b1 = lines[lineoffset + 1] == "# type: matrix";
            bool b2 = lines[lineoffset + 2].StartsWith("# rows: ");
            bool b3 = lines[lineoffset + 3].StartsWith("# columns: ");
            if (!(b0 && b1 && b2 && b3))
                return false;

            name = lines[lineoffset + 0].Remove(0, 8).Trim();
            int rows = int.Parse(lines[lineoffset + 2].Remove(0, 8));
            int cols = int.Parse(lines[lineoffset + 3].Remove(0, 11));
            if (name.Length == 0 || rows < 0 || cols < 0
                || (rows > 0 && cols == 0) || (rows == 0 && cols > 0)
                || lines.Length < lineoffset + 4 + rows)
                return false;

            var floatstyle = System.Globalization.NumberStyles.Float;
            List<float[]> jagged = new List<float[]>(rows);

            for (int i = lineoffset + 4; i < lineoffset + rows + 4; i++)
            {
                string[] rowvals = lines[i].Trim().Split(new char[] { ' ' });
                if (rowvals.Length == cols)
                {
                    float[] row = new float[cols];
                    for (int j = 0; j < cols; j++)
                    {
                        float value;
                        if (!float.TryParse(rowvals[j], floatstyle, enus, out value))
                            return false;
                        row[j] = value;
                    }
                    jagged.Add(row);
                }
                else return false;
            }

            matrix = new float[rows, cols];
            for (int i = 0; i < rows; i++)
                for (int j = 0; j < cols; j++)
                    matrix[i,j] = jagged[i][j];
            return true;
        }

        private static float[] CopyToArray(float[,] matrix)
        {
            int rows = matrix.GetLength(0);
            int cols = matrix.GetLength(1);
            float[] result = new float[rows * cols];
            for (int i = 0; i < rows; i++)
                for (int j = 0; j < cols; j++)
                    result[i + j * rows] = matrix[i, j];
            return result;
        }

        static public void LoadConfigLists(out float[] machlist, out float[] aoadeglist, System.Globalization.CultureInfo enus)
        {
            machlist = new float[0];
            aoadeglist = new float[0];

            string path = ConfigFilePath;
            if (!File.Exists(path))
                return;

            string[] lines = File.ReadAllLines(path, System.Text.Encoding.Default);
            if (lines.Length < 2 || !lines[0].StartsWith("# Created by"))
                return;

            int i = 1;
            while (i < lines.Length)
            {
                int blanklines;
                if (ReadWhiteSpace(lines, i, out blanklines))
                    i = i + blanklines;

                string mname; float[,] mvalues;
                if (ReadMatrix(lines, i, out mname, out mvalues, enus))
                {
                    i = i + 4 + mvalues.GetLength(0);
                    switch (mname)
                    {
                        case "mach":
                            machlist = CopyToArray(mvalues);
                            break;
                        case "aoa":
                            aoadeglist = CopyToArray(mvalues);
                            break;
                        default:
                            Debug.Log("[Rodhern][FAR] Matrix named '" + mname + "' (" + mvalues.GetLength(0) + " x " + mvalues.GetLength(1) + ") ignored;"
                                    + " only variables named 'mach' and 'aoa' are considered when loading static analysis export configuration.");
                            break;
                    }
                }
                else break;
            }
        }

        private static string Header()
        {
            return "mach; aoa; Cl; Cd; Cm";
        }

        private static string CSVRow(double[] values, System.Globalization.CultureInfo enus, string format)
        {
            var sb = new System.Text.StringBuilder();
            for (int i = 0; i < values.Length; i++)
            {
                sb.Append(values[i].ToString(format, enus));
                if (i + 1 < values.Length)
                    sb.Append(";  ");
            }
            return sb.ToString();
        }

        private string[] GetExportOutputLines()
        {
            string[] lines = new string[1 + datarows.Count];
            lines[0] = Header();
            for (int i = 0; i < datarows.Count; i++)
                lines[1 + i] = CSVRow(datarows[i], enus, floatstyleformat);
            return lines;
        }

        public void Export()
        {
            string path = TextFilePath;
            string[] lines = GetExportOutputLines();
            File.WriteAllLines(path, lines);
        }
    }
}
