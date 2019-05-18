/* Name:    FerramGraph (Graph GUI Plugin)
 * Version: 1.3   (KSP 0.22+)
Copyright 2019, Daumantas Kavolis, aka dkavolis

    This file is part of FerramGraph.

    FerramGraph is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FerramGraph is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FerramGraph.  If not, see <http://www.gnu.org/licenses/>. *

 * Disclaimer: You use this at your own risk; this is an alpha plugin for an alpha game; if your computer disintegrates, it's not my fault. :P
 *
 *
 */

using System;
using UnityEngine;

namespace ferram4
{
    public static class ferramDrawingExtensions
    {
        public static Color EmptyColor = new Color(0, 0, 0, 0);

        public static double[] replaceNaNs(this double[] array, double value = 0.0, string name = "double[]")
        {
            int elements = array.Length;
            double[] other = new double[elements];
            for (int i = 0; i < elements; i++)
            {
                if (double.IsNaN(array[i]))
                {
                    other[i] = value;
                    MonoBehaviour.print("[FAR] [ferramGraph] Warning: NaN in " + name + " array; value set to " + value);
                }
                else
                    other[i] = array[i];
            }
            return other;
        }

        public static double[] toPixelsF(this double[] raw, double zeroValue, double scale, double scaling = 1.0)
        {
            int elements = raw.Length;
            double[] pixels = new double[elements];
            for (int i = 0; i < elements; i++)
            {
                double tmp = raw[i] * scale;
                tmp -= zeroValue;
                tmp *= scaling;

                pixels[i] = tmp;
            }

            return pixels;
        }

        public static void Clear(this Texture2D tex, Color color)
        {
            for (int i = 0; i < tex.width; i++)
                for (int j = 0; j < tex.height; j++)
                    tex.SetPixel(i, j, color);
        }

        public static bool inBounds(this Texture2D tex, int i, int j)
        {
            return (i >= 0 && i < tex.width) && (j >= 0 && j < tex.height);
        }

        // ReSharper disable once UnusedMember.Global -> for reference
        /// <summary>
        /// Bresenham line drawing algorithm (quick)
        /// </summary>
        public static void DrawLineBresenham(this Texture2D tex, int x0, int y0, int x1, int y1, Color color)
        {
            bool steep = (y1 - y0) > (x1 - x0);

            if (steep)
            {
                Swap(ref x0, ref y0);
                Swap(ref x1, ref y1);
            }
            else if (x0 > x1)
            {
                Swap(ref x0, ref x1);
                Swap(ref y0, ref y1);
            }

            int dx = x1 - x0;
            int dy = Math.Abs(y1 - y0);

            int error = dx / 2;
            int ystep;
            int y = y0;

            if (y0 < y1)
                ystep = 1;
            else
                ystep = -1;

            for (int x = x0; x < x1; x++)
            {
                if (steep)
                {
                    if (tex.inBounds(y, x)) tex.SetPixel(y, x, color);
                }
                else
                {
                    if (tex.inBounds(x, y)) tex.SetPixel(x, y, color);
                }

                error = error - dy;
                if (error < 0)
                {
                    y = y + ystep;
                    error = error + dx;
                }
            }
        }

        /// <summary>
        /// Xiaolin Wu's line drawing algorithm with double endpoints and modified for arbitrary line width <paramref name="wd" />.
        /// </summary>
        public static void DrawLineAAF(this Texture2D tex, double x0, double y0, double x1, double y1, Color color, double wd)
        {
            double dx = x1 - x0;
            double dy = y1 - y0;
            bool steep = Math.Abs(dx) < Math.Abs(dy);

            if (steep)
            {
                Swap(ref x1, ref y1);
                Swap(ref x0, ref y0);
                Swap(ref dx, ref dy);
            }
            if (x1 < x0)
            {
                Swap(ref x0, ref x1);
                Swap(ref y0, ref y1);
            }

            double grad = dy / dx;
            double intery = y0 + rfpart(x0) * grad;

            // starting point
            int xend = round(x0);
            double yend = y0 + grad * (xend - x0) + 0.5 - 0.5 * wd;
            double xgap;
            int px = xend, py = (int) yend;
            double high = yend + wd;
            if (steep)
            {
                xgap = rfpart(y0 + 0.5);
                tex.SetPixelAA(py, px, color, rfpart(yend) * xgap);
                for (int y = py + 1; y < (int) high; y++)
                    tex.SetPixelAA(y, px, color, xgap);
                tex.SetPixelAA((int) high, px, color, fpart(high) * xgap);
            }
            else
            {
                xgap = rfpart(x0 + 0.5);
                tex.SetPixelAA(px, py, color, rfpart(yend) * xgap);
                for (int y = py + 1; y < (int) high; y++)
                    tex.SetPixelAA(px, y, color, xgap);
                tex.SetPixelAA(px, (int) high, color, fpart(high) * xgap);
            }
            int xstart = px + 1;

            // end point
            xend = round(x1);
            yend = y1 + grad * (xend - x1) + 0.5 - 0.5 * wd;
            px = xend;
            py = (int) yend;
            high = yend + wd;
            if (steep)
            {
                xgap = rfpart(y1 + 0.5);
                tex.SetPixelAA(py, px, color, rfpart(yend) * xgap);
                for (int y = py + 1; y < (int) high; y++)
                    tex.SetPixelAA(y, px, color, xgap);
                tex.SetPixelAA((int) high, px, color, fpart(high) * xgap);
            }
            else
            {
                xgap = rfpart(x1 + 0.5);
                tex.SetPixelAA(px, py, color, rfpart(yend) * xgap);
                for (int y = py + 1; y < (int) high; y++)
                    tex.SetPixelAA(px, y, color, xgap);
                tex.SetPixelAA(px, (int) high, color, fpart(high) * xgap);
            }

            // interior
            intery += 0.5 - 0.5 * wd;
            if (steep)
            {
                for (int x = xstart; x < xend; x++)
                {
                    // support for linewidth
                    double yhigh = intery + wd;
                    tex.SetPixelAA((int) intery, x, color, rfpart(intery));
                    tex.SetPixelAA((int) yhigh, x, color, fpart(yhigh));
                    for (int y = (int) intery + 1; y < (int) yhigh; y++)
                        tex.SetPixelAA(y, x, color);
                    intery += grad;
                }
            }
            else
            {
                for (int x = xstart; x < xend; x++)
                {
                    // support for linewidth
                    double yhigh = intery + wd;
                    tex.SetPixelAA(x, (int) intery, color, rfpart(intery));
                    tex.SetPixelAA(x, (int) yhigh, color, fpart(yhigh));
                    for (int y = (int) intery + 1; y < (int) yhigh; y++)
                        tex.SetPixelAA(x, y, color);
                    intery += grad;
                }
            }
        }

        public static void SetPixelAA(this Texture2D tex, int x, int y, Color color, double alpha = 1.0)
        {
            // for now assuming that all lines on tex will be drawn with the same color
            if (tex.inBounds(x, y))
            {
                float a = tex.GetPixel(x, y).a;
                color.a = a + (1 - a) * (float)alpha;
                tex.SetPixel(x, y, color);
            }
        }

        private static void Swap(ref double x, ref double y)
        {
            double tmp = y;
            y = x;
            x = tmp;
        }

        private static int ipart(double x)
        {
            return (int) Math.Floor(x);
        }

        private static int round(double x)
        {
            return ipart(x + 0.5);
        }

        // fractional part of x
        private static double fpart(double x)
        {
            return x - ipart(x);
        }

        private static double rfpart(double x)
        {
            return 1 - fpart(x);
        }

        private static void Swap(ref int x, ref int y)
        {
            int tmp = y;
            y = x;
            x = tmp;
        }
    }
}
