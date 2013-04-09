/*
 * This file is part of Laustracker.
 *
 * Copyright (C) 2013 Christian Jann <christian.jann@ymail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Copyright (c) 2011 the authors listed at the following URL, and/or
 * the authors of referenced articles or incorporated external code:
 * http://en.literateprograms.org/RGB_to_HSV_color_space_conversion_(C)?action=history&offset=20110802173944
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Retrieved from: http://en.literateprograms.org/RGB_to_HSV_color_space_conversion_(C)?oldid=17206
 */

#include <laustracker/color_conversion.h>


using namespace cv;

#define MIN3(x,y,z)  ((y) <= (z) ? \
                      ((x) <= (y) ? (x) : (y)) \
                          : \
                          ((x) <= (z) ? (x) : (z)))

#define MAX3(x,y,z)  ((y) >= (z) ? \
                      ((x) >= (y) ? (x) : (y)) \
                          : \
                          ((x) >= (z) ? (x) : (z)))


struct hsv_color rgb_to_hsv(struct rgb_color rgb)
{
    struct hsv_color hsv;
    unsigned char rgb_min, rgb_max;
    rgb_min = MIN3(rgb.r, rgb.g, rgb.b);
    rgb_max = MAX3(rgb.r, rgb.g, rgb.b);

    hsv.val = rgb_max;
    if (hsv.val == 0)
    {
        hsv.hue = hsv.sat = 0;
        return hsv;
    }

    hsv.sat = 255 * (long)(rgb_max - rgb_min) / hsv.val;
    if (hsv.sat == 0)
    {
        hsv.hue = 0;
        return hsv;
    }

    /* Compute hue */
    if (rgb_max == rgb.r)
    {
        hsv.hue = 0 + 43 * (rgb.g - rgb.b) / (rgb_max - rgb_min);
    }
    else if (rgb_max == rgb.g)
    {
        hsv.hue = 85 + 43 * (rgb.b - rgb.r) / (rgb_max - rgb_min);
    }
    else /* rgb_max == rgb.b */
    {
        hsv.hue = 171 + 43 * (rgb.r - rgb.g) / (rgb_max - rgb_min);
    }

    return hsv;
}

/* HSV colors
 * opencv:
 *       H: 0..179
 *       S: 0..255
 *       V: 0..255
 *
 */
Scalar bgr_to_cv_hsv(Scalar rgb)
{
    struct rgb_color _rgb;
    struct hsv_color _hsv;
    _rgb.r = (unsigned char)rgb[2];
    _rgb.g = (unsigned char)rgb[1];
    _rgb.b = (unsigned char)rgb[0];
    _hsv = rgb_to_hsv(_rgb);
    //printf("Hue: %d\nSaturation: %d\nValue: %d\n\n", _hsv.hue, _hsv.sat, _hsv.val);

    return Scalar((int)(_hsv.hue * 179 / 255.0), _hsv.sat, _hsv.val);
}

/* HSV colors
 * gimp:
 *       H: 0..360
 *       S: 0..100
 *       V: 0..100
 */
Scalar bgr_to_gimp_hsv(Scalar rgb)
{
    struct rgb_color _rgb;
    struct hsv_color _hsv;
    _rgb.r = (unsigned char)rgb[2];
    _rgb.g = (unsigned char)rgb[1];
    _rgb.b = (unsigned char)rgb[0];
    _hsv = rgb_to_hsv(_rgb);
    //printf("Hue: %d\nSaturation: %d\nValue: %d\n\n",
    //       _hsv.hue, _hsv.sat, _hsv.val);

    return Scalar((int)(_hsv.hue * 360 / 255.0),
                  (int)(_hsv.sat * 100 / 255.0),
                  (int)(_hsv.val * 100 / 255.0));
}

/* HSV colors
 * KColorChooser:
 *       H: 0..359
 *       S: 0..255
 *       V: 0..255
 */
Scalar bgr_to_kcc_hsv(Scalar rgb)
{
    struct rgb_color _rgb;
    struct hsv_color _hsv;
    _rgb.r = (unsigned char)rgb[2];
    _rgb.g = (unsigned char)rgb[1];
    _rgb.b = (unsigned char)rgb[0];
    _hsv = rgb_to_hsv(_rgb);
    //printf("Hue: %d\nSaturation: %d\nValue: %d\n\n", _hsv.hue, _hsv.sat, _hsv.val);

    return Scalar((int)(_hsv.hue * 359 / 255.0), (int)(_hsv.sat), (int)(_hsv.val));
}
