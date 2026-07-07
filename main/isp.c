/**
*****************************************************************************************
*     Copyright(c) 2022, Seeed Technology Corporation. All rights reserved.
*****************************************************************************************
* @file      isp.h
* @brief
* @author    Hongtai Liu (lht856@foxmail.com)
* @date      2022-05-19
* @version   v1.0
**************************************************************************************
* @attention
* <h2><center>&copy; COPYRIGHT 2022 Seeed Technology Corporation</center></h2>
**************************************************************************************
*/

#include <stdio.h>
#include <stdint.h>
#include "isp.h"

#define CLIP(value) (unsigned char)(((value) > 0xFF) ? 0xff : (((value) < 0) ? 0 : (value)))

const uint8_t _RGB565_TO_RGB_888_TABLE_5[] = { 0, 8, 16, 25, 33, 41, 49, 58, 66, 74, 82, 90, 99, 107, 115, 123, 132, 140, 148, 156, 165, 173, 181, 189, 197, 206, 214, 222, 230, 239, 247, 255 };

const uint8_t _RGB565_TO_RGB_888_TABLE_6[] = { 0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 45, 49, 53, 57, 61, 65, 69, 73, 77, 81, 85, 89, 93, 97, 101, 105, 109, 113, 117, 121, 125, 130, 134, 138, 142,
    146, 150, 154, 158, 162, 166, 170, 174, 178, 182, 186, 190, 194, 198, 202, 206, 210, 215, 219, 223, 227, 231, 235, 239, 243, 247, 251, 255 };

void rgb565_to_gray(uint8_t *pdst, const uint8_t *psrc, int h, int w, int th, int tw, uint8_t rotation, bool mirror)
{
    uint8_t r, g, b;
    uint32_t init_index, index;

    float beta_h = (float)h / th, beta_w = (float)w / tw;

    // ESP_LOGI(TAG, "h:%d, w:%d, th:%d, tw:%d, beta_h:%f, beta_w:%f", h, w, th, tw, beta_h, beta_w);

    for (int i = 0; i < th; i++)
    {
        for (int j = 0; j < tw; j++)
        {
            int tmph = i * beta_h, tmpw = beta_w * j;
            // select pixel
            index = i * tw + j;
            init_index = tmph * w + tmpw; // ou

            r = _RGB565_TO_RGB_888_TABLE_5[((psrc[init_index * 2] & 0xF8) >> 3)];
            g = _RGB565_TO_RGB_888_TABLE_6[((psrc[init_index * 2] & 0x07) << 3) | ((psrc[init_index * 2 + 1] & 0xE0) >> 5)];
            b = _RGB565_TO_RGB_888_TABLE_5[(psrc[init_index * 2 + 1] & 0x1F)];

            switch (rotation)
            {
                case ROTATION_LEFT:
                    index = (tw - 1 - index % tw) * (th) + index / tw;
                    break;
                case ROTATION_RIGHT:
                    index = (index % tw) * (th) + (th - 1 - index / tw);
                    break;
                default:
                    break;
            }

            if (mirror)
            {
                index = (th - 1 - index / th) * (tw) + index % tw;
            }

            pdst[index] = ((uint8_t)r * 299 + (uint8_t)g * 587 + (uint8_t)b * 114) / 1000;
        }
    }
}
