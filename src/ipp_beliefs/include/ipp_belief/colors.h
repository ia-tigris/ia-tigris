//
// Created by andrew on 2/19/22.
//

#ifndef ipp_belief_COLORS_H
#define ipp_belief_COLORS_H

#include <visualization_msgs/Marker.h>


struct Color {
    unsigned int r, g, b;
};

// https://sashamaps.net/docs/resources/20-colors/
Color COLORS[]{{230, 25, 75},   {60, 180, 75},   {255, 225, 25}, {0, 130, 200},
               {245, 130, 48},  {145, 30, 180},  {70, 240, 240}, {240, 50, 230},
               {210, 245, 60},  {250, 190, 212}, {0, 128, 128},  {220, 190, 255},
               {170, 110, 40},  {255, 250, 200}, {128, 0, 0},    {170, 255, 195},
               {128, 128, 0},   {255, 215, 180}, {0, 0, 128},    {128, 128, 128},
               {255, 255, 255}, {0, 0, 0}};

std_msgs::ColorRGBA get_color(unsigned int index) {
    if (index >= sizeof(COLORS) / sizeof(Color)) {
        index = index % (sizeof(COLORS) / sizeof(Color));
    }

    Color c = COLORS[index];
    std_msgs::ColorRGBA color;
    color.r = (float)c.r / 255 * 0.9;
    color.g = (float)c.g / 255 * 0.9;
    color.b = (float)c.b / 255 * 0.9;
    color.a = 1.0;
    return color;
}

#endif  // ipp_belief_COLORS_H
