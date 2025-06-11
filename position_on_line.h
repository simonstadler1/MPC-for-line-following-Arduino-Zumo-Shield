#ifndef POSITION_ON_LINE_H
#define POSITION_ON_LINE_H

#include <Arduino.h>

enum SegmentType {STRAIGHT, ARC};

struct PathSegment {
    uint8_t type;
    float start_x;
    float start_y;
    float heading;
    float length;
    float radius;
};

extern const PathSegment path[] PROGMEM;
extern const uint8_t numSegments;

void position_on_line(float x_r, float y_r, float& x_closest, float& y_closest, float& min_dist, float& ds, float& signed_dist);

#endif