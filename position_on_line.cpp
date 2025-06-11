#include <math.h>
#include <Arduino.h>
#include "position_on_line.h"

const PathSegment path[] PROGMEM = {
  {STRAIGHT, 0.05, 0.0, 0.0, 0.5, 0.0},
  {ARC, 0.5, 0.0, 0.0, 0.2356194, 0.15},
  {STRAIGHT, 0.65, 0.15, 1.570796, 0.3, 0.0},
};

/*const PathSegment path[] PROGMEM = {
  {STRAIGHT, 0.0, 0.0, 0.0, 3.0, 0.0},
  {ARC, 0.5, 0.0, 0.0, 0.2356194, 0.15},
  {STRAIGHT, 0.65, 0.15, 1.570796, 0.3, 0.0},
};*/

const uint8_t numSegments = sizeof(path) / sizeof(PathSegment);

void position_on_line(float x_r, float y_r,
                            float& x_closest, float& y_closest,
                            float& min_distance,
                            float& ds, float& signed_dist)
{
  
static int last_segment_number = -1; //static so the value remains in between calls of the function
static float last_s = 0; //s progress from the last call

min_distance = 10000000;
ds = 0;

float chosen_segment_s = 0;
int best_segment = -1;

  for (int i = 0; i < numSegments; i++) {
    PathSegment seg;
    memcpy_P(&seg, &path[i], sizeof(PathSegment));
    float x_p, y_p;
    float s = 0;

    if (seg.type == STRAIGHT) {
      float dx = x_r - seg.start_x;
      float dy = y_r - seg.start_y;
      float ux = cos(seg.heading);
      float uy = sin(seg.heading);

      s = dx * ux + dy * uy; //distance from the segment start to the robot's closest point on the path

      if(s < 0){
        s = 0;
      }

      if(s > seg.length){
        s = seg.length;
      } 

      x_p = seg.start_x + s * ux; //x_position of the robot's closes point on the line
      y_p = seg.start_y + s * uy; //y_position of the robot's closes point on the line
    }

    else if (seg.type == ARC) {

      float theta0 = seg.heading; //initial orientation of the segment
      float cx = seg.start_x - seg.radius * sin(theta0); //x position of the center of the arc
      float cy = seg.start_y + seg.radius * cos(theta0); //y position of the center of the arc

      float dx = x_r - cx; 
      float dy = y_r - cy;
      float angle_to_robot = atan2(dx, -dy); //angle between center of the arc and the robot

      float max_angle = seg.length / seg.radius;
      float angle_start = theta0;
      float angle_end = theta0 + max_angle;

      float angle = angle_to_robot;
      if(angle < angle_start){
        angle = angle_start;
      }

      if(angle > angle_end){
        angle = angle_end;
      } 

      s = (angle - theta0) * seg.radius;

      x_p = cx + seg.radius * sin(angle);
      y_p = cy - seg.radius * cos(angle);
    }

    float distance = sqrt(sq(x_p - x_r) + sq(y_p - y_r)); //computes  distance 

    if (distance < min_distance) {
      min_distance = distance;
      x_closest = x_p;
      y_closest = y_p;
      chosen_segment_s = s;
      best_segment = i;
    }
  }

  // Compute signed distance
float ux, uy;
  PathSegment seg;
  memcpy_P(&seg, &path[best_segment], sizeof(PathSegment));

  if(seg.type == ARC){
    float heading_for_s = seg.heading + chosen_segment_s/seg.radius;
    ux = cos(heading_for_s);
    uy = sin(heading_for_s);
  }

  else{
  ux = cos(seg.heading);
  uy = sin(seg.heading);
  }

  float rx = x_r - x_closest;
  float ry = y_r - y_closest;

  signed_dist = -(ux * ry - uy * rx); // Negative = left; positive = right


  // Computing ds
  if (last_segment_number == best_segment) {
    ds = chosen_segment_s - last_s;
  } else {
    ds = chosen_segment_s;  // reset the value for a new segment
  }

  // Update static memory
  last_segment_number = best_segment;
  last_s = chosen_segment_s;
  /*Serial.print("pop ");
  Serial.print(x_closest);
  Serial.print(" / ");
  Serial.println(y_closest);*/
}
