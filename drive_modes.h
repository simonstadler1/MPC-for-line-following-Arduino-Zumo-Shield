#ifndef DRIVE_MODES_H
#define DRIVE_MODES_H

#include <Arduino.h>
#include "robot_model.h"

extern const uint8_t next_try_possibilities[7][3];

extern const motion_option motion_option_array[7];
extern int current_motion_option;
extern int next_motion_option;

#endif