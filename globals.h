// globals.h
#ifndef GLOBALS_H
#define GLOBALS_H
#include <stdint.h>

#define sensor_count 6

extern int runs;
extern int runs2;
extern float motorStartTime;
extern int current_motion_option;
extern int next_motion_option;
extern uint16_t SensorValues[sensor_count];

#endif
