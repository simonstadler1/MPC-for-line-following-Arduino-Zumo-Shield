#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "robot_model.h"
#include "position_on_line.h"
#include "line_read.h"

void kalman_filter(RobotPos& current_position, motion_option opt, int current_motion_option);

#endif