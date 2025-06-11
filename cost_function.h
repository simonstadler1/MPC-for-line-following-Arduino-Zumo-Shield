#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H

#include "robot_model.h"

float cost_function(const RobotPos& current_position,const RobotPos& future_position, bool mode_changed);

#endif