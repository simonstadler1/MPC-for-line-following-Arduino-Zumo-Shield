#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

struct RobotPos {
    float x;
    float y;
    float theta; //this is in radians
};

struct motion_option {
    float speed;
    float omega;
};

RobotPos calc_future_position(RobotPos current_position, motion_option opt, int current_motion_option, float dt);

#endif