// this is the RobotModel

#include "robot_model.h"
#include <math.h>
#include "globals.h"
#include <Arduino.h>

RobotPos calc_future_position(RobotPos current_position, motion_option opt, int current_motion_option, float dt){

    float v = opt.speed;
    float omega = opt.omega;
    RobotPos next = current_position;
    
    /*Serial.print("x = ");
    Serial.println(next.x);
    Serial.print("y = ");
    Serial.println(next.y);
    Serial.print("dt = ");
    Serial.println(dt);*/

    if(current_motion_option == 3){
    next.x += v * cos(current_position.theta) * dt;
    next.y += v * sin(current_position.theta) * dt;
    }
    else 
    {
        next.x += (v/omega) * (sin(current_position.theta + omega * dt) -sin(current_position.theta)); //fault this might not be a fault but I need to recheck this
        next.y += (v/omega) * (-cos(current_position.theta + omega * dt) +cos(current_position.theta));
    }
    next.theta += omega * dt;

    return next;
}