#include "position_on_line.h"
#include "robot_model.h"
#include <Arduino.h>
#include <math.h>

// this is the cost function

float cost_function(const RobotPos& current_position, const RobotPos& future_position, bool mode_changed)
{
    float x_r_closest, y_r_closest; //position of robot on the line
    float x_f_closest, y_f_closest; //future position of robot on the line
    float distance_to_path, ds, signed_dist = 0;
    position_on_line(current_position.x, current_position.y, x_r_closest, y_r_closest, distance_to_path, ds, signed_dist);
    position_on_line(future_position.x, future_position.y, x_f_closest, y_f_closest, distance_to_path, ds, signed_dist);
    ds = sqrt(pow(x_f_closest - x_r_closest,2) + pow(y_f_closest - y_r_closest,2));
    float cost = distance_to_path*500; //this is max 0.03 m * 200 = 6
    cost -= ds*75; //this is at the max 0.033 * 200 = 6.6
    cost += mode_changed*0.5; //max value 2
    
    return cost;
}
