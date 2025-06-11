#include <ZumoMotors.h>
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
#include "line_read.h"
#include "position_on_line.h"
#include "robot_model.h"
#include "globals.h"
#include "kalman_filter.h"



//functions

//fault I still need to set the motors

bool invf3x3(const float A[3][3], float invA[3][3]) {
  float det =
      A[0][0] * (A[1][1]*A[2][2] - A[1][2]*A[2][1]) -
      A[0][1] * (A[1][0]*A[2][2] - A[1][2]*A[2][0]) +
      A[0][2] * (A[1][0]*A[2][1] - A[1][1]*A[2][0]);

  if (fabs(det) < 1e-6) { //if my matrix is zero or near zero it stops
    return false;
  }

  float invDet = 1.0 / det;

  invA[0][0] =  (A[1][1]*A[2][2] - A[1][2]*A[2][1]) * invDet;
  invA[0][1] = -(A[0][1]*A[2][2] - A[0][2]*A[2][1]) * invDet;
  invA[0][2] =  (A[0][1]*A[1][2] - A[0][2]*A[1][1]) * invDet;

  invA[1][0] = -(A[1][0]*A[2][2] - A[1][2]*A[2][0]) * invDet;
  invA[1][1] =  (A[0][0]*A[2][2] - A[0][2]*A[2][0]) * invDet;
  invA[1][2] = -(A[0][0]*A[1][2] - A[0][2]*A[1][0]) * invDet;

  invA[2][0] =  (A[1][0]*A[2][1] - A[1][1]*A[2][0]) * invDet;
  invA[2][1] = -(A[0][0]*A[2][1] - A[0][1]*A[2][0]) * invDet;
  invA[2][2] =  (A[0][0]*A[1][1] - A[0][1]*A[1][0]) * invDet;

  return true;
}

float mult1x3_3x1(float A[1][3], float B[3][1]) {
  float result = 0;
  for(int k = 0; k < 3; k++)
  {
    result += A[0][k] * B[k][0];
  }
  return result;
  }

void mult3x1_1x3(float A[3][1], float B[1][3], float result[3][3]){
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      result[i][j] = A[i][0] * B[0][j];
    }
  }
}

void mult3x3_3x1(float A[3][3], float B[3][1], float result[3][1]) {
  for (int i = 0; i < 3; i++) {
    result[i][0] = 0;
    for (int j = 0; j < 3; j++) {
      result[i][0] += A[i][j] * B[j][0];
    }
  }
}

void mult1x3_3x3(float A[1][3], float B[3][3], float result[1][3]) {
  for (int i = 0; i < 3; i++) {
    result[0][i] = 0;
    for (int j = 0; j < 3; j++) {
      result[0][i] += A[0][j] * B[j][i];
    }
  }
}

void mult3x3_3x3(float A[3][3], float B[3][3], float result[3][3]) {
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++){
      result[i][j] = 0;
      for (int k = 0; k < 3 ; k++){
        result[i][j] += A[i][k] * B[k][j];
      }
    }
  }
}

void transp3x3(float A[3][3], float result[3][3]){
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      result[j][i] = A[i][j];
    }
  }
}

void add3x3(float A[3][3], float B[3][3], float result[3][3]){
  for (int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      result[i][j] = A[i][j] + B[i][j];
    }
  }
}

void add3x1(float A[3][1], float B[3][1], float result[3][1]){
  for(int i = 0; i < 3; i++){
    result[i][0] = A[i][0] + B[i][0];
  }
}

void sub3x3(float A[3][3], float B[3][3], float result[3][3]){
  for (int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      result[i][j] = A[i][j] - B[i][j];
    }
  }
}

void sub3x1(float A[3][1], float B[3][1], float result[3][1]){
  for(int i = 0; i < 3; i++){
    result[i][0] = A[i][0] - B[i][0];
  }
}

void mults3x3(float scalar, float A[3][3], float result[3][3]){
  for (int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      result[i][j] = scalar * A[i][j];
    }
  }
}

void mults3x1(float scalar, float A[3][1], float result[3][1]){
  for (int i = 0; i < 3; i++){
    result[i][0] = scalar * A[i][0];
  
  }
}

float inv3x3[3][3] = {
                    {0,0,0},
                    {0,0,0},
                    {0,0,0}
};

float temp3x3[3][3] = {
                    {0,0,0},
                    {0,0,0},
                    {0,0,0}
};

float temp23x3[3][3] = {
                    {0,0,0},
                    {0,0,0},
                    {0,0,0}
};

float temp1x3[1][3] = {
                {0,0,0}
};

float temp3x1[3][1] = {
                {0},
                {0},
                {0},
};
float temp = 0;

//kalman gain gives information about how much to trust the new measurement
float K[3][3] = {
                {0,0,0},
                {0,0,0},
                {0,0,0}
}; 

//new position
float x_upd[3][1] = {
  {0.0},
  {0.0},
  {0.0},
};

// predicted position
float x_pred[3][1] = {
  {0},
  {0},
  {0},
};

// Covariance matrices
float P_pred[3][3];   // P_(k|k-1)
float P_upd[3][3] = {
                    {0.0001,0,0},
                    {0,0.0001,0},
                    {0,0,0.007614}
};    // P_(k|k) the inital values are used for the first step and are educated guesses 


//jacobian matrix F used to describe how the next state changes based on the previous state
float F[3][3] = {
                {1,0,0},
                {0,1,0},
                {0,0,1}
};

//jacobian matrix F transposed
float Ft[3][3] = {
                {1,0,0},
                {0,1,0},
                {0,0,1}
};

//Q describes process noise... How much additional uncertainty is in the model (wheel slip, timing errors, etc...)
float Q[3][3] = {
                {0.001,0,0},
                {0,0.001,0},
                {0,0,0.03491}
};

//identity matrix
float I[3][3] = {{1,0,0},
                {0,1,0},
                {0,0,1}};

//matrix that tells which measure values are used: x, y and theta which has a dummy value
float H[3][3] = {{1,0,0},
                {0,1,0},
                {0,0,1}};
                 
float Ht[3][3] = {{1,0,0},
                 {0,1,0},
                 {0,0,1}}; //transposed

float y_k[3][1] = {
                {0},
                {0},
                {0},
};

float y_k_innovation[3][1] = {
                {0},
                {0},
                {0},
};

//how much noise do I expect for each of the measure values, theta is extremely high because it is not measured but has a dummy value, that way I dont have to redesign the entire matrix calculations as much and I could use theta if I find a way
float R[3][3] = {{0.000004,0,0}, 
                {0,0.000004,0},
                {0,0,1000000}};

float v = 0;
float omega = 0;
float dt = 0.01; //initial value of time that passes between 2 state updates
unsigned long lastUpdateTime = 0;
float dx = 0;
float dy = 0;

//motion_option opt the datatype for opt is motion_option and is already filled with the correct motion_option
void kalman_filter(RobotPos& current_position, motion_option opt, int current_motion_option) {

v = opt.speed;
omega = opt.omega;

//determening dt
unsigned long currentTime = millis();
if(runs == 0){
  lastUpdateTime = motorStartTime;
};

dt = (currentTime - lastUpdateTime)/1000.0;
lastUpdateTime = currentTime;

//calculate the next position
RobotPos next_position = calc_future_position(current_position, opt, current_motion_option, dt);

x_pred[0][0] = next_position.x;
x_pred[1][0] = next_position.y;
x_pred[2][0] = next_position.theta;

//calculating jacobian matrix(model of the robot): this gives information on how changes in the current state affect the predicted state when an input is received
F[0][2] = -dt * v * sin(x_pred[2][0]);
F[1][2] = dt * v * cos(x_pred[2][0]);

//calculating jacobian matrix transposed(model of the robot)
Ft[2][0] = -dt * v * sin(x_pred[2][0]);
Ft[2][1] = dt * v * cos(x_pred[2][0]);

//this part takes the measured values
float deviation = get_line_deviation();
//Serial.println(deviation);

//this part gives the devation from the line in x and y coordinates, example: dx = 0.05, dy 0.0 in the world frame the robot is 5 cm further on the x axis
dx = -sin(x_upd[2][0]) * deviation;
dy = cos(x_upd[2][0]) * deviation;

//necessary variables for call of the position_on_line function
float path_x;
float path_y;
float distance_to_path;
float distance_traveled;
float distance_to_path_signed;

//function that gives me the position on the path my robot refers to, it might deviate from this path for effieciency reasons
position_on_line(x_pred[0][0], x_pred[1][0], path_x, path_y, distance_to_path, distance_traveled, distance_to_path_signed);
//Serial.println(distance_to_path_signed);
//taking the point on the path the robot should be, adding its deviation to know where the robot should be in the world coordinate system, it is the the measured position
y_k[0][0] = path_x + dx;
y_k[1][0] = path_y + dy;

//update uncertainty: P_pred = F * P_upd * Ft + Q;
mult3x3_3x3(F, P_upd, temp3x3);
mult3x3_3x3(temp3x3, Ft, temp23x3);
add3x3(temp23x3, Q, P_pred);

//compute y_k_innovation: y_k_innovation = y_k - H * x_pred
mult3x3_3x1(H,x_pred,temp3x1);
sub3x1(y_k,temp3x1,y_k_innovation);

//compute kalman gain
//K = P_pred * Ht * (H*P_pred*Ht + R)^(-1); 
mult3x3_3x3(H,P_pred, temp3x3);
mult3x3_3x3(temp3x3, Ht,temp23x3);
add3x3(temp23x3,R,temp3x3);
invf3x3(temp3x3, temp23x3); 
mult3x3_3x3(P_pred, Ht, temp3x3);
mult3x3_3x3(temp3x3,temp23x3, K);

//x_upd =  x_pred + K * y_k_innovation; this is the new position
mult3x3_3x1(K, y_k_innovation, temp3x1);
add3x1(x_pred, temp3x1, x_upd);

//covariance update; how certain I am about this position
//P_upd = (I - K* H) * P_pred;
mult3x3_3x3(K, H, temp3x3);
sub3x3(I, temp3x3, temp23x3);
mult3x3_3x3(temp23x3, P_pred, P_upd);

current_position.x = x_upd[0][0];
current_position.y = x_upd[1][0];
current_position.theta = x_upd[2][0];
}
