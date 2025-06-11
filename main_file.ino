#include "robot_model.h"
#include "line_read.h"
#include "cost_function.h"
#include <avr/pgmspace.h>
#include "kalman_filter.h"
#include "globals.h"
#include <Arduino.h>
#include <ZumoMotors.h>
#include "drive_modes.h"
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
#include <EEPROM.h>

ZumoMotors motors;
LSM303 compass;
L3G gyro;

const uint8_t next_try_possibilities[7][3] = {
    {0, 1, 2},
    {0, 1, 2},
    {1, 2, 3},
    {2, 3, 4},
    {3, 4, 5},
    {4, 5, 6},
    {4, 5, 6}
};

/*const motion_option motion_option_array[7] PROGMEM = {
    {0.0741f, 0.6732f}, // hard left
    {0.0924f, 0.4062f}, // medium left
    {0.1090f, 0.1614f}, // easy left
    {0.1239f, 0.0f},    // straight
    {0.1095f, 0.1745f}, // easy right
    {0.0889f, 0.4284f}, // medium right
    {0.0736f, 0.7363f}  // hard right
};*/

const motion_option motion_option_array[7] PROGMEM = {

    {0.1975f, 1.7952f},  // hard left
    {0.2465f, 1.0833f},  // medium left
    {0.2907f, 0.4304f},  // easy left
    {0.3304f, 0.0f},     // straight
    {0.2921f, -0.4654f},  // easy right
    {0.2370f, -1.1424f},  // medium right
    {0.1963f, -1.9635f}   // hard right
};



struct motion_for_arduino {
    int left_speed;
    int right_speed; 
};


const motion_for_arduino motion_for_arduino_array[7] PROGMEM = {
  
    {50, 200},  // hard left
    {100, 200}, // medium left
    {150, 200}, // easy left
    {200, 200},   // straight
    {200, 150}, // easy right
    {200, 100}, // medium right
    {200, 50}   // hard right
};


motion_for_arduino elected_motion;

RobotPos future_pos_lvl1[3] = {};
RobotPos future_pos_lvl2[9] = {};
float cost_lvl1[3] = {0,0,0};
float cost_lvl2[9] = {0,0,0,0,0,0,0,0,0};
motion_option opt = {}; //this tells me what speed and turning rate I currently have

RobotPos next_position = {0.0f,0.0f,0.0f};
int current_motion_option = 3;
int next_motion_option = 3;
int possible_motion_option = 0;
int possible_motion_option2 = 0;
RobotPos current_position = {0.0f,0.0f,0.0f};
float motorStartTime = 0;
int runs = 0;
int runs2 = 0;

//this is for the line sensor
uint16_t SensorValues[sensor_count];

void setup() {
  Serial.begin(9600);
  Wire.begin();
  // Initialize LSM303 and L3G sensors
  if (!compass.init()) {
    Serial.println("Failed to detect LSM303!");
    while (1);  // halt
  }
  compass.enableDefault();

  if (!gyro.init()) {
    Serial.println("Failed to detect L3G!");
    while (1);  // halt
  }
  gyro.enableDefault();
  // put your setup code here, to run once:

  reflectanceSensors.init();
  reflectanceSensors.emittersOn();

  Serial.println("Starting calibration");
  for (uint8_t i = 0; i < 200; i++){
    reflectanceSensors.calibrate();
    delay(20);
  }
  Serial.println("Calibration finished");
  delay(3000);
}

void loop(){

memcpy_P(&elected_motion, &motion_for_arduino_array[current_motion_option], sizeof(elected_motion));
motors.setSpeeds(elected_motion.left_speed, elected_motion.right_speed);
motorStartTime = millis();

//I overwrite the current_motion_option with the new aka next_motion_option
current_motion_option = next_motion_option;

memcpy_P(&opt, &motion_option_array[current_motion_option], sizeof(motion_option));//here opt gets loaded with the current motion option

//I calculate the current position
kalman_filter(current_position, opt, current_motion_option);

if(runs % 2 == 0 && runs < 120){
  int eepromAddr = runs/2*(sizeof(RobotPos));
EEPROM.put(eepromAddr, current_position);
};

//MPC: I calculate where the robot would be based on different motion options, after that a new motion option is elected

int k = 0;
  for(int i = 0; i <3; i++){

      float dt = 0.2;
      possible_motion_option = next_try_possibilities[current_motion_option][i];
      bool mode_changed = (current_motion_option != possible_motion_option);
      memcpy_P(&opt, &motion_option_array[possible_motion_option], sizeof(motion_option)); //this gives the values from the motion option array saved on MEMPROM
      future_pos_lvl1[i] = calc_future_position(current_position, opt, possible_motion_option, dt);
      /*if (current_position.x > 0.49 && runs2 < 4){
      int eepromAddr = (sizeof(RobotPos)*(i)*4)+runs2*sizeof(RobotPos)*12;
      EEPROM.put(eepromAddr, future_pos_lvl1[i]);
          };*/
      /*Serial.print("lvl1 ");
      Serial.print(i);
      Serial.print(" = ");
      Serial.print(future_pos_lvl1[i].x,4);
      Serial.print(" / ");
      Serial.print(future_pos_lvl1[i].y,4);
      Serial.print(" / ");
      Serial.println(future_pos_lvl1[i].theta,4);*/
      cost_lvl1[i] = cost_function(current_position, future_pos_lvl1[i], mode_changed);
      /*Serial.print("c ");
      Serial.println(cost_lvl1[i]);*/
       
      
        for(int j = 0; j <3; j++){
          possible_motion_option2 = next_try_possibilities[possible_motion_option][j];
          mode_changed = (possible_motion_option != possible_motion_option2);
          memcpy_P(&opt, &motion_option_array[possible_motion_option2], sizeof(motion_option)); //this gives the values from the motion option array saved on MEMPROM 
          future_pos_lvl2[k] = calc_future_position(future_pos_lvl1[i], opt, possible_motion_option2, dt);
          /*if (current_position.x > 0.49 && runs2 < 4){
          int eepromAddr = sizeof(RobotPos) + sizeof(RobotPos)*j + sizeof(RobotPos)*4*i+runs2*sizeof(RobotPos)*12;
          EEPROM.put(eepromAddr, future_pos_lvl2[j]);
          };*/
          /*Serial.print("lvl2 ");
          Serial.print(j);
          Serial.print(" = ");
          Serial.print(future_pos_lvl2[k].x,4);
          Serial.print(" / ");
          Serial.print(future_pos_lvl2[k].y,4);
          Serial.print(" / ");
          Serial.println(future_pos_lvl2[k].theta,4);*/
          cost_lvl2[k] = cost_lvl1[i] + cost_function(future_pos_lvl1[i], future_pos_lvl2[k], mode_changed);
          /*Serial.print("c ");
          Serial.println(cost_lvl2[j]);*/
           k++;
           
/*
          for(int l = 0; l < 3; l++){
            possible_motion_option3 = next_try_possibilities[possible_motion_option][j];
            mode_changed = (possible_motion_option3 != possible_motion_option2);
            memcpy_P(&opt, &motion_option_array[possible_motion_option3], sizeof(motion_option)); //this gives the values from the motion option array saved on MEMPROM 
            future_pos_lvl3[k] = calc_future_position(future_pos_lvl2[i], opt, possible_motion_option3, dt);        
            cost_lvl3[k] = cost_lvl2[i] + cost_function(future_pos_lvl2[i], future_pos_lvl3[k], mode_changed);
            
          }*/
        }
};

float lowest_cost = 500;
int lowest_cost_number = 0;

for(int i = 0; i<9; i++){
    //Serial.println(cost_lvl2[i]);
    if(cost_lvl2[i] < lowest_cost){
    lowest_cost = cost_lvl2[i];
    lowest_cost_number = i;
    }
};
//Serial.println(lowest_cost_number);
int above_same_below = (lowest_cost_number/3);
next_motion_option = next_try_possibilities[current_motion_option][above_same_below];
//Serial.println(next_motion_option);
//Serial.print("next motion option: ");
//Serial.println(next_motion_option);
/*if(current_position.x >0.49){
  runs2 += 1;
}*/
runs += 1;
/*if(runs >= 60){
  while(true);
};*/
/*Serial.println();
Serial.print("runs = ");
Serial.println(runs);*/

//Serial.println(millis());
//Serial.println(sizeof(RobotPos));  // Likely 12
/*Serial.println(sizeof(future_pos_lvl2));  // Is it 324?
Serial.println(sizeof(cost_lvl2));        // Is it 108?*/

}