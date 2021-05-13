/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  A controller using the PID technic to control an inverted
 *               pendulum.
 */

#include <math.h>
#include <stdlib.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/gyro.h>
#include <stdio.h>

#define MAX_FORCE 1000   
#define CENTERING_ANGLE 0.7
#define CENTERING_FORCE 4
#define TIME_STEP 32

static WbDeviceTag horizontal_motor, horizontal_position_sensor, hip, robot_gyro_sensor;
double previous_position;
static double K[] = {-3.1623, -5.5583, -67.3167, -21.9235};    //feedback facter K
double velocity, angle;

static void initialize() {
  wb_robot_init();
  horizontal_motor = wb_robot_get_device("horizontal_motor");
  horizontal_position_sensor = wb_robot_get_device("horizontal position sensor");
  hip = wb_robot_get_device("hip");
  robot_gyro_sensor = wb_robot_get_device("robot_gyro");
  
  wb_position_sensor_enable(horizontal_position_sensor, TIME_STEP);
  wb_position_sensor_enable(hip, TIME_STEP);
  wb_gyro_enable(robot_gyro_sensor, TIME_STEP);
  
  wb_robot_step(TIME_STEP);

  previous_position = wb_position_sensor_get_value(horizontal_position_sensor);
  angle = wb_position_sensor_get_value(hip);
}

static void run() {
  double position = wb_position_sensor_get_value(horizontal_position_sensor);
  printf("position:%.5f ", position);
  
  angle = wb_position_sensor_get_value(hip);
  velocity = (position - previous_position) / ((double)TIME_STEP / 1000);
  const double *gyro = wb_gyro_get_values(robot_gyro_sensor);
  
  printf("velocity:%.5f ", velocity);
  printf("angle %.5f ", angle);
  printf("gyro[0]:%.5f ", gyro[0]);
  
  //u = -kx
  double power = K[0] * position + K[1] * velocity + K[2] * angle + K[3] * gyro[0];
  power *= -1;
  printf("power:%.5f\n", power);

  power = power < MAX_FORCE ? power : MAX_FORCE;
  power = power > -MAX_FORCE ? power : -MAX_FORCE;

  wb_motor_set_force(horizontal_motor, power);

  previous_position = position;
}

int main() {
  initialize();
  while (1) {
    wb_robot_step(TIME_STEP);
    run();
  }
  return 0;
}
