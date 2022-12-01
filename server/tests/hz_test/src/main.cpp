/*******************************************************************************
* Copyright 2022 Daniel M. Lofaro
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
*******************************************************************************/

/* Author: Daniel M. Lofaro */

#include "lofaro_dynamixel.h"
#include <unistd.h>

#define MOT_ID 4
double mot_pos = 0.2;

int main()
{
  printf("Make System Object\n");
  DynamixelLofaro dl = DynamixelLofaro();
  int r = 0;
  
  printf("Opening Serial Port and set to low latency\n");
  r = dl.setup(true);
  printf("Status = %d\n\n",r);

  printf("Turn on actuator\n");
  r = dl.idAdd(MOT_ID);
  r = dl.on();
  //r = dl.on(MOT_ID);
  printf("Status = %d\n\n",r);

  while(true)
  {
    mot_pos = -mot_pos;
    double t = dl.time();
    r = dl.setMotPos(    MOT_ID, mot_pos);
    r = dl.setMotSpeed(  MOT_ID, 100.0);
    r = dl.setMotTorque( MOT_ID, 0.3);
    r = dl.stageMotor(MOT_ID); 
    r = dl.putMotor();
    r = dl.getMotor(MOT_ID);
    printf("time %f ref %f state %f\n",t, 
                                       dl.dynamixel_data.motor_ref[MOT_ID].pos,
                                       dl.dynamixel_data.motor_state[MOT_ID].pos);
    dl.sleep(0.05);
  }

  printf("Turn off actuator\n");
  r = dl.off(MOT_ID);
  printf("Status = %d\n\n",r);
  
  printf("Sleep for 3 seconds\n");
  r = dl.sleep(3.0);
  printf("Status = %d\n\n",r);

  return 0;
}
