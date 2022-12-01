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
  r = dl.on(MOT_ID);
  printf("Status = %d\n\n",r);

  for (int i = 0; i < 10; i++)
  {
    mot_pos = -mot_pos;
    r = dl.setMotPos(    MOT_ID, mot_pos);
    r = dl.setMotSpeed(  MOT_ID, 100.0);
    r = dl.setMotTorque( MOT_ID, 100.0);
    r = dl.stageMotor(MOT_ID); 
    r = dl.putMotor(MOT_ID);
    printf("Set = %f - Status = %d\n",mot_pos, r);
    r = dl.getMotor(MOT_ID);
    printf("Pos = %f\n", dl.dynamixel_data.motor_state[MOT_ID].pos);
    dl.sleep(2.0);
  }

  printf("Turn off actuator\n");
  r = dl.off(MOT_ID);
  printf("Status = %d\n\n",r);
  
  printf("Sleep for 3 seconds\n");
  r = dl.sleep(3.0);
  printf("Status = %d\n\n",r);

  return 0;
}
