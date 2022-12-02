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

#include "lofaro_dynamixel_ach_client.h"
#include <unistd.h>
#include <string.h>

#define MOT_ID 4

int main()
{ 
  /* Make System Object */
  DynamixelAchClient dac = DynamixelAchClient();

  dac.rate(100.0);

  int r = 0;

  while(1)
  {
    /* Get State and Wait for New State */
    dac.getState(); 
    int    id  = MOT_ID;
    double ref = dac.dynamixel_state.motor_ref[id].pos;     
    double pos = dac.dynamixel_state.motor_state[id].pos;
    double t   = dac.time();     
    printf("t %f id %d ref %f pos %f\n", t, id, ref, pos);
    dac.sleep();
  }
  return 0;
}
