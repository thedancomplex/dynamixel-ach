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

#include "dynamixel_ach_client.h"
#include <unistd.h>
#include <string.h>

int16_t mot = 4;

int main()
{ 
  /* Make System Object */
  DynamixelAchClient dac = DynamixelAchClient();
  dac.setRefMode(MODE_REF);

  int r = 0;

  r = dac.cmd(DYNAMIXEL_CMD_ID_ADD, mot);
  if( r == DYNAMIXEL_CMD_OK ){ r=0; }
  else{ printf("1\n"); return 1; }

  /* Turn On System */
  r = dac.cmd(DYNAMIXEL_CMD_ON, true);
  if( r == DYNAMIXEL_CMD_OK ){ r=0; }
  else{ printf("1\n"); return 1; }

  /* Get into home positon */
 
  while(1)
  { 
//    dac.stageRefPos(mot, -0.05);
    dac.stageRefVel(mot, 0.0);
    dac.stageRefTorque(mot, 0.0);
    dac.postRef();
    dac.sleep(2.0);
    
//    dac.stageRefPos(mot, 0.05);
    dac.stageRefVel(mot, 0.0);
    dac.stageRefTorque(mot, 0.0);
    dac.postRef();
    dac.sleep(0.5);
    printf("0\n");
  }
  return 0;
}
