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

#include "lofaro_darwin_ach_client.h"
#include <unistd.h>
#include <string.h>

int main(int argc, char *argv[])
{

  bool do_wait = true;
  if( argc >= 2)
  {
    std::string str = argv[1];

    if(str.compare("no_wait") == 0)
    {
      do_wait = false;
    }
  } 

  /* Make System Object */
  DarwinAchClient dac = DarwinAchClient();
  dac.setRefMode(MODE_REF);

  int r = 0;

  /* Turn On System */
  r = dac.cmd(DARWIN_CMD_ON, do_wait);
  if( r == DARWIN_CMD_OK ){ r=0; }
  else{ printf("1\n"); return 1; }

  /* Get into home positon */
  for(int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++)
  {
    dac.stageRefVel(i, DARWIN_REF_VEL_0);
    dac.stageRefPos(i, DARWIN_REF_POS_0);
  }
  dac.postRef();
  dac.sleep(2.0);

  /* Set Lower Body to be high torque and fast */
  for(int i = DARWIN_MOTOR_MIN_LOWER; i <= DARWIN_MOTOR_MAX_LOWER; i++)
  {
    dac.stageRefVel(i, 100.0);
    dac.stageRefTorque(i, 100.0);
  }
  dac.postRef();
  dac.sleep(0.5);
  printf("0\n");
  return 0;
}
