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

#include "lofaro_darwin_ach.h"
#include <unistd.h>
int main()
{
  printf("Make System Object\n");
  DarwinAch da = DarwinAch();
  da.setDebug(true);
  int r = 0;
  
  printf("Opening Serial Port\n");
  r = da.open();

  printf("Start System Loop\n");
  r = da.loop(125.0, HZ_STATE_125_IMU);
  
  return 0;
}
