#ifndef DYNAMIXEL_LOFARO_ACH 

#include "lofaro_dynamixel_includes_ach.h"
#include "lofaro_dynamixel_defines_ach.h"
#include "lofaro_defines.h"
#include "lofaro_utils.h"
#include "lofaro_types.h"
#include <cstring>
#include <chrono>
#include <functional>
#include <memory>
#include <string>



class DynamixelAchClient
{
  public:
    DynamixelAchClient();
    DynamixelAchClient(bool do_flush);
    int cmd(int cmd);
    int cmd(int cmd, bool block);
    int cmd(int cmd, int16_t d0);
    int cmd(int cmd, int16_t d0, bool block);
    int cmd(int cmd, int16_t d0, int16_t d1);
    int cmd(int cmd, int16_t d0, int16_t d1, bool block);
    int cmd(int cmd, int16_t d0, double f0);
    int cmd(int cmd, int16_t d0, double f0, bool block);
    int cmd(int cmd, int16_t data[4]);
    int cmd(int cmd, int16_t data[4], bool block);
    int cmd(int cmd, int16_t data[4], double data_float[4]);
    int cmd(int cmd, int16_t data[4], double data_float[4], bool block);

    int rate(double hz);
    int sleep();
    int sleep(double val);
    double time();

    /* Update Methods */
    int getState();
    int getState(bool wait);
    int getTime();
    int getTime(bool wait);
    int stageRefPos(int mot, double val);
    int stageRefPosD(int mot, double val);
    int stageRefVel(int mot, double val);
    int stageRefTorque(int mot, double val);
    int stagePGain(int mot, double val);
    int stageIGain(int mot, double val);
    int stageDGain(int mot, double val);
    int setRefMode(int mode);
    int postRef();
    
    int stageCmdVelMode(int val);
    int stageCmdVelX(double val);
    int stageCmdVelY(double val);
    int stageCmdVelThetaZ(double val);
    int postCmdVel();
    int getCmdVel();
    int getCmdVelMode();
    double getCmdVelX(); 
    double getCmdVelY();  
    double getCmdVelThetaZ(); 


    /* Data types */
    dynamixel_data_def_t     dynamixel_ref;
    dynamixel_data_def_t     dynamixel_state;
    dynamixel_cmd_def_t      dynamixel_cmd;
    dynamixel_cmd_def_t      dynamixel_cmd_return;
    dynamixel_cmd_vel_def_t  dynamixel_cmd_vel;
    double                dynamixel_time = 0.0;

  private:
    int ref_mode = MODE_REF;
    int stageGain(int mot, double val);
    void constructDynamixelAchClient(bool do_flush);

    LofaroUtils* lu = new LofaroUtils();

    bool run_loop = false;

    /* Reference Channel */
    ach_channel_t chan_dynamixel_ref;  

    /* Reference Walking Channel */
    ach_channel_t chan_dynamixel_ref_walking;  

    /* State Feedback Channel */
    ach_channel_t chan_dynamixel_state;

    /* Command channel */
    ach_channel_t chan_dynamixel_cmd;

    /* Command Channel Return */
    ach_channel_t chan_dynamixel_cmd_return;

    /* Command Vel Channel */
    ach_channel_t chan_dynamixel_cmd_vel;

    /* Command Time Channel */
    ach_channel_t chan_dynamixel_time;

};
DynamixelAchClient::DynamixelAchClient()
{
  this->constructDynamixelAchClient(false);
  return;
}

DynamixelAchClient::DynamixelAchClient(bool do_flush)
{
  this->constructDynamixelAchClient(do_flush);
  return;
}

void DynamixelAchClient::constructDynamixelAchClient(bool do_flush)
{
  /* Zero Data */
  memset(&this->dynamixel_ref,          0, sizeof(this->dynamixel_ref));
  memset(&this->dynamixel_state,        0, sizeof(this->dynamixel_state));
  memset(&this->dynamixel_cmd,          0, sizeof(this->dynamixel_cmd));
  memset(&this->dynamixel_cmd_return,   0, sizeof(this->dynamixel_cmd_return));
  memset(&this->dynamixel_cmd_vel,      0, sizeof(this->dynamixel_cmd_vel));


  for( int i = 0; i <= DYNAMIXEL_MOTOR_MAX; i++ )
  {
    this->dynamixel_ref.motor_ref[i].pos    = DYNAMIXEL_REF_POS_0;
    this->dynamixel_ref.motor_ref[i].speed  = DYNAMIXEL_REF_VEL_0;
    this->dynamixel_ref.motor_ref[i].torque = DYNAMIXEL_REF_TOR_0;
  }


  /* Make Ach Channels */
  ach_status_t r = ACH_OK;

  /* Open Channels */
  r = ach_open(&this->chan_dynamixel_ref,         DYNAMIXEL_ACH_CHAN_REF,         NULL);
  r = ach_open(&this->chan_dynamixel_state,       DYNAMIXEL_ACH_CHAN_STATE,       NULL);
  r = ach_open(&this->chan_dynamixel_cmd,         DYNAMIXEL_ACH_CHAN_CMD,         NULL);
  r = ach_open(&this->chan_dynamixel_cmd_return,  DYNAMIXEL_ACH_CHAN_CMD_RETURN,  NULL);
  r = ach_open(&this->chan_dynamixel_time,        DYNAMIXEL_ACH_CHAN_TIME,        NULL);

  /* Flush Ach Channels */
  if(do_flush)
  {
    r = ach_flush(&this->chan_dynamixel_state);
    r = ach_flush(&this->chan_dynamixel_cmd_return);
    r = ach_flush(&this->chan_dynamixel_time);
  }

  /* Do initial put on the channel to make sure the exist */
/*
  ach_put(&this->chan_dynamixel_ref,        &this->dynamixel_ref,        sizeof(this->dynamixel_ref));
  ach_put(&this->chan_dynamixel_state,      &this->dynamixel_state,      sizeof(this->dynamixel_state));
  ach_put(&this->chan_dynamixel_cmd,        &this->dynamixel_cmd,        sizeof(this->dynamixel_cmd));
  ach_put(&this->chan_dynamixel_cmd_return, &this->dynamixel_cmd_return, sizeof(this->dynamixel_cmd_return));
*/
  return;
}

double DynamixelAchClient::time()
{
  return this->lu->getTime();
}

int DynamixelAchClient::sleep(double val)
{
  return this->lu->sleep(val);
}

int DynamixelAchClient::sleep()
{
  return this->lu->sleep();
}

int DynamixelAchClient::rate(double hz)
{
  return this->lu->rate(hz);
}

int DynamixelAchClient::getTime()
{
  size_t fs;
  ach_status_t r = ach_get( &this->chan_dynamixel_time, &this->dynamixel_time, sizeof(this->dynamixel_time), &fs, NULL, ACH_O_LAST );
  if( ( r == ACH_OK ) | ( r == ACH_MISSED_FRAME ) ) return 0;
  return 1;
}

int DynamixelAchClient::getTime(bool wait)
{
  if(wait)
  {
    size_t fs;
    ach_status_t r = ach_get( &this->chan_dynamixel_time, &this->dynamixel_time, sizeof(this->dynamixel_time), &fs, NULL, ACH_O_WAIT );
    if( ( r == ACH_OK ) | ( r == ACH_MISSED_FRAME ) ) return 0;
    return 1;
  }
  else
  {
    return this->getTime();
  }
  return 1;
}

int DynamixelAchClient::getState()
{
  size_t fs;
  ach_status_t r = ach_get( &this->chan_dynamixel_state, &this->dynamixel_state, sizeof(this->dynamixel_state), &fs, NULL, ACH_O_LAST );
  return (int)r;
}

int DynamixelAchClient::getState(bool wait)
{
  if(wait)
  {
    size_t fs;
    ach_status_t r = ach_get( &this->chan_dynamixel_state, &this->dynamixel_state, sizeof(this->dynamixel_state), &fs, NULL, ACH_O_WAIT );
    return (int)r;
  }
  else
  {
    return this->getState();
  }
  return 1;
}

int DynamixelAchClient::getCmdVel()
{
  size_t fs;
  ach_status_t r = ach_get( &this->chan_dynamixel_cmd_vel, &this->dynamixel_cmd_vel, sizeof(this->dynamixel_cmd_vel), &fs, NULL, ACH_O_LAST );
  if( ( r == ACH_OK ) | ( r == ACH_MISSED_FRAME ) ) return 0;
  return 1;
}

int DynamixelAchClient::getCmdVelMode(){      return (int)this->dynamixel_cmd_vel.mode;  }
double DynamixelAchClient::getCmdVelX(){      return this->dynamixel_cmd_vel.linear.x;  }
double DynamixelAchClient::getCmdVelY(){      return this->dynamixel_cmd_vel.linear.y;  }
double DynamixelAchClient::getCmdVelThetaZ(){ return this->dynamixel_cmd_vel.angular.z; }

int DynamixelAchClient::stageCmdVelMode(int val)
{
  this->dynamixel_cmd_vel.mode = val;
  return 0;
}

int DynamixelAchClient::stageCmdVelX(double val)
{
  this->dynamixel_cmd_vel.linear.x = val;
  return 0;
}

int DynamixelAchClient::stageCmdVelY(double val)
{
  this->dynamixel_cmd_vel.linear.y = val;
  return 0;
}

int DynamixelAchClient::stageCmdVelThetaZ(double val)
{
  this->dynamixel_cmd_vel.angular.z = val;
  return 0;
}
int DynamixelAchClient::postCmdVel()
{
  ach_status_t r = ACH_OK;
  r = ach_put(&this->chan_dynamixel_cmd_vel, &this->dynamixel_cmd_vel, sizeof(this->dynamixel_cmd_vel));
  if (r > 0) return 1;
  return 0;
}

int DynamixelAchClient::cmd(int cmd){ return this->cmd(cmd, false); }
int DynamixelAchClient::cmd(int cmd, bool block)
{
  int16_t data[4];
  double  data_float[4];
  memset(&data,       0, sizeof(data));
  memset(&data_float, 0, sizeof(data_float));
  return this->cmd(cmd, data, data_float, block);
}

int DynamixelAchClient::cmd(int cmd, int16_t d0){ return this->cmd(cmd, d0, false); }
int DynamixelAchClient::cmd(int cmd, int16_t d0, bool block)
{
  int16_t data[4];
  double  data_float[4];
  memset(&data,       0, sizeof(data));
  memset(&data_float, 0, sizeof(data_float));
  data[0] = d0;
  return this->cmd(cmd, data, data_float, block);
}

int DynamixelAchClient::cmd(int cmd, int16_t d0, int16_t d1){ return this->cmd(cmd, d0, d1, false); }
int DynamixelAchClient::cmd(int cmd, int16_t d0, int16_t d1, bool block)
{
  int16_t data[4];
  double  data_float[4];
  memset(&data,       0, sizeof(data));
  memset(&data_float, 0, sizeof(data_float));
  data[0] = d0;
  data[1] = d1;
  return this->cmd(cmd, data, data_float, block);
}

int DynamixelAchClient::cmd(int cmd, int16_t d0, double f0){ return this->cmd(cmd, d0, f0, false); }
int DynamixelAchClient::cmd(int cmd, int16_t d0, double f0, bool block)
{
  int16_t data[4];
  double  data_float[4];
  memset(&data,       0, sizeof(data));
  memset(&data_float, 0, sizeof(data_float));
  data[0] = d0;
  data_float[0] = f0;
  return this->cmd(cmd, data, data_float, block);
}

int DynamixelAchClient::cmd(int cmd, int16_t data[4]){ return this->cmd(cmd, data, false); }
int DynamixelAchClient::cmd(int cmd, int16_t data[4], bool block)
{
  double  data_float[4];
  memset(&data_float, 0, sizeof(data_float));
  return this->cmd(cmd, data, data_float, block);
}

int DynamixelAchClient::cmd(int cmd, int16_t data[4], double data_float[4]){ return this->cmd(cmd, data, data_float, false); }
int DynamixelAchClient::cmd(int cmd, int16_t data[4], double data_float[4], bool block)
{
  if( cmd == DYNAMIXEL_CMD_MODE ) ref_mode = data[0];

  size_t fs;
  ach_status_t r = ACH_OK;
  memset(&this->dynamixel_cmd,   0, sizeof(this->dynamixel_cmd));
  this->dynamixel_cmd.cmd = cmd;
  for( int i = 0; i < 4; i++ )
  {
    this->dynamixel_cmd.data[i] = data[i];
    this->dynamixel_cmd.data_float[i] = data_float[i];
  }
  r = ach_put(&this->chan_dynamixel_cmd, &this->dynamixel_cmd, sizeof(this->dynamixel_cmd));

  /* Waits until return of cmd if the a block of "ture" is sent */
  if(block)
  {
    ach_flush(&this->chan_dynamixel_cmd_return);
    memset(&this->dynamixel_cmd_return,   0, sizeof(this->dynamixel_cmd_return));
    ach_status_t r = ach_get( &this->chan_dynamixel_cmd_return, &this->dynamixel_cmd_return, sizeof(this->dynamixel_cmd_return), &fs, NULL, ACH_O_WAIT );
  }
  return (int)r;
}

int DynamixelAchClient::stagePGain(int mot, double val)
{
  if( mot > DYNAMIXEL_MOTOR_MAX ) return 1;
  if( mot < DYNAMIXEL_MOTOR_MIN ) return 1;
  this->dynamixel_ref.motor_ref[mot].p_gain;
  return 0;
}

int DynamixelAchClient::stageIGain(int mot, double val)
{
  if( mot > DYNAMIXEL_MOTOR_MAX ) return 1;
  if( mot < DYNAMIXEL_MOTOR_MIN ) return 1;
  this->dynamixel_ref.motor_ref[mot].i_gain;
  return 0;
}

int DynamixelAchClient::stageDGain(int mot, double val)
{
  if( mot > DYNAMIXEL_MOTOR_MAX ) return 1;
  if( mot < DYNAMIXEL_MOTOR_MIN ) return 1;
  this->dynamixel_ref.motor_ref[mot].d_gain;
  return 0;
}

int DynamixelAchClient::stageRefPosD(int mot, double val)
{
  val = val / 180.0 * M_PI;
  return this->stageRefPos(mot, val);
}

int DynamixelAchClient::stageRefPos(int mot, double val)
{
  if(mot > DYNAMIXEL_MOTOR_MAX) return 1;
  this->dynamixel_ref.motor_ref[mot].pos = val;
  return 0;
}

int DynamixelAchClient::stageRefVel(int mot, double val)
{
  if(mot > DYNAMIXEL_MOTOR_MAX) return 1;
  this->dynamixel_ref.motor_ref[mot].speed = val;
  return 0;
}

int DynamixelAchClient::stageRefTorque(int mot, double val)
{
  if(mot > DYNAMIXEL_MOTOR_MAX) return 1;
  this->dynamixel_ref.motor_ref[mot].torque = val;
  return 0;
}

int DynamixelAchClient::setRefMode(int mode)
{
  if( mode >= MODE_COUNT ) return 1;
  if( mode < 0           ) return 1;
  this->cmd(DYNAMIXEL_CMD_MODE, (int16_t)mode);
  return 0;
}

int DynamixelAchClient::postRef()
{
  ach_status_t r = ACH_OK;
  r = ach_put(&this->chan_dynamixel_ref, &this->dynamixel_ref, sizeof(this->dynamixel_ref));
  return (int)r;
}

#endif


#define DYNAMIXEL_LOFARO_ACH 1
