#ifndef DARWIN_LOFARO_ACH 

#include "lofaro_includes_ach.h"
#include "lofaro_defines_ach.h"
#include "lofaro_defines.h"
#include "lofaro_utils.h"
#include "lofaro_types.h"
#include <cstring>
#include <chrono>
#include <functional>
#include <memory>
#include <string>



class DarwinAchClient
{
  public:
    DarwinAchClient();
    DarwinAchClient(bool do_flush);
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
    darwin_data_def_t     darwin_ref;
    darwin_data_def_t     darwin_state;
    darwin_cmd_def_t      darwin_cmd;
    darwin_cmd_def_t      darwin_cmd_return;
    darwin_cmd_vel_def_t  darwin_cmd_vel;
    double                darwin_time = 0.0;

  private:
    int ref_mode = MODE_REF;
    int stageGain(int mot, double val);
    void constructDarwinAchClient(bool do_flush);

    LofaroUtils* lu = new LofaroUtils();

    bool run_loop = false;

    /* Reference Channel */
    ach_channel_t chan_darwin_ref;  

    /* Reference Walking Channel */
    ach_channel_t chan_darwin_ref_walking;  

    /* State Feedback Channel */
    ach_channel_t chan_darwin_state;

    /* Command channel */
    ach_channel_t chan_darwin_cmd;

    /* Command Channel Return */
    ach_channel_t chan_darwin_cmd_return;

    /* Command Vel Channel */
    ach_channel_t chan_darwin_cmd_vel;

    /* Command Time Channel */
    ach_channel_t chan_darwin_time;

};
DarwinAchClient::DarwinAchClient()
{
  this->constructDarwinAchClient(false);
  return;
}

DarwinAchClient::DarwinAchClient(bool do_flush)
{
  this->constructDarwinAchClient(do_flush);
  return;
}

void DarwinAchClient::constructDarwinAchClient(bool do_flush)
{
  /* Zero Data */
  memset(&this->darwin_ref,          0, sizeof(this->darwin_ref));
  memset(&this->darwin_state,        0, sizeof(this->darwin_state));
  memset(&this->darwin_cmd,          0, sizeof(this->darwin_cmd));
  memset(&this->darwin_cmd_return,   0, sizeof(this->darwin_cmd_return));
  memset(&this->darwin_cmd_vel,      0, sizeof(this->darwin_cmd_vel));


  for( int i = 0; i <= DARWIN_MOTOR_MAX; i++ )
  {
    this->darwin_ref.motor_ref[i].pos    = DARWIN_REF_POS_0;
    this->darwin_ref.motor_ref[i].speed  = DARWIN_REF_VEL_0;
    this->darwin_ref.motor_ref[i].torque = DARWIN_REF_TOR_0;
  }


  /* Make Ach Channels */
  ach_status_t r = ACH_OK;

  /* Open Channels */
  r = ach_open(&this->chan_darwin_ref,         DARWIN_ACH_CHAN_REF,         NULL);
  r = ach_open(&this->chan_darwin_ref_walking, DARWIN_ACH_CHAN_REF_WALKING, NULL);
  r = ach_open(&this->chan_darwin_state,       DARWIN_ACH_CHAN_STATE,       NULL);
  r = ach_open(&this->chan_darwin_cmd,         DARWIN_ACH_CHAN_CMD,         NULL);
  r = ach_open(&this->chan_darwin_cmd_return,  DARWIN_ACH_CHAN_CMD_RETURN,  NULL);
  r = ach_open(&this->chan_darwin_cmd_vel,     DARWIN_ACH_CHAN_CMD_VEL,     NULL);
  r = ach_open(&this->chan_darwin_time,        DARWIN_ACH_CHAN_TIME,        NULL);

  /* Flush Ach Channels */
  if(do_flush)
  {
    r = ach_flush(&this->chan_darwin_state);
    r = ach_flush(&this->chan_darwin_cmd_return);
    r = ach_flush(&this->chan_darwin_time);
  }

  /* Do initial put on the channel to make sure the exist */
/*
  ach_put(&this->chan_darwin_ref,        &this->darwin_ref,        sizeof(this->darwin_ref));
  ach_put(&this->chan_darwin_state,      &this->darwin_state,      sizeof(this->darwin_state));
  ach_put(&this->chan_darwin_cmd,        &this->darwin_cmd,        sizeof(this->darwin_cmd));
  ach_put(&this->chan_darwin_cmd_return, &this->darwin_cmd_return, sizeof(this->darwin_cmd_return));
*/
  return;
}

double DarwinAchClient::time()
{
  return this->lu->getTime();
}

int DarwinAchClient::sleep(double val)
{
  return this->lu->sleep(val);
}

int DarwinAchClient::sleep()
{
  return this->lu->sleep();
}

int DarwinAchClient::rate(double hz)
{
  return this->lu->rate(hz);
}

int DarwinAchClient::getTime()
{
  size_t fs;
  ach_status_t r = ach_get( &this->chan_darwin_time, &this->darwin_time, sizeof(this->darwin_time), &fs, NULL, ACH_O_LAST );
  if( ( r == ACH_OK ) | ( r == ACH_MISSED_FRAME ) ) return 0;
  return 1;
}

int DarwinAchClient::getTime(bool wait)
{
  if(wait)
  {
    size_t fs;
    ach_status_t r = ach_get( &this->chan_darwin_time, &this->darwin_time, sizeof(this->darwin_time), &fs, NULL, ACH_O_WAIT );
    if( ( r == ACH_OK ) | ( r == ACH_MISSED_FRAME ) ) return 0;
    return 1;
  }
  else
  {
    return this->getTime();
  }
  return 1;
}

int DarwinAchClient::getState()
{
  size_t fs;
  ach_status_t r = ach_get( &this->chan_darwin_state, &this->darwin_state, sizeof(this->darwin_state), &fs, NULL, ACH_O_LAST );
  return (int)r;
}

int DarwinAchClient::getState(bool wait)
{
  if(wait)
  {
    size_t fs;
    ach_status_t r = ach_get( &this->chan_darwin_state, &this->darwin_state, sizeof(this->darwin_state), &fs, NULL, ACH_O_WAIT );
    return (int)r;
  }
  else
  {
    return this->getState();
  }
  return 1;
}

int DarwinAchClient::getCmdVel()
{
  size_t fs;
  ach_status_t r = ach_get( &this->chan_darwin_cmd_vel, &this->darwin_cmd_vel, sizeof(this->darwin_cmd_vel), &fs, NULL, ACH_O_LAST );
  if( ( r == ACH_OK ) | ( r == ACH_MISSED_FRAME ) ) return 0;
  return 1;
}

int DarwinAchClient::getCmdVelMode(){      return (int)this->darwin_cmd_vel.mode;  }
double DarwinAchClient::getCmdVelX(){      return this->darwin_cmd_vel.linear.x;  }
double DarwinAchClient::getCmdVelY(){      return this->darwin_cmd_vel.linear.y;  }
double DarwinAchClient::getCmdVelThetaZ(){ return this->darwin_cmd_vel.angular.z; }

int DarwinAchClient::stageCmdVelMode(int val)
{
  this->darwin_cmd_vel.mode = val;
  return 0;
}

int DarwinAchClient::stageCmdVelX(double val)
{
  this->darwin_cmd_vel.linear.x = val;
  return 0;
}

int DarwinAchClient::stageCmdVelY(double val)
{
  this->darwin_cmd_vel.linear.y = val;
  return 0;
}

int DarwinAchClient::stageCmdVelThetaZ(double val)
{
  this->darwin_cmd_vel.angular.z = val;
  return 0;
}
int DarwinAchClient::postCmdVel()
{
  ach_status_t r = ACH_OK;
  r = ach_put(&this->chan_darwin_cmd_vel, &this->darwin_cmd_vel, sizeof(this->darwin_cmd_vel));
  if (r > 0) return 1;
  return 0;
}

int DarwinAchClient::cmd(int cmd){ return this->cmd(cmd, false); }
int DarwinAchClient::cmd(int cmd, bool block)
{
  int16_t data[4];
  double  data_float[4];
  memset(&data,       0, sizeof(data));
  memset(&data_float, 0, sizeof(data_float));
  return this->cmd(cmd, data, data_float, block);
}

int DarwinAchClient::cmd(int cmd, int16_t d0){ return this->cmd(cmd, d0, false); }
int DarwinAchClient::cmd(int cmd, int16_t d0, bool block)
{
  int16_t data[4];
  double  data_float[4];
  memset(&data,       0, sizeof(data));
  memset(&data_float, 0, sizeof(data_float));
  data[0] = d0;
  return this->cmd(cmd, data, data_float, block);
}

int DarwinAchClient::cmd(int cmd, int16_t d0, int16_t d1){ return this->cmd(cmd, d0, d1, false); }
int DarwinAchClient::cmd(int cmd, int16_t d0, int16_t d1, bool block)
{
  int16_t data[4];
  double  data_float[4];
  memset(&data,       0, sizeof(data));
  memset(&data_float, 0, sizeof(data_float));
  data[0] = d0;
  data[1] = d1;
  return this->cmd(cmd, data, data_float, block);
}

int DarwinAchClient::cmd(int cmd, int16_t d0, double f0){ return this->cmd(cmd, d0, f0, false); }
int DarwinAchClient::cmd(int cmd, int16_t d0, double f0, bool block)
{
  int16_t data[4];
  double  data_float[4];
  memset(&data,       0, sizeof(data));
  memset(&data_float, 0, sizeof(data_float));
  data[0] = d0;
  data_float[0] = f0;
  return this->cmd(cmd, data, data_float, block);
}

int DarwinAchClient::cmd(int cmd, int16_t data[4]){ return this->cmd(cmd, data, false); }
int DarwinAchClient::cmd(int cmd, int16_t data[4], bool block)
{
  double  data_float[4];
  memset(&data_float, 0, sizeof(data_float));
  return this->cmd(cmd, data, data_float, block);
}

int DarwinAchClient::cmd(int cmd, int16_t data[4], double data_float[4]){ return this->cmd(cmd, data, data_float, false); }
int DarwinAchClient::cmd(int cmd, int16_t data[4], double data_float[4], bool block)
{
  if( cmd == DARWIN_CMD_MODE ) ref_mode = data[0];

  size_t fs;
  ach_status_t r = ACH_OK;
  memset(&this->darwin_cmd,   0, sizeof(this->darwin_cmd));
  this->darwin_cmd.cmd = cmd;
  for( int i = 0; i < 4; i++ )
  {
    this->darwin_cmd.data[i] = data[i];
    this->darwin_cmd.data_float[i] = data_float[i];
  }
  r = ach_put(&this->chan_darwin_cmd, &this->darwin_cmd, sizeof(this->darwin_cmd));

  /* Waits until return of cmd if the a block of "ture" is sent */
  if(block)
  {
    ach_flush(&this->chan_darwin_cmd_return);
    memset(&this->darwin_cmd_return,   0, sizeof(this->darwin_cmd_return));
    ach_status_t r = ach_get( &this->chan_darwin_cmd_return, &this->darwin_cmd_return, sizeof(this->darwin_cmd_return), &fs, NULL, ACH_O_WAIT );
  }
  return (int)r;
}

int DarwinAchClient::stagePGain(int mot, double val)
{
  if( mot > DARWIN_MOTOR_MAX ) return 1;
  if( mot < DARWIN_MOTOR_MIN ) return 1;
  this->darwin_ref.motor_ref[mot].p_gain;
  return 0;
}

int DarwinAchClient::stageIGain(int mot, double val)
{
  if( mot > DARWIN_MOTOR_MAX ) return 1;
  if( mot < DARWIN_MOTOR_MIN ) return 1;
  this->darwin_ref.motor_ref[mot].i_gain;
  return 0;
}

int DarwinAchClient::stageDGain(int mot, double val)
{
  if( mot > DARWIN_MOTOR_MAX ) return 1;
  if( mot < DARWIN_MOTOR_MIN ) return 1;
  this->darwin_ref.motor_ref[mot].d_gain;
  return 0;
}

int DarwinAchClient::stageRefPosD(int mot, double val)
{
  val = val / 180.0 * M_PI;
  return this->stageRefPos(mot, val);
}

int DarwinAchClient::stageRefPos(int mot, double val)
{
  if(mot > DARWIN_MOTOR_MAX) return 1;
  this->darwin_ref.motor_ref[mot].pos = val;
  return 0;
}

int DarwinAchClient::stageRefVel(int mot, double val)
{
  if(mot > DARWIN_MOTOR_MAX) return 1;
  this->darwin_ref.motor_ref[mot].speed = val;
  return 0;
}

int DarwinAchClient::stageRefTorque(int mot, double val)
{
  if(mot > DARWIN_MOTOR_MAX) return 1;
  this->darwin_ref.motor_ref[mot].torque = val;
  return 0;
}

int DarwinAchClient::setRefMode(int mode)
{
  if( mode >= MODE_COUNT ) return 1;
  if( mode < 0           ) return 1;
  this->cmd(DARWIN_CMD_MODE, (int16_t)mode);
  return 0;
}

int DarwinAchClient::postRef()
{
  ach_status_t r = ACH_OK;
  if( (ref_mode == MODE_WALKING) | (ref_mode == MODE_WALKING_LOWER_ONLY) )
  {
    r = ach_put(&this->chan_darwin_ref_walking, &this->darwin_ref, sizeof(this->darwin_ref));
  }
  else
  {
    r = ach_put(&this->chan_darwin_ref, &this->darwin_ref, sizeof(this->darwin_ref));
  }
  return (int)r;
}

#endif


#define DARWIN_LOFARO_ACH 1
