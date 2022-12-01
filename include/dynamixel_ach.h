#define DYNAMIXEL_LOFARO_ACH 1
#define DYNAMIXEL_LOFARO_DYN 1

#include "lofaro_dynamixel.h"
#include "lofaro_dynamixel_includes_ach.h"
#include "lofaro_dynamixel_defines_ach.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>


class DynamixelAch
{
  public:
    DynamixelAch();
    int cmd(int cmd);
    int cmd(int cmd, bool block);
    int loop();
    int loop(double hz);
    int loop(double hz, int mode_state);
    int loop(double hz, int mode_state, int mode_ref);
    int sleep(double val);
    int setDebug(bool val);
    int open();

    /* Update Methods */
    int getState();

    /* Data types */
    dynamixel_data_def_t     dynamixel_ref;
    dynamixel_data_def_t     dynamixel_state;
    dynamixel_cmd_def_t      dynamixel_cmd;
    dynamixel_cmd_def_t      dynamixel_cmd_return;
    double                   dynamixel_time = 0.0;

  private:
    int main_loop();
    int main_loop(int mode_state);
    int main_loop(int mode_state, int mode_ref);
    int do_ref(int mode);
    int do_state(int mode);
    int do_cmd(int mode);
    int do_gain();
    int do_debug();
    int do_time();
    int do_save_previous_state();
    int m_REF_MODE = MODE_REF;
    int id_clear();
    int id_add(int the_id);

    int the_mode_state = 0;
    int the_mode_ref   = 0;

    /* Led State */
    uint8_t led_mode = LED_MODE_0;

    DynamixelLofaro* dl = new DynamixelLofaro();

    /* Data types */
    dynamixel_data_def_t dynamixel_ref_0;

    bool run_loop = false;
    bool debug_flag = false;
    /* Reference Channel */
    ach_channel_t chan_dynamixel_ref;  

    /* State Feedback Channel */
    ach_channel_t chan_dynamixel_state;

    /* Command channel */
    ach_channel_t chan_dynamixel_cmd;

    /* Command Channel Return */
    ach_channel_t chan_dynamixel_cmd_return;

    /* Command Time Channel */
    ach_channel_t chan_dynamixel_time;

    /* index vector */
    std::vector< int16_t > dyn_id;
};

DynamixelAch::DynamixelAch()
{
  /* Zero Data */
  memset(&this->dynamixel_ref,          0, sizeof(this->dynamixel_ref));
  memset(&this->dynamixel_ref_0,        0, sizeof(this->dynamixel_ref_0));
  memset(&this->dynamixel_state,        0, sizeof(this->dynamixel_state));
  memset(&this->dynamixel_cmd,          0, sizeof(this->dynamixel_cmd));
  memset(&this->dynamixel_cmd_return,   0, sizeof(this->dynamixel_cmd_return));

  for( int i = 0; i <= DYNAMIXEL_MOTOR_MAX; i++ )
  {
    this->dynamixel_ref.motor_ref[i].pos            = DYNAMIXEL_REF_POS_0;
    this->dynamixel_ref.motor_ref[i].speed          = DYNAMIXEL_REF_VEL_0;
    this->dynamixel_ref.motor_ref[i].torque         = DYNAMIXEL_REF_TOR_0;
  }

  ach_status_t r = ACH_OK;
  /* Create Channels if they need to be created */
/*
  r = ach_create(DYNAMIXEL_ACH_CHAN_REF,        10, 2000, NULL );
  r = ach_create(DYNAMIXEL_ACH_CHAN_STATE,      10, 2000, NULL );
  r = ach_create(DYNAMIXEL_ACH_CHAN_CMD,        10, 2000, NULL );
  r = ach_create(DYNAMIXEL_ACH_CHAN_CMD_RETURN, 10, 2000, NULL );
*/

  /* Open Channels */
  r = ach_open(&this->chan_dynamixel_ref,         DYNAMIXEL_ACH_CHAN_REF,         NULL);
  r = ach_open(&this->chan_dynamixel_state,       DYNAMIXEL_ACH_CHAN_STATE,       NULL);
  r = ach_open(&this->chan_dynamixel_cmd,         DYNAMIXEL_ACH_CHAN_CMD,         NULL);
  r = ach_open(&this->chan_dynamixel_cmd_return,  DYNAMIXEL_ACH_CHAN_CMD_RETURN,  NULL);
  r = ach_open(&this->chan_dynamixel_time,        DYNAMIXEL_ACH_CHAN_TIME,        NULL);

  /* Flush all channels */
  ach_flush(&this->chan_dynamixel_ref);
  ach_flush(&this->chan_dynamixel_state);
  ach_flush(&this->chan_dynamixel_cmd);
  ach_flush(&this->chan_dynamixel_cmd_return);
  ach_flush(&this->chan_dynamixel_time);

  /* Do initial put on the channel to make sure the exist */
  ach_put(&this->chan_dynamixel_ref,         &this->dynamixel_ref,         sizeof(this->dynamixel_ref));
  ach_put(&this->chan_dynamixel_state,       &this->dynamixel_state,       sizeof(this->dynamixel_state));
  ach_put(&this->chan_dynamixel_cmd,         &this->dynamixel_cmd,         sizeof(this->dynamixel_cmd));
  ach_put(&this->chan_dynamixel_cmd_return,  &this->dynamixel_cmd_return,  sizeof(this->dynamixel_cmd_return));

  this->dynamixel_time = this->dl->time();
  ach_put(&this->chan_dynamixel_time,        &this->dynamixel_time,        sizeof(this->dynamixel_time));
  return;
}

int DynamixelAch::id_clear()
{
  this->dyn_id.clear();
  return 0;
}
int DynamixelAch::id_add(int the_id)
{
  this->dyn_id.push_back(the_id);
  return 0;
}
int DynamixelAch::do_time()
{
  this->dynamixel_time = this->dl->time();
  ach_put(&this->chan_dynamixel_time,  &this->dynamixel_time,  sizeof(this->dynamixel_time));
  return 0;
}

int DynamixelAch::open()
{
  this->dl->setup("/dev/ttyUSB0", true);
  this->dl->sleep(2.0);

  uint8_t b = 1;
  b = b << this->led_mode;
  this->dl->setLed(b);   

  return 0;
}

int DynamixelAch::sleep(double val)
{
  return this->dl->sleep(val);
}

int DynamixelAch::getState()
{
  size_t fs;
  ach_status_t r = ach_get( &this->chan_dynamixel_state, &this->dynamixel_state, sizeof(this->dynamixel_state), &fs, NULL, ACH_O_LAST );
  return (int)r;
}

int DynamixelAch::do_cmd(int mode)
{
  size_t fs;
  /* Get the latest cmd channel */
  ach_status_t r = ach_get( &this->chan_dynamixel_cmd, &this->dynamixel_cmd, sizeof(this->dynamixel_cmd), &fs, NULL, ACH_O_LAST );
//  if(ACH_OK != r) {fprintf(stderr, "Ref r = %s\n",ach_result_to_string(r));}

  bool do_return = false;
  if( ( r == ACH_OK ) | ( r == ACH_MISSED_FRAME ) )
  {
    switch (this->dynamixel_cmd.cmd)
    {
      case DYNAMIXEL_CMD_ID_ADD:
      {
        this->id_add(dynamixel_cmd.data[0]);
        do_return = true;
        break;
      } 
      case DYNAMIXEL_CMD_ID_RESET:
      {
        this->id_clear();
        do_return = true;
        break;
      }
      case DYNAMIXEL_CMD_CLOSE:
      {
        this->dl->close();
        this->dl->sleep(2.0);
        this->run_loop = false;
        do_return = true;
        break;
      }
      case DYNAMIXEL_CMD_OPEN:
      {
        this->dl->setup("/dev/ttyUSB0", true);
        this->dl->sleep(2.0);
        this->run_loop = true;
        do_return = true;
        break;
      }
      case DYNAMIXEL_CMD_ON:
      {
        this->dl->setup("/dev/ttyUSB0", true);
        this->dl->sleep(1.0);
        this->dl->on();
        this->dl->sleep(1.0);
        this->run_loop = true;
        do_return = true;
        break;
      }
      case DYNAMIXEL_CMD_ON_MOTOR:
      {
        this->dl->setup("/dev/ttyUSB0", true);
        this->dl->sleep(1.0);
        this->dl->on(dynamixel_cmd.data[0]);
        this->dl->sleep(1.0);
        this->run_loop = true;
        do_return = true;
        break;
      }
      case DYNAMIXEL_CMD_OFF:
      {
        this->run_loop = false;
        this->dl->off();
        this->dl->stop();
        do_return = true;
        break;
      }
      case DYNAMIXEL_CMD_LOOP_MODE:
      {
        int d0 = this->dynamixel_cmd.data[0];
        if( d0 == HZ_STATE_MOTORS )
        { 
          this->the_mode_state = d0;
          //this->dl->rate(200.0);
          printf("Set HZ_STATE_MOTOR\n");
          do_return = true; 
        }
        break;
      }
      case DYNAMIXEL_CMD_MODE:
      {
        int d0 = this->dynamixel_cmd.data[0];
        if( d0 == MODE_REF )
        { 
          m_REF_MODE = d0;                   
          printf("Set MODE_REF\n");
          do_return = true; 
        }
        else
        {
          do_return = true; 
        }
        break;
      }
      default:
          do_return = true; 
        break;
    }
  }

  if(do_return)
  {
    memset(&this->dynamixel_cmd_return,   0, sizeof(this->dynamixel_cmd_return));
    this->dynamixel_cmd_return.cmd = DYNAMIXEL_CMD_OK;
    ach_put(&this->chan_dynamixel_cmd_return, &this->dynamixel_cmd_return, sizeof(this->dynamixel_cmd_return));
    return 0;
  }
  else
  {
    memset(&this->dynamixel_cmd_return,   0, sizeof(this->dynamixel_cmd_return));
    this->dynamixel_cmd_return.cmd = DYNAMIXEL_CMD_FAIL;
    ach_put(&this->chan_dynamixel_cmd_return, &this->dynamixel_cmd_return, sizeof(this->dynamixel_cmd_return));
    return 1;
  }

  return 0;
}


int DynamixelAch::cmd(int cmd)
{
    return this->cmd(cmd,false);
}
int DynamixelAch::cmd(int cmd, bool block)
{
  size_t fs;
  ach_status_t r = ACH_OK;
  memset(&this->dynamixel_cmd,   0, sizeof(this->dynamixel_cmd));
  this->dynamixel_cmd.cmd = cmd;
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


int DynamixelAch::loop()
{
  return this->loop(HZ_RATE_DEFAULT);
}

int DynamixelAch::loop(double hz)
{
  return this->loop(hz, HZ_STATE_DEFAULT);
}

int DynamixelAch::loop(double hz, int mode_state)
{
  return this->loop(hz, mode_state, HZ_REF_DEFAULT);
}

int DynamixelAch::loop(double hz, int mode_state, int mode_ref)
{
  int ref = 0;

  this->dl->rate(hz);
  
  bool do_loop = true;
  this->the_mode_state = mode_state;
  this->the_mode_ref   = mode_ref;
  while(do_loop)
  { 
    this->do_time();

    this->main_loop(mode_state, mode_ref);
    //if(this->run_loop) this->main_loop(mode_state, mode_ref);

    this->do_debug();
    this->do_save_previous_state();

/* No button */
//    this->do_button();

    /* Update the Mode if needed */
    mode_state = this->the_mode_state;
    mode_ref   = this->the_mode_ref;

    this->dl->sleep();
  }

  if( ref > 0 ) ref = 1;
  return ref;
}


/*
int DynamixelAch::button_reset()
{
  printf("Button: Resetting System\n");
  this->run_loop = false;
  this->dl->off();
  std::system("dynamixel-ach stop walking");
  std::system("dynamixel-ach stop server no_wait");
  return 0;
}

int DynamixelAch::button_off()
{
  printf("Button: Turning off power\n");
  this->run_loop = false;
  this->dl->off();
  return 0;
}

int DynamixelAch::button_on()
{
  printf("Button: Turning on power\n");
  this->dl->on();
  this->dl->sleep(1.0);
  this->run_loop = true;
  return 0;
}
*/

int DynamixelAch::do_save_previous_state()
{
  this->dynamixel_ref_0 = this->dynamixel_ref;
  return 0;
}


int DynamixelAch::setDebug(bool val)
{
  debug_flag = val;
  return 0;
}

int DynamixelAch::do_debug()
{
  int ret = 0;
  if(this->debug_flag)
  {
    for( int i = DYNAMIXEL_MOTOR_MIN; i <= DYNAMIXEL_MOTOR_MAX; i++ )
    {
      double p0 = this->dynamixel_ref_0.motor_ref[i].pos;
      double p1 = this->dynamixel_ref.motor_ref[i].pos;
      double dt = abs(p0-p1);
      if( dt > 0.1 )
      {
        printf("Error: (dt = %f) (mot = %d)\n",dt,i);
        ret += 1;
      }
    }
  }
  if( ret > 0 ) ret = 1;
  else ret = 0;
  return ret;
}

int ft_i = 0;
int upper_i = 0;

int DynamixelAch::main_loop()
{
  return this->main_loop(HZ_STATE_DEFAULT);
}

int DynamixelAch::main_loop(int mode_state)
{
  if (mode_state >= HZ_MODE_COUNT) return 1;
  return this->main_loop(mode_state, HZ_REF_DEFAULT);
}

int DynamixelAch::main_loop(int mode_state, int mode_ref)
{
  int ret = 0;
  ret += this->do_cmd(0);
  if(this->run_loop)
  {
    ret += this->do_gain();
    ret += this->do_ref(mode_ref);
    ret += this->do_state(mode_state);
  }
  else ret += 1;

  if( ret > 0 ) ret = 1;
  return ret;
}

int DynamixelAch::do_gain()
{
  int ret = 0;
  for(int i = DYNAMIXEL_MOTOR_MIN; i <= DYNAMIXEL_MOTOR_MAX; i++)
  {
    if( (uint8_t)this->dynamixel_ref.motor_ref[i].p_gain != (uint8_t)this->dynamixel_ref_0.motor_ref[i].p_gain ) ret += this->dl->setPGain(i, this->dynamixel_ref.motor_ref[i].p_gain);
    if( (uint8_t)this->dynamixel_ref.motor_ref[i].i_gain != (uint8_t)this->dynamixel_ref_0.motor_ref[i].i_gain ) ret += this->dl->setPGain(i, this->dynamixel_ref.motor_ref[i].i_gain);
    if( (uint8_t)this->dynamixel_ref.motor_ref[i].d_gain != (uint8_t)this->dynamixel_ref_0.motor_ref[i].d_gain ) ret += this->dl->setPGain(i, this->dynamixel_ref.motor_ref[i].d_gain);
  }
  if( ret > 0 ) ret = 1;
  return ret;
}


int DynamixelAch::do_ref(int mode)
{
  size_t fs;
  /* Get the latest reference channel */
  ach_status_t r = ACH_OK;
  r = ach_get( &this->chan_dynamixel_ref,         &this->dynamixel_ref,         sizeof(this->dynamixel_ref),         &fs, NULL, ACH_O_LAST );

  int ret = 0;
  /* Set Reference Here */
  int move_mode = m_REF_MODE;
  //int move_mode = this->dynamixel_ref.mode;

  switch (move_mode)
  {
    for (int16_t& i: this->dyn_id)
    {
       if( (i <= DYNAMIXEL_MOTOR_MAX) & ( i >= 0) )
       {
         this->dl->dynamixel_data.motor_ref[i].pos    = this->dynamixel_ref.motor_ref[i].pos;
         this->dl->dynamixel_data.motor_ref[i].speed  = this->dynamixel_ref.motor_ref[i].speed;
         this->dl->dynamixel_data.motor_ref[i].torque = this->dynamixel_ref.motor_ref[i].torque;
         ret += this->dl->stageMotor(i);
       }
    }
    ret += this->dl->putMotor(); 
  }

  if( ret > 1 ) ret = 1;
  return ret;
}


int DynamixelAch::do_state(int mode)
{
  size_t fs;
  int ret = 0;
  /* Get State */
  switch (mode)
  {
    case HZ_STATE_MOTORS:
    {
      for (int16_t& i: this->dyn_id)
      {
        ret += this->dl->getMotor(i);
      }
      break;
    }
    default:
    {
      ret += 1;
      break;  
    }
  }

  this->dynamixel_state = this->dl->dynamixel_data;

  /* Put the data back on the Ach Channel */
  ach_put(&this->chan_dynamixel_state, &this->dynamixel_state, sizeof(this->dynamixel_state));

  if( ret > 0 ) ret = 1;
  return ret;
}
