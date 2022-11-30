#define DARWIN_LOFARO_ACH 1
#define DARWIN_LOFARO_DYN 1

#include "lofaro_darwin.h"
#include "lofaro_includes_ach.h"
#include "lofaro_defines_ach.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>



class DarwinAch
{
  public:
    DarwinAch();
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
    darwin_data_def_t     darwin_ref;
    darwin_data_def_t     darwin_ref_walking;
    darwin_data_def_t     darwin_state;
    darwin_cmd_def_t      darwin_cmd;
    darwin_cmd_def_t      darwin_cmd_return;
    darwin_cmd_vel_def_t  darwin_cmd_vel;
    double                darwin_time = 0.0;

  private:
    int main_loop();
    int main_loop(int mode_state);
    int main_loop(int mode_state, int mode_ref);
    int do_ref(int mode);
    int do_state(int mode);
    int do_cmd(int mode);
    int do_gain();
    int do_button();
    int do_debug();
    int do_time();
    int do_save_previous_state();
    int m_REF_MODE = MODE_REF;

    /* Previous Button States */
    int button_mode_0  = 0;
    int button_start_0 = 0;

    int button_on();
    int button_off();
    int button_walking();
    int button_reset();

    int the_mode_state = 0;
    int the_mode_ref   = 0;

    /* Led State */
    uint8_t led_mode = LED_MODE_0;

    DarwinLofaro* dl = new DarwinLofaro();

    /* Data types */
    darwin_data_def_t darwin_ref_0;

    bool run_loop = false;
    bool debug_flag = false;
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

DarwinAch::DarwinAch()
{
  /* Zero Data */
  memset(&this->darwin_ref,          0, sizeof(this->darwin_ref));
  memset(&this->darwin_ref_walking,  0, sizeof(this->darwin_ref_walking));
  memset(&this->darwin_ref_0,        0, sizeof(this->darwin_ref_0));
  memset(&this->darwin_state,        0, sizeof(this->darwin_state));
  memset(&this->darwin_cmd,          0, sizeof(this->darwin_cmd));
  memset(&this->darwin_cmd_return,   0, sizeof(this->darwin_cmd_return));
  memset(&this->darwin_cmd_vel,      0, sizeof(this->darwin_cmd_vel));

  for( int i = 0; i <= DARWIN_MOTOR_MAX; i++ )
  {
    this->darwin_ref.motor_ref[i].pos            = DARWIN_REF_POS_0;
    this->darwin_ref.motor_ref[i].speed          = DARWIN_REF_VEL_0;
    this->darwin_ref.motor_ref[i].torque         = DARWIN_REF_TOR_0;
    this->darwin_ref_walking.motor_ref[i].pos    = DARWIN_REF_POS_0;
    this->darwin_ref_walking.motor_ref[i].speed  = DARWIN_REF_VEL_0;
    this->darwin_ref_walking.motor_ref[i].torque = DARWIN_REF_TOR_0;
  }



  /* Make Ach Channels */
/*
  printf("ACH_OK = %d\n", ACH_OK);
  printf("ACH_EEXIST = %d\n", ACH_EEXIST);
  printf("ACH_ENOENT = %d\n", ACH_ENOENT);
  printf("r = %d\n", r);
*/

  ach_status_t r = ACH_OK;
  /* Create Channels if they need to be created */
/*
  r = ach_create(DARWIN_ACH_CHAN_REF,        10, 2000, NULL );
  r = ach_create(DARWIN_ACH_CHAN_STATE,      10, 2000, NULL );
  r = ach_create(DARWIN_ACH_CHAN_CMD,        10, 2000, NULL );
  r = ach_create(DARWIN_ACH_CHAN_CMD_RETURN, 10, 2000, NULL );
*/

  /* Open Channels */
  r = ach_open(&this->chan_darwin_ref,         DARWIN_ACH_CHAN_REF,         NULL);
  r = ach_open(&this->chan_darwin_ref_walking, DARWIN_ACH_CHAN_REF_WALKING, NULL);
  r = ach_open(&this->chan_darwin_state,       DARWIN_ACH_CHAN_STATE,       NULL);
  r = ach_open(&this->chan_darwin_cmd,         DARWIN_ACH_CHAN_CMD,         NULL);
  r = ach_open(&this->chan_darwin_cmd_return,  DARWIN_ACH_CHAN_CMD_RETURN,  NULL);
  r = ach_open(&this->chan_darwin_cmd_vel,     DARWIN_ACH_CHAN_CMD_VEL,     NULL);
  r = ach_open(&this->chan_darwin_time,        DARWIN_ACH_CHAN_TIME,        NULL);

  /* Flush all channels */
  ach_flush(&this->chan_darwin_ref);
  ach_flush(&this->chan_darwin_ref_walking);
  ach_flush(&this->chan_darwin_state);
  ach_flush(&this->chan_darwin_cmd);
  ach_flush(&this->chan_darwin_cmd_return);
  ach_flush(&this->chan_darwin_cmd_vel);
  ach_flush(&this->chan_darwin_time);

  /* Do initial put on the channel to make sure the exist */
  ach_put(&this->chan_darwin_ref,         &this->darwin_ref,         sizeof(this->darwin_ref));
  ach_put(&this->chan_darwin_ref_walking, &this->darwin_ref_walking, sizeof(this->darwin_ref_walking));
  ach_put(&this->chan_darwin_state,       &this->darwin_state,       sizeof(this->darwin_state));
  ach_put(&this->chan_darwin_cmd,         &this->darwin_cmd,         sizeof(this->darwin_cmd));
  ach_put(&this->chan_darwin_cmd_return,  &this->darwin_cmd_return,  sizeof(this->darwin_cmd_return));
  ach_put(&this->chan_darwin_cmd_vel,  &this->darwin_cmd_vel,  sizeof(this->darwin_cmd_vel));

  this->darwin_time = this->dl->time();
  ach_put(&this->chan_darwin_time,  &this->darwin_time,  sizeof(this->darwin_time));
  return;
}

int DarwinAch::do_time()
{
  this->darwin_time = this->dl->time();
  ach_put(&this->chan_darwin_time,  &this->darwin_time,  sizeof(this->darwin_time));
  return 0;
}

int DarwinAch::open()
{
  this->dl->setup("/dev/ttyUSB0", true);
  this->dl->sleep(2.0);

  uint8_t b = 1;
  b = b << this->led_mode;
  this->dl->setLed(b);   

  return 0;
}

int DarwinAch::sleep(double val)
{
  return this->dl->sleep(val);
}

int DarwinAch::getState()
{
  size_t fs;
  ach_status_t r = ach_get( &this->chan_darwin_state, &this->darwin_state, sizeof(this->darwin_state), &fs, NULL, ACH_O_LAST );
  return (int)r;
}

int DarwinAch::do_cmd(int mode)
{
  size_t fs;
  /* Get the latest cmd channel */
  ach_status_t r = ach_get( &this->chan_darwin_cmd, &this->darwin_cmd, sizeof(this->darwin_cmd), &fs, NULL, ACH_O_LAST );
//  if(ACH_OK != r) {fprintf(stderr, "Ref r = %s\n",ach_result_to_string(r));}

  bool do_return = false;
  if( ( r == ACH_OK ) | ( r == ACH_MISSED_FRAME ) )
  {
    switch (this->darwin_cmd.cmd)
    {
      case DARWIN_CMD_CLOSE:
      {
        this->dl->close();
        this->dl->sleep(2.0);
        this->run_loop = false;
        do_return = true;
        break;
      }
      case DARWIN_CMD_OPEN:
      {
        this->dl->setup("/dev/ttyUSB0", true);
        this->dl->sleep(2.0);
        this->run_loop = true;
        do_return = true;
        break;
      }
      case DARWIN_CMD_ON:
      {
        this->dl->setup("/dev/ttyUSB0", true);
        this->dl->sleep(1.0);
        this->dl->on();
        this->dl->sleep(1.0);
        this->run_loop = true;
        do_return = true;
        break;
      }
      case DARWIN_CMD_ON_MOTOR:
      {
        this->dl->setup("/dev/ttyUSB0", true);
        this->dl->sleep(1.0);
        this->dl->on(darwin_cmd.data[0]);
        this->dl->sleep(1.0);
        this->run_loop = true;
        do_return = true;
        break;
      }
      case DARWIN_CMD_OFF:
      {
        this->run_loop = false;
        this->dl->off();
        this->dl->stop();
        do_return = true;
        break;
      }
      case DARWIN_CMD_LOOP_MODE:
      {
        int d0 = this->darwin_cmd.data[0];
        if( d0 == HZ_STATE_50_IMU_MOTOR_FT )
        { 
          this->the_mode_state = d0;
          this->dl->rate(50.0);
          printf("Set HZ_STATE_50_IMU_MOTOR_FT\n");
          do_return = true; 
        }
        else if( d0 == HZ_STATE_50_IMU )
        { 
          this->the_mode_state = d0;
          this->dl->rate(50.0);
          printf("Set HZ_STATE_50_IMU\n");
          do_return = true; 
        }
        else if( d0 == HZ_STATE_125_IMU )
        { 
          this->the_mode_state = d0;
          this->dl->rate(125.0);
          printf("Set HZ_STATE_125_IMU\n");
          do_return = true; 
        }
        else if( d0 == HZ_STATE_100_IMU_FT_SLOW )
        { 
          this->the_mode_state = d0;
          this->dl->rate(100.0);
          printf("Set HZ_STATE_100_IMU_FT_SLOW\n");
          do_return = true; 
        }
        else if( d0 == HZ_STATE_100_IMU_MOTORS_SLOW )
        { 
          this->the_mode_state = d0;
          this->dl->rate(100.0);
          printf("Set HZ_STATE_100_IMU_MOTORS_SLOW\n");
          do_return = true; 
        }
        else if( d0 == HZ_STATE_DEFAULT )
        { 
          this->the_mode_state = d0;
          this->dl->rate(100.0);
          printf("Set HZ_STATE_DEFAULT\n");
          do_return = true; 
        }
        else if( d0 == HZ_REF_SLOW_TOP )
        { 
          this->the_mode_ref = d0;
          printf("Set HZ_REF_SLOW_TOP\n");
          do_return = true; 
        }
        else if( d0 == HZ_REF_NORMAL )
        { 
          this->the_mode_ref = d0;
          printf("Set HZ_REF_NORMAL\n");
          do_return = true; 
        }
        else if( d0 == HZ_REF_DEFAULT )
        { 
          this->the_mode_ref = d0;
          printf("Set HZ_REF_DEFAULT\n");
          do_return = true; 
        }
        break;
      }
      case DARWIN_CMD_MODE:
      {
        int d0 = this->darwin_cmd.data[0];
        if( d0 == MODE_REF )
        { 
          m_REF_MODE = d0;                   
          printf("Set MODE_REF\n");
          do_return = true; 
        }
        else if( d0 == MODE_WALKING )
        { 
          m_REF_MODE = d0;            
          printf("Set MODE_WALKING\n");
          do_return = true; 
        }
        else if( d0 == MODE_WALKING_LOWER_ONLY )
        { 
          m_REF_MODE = d0; 
          printf("Set MODE_WALKING_LOWER_ONLY\n");
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
    memset(&this->darwin_cmd_return,   0, sizeof(this->darwin_cmd_return));
    this->darwin_cmd_return.cmd = DARWIN_CMD_OK;
    ach_put(&this->chan_darwin_cmd_return, &this->darwin_cmd_return, sizeof(this->darwin_cmd_return));
    return 0;
  }
  else
  {
    memset(&this->darwin_cmd_return,   0, sizeof(this->darwin_cmd_return));
    this->darwin_cmd_return.cmd = DARWIN_CMD_FAIL;
    ach_put(&this->chan_darwin_cmd_return, &this->darwin_cmd_return, sizeof(this->darwin_cmd_return));
    return 1;
  }

  return 0;
}


int DarwinAch::cmd(int cmd)
{
    return this->cmd(cmd,false);
}
int DarwinAch::cmd(int cmd, bool block)
{
  size_t fs;
  ach_status_t r = ACH_OK;
  memset(&this->darwin_cmd,   0, sizeof(this->darwin_cmd));
  this->darwin_cmd.cmd = cmd;
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


int DarwinAch::loop()
{
  return this->loop(HZ_RATE_DEFAULT);
}

int DarwinAch::loop(double hz)
{
  return this->loop(hz, HZ_STATE_DEFAULT);
}

int DarwinAch::loop(double hz, int mode_state)
{
  return this->loop(hz, mode_state, HZ_REF_DEFAULT);
}

int DarwinAch::loop(double hz, int mode_state, int mode_ref)
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

    this->do_button();

    /* Update the Mode if needed */
    mode_state = this->the_mode_state;
    mode_ref   = this->the_mode_ref;

    this->dl->sleep();
  }

  if( ref > 0 ) ref = 1;
  return ref;
}

int DarwinAch::do_button()
{
  try
  {
    uint8_t buff = this->dl->getButton();
    int button_mode  = this->dl->getButton(DARWIN_BUTTON_MODE,  buff);
    int button_start = this->dl->getButton(DARWIN_BUTTON_START, buff);

    /* Debounce */
    int button_mode_pressed  = 0;
    int button_start_pressed = 0;
    if( (button_mode  == 0) & (button_mode_0  == 1) ) button_mode_pressed  = 1;
    if( (button_start == 0) & (button_start_0 == 1) ) button_start_pressed = 1;

    /* Progress Buttons */
    if(button_mode_pressed == 1)
    {
      uint8_t the_mode = 0;
      if     (this->led_mode == LED_MODE_0) this->led_mode = LED_MODE_1;
      else if(this->led_mode == LED_MODE_1) this->led_mode = LED_MODE_2;
      else if(this->led_mode == LED_MODE_2) this->led_mode = LED_MODE_3;
      else if(this->led_mode == LED_MODE_3) this->led_mode = LED_MODE_0;
      else                                  this->led_mode = LED_MODE_0;
      the_mode = this->led_mode;
      uint8_t b = 1;
      b = b << the_mode;
      if(the_mode == LED_MODE_3) b = 7;
      this->dl->setLed(b);   
    }

    if(button_start_pressed)
    {
      if     (this->led_mode == LED_MODE_0)  this->button_on();
      else if(this->led_mode == LED_MODE_1)  this->button_walking();
      else if(this->led_mode == LED_MODE_2)  this->button_off();
      else if(this->led_mode == LED_MODE_3)  this->button_reset();
      else return 1;
    }

    this->button_mode_0  = button_mode;
    this->button_start_0 = button_start;
  }
  catch(...)
  {
    return 1;
  }

  return 0;
}

int DarwinAch::button_reset()
{
  printf("Button: Resetting System\n");
  this->run_loop = false;
  this->dl->off();
  std::system("darwin-ach stop walking");
  std::system("darwin-ach stop server no_wait");
  return 0;
}

int DarwinAch::button_walking()
{
  printf("Button: Turning on Walking\n");
  std::system("darwin-ach start walking");
  return 0;
}

int DarwinAch::button_off()
{
  printf("Button: Turning off power\n");
  this->run_loop = false;
  this->dl->off();
  return 0;
}

int DarwinAch::button_on()
{
  printf("Button: Turning on power\n");
  this->dl->on();
  this->dl->sleep(1.0);
  this->run_loop = true;
  return 0;
}

int DarwinAch::do_save_previous_state()
{
  this->darwin_ref_0 = this->darwin_ref;
  return 0;
}


int DarwinAch::setDebug(bool val)
{
  debug_flag = val;
  return 0;
}

int DarwinAch::do_debug()
{
  int ret = 0;
  if(this->debug_flag)
  {
    for( int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++ )
    {
      double p0 = this->darwin_ref_0.motor_ref[i].pos;
      double p1 = this->darwin_ref.motor_ref[i].pos;
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

int DarwinAch::main_loop()
{
  return this->main_loop(HZ_STATE_DEFAULT);
}

int DarwinAch::main_loop(int mode_state)
{
  if (mode_state >= DARWIN_HZ_MODE_COUNT) return 1;
  return this->main_loop(mode_state, HZ_REF_DEFAULT);
}

int DarwinAch::main_loop(int mode_state, int mode_ref)
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

int DarwinAch::do_gain()
{
  int ret = 0;
  for(int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++)
  {
    if( (uint8_t)this->darwin_ref.motor_ref[i].p_gain != (uint8_t)this->darwin_ref_0.motor_ref[i].p_gain ) ret += this->dl->setPGain(i, this->darwin_ref.motor_ref[i].p_gain);
    if( (uint8_t)this->darwin_ref.motor_ref[i].i_gain != (uint8_t)this->darwin_ref_0.motor_ref[i].i_gain ) ret += this->dl->setPGain(i, this->darwin_ref.motor_ref[i].i_gain);
    if( (uint8_t)this->darwin_ref.motor_ref[i].d_gain != (uint8_t)this->darwin_ref_0.motor_ref[i].d_gain ) ret += this->dl->setPGain(i, this->darwin_ref.motor_ref[i].d_gain);
  }
  if( ret > 0 ) ret = 1;
  return ret;
}


int DarwinAch::do_ref(int mode)
{
  size_t fs;
  /* Get the latest reference channel */
  ach_status_t r = ACH_OK;
  r = ach_get( &this->chan_darwin_ref,         &this->darwin_ref,         sizeof(this->darwin_ref),         &fs, NULL, ACH_O_LAST );
  r = ach_get( &this->chan_darwin_ref_walking, &this->darwin_ref_walking, sizeof(this->darwin_ref_walking), &fs, NULL, ACH_O_LAST );

  int ret = 0;
  /* Set Reference Here */
  int move_mode = m_REF_MODE;
  //int move_mode = this->darwin_ref.mode;

  switch (move_mode)
  {
    case MODE_REF:
    {
//      printf("Mode: MODE_REF\n");
      for( int i = 0; i <= DARWIN_MOTOR_MAX; i++ )
      {
        this->dl->darwin_data.motor_ref[i].pos    = this->darwin_ref.motor_ref[i].pos;
        this->dl->darwin_data.motor_ref[i].speed  = this->darwin_ref.motor_ref[i].speed;
       this->dl->darwin_data.motor_ref[i].torque = this->darwin_ref.motor_ref[i].torque;
      }
      break;
    }

    case MODE_WALKING:
    {
//      printf("Mode: MODE_WALKING\n");
      for( int i = 0; i <= DARWIN_MOTOR_MAX; i++ )
      {
        this->dl->darwin_data.motor_ref[i].pos    = this->darwin_ref_walking.motor_ref[i].pos;
        this->dl->darwin_data.motor_ref[i].speed  = this->darwin_ref_walking.motor_ref[i].speed;
        this->dl->darwin_data.motor_ref[i].torque = this->darwin_ref_walking.motor_ref[i].torque;
      }
      break;
    }

    case MODE_WALKING_LOWER_ONLY_STIFF_HIP_PITCH:
    {
//      printf("Mode: MODE_WALKING_LOWER_ONLY\n");
      for( int i = 0; i <= DARWIN_MOTOR_MAX; i++ )
      {
        if( (i >= DARWIN_MOTOR_MIN_LOWER) & (i <= DARWIN_MOTOR_MAX_LOWER) )
        {
          this->dl->darwin_data.motor_ref[i].pos    = this->darwin_ref_walking.motor_ref[i].pos;
          this->dl->darwin_data.motor_ref[i].speed  = this->darwin_ref_walking.motor_ref[i].speed;
          this->dl->darwin_data.motor_ref[i].torque = this->darwin_ref_walking.motor_ref[i].torque;
        }
        else
        {
          this->dl->darwin_data.motor_ref[i].pos    = this->darwin_ref.motor_ref[i].pos;
          this->dl->darwin_data.motor_ref[i].speed  = this->darwin_ref.motor_ref[i].speed;
          this->dl->darwin_data.motor_ref[i].torque = this->darwin_ref.motor_ref[i].torque;
        }
      }
      break;
    }
    case MODE_WALKING_LOWER_ONLY:
    {
//      printf("Mode: MODE_WALKING_LOWER_ONLY\n");
      for( int i = 0; i <= DARWIN_MOTOR_MAX; i++ )
      {
        if( (i >= DARWIN_MOTOR_MIN_LOWER) & (i <= DARWIN_MOTOR_MAX_LOWER) )
        {
          if( (i == RHP) )
          {
            this->dl->darwin_data.motor_ref[i].pos    = this->darwin_ref_walking.motor_ref[i].pos
                                                        +
                                                        this->darwin_ref.motor_ref[RHP].pos;
          }
          else if( (i == LHP) )
          {
            this->dl->darwin_data.motor_ref[i].pos    = this->darwin_ref_walking.motor_ref[i].pos
                                                        -
                                                        this->darwin_ref.motor_ref[RHP].pos;
          }
	  else
          {
            this->dl->darwin_data.motor_ref[i].pos    = this->darwin_ref_walking.motor_ref[i].pos;
          }
          this->dl->darwin_data.motor_ref[i].speed  = this->darwin_ref_walking.motor_ref[i].speed;
          this->dl->darwin_data.motor_ref[i].torque = this->darwin_ref_walking.motor_ref[i].torque;
        }
        else
        {
          this->dl->darwin_data.motor_ref[i].pos    = this->darwin_ref.motor_ref[i].pos;
          this->dl->darwin_data.motor_ref[i].speed  = this->darwin_ref.motor_ref[i].speed;
          this->dl->darwin_data.motor_ref[i].torque = this->darwin_ref.motor_ref[i].torque;
        }
      }
      break;
    }
    
    default:
    {
//      printf("Mode: MODE_REF (Default)\n");
      for( int i = 0; i <= DARWIN_MOTOR_MAX; i++ )
      {
        this->dl->darwin_data.motor_ref[i].pos    = this->darwin_ref.motor_ref[i].pos;
        this->dl->darwin_data.motor_ref[i].speed  = this->darwin_ref.motor_ref[i].speed;
        this->dl->darwin_data.motor_ref[i].torque = this->darwin_ref.motor_ref[i].torque;
      }
      break;
    }
  }

  switch (mode)
  {
   case HZ_REF_SLOW_TOP:
   {
      const int LOWER_START = 7;
      const int LOWER_END   = 18;

      int upper_array[] = {1,2,3,4,5,6,19,20};
      int UPPER_LENGTH  = 8;

      /* Set one upper Ref per cycle */
      upper_i++;
      if(upper_i >= UPPER_LENGTH) upper_i = 0;
      ret += this->dl->stageMotor(upper_array[upper_i]);

      /* Always stage lower body */
      for(int i = LOWER_START; i <= LOWER_END; i++)
      {
        ret += this->dl->stageMotor(i);
      }
      break;
    }
    default: 
    {
      ret += this->dl->stageMotor();
      ret += this->dl->putMotor(); 
      break;
    }
  }
  if( ret > 1 ) ret = 1;
  return ret;
}


int DarwinAch::do_state(int mode)
{
  size_t fs;
  int ret = 0;
  /* Get State */
  switch (mode)
  {
    case HZ_STATE_50_IMU_MOTOR_FT:
    {
      ret += this->dl->getImu();
      ret += this->dl->getFt();
      ret += this->dl->getMotorSlow(1);
      break;
    }
    case HZ_STATE_50_IMU:
    {
      ret += this->dl->getImu();
      break;
    }
    case HZ_STATE_125_IMU:
    {
      ret += this->dl->getImu();
      break;
    }
    case HZ_STATE_100_IMU_FT_SLOW:
    {
      /* Get IMU State */
      ret += this->dl->getImu();

      /* Get Ft State every other cycle */
      if(ft_i == 0) ft_i = 1;
      else ft_i = 0;
      ret += this->dl->getFt(ft_i);
      break;
    }
    case HZ_STATE_100_IMU_MOTORS_SLOW:
    {
      /* Get IMU State */
      ret += this->dl->getImu();

      /* Get Motor State One per cycle */
      ret += this->dl->getMotorSlow(1);
    }
    default:
    {
      ret += 1;
      break;  
    }
  }

  this->darwin_state = this->dl->darwin_data;

  /* Put the data back on the Ach Channel */
  ach_put(&this->chan_darwin_state, &this->darwin_state, sizeof(this->darwin_state));

  if( ret > 0 ) ret = 1;
  return ret;
}
