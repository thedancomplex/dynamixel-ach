#ifndef LOFARO_UTILS 

#include <sys/time.h>
#include <unistd.h>
#include <cstddef>


class LofaroUtils
{
  #define MOTOR_SPEED_SCALE     0.11 / 60.0  * 2.0 * M_PI
  #define M_PI                  3.14159265358979323846
  public:
    LofaroUtils() 
    {
      this->tick = this->getTime();
      this->tock = this->getTime();
      this->T    = 1.0/this->hz;
    }

    double getTime()
    {
      long seconds, useconds;
      double duration;
      timeval the_time;
      gettimeofday(&the_time, NULL);
      seconds = the_time.tv_sec;
      useconds = the_time.tv_usec;

      duration = seconds + useconds / 1000000.0;

     //  printf("%f\n", duration);
      return duration;
    }

    int sleep(double val)
    {
      long usec = (long)(val * 1000000);
      return usleep(usec);
    }

    int sleep()
    {
      tock = this->getTime();
      double dt = T - (tock - tick);
      if (dt < 0) dt = 0.0;
      this->sleep(dt);
      tick = this->getTime();
      return 0;
    }

    int rate(double hz_des)
    {
      if(hz_des <= 0) return 1;
      this->hz = hz_des;
      this->T  = 1.0/hz_des;
      return 0;
    }

    double dynEncoder2rad(uint16_t enc, uint16_t rez)
    {
      return ( (double)enc - ((double)rez/2.0) ) * 2.0 * M_PI / (double)rez;
    }
    
    double dynEncoder2rad(uint16_t enc)
    {
      return this->dynEncoder2rad(enc, 4096);
    }

    double dynEncoderSpeed2radPerSec(uint16_t enc)
    {
      uint16_t mag_16 = 0x3ff & enc;
      uint16_t dir_16 = 0x400 & enc;
      double dir = 1.0;
      if(dir_16 > 0) dir = -1.0;
      double mag = (double)mag_16 * dir * MOTOR_SPEED_SCALE;
      return mag;
    }

    double dynSensorLoad2Percent(uint16_t enc)
    {
      uint16_t mag_16 = 0x3ff & enc;
      uint16_t dir_16 = 0x400 & enc;
      double dir = 1.0;
      if(dir_16 > 0) dir = -1.0;
      double mag = (double)mag_16 * dir / (double)0x3ff;
      return mag;
    }

  private:
    double tick = 0.0;
    double tock = 0.0;   
    double hz   = 100.0;
    double T    = 0.01;

};
#endif
#define LOFARO_UTILS 1
