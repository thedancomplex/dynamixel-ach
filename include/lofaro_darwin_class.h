#ifndef DARWIN_TYPES
#include "lofaro_types.h"
#endif

#ifndef LOFARO_UTILS
#include "lofaro_utils.h"
#endif

//#include "dynamixel_sdk/dynamixel_sdk.h"
//#include "/usr/local/include/dynamixel_sdk/dynamixel_sdk.h"


class DarwinLofaro
{
  public:
    DarwinLofaro();

    /* Setup system with default values */
    int setup();

    /* Setup system with optional port */
    int setup(std::string str);
    int setup(const char *port);

    /* Setup system with low latency flag */
    int setup(bool low_latency);

    /* Setup system with optional port and low latency flag */
    int setup(std::string sport, bool low_latency);
    int setup(const char *port, bool low_latency);

    /* Set Motor Position */
    int setMotor(int mot, double val);

    /* Get All Motor States */
    int getMotor();

    /* Get Motor State */
    int getMotor(int id);

    /* Get Motor State Slow (every "val" cycles) */
    int getMotorSlow(int val);

    /* Stage Motor Position */
    int stageMotor(int mot);

    /* Stage all motor positions torques and speeds */
    int stageMotor();

    /* Send staged motor positions to all motors */
    int putMotor();

    /* Send staged motor positions to motor "mot" */
    int putMotor(int mot);

    /* Get IMU State */
    int getImu();

    /* Get Left and Right FT states */
    int getFt();

    /* Get "id" FT state */
    int getFt(int id);

    /* Turn on all */
    int on();

    /* Turn on "id" */
    int on(int id);

    /* Turn off all */
    int off();

    /* Turn off "id" */
    int off(int id);

    /* Pass through for the sleep method */
    int sleep(double val);

    /* Stops and turns off everything */
    int stop();

    /* State and Reference Data */
    darwin_data_def_t darwin_data;

    /* set the rate for the auto sleeping timer */
    int rate(double hz);

    /* Sleep for the remainder of time needed to keep a rate of hz as defined by rate() */
    int sleep();

    /* Gets the time in seconds from the wall clock */
    double time();

    /* Sets the motor desired max load in percentage */
    int setMotTorque(int mot, double val);

    /* Sets the motor desired speed in rad/sec */
    int setMotSpeed(int mot, double val);

    /* Set the motor desired reference in rad */
    int setMotPos(int mot, double val);

    /* Set Gain */
    int setGain(int mot, double val, int mode);

    /* Set P Gain */
    int setPGain(int mot, double val);

    /* Set I Gain */
    int setIGain(int mot, double val);

    /* Set D Gain */
    int setDGain(int mot, double val);

    /* Set LED */
    int setLed(uint8_t val);
    int setLed(int led, int val);

    /* Get LED */
    uint8_t getLed();
    int getLed(int val);

    /* Get Button */
    uint8_t getButton();
    int getButton(int butt);
    int getButton(int butt, uint8_t buff);

    /* Closes port */
    int close();

  private:
    /* Open Port */
    int open();

    /* Open Port and change port number */
    int open(const char *port);

    /* Sets low-latency for serial port */
    int setLowLatency(const char* the_serial_port, bool low_latency);

    LofaroUtils* lut = new LofaroUtils();

    /* IMU specific int to double */
    double int2double(uint16_t val);

    /* FT specific char to double */
    double ft_char2double(uint8_t val, int* err);

    /* double to uint16 for motor pos set */
    uint16_t double2uint16(double val);

    /* Direct write packet to try to get around the seg fault */
    int write(uint8_t *txpacket);
    int write(uint8_t id, uint8_t addr, uint8_t d0);

    /* Reads one packet */
    int read(uint8_t id, uint8_t addr, uint8_t* buff);
    uint8_t read(uint8_t id, uint8_t addr);

    /* LED current state */
    uint8_t led_state = 0;
};

