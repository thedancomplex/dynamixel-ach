#define DYNAMIXEL_METHODS_DYN 1

#include "dynamixel_sdk/dynamixel_sdk.h"
//#include "/usr/local/include/dynamixel_sdk/dynamixel_sdk.h"

// Initialize PortHandler instance
// Set the port path
// Get methods and members of PortHandlerLinux or PortHandlerWindows
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

// Initialize PacketHandler instance
// Set the protocol version
// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

// Initialize GroupBulkRead instance
dynamixel::GroupBulkRead groupBulkReadImu(    portHandler, packetHandler);
dynamixel::GroupBulkRead groupBulkReadFt(     portHandler, packetHandler);
dynamixel::GroupBulkRead groupBulkReadMotor(  portHandler, packetHandler);

/* init */
DynamixelLofaro::DynamixelLofaro()
{
//  portHandler   = dynamixel::PortHandler::getPortHandler(DEVICENAME);

//  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  memset(&this->dynamixel_data, 0, sizeof(this->dynamixel_data));
  for( int i = DYNAMIXEL_MOTOR_MIN; i <= DYNAMIXEL_MOTOR_MAX; i++ )
  {
    this->dynamixel_data.motor_ref[i].torque = MOTOR_TORQUE_MAX;
  }
  return;
}

/* Open Port */
int DynamixelLofaro::open()
{
  std::string str  = SERIAL_PORT_DEFAULT;
  char *cstr = new char[str.length() + 1];
  strcpy(cstr, str.c_str());
  int ret = this->open(cstr);
  delete [] cstr;
  return ret;
}

/* Open Port and change port number */
int DynamixelLofaro::open(const char *port)
{
 try
 {
  portHandler->setPacketTimeout(0.001);

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    return RETURN_FAIL;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    return RETURN_FAIL;
  }
 }
 catch(...)
 {
    return RETURN_FAIL;
 }

  return RETURN_OK;
}

/* Setup system with default values */
int DynamixelLofaro::setup()
{
  std::string str  = SERIAL_PORT_DEFAULT;
  char *cstr = new char[str.length() + 1];
  strcpy(cstr, str.c_str());
  int ret = this->setup(cstr, SERIAL_PORT_LOW_LATENCY_DEFAULT);
  delete [] cstr;
  return ret;
}

int DynamixelLofaro::setup(std::string str)
{
  char *cstr = new char[str.length() + 1];
  strcpy(cstr, str.c_str());
  int ret = this->setup(cstr);
  delete [] cstr;
  return ret;
}

/* Setup system with optional port */
int DynamixelLofaro::setup(const char *port)
{
  return this->setup(port, SERIAL_PORT_LOW_LATENCY_DEFAULT);
}

/* Setup system with low latency flag */
int DynamixelLofaro::setup(bool low_latency)
{
  std::string str  = SERIAL_PORT_DEFAULT;
  char *cstr = new char[str.length() + 1];
  strcpy(cstr, str.c_str());
  int ret = this->setup(cstr, low_latency);
  delete [] cstr;
  return ret;
}

int DynamixelLofaro::setup(std::string str, bool low_latency)
{
  char *cstr = new char[str.length() + 1];
  strcpy(cstr, str.c_str());
  int ret = this->setup(cstr, low_latency);
  delete [] cstr;
  return ret;
}

/* Setup system with optional port and low latency flag */
int DynamixelLofaro::setup(const char *port, bool low_latency)
{
  this->open(port);
  this->setLowLatency(port, low_latency);
  return RETURN_OK;
}

/* Sets low-latency for serial port */
int DynamixelLofaro::setLowLatency(const char* the_serial_port, bool low_latency)
{
  if( low_latency )
  {
    const char* head = "setserial ";
    const char* foot = " low_latency";
    char *s = new char[strlen(head) + strlen(foot) + strlen(the_serial_port) + 1];
    strcpy(s, head);
    strcat(s, the_serial_port);
    strcat(s, foot);
    std::system(s);
    sleep(2.0);
    return RETURN_OK;
  }
  return RETURN_FAIL;
}

/* Turn on all */
int DynamixelLofaro::on()
{
  int ret = 0;
  ret += this->on(ID_CM730);
  this->lut->sleep(2.0);
  ret += this->on(ID_FT_RIGHT);
  this->lut->sleep(0.1);
  ret += this->on(ID_FT_LEFT);
  this->lut->sleep(0.1);
  for(int i = DYNAMIXEL_MOTOR_MIN; i <= DYNAMIXEL_MOTOR_MAX; i++)
  {
    ret += this->on(i);
    this->lut->sleep(0.05);
  }
  if( ret > 0 ) return RETURN_FAIL;
  return RETURN_OK;
}

int DynamixelLofaro::write(uint8_t id, uint8_t addr, uint8_t d0)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 
                                                          id, 
                                                          addr, 
                                                          d0, 
                                                          &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      return RETURN_FAIL;
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      return RETURN_FAIL;
    }
    else
    {
      return RETURN_OK;;
    }
    return RETURN_OK;;
}

int DynamixelLofaro::on(int id)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 
                                                          id, 
                                                          CM730_ADDRESS_DYN_POWER, 
                                                          CM730_ON, 
                                                          &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      return RETURN_FAIL;
    }
    else
    {
      printf("Dynamixel has been successfully turned on \n");
    }
    return RETURN_OK;;
}

int DynamixelLofaro::sleep(double val)
{
  return this->lut->sleep(val);
}

int DynamixelLofaro::write(uint8_t *txpacket)
{

  int ret = packetHandler->txPacket(portHandler, txpacket);
  portHandler->is_using_ =  false;
  return ret;
}

/* Turn off all */
int DynamixelLofaro::off()
{
  int ret = 0;
  ret += this->off(ID_CM730);
  this->lut->sleep(2.0);
  ret += this->off(ID_FT_RIGHT);
  this->lut->sleep(0.1);
  ret += this->off(ID_FT_LEFT);
  this->lut->sleep(0.1);
  for(int i = DYNAMIXEL_MOTOR_MIN; i <= DYNAMIXEL_MOTOR_MAX; i++)
  {
    ret += this->off(i);
    this->lut->sleep(0.05);
  }
  if( ret > 0 ) return RETURN_FAIL;
  return RETURN_OK;
}

/* Turn off "id" */
int DynamixelLofaro::off(int id)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 
                                                          id, 
                                                          CM730_ADDRESS_DYN_POWER, 
                                                          CM730_OFF, 
                                                          &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      return RETURN_FAIL;
    }
    else
    {
      printf("Dynamixel has been successfully turned off \n");
    }
    return RETURN_OK;
}

/* Stops and turns off everything */
int DynamixelLofaro::stop()
{
  int ret = off();
  ret += close();
  if( ret > 0 ) return RETURN_FAIL;
  return RETURN_OK;  
}

/* Closes port */
int DynamixelLofaro::close()
{
  // Close port
  portHandler->closePort();
  return RETURN_OK;
}


int DynamixelLofaro::sleep()
{
  return this->lut->sleep();
}

int DynamixelLofaro::rate(double hz)
{
  return this->lut->rate(hz);
}

double DynamixelLofaro::time()
{
  return this->lut->getTime();
}

uint16_t DynamixelLofaro::double2uint16(double val)
{
  uint16_t the_out = 0;

  the_out = (uint16_t)((val / M_PI * 0x800) + 0x800);

  if(the_out > 0xfff) the_out = 0xfff;
  else if( the_out < 0) the_out = 0;

  return the_out;
}

int DynamixelLofaro::setMotPos(int mot, double val)
{
  /* Sets the motor desired position in rad */
  if( ( mot > DYNAMIXEL_MOTOR_MAX ) | ( mot < DYNAMIXEL_MOTOR_MIN ) ) return RETURN_FAIL;
  int id = mot;

  this->dynamixel_data.motor_ref[id].pos = val;
  return RETURN_OK;
}

int DynamixelLofaro::setMotTorque(int mot, double val)
{
  /* Sets the motor desired max load in percentage */
  if( ( mot > DYNAMIXEL_MOTOR_MAX ) | ( mot < DYNAMIXEL_MOTOR_MIN ) ) return RETURN_FAIL;
  int id = mot;

  if(val < 0) return RETURN_FAIL;

  this->dynamixel_data.motor_ref[id].torque = val;
  return RETURN_OK;
}

int DynamixelLofaro::setMotSpeed(int mot, double val)
{
  /* Sets the motor desired speed in rad/sec */
  if( ( mot > DYNAMIXEL_MOTOR_MAX ) | ( mot < DYNAMIXEL_MOTOR_MIN ) ) return RETURN_FAIL;
  int id = mot;

  if(val < 0) return RETURN_FAIL;

  this->dynamixel_data.motor_ref[id].speed = val;
  return RETURN_OK;
}

int DynamixelLofaro::stageMotor()
{
  /* Stage all motor positions torques and speeds */
  int ret = 0;
  for (int i = DYNAMIXEL_MOTOR_MIN; i <= DYNAMIXEL_MOTOR_MAX; i++)
  {
    ret += this->stageMotor(i);
  }
  return RETURN_FAIL; 
  if (ret > 0) return RETURN_FAIL;
  return RETURN_OK;
}

/* Stage Motor Position */
int DynamixelLofaro::stageMotor(int mot)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    if( ( mot > DYNAMIXEL_MOTOR_MAX ) | ( mot < DYNAMIXEL_MOTOR_MIN ) ) return RETURN_FAIL;

    uint8_t  id = (uint8_t)mot;
    uint8_t  address = MX_ADDRESS_REF_START;
    uint8_t  length  = MX_ADDRESS_REF_LENGTH;
    uint16_t pos     = this->double2uint16(this->dynamixel_data.motor_ref[id].pos);    
    double   vel_d   = (this->dynamixel_data.motor_ref[id].speed * MOTOR_REF_SPEED_SCALE);
    uint16_t vel     = (uint16_t) (vel_d);
    double   tor_d   = (this->dynamixel_data.motor_ref[id].torque * 0x3ff);
    uint16_t tor     = (uint16_t)(tor_d);

    if ( vel > 0x3ff ) vel = 0;
    if ( tor > 0x3ff ) tor = 0x3ff;

    uint8_t pos_0 =  pos       & 0xff;
    uint8_t pos_1 = (pos >> 8) & 0xff;
    uint8_t vel_0 =  vel       & 0xff;
    uint8_t vel_1 = (vel >> 8) & 0xff;
    uint8_t tor_0 =  tor       & 0xff;
    uint8_t tor_1 = (tor >> 8) & 0xff;
   
/* 
    uint8_t * data = malloc(sizeof(uint8_t) * 6);
    data[0] = pos_0;
    data[1] = pos_1;
    data[2] = vel_0;
    data[3] = vel_1;
    data[4] = tor_0;
    data[5] = tor_1;
*/
/*
    uint8_t *data_buff = (uint8_t *)malloc(12);
    data_buff[0] = pos_0;
    data_buff[1] = pos_1;
    data_buff[2] = vel_0;
    data_buff[3] = vel_1;
    data_buff[4] = tor_0;
    data_buff[5] = tor_1;
*/

    uint8_t length_d = 9;
    uint8_t data[] = {0xff, 0xff, id, 
                      length_d, 
                      MX_PACKET_REG_WRITE,
                      MX_ADDRESS_POS_GOAL,
                      pos_0, 
                      pos_1, 
                      vel_0, 
                      vel_1, 
                      tor_0, 
                      tor_1,
                      0x00};

    dxl_comm_result = this->write(data);
    
/*

    dxl_comm_result = packetHandler->regWriteTxOnly(portHandler, 
                                                    id,
                                                    address,
                                                    length,
                                                    data);
    
*/

/*
    dxl_comm_result = packetHandler->regWriteTxRx(portHandler, 
                                mot,
                                address,
                                length,
                                data,
                                &dxl_error);
*/
//  free(data_buff);
  if (dxl_comm_result != COMM_SUCCESS) return RETURN_FAIL;
  else if (dxl_error != 0)             return RETURN_FAIL;
  return RETURN_OK; 
}

/* Send staged motor positions to all motors */
int DynamixelLofaro::putMotor()
{
  return this->putMotor(ID_ALL); 
}

/* Send staged motor positions to motor "mot" */
int DynamixelLofaro::putMotor(int mot)
{
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  try
  {
    dxl_comm_result = packetHandler->action(portHandler, mot);
  }
  catch(...)
  {
    return RETURN_FAIL;
  }

  if (dxl_comm_result != COMM_SUCCESS) return RETURN_FAIL;

  return RETURN_OK; 
}

int DynamixelLofaro::getMotor()
{
  /* Stage all motor positions torques and speeds */
  int ret = 0;
  for (int i = DYNAMIXEL_MOTOR_MIN; i <= DYNAMIXEL_MOTOR_MAX; i++)
  {
    ret += this->getMotor(i);
  }
  return RETURN_FAIL; 
  if (ret > 0) return RETURN_FAIL;
  return RETURN_OK;
}

/* Get Motor State */
int DynamixelLofaro::getMotor(int id)
{
  // dynamixel::GroupBulkRead groupBulkReadImu(portHandler, packetHandler);
  bool dxl_addparam_result = false;               // addParam result
  groupBulkReadMotor.clearParam();

  // Add parameter storage for Dynamixel#1 present position value
  // +1 is added to read the voltage
  dxl_addparam_result = groupBulkReadMotor.addParam(id, 
                                                        MX_ADDRESS_STATE_START, 
                                                        MX_ADDRESS_STATE_LENGTH);

  if (dxl_addparam_result != true) return RETURN_FAIL;
  bool dxl_getdata_result = false;                // GetParam result
  uint8_t dxl_error = 0;                          // Dynamixel error

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  dxl_comm_result = groupBulkReadMotor.txRxPacket();
  packetHandler->getTxRxResult(dxl_comm_result);
  if (groupBulkReadMotor.getError(id, &dxl_error)) return RETURN_FAIL;

  // Check if data is avaliable
  dxl_getdata_result = groupBulkReadMotor.isAvailable(id, 
                                                          MX_ADDRESS_STATE_START, 
                                                          MX_ADDRESS_STATE_LENGTH);
  if (dxl_getdata_result != true) return RETURN_FAIL;
  // Assign the data
  uint16_t buff_pos       = groupBulkReadMotor.getData(id, MX_ADDRESS_POS    , 2);
  uint16_t buff_speed     = groupBulkReadMotor.getData(id, MX_ADDRESS_SPEED  , 2);
  uint16_t buff_load      = groupBulkReadMotor.getData(id, MX_ADDRESS_LOAD   , 2);
  uint8_t  buff_voltage   = groupBulkReadMotor.getData(id, MX_ADDRESS_VOLTAGE, 1);
  uint8_t  buff_temp      = groupBulkReadMotor.getData(id, MX_ADDRESS_TEMP   , 1);

//  this->dynamixel_data.motor_state[id].pos      = this->int2double(buff_pos)    * MOTOR_POS_SCALE;
//  this->dynamixel_data.motor_state[id].speed    = this->int2double(buff_speed)  * MOTOR_SPEED_SCALE;
//  this->dynamixel_data.motor_state[id].load     = this->int2double(buff_load)   * MOTOR_LOAD_SCALE;

  this->dynamixel_data.motor_state[id].pos      = lut->dynEncoder2rad(buff_pos, MOTOR_ENC_REZ);
  this->dynamixel_data.motor_state[id].speed    = lut->dynEncoderSpeed2radPerSec(buff_speed);
  this->dynamixel_data.motor_state[id].load     = lut->dynSensorLoad2Percent(buff_load);
  this->dynamixel_data.motor_state[id].voltage  = (double)buff_voltage  * MOTOR_VOLTAGE_SCALE;
  this->dynamixel_data.motor_state[id].temp     = (double)buff_temp     * MOTOR_TEMP_SCALE;

  return RETURN_OK; 
}


/* Set Gain */
int DynamixelLofaro::setGain(int mot, double val, int mode)
{
    if(mot > DYNAMIXEL_MOTOR_MAX) return RETURN_FAIL;
    if(mot < DYNAMIXEL_MOTOR_MIN) return RETURN_FAIL;

    uint8_t addr = MX_ADDRESS_P_GAIN;
    if     ( mode == DYNAMIXEL_ENUM_P_GAIN ) addr = MX_ADDRESS_P_GAIN;
    else if( mode == DYNAMIXEL_ENUM_I_GAIN ) addr = MX_ADDRESS_I_GAIN;
    else if( mode == DYNAMIXEL_ENUM_D_GAIN ) addr = MX_ADDRESS_D_GAIN;
    else return RETURN_FAIL;


    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    uint8_t v = (uint8_t)val;

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler,
                                                          mot,
                                                          addr,
                                                          v,
                                                          &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      return RETURN_FAIL;
    }
    else if (dxl_error != 0)
    {
      return RETURN_FAIL;
    }
    else
    {
      return RETURN_OK;
    }
    return RETURN_OK;

}

/* Set P Gain */
int DynamixelLofaro::setPGain(int mot, double val)
{
  return this->setGain(mot, val, DYNAMIXEL_ENUM_P_GAIN);
}

/* Set I Gain */
int DynamixelLofaro::setIGain(int mot, double val)
{
  return this->setGain(mot, val, DYNAMIXEL_ENUM_I_GAIN);
}

/* Set D Gain */
int DynamixelLofaro::setDGain(int mot, double val)
{
  return this->setGain(mot, val, DYNAMIXEL_ENUM_D_GAIN);
}

/* Read one byte */
uint8_t DynamixelLofaro::read(uint8_t id, uint8_t addr)
{
  uint8_t buff = 0;
  this->read(id, addr, &buff);
  return buff;
}

int DynamixelLofaro::read(uint8_t id, uint8_t addr, uint8_t* buff)
{
  uint8_t dxl_error = 0;
  uint8_t buff_err = 128;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  // Read 1 byte
  dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, addr, buff, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    return RETURN_FAIL;
//    buff = buff | buff_err;
    // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    return RETURN_FAIL;
//    buff = buff | buff_err;
    // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  return RETURN_OK;
}

  /* Set Motor Position */
  int setMotor(int mot, double val)
  { return RETURN_OK; }

