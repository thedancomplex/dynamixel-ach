#define DARWIN_METHODS_DYN 1

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
DarwinLofaro::DarwinLofaro()
{
//  portHandler   = dynamixel::PortHandler::getPortHandler(DEVICENAME);

//  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  memset(&this->darwin_data, 0, sizeof(this->darwin_data));
  for( int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++ )
  {
    this->darwin_data.motor_ref[i].torque = MOTOR_TORQUE_MAX;
  }
  return;
}

/* Open Port */
int DarwinLofaro::open()
{
  std::string str  = SERIAL_PORT_DEFAULT;
  char *cstr = new char[str.length() + 1];
  strcpy(cstr, str.c_str());
  int ret = this->open(cstr);
  delete [] cstr;
  return ret;
}

/* Open Port and change port number */
int DarwinLofaro::open(const char *port)
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
int DarwinLofaro::setup()
{
  std::string str  = SERIAL_PORT_DEFAULT;
  char *cstr = new char[str.length() + 1];
  strcpy(cstr, str.c_str());
  int ret = this->setup(cstr, SERIAL_PORT_LOW_LATENCY_DEFAULT);
  delete [] cstr;
  return ret;
}

int DarwinLofaro::setup(std::string str)
{
  char *cstr = new char[str.length() + 1];
  strcpy(cstr, str.c_str());
  int ret = this->setup(cstr);
  delete [] cstr;
  return ret;
}

/* Setup system with optional port */
int DarwinLofaro::setup(const char *port)
{
  return this->setup(port, SERIAL_PORT_LOW_LATENCY_DEFAULT);
}

/* Setup system with low latency flag */
int DarwinLofaro::setup(bool low_latency)
{
  std::string str  = SERIAL_PORT_DEFAULT;
  char *cstr = new char[str.length() + 1];
  strcpy(cstr, str.c_str());
  int ret = this->setup(cstr, low_latency);
  delete [] cstr;
  return ret;
}

int DarwinLofaro::setup(std::string str, bool low_latency)
{
  char *cstr = new char[str.length() + 1];
  strcpy(cstr, str.c_str());
  int ret = this->setup(cstr, low_latency);
  delete [] cstr;
  return ret;
}

/* Setup system with optional port and low latency flag */
int DarwinLofaro::setup(const char *port, bool low_latency)
{
  this->open(port);
  this->setLowLatency(port, low_latency);
  return RETURN_OK;
}

/* Sets low-latency for serial port */
int DarwinLofaro::setLowLatency(const char* the_serial_port, bool low_latency)
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

/* Get IMU State */
int DarwinLofaro::getImu()
{
  // dynamixel::GroupBulkRead groupBulkReadImu(portHandler, packetHandler);
  bool dxl_addparam_result = false;               // addParam result
  groupBulkReadImu.clearParam();

  // Add parameter storage for Dynamixel#1 present position value
  // +1 is added to read the voltage
  dxl_addparam_result = groupBulkReadImu.addParam(ID_CM730, 
                                                        CM730_ADDRESS_IMU_START, 
                                                        CM730_ADDRESS_IMU_LENGTH+1);

  if (dxl_addparam_result != true) return RETURN_FAIL;
  bool dxl_getdata_result = false;                // GetParam result
  uint8_t dxl_error = 0;                          // Dynamixel error

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  dxl_comm_result = groupBulkReadImu.txRxPacket();
  packetHandler->getTxRxResult(dxl_comm_result);
  if (groupBulkReadImu.getError(ID_CM730, &dxl_error)) return RETURN_FAIL;

  // Check if data is avaliable
  dxl_getdata_result = groupBulkReadImu.isAvailable(ID_CM730, 
                                                          CM730_ADDRESS_IMU_START, 
                                                          CM730_ADDRESS_IMU_LENGTH);
  if (dxl_getdata_result != true) return RETURN_FAIL;

  // Assign the data
  uint16_t buff_gyro_x  = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_GYRO_X, 2);
  uint16_t buff_gyro_y  = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_GYRO_Y, 2);
  uint16_t buff_gyro_z  = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_GYRO_Z, 2);
  uint16_t buff_acc_x   = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_ACC_X, 2);
  uint16_t buff_acc_y   = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_ACC_Y, 2);
  uint16_t buff_acc_z   = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_IMU_ACC_Z, 2);
  uint8_t  buff_voltage = groupBulkReadImu.getData(ID_CM730, CM730_ADDRESS_VOLTAGE, 1);

  this->darwin_data.imu.gyro_x  = this->int2double(buff_gyro_x) * IMU_GYRO_SCALE;
  this->darwin_data.imu.gyro_y  = this->int2double(buff_gyro_y) * IMU_GYRO_SCALE;
  this->darwin_data.imu.gyro_z  = this->int2double(buff_gyro_z) * IMU_GYRO_SCALE;
  this->darwin_data.imu.acc_x   = this->int2double(buff_acc_x)  * IMU_ACC_SCALE;
  this->darwin_data.imu.acc_y   = this->int2double(buff_acc_y)  * IMU_ACC_SCALE;
  this->darwin_data.imu.acc_z   = this->int2double(buff_acc_z)  * IMU_ACC_SCALE;
  this->darwin_data.imu.voltage = (double)buff_voltage    / VOLTAGE_SCALE;
 
  return RETURN_OK;
}

/* Turn on all */
int DarwinLofaro::on()
{
  int ret = 0;
  ret += this->on(ID_CM730);
  this->lut->sleep(2.0);
  ret += this->on(ID_FT_RIGHT);
  this->lut->sleep(0.1);
  ret += this->on(ID_FT_LEFT);
  this->lut->sleep(0.1);
  for(int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++)
  {
    ret += this->on(i);
    this->lut->sleep(0.05);
  }
  if( ret > 0 ) return RETURN_FAIL;
  return RETURN_OK;
}

int DarwinLofaro::write(uint8_t id, uint8_t addr, uint8_t d0)
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

int DarwinLofaro::on(int id)
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

int DarwinLofaro::sleep(double val)
{
  return this->lut->sleep(val);
}

int DarwinLofaro::write(uint8_t *txpacket)
{

  int ret = packetHandler->txPacket(portHandler, txpacket);
  portHandler->is_using_ =  false;
  return ret;
}

/* Turn off all */
int DarwinLofaro::off()
{
  int ret = 0;
  ret += this->off(ID_CM730);
  this->lut->sleep(2.0);
  ret += this->off(ID_FT_RIGHT);
  this->lut->sleep(0.1);
  ret += this->off(ID_FT_LEFT);
  this->lut->sleep(0.1);
  for(int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++)
  {
    ret += this->off(i);
    this->lut->sleep(0.05);
  }
  if( ret > 0 ) return RETURN_FAIL;
  return RETURN_OK;
}

/* Turn off "id" */
int DarwinLofaro::off(int id)
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

double DarwinLofaro::int2double(uint16_t val)
{
  double the_out = (double)((int32_t)val - 512) / 1023.0;
  return the_out;
}

/* Stops and turns off everything */
int DarwinLofaro::stop()
{
  int ret = off();
  ret += close();
  if( ret > 0 ) return RETURN_FAIL;
  return RETURN_OK;  
}

/* Closes port */
int DarwinLofaro::close()
{
  // Close port
  portHandler->closePort();
  return RETURN_OK;
}


int DarwinLofaro::sleep()
{
  return this->lut->sleep();
}

int DarwinLofaro::rate(double hz)
{
  return this->lut->rate(hz);
}

double DarwinLofaro::time()
{
  return this->lut->getTime();
}

/* Get Left and Right FT states */
int DarwinLofaro::getFt()
{ 
  int ret = getFt(ID_FT_LEFT);
  ret    += getFt(ID_FT_RIGHT);
  if( ret > 0 ) return RETURN_FAIL;
  return RETURN_OK; 
}

/* FT specific char 2 double */
double DarwinLofaro::ft_char2double(uint8_t val, int* err)
{
    if( val == 255)
    {
      *err = RAISED;
      return 0.0;
    }

    double the_out = (double)val - 127.0 / 127.0;
    *err = NOT_RAISED;
    return the_out;
}

/* Get "id" FT state */
int DarwinLofaro::getFt(int id)
{ 
//  dynamixel::GroupBulkRead groupBulkReadFt(portHandler, packetHandler);

  int the_index = -1;
  if      (id == ID_FT_LEFT)      the_index = ENUM_FT_LEFT;
  else if (id == ID_FT_RIGHT)     the_index = ENUM_FT_RIGHT;
  else if (id == ENUM_FT_LEFT)  { the_index = ENUM_FT_LEFT;  id = ID_FT_LEFT; }
  else if (id == ENUM_FT_RIGHT) { the_index = ENUM_FT_RIGHT; id = ID_FT_RIGHT; }
  else return RETURN_FAIL;

  bool dxl_addparam_result = false;               // addParam result
  groupBulkReadFt.clearParam();

  // Add parameter storage for Dynamixel#1 present position value
  // +1 is added to read the voltage
  dxl_addparam_result = groupBulkReadFt.addParam(id, FT_ADDRESS_START, FT_ADDRESS_LENGTH);
  if (dxl_addparam_result != true) return RETURN_FAIL;

  bool dxl_getdata_result = false;                // GetParam result
  uint8_t dxl_error = 0;                          // Dynamixel error

  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  dxl_comm_result = groupBulkReadFt.txRxPacket();
  packetHandler->getTxRxResult(dxl_comm_result);
  if (groupBulkReadFt.getError(id, &dxl_error)) return RETURN_FAIL;

  // Check if data is avaliable
  dxl_getdata_result = groupBulkReadFt.isAvailable(id, FT_ADDRESS_START, FT_ADDRESS_LENGTH);
  if (dxl_getdata_result != true) return RETURN_FAIL;

  // Assign the data
  uint16_t buff_s1       = groupBulkReadFt.getData(id, FT_ADDRESS_S1, 2);
  uint16_t buff_s2       = groupBulkReadFt.getData(id, FT_ADDRESS_S2, 2);
  uint16_t buff_s3       = groupBulkReadFt.getData(id, FT_ADDRESS_S3, 2);
  uint16_t buff_s4       = groupBulkReadFt.getData(id, FT_ADDRESS_S4, 2);
  uint16_t buff_fsr_x    = groupBulkReadFt.getData(id, FT_ADDRESS_FSR_X, 2);
  uint16_t buff_fsr_y    = groupBulkReadFt.getData(id, FT_ADDRESS_FSR_Y, 2);
  uint8_t  buff_voltage  = groupBulkReadFt.getData(id, FT_ADDRESS_VOLTAGE, 1);

  this->darwin_data.ft[the_index].s0   = this->int2double(buff_s1)  * FT_SCALE;
  this->darwin_data.ft[the_index].s1   = this->int2double(buff_s2)  * FT_SCALE;
  this->darwin_data.ft[the_index].s2   = this->int2double(buff_s3)  * FT_SCALE;
  this->darwin_data.ft[the_index].s3   = this->int2double(buff_s4)  * FT_SCALE;

  int ft_fsr_raised_x = 0;
  int ft_fsr_raised_y = 0;

  this->darwin_data.ft[the_index].x    = this->ft_char2double(buff_fsr_x, &ft_fsr_raised_x) * FSR_SCALE_X;
  this->darwin_data.ft[the_index].y    = this->ft_char2double(buff_fsr_y, &ft_fsr_raised_y) * FSR_SCALE_Y;

  this->darwin_data.ft[the_index].raised_x = ft_fsr_raised_x;
  this->darwin_data.ft[the_index].raised_y = ft_fsr_raised_y;
   
  this->darwin_data.ft[the_index].voltage  = (double)buff_voltage / VOLTAGE_SCALE;

return RETURN_OK; 
}

uint16_t DarwinLofaro::double2uint16(double val)
{
  uint16_t the_out = 0;

  the_out = (uint16_t)((val / M_PI * 0x800) + 0x800);

  if(the_out > 0xfff) the_out = 0xfff;
  else if( the_out < 0) the_out = 0;

  return the_out;
}

int DarwinLofaro::setMotPos(int mot, double val)
{
  /* Sets the motor desired position in rad */
  if( ( mot > DARWIN_MOTOR_MAX ) | ( mot < DARWIN_MOTOR_MIN ) ) return RETURN_FAIL;
  int id = mot;

  this->darwin_data.motor_ref[id].pos = val;
  return RETURN_OK;
}

int DarwinLofaro::setMotTorque(int mot, double val)
{
  /* Sets the motor desired max load in percentage */
  if( ( mot > DARWIN_MOTOR_MAX ) | ( mot < DARWIN_MOTOR_MIN ) ) return RETURN_FAIL;
  int id = mot;

  if(val < 0) return RETURN_FAIL;

  this->darwin_data.motor_ref[id].torque = val;
  return RETURN_OK;
}

int DarwinLofaro::setMotSpeed(int mot, double val)
{
  /* Sets the motor desired speed in rad/sec */
  if( ( mot > DARWIN_MOTOR_MAX ) | ( mot < DARWIN_MOTOR_MIN ) ) return RETURN_FAIL;
  int id = mot;

  if(val < 0) return RETURN_FAIL;

  this->darwin_data.motor_ref[id].speed = val;
  return RETURN_OK;
}

int DarwinLofaro::stageMotor()
{
  /* Stage all motor positions torques and speeds */
  int ret = 0;
  for (int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++)
  {
    ret += this->stageMotor(i);
  }
  return RETURN_FAIL; 
  if (ret > 0) return RETURN_FAIL;
  return RETURN_OK;
}

/* Stage Motor Position */
int DarwinLofaro::stageMotor(int mot)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    if( ( mot > DARWIN_MOTOR_MAX ) | ( mot < DARWIN_MOTOR_MIN ) ) return RETURN_FAIL;

    uint8_t  id = (uint8_t)mot;
    uint8_t  address = MX_ADDRESS_REF_START;
    uint8_t  length  = MX_ADDRESS_REF_LENGTH;
    uint16_t pos     = this->double2uint16(this->darwin_data.motor_ref[id].pos);    
    double   vel_d   = (this->darwin_data.motor_ref[id].speed * MOTOR_REF_SPEED_SCALE);
    uint16_t vel     = (uint16_t) (vel_d);
    double   tor_d   = (this->darwin_data.motor_ref[id].torque * 0x3ff);
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
int DarwinLofaro::putMotor()
{
  return this->putMotor(ID_ALL); 
}

/* Send staged motor positions to motor "mot" */
int DarwinLofaro::putMotor(int mot)
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


int gms_i   = DARWIN_MOTOR_MIN;
int gms_val = 0;
int DarwinLofaro::getMotorSlow(int val)
{
  /* Stage all motor positions torques and speeds */
  if(gms_val >= val) gms_val = 0;

  if (gms_val == 0)
  {
    gms_i++;
    if( gms_i > DARWIN_MOTOR_MAX ) gms_i = DARWIN_MOTOR_MIN;
    gms_val++;
    return this->getMotor(gms_i);
  }

  return RETURN_FAIL;
}

int DarwinLofaro::getMotor()
{
  /* Stage all motor positions torques and speeds */
  int ret = 0;
  for (int i = DARWIN_MOTOR_MIN; i <= DARWIN_MOTOR_MAX; i++)
  {
    ret += this->getMotor(i);
  }
  return RETURN_FAIL; 
  if (ret > 0) return RETURN_FAIL;
  return RETURN_OK;
}

/* Get Motor State */
int DarwinLofaro::getMotor(int id)
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

//  this->darwin_data.motor_state[id].pos      = this->int2double(buff_pos)    * MOTOR_POS_SCALE;
//  this->darwin_data.motor_state[id].speed    = this->int2double(buff_speed)  * MOTOR_SPEED_SCALE;
//  this->darwin_data.motor_state[id].load     = this->int2double(buff_load)   * MOTOR_LOAD_SCALE;

  this->darwin_data.motor_state[id].pos      = lut->dynEncoder2rad(buff_pos, MOTOR_ENC_REZ);
  this->darwin_data.motor_state[id].speed    = lut->dynEncoderSpeed2radPerSec(buff_speed);
  this->darwin_data.motor_state[id].load     = lut->dynSensorLoad2Percent(buff_load);
  this->darwin_data.motor_state[id].voltage  = (double)buff_voltage  * MOTOR_VOLTAGE_SCALE;
  this->darwin_data.motor_state[id].temp     = (double)buff_temp     * MOTOR_TEMP_SCALE;

  return RETURN_OK; 
}


/* Set Gain */
int DarwinLofaro::setGain(int mot, double val, int mode)
{
    if(mot > DARWIN_MOTOR_MAX) return RETURN_FAIL;
    if(mot < DARWIN_MOTOR_MIN) return RETURN_FAIL;

    uint8_t addr = MX_ADDRESS_P_GAIN;
    if     ( mode == DARWIN_ENUM_P_GAIN ) addr = MX_ADDRESS_P_GAIN;
    else if( mode == DARWIN_ENUM_I_GAIN ) addr = MX_ADDRESS_I_GAIN;
    else if( mode == DARWIN_ENUM_D_GAIN ) addr = MX_ADDRESS_D_GAIN;
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
int DarwinLofaro::setPGain(int mot, double val)
{
  return this->setGain(mot, val, DARWIN_ENUM_P_GAIN);
}

/* Set I Gain */
int DarwinLofaro::setIGain(int mot, double val)
{
  return this->setGain(mot, val, DARWIN_ENUM_I_GAIN);
}

/* Set D Gain */
int DarwinLofaro::setDGain(int mot, double val)
{
  return this->setGain(mot, val, DARWIN_ENUM_D_GAIN);
}

/* Get Button */
uint8_t DarwinLofaro::getButton()
{
  uint8_t buff = 0;
  int e = this->read(ID_CM730, CM730_ADDRESS_BUTTON, &buff);
  return buff;
}

int DarwinLofaro::getButton(int butt, uint8_t buff)
{
  if(butt > 1) return -1;
  if(butt < 0) return -1;

  uint8_t b = 1;
  b = b << butt;
  
  buff = buff & b;
  if( buff > 0 ) return 1;
  return 0;
}

int DarwinLofaro::getButton(int butt)
{
  uint8_t buff = this->getButton();
  return this->getButton(butt, buff);
}

/* Get LED */
int DarwinLofaro::getLed(int val)
{
  if( val > 2 ) return -1;

  uint8_t buff = this->getLed();
  uint8_t err = 128 & buff;

  if( err > 0 ) return -1;

  uint8_t the_out = 1;
  the_out = the_out << val;
  the_out = the_out & buff;
  if( buff > 0 ) return 1;
  return 0;
}

uint8_t DarwinLofaro::getLed()
{
  uint8_t buff     = 0;
  uint8_t buff_err = 128;
  int e = this->read(ID_CM730, CM730_ADDRESS_LED_PANNEL, &buff);
  if( e == RETURN_FAIL) buff = buff | buff_err;
  return buff;
}

/* Set LED */
int DarwinLofaro::setLed(uint8_t val)
{
  return this->write(ID_CM730, CM730_ADDRESS_LED_PANNEL, val);
}

int DarwinLofaro::setLed(int led, int val)
{
  if(led > 2 ) return RETURN_FAIL;

  uint8_t d = 0;
  if( val > 0 ) d = 1;

  uint8_t tmp = d << led;
  this->led_state = this->led_state | tmp;
  return this->setLed(this->led_state);
}

/* Read one byte */
uint8_t DarwinLofaro::read(uint8_t id, uint8_t addr)
{
  uint8_t buff = 0;
  this->read(id, addr, &buff);
  return buff;
}

int DarwinLofaro::read(uint8_t id, uint8_t addr, uint8_t* buff)
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

