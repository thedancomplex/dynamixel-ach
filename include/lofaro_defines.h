#ifndef DARWIN_DEFINES 

typedef enum {
        JOINT_NONE=0,
	RSP=1,
	LSP=2,
	RSR=3,
	LSR=4,
	REP=5,
	LEP=6,
	RHY=7,
	LHY=8,
	RHR=9,
	LHR=10,
	RHP=11,
	LHP=12,
	RKP=13,
	LKP=14,
	RAP=15,
	LAP=16,
	RAR=17,
	LAR=18,
	NKY=19,
	NKP=20,
        DARWIN_JOINT_NAME_COUNT
}__attribute__((packed)) darwin_joint_name_index_t;


# define M_PI                              3.14159265358979323846

// Buffer positions
#define BUFFER_ID                          2
#define BUFFER_LENGTH                      3       

// Enumbs and defines
#define CM730_ON                           1
#define DYN_ON                             CM730_ON  
#define CM730_OFF                          0
#define DYN_OFF                            CM730_OFF
#define DARWIN_ON                          CM730_ON
#define DARWIN_OFF                         CM730_OFF
//  #define IMU_ACC_SCALE 1.0
#define IMU_ACC_SCALE                      70.67723342939482
#define IMU_GYRO_SCALE                     500.0
#define VOLTAGE_SCALE                      10.0
#define FT_SCALE                           1/1000.0
#define FSR_SCALE_X                        1.0
#define FSR_SCALE_Y                        1.0
#define RETURN_OK                          0
#define RETURN_FAIL                        1
#define MOTOR_VOLTAGE_SCALE                0.1
#define MOTOR_POS_SCALE                    1.0 / 4096.0 * 2.0 * M_PI
#define MOTOR_SPEED_SCALE                  0.11 / 60.0  * 2.0 * M_PI
#define MOTOR_LOAD_SCALE                   1.0
#define MOTOR_TEMP_SCALE                   1.0
#define MOTOR_ENC_REZ                      4096
#define MOTOR_SPEED_REZ                    2048
#define MOTOR_LOAD_REZ                     2048
#define ENUM_FT_LEFT                       0
#define ENUM_FT_RIGHT                      1
#define MOTOR_REF_SPEED_SCALE              0x3ff / (116.62 / 60.0 * 2.0 * M_PI)
//#define MOTOR_REF_SPEED_SCALE              0.114 / 60.0 * 2.0 * M_PI
// 0.114 Revolutions / Minute / tick * 1 Minute / 60.0 Seconds * 2.0 * M_PI Rad / Revolution
// 0.114 / 60.0 * 2.0 * M_PI Rad / Sec / tick

#define MOTOR_TORQUE_MAX                   1.0

#define DYN_ID_ALL                         0xfe
#define ID_ALL                             DYN_ID_ALL

// Motor IDs
#define ID_CM730                           200
#define ID_DARWIN                          ID_CM730
#define ID_FT_LEFT                         112
#define ID_FT_RIGHT                        111
#define ID_FT                              1000


// Addresses 
#define CM730_ADDRESS_DYN_POWER            24
#define DYN_ADDRESS_DYN_POWER              CM730_ADDRESS_DYN_POWER
#define CM730_ADDRESS_ID                   3

#define CM730_ADDRESS_VOLTAGE              50
#define CM730_ADDRESS_STATUS_RETURN_LEVEL  16

#define CM730_ADDRESS_IMU_START            38
#define CM730_ADDRESS_IMU_LENGTH           12
#define CM730_ADDRESS_READ_DATA_OFFSET     5

#define CM730_ADDRESS_IMU_GYRO_Z           38
#define CM730_ADDRESS_IMU_GYRO_Y           40
#define CM730_ADDRESS_IMU_GYRO_X           42
#define CM730_ADDRESS_IMU_ACC_X            44
#define CM730_ADDRESS_IMU_ACC_Y            46
#define CM730_ADDRESS_IMU_ACC_Z            48

#define CM730_ADDRESS_LED_PANNEL           25
#define CM730_ADDRESS_BUTTON               30


#define MX_ID                              2  
#define MX_ADDRESS_READ_DATA_OFFSET        5
//#define MX_ADDRESS_STATE_START             36
#define MX_ADDRESS_STATE_LENGTH            8

#define MX_PACKET_PING                     0X01
#define MX_PACKET_READ                     0X02
#define MX_PACKET_WRITE                    0x03
#define MX_PACKET_REG_WRITE                0X04
#define MX_PACKET_ACTION                   0x05
#define MX_PACKET_FACTORY_RESET            0x06
#define MX_PACKET_REBOOT                   0x08
#define MX_PACKET_SYNC_WRITE               0x83
#define MX_PACKET_BULK_READ                0x92

#define MX_ADDRESS_POS_GOAL                30
#define MX_ADDRESS_VEL_GOAL                32
#define MX_ADDRESS_TORQUE_MAX              34
#define MX_ADDRESS_REF_LENGTH              6
#define MX_ADDRESS_REF_START               MX_ADDRESS_POS_GOAL

#define MX_ADDRESS_POS                     36
#define MX_ADDRESS_SPEED                   38
#define MX_ADDRESS_LOAD                    40
#define MX_ADDRESS_VOLTAGE                 42
#define MX_ADDRESS_TEMP                    43
#define MX_ADDRESS_DELAY                   5
#define MX_ADDRESS_STATUS_RETURN_LEVEL     16
#define MX_ADDRESS_GOAL_POS                30
#define MX_ADDRESS_STATE_LENGTH            8
#define MX_ADDRESS_STATE_START             MX_ADDRESS_POS

#define MX_ADDRESS_P_GAIN                  28
#define MX_ADDRESS_I_GAIN                  27
#define MX_ADDRESS_D_GAIN                  26


#define FT_ADDRESS_STATUS_RETURN_LEVEL     16
#define FT_ADDRESS_READ_DATA_OFFSET        5
#define FT_ADDRESS_START                   26
#define FT_ADDRESS_LENGTH                  16
#define FT_ADDRESS_S1                      26
#define FT_ADDRESS_S2                      28
#define FT_ADDRESS_S3                      30
#define FT_ADDRESS_S4                      32
#define FT_ADDRESS_FSR_X                   34
#define FT_ADDRESS_FSR_Y                   35
#define FT_ADDRESS_VOLTAGE                 42
#define FT_ADDRESS_DELAY                   5

#define SERIAL_PORT_DEFAULT                "/dev/ttyUSB0"
#define DEVICENAME                         SERIAL_PORT_DEFAULT


#define DARWIN_MOTOR_BROADCAST             0Xfe
#define DARWIN_MOTOR_NUM                   20
#define DARWIN_MOTOR_MIN                   1
#define DARWIN_MOTOR_MAX                   20
#define DARWIN_FT_NUM                      2
#define DARWIN_MOTOR_MIN_LOWER             7
#define DARWIN_MOTOR_MAX_LOWER             18
#define DARWIN_MOTOR_MIN_HEAD              19
#define DARWIN_MOTOR_MAX_HEAD              20
#define DARWIN_MOTOR_MIN_UPPER             1
#define DARWIN_MOTOR_MAX_UPPER             6


#define ERROR                              1
#define NO_ERROR                           0
#define RAISED                             1
#define NOT_RAISED                         0

#define BAUDRATE                           1000000
#define STDIN_FILENO                       0
#define SERIAL_PORT_LOW_LATENCY_DEFAULT    false

#define PROTOCOL_VERSION                   1.0

#define DARWIN_X                           1
#define DARWIN_Y                           2
#define DARWIN_Z                           3

#define DARWIN_ENUM_P_GAIN                 1
#define DARWIN_ENUM_I_GAIN                 2
#define DARWIN_ENUM_D_GAIN                 3

#define DARWIN_BUTTON_START                1
#define DARWIN_BUTTON_MODE                 0

#define LED_MODE_0                         0
#define LED_MODE_1                         1
#define LED_MODE_2                         2
#define LED_MODE_3                         3


#endif

# define DARWIN_DEFINES 1
