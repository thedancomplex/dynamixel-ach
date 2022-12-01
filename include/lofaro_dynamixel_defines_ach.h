#ifndef LOFARO_DEFINE_ACH

typedef enum {
        HZ_STATE_NULL,
        HZ_REF_NULL,
        HZ_STATE_MOTORS,
        HZ_MODE_COUNT
}__attribute__((packed)) hz_mode_index_t;

typedef enum {
        MODE_REF = 0,
        MODE_COUNT
}__attribute__((packed)) ach_mode_index_t;

typedef enum {
        MODE_BRIDGE_REF,
        MODE_BRIDGE_STATE,
        MODE_BRIDGE_COUNT
}__attribute__((packed)) bridge_mode_index_t;



#define HZ_REF_DEFAULT HZ_REF_NULL
#define HZ_STATE_DEFAULT HZ_STATE_MOTORS
#define HZ_RATE_DEFAULT 100.0

#define DYNAMIXEL_ACH_CHAN_REF          "dynamixel-ach-chan-ref"
#define DYNAMIXEL_ACH_CHAN_STATE        "dynamixel-ach-chan-state"
#define DYNAMIXEL_ACH_CHAN_CMD          "dynamixel-ach-chan-cmd"
#define DYNAMIXEL_ACH_CHAN_CMD_RETURN   "dynamixel-ach-chan-ret"
#define DYNAMIXEL_ACH_CHAN_TIME         "dynamixel-ach-chan-time"

//#include "lofaro_defines_ach.h"

#define DYNAMIXEL_REF_POS_0 0.0
#define DYNAMIXEL_REF_VEL_0 0.75
#define DYNAMIXEL_REF_TOR_0 0.5


typedef enum {
	DYNAMIXEL_CMD_OK,
	DYNAMIXEL_CMD_FAIL,
	DYNAMIXEL_CMD_ON,
	DYNAMIXEL_CMD_OFF,
	DYNAMIXEL_CMD_START,
	DYNAMIXEL_CMD_MODE,
	DYNAMIXEL_CMD_ON_MOTOR,
	DYNAMIXEL_CMD_OFF_MOTOR,
	DYNAMIXEL_CMD_OPEN,
	DYNAMIXEL_CMD_CLOSE,
	DYNAMIXEL_CMD_LOOP_MODE,
	DYNAMIXEL_CMD_ID_ADD,
	DYNAMIXEL_CMD_ID_RESET,
	DYNAMIXEL_CMD_COUNT
        
}__attribute__((packed)) dynamixel_cmd_mode_index_t;

#endif

#define LOFARO_DEFINE_ACH 1
