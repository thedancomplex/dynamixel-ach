#ifndef LOFARO_DEFINE_ACH

typedef enum {
        HZ_STATE_NULL,
        HZ_REF_NULL,
	HZ_STATE_50_IMU_MOTOR_FT,
	HZ_STATE_50_IMU,
	HZ_STATE_125_IMU,
	HZ_STATE_100_IMU_FT_SLOW,
	HZ_STATE_100_IMU_MOTORS_SLOW,
	HZ_REF_SLOW_TOP,
	HZ_REF_NORMAL,
	WALKING_START,
	WALKING_STOP,
        DARWIN_HZ_MODE_COUNT
}__attribute__((packed)) darwin_hz_mode_index_t;

typedef enum {
        MODE_REF = 0,
        MODE_WALKING,
	MODE_WALKING_LOWER_ONLY,
	MODE_WALKING_LOWER_ONLY_STIFF_HIP_PITCH,
        MODE_COUNT
}__attribute__((packed)) darwin_ach_mode_index_t;

typedef enum {
        MODE_BRIDGE_REF,
        MODE_BRIDGE_STATE,
        MODE_BRIDGE_COUNT
}__attribute__((packed)) darwin_bridge_mode_index_t;



#define HZ_REF_DEFAULT HZ_REF_NULL
#define HZ_STATE_DEFAULT HZ_STATE_100_IMU_FT_SLOW
#define HZ_RATE_DEFAULT 100.0

#define DARWIN_ACH_CHAN_REF          "darwin-ach-chan-ref"
#define DARWIN_ACH_CHAN_REF_WALKING  "darwin-ach-chan-ref-walking"
#define DARWIN_ACH_CHAN_STATE        "darwin-ach-chan-state"
#define DARWIN_ACH_CHAN_CMD          "darwin-ach-chan-cmd"
#define DARWIN_ACH_CHAN_CMD_RETURN   "darwin-ach-chan-ret"
#define DARWIN_ACH_CHAN_CMD_VEL      "darwin-ach-chan-cmd-vel"
#define DARWIN_ACH_CHAN_TIME         "darwin-ach-chan-time"

//#include "lofaro_defines_ach.h"

#define DARWIN_REF_POS_0 0.0
#define DARWIN_REF_VEL_0 0.75
#define DARWIN_REF_TOR_0 0.5


typedef enum {
	DARWIN_CMD_OK,
	DARWIN_CMD_FAIL,
	DARWIN_CMD_ON,
	DARWIN_CMD_OFF,
	DARWIN_CMD_START,
	DARWIN_CMD_MODE,
	DARWIN_CMD_ON_MOTOR,
	DARWIN_CMD_OFF_MOTOR,
	DARWIN_CMD_OPEN,
	DARWIN_CMD_CLOSE,
	DARWIN_CMD_LOOP_MODE,
	DARWIN_CMD_COUNT
        
}__attribute__((packed)) darwin_cmd_mode_index_t;

#endif

#define LOFARO_DEFINE_ACH 1
