
#ifndef DARWIN_DEFINES
#include "lofaro_defines.h"
#endif


#ifndef DARWIN_TYPES
#include <stdint.h>

typedef struct imu_state_def {
        double acc_x;
        double acc_y;
        double acc_z;
        double gyro_x;
        double gyro_y;
        double gyro_z;
        double voltage;
}__attribute__((packed)) imu_state_def_t;

typedef struct ft_state_def {
        double s0;
        double s1;
        double s2;
        double s3;
        double x;
        double y;
        double voltage;
        int16_t raised_x;
        int16_t raised_y;
        int16_t raised;
}__attribute__((packed)) ft_state_def_t;

typedef struct motor_state_def {
        double pos;
        double speed;
        double load;
        double voltage;
        double temp;
}__attribute__((packed)) motor_state_def_t;

typedef struct motor_ref_def {
        double pos;
        double speed;
        double torque;
        double p_gain;
        double i_gain;
        double d_gain;
//        double voltage;
}__attribute__((packed)) motor_ref_def_t;

typedef struct darwin_cmd_def {
        int16_t cmd;
        int16_t data[4];
        double  data_float[4];
}__attribute__((packed)) darwin_cmd_def_t;

typedef struct darwin_vec3_def {
        double x;
        double y;
        double z;
}__attribute__((packed)) darwin_vec3_def_t;

typedef struct darwin_twist_def {
        darwin_vec3_def_t linear;
        darwin_vec3_def_t angular;
        int16_t           cmd;
}__attribute__((packed)) darwin_twist_def_t;



typedef struct darwin_data_def {
  motor_ref_def_t     motor_ref[DARWIN_MOTOR_NUM+1];
  motor_state_def_t   motor_state[DARWIN_MOTOR_NUM+1];
  imu_state_def_t     imu;
  ft_state_def_t      ft[DARWIN_FT_NUM];
  int16_t             mode;
  darwin_twist_def_t  cmd_vel;
}__attribute__((packed)) darwin_data_def_t;

typedef struct darwin_cmd_vel_def {
  int16_t             mode;
  darwin_vec3_def_t linear;
  darwin_vec3_def_t angular;
}__attribute__((packed)) darwin_cmd_vel_def_t;

#endif
#define DARWIN_TYPES 1	
