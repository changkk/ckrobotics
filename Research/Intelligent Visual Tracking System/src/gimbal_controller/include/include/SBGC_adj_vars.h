/*
	SimpleBGC Serial API  library - Adjustable Variables
	More info: http://www.basecamelectronics.com/serialapi/

  Copyright (c) 2014-2015 Aleksei Moskalenko
  All rights reserved.
	
	See license info in the SBGC.h
*/

#ifndef  __SBGC_adj_vars__
#define  __SBGC_adj_vars__

#include <inttypes.h>

// ID of all available variables
enum {
	ADJ_VAR_P_ROLL = 0
	,	ADJ_VAR_P_PITCH
	,	ADJ_VAR_P_YAW
	,	ADJ_VAR_I_ROLL
	,	ADJ_VAR_I_PITCH
	,	ADJ_VAR_I_YAW
	,	ADJ_VAR_D_ROLL
	,	ADJ_VAR_D_PITCH
	,	ADJ_VAR_D_YAW
	,	ADJ_VAR_POWER_ROLL
	,	ADJ_VAR_POWER_PITCH
	,	ADJ_VAR_POWER_YAW
	,	ADJ_VAR_ACC_LIMITER
	,	ADJ_VAR_FOLLOW_SPEED_ROLL
	,	ADJ_VAR_FOLLOW_SPEED_PITCH
	,	ADJ_VAR_FOLLOW_SPEED_YAW
	,	ADJ_VAR_FOLLOW_LPF_ROLL
	,	ADJ_VAR_FOLLOW_LPF_PITCH
	,	ADJ_VAR_FOLLOW_LPF_YAW
	,	ADJ_VAR_RC_SPEED_ROLL
	,	ADJ_VAR_RC_SPEED_PITCH
	,	ADJ_VAR_RC_SPEED_YAW
	,	ADJ_VAR_RC_LPF_ROLL
	,	ADJ_VAR_RC_LPF_PITCH
	,	ADJ_VAR_RC_LPF_YAW
	,	ADJ_VAR_RC_TRIM_ROLL
	,	ADJ_VAR_RC_TRIM_PITCH
	,	ADJ_VAR_RC_TRIM_YAW
	,	ADJ_VAR_RC_DEADBAND
	,	ADJ_VAR_RC_EXPO_RATE
	,	ADJ_VAR_FOLLOW_MODE
	,	ADJ_VAR_RC_FOLLOW_YAW
	,	ADJ_VAR_FOLLOW_DEADBAND
	,	ADJ_VAR_FOLLOW_EXPO_RATE
	,	ADJ_VAR_FOLLOW_ROLL_MIX_START
	,	ADJ_VAR_FOLLOW_ROLL_MIX_RANGE
	,	ADJ_VAR_GYRO_TRUST
	, ADJ_VAR_FRAME_HEADING
	, ADJ_VAR_GYRO_HEADING_CORR
	, ADJ_VAL_ACC_LIMITER_ROLL
	, ADJ_VAL_ACC_LIMITER_PITCH
	, ADJ_VAL_ACC_LIMITER_YAW
};


#define ADJ_VAR_NAME_MAX_LENGTH 10

// Descriptors for adjustable variables
typedef struct {
	uint8_t id;
	char *name;  // max. 10 characters
	int32_t min_val;
	int32_t	max_val;
} adjustable_var_cfg_t;

#define ADJ_VAR_DEF_P_ROLL { ADJ_VAR_P_ROLL, "PID_P.R", 0, 255 }
#define ADJ_VAR_DEF_P_PITCH { ADJ_VAR_P_PITCH, "PID_P.P", 0, 255 }
#define ADJ_VAR_DEF_P_YAW { ADJ_VAR_P_YAW, "PID_P.Y", 0, 255 }
#define ADJ_VAR_DEF_I_ROLL { ADJ_VAR_I_ROLL, "PID_I.R", 0, 255 }
#define ADJ_VAR_DEF_I_PITCH { ADJ_VAR_I_PITCH, "PID_I.P", 0, 255 }
#define ADJ_VAR_DEF_I_YAW { ADJ_VAR_I_YAW, "PID_I.Y", 0, 255 }
#define ADJ_VAR_DEF_D_ROLL { ADJ_VAR_D_ROLL, "PID_D.R", 0, 255 }
#define ADJ_VAR_DEF_D_PITCH { ADJ_VAR_D_PITCH, "PID_D.P", 0, 255 }
#define ADJ_VAR_DEF_D_YAW { ADJ_VAR_D_YAW, "PID_D.Y", 0, 255 }
#define ADJ_VAR_DEF_POWER_ROLL { ADJ_VAR_POWER_ROLL, "POWER.R", 0, 255 }
#define ADJ_VAR_DEF_POWER_PITCH { ADJ_VAR_POWER_PITCH, "POWER.P", 0, 255 }
#define ADJ_VAR_DEF_POWER_YAW { ADJ_VAR_POWER_YAW, "POWER.Y", 0, 255 }
#define ADJ_VAR_DEF_ACC_LIMIT { ADJ_VAR_ACC_LIMITER, "ACC_LIMIT", 0, 200}
#define ADJ_VAR_DEF_FOLLOW_SPEED_ROLL { ADJ_VAR_FOLLOW_SPEED_ROLL, "F_SPD.R", 0, 255 }
#define ADJ_VAR_DEF_FOLLOW_SPEED_PITCH { ADJ_VAR_FOLLOW_SPEED_PITCH, "F_SPD.P", 0, 255 }
#define ADJ_VAR_DEF_FOLLOW_SPEED_YAW { ADJ_VAR_FOLLOW_SPEED_YAW, "F_SPD.Y", 0, 255 }
#define ADJ_VAR_DEF_FOLLOW_LPF_ROLL { ADJ_VAR_FOLLOW_LPF_ROLL, "F_LPF.R", 0, 16 }
#define ADJ_VAR_DEF_FOLLOW_LPF_PITCH { ADJ_VAR_FOLLOW_LPF_PITCH, "F_LPF.P", 0, 16 }
#define ADJ_VAR_DEF_FOLLOW_LPF_YAW { ADJ_VAR_FOLLOW_LPF_YAW, "F_LPF.Y", 0, 16 }
#define ADJ_VAR_DEF_RC_SPEED_ROLL { ADJ_VAR_RC_SPEED_ROLL, "RC_SPD.R", 0, 255 }
#define ADJ_VAR_DEF_RC_SPEED_PITCH { ADJ_VAR_RC_SPEED_PITCH, "RC_SPD.P", 0, 255 }
#define ADJ_VAR_DEF_RC_SPEED_YAW { ADJ_VAR_RC_SPEED_YAW, "RC_SPD.Y", 0, 255 }
#define ADJ_VAR_DEF_RC_LPF_ROLL { ADJ_VAR_RC_LPF_ROLL, "RC_LPF.R", 0, 16 }
#define ADJ_VAR_DEF_RC_LPF_PITCH { ADJ_VAR_RC_LPF_PITCH, "RC_LPF.P", 0, 16 }
#define ADJ_VAR_DEF_RC_LPF_YAW { ADJ_VAR_RC_LPF_YAW, "RC_LPF.Y", 0, 16 }
#define ADJ_VAR_DEF_RC_TRIM_ROLL { ADJ_VAR_RC_TRIM_ROLL, "RC_TRIM.R", -127, 127 }
#define ADJ_VAR_DEF_RC_TRIM_PITCH { ADJ_VAR_RC_TRIM_PITCH, "RC_TRIM.P", -127, 127 }
#define ADJ_VAR_DEF_RC_TRIM_YAW { ADJ_VAR_RC_TRIM_YAW, "RC_TRIM.Y", -127, 127 }
#define ADJ_VAR_DEF_RC_DEADBAND { ADJ_VAR_RC_DEADBAND, "RC_DBND", 0, 255 }
#define ADJ_VAR_DEF_RC_EXPO_RATE { ADJ_VAR_RC_EXPO_RATE, "RC_EXPO", 0, 100 }
#define ADJ_VAR_DEF_FOLLOW_MODE { ADJ_VAR_FOLLOW_MODE, "F.MODE", 0, 2 }
#define ADJ_VAR_DEF_FOLLOW_DEADBAND { ADJ_VAR_FOLLOW_DEADBAND, "F.DBND", 0, 255 }
#define ADJ_VAR_DEF_FOLLOW_EXPO_RATE { ADJ_VAR_FOLLOW_EXPO_RATE, "F.EXPO", 0, 100 }
#define ADJ_VAR_DEF_FOLLOW_ROLL_MIX_START { ADJ_VAR_FOLLOW_ROLL_MIX_START, "F.RMS", 0, 90 }
#define ADJ_VAR_DEF_FOLLOW_ROLL_MIX_RANGE { ADJ_VAR_FOLLOW_ROLL_MIX_RANGE, "F.RMR", 0, 90 }
#define ADJ_VAR_DEF_GYRO_TRUST { ADJ_VAR_GYRO_TRUST, "GYRO_TRUST", 1, 255}




#endif // __SBGC_adj_vars__