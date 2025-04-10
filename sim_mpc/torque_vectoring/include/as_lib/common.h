#ifndef __COMMON_H__
#define __COMMON_H__


typedef enum {
	ACCELERATION, SKIDPAD, AUTOCROSS, TRACKDRIVE, EBS_TEST, INSPECTION, MANUAL_DRIVING
}as_missions;

typedef enum {
	AS_OFF, AS_READY, AS_DRIVING, AS_FINISHED, AS_EMERGENCY, AS_MANUAL
}as_states;

typedef enum {
	BLUE_CONE, YELLOW_CONE, ORANGE_CONE, BIG_ORANGE_CONE, UNKNOWN_CONE
}as_cone_colors;

typedef enum {
	EBS_UNAVAILABLE, EBS_ARMED, EBS_ACTIVATED
}ebs_states;

typedef enum {
	ASSI_OFF, ASSI_Y_C, ASSI_Y_F, ASSI_B_C, ASSI_B_F, ASSI_MANUAL
}assi_states;

typedef enum {
	OPEN, CLOSED
}sdc_status;

#endif
