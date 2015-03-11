

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <pthread.h>

#include "eORL.h"

#ifdef GLOBAL_VAR_EXTERN
#define EXTRN  extern
#else
#define EXTRN
#endif

#define false      0
#define true       1
#define LAST_MESS   0

/*#define RML*/

#ifdef RML
#define MAX_NUM_ARMS    3
#else
#define MAX_NUM_ARMS    1
#endif

#define STRING_IP_CNTRL        "172.22.178.100"
#define STRING_SYS_ID          "CNTRLC5G_100"


#define SINUSOIDE_MR            1
#define SINUSOIDE_DEGREE        2
#define SINUSOIDE_CART          3
#define KEYBOARD_JOYSTICK_MR    4
#define KEYBOARD_JOYSTICK_CART  5
#define TYPE_DEMO  SINUSOIDE_MR

typedef struct
{
  ORL_cartesian_position ideal;
  ORL_cartesian_position real;
  ORL_cartesian_position speed;
}delta_position_t;


/** TODO EXTRN==#define EXTRN extern? .-. */
EXTRN ORL_joint_value current_joints[MAX_NUM_ARMS];
EXTRN ORL_cartesian_position current_position[MAX_NUM_ARMS];

EXTRN delta_position_t Delta_Position[MAX_NUM_ARMS];

EXTRN char flag_RunningMove[MAX_NUM_ARMS];
EXTRN char flag_ExitFromOpen[MAX_NUM_ARMS];
EXTRN char flag_MoveKeyboard[MAX_NUM_ARMS];
EXTRN unsigned int modality_active[MAX_NUM_ARMS];
EXTRN unsigned int modality_old[MAX_NUM_ARMS];

EXTRN long long        cnt_modality[MAX_NUM_ARMS];
EXTRN int         si_k[MAX_NUM_ARMS];
EXTRN int         mask_moving_arms;
EXTRN int         cycle_active;

int user_callback (int);

/*test_crc_open_util.c*/
int initialize_Control_position (void);
void loopConsole (void);
void printLastReceived(void);
void apply_delta_position(delta_position_t * Delta_position, float override );
double function_sine(long long time, double ampiezza, double freq);
double function_triangle(long long cnt, double peak, double time_peak);
double frequency_modulation(long long cnt, double peak, double time_peak);
double amplitude_modulation(long long cnt, double peak, double time_peak);
int move_cal_Sys(int idx_cntrl, int idx_arm);
int move_Arm(ORL_joint_value* px_target_jnt,int type_move,int idx_cntrl, int idx_arm);
void decode_modality( int si_modality, char* string);
