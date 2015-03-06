#define GLOBAL_VAR_EXTERN
#include "demo_c5gopen.h"

void printLastReceived(void)
{
  int si_arm, res;
  ORL_joint_value sx_joint_pos;
  long sm_out_maskjnt;
  char s_modality[40];
  int modality;

  printf("mask_moving_arms %c %c %c %c\n",((mask_moving_arms & (1<<ORL_ARM1))?'1':'-'),((mask_moving_arms & (1<<ORL_ARM2))?'2':'-'),((mask_moving_arms & (1<<ORL_ARM3))?'3':'-'),((mask_moving_arms & (1<<ORL_ARM4))?'4':'-')   );
  res = ORLOPEN_GetPowerlinkState(ORL_SILENT);

  if (res == PWL_ACTIVE)
  {
    for (si_arm= 0;si_arm<MAX_NUM_ARMS;si_arm++)
    {
      memset(&sx_joint_pos,0x00,sizeof(ORL_joint_value));
      sm_out_maskjnt = ORLOPEN_GetOpenMask        ( ORL_SILENT,ORL_CNTRL01, si_arm );
      modality = ORLOPEN_GetModeMasterAx(ORL_SILENT, ORL_CNTRL01, si_arm);
      decode_modality( modality, s_modality);
      printf ("\n------ ARM %d MODE %d %s - %s mask %x ------------ \n", 
             si_arm+1,
             modality,s_modality,
             ((ORLOPEN_GetStatusMasterAx(ORL_SILENT, ORL_CNTRL01, si_arm)==4)?"DRIVE_ON":"DRIVEOFF"),
             (unsigned int)sm_out_maskjnt
      );
      ORLOPEN_get_pos_target_mr (&sx_joint_pos,&sm_out_maskjnt, LAST_MESS, ORL_SILENT, ORL_CNTRL01, si_arm);
      printf ("POS_TARGET (mr %d)\t %f\t%f\t%f\t%f\t%f\t%f\t%f\n",(int)sx_joint_pos.unit_type,sx_joint_pos.value[ORL_AX1],sx_joint_pos.value[ORL_AX2],sx_joint_pos.value[ORL_AX3],sx_joint_pos.value[ORL_AX4], sx_joint_pos.value[ORL_AX5],sx_joint_pos.value[ORL_AX6],sx_joint_pos.value[ORL_AX7]);
      ORLOPEN_get_speed_target_mrpm (&sx_joint_pos,&sm_out_maskjnt, LAST_MESS, ORL_SILENT, ORL_CNTRL01, si_arm);
      printf ("SPD_TARGET (mr/min %d)\t %f\t%f\t%f\t%f\t%f\t%f\t%f\n",(int)sx_joint_pos.unit_type,sx_joint_pos.value[ORL_AX1],sx_joint_pos.value[ORL_AX2],sx_joint_pos.value[ORL_AX3],sx_joint_pos.value[ORL_AX4], sx_joint_pos.value[ORL_AX5],sx_joint_pos.value[ORL_AX6],sx_joint_pos.value[ORL_AX7]);
      ORLOPEN_get_pos_measured_mr (&sx_joint_pos,&sm_out_maskjnt, LAST_MESS, ORL_SILENT, ORL_CNTRL01, si_arm);
      printf ("POS_MEASURED (mr %d)\t %f\t%f\t%f\t%f\t%f\t%f\t%f\n",(int)sx_joint_pos.unit_type,sx_joint_pos.value[ORL_AX1],sx_joint_pos.value[ORL_AX2],sx_joint_pos.value[ORL_AX3],sx_joint_pos.value[ORL_AX4], sx_joint_pos.value[ORL_AX5],sx_joint_pos.value[ORL_AX6],sx_joint_pos.value[ORL_AX7]);
      ORLOPEN_get_speed_measured_mrpm (&sx_joint_pos,&sm_out_maskjnt, LAST_MESS, ORL_SILENT, ORL_CNTRL01, si_arm);
      printf ("SPD_MEASURED (mr/min %d) %f\t%f\t%f\t%f\t%f\t%f\t%f\n",(int)sx_joint_pos.unit_type,sx_joint_pos.value[ORL_AX1],sx_joint_pos.value[ORL_AX2],sx_joint_pos.value[ORL_AX3],sx_joint_pos.value[ORL_AX4], sx_joint_pos.value[ORL_AX5],sx_joint_pos.value[ORL_AX6],sx_joint_pos.value[ORL_AX7]);
      ORLOPEN_get_current_measured (&sx_joint_pos,&sm_out_maskjnt, LAST_MESS, ORL_SILENT, ORL_CNTRL01, si_arm);
      printf ("CUR_MEASURED (amp %d)\t %f\t%f\t%f\t%f\t%f\t%f\t%f\n",(int)sx_joint_pos.unit_type,sx_joint_pos.value[ORL_AX1],sx_joint_pos.value[ORL_AX2],sx_joint_pos.value[ORL_AX3],sx_joint_pos.value[ORL_AX4], sx_joint_pos.value[ORL_AX5],sx_joint_pos.value[ORL_AX6],sx_joint_pos.value[ORL_AX7]);
      ORLOPEN_get_pos_measured(&current_joints[si_arm],&current_position[si_arm], LAST_MESS, ORL_SILENT, ORL_CNTRL01, si_arm);
      printf("POS_MIS J (%d)\t%f %f %f %f %f %f %f\n", (int)current_joints[si_arm].unit_type,current_joints[si_arm].value[ORL_AX1],current_joints[si_arm].value[ORL_AX2],current_joints[si_arm].value[ORL_AX3],current_joints[si_arm].value[ORL_AX4],current_joints[si_arm].value[ORL_AX5],current_joints[si_arm].value[ORL_AX6],current_joints[si_arm].value[ORL_AX7]);
      printf("POS_MIS P (%d)\t%f %f %f %f %f %f %s\n", (int)current_position[si_arm].unit_type,current_position[si_arm].x,current_position[si_arm].y,current_position[si_arm].z,current_position[si_arm].a,current_position[si_arm].e,current_position[si_arm].r,current_position[si_arm].config_flags);
    }
  }
}

int initialize_Control_position ( void )
{
  int si_arm, res;
  long sm_out_maskjnt;
  ORL_System_Variable orl_sys_var;
  char s_modality[40];
  int modality;

  res = ORLOPEN_GetPowerlinkState(ORL_VERBOSE);

  if (res == PWL_ACTIVE)
  {
    for (si_arm= 0;si_arm<MAX_NUM_ARMS;si_arm++)
    {
      sm_out_maskjnt = ORLOPEN_GetOpenMask        ( ORL_SILENT,ORL_CNTRL01, si_arm );
      modality = ORLOPEN_GetModeMasterAx(ORL_SILENT, ORL_CNTRL01, si_arm);
      decode_modality( modality, s_modality);
      printf ("\n------ ARM %d MODE %d %s - %s mask %x ------------ \n", 
             si_arm+1,
             modality,s_modality,
             ((ORLOPEN_GetStatusMasterAx(ORL_SILENT, ORL_CNTRL01, si_arm)==4)?"DRIVE_ON":"DRIVEOFF"),
             (unsigned int)sm_out_maskjnt
      );
      ORLOPEN_sync_position(&current_joints[si_arm], ORL_SILENT, ORL_CNTRL01, si_arm);
      ORL_direct_kinematics(&current_position[si_arm],&current_joints[si_arm],ORL_SILENT, ORL_CNTRL01,si_arm);
      printf("ORLOPEN_sync_position J %f %f %f %f %f %f %f\n", current_joints[si_arm].value[ORL_AX1],current_joints[si_arm].value[ORL_AX2],current_joints[si_arm].value[ORL_AX3],current_joints[si_arm].value[ORL_AX4],current_joints[si_arm].value[ORL_AX5],current_joints[si_arm].value[ORL_AX6],current_joints[si_arm].value[ORL_AX7]);
      printf("                      P %f %f %f %f %f %f %s\n", current_position[si_arm].x,current_position[si_arm].y,current_position[si_arm].z,current_position[si_arm].a,current_position[si_arm].e,current_position[si_arm].r,current_position[si_arm].config_flags);
      sprintf((char *)orl_sys_var.sysvar_name,"$ARM_DATA[%d].ARM_OVR",si_arm+1);
      orl_sys_var.ctype = ORL_INT;
      orl_sys_var.iv = 20;
      ORL_set_data(orl_sys_var, ORL_SILENT, ORL_CNTRL01);
    }
    return 1;

  }
  return 0;
}

double function_sine(long long time, double ampiezza, double freq)
{
  /*double res = ampiezza * (float)sin( (double)(2.0 * pi * freq * ((double)time*4/10000.0)) );    Per freq in Hz*/
  double res = ampiezza * sin( freq * time * 0.0004 );

  return res;
}

double function_triangle(long long cnt, double peak, double time_peak)
{
  double res;
  double cnt_peak = time_peak/4.0*10000.0;

  if (cnt < cnt_peak)
    res = peak/cnt_peak * (double)cnt;
  else if (cnt < (cnt_peak*2))
   res = - peak/cnt_peak * (double)cnt + 2 * peak;
  else
    res = 0;

  return res;
}

double amplitude_modulation(long long cnt, double peak, double time_peak)
{
  double res;
  double pi = 3.14159265;
  double cnt_peak = time_peak/4.0*10000.0;

  if (cnt < cnt_peak*2)
    res = peak * sin(pi/2 * (cnt / cnt_peak));
  else
    res = 0;

  return res;
}

double frequency_modulation(long long cnt, double peak, double time_peak)
{
  double res;
  double pi = 3.14159265;
  double cnt_peak = time_peak/4.0*10000.0;

  if (cnt < cnt_peak*2)
    res = peak * sin(pi/2 * (cnt / cnt_peak));
  else
    res = 0;

  return res;
}


void apply_delta_position(delta_position_t * px_Delta_position, float override )
{
  px_Delta_position->real.unit_type = ORL_CART_POSITION;
  if (    ( (px_Delta_position->real.x < px_Delta_position->ideal.x) && (px_Delta_position->speed.x > 0))
      ||  ( (px_Delta_position->real.x > px_Delta_position->ideal.x) && (px_Delta_position->speed.x < 0)) )
    px_Delta_position->real.x += px_Delta_position->speed.x*override;
  if (    ( (px_Delta_position->real.y < px_Delta_position->ideal.y) && (px_Delta_position->speed.y > 0))
      ||  ( (px_Delta_position->real.y > px_Delta_position->ideal.y) && (px_Delta_position->speed.y < 0)) )
    px_Delta_position->real.y += px_Delta_position->speed.y*override;
  if (    ( (px_Delta_position->real.z < px_Delta_position->ideal.z) && (px_Delta_position->speed.z > 0))
      ||  ( (px_Delta_position->real.z > px_Delta_position->ideal.z) && (px_Delta_position->speed.z < 0)) )
    px_Delta_position->real.z += px_Delta_position->speed.z*override;
  if (    ( (px_Delta_position->real.a < px_Delta_position->ideal.a) && (px_Delta_position->speed.a > 0))
      ||  ( (px_Delta_position->real.a > px_Delta_position->ideal.a) && (px_Delta_position->speed.a < 0)) )
    px_Delta_position->real.a += px_Delta_position->speed.a*override;
  if (    ( (px_Delta_position->real.e < px_Delta_position->ideal.e) && (px_Delta_position->speed.e > 0))
      ||  ( (px_Delta_position->real.e > px_Delta_position->ideal.e) && (px_Delta_position->speed.e < 0)) )
    px_Delta_position->real.e += px_Delta_position->speed.e*override;
  if (    ( (px_Delta_position->real.r < px_Delta_position->ideal.r) && (px_Delta_position->speed.r > 0))
      ||  ( (px_Delta_position->real.r > px_Delta_position->ideal.r) && (px_Delta_position->speed.r < 0)) )
    px_Delta_position->real.r += px_Delta_position->speed.r*override;
}

int move_cal_Sys(int idx_cntrl, int idx_arm)
{
  ORL_joint_value target_jnt;
  memset(&target_jnt,0x00,sizeof(ORL_joint_value));
  int type_move = ORL_TRJNT;
  int sl_d = ORLOPEN_RES_OK;

  target_jnt.unit_type = ORL_POSITION_LINK_DEGREE;
  target_jnt.value[ORL_AX1] = 0.0;
  target_jnt.value[ORL_AX2] = 0.0;
  target_jnt.value[ORL_AX3] = -90.0;
  target_jnt.value[ORL_AX4] = 0.0;
  target_jnt.value[ORL_AX5] = 90.0;
  target_jnt.value[ORL_AX6] = 0.0;
  target_jnt.value[ORL_AX7] = 0.0;

  sl_d = ORL_set_move_parameters(ORL_NO_FLY, ORL_WAIT, ORL_FLY_NORMAL, type_move, NULL, &target_jnt, ORL_SILENT, idx_cntrl, idx_arm);
  mask_moving_arms = mask_moving_arms | (1<<idx_arm);
  flag_RunningMove[idx_arm] = true;
  printf("--> Move acquired.\n");

  if (type_move == ORL_TRJNT)
    printf("--> Corresponding PDL2 move line: MOVE ARM[%d] JOINT TO {%f, %f, %f, %f, %f, %f, %f}\n",idx_arm+1,target_jnt.value[ORL_AX1],target_jnt.value[ORL_AX2],target_jnt.value[ORL_AX3],target_jnt.value[ORL_AX4],target_jnt.value[ORL_AX5],target_jnt.value[ORL_AX6],target_jnt.value[ORL_AX7]);
  else
    printf("--> Corresponding PDL2 move line: MOVE ARM[%d] LINEAR TO {%f, %f, %f, %f, %f, %f, %f}\n",idx_arm+1,target_jnt.value[ORL_AX1],target_jnt.value[ORL_AX2],target_jnt.value[ORL_AX3],target_jnt.value[ORL_AX4],target_jnt.value[ORL_AX5],target_jnt.value[ORL_AX6],target_jnt.value[ORL_AX7]);

  si_k[idx_arm] = 0;
  printf("[MSG] Robot is about to move...\n");
  return sl_d;

}

int move_Arm(ORL_joint_value* px_target_jnt,int type_move,int idx_cntrl, int idx_arm)
{
  int sl_d = ORLOPEN_RES_OK;

  sl_d = ORL_set_move_parameters(ORL_NO_FLY, ORL_WAIT, ORL_FLY_NORMAL, type_move, NULL, px_target_jnt, ORL_SILENT, idx_cntrl, idx_arm);
  if ( sl_d != ORLOPEN_RES_OK)
  {
    printf("--> Errore prec %d\n",sl_d);
    return sl_d;
  }
  mask_moving_arms = mask_moving_arms | (1<<idx_arm);
  flag_RunningMove[idx_arm] = true;
  printf("--> Move acquired.\n");

  if (type_move == ORL_TRJNT)
    printf("--> Corresponding PDL2 move line: MOVE ARM[%d] JOINT TO {%f, %f, %f, %f, %f, %f, %f}\n",idx_arm+1,px_target_jnt->value[ORL_AX1],px_target_jnt->value[ORL_AX2],px_target_jnt->value[ORL_AX3],px_target_jnt->value[ORL_AX4],px_target_jnt->value[ORL_AX5],px_target_jnt->value[ORL_AX6],px_target_jnt->value[ORL_AX7]);
  else
    printf("--> Corresponding PDL2 move line: MOVE ARM[%d] LINEAR TO {%f, %f, %f, %f, %f, %f, %f}\n",idx_arm+1,px_target_jnt->value[ORL_AX1],px_target_jnt->value[ORL_AX2],px_target_jnt->value[ORL_AX3],px_target_jnt->value[ORL_AX4],px_target_jnt->value[ORL_AX5],px_target_jnt->value[ORL_AX6],px_target_jnt->value[ORL_AX7]);

  si_k[idx_arm] = 0;
  printf("[MSG] Robot is about to move...\n");
  return sl_d;

}


void decode_modality( int si_modality, char* string)
{

  switch(si_modality)
  {
  case CRCOPEN_LISTEN:
    sprintf(string,"CRCOPEN_LISTEN");
    break;
  case CRCOPEN_POS_ABSOLUTE:
    sprintf(string,"CRCOPEN_POS_ABSOLUTE");
    break;
  case CRCOPEN_POS_RELATIVE:
    sprintf(string,"CRCOPEN_POS_RELATIVE");
    break;
  case CRCOPEN_POS_ADDITIVE:
    sprintf(string,"CRCOPEN_POS_ADDITIVE");
    break;
  case CRCOPEN_POS_ADDITIVE_SB:
    sprintf(string,"CRCOPEN_POS_ADDITIVE_SB");
    break;
  case CRCOPEN_POS_ADDITIVE_SBE:
    sprintf(string,"CRCOPEN_POS_ADDITIVE_SBE");
    break;
  default:
    sprintf(string,"--");
    break;
  }
}
