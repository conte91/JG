#define GLOBAL_VAR_EXTERN
#include <C5G/userCallback.h>
#include <stdio.h>
#include <string.h>

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
      ORLOPEN_sync_position(&current_joints[si_arm], ORL_SILENT, ORL_CNTRL01, si_arm);
      ORL_direct_kinematics(&current_position[si_arm],&current_joints[si_arm],ORL_SILENT, ORL_CNTRL01,si_arm);
      sprintf((char *)orl_sys_var.sysvar_name,"$ARM_DATA[%d].ARM_OVR",si_arm+1);
      orl_sys_var.ctype = ORL_INT;
      orl_sys_var.iv = 20;
      ORL_set_data(orl_sys_var, ORL_SILENT, ORL_CNTRL01);
    }
    return 1;

  }
  return 0;
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
int user_callback (int period)
{
  int res;
  char flag_new_modality[MAX_NUM_ARMS], flag_MustSetComplete[MAX_NUM_ARMS];
  float override = 0.01 * period;
  long mask;
  ORL_joint_value sx_jnt_pos;
  char s_modality[40];

  int armIndex=0;

  flag_new_modality[armIndex] = false;
  modality_old[armIndex] = modality_active[armIndex];
  modality_active[armIndex] = ORLOPEN_GetModeMasterAx(ORL_SILENT,ORL_CNTRL01, armIndex);
  mask = ORLOPEN_GetOpenMask( ORL_SILENT,ORL_CNTRL01,armIndex );
  if(modality_old[armIndex] != modality_active[armIndex])
  {
    flag_new_modality[armIndex] = true;
    decode_modality( (unsigned int)modality_active[armIndex], s_modality);
    printf("ARM %d Modality %d %s\n", armIndex+1,(unsigned int)modality_active[armIndex],s_modality);
  }
  else
  {
    flag_new_modality[armIndex] = false;
  }

  if (flag_MoveKeyboard[armIndex] && ( (modality_active[armIndex] != CRCOPEN_POS_RELATIVE) && (modality_active[armIndex] != CRCOPEN_POS_ABSOLUTE)))
  {
    flag_MoveKeyboard[armIndex] = false;
  }
  switch (modality_active[armIndex])
  {
    case CRCOPEN_LISTEN:
      ORLOPEN_sync_position(&current_joints[armIndex], ORL_SILENT, ORL_CNTRL01, armIndex);
      ORL_direct_kinematics(&current_position[armIndex],&current_joints[armIndex],ORL_SILENT, ORL_CNTRL01,armIndex);
      break;
    case CRCOPEN_POS_ABSOLUTE:
      if (cycle_active)
      {
        if ((flag_new_modality[armIndex]) && (modality_active[armIndex] == CRCOPEN_POS_ABSOLUTE))
        {
          move_cal_Sys(ORL_CNTRL01, armIndex);
        }
      }

      if (mask_moving_arms & (1<<armIndex))
        res = ORL_get_next_interpolation_step (&sx_jnt_pos, ORL_SILENT, ORL_CNTRL01, armIndex);
      else
        res = -42;

      if (res==0)
      {
        si_k[armIndex]++;
        memcpy(&current_joints[armIndex],&sx_jnt_pos,sizeof(ORL_joint_value));
      }
      else
      {
        if (res==-42) /*no active move*/
        {
          //flag_RunningMove[armIndex] = false;
        }
        else if (res==2) /*the movement has been finished*/
        {
          if(flag_RunningMove[armIndex]){
            flag_MustSetComplete[armIndex]=1;
          }
          printf("%d", flag_RunningMove[armIndex]);
          flag_RunningMove[armIndex] = false;
          printf("%d", flag_RunningMove[armIndex]);
          mask_moving_arms = mask_moving_arms & ~(1<<armIndex);
          printf("\n--------------------- END MOVE ARM_%d: %d step ----------------\n",armIndex+1,si_k[armIndex]);
          si_k[armIndex]=0;
          if (cycle_active)
          {
            flag_ExitFromOpen[armIndex] = true;
          }

        }
        memcpy(&sx_jnt_pos,&current_joints[armIndex],sizeof(ORL_joint_value));
        ORL_direct_kinematics(&current_position[armIndex],&current_joints[armIndex],ORL_SILENT, ORL_CNTRL01,armIndex);
      }
      ORLOPEN_set_absolute_pos_target_degree( &sx_jnt_pos, ORL_SILENT, ORL_CNTRL01, armIndex );
      break;
    default:
      break;
  }

  if (flag_ExitFromOpen[armIndex])
  {
    printf("Exiting from OPEN\n");
    ORLOPEN_ExitFromOpen( ORL_SILENT,  ORL_CNTRL01, armIndex);
    flag_ExitFromOpen[armIndex] = false;
  }
  if(flag_MustSetComplete[armIndex]){
    flag_MustSetComplete[armIndex]=0;
    flag_hasCompletedTheMovement[armIndex] = 1;
  }
  return ORLOPEN_RES_OK;
}
