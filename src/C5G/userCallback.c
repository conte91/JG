#define GLOBAL_VAR_EXTERN
#include <C5G/userCallback.h>

int user_callback (int period)
{
  int res, si_arm;
  char flag_new_modality[MAX_NUM_ARMS];
  float override = 0.01 * period;
  double sine_amplitude, sine_freq;
  long delay = (long)(0.2/4.0*10000.0);/* 0.2 sec*/
  long mask;
  ORL_joint_value sx_jnt_pos;
  char s_modality[40];

  for (si_arm=0;si_arm<MAX_NUM_ARMS;si_arm++)
  {
    flag_new_modality[si_arm] = false;
    modality_old[si_arm] = modality_active[si_arm];
    modality_active[si_arm] = ORLOPEN_GetModeMasterAx(ORL_SILENT,ORL_CNTRL01, si_arm);
    mask = ORLOPEN_GetOpenMask( ORL_SILENT,ORL_CNTRL01,si_arm );
    if(modality_old[si_arm] != modality_active[si_arm])
    {
      flag_new_modality[si_arm] = true;
      decode_modality( (unsigned int)modality_active[si_arm], s_modality);
      printf("ARM %d Modality %d %s\n", si_arm+1,(unsigned int)modality_active[si_arm],s_modality);
    }
    else
    {
      flag_new_modality[si_arm] = false;
    }

    if (flag_MoveKeyboard[si_arm] && ( (modality_active[si_arm] != CRCOPEN_POS_RELATIVE) && (modality_active[si_arm] != CRCOPEN_POS_ABSOLUTE)))
    {
      flag_MoveKeyboard[si_arm] = false;
    }
    switch (modality_active[si_arm])
    {
      case CRCOPEN_LISTEN:
        ORLOPEN_sync_position(&current_joints[si_arm], ORL_SILENT, ORL_CNTRL01, si_arm);
        ORL_direct_kinematics(&current_position[si_arm],&current_joints[si_arm],ORL_SILENT, ORL_CNTRL01,si_arm);
        break;
      case CRCOPEN_POS_ABSOLUTE:
        if (cycle_active)
        {
          if ((flag_new_modality[si_arm]) && (modality_active[si_arm] == CRCOPEN_POS_ABSOLUTE))
          {
            move_cal_Sys(ORL_CNTRL01, si_arm);
          }
        }

        if (mask_moving_arms & (1<<si_arm))
          res = ORL_get_next_interpolation_step (&sx_jnt_pos, ORL_SILENT, ORL_CNTRL01, si_arm);
        else
          res = -42;

        if (res==0)
        {
          si_k[si_arm]++;
          memcpy(&current_joints[si_arm],&sx_jnt_pos,sizeof(ORL_joint_value));
          /*printf("[INTL] 0 %d step [%3d]: %f,%f,%f,%f,%f,%f,%f\n",ORL_ARM1+si_arm, si_k[si_arm], current_joints[si_arm].value[ORL_AX1],current_joints[si_arm].value[ORL_AX2],current_joints[si_arm].value[ORL_AX3],current_joints[si_arm].value[ORL_AX4],current_joints[si_arm].value[ORL_AX5],current_joints[si_arm].value[ORL_AX6],current_joints[si_arm].value[ORL_AX7]);*/
        }
        else
        {
          if (res==-42) /*no active move*/
          {
            flag_RunningMove[si_arm] = false;
          }
          else if (res==2) /*the movement has been finished*/
          {
            flag_RunningMove[si_arm] = false;
            mask_moving_arms = mask_moving_arms & !(1<<si_arm);
            printf("\n--------------------- END MOVE ARM_%d: %d step ----------------\n",si_arm+1,si_k[si_arm]);
            si_k[si_arm]=0;
            if (cycle_active)
            {
              flag_ExitFromOpen[si_arm] = true;
            }

          }
          memcpy(&sx_jnt_pos,&current_joints[si_arm],sizeof(ORL_joint_value));
          ORL_direct_kinematics(&current_position[si_arm],&current_joints[si_arm],ORL_SILENT, ORL_CNTRL01,si_arm);
        }
        ORLOPEN_set_absolute_pos_target_degree( &sx_jnt_pos, ORL_SILENT, ORL_CNTRL01, si_arm );
        break;
      case CRCOPEN_POS_RELATIVE:
#if 0
        if ((flag_new_modality[si_arm]) && (modality_active[si_arm] == CRCOPEN_POS_RELATIVE))
        {
          cnt_modality[si_arm] = - delay;
          memset(&Delta_Position[si_arm].ideal,0x00,sizeof(ORL_cartesian_position));
          memset(&Delta_Position[si_arm].real,0x00,sizeof(ORL_cartesian_position));
          memset(&Delta_Position[si_arm].speed,0x00,sizeof(ORL_cartesian_position));
        }
        memset(&sx_jnt_pos,0x00,sizeof(ORL_joint_value));
        if ( (si_arm == ORL_ARM1) || (si_arm == ORL_ARM2) )
        {
        }
        else if (si_arm == ORL_ARM3)
        {
          sx_jnt_pos.unit_type = ORL_POSITION_MOTORROUNDS;
          if (cnt_modality[si_arm] < 0)
            sine_amplitude = 0.0;
          else
            sine_amplitude = function_triangle(cnt_modality[si_arm], 6.0, 20.0);
          /*apply_delta_position(&Delta_Position[si_arm],override);
            sx_jnt_pos.value[ORL_AX1] = Delta_Position[si_arm].real.x;*/
          sx_jnt_pos.value[ORL_AX1] = function_sine(cnt_modality[si_arm],sine_amplitude,0.1);
          cnt_modality[si_arm] += period;
          ORLOPEN_set_relative_pos_target_mr(&sx_jnt_pos,&mask,ORL_SILENT, ORL_CNTRL01, si_arm);
        }

        if (cycle_active)
        {
          if ((sine_freq == 0.0) && (cnt_modality[si_arm] > 12500)/*5 seconds*/ /*durata_secondi/4.0*10000.0*/)
            flag_ExitFromOpen[si_arm] = true;
        }

        break;

      case CRCOPEN_POS_ADDITIVE:
      case CRCOPEN_POS_ADDITIVE_SB:
      case CRCOPEN_POS_ADDITIVE_SBE:
        if ((flag_new_modality[si_arm]) && (modality_active[si_arm] == CRCOPEN_POS_ADDITIVE))
        {
          cnt_modality[si_arm] = - delay;
          memset(&Delta_Position[si_arm].ideal,0x00,sizeof(ORL_cartesian_position));
          memset(&Delta_Position[si_arm].real,0x00,sizeof(ORL_cartesian_position));
          memset(&Delta_Position[si_arm].speed,0x00,sizeof(ORL_cartesian_position));
        }
        memset(&sx_jnt_pos,0x00,sizeof(ORL_joint_value));
        if ( (si_arm == ORL_ARM1) || (si_arm == ORL_ARM2))
        {

        }
        else if (si_arm == ORL_ARM3)
        {
          sx_jnt_pos.unit_type = ORL_POSITION_MOTORROUNDS;
          /*apply_delta_position(&Delta_Position[si_arm],override);
            sx_jnt_pos.value[ORL_AX1] = Delta_Position[si_arm].real.x;*/
          if (cnt_modality[si_arm] < 0)
            sine_amplitude = 0.0;
          else
            sine_amplitude = function_triangle(cnt_modality[si_arm], 6.0, 20.0);
          sx_jnt_pos.value[ORL_AX1] = function_sine(cnt_modality[si_arm],sine_amplitude,0.1);
          cnt_modality[si_arm] += period;
          ORLOPEN_set_additive_pos_target_mr(&sx_jnt_pos,&mask,ORL_SILENT, ORL_CNTRL01, si_arm);
        }

        if (cycle_active)
        {
          if ((sine_freq == 0.0) && (cnt_modality[si_arm] > 12500)/*5 seconds*/ /*durata_secondi/4.0*10000.0*/)
            flag_ExitFromOpen[si_arm] = true;
        }

        break;

#endif
      default:
        break;
    }

    if (flag_ExitFromOpen[si_arm])
    {
      ORLOPEN_ExitFromOpen( ORL_SILENT,  ORL_CNTRL01, si_arm);
      flag_ExitFromOpen[si_arm] = false;
    }
  }
  return ORLOPEN_RES_OK;
}
