#include "keyboard.h"

#define GLOBAL_VAR_EXTERN
#include "demo_c5gopen.h"


void loopConsole (void)
{
  int  i, si_timeout = 0, input_from_user_type, input_from_user_type_end_point, idx_arm;
  unsigned long sm_maskjnt;
  float delta = 0.5;
  char a_1, a_2, a_3, keep_on_running = true,
       f_1_command = false,
       f_2_arm_to_move = false,
       f_2_arm_to_exit = false,
       f_2_read_sys_var_name = false,
       f_2_modifie_sys_var_name = false,
       f_3_type_move = false,
       f_3_read_sys_var_type = false,
       f_3_modifie_sys_var_type = false,
       f_4_modifie_sys_var_value = false,
       f_4_set_move_parameters = false,
       f_4_read_sys_var = false,
       f_2_loop_keyboard = false,
       temp[80];

  ORL_cartesian_position  target_pos, temp_pos;
  ORL_joint_value         target_jnt, temp_joints;
  ORL_System_Variable     orl_sys_var;

  memset(temp,0x00,80);

  while (keep_on_running)
  {
    f_1_command = true;

    if (f_1_command)
    {
      printf("--> Do you want to move [J]oint, [L]inear? \n");
      printf("(Press O to print the last message received from the C5G)\n");
      printf("(Press E to exit from Open)\n");
      printf("(Press C to close the application)\n");
      printf("(Press R to read system variables from configuration file)\n");
      printf("(Press W to modify system variables)\n");
#if ( ( TYPE_DEMO == KEYBOARD_JOYSTICK_MR  ) || ( TYPE_DEMO == KEYBOARD_JOYSTICK_CART  ))
      printf("(Press T to move the robot with the Keyboard)\n");
#endif
      input_console(temp,sizeof(temp));
      a_1    = temp[0];
      switch (a_1)
      {
      case 'J':
      case 'j':
        input_from_user_type = ORL_TRJNT;
        f_1_command = false;
        f_2_arm_to_move = true;
        break;
      case 'L':
      case 'l':
        input_from_user_type = ORL_TRCARLIN;
        f_1_command = false;
        f_2_arm_to_move = true;
        break;
      case 'T':
      case 't':
        f_1_command = false;
        f_2_loop_keyboard = true;
        break;
      case 'E':
      case 'e':
        f_1_command = false;
        f_2_arm_to_exit = true;
        break;
      case 'R':
      case 'r':
        memset(&orl_sys_var,0x00,sizeof(ORL_System_Variable));
        f_1_command = false;
        f_2_read_sys_var_name = true;
        break;
      case 'W':
      case 'w':
        memset(&orl_sys_var,0x00,sizeof(ORL_System_Variable));
        f_1_command = false;
        f_2_modifie_sys_var_name = true;
        break;
      case 'C':
      case 'c':
        flag_ExitFromOpen[ORL_ARM1] = true;
        flag_ExitFromOpen[ORL_ARM2] = true;
        flag_ExitFromOpen[ORL_ARM3] = true;
        printf("Perform a DriveOFF on the TeachPendent\n");
        sleep(6);
        ORLOPEN_StopCommunication( ORL_SILENT);
        sleep(4);
        keep_on_running = false;
        break;
      case 'O':
      case 'o':
        printLastReceived();
        break;
      default:
        printf("--! Invalid value!\n");
        f_1_command = true;
        break;
      }

    }

    if (f_2_arm_to_move)
    {
#if (MAX_NUM_ARMS > 1)
      printf("--> Which Arm o you want to move [1], [2], [3]? \n");
      input_console(temp,sizeof(temp));
      a_3    = temp[0];
      switch (a_3)
      {
      case '1':
        idx_arm = ORL_ARM1;
        f_2_arm_to_move = false;
        f_3_type_move = true;
        break;
      case '2':
        idx_arm = ORL_ARM2;
        f_2_arm_to_move = false;
        f_3_type_move = true;
        break;
      case '3':
        idx_arm = ORL_ARM3;
        f_2_arm_to_move = false;
        f_3_type_move = true;
        break;
      case '4':
        idx_arm = ORL_ARM4;
        f_2_arm_to_move = false;
        f_3_type_move = true;
        break;
      default:
        idx_arm = -1;
        printf("--! Invalid value!\n");
        f_1_command = true;
        f_2_arm_to_move = false;
        f_3_type_move = false;
        break;
      }
#else
      idx_arm = ORL_ARM1;
      f_2_arm_to_move = false;
      f_3_type_move = true;
#endif
    }

    if (f_2_arm_to_exit)
    {
#if (MAX_NUM_ARMS > 1)
      printf("--> Which Arm o you want to exit from open [1], [2], [3], [A]ll? \n");
      input_console(temp,sizeof(temp));
      a_3    = temp[0];
      switch (a_3)
      {
      case '1':
        flag_ExitFromOpen[ORL_ARM1] = true;
        f_2_arm_to_exit = false;
        f_1_command = true;
        break;
      case '2':
        flag_ExitFromOpen[ORL_ARM2] = true;
        f_2_arm_to_exit = false;
        f_1_command = true;
        break;
      case '3':
        flag_ExitFromOpen[ORL_ARM3] = true;
        f_2_arm_to_exit = false;
        f_1_command = true;
        break;
      case '4':
        flag_ExitFromOpen[ORL_ARM4] = true;
        f_2_arm_to_exit = false;
        f_1_command = true;
        break;
      case 'a':
      case 'A':
        flag_ExitFromOpen[ORL_ARM1] = true;
        flag_ExitFromOpen[ORL_ARM2] = true;
        flag_ExitFromOpen[ORL_ARM3] = true;
        flag_ExitFromOpen[ORL_ARM4] = true;
        f_2_arm_to_exit = false;
        f_1_command = true;
        break;
      default:
        idx_arm = -1;
        printf("--! Invalid value!\n");
        f_2_arm_to_exit = false;
        f_1_command = true;
        break;
      }
#else
      flag_ExitFromOpen[ORL_ARM1] = true;
      f_2_arm_to_exit = false;
      f_1_command = true;
#endif
    }

    if (f_2_loop_keyboard)
    {
#if (MAX_NUM_ARMS > 1)
      printf("Use Keyboard for moving the robot\n");
      printf("     ARM1         ARM2         ARM3\n");
      printf("   x+ y+ z+     x+ y+ z+         + \n");
      printf("   A  S  D      G  H  J          K \n\n");
      printf("   x- y- z-     x- y- z-         - \n");
      printf("   a  s  c      g  h  j          M \n\n");
      printf("   a+ e+ r+     a+ e+ r+         + \n");
      printf("   Z  X  C      V  B  N          K \n\n");
      printf("   a+ e+ r+     a+ e+ r+         + \n");
      printf("   z  x  c      v  b  n          M \n\n");
#else
      printf("Use Keyboard for moving the robot\n");
      printf("     ARM1     \n");
      printf("   x+ y+ z+   \n");
      printf("   A  S  D    \n\n");
      printf("   x- y- z-   \n");
      printf("   a  s  c    \n\n");
      printf("   a+ e+ r+   \n");
      printf("   Z  X  C    \n\n");
      printf("   a+ e+ r+   \n");
      printf("   z  x  c    \n\n");
#endif
      printf("L to stop the robot\n\n");
      printf("Q to return at the main menu\n");

      while(f_2_loop_keyboard)
      {
        if(     (modality_active[ORL_ARM1] == CRCOPEN_POS_RELATIVE) || (modality_active[ORL_ARM1] == CRCOPEN_POS_ADDITIVE) || (modality_active[ORL_ARM1] == CRCOPEN_POS_ADDITIVE_SB)
          )
        {
          /* TODO: non compila in c++
           * a_3 = getch();
           */
          a_3 = 'Q';
          switch (a_3)
          {
          /*ARM1 x+*/
          case 'L':
          case 'l':
            Delta_Position[ORL_ARM1].speed.x = 0.0;
            Delta_Position[ORL_ARM1].ideal.x = Delta_Position[ORL_ARM1].real.x;
            Delta_Position[ORL_ARM1].speed.y = 0.0;
            Delta_Position[ORL_ARM1].ideal.y = Delta_Position[ORL_ARM1].real.y;
            Delta_Position[ORL_ARM1].speed.z = 0.0;
            Delta_Position[ORL_ARM1].ideal.z = Delta_Position[ORL_ARM1].real.z;
            Delta_Position[ORL_ARM1].speed.a = 0.0;
            Delta_Position[ORL_ARM1].ideal.a = Delta_Position[ORL_ARM1].real.a;
            Delta_Position[ORL_ARM1].speed.e = 0.0;
            Delta_Position[ORL_ARM1].ideal.e = Delta_Position[ORL_ARM1].real.e;
            Delta_Position[ORL_ARM1].speed.r = 0.0;
            Delta_Position[ORL_ARM1].ideal.r = Delta_Position[ORL_ARM1].real.r;
            Delta_Position[ORL_ARM2].speed.x = 0.0;
            Delta_Position[ORL_ARM2].ideal.x = Delta_Position[ORL_ARM2].real.x;
            Delta_Position[ORL_ARM2].speed.y = 0.0;
            Delta_Position[ORL_ARM2].ideal.y = Delta_Position[ORL_ARM2].real.y;
            Delta_Position[ORL_ARM2].speed.z = 0.0;
            Delta_Position[ORL_ARM2].ideal.z = Delta_Position[ORL_ARM2].real.z;
            Delta_Position[ORL_ARM2].speed.a = 0.0;
            Delta_Position[ORL_ARM2].ideal.a = Delta_Position[ORL_ARM2].real.a;
            Delta_Position[ORL_ARM2].speed.e = 0.0;
            Delta_Position[ORL_ARM2].ideal.e = Delta_Position[ORL_ARM2].real.e;
            Delta_Position[ORL_ARM2].speed.r = 0.0;
            Delta_Position[ORL_ARM2].ideal.r = Delta_Position[ORL_ARM2].real.r;
            Delta_Position[ORL_ARM3].speed.x = 0.0;
            Delta_Position[ORL_ARM3].ideal.x = Delta_Position[ORL_ARM3].real.x;
            break;
          /*ARM1 x+*/
          case 'A':
             Delta_Position[ORL_ARM1].speed.x = 1.0;
            Delta_Position[ORL_ARM1].ideal.x += delta;
            break;
          /*ARM1 x-*/
          case 'a':
            Delta_Position[ORL_ARM1].speed.x = - 1.0;
            Delta_Position[ORL_ARM1].ideal.x -= delta;
            break;
          /*ARM1 y+*/
          case 'S':
            Delta_Position[ORL_ARM1].speed.y = 1.0;
            Delta_Position[ORL_ARM1].ideal.y += delta;
            break;
          /*ARM1 y-*/
          case 's':
            Delta_Position[ORL_ARM1].speed.y = - 1.0;
            Delta_Position[ORL_ARM1].ideal.y -= delta;
            break;
          /*ARM1 z+*/
          case 'D':
            Delta_Position[ORL_ARM1].speed.z = 1.0;
            Delta_Position[ORL_ARM1].ideal.z += delta;
            break;
          /*ARM1 z-*/
          case 'd':
            Delta_Position[ORL_ARM1].speed.z = - 1.0;
            Delta_Position[ORL_ARM1].ideal.z -= delta;
            break;

          /*ARM1 a+*/
          case 'Z':
            Delta_Position[ORL_ARM1].speed.a = 1.0;
            Delta_Position[ORL_ARM1].ideal.a += delta;
            break;
          /*ARM1 a-*/
          case 'z':
            Delta_Position[ORL_ARM1].speed.a = - 1.0;
            Delta_Position[ORL_ARM1].ideal.a -= delta;
            break;
          /*ARM1 e+*/
          case 'X':
            Delta_Position[ORL_ARM1].speed.e = 1.0;
            Delta_Position[ORL_ARM1].ideal.e += delta;
            break;
          /*ARM1 e-*/
          case 'x':
            Delta_Position[ORL_ARM1].speed.e = - 1.0;
            Delta_Position[ORL_ARM1].ideal.e -= delta;
            break;
          /*ARM1 r+*/
          case 'C':
            Delta_Position[ORL_ARM1].speed.r = 1.0;
            Delta_Position[ORL_ARM1].ideal.r += delta;
            break;
          /*ARM1 r-*/
          case 'c':
            Delta_Position[ORL_ARM1].speed.r = - 1.0;
            Delta_Position[ORL_ARM1].ideal.r -= delta;
            break;

          /*ARM2 x+*/
          case 'G':
            Delta_Position[ORL_ARM2].speed.x = 1.0;
            Delta_Position[ORL_ARM2].ideal.x += delta;
            break;
          /*ARM2 x-*/
          case 'g':
            Delta_Position[ORL_ARM2].speed.x = - 1.0;
            Delta_Position[ORL_ARM2].ideal.x -= delta;
            break;
          /*ARM2 y+*/
          case 'H':

            Delta_Position[ORL_ARM2].speed.y = 1.0;
            Delta_Position[ORL_ARM2].ideal.y += delta;
            break;
          /*ARM2 y-*/
          case 'h':
            Delta_Position[ORL_ARM2].speed.y = - 1.0;
            Delta_Position[ORL_ARM2].ideal.y -= delta;
            break;
          /*ARM2 z+*/
          case 'J':
            Delta_Position[ORL_ARM2].speed.z = 1.0;
            Delta_Position[ORL_ARM2].ideal.z += delta;
            break;
          /*ARM2 z-*/
          case 'j':
            Delta_Position[ORL_ARM2].speed.z = - 1.0;
            Delta_Position[ORL_ARM2].ideal.z -= delta;
            break;

          /*ARM2 a+*/
          case 'V':
            Delta_Position[ORL_ARM2].speed.a = 1.0;
            Delta_Position[ORL_ARM2].ideal.a += delta;
            break;
          /*ARM2 a-*/
          case 'v':
            Delta_Position[ORL_ARM2].speed.a = - 1.0;
            Delta_Position[ORL_ARM2].ideal.a -= delta;
            break;
          /*ARM2 e+*/
          case 'B':
            Delta_Position[ORL_ARM2].speed.e = 1.0;
            Delta_Position[ORL_ARM2].ideal.e += delta;
            break;
          /*ARM2 e-*/
          case 'b':
            Delta_Position[ORL_ARM2].speed.e = - 1.0;
            Delta_Position[ORL_ARM2].ideal.e -= delta;
            break;
          /*ARM2 r+*/
          case 'N':
            Delta_Position[ORL_ARM2].speed.r = 1.0;
            Delta_Position[ORL_ARM2].ideal.r += delta;
            break;
          /*ARM2 r-*/
          case 'n':
            Delta_Position[ORL_ARM2].speed.r = - 1.0;
            Delta_Position[ORL_ARM2].ideal.r -= delta;
            break;

          /*ARM3 +*/
          case 'K':
          case 'k':
            Delta_Position[ORL_ARM3].speed.x = 1.0/100;
            Delta_Position[ORL_ARM3].ideal.x += delta/100;
            break;
          /*ARM3 -*/
          case 'M':
          case 'm':
            Delta_Position[ORL_ARM3].speed.x = - 1.0/100;
            Delta_Position[ORL_ARM3].ideal.x -= delta/100;
            break;

          case 'q':
          case 'Q':
            f_1_command = true;
            f_2_loop_keyboard = false;
            break;
          }
        }
        else
        {
          f_2_loop_keyboard = false;
          f_1_command = true;
        }
      }
    }

#if 0
    if (f_2_loop_joystick)
    {
      int fd, rc;
      float eventiJoystick[2][10];
      struct js_event jse;

      fd = open_joystick(JOYSTICK_DEVNAME);
      if (fd < 0)
      {
       printf("Error opening Joystick device. Use keyboard\n");
       f_2_loop_joystick = false;
       f_2_loop_keyboard = true;
      }
      else
      {
        memset (&eventiJoystick[0][0],0x00,sizeof(float)*2*10);
        printf("Control the robot with The joystick\n");
        printf("Button 1 to display the last message received\n");
        printf("Button 2 to stop using the Joytick\n");
        printf("ARM1 on Left Side of the Joytick\n");
        printf("ARM2 on Right Side of the Joytick\n");
        printf("ARM3 on Left Side of the Joytick\n");
      }
      while(f_2_loop_joystick)
      {
        if(     (modality_active[ORL_ARM1] == CRCOPEN_POS_RELATIVE) || (modality_active[ORL_ARM1] == CRCOPEN_POS_ADDITIVE) || (modality_active[ORL_ARM1] == CRCOPEN_POS_ADDITIVE_SB)
            ||  (modality_active[ORL_ARM2] == CRCOPEN_POS_RELATIVE) || (modality_active[ORL_ARM2] == CRCOPEN_POS_ADDITIVE) || (modality_active[ORL_ARM2] == CRCOPEN_POS_ADDITIVE_SB)
            ||  (modality_active[ORL_ARM3] == CRCOPEN_POS_RELATIVE) || (modality_active[ORL_ARM3] == CRCOPEN_POS_ADDITIVE) || (modality_active[ORL_ARM3] == CRCOPEN_POS_ADDITIVE_SB)
          )
        {
          rc = read_joystick_event(&jse);
          usleep(1000);
          if (rc == 1)
          {
            /*printf("Event: time %8u, value %8hd, type: %3u, axis/button: %u\n",jse.time, jse.value, jse.type, jse.number);*/
            if((jse.type-1 < 2) && (jse.type-1 >= 0) && (jse.number < 10) && (jse.number >= 0) )
            {
              eventiJoystick[jse.type-1][jse.number] = jse.value;
            }
          }

          for(jse.type = 1;jse.type <= 2;jse.type++)
          {
            for(jse.number = 0;jse.number < 10;jse.number++)
            {
              jse.value = eventiJoystick[jse.type-1][jse.number];

              switch(jse.type)
              {
              case JS_EVENT_BUTTON:    /* 0x01     button pressed/released */
                switch(jse.number)
                {
                case JS_FUNC_1:            /*0*/
                  if (jse.value != 0)
                  {
                    printLastReceived();
                    eventiJoystick[jse.type-1][jse.number] = jse.value = 0;
                  }
                  break;
                case JS_FUNC_2:            /*1*/
                  if (jse.value != 0)
                  {
                    eventiJoystick[jse.type-1][jse.number] = jse.value = 0;
                  }
                  break;
                case JS_FUNC_3:            /*2*/
                  if (jse.value != 0)
                  {
                    eventiJoystick[jse.type-1][jse.number] = jse.value = 0;
                  }
                  break;
                case JS_FUNC_4:            /*3*/
                  if (jse.value != 0)
                  {
                    f_2_loop_keyboard = true;
                    f_2_loop_joystick = false;
                  }
                  break;
                case JS_Z1_UP:                /*4*/
                  Delta_Position[ORL_ARM1].speed.z = (double)jse.value;
                  if (jse.value == 0)
                    Delta_Position[ORL_ARM1].ideal.z = Delta_Position[ORL_ARM1].real.z;
                  else
                    Delta_Position[ORL_ARM1].ideal.z += delta;
                  break;
                case JS_Z2_UP:                /*5*/
                  Delta_Position[ORL_ARM2].speed.z = (double)jse.value;
                  if (jse.value == 0)
                    Delta_Position[ORL_ARM2].ideal.z = Delta_Position[ORL_ARM2].real.z;
                  else
                    Delta_Position[ORL_ARM2].ideal.z += delta;
                  break;
                case JS_Z1_DOWN:              /*6*/
                  if(Delta_Position[ORL_ARM1].speed.z == 0.0)
                  {
                    Delta_Position[ORL_ARM1].speed.z = - (double)jse.value;
                    if (jse.value == 0)
                      Delta_Position[ORL_ARM1].ideal.z = Delta_Position[ORL_ARM1].real.z;
                    else
                      Delta_Position[ORL_ARM1].ideal.z -= delta;
                  }
                  break;
                case JS_Z2_DOWN:              /*7*/
                  if(Delta_Position[ORL_ARM2].speed.z == 0.0)
                  {
                    Delta_Position[ORL_ARM2].speed.z = - (double)jse.value;
                     if (jse.value == 0)
                      Delta_Position[ORL_ARM2].ideal.z = Delta_Position[ORL_ARM2].real.z;
                    else
                      Delta_Position[ORL_ARM2].ideal.z -= delta;
                  }
                  break;
                case JS_FUNC_9:            /*8*/
                  if (jse.value != 0)
                  {
                    eventiJoystick[jse.type-1][jse.number] = jse.value = 0;
                  }
                  break;
                case JS_FUNC_10:           /*9*/
                  if (jse.value != 0)
                  {
                    eventiJoystick[jse.type-1][jse.number] = jse.value = 0;
                  }
                  break;
                default:
                  break;
                }
                break;
                case JS_EVENT_AXIS:    /* 0x02     joystick moved */
                  switch(jse.number)
                  {
                  case  JS_Y1:                 /*0*/
                    Delta_Position[ORL_ARM1].speed.y = (double)jse.value / 32767;
                    if (jse.value == 0)
                      Delta_Position[ORL_ARM1].ideal.y = Delta_Position[ORL_ARM1].real.y;
                    else if (jse.value <0)
                      Delta_Position[ORL_ARM1].ideal.y -= delta;
                    else
                      Delta_Position[ORL_ARM1].ideal.y += delta;
                    break;
                  case  JS_X1:                 /*1*/
                    Delta_Position[ORL_ARM1].speed.x = (double)jse.value / 32767;
                    if (jse.value == 0)
                      Delta_Position[ORL_ARM1].ideal.x = Delta_Position[ORL_ARM1].real.x;
                    else if (jse.value <0)
                      Delta_Position[ORL_ARM1].ideal.x -= delta;
                    else
                      Delta_Position[ORL_ARM1].ideal.x += delta;
                    break;
                  case  JS_Y2:                 /*2*/
                    Delta_Position[ORL_ARM2].speed.y = (double)jse.value / 32767;
                    if (jse.value == 0)
                      Delta_Position[ORL_ARM2].ideal.y = Delta_Position[ORL_ARM2].real.y;
                    else if (jse.value <0)
                      Delta_Position[ORL_ARM2].ideal.y -= delta;
                    else
                      Delta_Position[ORL_ARM2].ideal.y += delta;
                   break;
                  case  JS_X2:                 /*3*/
                    Delta_Position[ORL_ARM2].speed.x = (double)jse.value / 32767;
                    if (jse.value == 0)
                      Delta_Position[ORL_ARM2].ideal.x = Delta_Position[ORL_ARM2].real.x;
                    else if (jse.value <0)
                      Delta_Position[ORL_ARM2].ideal.x -= delta;
                    else
                      Delta_Position[ORL_ARM2].ideal.x += delta;
                   break;
                  case  JS_MENU_LEFT_RIGHT:    /*4*/
                    Delta_Position[ORL_ARM3].speed.x = (double)jse.value / 32767 / 100;
                    if (jse.value == 0)
                      Delta_Position[ORL_ARM3].ideal.x = Delta_Position[ORL_ARM3].real.x;
                    else if (jse.value <0)
                      Delta_Position[ORL_ARM3].ideal.x -= delta / 100;
                    else
                      Delta_Position[ORL_ARM3].ideal.x += delta / 100;
                    break;
                  case  JS_MENU_UP_DOWN:       /*5*/
                    break;
                  default:
                    break;
                  }
                  break;
                case JS_EVENT_INIT:    /* 0x80    initial state of device */
                  break;
                default:
                  break;
              }
            }
          }
        }
        else
        {
          f_2_loop_joystick = false;
          f_1_command = true;
          printf("flag_MoveKeyboard[ORL_ARM1] %d, flag_MoveKeyboard[ORL_ARM2] %d, flag_MoveKeyboard[ORL_ARM3] %d\n",flag_MoveKeyboard[ORL_ARM1], flag_MoveKeyboard[ORL_ARM2], flag_MoveKeyboard[ORL_ARM3]);
        }

      }
    }
#endif

    if (f_2_read_sys_var_name)
    {
      printf("--> Write system Variable Name: \n");
      if (input_console(temp,sizeof(temp)) == 0)
      {
        printf("--! Invalid value!\n");
      }
      else
      {
        strcpy(orl_sys_var.sysvar_name,temp);
      }
      f_2_read_sys_var_name = false;
      f_3_read_sys_var_type = true;
    }

    if (f_2_modifie_sys_var_name)
    {
      printf("--> Write system Variable Name: \n");
      if (input_console(temp,sizeof(temp)) == 0)
      {
        printf("--! Invalid value!\n");
      }
      else
      {
        strcpy(orl_sys_var.sysvar_name,temp);
      }
      f_2_modifie_sys_var_name = false;
      f_3_modifie_sys_var_type = true;
    }

    if (f_3_read_sys_var_type)
    {
      printf("--> Select system Variable's TYPE: [I]nt, [B]ool, [R]eal, [S]tring?\n");
      input_console(temp,sizeof(temp));
      a_2    = temp[0];
      printf("\n");
      switch (a_2)
      {
      case 'I':
      case 'i':
        orl_sys_var.ctype = ORL_INT;
        f_3_read_sys_var_type = false;
        f_4_read_sys_var = true;
        break;
      case 'B':
      case 'b':
        orl_sys_var.ctype = ORL_BOOL;
        f_3_read_sys_var_type = false;
        f_4_read_sys_var = true;
        break;
      case 'R':
      case 'r':
        orl_sys_var.ctype = ORL_REAL;
        f_3_read_sys_var_type = false;
        f_4_read_sys_var = true;
        break;
      case 'S':
      case 's':
        orl_sys_var.ctype = ORL_STRING;
        f_3_read_sys_var_type = false;
        f_4_read_sys_var = true;
        break;
      default:
        printf("--! Invalid value!\n");
        f_1_command = true;
        f_3_read_sys_var_type = false;
        break;
      }
    }

    if (f_3_modifie_sys_var_type)
    {
      printf("--> Select system Variable's TYPE: [I]nt, [B]ool, [R]eal, [S]tring?\n");
      input_console(temp,sizeof(temp));
      a_2    = temp[0];
      printf("\n");
      switch (a_2)
      {
      case 'I':
      case 'i':
        orl_sys_var.ctype = ORL_INT;
        f_3_modifie_sys_var_type = false;
        f_4_modifie_sys_var_value = true;
        break;
      case 'B':
      case 'b':
        orl_sys_var.ctype = ORL_BOOL;
        f_3_modifie_sys_var_type = false;
        f_4_modifie_sys_var_value = true;
        break;
      case 'R':
      case 'r':
        orl_sys_var.ctype = ORL_REAL;
        f_3_modifie_sys_var_type = false;
        f_4_modifie_sys_var_value = true;
        break;
      case 'S':
      case 's':
        orl_sys_var.ctype = ORL_STRING;
        f_3_modifie_sys_var_type = false;
        f_4_modifie_sys_var_value = true;
        break;
      default:
        printf("--! Invalid value!\n");
        f_1_command = true;
        f_3_modifie_sys_var_type = false;
        break;

      }
    }

    if (f_4_modifie_sys_var_value)
    {
      if (ORL_get_data(&orl_sys_var, ORL_VERBOSE, ORL_CNTRL01) == ORLOPEN_RES_OK )
      {
        printf("--> Write the new %s (%s) VALUE: \n", orl_sys_var.sysvar_name,orl_sys_var.sv);
        if (input_console(temp,sizeof(temp)) == 0)
        {
          printf("--! Invalid value!\n");
        }
        else
        {
          strcpy(orl_sys_var.sv,temp);
          switch (orl_sys_var.ctype)
          {
          case ORL_INT:
          case ORL_BOOL:
            orl_sys_var.iv = atoi(orl_sys_var.sv);
            orl_sys_var.dv = 0.0;
            break;
          case ORL_REAL:
            orl_sys_var.iv = 0;
            orl_sys_var.dv = atof(orl_sys_var.sv);
            break;
          case ORL_STRING:
            break;
          }
          if (ORL_set_data(orl_sys_var, ORL_VERBOSE, ORL_CNTRL01) != ORLOPEN_RES_OK )
          {
            printf("%s>\tError\n",orl_sys_var.sysvar_name);
          }
          printf("%s>\t permission %d\tstring %s\n",orl_sys_var.sysvar_name,orl_sys_var.permission,orl_sys_var.sv);
        }
      }

      f_1_command = true;
      f_4_modifie_sys_var_value = false;
    }


    if (f_4_read_sys_var)
    {
      if (ORL_get_data(&orl_sys_var, ORL_VERBOSE, ORL_CNTRL01) != ORLOPEN_RES_OK )
      {
        printf("%s>\tError\n",orl_sys_var.sysvar_name);
      }
      switch (orl_sys_var.ctype)
      {
      case ORL_INT:
      case ORL_BOOL:
        printf("%s>\tint/bool %d\n",orl_sys_var.sysvar_name,orl_sys_var.iv);
        break;
      case ORL_REAL:
        printf("%s>\treal %f\n",orl_sys_var.sysvar_name,orl_sys_var.dv);
        break;
      case ORL_STRING:
        break;
      }
      printf("%s>\t permission %d\tstring %s\n",orl_sys_var.sysvar_name,orl_sys_var.permission,orl_sys_var.sv);

      f_1_command = true;
      f_4_read_sys_var = false;
    }

    if (f_3_type_move)
    {
      printf("--> Select target type: [J]oint or [P]osition? \n");
      input_console(temp,sizeof(temp));
      a_2    = temp[0];
      printf("\n");
      switch (a_2)
      {
      case 'J':
      case 'j':
        printf("<-- ARM %d\n",idx_arm+1);
        printf("POS_MIS J %f %f %f %f %f %f %f\n", current_joints[idx_arm].value[ORL_AX1],current_joints[idx_arm].value[ORL_AX2],current_joints[idx_arm].value[ORL_AX3],current_joints[idx_arm].value[ORL_AX4],current_joints[idx_arm].value[ORL_AX5],current_joints[idx_arm].value[ORL_AX6],current_joints[idx_arm].value[ORL_AX7]);
        printf("POS_MIS C %f %f %f %f %f %f %s\n", current_position[idx_arm].x,current_position[idx_arm].y,current_position[idx_arm].z,current_position[idx_arm].a,current_position[idx_arm].e,current_position[idx_arm].r,current_position[idx_arm].config_flags);
        sm_maskjnt = ORLOPEN_GetOpenMask        ( ORL_SILENT,ORL_CNTRL01, idx_arm );
        target_jnt.unit_type = ORL_POSITION_LINK_DEGREE;
        for (i = 0; i <= ORL_MAX_AXIS ; i++)
        {
          if(sm_maskjnt & (1<<i))
          {
            printf("---> J[%d] value (in degrees) [%f]: ",i+1,current_joints[idx_arm].value[i]);
            if (input_console(temp,sizeof(temp)) == 0)
              target_jnt.value[i] = current_joints[idx_arm].value[i];
            else
              target_jnt.value[i] = atof(temp);
            printf("<-- %f\n",target_jnt.value[i]);
          }
        }

        if( ORL_direct_kinematics(&temp_pos, &target_jnt, ORL_SILENT, ORL_CNTRL01, idx_arm) != 0 )
        {
          printf("--! Direct Kinematics fails! Check joint values...\n");
          f_1_command = true;
          f_3_type_move = false;
          break;
        }
        input_from_user_type_end_point = 0;/*END POINT TYPE JOINT*/
        f_3_type_move = false;
        f_4_set_move_parameters = true;
        break;
      case 'P':
      case 'p':
        printf("<-- ARM %d\n",idx_arm+1);
        printf("POS_MIS J %f %f %f %f %f %f %f\n", current_joints[idx_arm].value[ORL_AX1],current_joints[idx_arm].value[ORL_AX2],current_joints[idx_arm].value[ORL_AX3],current_joints[idx_arm].value[ORL_AX4],current_joints[idx_arm].value[ORL_AX5],current_joints[idx_arm].value[ORL_AX6],current_joints[idx_arm].value[ORL_AX7]);
        printf("POS_MIS C %f %f %f %f %f %f %s\n", current_position[idx_arm].x,current_position[idx_arm].y,current_position[idx_arm].z,current_position[idx_arm].a,current_position[idx_arm].e,current_position[idx_arm].r,current_position[idx_arm].config_flags);
        printf("---> X value (in millimeters) [%f]: ",current_position[idx_arm].x);
        target_pos.unit_type = ORL_CART_POSITION;
        if (input_console(temp,sizeof(temp)) == 0)
          target_pos.x = current_position[idx_arm].x;
        else
          target_pos.x = atof(temp);
        printf("<-- %f\n",target_pos.x);
        printf("---> Y value (in millimeters) [%f]: ",current_position[idx_arm].y);
        if (input_console(temp,sizeof(temp)) == 0)
          target_pos.y = current_position[idx_arm].y;
        else
          target_pos.y = atof(temp);
        printf("<-- %f\n",target_pos.y);
        printf("---> Z value (in millimeters) [%f]: ",current_position[idx_arm].z);
        if (input_console(temp,sizeof(temp)) == 0)
          target_pos.z = current_position[idx_arm].z;
        else
          target_pos.z = atof(temp);
        printf("<-- %f\n",target_pos.z);
        printf("---> A value (in degrees)     [%f]: ",current_position[idx_arm].a);
        if (input_console(temp,sizeof(temp)) == 0)
          target_pos.a = current_position[idx_arm].a;
        else
          target_pos.a = atof(temp);
        printf("<-- %f\n",target_pos.a);
        printf("---> E value (in degrees)     [%f]: ",current_position[idx_arm].e);
        if (input_console(temp,sizeof(temp)) == 0)
          target_pos.e = current_position[idx_arm].e;
        else
          target_pos.e = atof(temp);
        printf("<-- %f\n",target_pos.e);
        printf("---> R value (in degrees)     [%f]: ",current_position[idx_arm].r);
        if (input_console(temp,sizeof(temp)) == 0)
          target_pos.r = current_position[idx_arm].r;
        else
          target_pos.r = atof(temp);
        printf("<-- %f\n",target_pos.r);
        printf("---> Configuration            ['%s']: ",current_position[idx_arm].config_flags);
        if (input_console(temp,sizeof(temp)) == 0)
          strcpy(target_pos.config_flags,current_position[idx_arm].config_flags);
        else
          strcpy(target_pos.config_flags,temp);
        printf("<-- %s\n",target_pos.config_flags);

        if( ORL_inverse_kinematics(&target_pos, &temp_joints, ORL_SILENT, ORL_CNTRL01, idx_arm) != 0 )
        {
          printf("--! Inverse Kinematics fails! Check joint values...\n");
          f_1_command = true;
          f_3_type_move = false;
          break;
        }
        input_from_user_type_end_point = 1;/*END POINT TYPE CARTESIAN*/
        f_3_type_move = false;
        f_4_set_move_parameters = true;
        break;
      default:
        printf("--! Invalid value!\n");
        f_1_command = true;
        f_3_type_move = false;
        break;

      }
    }


    if (f_4_set_move_parameters)
    {
      if (input_from_user_type_end_point)/*END POINT TYPE CARTESIAN*/
      {
        ORL_set_move_parameters(ORL_NO_FLY, ORL_WAIT, ORL_FLY_NORMAL, input_from_user_type, &target_pos, NULL, ORL_SILENT, ORL_CNTRL01, idx_arm);
        mask_moving_arms = mask_moving_arms | (1<<idx_arm);
        flag_RunningMove[idx_arm] = true;
        printf("--> Move acquired.\n");

        if (input_from_user_type == ORL_TRJNT)
          printf("--> Corresponding PDL2 move line: MOVE ARM[%d] JOINT TO POS(%f, %f, %f, %f, %f, %f, %s')\n",idx_arm+1,target_pos.x,target_pos.y,target_pos.z,target_pos.a,target_pos.e,target_pos.r,target_pos.config_flags);
        else
          printf("--> Corresponding PDL2 move line: MOVE ARM[%d]  LINEAR TO POS(%f, %f, %f, %f, %f, %f, %s')\n",idx_arm+1,target_pos.x,target_pos.y,target_pos.z,target_pos.a,target_pos.e,target_pos.r,target_pos.config_flags);
      }
      else/*END POINT TYPE JOINT*/
      {
        ORL_set_move_parameters(ORL_NO_FLY, ORL_WAIT, ORL_FLY_NORMAL, input_from_user_type, NULL, &target_jnt, ORL_SILENT, ORL_CNTRL01, idx_arm);
        mask_moving_arms = mask_moving_arms | (1<<idx_arm);
        flag_RunningMove[idx_arm] = true;
        printf("--> Move acquired.\n");

        if (input_from_user_type == ORL_TRJNT)
          printf("--> Corresponding PDL2 move line: MOVE ARM[%d] JOINT TO {%f, %f, %f, %f, %f, %f, %f}\n",idx_arm+1,target_jnt.value[ORL_AX1],target_jnt.value[ORL_AX2],target_jnt.value[ORL_AX3],target_jnt.value[ORL_AX4],target_jnt.value[ORL_AX5],target_jnt.value[ORL_AX6],target_jnt.value[ORL_AX7]);
        else
          printf("--> Corresponding PDL2 move line: MOVE ARM[%d] LINEAR TO {%f, %f, %f, %f, %f, %f, %f}\n",idx_arm+1,target_jnt.value[ORL_AX1],target_jnt.value[ORL_AX2],target_jnt.value[ORL_AX3],target_jnt.value[ORL_AX4],target_jnt.value[ORL_AX5],target_jnt.value[ORL_AX6],target_jnt.value[ORL_AX7]);
      }
      si_k[idx_arm] = 0;
      printf("[MSG] Robot is about to move...\n");

      f_1_command = true;
      f_2_arm_to_move = false;
      f_2_arm_to_exit = false;
      f_3_type_move = false;
      f_4_set_move_parameters = false;
      /*Wait the end f the previous move*/
#if 0
      si_timeout = 0;
      while (mask_moving_arms != 0)
      {/*WAIT*/
        sleep(1);
        si_timeout ++;
        if (si_timeout > 30)
        {
          si_timeout = -1;
          /*CANCEL MOVE*/
          ORL_cancel_motion (ORL_SILENT,ORL_CNTRL01, ORL_ARM1);
          ORL_cancel_motion (ORL_SILENT,ORL_CNTRL01, ORL_ARM2);
          ORL_cancel_motion (ORL_SILENT,ORL_CNTRL01, ORL_ARM3);
          mask_moving_arms = 0;
        }
      }

      if(si_timeout == -1)
        printf("[MSG] Error moving Arm %d: move cancelled\n",idx_arm+1);
      else
#endif
        printf("[MSG] End move\n");

    }
  }
}
