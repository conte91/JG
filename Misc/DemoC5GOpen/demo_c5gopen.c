#include <unistd.h>
#include "demo_c5gopen.h"


static char DEFAULT_STRING_IP_CNTRL[]="172.22.178.102";
int  main (int argc, char **argv)
{
/*  ORL_joint_value sx_jnt_pos_cal,sx_jnt_pos_test,sx_jnt_out_test;
  int si_sts[3];*/

  int si_arm, res, si_i, period;
  ORL_cartesian_position sx_base, sx_tool, sx_uframe;
  STRING_IP_CNTRL=NULL;
  for (si_i = 1; si_i < argc;si_i++)
  {
 	if(!strcmp(argv[si_i], "-S")){
		si_i++;
		STRING_IP_CNTRL=argv[si_i];
	}
  }
  if(STRING_IP_CNTRL==NULL){
    fprintf(stderr, "You SHOULD specify a server with -S. Using default of %s\n", DEFAULT_STRING_IP_CNTRL);
    STRING_IP_CNTRL=DEFAULT_STRING_IP_CNTRL;
  }

  printf("Connection to %s: %s.c5g\n",STRING_IP_CNTRL, STRING_SYS_ID);

  printf("argc %d argv[0] %s argv[1] %s argv[2] %s\n",argc,argv[0],argv[1],argv[2] );
  cycle_active = false;
  period = ORL_0_4_MILLIS;
  for (si_i = 1; si_i < argc;si_i++)
  {
    res = strcmp( argv[si_i], "-c");
    res = strcmp( argv[si_i], "-t");

    /*sscanf(argv[1],"%d", &arm2);*/
    if (!strcmp( argv[si_i], "-c"))
    {
      cycle_active = true;
      printf("CYCLE Enabled\n");
    }
    if (!strcmp( argv[si_i], "-t"))
    {
      si_i++;
      switch(argv[si_i][0])
      {
      case '0':
        period = ORL_0_4_MILLIS;
        break;
      case '2':
        period = ORL_2_0_MILLIS;
        break;
      case '4':
        period = ORL_4_0_MILLIS;
        break;
      case '8':
        period = ORL_8_0_MILLIS;
        break;
      case '1':
        if(argv[si_i][1] == '6')
          period = ORL_16_0_MILLIS;
        else
          period = ORL_0_4_MILLIS;
        break;
      }
    }
  }


  /*freopen("stderr.log", "w", stderr);*/
  for(si_arm= 0; si_arm<MAX_NUM_ARMS;si_arm++)
  {
    flag_RunningMove[si_arm] = false;
    flag_ExitFromOpen[si_arm] = false;
    modality_active[si_arm] = CRCOPEN_LISTEN;
    modality_old[si_arm] = CRCOPEN_LISTEN;
    memset(&Delta_Position[si_arm].ideal,0x00,sizeof(ORL_cartesian_position));
    memset(&Delta_Position[si_arm].real,0x00,sizeof(ORL_cartesian_position));
    memset(&Delta_Position[si_arm].speed,0x00,sizeof(ORL_cartesian_position));
    Delta_Position[si_arm].ideal.unit_type = ORL_CART_POSITION;
    Delta_Position[si_arm].real.unit_type = ORL_CART_POSITION;
    Delta_Position[si_arm].speed.unit_type = ORL_TO_DEFINE;
  }
  mask_moving_arms = 0;

  if( (ORLOPEN_initialize_controller(STRING_IP_CNTRL,STRING_SYS_ID,ORL_SILENT,ORL_CNTRL01)) != 0 )
  {
    printf("error ORL_initialize_robot\n");
    exit(0);
  }
  else
    printf("%s: %s.c5g OK\n",STRING_IP_CNTRL, STRING_SYS_ID);

  ORLOPEN_set_period(period, ORL_VERBOSE, ORL_CNTRL01);

  /* $TOOL */
  sx_tool.unit_type = ORL_CART_POSITION;
  sx_tool.x = 0; sx_tool.y = 0; sx_tool.z = 100;
  sx_tool.a = 0; sx_tool.e = 0; sx_tool.r = 0;

  /* $UFRAME */
  sx_uframe.unit_type = ORL_CART_POSITION;
  sx_uframe.x = 0; sx_uframe.y = 0; sx_uframe.z = 0;
  sx_uframe.a = 0; sx_uframe.e = 0; sx_uframe.r = 0;

  sx_base.unit_type = ORL_CART_POSITION;
#ifdef RML
  /* $BASE ARM 3*/
  sx_base.x = 1375; sx_base.y = 0; sx_base.z = 2424;
  sx_base.a = -90; sx_base.e = 180; sx_base.r = 90;
  ORL_initialize_frames(sx_base, sx_tool, sx_uframe, ORL_SILENT, ORL_CNTRL01, ORL_ARM3);
  /* $BASE ARM 1
  sx_base.x = 481,422; sx_base.y = 141,28101; sx_base.z = 330,125;
  sx_base.a = -110; sx_base.e = -70; sx_base.r = 0;*/
  sx_base.x = 1856.420; sx_base.y = -141.280; sx_base.z = 2093.875;
  sx_base.a = -70; sx_base.e = 110; sx_base.r = 0;
  ORL_initialize_frames(sx_base, sx_tool, sx_uframe, ORL_SILENT, ORL_CNTRL01, ORL_ARM1);
  /* $BASE ARM 2
  sx_base.x = 481,422; sx_base.y = -141,28101; sx_base.z = 330,125;
  sx_base.a = 110; sx_base.e = -70; sx_base.r = 0;*/
  sx_base.x = 1856.420; sx_base.y = 141.280; sx_base.z = 2093.875;
  sx_base.a = 70; sx_base.e = 110; sx_base.r = 0;
  ORL_initialize_frames(sx_base, sx_tool, sx_uframe, ORL_SILENT, ORL_CNTRL01, ORL_ARM2);
#else
  /* $BASE ARM*/
  sx_base.x = 0; sx_base.y = 0; sx_base.z = 0;
  sx_base.a = 0; sx_base.e = 0; sx_base.r = 0;
  /*ORL_initialize_frames(sx_base, sx_tool, sx_uframe, ORL_SILENT, ORL_CNTRL01, ORL_ARM1);*/
  /* $BASE ARM2
  sx_base.x = 0; sx_base.y = 2000; sx_base.z = 0;
  sx_base.a = 0; sx_base.e = 0; sx_base.r = 0;
  if(MAX_NUM_ARMS >= ORL_ARM2)
    ORL_initialize_frames(sx_base, sx_tool, sx_uframe, ORL_SILENT, ORL_CNTRL01, ORL_ARM2);*/
#endif

  ORLOPEN_SetCallBackFunction( &user_callback, ORL_SILENT, ORL_CNTRL01);

  sleep(1);

  /******************************************************************/
  /******************************************************************/

  res = ORLOPEN_StartCommunication(ORL_SILENT);
  if ( res != 0 )
  {
    ORLOPEN_GetPowerlinkState(ORL_SILENT);
    exit(0);
  }
  sleep(2);

  initialize_Control_position();
  loopConsole();

  ORL_terminate_controller(ORL_SILENT,ORL_CNTRL01);
  return 0;
}

// EOF

