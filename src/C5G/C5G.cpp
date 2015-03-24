#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <C5G/C5G.h>
#include <C5G/userCallback.h>
#include <eORL.h>

namespace C5G{
  void C5G::moveCartesianGlobal(const Pose& p){
    ORL_cartesian_position  target_pos;
    target_pos.x=p.x;
    target_pos.y=p.y;
    target_pos.z=p.z;
    target_pos.a=p.alpha;
    target_pos.e=p.beta;
    target_pos.r=p.gamma;
    std::cout << "Global movement to (" << target_pos.x << ", " << target_pos.y << ", " << target_pos.z << ")\nOrientation: (" << target_pos.a << ", " << target_pos.e << ", " << target_pos.r << "\n";
  }
  /** TODO CHECK THIS!*/
  const Pose C5G::safePose={0.3, 0, 0.7, 0, 0, 0};
  void C5G::moveCartesian(const Pose& p){
    ORL_cartesian_position  target_pos;
    target_pos.x=p.x;
    target_pos.y=p.y;
    target_pos.z=p.z;
    target_pos.a=p.alpha;
    target_pos.e=p.beta;
    target_pos.r=p.gamma;
    std::cout << "Relative movement to (" << target_pos.x << ", " << target_pos.y << ", " << target_pos.z << ")\nOrientation: (" << target_pos.a << ", " << target_pos.e << ", " << target_pos.r << "\n";
  }

  void C5G::init(const std::string& STRING_IP_CNTRL, const std::string& STRING_SYS_ID){
    std::cout << "Initing the system..\n";
    int si_arm, res, si_i, period;
    ORL_cartesian_position sx_base, sx_tool, sx_uframe;

    std::cout << "Connection to " << STRING_IP_CNTRL << ": " << STRING_SYS_ID << ".c5g\n";

    cycle_active = false;
    period = ORL_0_4_MILLIS;

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

    if( (ORLOPEN_initialize_controller(STRING_IP_CNTRL.c_str(),STRING_SYS_ID.c_str(),ORL_SILENT,ORL_CNTRL01)) != 0 )
    {
      std::cerr << "error ORL_initialize_robot\n" ;
      throw std::string("Error in ORL_initialize_robot\n");
    }
    else{
      std::cout << STRING_IP_CNTRL << ": " << STRING_SYS_ID << ".c5g OK\n";
    }

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
    /* $BASE ARM*/
    sx_base.x = 0; sx_base.y = 0; sx_base.z = 0;
    sx_base.a = 0; sx_base.e = 0; sx_base.r = 0;
    /*ORL_initialize_frames(sx_base, sx_tool, sx_uframe, ORL_SILENT, ORL_CNTRL01, ORL_ARM1);*/

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

    std::cout << "Done.\n";
  }

  void C5G::standby(){
    ORL_terminate_controller(ORL_SILENT,ORL_CNTRL01);
    std::cout << "Goodbye, cruel world..\n";
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
    std::cout << "I'm leaving you today..\n";
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
    std::cout << "Goodbye..\n";
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
    std::cout << "Goodbye..\n";
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
    std::cout << "Goodbye.\n";
  }

  C5G::~C5G(){
    standby();
  }

  void C5G::executeGrasp(const Grasp& g){
    std::cout << "Executing grasp for object " << g.object << "\n";
    //moveCartesianGlobal(Shelf.getBinSafePose(

  }

  void C5G::setGripping(double strength){
    std::cout << "Closing the plier with strength " << strength << "\n";
    boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
  }
}
