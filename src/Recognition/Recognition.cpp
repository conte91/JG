#include <fstream>
#include <stdexcept>
#include <Parser/RobotData.h>
#include <Recognition/GiorgioUtils.h>
#include <Recognition/Renderer3d.h>
#include <Camera/ImageViewer.h>
#include <C5G/Pose.h>
#include <Eigen/Core>
//#include <Eigen/Matrix>
#include <opencv2/opencv.hpp>
#include <Recognition/RecognitionData.h>

namespace Recognition{
  C5G::Pose recognizeBalls(int row, int column);



  void updateGiorgio(int row, int column){
    std::map<std::string, C5G::Pose> dovesono;
    dovesono["expo_dry_erase_board_eraser"]=C5G::Pose({1.20,-0.30, 0.15,0,0,0});
    dovesono["champion_copper_plus_spark_plug"]=C5G::Pose({1.20,-0.30, -0.45,0,0,0});
    dovesono["rollodex_mesh_collection_jumbo_pencil_cup"]=C5G::Pose({1.20,0.30, 0.30,0,0,0});
    dovesono["kygen_squeakin_eggs_plush_puppies"]=C5G::Pose({1.20,0.30, 0.30,0,0,0});
    using InterProcessCommunication::RobotData;
    RobotData& r=RobotData::getInstance();
    for(int i=0; i<RobotData::MAX_ITEM_N; ++i){
      std::string name =r.getBinItem(row, column, i);
      if(name==""){
        continue;
      }
      //if(name=="kygen_squeakin_eggs_plush_puppies"){
      //  std::cout << "Searching for a ball..\n";
      //  auto thePose=recognizeBalls(row, column);
      //  //SUGGESTION: draw object skeleton
      //  r.setObjPose(row, column, i, thePose);
      //  std::cout << "Done. Ball pose: " << r.getObjPose(row, column, i) << "\n";
      //} 
//      else if(name=="genuine_joe_plastic_stir_sticks" || name=="highland_6539_self_stick_notes" || name=="paper_mate_12_count_mirado_black_warrior"){
//        auto thePose=RecognitionData::getInstance().recognize(r.getFrame(row, column), name);
//      }
      else {

        try{
          auto thePose=dovesono.at(name);
          r.setObjPose(row,column,i,thePose);
        }
        catch(std::out_of_range sc){
          /** Everything else shall be recognized by hand */
          std::cout << "I'm sorry baby, you have to take it by urself\n";
          r.setObjPose(row, column, i, {-10000, 0, 0, 0, 0, 0});
        }

      }
      r.demoViewer.showImage(r.getPhoto(row,column));
      r.demoViewer.setTitle("Pose is: _____");
    }
    r.setDirty(row, column, false);
  }


  C5G::Pose recognizeBalls(int row, int column){
    using InterProcessCommunication::RobotData;
    RobotData& r=RobotData::getInstance();
    Img::Image p=r.getFrame(row, column);
    cv::imwrite("/tmp/myballs.png", p.rgb);
    system("python ./hopersolepalle.py");
    double x, y, z, a, b, g;
    std::ifstream resultFile("/tmp/grasp.pose");
    resultFile >> x >> y >> z >> a >> b >> g;
    return C5G::Pose({x,y,z,a,b,g});
    /***TODO maybe we shall remove the balls from the image, but for now its'a okay
     *
     */
  }

}
