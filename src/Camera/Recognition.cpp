#include <fstream>
#include <Parser/RobotData.h>
#include <Camera/Recognition.h>
#include <C5G/Pose.h>
#include <opencv2/opencv.hpp>

namespace Camera{
  void updateGiorgio(int row, int column){
    using InterProcessCommunication::RobotData;
    RobotData& r=RobotData::getInstance();
    //int binN=RobotData::xyToBin(row,column);
    for(int i=0; i<RobotData::MAX_ITEM_N; ++i){
      std::string name =r.getBinItem(row, column, i);
      if(name=="kygen_squeakin_eggs_plush_puppies"){
        std::cout << "Searching for a ball..\n";
        auto thePose=recognizeBalls(row, column);
        r.setObjPose(row, column, i, thePose);
        std::cout << "Done. Ball pose: " << r.getObjPose(row, column, i) << "\n";
      }
      //Image p=r.bin[binN].photo;
      else {
        /** Everything else shall be recognized by hand */
        std::cout << "I'm sorry baby, you have to take it by urself\n";
      }
    }

    r.setDirty(row, column, false);
        
  }


  C5G::Pose recognizeBalls(int row, int column){
    using InterProcessCommunication::RobotData;
    RobotData& r=RobotData::getInstance();
    Camera::Image p=r.getFrame(row, column);
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
