#include <Camera/Recognition.h>
#include <Parser/RobotData.h>

namespace Camera{
  void updateGiorgio(int row, int column){
    using InterProcessCommunication::RobotData;
    RobotData& r=RobotData.getInstance();
    int binN=RobotData::xyToBin(row,column);
    for(int i=0; i<RobotData::MAX_ITEM_N; ++i){
      std::string name=r.bin[binN].object[i];
      if(name=="kygen_squeakin_eggs_plush_puppies"){
        recognizeBalls();
      }
      //Image p=r.bin[binN].photo;


  }


  void 
}
