#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <Recognition/RecognitionData.h>

int main(int argc, char** argv){

  if(argc!=3){
    std::cerr << "Usage: " << argv[0] << " /path/to/objects object_to_draw\n";
  }

  cv::FileStorage lol("./camera_data.yml", cv::FileStorage::READ);
  const auto& cam=Camera::CameraModel::readFrom(lol["camera_model"]);
  const auto& data=Recognition::RecognitionData(argv[1], cam);
  auto obj=data.getModel(argv[2]);
  /** Renders the result - REMOVE ME */
  //Eigen::Translation3f sposta[]={{0,0.2,0},{0.5, 1, 0},{-0.5,1,0},{0,1,0.3},{0,1,-0.3}};
  Eigen::Translation3f sposta[]={{0,0,1.2},{0.5,0,1},{-0.5,0,1},{0,0.3,1},{0,-0.3,1}};
 //Eigen::Translation3f sposta[]={{0,0,-1.2},{0.5,0,-1},{-0.5,0,-1},{0,0.3,-1},{0,-0.3,-1}};
  cv::Scalar colori[]={{255,0,0},{0,255,0},{0,0,255},{255,255,255},{0,0,0}};
  cv::Mat newFrame(480, 640, CV_8UC3);
  newFrame.setTo(cv::Scalar{50,50,50});
  cv::Mat sticazzi, stimazzi, stimaski;
  cv::Rect stiretti;
  for(int i=0; i<5; ++i) {
    auto jnsinv=Eigen::Affine3f(sposta[i]);//Eigen::AngleAxisf(0.6, Eigen::Vector3f::UnitX())*Eigen::AngleAxisf(0.6, Eigen::Vector3f::UnitZ())*sposta[i];
    std::cout << "JNSINV: " << jnsinv.matrix() << "\n";
    obj.render(jnsinv, stimazzi, sticazzi, stimaski, stiretti);
    stimazzi.copyTo(newFrame.rowRange(stiretti.y, stiretti.y+stiretti.height).colRange(stiretti.x,stiretti.x+stiretti.width));
    cv::rectangle(newFrame,stiretti,colori[i],2);
    std::stringstream s;
    s << Eigen::Affine3f(sposta[i]).translation();
    cv::putText(newFrame, "Position: ", cv::Point2f(stiretti.x,stiretti.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255-colori[i][0], 255-colori[i][1], 255-colori[i][2]));
    int step=cv::getTextSize("Position: ", cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr).height+5;
    cv::putText(newFrame, s.str(), cv::Point2f(stiretti.x,stiretti.y+step), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255-colori[i][0], 255-colori[i][1], 255-colori[i][2]));
  }
  imshow("REHFIWEUJHFWEIJFWEOIUFHWEIUFHWEIUFHWEIGFH", newFrame);
  while(cv::waitKey(100)&0xFF!='q');
  return 0;
}
