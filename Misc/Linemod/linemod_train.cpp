/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Willow Garage, Inc., Giorgio Toscana, Simone Baratta.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <set>
#include <cmath>

#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <Recognition/Model.h>
#include <Recognition/Renderer3d.h>
#include <Camera/CameraModel.h>
#include <Recognition/GLUTInit.h>
#include <C5G/Pose.h>

#include <opencv2/highgui/highgui.hpp>


bool visualize_ ;
int renderer_n_points_;
int renderer_angle_step_;
double renderer_radius_min_;
double renderer_radius_max_;
double renderer_radius_step_;
double renderer_near_;
double renderer_far_;

void trainObject(const boost::filesystem::path& trainDir, const std::string& object_id_, const Camera::CameraModel& cam)
{ 
  using boost::filesystem::path;
  path p=trainDir / path(object_id_);
  /** Check if the directory we found matches a renderable object */
  assert(is_directory(p) && "Object path is not a directory!");

  //set the object id as the name of the object itself
  std::cout<<"Elaborating " << object_id_<<"..\n";

  //Use our own models made in Blender....
  path mesh_path = p / path("textured_meshes") / path(object_id_+".obj");
  std::cout<< "Mesh filename: " << mesh_path.string() << "\n";

  Recognition::Model model(object_id_, mesh_path.string(), cam);

  /** Takes snapshots of the (ideal) object */
  int totalTemplates=(renderer_radius_max_-renderer_radius_min_)/renderer_radius_step_+1;
  totalTemplates*=2*M_PI/(renderer_angle_step_/180.0*M_PI);
  totalTemplates*=2*M_PI/(renderer_angle_step_/180.0*M_PI);
  totalTemplates*=2*M_PI/(renderer_angle_step_/180.0*M_PI);
  int i=0; 
  for (double radius=renderer_radius_min_; radius<=renderer_radius_max_; radius+=renderer_radius_step_)
  {
    for(double alpha=0; alpha<2*M_PI; alpha+=renderer_angle_step_/180.0*M_PI){
    for(double beta=0; beta<2*M_PI; beta+=renderer_angle_step_/180.0*M_PI){
    for(double gamma=0; gamma<2*M_PI; gamma+=renderer_angle_step_/180.0*M_PI){
        i++;
        std::stringstream status;
        status << "Loading images " << (i) << "/" << totalTemplates;
        std::cout << status.str();

        model.addTraining(radius,alpha,beta,gamma,cam);
        if((i % 5 ) && visualize_){
          cv::Mat image2show(cam.getHeight(), cam.getWidth(), CV_8UC3);
          cv::Mat depth2show(cam.getHeight(), cam.getWidth(), CV_16U);
          cv::Mat image, depth, mask;
          cv::Rect rect;
          model.render(C5G::Pose{0,0,radius,alpha,beta,gamma}, image, depth, mask, rect);
          image.copyTo(image2show(rect));
          depth.copyTo(depth2show(rect));
          imshow("mastamazza", image2show);
          imshow("mastadepth", depth2show);
          cv::waitKey(1);
        }



        // Delete the status
        for (size_t j = 0; j < status.str().size(); ++j) {
          std::cout << '\b';
        }
      }
    }
    }
  }

  //write the template + R + t + dist + K for each class
  model.saveToDirectory(trainDir);

  std::cout<<std::endl;

  /** Check the save file */
  std::cout<<"Reading back the data I just wrote..\n";
  {
    Recognition::Model testModel(object_id_, trainDir);
    for(int tID=0; tID<testModel.numTemplates(); ++tID){
      cv::Matx33d rTest(testModel.getR(tID));
      float dTest=testModel.getDist(tID);
      cv::Matx33f kTest(testModel.getK(tID));
      cv::Mat hTest(testModel.getHueHist(tID));
      assert(cv::countNonZero(rTest!=model.getR(tID))==0 && "Failed to read back data");
      assert(cv::countNonZero(kTest!=model.getK(tID))==0 && "Failed to read back data");
      assert(dTest==model.getDist(tID) && "Failed to read back data");
      assert(cv::countNonZero(hTest!=model.getHueHist(tID))==0 && "Failed to read back data");
    }
  }
  std::cout << "Success!\n";
}

int main(int argc, char* argv[])
{
  namespace fs = boost::filesystem; 

  cv::FileStorage config ("global_config.yml", cv::FileStorage::READ);
  const cv::FileNode& lmConfig=config["Linemod"];

  const cv::FileNode& lmTConfig=lmConfig["training"];
  /** Path on which the training will be done */
  std::string objsfolder_path;
  lmTConfig["trainPath"] >> objsfolder_path;

  /** True or False to output debug image */
  lmTConfig["visualizeTraining"] >> visualize_;
  const cv::FileNode& rParams=lmConfig["rendering"];

  //Set the Values:
  // Define the display
  //assign the parameters of the renderer
  rParams["nPoints"] >> renderer_n_points_ ;
  rParams["angleStep"] >> renderer_angle_step_ ;
  rParams["radiusMin"] >> renderer_radius_min_ ;
  rParams["radiusMax"] >> renderer_radius_max_ ;
  rParams["radiusStep"] >> renderer_radius_step_ ;
  rParams["renderNear"] >> renderer_near_ ;
  rParams["renderFar"] >> renderer_far_ ;
  /** Reads the model for the renderer */
  std::string camera_file;
  rParams["cameraModel"] >> camera_file;
  cv::FileStorage camFile(camera_file, cv::FileStorage::READ);
  const Camera::CameraModel& camModel=Camera::CameraModel::readFrom(camFile["camera_model"]);

  /** File containing the names of the objects which must be trained */
  fs::path objNamesPath=objsfolder_path / fs::path("names.txt");
  if(!fs::exists(objNamesPath) || !fs::is_regular_file(objNamesPath)){
    std::cerr << "Sry, to run the train program make sure that " << objNamesPath << " exists and contains the names of the objects to train!\n";
    return -1;
  }

  std::cout << "Reading model names from " << objNamesPath << "..\n";
  {
    std::ifstream names(objNamesPath.string());
    while(names.good()){
      std::string s;
      names >> s;
      std::cout << "Elaborating directory: " << s << "\n";
      trainObject(objsfolder_path, s, camModel);
    }
    if(!names.eof()){
      std::cout << "Could not read model names.";
      return -1;
    }
  }



  std::cout << "Ended :)\n";
  return 0;
}
