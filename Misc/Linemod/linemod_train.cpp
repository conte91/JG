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

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <Recognition/Model.h>
#include <Recognition/Renderer3d.h>
#include <Recognition/GiorgioUtils.h>
#include <Camera/CameraModel.h>
#include <Recognition/GLUTInit.h>

#include <opencv2/highgui/highgui.hpp>


bool visualize_ ;
int renderer_n_points_;
int renderer_angle_step_;
double renderer_radius_min_;
double renderer_radius_max_;
double renderer_radius_step_;
int renderer_width_;
int renderer_height_;
double renderer_near_;
double renderer_far_;
double renderer_focal_length_x_;
double renderer_focal_length_y_;

// Functions to store detector and templates in single XML/YAML file
static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename)
{
  cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector->read(fs.root());

  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
    detector->readClass(*i);

  return detector;
}

static void writeLinemod(const cv::Ptr<cv::linemod::Detector>& detector, const std::string& filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  detector->write(fs);

  std::vector<std::string> ids = detector->classIds();
  fs << "classes" << "[";
  for (int i = 0; i < (int)ids.size(); ++i)
  {
    fs << "{";
    detector->writeClass(ids[i], fs);
    fs << "}"; // current class
  }
  fs << "]"; // classes
  fs.release();
}

static void writeLinemodAndPoses(const cv::Ptr<cv::linemod::Detector>& detector, const std::string& filename, const std::string& mesh_file_path,
    const std::vector<cv::Mat>& R, const std::vector<cv::Mat>& T,
    const std::vector<cv::Mat>& Ks, const std::vector<float>& dist, const std::vector<cv::Mat>& HueHist)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  detector->write(fs);

  std::vector<std::string> ids = detector->classIds();
  std::cout<<"Num Classes of the detector to write down: "<<(int)ids.size()<<"\n";
  fs << "classes" << "[";
  for (int i = 0; i < (int)ids.size(); ++i)
  {
    fs << "{";
    detector->writeClass(ids[i], fs);
    fs << "}"; // current class
  }
  fs << "]"; // classes

  //TODO: Safe only if 1 class in the detector !!!!
  for (int i = 0; i < (int)ids.size(); ++i)
  {
    //save : R, T, dist, and Ks for that class
    fs << "Rot" << "[";
    for (int R_idx=0;R_idx<R.size();++R_idx)
    {
      fs << R[R_idx];
    }
    fs << "]";

    fs << "Transl" << "[";
    for (int T_idx=0;T_idx<T.size();++T_idx)
    {
      fs << T[T_idx];
    }
    fs << "]";

    fs << "dist" << "[";
    for (int dist_idx=0;dist_idx<dist.size();++dist_idx)
    {
      fs << dist[dist_idx];
    }
    fs << "]";

    fs << "Ks" << "[";
    for (int Ks_idx=0;Ks_idx<Ks.size();++Ks_idx)
    {
      fs << Ks[Ks_idx];
    }
    fs << "]";

    fs << "Hue" << "[";
    for (int Hue_idx=0;Hue_idx<HueHist.size();++Hue_idx)
    {
      fs << HueHist[Hue_idx];
    }
    fs << "]";
  }

  fs << "mesh_file_path" << mesh_file_path ;

  fs.release();

}

void trainObject(const boost::filesystem::path& trainDir, const std::string& object_id_, const Camera::CameraModel& cam)
{ 
  using boost::filesystem::path;
  path p=trainDir / path(object_id_);
  assert(is_directory(p) && "Object path is not a directory!");

  /** Check if the directory we found matches a renderable object */
  std::vector<cv::Mat>  Rs_;
  std::vector<cv::Mat>  Ts_;
  std::vector<float>  distances_;
  std::vector<cv::Mat>  Ks_;
  std::vector<cv::Mat>  HueHist_;

  //set the object id as the name of the object itself
  std::cout<<"Elaborating " << object_id_<<"..\n";

  //Use our own models made in Blender....
  path mesh_path = p / path("textured_meshes") / path(object_id_+".obj");
  std::cout<< "Mesh filename: " << mesh_path.string() << "\n";

  cv::Ptr<cv::linemod::Detector> detector_ = cv::linemod::getDefaultLINEMOD();
  std::shared_ptr<Renderer3d> rendererPtr (new Renderer3d(mesh_path.string()));
  rendererPtr->set_parameters(cam.getWidth(), cam.getHeight(), cam.getFx(), cam.getFy(), renderer_near_, renderer_far_);

  std::shared_ptr<RendererIterator> rendererIteratorPtr (new RendererIterator(rendererPtr, renderer_n_points_));   
  rendererIteratorPtr->angle_step_ = renderer_angle_step_;
  rendererIteratorPtr->radius_min_ = float(renderer_radius_min_);
  rendererIteratorPtr->radius_max_ = float(renderer_radius_max_);
  rendererIteratorPtr->radius_step_ = float(renderer_radius_step_);

  cv::Mat image, depth, mask;
  cv::Matx33d R;
  cv::Vec3d T;

  /** Takes snapshots of the (ideal) object */
  for (size_t i = 0; !rendererIteratorPtr->isDone(); ++i, ++(*rendererIteratorPtr) )
  {
    std::stringstream status;
    status << "Loading images " << (i+1) << "/" << rendererIteratorPtr->n_templates();
    std::cout << status.str();

    cv::Rect rect;
    rendererIteratorPtr->render(image, depth, mask, rect);

    R = rendererIteratorPtr->R_obj();
    T = rendererIteratorPtr->T();
    float distance = fabs(rendererIteratorPtr->D_obj() - float(depth.at<ushort>(depth.rows/2.0f, depth.cols/2.0f)/1000.0f));

    std::vector<cv::Mat> sources(2);
    sources[0] = image;
    sources[1] = depth;
    if (visualize_)
    {
      cv::namedWindow("Rendering");
      if (!image.empty()) {
        cv::imshow("Rendering", image);
        cv::waitKey(10);
      }
    }

    if(image.empty())
    {
      // Delete the status   
      for (size_t j = 0; j < status.str().size(); ++j)
        std::cout << '\b';
      std::cout<<"Empty Image"<<"\n";
      continue;
    }


    int template_in = detector_->addTemplate(sources, object_id_, mask); //object id as string
    if (template_in == -1)
    {
      // Delete the status
      for (size_t j = 0; j < status.str().size(); ++j)
        std::cout << '\b';
      std::cout<<template_in<<"\n";
      continue;
    }

    /*****COMPUTE HUE HISTOGRAM*******/
    //Conversione RBG -> HSV
    cv::Mat hsv_mat;
    cv::cvtColor(image, hsv_mat, CV_BGR2HSV);
    std::vector<cv::Mat> hsv_planes;
    cv::split( hsv_mat, hsv_planes );

    //discretizzai valori di Hue a 30 livelli (Bins)
    int Hbins = 30;
    int histSize = Hbins; //per Histogramma 1D
    // Hue varia da 0 a 179 compreso
    float hranges[] = { 0, 180 };
    const float* histRange = { hranges };
    cv::Mat Hue_Hist;
    //          	cv::imshow("mask",mask);
    //          	cv::waitKey();
    //Compute Hist only on the mask
    cv::calcHist( &hsv_planes[0], 1, 0, mask, Hue_Hist, 1, &histSize, &histRange, true, false );
    //normalizza da 0 100.0
    cv::normalize(Hue_Hist, Hue_Hist, 0.0, 1.0, cv::NORM_MINMAX, -1, cv::Mat() );
    //store the Hue Hist of that template
    HueHist_.push_back(Hue_Hist);
    /*****END COMPUTE HUE HISTOGRAM*******/


    // Also store the pose of each template
    Rs_.push_back(cv::Mat(R));
    Ts_.push_back(cv::Mat(T));
    distances_.push_back(distance);
    Ks_.push_back(cam.getIntrinsic());
    HueHist_.push_back(Hue_Hist);

    // Delete the status
    for (size_t j = 0; j < status.str().size(); ++j) {
      std::cout << '\b';
    }

  }

  //write the template + R + t + dist + K for each class
  path saveLinemodPath = p / path(object_id_+"_Linemod.yml");
  //writeLinemod(detector_,saveLinemodPath.string());
  writeLinemodAndPoses(detector_, saveLinemodPath.string(), mesh_path.string(),
      Rs_, Ts_, Ks_, distances_, HueHist_);

  std::cout<<std::endl;

  /** Check the save file */
  std::cout<<"Reading back the data I just wrote..\n";
  {
    Recognition::Model testModel(object_id_, trainDir);
    for(int tID=0; tID<testModel.numTemplates(); ++tID){
      cv::Mat rTest(testModel.getR(tID));
      cv::Mat tTest(testModel.getT(tID));
      float dTest=testModel.getDist(tID);
      cv::Mat kTest(testModel.getK(tID));
      cv::Mat hTest(testModel.getHueHist(tID));
      assert(cv::countNonZero(rTest!=Rs_[tID])==0 && "Failed to read back data");
      assert(cv::countNonZero(tTest!=Ts_[tID])==0 && "Failed to read back data");
      assert(cv::countNonZero(kTest!=Ks_[tID])==0 && "Failed to read back data");
      assert(dTest==distances_[tID] && "Failed to read back data");
      assert(cv::countNonZero(hTest!=HueHist_[tID])==0 && "Failed to read back data");
    }
  }
  std::cout << "Success!\n";
}

int main(int argc, char* argv[])
{
  namespace fs = boost::filesystem; 

  cv::FileStorage config ("global_config.yml", cv::FileStorage::READ);
  const cv::FileNode& lmConfig=config["Linemod"];

  /** Path on which the training will be done */
  std::string objsfolder_path;
  lmConfig["trainPath"] >> objsfolder_path;

  /** True or False to output debug image */
  lmConfig["visualizeTraining"] >> visualize_;
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



  return 0;
}
