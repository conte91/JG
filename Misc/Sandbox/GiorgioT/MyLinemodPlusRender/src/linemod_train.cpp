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

#include "renderer3d.h"
#include "utils.h"

#include <opencv2/highgui/highgui.hpp>


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

static cv::Ptr<cv::linemod::Detector> readLinemodAndPoses(const std::string& filename, std::string& mesh_file_path,
    std::map<std::string,std::vector<cv::Mat> >& Rmap,
    std::map<std::string,std::vector<cv::Mat> >& Tmap,
    std::map<std::string,std::vector<cv::Mat> >& Kmap,
    std::map<std::string,std::vector<float> >& dist_map,
    std::map<std::string,std::vector<cv::Mat> >& HueHist)
{
  cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector->read(fs.root());

  //Read mesh_file_path
  fs["mesh_file_path"] >> mesh_file_path;

  cv::FileNode fn = fs["classes"];
  //size_t num_classes=0;
  //for each class:
  //int forRot=0;
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
  {

    std::string class_id_tmp = (*i)["class_id"];
    std::cout<<"class_id_tmp: "<<class_id_tmp <<"\n";
    detector->readClass(*i);
    /**Read R**/
    cv::FileNode n = fs["Rot"];                         // Read string sequence - Get node
    if (n.type() != cv::FileNode::SEQ)
    {
      std::cerr << "strings is not a sequence! FAIL" << std::endl;
      return 0;
    }
    cv::FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
    for (; it != it_end;) //++it)
    {
      cv::Mat m; it >> m;
      Rmap[class_id_tmp].push_back( m );
      //++forRot;
    }

    /**Read T**/
    n = fs["Transl"];                         // Read string sequence - Get node
    if (n.type() != cv::FileNode::SEQ)
    {
      std::cerr << "strings is not a sequence! FAIL" << std::endl;
      return 0;
    }
    it = n.begin(), it_end = n.end(); // Go through the node
    for (; it != it_end; )//++it)
    {
      cv::Mat m; it >> m;
      Tmap[class_id_tmp].push_back( m );
    }

    /**Read K**/
    n = fs["Ks"];                         // Read string sequence - Get node
    if (n.type() != cv::FileNode::SEQ)
    {
      std::cerr << "strings is not a sequence! FAIL" << std::endl;
      return 0;
    }
    it = n.begin(), it_end = n.end(); // Go through the node
    for (; it != it_end;)//++it)
    {
      cv::Mat m; it >> m;
      Kmap[class_id_tmp].push_back( m );
    }

    /**Read Dist**/
    n = fs["dist"];                         // Read string sequence - Get node
    if (n.type() != cv::FileNode::SEQ)
    {
      std::cerr << "strings is not a sequence! FAIL" << std::endl;
      return 0;
    }
    it = n.begin(), it_end = n.end(); // Go through the node
    for (; it != it_end; )
    {
      float d; it >> d;
      dist_map[class_id_tmp].push_back( d );
    }

    /**Read HueHist**/
    n = fs["Hue"];                         // Read string sequence - Get node
    if (n.type() != cv::FileNode::SEQ)
    {
      std::cerr << "strings is not a sequence! FAIL" << std::endl;
      return 0;
    }
    it = n.begin(), it_end = n.end(); // Go through the node
    for (; it != it_end; )
    {
      cv::Mat m; it >> m;
      HueHist[class_id_tmp].push_back( m );
    }

  }



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

int main(int argc, char* argv[])
{
  namespace fs = boost::filesystem; 

  if(argc!=3)
  {
    std::cout<<"Usage: ./"<<argv[0]<<" "<<"path/to/objs_folders"<<" "<< "visualize as 0(false) 1(true)\n";
    return -1;
  }

  /** True or False to output debug image */
  bool visualize_ = (atoi(argv[2]) ? true:false);
  std::string objsfolder_path(argv[1]);

  /** File containing the names of the objects which must be trained */
  fs::path objNamesPath=objsfolder_path / fs::path("names.txt");
  if(!fs::exists(objNamesPath) || !fs::is_regular_file(objNamesPath)){
    std::cerr << "Sry, to run the train program make sure that " << objNamesPath << " exists and contains the names of the objects to train!\n";
    return -1;
  }


  std::set<std::string> objNames;
  std::cout << "Reading model names from " << objNamesPath << "..\n";
  {
    std::ifstream names(objNamesPath.string());
    while(names.good()){
      std::string s;
      names >> s;
      std::cout << "Will elaborate directory: " << s << "\n";
      objNames.insert(s);
    }
    if(!names.eof()){
      std::cout << "Could not read model names.";
      return -1;
    }
  }

  fs::path mesh_path;

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
  //Set the Values:
  // Define the display
  //assign the parameters of the renderer
  renderer_n_points_ = 150;//= param_n_points_;
  renderer_angle_step_ = 10;//param_angle_step_;
  renderer_radius_min_ = 0.6;//param_radius_min_;
  renderer_radius_max_ = 1.1;//param_radius_max_;
  renderer_radius_step_ = 0.4;//param_radius_step_;
  renderer_width_ = 640;//param_width_;
  renderer_height_ = 480;//param_height_;
  renderer_near_ = 0.1;//param_near_;
  renderer_far_ = 1000.0;//param_far_;
  renderer_focal_length_x_ = 525.0;//param_focal_length_x_;
  renderer_focal_length_y_ = 525.0;//param_focal_length_y_;


  //iterate over the Training Database
  fs::path targetDir(objsfolder_path); 

  fs::directory_iterator it(targetDir), eod;

  BOOST_FOREACH(fs::path const &p, std::make_pair(it, eod))   
  { 
    if(is_directory(p))
    {

      /** Check if the directory we found matches a renderable object */
      if(objNames.find(p.filename().string()) == objNames.end()){
        continue;
      }

      std::vector<cv::Mat>  Rs_;
      std::vector<cv::Mat>  Ts_;
      std::vector<float>  distances_;
      std::vector<cv::Mat>  Ks_;
      std::vector<cv::Mat>  HueHist_;

      //set the object id as the name of the object itself
      std::string object_id_(p.filename().string());
      std::cout<<"Elaborating " << object_id_<<"..\n";

      //Use our own models made in Blender....
      mesh_path = p / fs::path("textured_meshes") / fs::path(object_id_+".obj");
      std::cout<< "Mesh filename: " << mesh_path.filename().string() << "\n";

      cv::Ptr<cv::linemod::Detector> detector_ = cv::linemod::getDefaultLINEMOD();
      boost::shared_ptr<Renderer3d> rendererPtr (new Renderer3d(mesh_path.string()));
      rendererPtr->set_parameters(renderer_width_, renderer_height_, renderer_focal_length_x_, renderer_focal_length_y_, renderer_near_, renderer_far_);

      boost::shared_ptr<RendererIterator> rendererIteratorPtr (new RendererIterator(rendererPtr, renderer_n_points_));   
      rendererIteratorPtr->angle_step_ = renderer_angle_step_;
      rendererIteratorPtr->radius_min_ = float(renderer_radius_min_);
      rendererIteratorPtr->radius_max_ = float(renderer_radius_max_);
      rendererIteratorPtr->radius_step_ = float(renderer_radius_step_);

      cv::Mat image, depth, mask;
      cv::Matx33d R;
      cv::Vec3d T;
      cv::Matx33f K;
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
        K = cv::Matx33f(float(renderer_focal_length_x_), 0.0f, float(rect.width)/2.0f, 0.0f, float(renderer_focal_length_y_), float(rect.height)/2.0f, 0.0f, 0.0f, 1.0f);

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
        Ks_.push_back(cv::Mat(K));

        // Delete the status
        for (size_t j = 0; j < status.str().size(); ++j)
          std::cout << '\b';

      }//end for

      //write the template + R + t + dist + K for each class
      //writeLinemod(detector_,"/home/giorgio/testRender.yml");
      fs::path saveLinemodPath = p / fs::path(object_id_+"_Linemod.yml");
      //writeLinemod(detector_,saveLinemodPath.string());
      writeLinemodAndPoses(detector_, saveLinemodPath.string(), mesh_path.string(),
          Rs_, Ts_, Ks_, distances_, HueHist_);

      std::cout<<std::endl;
      std::cout<<"Rs_.size() "<< Rs_.size()<<"\n";  
      std::cout<<"Rs_[7] "<< Rs_[7]<<"\n";
      std::cout<<"HueHist_.size() "<< HueHist_.size()<<"\n";  
      std::cout<<"HueHist_[178] "<< HueHist_[178]<<"\n";                       
      //DEBUG
      std::cout<<"#DEBUG READ Back the just written data#"<<"\n";
      std::map<std::string,std::vector<cv::Mat> > Rmap;
      std::map<std::string,std::vector<cv::Mat> > Tmap;
      std::map<std::string,std::vector<cv::Mat> > Kmap;
      std::map<std::string,std::vector<cv::Mat> > HueHistmap;
      std::map<std::string,std::vector<float> > dist_map;
      std::string mesh_file_path_debug;
      //reading...
      readLinemodAndPoses(saveLinemodPath.string(), mesh_file_path_debug ,Rmap,Tmap,Kmap,dist_map,HueHistmap);

      std::cout<<"Rmap[object_id_].size() "<< Rmap[object_id_].size()<<"\n";
      std::cout<<"Rmap[object_id_][7] "<< Rmap[object_id_][7]<<"\n";
      std::cout<<"Tmap[object_id_].size() "<< Tmap[object_id_].size()<<"\n";
      std::cout<<"Kmap[object_id_].size() "<< Kmap[object_id_].size()<<"\n";
      std::cout<<"HueHistmap[object_id_].size() "<< HueHistmap[object_id_].size()<<"\n";
      std::cout<<"HueHistmap[object_id_][178] "<< HueHistmap[object_id_][178]<<"\n";        
      std::cout<<"dist_map[object_id_].size() "<< dist_map[object_id_].size()<<"\n";
      std::cout<<"mesh_file_path_debug "<< mesh_file_path_debug<<"\n";


      } //end if(si_directory(p))
}//end BOOST_FOREACH

return 0;
}
