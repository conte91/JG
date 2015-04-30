/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

// MY MODs...

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

//#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

//#include <object_recognition_core/common/json.hpp>
//#include <object_recognition_core/db/db.h>
//#include <object_recognition_core/db/document.h>
//#include <object_recognition_core/db/model_utils.h>

#include "renderer3d.h"
#include "utils.h"

//#if LINEMOD_VIZ_IMG
  #include <opencv2/highgui/highgui.hpp>
  
namespace fs = boost::filesystem; 

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
//std::cout<<"-1"<<"\n";
  cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector->read(fs.root());
//std::cout<<"0"<<"\n";

   //Read mesh_file_path
   fs["mesh_file_path"] >> mesh_file_path;

  cv::FileNode fn = fs["classes"];
  //size_t num_classes=0;
  //for each class:
  //int forRot=0;
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
  {
  
    //std::cout<<"1"<<"\n";
    std::string class_id_tmp = (*i)["class_id"];
    std::cout<<"class_id_tmp: "<<class_id_tmp <<"\n";
    detector->readClass(*i);
    //std::cout<<"2"<<"\n";
    //cv::FileStorage fs2(filename, cv::FileStorage::READ);
    //std::cout<<"3"<<"\n";
    /**Read R**/
    cv::FileNode n = fs["Rot"];                         // Read string sequence - Get node
    if (n.type() != cv::FileNode::SEQ)
    {
      std::cerr << "strings is not a sequence! FAIL" << std::endl;
      return 0;
    }
    //std::cout<<"4"<<"\n";
    cv::FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
    for (; it != it_end;) //++it)
    {
      cv::Mat m; it >> m;
      Rmap[class_id_tmp].push_back( m );
      //++forRot;
    }
    //std::cout<<"ForRor: "<<forRot<<"\n";
      
    /**Read T**/
    n = fs["Transl"];                         // Read string sequence - Get node
    if (n.type() != cv::FileNode::SEQ)
    {
      std::cerr << "strings is not a sequence! FAIL" << std::endl;
      return 0;
    }
    //std::cout<<"5"<<"\n";
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
    //std::cout<<"6"<<"\n";
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
    //std::cout<<"7"<<"\n";
    it = n.begin(), it_end = n.end(); // Go through the node
    for (; it != it_end; )//++it)
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
    //std::cout<<"7"<<"\n";
    it = n.begin(), it_end = n.end(); // Go through the node
    for (; it != it_end; )//++it)
    {
      cv::Mat m; it >> m;
      HueHist[class_id_tmp].push_back( m );
    }
    
    //++num_classes;
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

    if(argc!=4)
    {
      std::cout<<"Usage: ./"<<argv[0]<<" "<<"path/to/objs_folders"<<" "<< "visualize as 0(false) 1(true)"<<" [Object Name to Train]!!Not Used Anymore!!"<<"\n";
      return -1;
    }

    /** True or False to output debug image */
    bool visualize_ = (atoi(argv[2]) ? true:false);
    /** The DB parameters as a JSON string */
    //std::string json_db_;
    std::string objsfolder_path(argv[1]);
    //std::string mesh_path;
    fs::path mesh_path;
    /** The id of the object to generate a trainer for */
    std::string object_id_;
    //cv::linemod::Detector detector_;
    std::vector<cv::Mat>  Rs_;
    std::vector<cv::Mat>  Ts_;
    std::vector<float>  distances_;
    std::vector<cv::Mat>  Ks_;
    std::vector<cv::Mat>  HueHist_;

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
    
    
    //TODO: I did this due to the error freeglut illegal glutInit() reinitialization attempt when train multiple object in a loop
    //No time to solve it now!!!!
    //TODO: OK SOLVED !!! no use it anymore!
    std::string object_name_to_train = std::string(argv[3]);
    
    //iterate over the Training Database
    fs::path targetDir(objsfolder_path); 

    fs::directory_iterator it(targetDir), eod;

    BOOST_FOREACH(fs::path const &p, std::make_pair(it, eod))   
    { 
        if(is_directory(p))
        {
        
            //TODO: For Now only Crayola is Loaded...Remove it in the future
            if( (p.filename().string().compare("crayola_64_ct") !=0) && (p.filename().string().compare("genuine_joe_plastic_stir_sticks") !=0) && 
                  && (p.filename().string().compare("champion_copper_plus_spark_plug")!=0) && (p.filename().string().compare("highland_6539_self_stick_notes") !=0)  && (p.filename().string().compare("paper_mate_12_count_mirado_black_warrior") !=0)
                  && (p.filename().string().compare("mark_twain_huckleberry_finn") !=0) )
               continue;
         
        
            //clear old values
            Rs_.clear();
            Ts_.clear();
            distances_.clear();
            Ks_.clear();
            HueHist_.clear();
            //set the object id as the name of the object itself
            object_id_ = p.filename().string();
            std::cout<<object_id_<<"\n";
            
            //READ the metadata.yaml to see if the .ply file is the one suggested.
//             fs::path meta_path = p / fs::path("metadata.yaml");
//             std::ifstream metadata_yaml_file(meta_path.string().c_str());
//             //line 12 has: recommended_mesh: poisson
//             metadata_yaml_file.seekg(std::ios::beg);
//             for(int i=0; i < 12 - 1; ++i){
//                 metadata_yaml_file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
//             }
//             std::string line12;
//             metadata_yaml_file >> line12;
//             std::string recommended_mesh;
//             metadata_yaml_file >> recommended_mesh;
//             //std::cout<<recommended_mesh<<"\n";
//             //Search for the relative .obj file
//             metadata_yaml_file.close();
//             if(recommended_mesh.compare("poisson")==0)
//             {
//               mesh_path = p / fs::path("textured_meshes") / fs::path("optimized_poisson_texture_mapped_mesh.obj"); //("crayolaTextured.obj");
//               std::cout<<mesh_path.filename().string()<<"\n"; 
//               
//             }
//             else if(recommended_mesh.compare("tsdf_complete")==0)
//             {
//               mesh_path = p / fs::path("textured_meshes") / fs::path("completed_tsdf_texture_mapped_mesh.obj");
//               std::cout<<mesh_path.filename().string()<<"\n";
//             }
//             else if(recommended_mesh.compare("depth_integrated")==0)
//             {
//               mesh_path = p / fs::path("textured_meshes") / fs::path("depth_integrated_visual_hull_culled_sr_smoothed.obj");
//               std::cout<<mesh_path.filename().string()<<"\n";
//             }
//             else if(recommended_mesh.compare("visual_hull")==0)
//             {
//               mesh_path = p / fs::path("textured_meshes") / fs::path("visual_hull_refined_smoothed.obj");
//               std::cout<<mesh_path.filename().string()<<"\n";
//             }
//             else
//             {
//               std::cout<<"ERROR: no match for recommended mesh: "<<recommended_mesh<<"\n";
//               return -1;
//             }

               //Use our own models made in Blender....
               std::string object_id_obj = object_id_ + std::string(".obj");
               mesh_path = p / fs::path("textured_meshes") / fs::path(object_id_obj);
               std::cout<<mesh_path.filename().string()<<"\n";
             
            
//        } //end if(si_directory(p))
//    }//end BOOST_FOREACH
    //return 0;
    
          cv::Ptr<cv::linemod::Detector> detector_ = cv::linemod::getDefaultLINEMOD();
          //*detector_ //= *detector_ptr;

          // the model name can be specified on the command line.
//          Renderer3d renderer = Renderer3d(mesh_path.string());
//          renderer.set_parameters(renderer_width_, renderer_height_, renderer_focal_length_x_,
//                                  renderer_focal_length_y_, renderer_near_, renderer_far_);
         boost::shared_ptr<Renderer3d> rendererPtr (new Renderer3d(mesh_path.string()));
         rendererPtr->set_parameters(renderer_width_, renderer_height_, renderer_focal_length_x_, renderer_focal_length_y_, renderer_near_, renderer_far_);

          //cancella il file letto  
          //std::remove(objsfolder_path.c_str());

//          RendererIterator renderer_iterator = RendererIterator(&renderer, renderer_n_points_);
//          //set the RendererIterator parameters
//          renderer_iterator.angle_step_ = renderer_angle_step_;
//          renderer_iterator.radius_min_ = float(renderer_radius_min_);
//          renderer_iterator.radius_max_ = float(renderer_radius_max_);
//          renderer_iterator.radius_step_ = float(renderer_radius_step_);

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
            status << "Loading images " << (i+1) << "/"
                << rendererIteratorPtr->n_templates();
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
            
//            std::cout<<"image.size(): "<< image.size()<<"\n";
//            std::cout<<"image.type(): "<< image.type()<<"\n";
//            std::cout<<image<<"\n";
//            return 0;

      //#if LINEMOD_VIZ_IMG
            // Display the rendered image
            if (visualize_)
            {
              cv::namedWindow("Rendering");
              if (!image.empty()) {
                cv::imshow("Rendering", image);
                cv::waitKey(10);
              }
            }
      //#endif
      
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
          
          //only 1 class
          //return 0;

            //return ecto::OK;
          //}
          
      } //end if(si_directory(p))
    }//end BOOST_FOREACH

    return 0;
}
  //};
//} // namespace ecto_linemod

//ECTO_CELL(ecto_linemod, ecto_linemod::Trainer, "Trainer", "Train the LINE-MOD object detection algorithm.")
