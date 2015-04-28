#include <fstream>
#include <iostream>
#include <limits>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/rgbd.hpp>


#include "db_linemod.h"

#include "utils.h"
#include "renderer3d.h"

#include "linemod_icp.h"

//PCL to debug...
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>

//PCL ICP
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <opencv2/core/eigen.hpp>

#include <algorithm>    // std::min_element, std::max_element


namespace fs = boost::filesystem; 

/*********  FFW Declarations  ************/
static cv::Ptr<cv::linemod::Detector> readLinemodAndPoses(const std::string& filename, std::string& mesh_file_path,
                                                          std::map<std::string,std::vector<cv::Mat> >& Rmap,
                                                          std::map<std::string,std::vector<cv::Mat> >& Tmap,
                                                          std::map<std::string,std::vector<cv::Mat> >& Kmap,
                                                          std::map<std::string,std::vector<float> >& dist_map,
                                                          std::map<std::string,std::vector<cv::Mat> >& HueHist);


bool updateGiorgio(cv::Mat& rgb, const cv::Mat& depth_meter, 
                  cv::Ptr<cv::linemod::Detector>& detector_, std::map<std::string, boost::shared_ptr<RendererIterator> >& renderer_iterators_, 
                  std::map<std::string,std::vector<cv::Mat> >& Rs_ , std::map<std::string,std::vector<cv::Mat> >& Ts_, 
                  std::map<std::string,std::vector<cv::Mat> >& Ks_ , std::map<std::string,std::vector<float> >& distances_, std::map<std::string,std::vector<cv::Mat> >& HueHist_,
                  cv::Mat_<float>& Pose, const std::vector<std::string>& vect_objs_to_pick);
                  
                  
void
drawResponse(const std::vector<cv::linemod::Template>& templates, int num_modalities, cv::Mat& dst, cv::Point offset,
             int T);
/*********END  FFW Declarations  ************/


/******** PCL VIZ DEBUG Function *********/
void rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer_, std::string name)
{
  // --------------------------------------------
  // -----Open 3D viewer_ and add point cloud-----
  // --------------------------------------------
  if(!viewer_)
  {
     viewer_.reset(new pcl::visualization::PCLVisualizer ("3D viewer_"));
     viewer_->setBackgroundColor (0, 0, 0);
     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_(cloud);
     viewer_->addPointCloud<pcl::PointXYZRGB> (cloud, rgb_, name);
     viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
     viewer_->addCoordinateSystem (1.0);
     viewer_->initCameraParameters ();
     std::cout<<"!viewer_"<<"\n";
  }
  else
  {
     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_(cloud);
     viewer_->addPointCloud<pcl::PointXYZRGB> (cloud, rgb_, name);
     viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
     std::cout<<"viewer_"<<"\n";
  }
  //return (viewer_);
}

/********END PCL VIZ DEBUG Function *********/


void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


void checkPointsInsidePoly(cv::Mat& rgb, const cv::Rect& roi, const std::vector<cv::Point>& poly_vertices, cv::Mat& mask_out)
{
      int nvert = poly_vertices.size();
      mask_out = cv::Mat::zeros(rgb.size(),CV_8UC1);
//      std::cout<<"poly_vertices.size() "<<nvert<<"\n";
      //for each query point in rgb
      //int rgb_size = rgb.rows*rgb.cols;
      for(int y = roi.y; y < roi.y + roi.height; y++)
      {
         for(int x = roi.x; x < roi.x + roi.width; x++)
         {
//            std::cout<<"x: "<<x<<"\n";
//            std::cout<<"y: "<<y<<"\n";
            int testx = x;
            int testy = y;
            int i, j;
            bool c = false;
            for (i = 0, j = nvert-1; i < nvert; j = i++) 
            {
             if ( ((poly_vertices[i].y>=testy) != (poly_vertices[j].y>=testy)) && (testx <= (poly_vertices[j].x-poly_vertices[i].x) * (testy-poly_vertices[i].y) / (poly_vertices[j].y-poly_vertices[i].y) + poly_vertices[i].x) )
                c = !c;
            }
            if (c)
            {
//               std::cout<<"c==1: "<<"\n";
               rgb.at<cv::Vec3b>(y,x)[0] = 255;
               rgb.at<cv::Vec3b>(y,x)[1] = 255;
               rgb.at<cv::Vec3b>(y,x)[2] = 0;
               
               mask_out.at<uint8_t>(y,x) = 255;
            }
         }
     }
}


//Training Params
const int renderer_n_points_ = 150;
const int renderer_angle_step_ = 10;
const float renderer_radius_min_ = 0.6;
const float renderer_radius_max_ = 1.1;
const float renderer_radius_step_ = 0.4;
const int renderer_width_ = 640;
const int renderer_height_ = 480;
const float renderer_near_ = 0.1;
const float renderer_far_ = 1000.0;
const float renderer_focal_length_x_ = 525.0;
const float renderer_focal_length_y_ = 525.0;

//Detector Params
float threshold_; //"threshold", "Matching threshold, as a percentage", 93.0f
const float th_obj_dist_ = 0.04f; //"th_obj_dist", "Threshold on minimal distance between detected objects.", 0.04f);
const float icp_dist_min_ = 0.06f;
const float px_match_min_ = 0.25f;

//Depth Camera Matrix
//TODO: Safier to read it from the calibration file
const double dfx_ = 598.197811409705;
const double dfy_ = 593.372644257344;
const double dcx_ = 309.086101744595;
const double dcy_ = 243.497417130779;
const cv::Mat K_depth_ = (cv::Mat_<double>(3,3) << dfx_, 0, dcx_, 0, dfy_, dcy_, 0, 0, 1);


//struct ObjectLoaded
//{
//   std::string object_id;
//   RendererIterator* render_iterator;
//   std::vector<cv::Mat> R;
//   std::vector<cv::Mat> K;
//   std::vector<cv::Mat> T;
//   std::vector<float> dist;
//   
//};

int main(int argc, char* argv[])
{


   if(argc!=5)
    {
      std::cout<<"Usage: ./"<<argv[0]<<" "<<"path/to/objs_folders"<<" "<< "path/to/depth&rgb.yml"<<" "<<"The Only object name to pick"<<" threshold Percentage match" <<"\n";
      return -1;
    }
    
    std::cout<<"K_depth_:\n"<<K_depth_<<"\n";
    
    threshold_ = atof(argv[4]); //"threshold", "Matching threshold, as a percentage", 93.0f


   std::string objsfolder_path(argv[1]);
   
   //entry point simulate RGB and DEPTH images given by Simo
   //Read martrices from yml file
   cv::Mat depth_meters, depth_gray, rgb_img;
   
   cv::FileStorage file(argv[2], cv::FileStorage::READ);
   file["depth_mm"] >> depth_meters;
   file["depth_gray_img"] >> depth_gray;//for visualization purposes
   file["rgb"] >> rgb_img;
   
   file.release();
   
   //Debug Shows images
   cv::imshow("rgb",rgb_img);
   cv::imshow("depth_gray",depth_gray);
   
   //cv::waitKey();//Hit enter to continue
   
   /*** Init Process at Start-Up: only Once !!! ***/
   
   /** LINE-MOD detector */
    cv::Ptr<cv::linemod::Detector> detector_ = cv::linemod::getDefaultLINEMOD();
   /** The renderer initialized with objects meshes, per object*/
    //std::map<std::string, RendererIterator*> renderer_iterators_;
    std::map<std::string, boost::shared_ptr<RendererIterator> > renderer_iterators_;
   /** maps to cache the poses **/
   std::map<std::string,std::vector<cv::Mat> > Rmap;
   std::map<std::string,std::vector<cv::Mat> > Tmap;
   std::map<std::string,std::vector<cv::Mat> > Kmap;
   std::map<std::string,std::vector<cv::Mat> > HueHistmap;
   std::map<std::string,std::vector<float> > dist_map;
    
    //Load the detector_ with the objects in the DB
    //iterate over the Training Database
    fs::path targetDir(objsfolder_path); 

    fs::directory_iterator it(targetDir), eod;
    BOOST_FOREACH(fs::path const &p, std::make_pair(it, eod))   
    { 
        if(is_directory(p))
        {
         //std::cout<<p.filename().string()<<"\n";
         //TODO: For Now only Crayola is Loaded...Remove it in the future
         if( (p.filename().string().compare("crayola_64_ct") !=0) && (p.filename().string().compare("genuine_joe_plastic_stir_sticks") !=0) && 
                  (p.filename().string().compare("highland_6539_self_stick_notes") !=0)  && (p.filename().string().compare("paper_mate_12_count_mirado_black_warrior") !=0)
                  && (p.filename().string().compare("mark_twain_huckleberry_finn") !=0) )
               continue;
         
         //object_id
         std::string object_id_ = p.filename().string();
         
         std::cout<<"-object_id_: "<<object_id_<<"\n";
         
         //Load the yml of that class obtained in the Training phase
         fs::path saveLinemodPath = p / fs::path(object_id_+"_Linemod.yml");
         
         
         std::string mesh_file_path;
         std::string saveLinemodPathString(saveLinemodPath.string());
         //reading...
         cv::Ptr<cv::linemod::Detector> detector = readLinemodAndPoses(saveLinemodPathString,mesh_file_path,Rmap,Tmap,Kmap,dist_map,HueHistmap);
         //DEBUG
         std::cout<<"\tnumTemplates: "<<detector->numTemplates()<<"\n";
         std::cout<<"\tmesh_file_path_debug: "<< mesh_file_path<<"\n";
         //add class template to detector_
         for (size_t template_id = 0; template_id < detector->numTemplates(); ++template_id) {
           const std::vector<cv::linemod::Template> &templates_original = detector->getTemplates(object_id_, template_id);
           detector_->addSyntheticTemplate(templates_original, object_id_);
         }
         //std::cout<<"new *renderer"<<"\n";
         
         

//*********TODO: BOOST SHARED POINTERS   change the RendererIterator          
         boost::shared_ptr<Renderer3d> rendererPtr (new Renderer3d(mesh_file_path));
         rendererPtr->set_parameters(renderer_width_, renderer_height_, renderer_focal_length_x_, renderer_focal_length_y_, renderer_near_, renderer_far_);
//         std::cout<<"1- rendererPtr.use_count(): "<<rendererPtr.use_count()<<"\n";
         
         
         boost::shared_ptr<RendererIterator> rendererIteratorPtr (new RendererIterator(rendererPtr, renderer_n_points_)); 
//         std::cout<<"2- rendererPtr.use_count(): "<<rendererPtr.use_count()<<"\n";  
//         std::cout<<"2- rendererIteratorPtr.use_count(): "<<rendererIteratorPtr.use_count()<<"\n";
         rendererIteratorPtr->angle_step_ = renderer_angle_step_;
         rendererIteratorPtr->radius_min_ = float(renderer_radius_min_);
         rendererIteratorPtr->radius_max_ = float(renderer_radius_max_);
         rendererIteratorPtr->radius_step_ = float(renderer_radius_step_);
         renderer_iterators_.insert(std::pair<std::string,boost::shared_ptr<RendererIterator> >(object_id_, rendererIteratorPtr));
//         std::cout<<"3- rendererIteratorPtr.use_count(): "<<rendererIteratorPtr.use_count()<<"\n";
        }//end if(is_directory(p))
    }//end BOOST_FOREACH
    
    
    //END of the Init Process: Now Simo can call updateGiorgio() whenever he wants...
   
   
   //Let's Start....
   
   //Indicate the Only object to Pick
   //TODO: Multi Objects to Pick!
   std::vector<std::string> vect_objs_to_pick_; vect_objs_to_pick_.resize(1);
   std::string temp_str = std::string(argv[3]);
   std::cout<<"Object To Pick: "<<temp_str<<"\n";
   vect_objs_to_pick_[0] = temp_str;
   cv::Mat_<float> pose;
   updateGiorgio(rgb_img, depth_meters,
                 detector_, renderer_iterators_ , Rmap , Tmap , Kmap , dist_map, HueHistmap,
                 pose, vect_objs_to_pick_);


   return 0;
}


bool updateGiorgio(cv::Mat& rgb, const cv::Mat& depth_meter, 
                  cv::Ptr<cv::linemod::Detector>& detector_, std::map<std::string, boost::shared_ptr<RendererIterator> >& renderer_iterators_, 
                  std::map<std::string,std::vector<cv::Mat> >& Rs_ , std::map<std::string,std::vector<cv::Mat> >& Ts_, 
                  std::map<std::string,std::vector<cv::Mat> >& Ks_ , std::map<std::string,std::vector<float> >& distances_, std::map<std::string,std::vector<cv::Mat> >& HueHist_,
                  cv::Mat_<float>& Pose, const std::vector<std::string>& vect_objs_to_pick)
{

   //The depth_ matrix is given in Meters CV_32F == 5
   //std::cout<<"depth_meter.depth(): "<<depth_meter.depth()<<"\n";
   //std::cout<<"depth_meter.type(): "<<depth_meter.type()<<"\n";
   
   //TODO: Speed Up w/ pointers in Continous matrices
   cv::Mat depth_mm(depth_meter.size(),CV_16UC1);
   for (int r=0; r<depth_meter.rows; ++r)
   {
      for (int c=0; c<depth_meter.cols; ++c)
      {
         float px = depth_meter.at<float>(r,c);
         if(std::isfinite(px))
         {
            uint16_t p = static_cast<uint16_t>(px*1000.0);
            depth_mm.at<uint16_t>(r,c) = p;
         }
         else
         {
            depth_mm.at<uint16_t>(r,c) = 0;
         }
         
      }
   }
   
   //ONLY FOR DEBUG
//   double minVal, maxVal;
//   cv::Mat depth_gray_img;
//   cv::minMaxLoc(depth_mm,&minVal,&maxVal);
//   depth_mm.convertTo(depth_gray_img,CV_8U,255.0/(maxVal - minVal), -minVal*255.0/(maxVal-minVal));
//   
//   cv::imshow("reconstructed depth",depth_mm);
//   cv::waitKey();
   //

   if (detector_->classIds().empty())
   {   
      std::cout<<"WARNING: detector_->classIds().empty()"<<"\n";
      return false;
   }

   std::vector<cv::Mat> sources;
   
   //TODO: Resize color to 640x480 if neeeded
   std::cout<<"rgb.size(): [cols,rows] "<<rgb.size()<<"\n";
   sources.push_back(rgb);
   sources.push_back(depth_mm);
   
//   cv::imshow("rgb2",sources[0]);
//   std::cout<<"sources[1].size()"<<sources[1].size()<<"\n";
//   
//   cv::waitKey();
   
   std::cout<<"matching..."<<"\n";
   std::vector<cv::linemod::Match> matches;
   //TODO: Use also mask to represents a valid pixel in order to reduce the search space !!!
   detector_->match(sources, threshold_, matches,vect_objs_to_pick);

   std::cout<<"Done: matches.size(): "<<matches.size()<<"\n";

   int num_modalities = (int) detector_->getModalities().size();
   
   cv::Mat_<cv::Vec3f> depth_real_ref_raw;
   cv::Mat_<float> K;
   K_depth_.convertTo(K, CV_32F);
   cv::rgbd::depthTo3d(depth_meter, K, depth_real_ref_raw);
   
   /** The buffer with detected objects and their info */
    std::vector <object_recognition_core::db::ObjData> objs_;
    int iter = 0;
    
    std::vector<std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > > vectVectPCPtr;
    
    vectVectPCPtr.resize(matches.size());
    for(int ix =0; ix<matches.size(); ++ix)
    {
      vectVectPCPtr[ix].resize(3); //1)Model,2)Ref,3)Aligned PC
    }
    
    BOOST_FOREACH(const cv::linemod::Match & match, matches) {
    
      
    
      const std::vector<cv::linemod::Template>& templates =
          detector_->getTemplates(match.class_id, match.template_id);

// DEBUG
        drawResponse(templates, num_modalities, rgb,
            cv::Point(match.x, match.y), detector_->getT(0));
        
         cv::imshow("rgb",rgb);
        //cv::waitKey();  

      // Fill the Pose object
      cv::Matx33d R_match = Rs_.at(match.class_id)[match.template_id].clone();
      cv::Vec3d T_match = Ts_.at(match.class_id)[match.template_id].clone();
      float D_match = distances_.at(match.class_id)[match.template_id];
      cv::Mat K_match = Ks_.at(match.class_id)[match.template_id];  
      
      //get the point cloud of the rendered object model
      cv::Mat mask;
      cv::Rect rect;
      cv::Matx33d R_temp(R_match.inv());
      cv::Vec3d up(-R_temp(0,1), -R_temp(1,1), -R_temp(2,1));
      std::cout<<"***match.class_id: ***"<<match.class_id<<"\n";
//      RendererIterator* it_r = renderer_iterators_.at(match.class_id);
      boost::shared_ptr<RendererIterator> it_r = renderer_iterators_.at(match.class_id);
      cv::Mat depth_ref_;
      it_r->renderDepthOnly(depth_ref_, mask, rect, -T_match, up);
      
      //cv::imshow("depth_ref_",depth_ref_);
      cv::Mat mask_copy; mask.copyTo(mask_copy);
      cvtColor(mask_copy,mask_copy,CV_GRAY2BGR);
      cv::imshow("mask",mask_copy);
      cv::waitKey();
      
      cv::Mat_<cv::Vec3f> depth_real_model_raw;
      cv::rgbd::depthTo3d(depth_ref_, K_match, depth_real_model_raw);
      
      //prepare the bounding box for the model and reference point clouds
      cv::Rect_<int> rect_model(0, 0, depth_real_model_raw.cols, depth_real_model_raw.rows);
      //prepare the bounding box for the reference point cloud: add the offset
      cv::Rect_<int> rect_ref(rect_model);
      rect_ref.x += match.x;
      rect_ref.y += match.y;
      
      rect_ref = rect_ref & cv::Rect(0, 0, depth_real_ref_raw.cols, depth_real_ref_raw.rows);
      if ((rect_ref.width < 5) || (rect_ref.height < 5))
        continue;
      //adjust both rectangles to be equal to the smallest among them
      if (rect_ref.width > rect_model.width)
        rect_ref.width = rect_model.width;
      if (rect_ref.height > rect_model.height)
        rect_ref.height = rect_model.height;
      if (rect_model.width > rect_ref.width)
        rect_model.width = rect_ref.width;
      if (rect_model.height > rect_ref.height)
        rect_model.height = rect_ref.height;

      //prepare the reference data: from the sensor : crop images
      cv::Mat_<cv::Vec3f> depth_real_ref = depth_real_ref_raw(rect_ref);
      //prepare the model data: from the match
      cv::Mat_<cv::Vec3f> depth_real_model = depth_real_model_raw(rect_model);
      
      //plot Red Rect around the match in the full image
      cv::rectangle(rgb,rect_ref,cv::Scalar(0,0,255),2);
 
      cv::imshow("rgb",rgb);

      cv::waitKey();


      //initialize the translation based on reference data
      cv::Vec3f T_crop = depth_real_ref(depth_real_ref.rows / 2.0f, depth_real_ref.cols / 2.0f);
      //add the object's depth
      T_crop(2) += D_match;
      
      if (!cv::checkRange(T_crop))
        continue;
      cv::Vec3f T_real_icp(T_crop);
      
      //initialize the rotation based on model data
      if (!cv::checkRange(R_match))
        continue;
      cv::Matx33f R_real_icp(R_match);
      
      //get the point clouds (for both reference and model)
      std::vector<cv::Vec3f> pts_real_model_temp;
      std::vector<cv::Vec3f> pts_real_ref_temp;
      float px_ratio_missing = matToVec(depth_real_model, depth_real_ref, pts_real_model_temp, pts_real_ref_temp);
      std::cout<<"px_ratio_missing > px_match_min_ ?: "<<px_ratio_missing<<" > " <<  px_match_min_ <<"\n";
      if (px_ratio_missing > px_match_min_)
        continue;
      
      /***** DEFAULT ICP *****/  
    
      /*****END DEFAULT ICP *****/  

      
      /***** PCL ICP *****/  
      /*** ***/
      //Conversion Mat -> PCL
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelCloudPtr (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr refCloudPtr (new pcl::PointCloud<pcl::PointXYZRGB>);
      
      /*MODEL Cloud*/
      modelCloudPtr->points.resize(pts_real_model_temp.size());
      modelCloudPtr->width =  modelCloudPtr->points.size();
      modelCloudPtr->height = 1;
      modelCloudPtr->is_dense = true;
      
      //TODO: Speed Up using Mat pointers
      for(int ii=0;ii<modelCloudPtr->points.size();++ii)
      {
         modelCloudPtr->points[ii].x =  pts_real_model_temp[ii][0];
         modelCloudPtr->points[ii].y =  pts_real_model_temp[ii][1];
         modelCloudPtr->points[ii].z =  pts_real_model_temp[ii][2];
         
         modelCloudPtr->points[ii].r = 0;
         modelCloudPtr->points[ii].g = 255;
         modelCloudPtr->points[ii].b = 0;
         
      }
      /* END MODEL Cloud*/
      
      /*REF Cloud*/
      refCloudPtr->points.resize(pts_real_ref_temp.size());
      refCloudPtr->width =  refCloudPtr->points.size();
      refCloudPtr->height = 1;
      refCloudPtr->is_dense = true;
      
      //TODO: Speed Up using Mat pointers
      for(int ii=0;ii<refCloudPtr->points.size();++ii)
      {
         refCloudPtr->points[ii].x =  pts_real_ref_temp[ii][0];
         refCloudPtr->points[ii].y =  pts_real_ref_temp[ii][1];
         refCloudPtr->points[ii].z =  pts_real_ref_temp[ii][2];
         
         refCloudPtr->points[ii].r = 255;
         refCloudPtr->points[ii].g = 0;
         refCloudPtr->points[ii].b = 0;

      }
      /* END REF Cloud*/
      
      pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
      icp.setMaximumIterations (10);
      icp.setInputSource (modelCloudPtr);//Model
      icp.setInputTarget (refCloudPtr);//Ref scene
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalModelCloudPtr (new pcl::PointCloud<pcl::PointXYZRGB>);
      
      //Get T matrix
      cv::Mat Tguess(4,4,CV_32F);
      cv::Mat Tguess_temp(3,4,CV_32F);
      cv::Matx33f R_real_icp_pcl(R_match);
      //std::cout<<"T_real_icp: \n"<<T_crop[0] <<";"<<T_crop[1]<<";"<<T_crop[2]<<"\n";
      cv::hconcat(R_real_icp_pcl,T_crop,Tguess_temp);
      //std::cout<<"Tguess_temp: \n"<<Tguess_temp<<"\n";
      cv::Mat lastrow = cv::Mat::zeros(1,4,CV_32F); lastrow.at<float>(0,3) = 1.0f;
      //std::cout<<"lastrow: \n"<<lastrow<<"\n";
      cv::vconcat(Tguess_temp,lastrow,Tguess);
      
      //std::cout<<"Tguess: \n"<<Tguess<<"\n";
      
      //Initial Pose Guess
      std::cout<<"ITER: "<<iter<<"\n";
      Eigen::Matrix4f initGuessPoseMat;
      cv2eigen(Tguess,initGuessPoseMat);
      std::cout<<"initGuessPoseMat: \n"<<initGuessPoseMat<<"\n";
      icp.align (*finalModelCloudPtr);//,initGuessPoseMat);
      Eigen::Matrix4d finalTransformationMatrix = Eigen::Matrix4d::Identity ();
      finalTransformationMatrix = icp.getFinalTransformation().cast<double>();
      
      Eigen::Matrix4d initGuessPoseMatdouble = initGuessPoseMat.cast<double>();
      
      std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
      
      std::cout<<"finalTransformationMatrix: \n"<<initGuessPoseMatdouble*finalTransformationMatrix<<"\n";
      
      vectVectPCPtr[iter][0] = modelCloudPtr;
      vectVectPCPtr[iter][1] = refCloudPtr;
      
      //Color the aligned PC
      for(int ii=0;ii<finalModelCloudPtr->points.size();++ii)
      {
         
         
         finalModelCloudPtr->points[ii].r = 0;
         finalModelCloudPtr->points[ii].g = 0;
         finalModelCloudPtr->points[ii].b = 255;
         
      } 
      
      vectVectPCPtr[iter][2] = finalModelCloudPtr;
      

      /*** ***/
      /*****END PCL ICP *****/ 
      
//      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; viewer.reset();

//      rgbVis(modelCloudPtr,boost::ref(viewer),"model");

// 
//      rgbVis(refCloudPtr,boost::ref(viewer),"ref");
//      
//      //Color the aligned PC
//      for(int ii=0;ii<finalModelCloudPtr->points.size();++ii)
//      {
//         
//         
//         finalModelCloudPtr->points[ii].r = 0;
//         finalModelCloudPtr->points[ii].g = 0;
//         finalModelCloudPtr->points[ii].b = 255;
//         
//      } 
//      
//        
//      
//      rgbVis(finalModelCloudPtr,boost::ref(viewer),"aligned");
//      while (!viewer->wasStopped ())
//        {
//          viewer->spinOnce (100);
//          boost::this_thread::sleep (boost::posix_time::microseconds (100000));
////          char k='o';
////          k = cv::waitKey(100);
////          if (k=='c')
////          {
////            viewer->close();
////          }
//            
//        }
      
      
      
      //keep the object match
     // objs_.push_back(object_recognition_core::db::ObjData(pts_real_ref_temp, pts_real_model_temp, match.class_id, match.similarity, icp_dist , px_ratio_match_inliers, R_real_icp, T_crop));
           
      ++iter;      
   }//END BOOST_FOREACH
   
   
   //Visualize all the matches
    for(int ix =0; ix<vectVectPCPtr.size(); ++ix)
    {
      //1)Model,2)Ref,3)Aligned PC
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; viewer.reset();

      rgbVis(vectVectPCPtr[ix][0],boost::ref(viewer),"model");

      rgbVis(vectVectPCPtr[ix][1],boost::ref(viewer),"ref");
      
      rgbVis(vectVectPCPtr[ix][2],boost::ref(viewer),"aligned");
      
      while (!viewer->wasStopped ())
      {
          viewer->spinOnce (100);
          boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//          char k='o';
//          k = cv::waitKey(100);
//          if (k=='c')
//          {
//            viewer->close();
//          }
            
      }
      
    }
   
   
   
   return true;
}


void
drawResponse(const std::vector<cv::linemod::Template>& templates, int num_modalities, cv::Mat& dst, cv::Point offset,
             int T)
{
  static const cv::Scalar COLORS[5] =
  { CV_RGB(0, 0, 255), CV_RGB(0, 255, 0), CV_RGB(255, 255, 0), CV_RGB(255, 140, 0), CV_RGB(255, 0, 0) };
  if (dst.channels() == 1)
    cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);

  cv::circle(dst, cv::Point(offset.x + 20, offset.y + 20), T / 2, COLORS[4]);
  if (num_modalities > 5)
    num_modalities = 5;
  for (int m = 0; m < num_modalities; ++m)
  {
// NOTE: Original demo recalculated max response for each feature in the TxT
// box around it and chose the display color based on that response. Here
// the display color just depends on the modality.
    cv::Scalar color = COLORS[m];

    for (int i = 0; i < (int) templates[m].features.size(); ++i)
    {
      cv::linemod::Feature f = templates[m].features[i];
      cv::Point pt(f.x + offset.x, f.y + offset.y);
      cv::circle(dst, pt, T / 2, color);
    }
  }
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

