#include <Recognition/RecognitionData.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/core/eigen.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <cmath>
#include <opencv2/rgbd.hpp>
#include <Recognition/db_linemod.h>
#include <Recognition/linemod_icp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
//PCL ICP
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Recognition/Renderer3d.h>
#include <Recognition/GiorgioUtils.h>
#include <Recognition/GLUTInit.h>

static double _threshold;
static cv::Mat K_depth_;
static float px_match_min_;
static float icp_dist_min_;
static float th_obj_dist_;


namespace Recognition{

  std::string RecognitionData::current_default_path;

static std::shared_ptr<cv::linemod::Detector> readLinemodAndPoses(const std::string& filename, std::string& mesh_file_path,
    std::map<std::string,std::vector<cv::Mat> >& Rmap,
    std::map<std::string,std::vector<cv::Mat> >& Tmap,
    std::map<std::string,std::vector<cv::Mat> >& Kmap,
    std::map<std::string,std::vector<float> >& dist_map,
    std::map<std::string,std::vector<cv::Mat> >& HueHist)
{
  //std::cout<<"-1"<<"\n";
  std::shared_ptr<cv::linemod::Detector> detector (new cv::linemod::Detector);
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

  /***************************************************************************************************************************
   *      AS THIS WILL BE POSTED TO GITHUB SOME DAY: DEAR PERSON-WHO-WANTS-TO-HIRE-ME, READ THIS BEFORE BURNING MY CV        *
   * I'm not responsible for this static functions. It's not me who made it. Naming is horrible. Pointers do kill people.    *
   * I know this. But it's 3 days to the deadline and I don't have enough time to explain to the "developer" who made        *
   * this crap how crappy his crap is. Just don't touch it and it shall work.                                                *
   ***************************************************************************************************************************/
  bool RecognitionData::updateGiorgio(const cv::Mat& const_rgb, const cv::Mat& depth_meter, const cv::Mat& filter_mask,
      cv::Ptr<cv::linemod::Detector>& detector_, std::map<std::string, std::shared_ptr<RendererIterator> >& renderer_iterators_, 
      std::map<std::string,std::vector<cv::Mat> >& Rs_ , std::map<std::string,std::vector<cv::Mat> >& Ts_, 
      std::map<std::string,std::vector<cv::Mat> >& Ks_ , std::map<std::string,std::vector<float> >& distances_,
      cv::Mat& Pose, const std::vector<std::string>& vect_objs_to_pick)
  {

    cv::Mat rgb=const_rgb;
    //The depth_ matrix is given in Meters CV_32F == 5
    //std::cout<<"depth_meter.depth(): "<<depth_meter.depth()<<"\n";
    //std::cout<<"depth_meter.type(): "<<depth_meter.type()<<"\n";


    CV_Assert(filter_mask.depth() == CV_8UC1);
    CV_Assert(depth_meter.depth() == CV_16UC1);
    CV_Assert(depth_meter.depth() == CV_16UC1);
    cv::Mat depth_mm(depth_meter.size(),CV_16UC1);
#if 0
    for (int r=0; r<depth_meter.rows; ++r)
    {
      for (int c=0; c<depth_meter.cols; ++c)
      {
        float px = depth_meter.at<uint16_t>(r,c);
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
#else
    depth_meter.copyTo(depth_mm);
#endif


    if (detector_->classIds().empty())
    {   
      std::cout<<"WARNING: detector_->classIds().empty()"<<"\n";
      return false;
    }


    std::vector<cv::Mat> sources;

    sources.push_back(rgb);
    sources.push_back(depth_mm);

    std::cout<<"Matching..."<<"\n";
    std::vector<cv::linemod::Match> nonconst_matches;
    std::vector<cv::Mat> theMasks;
    theMasks.push_back(filter_mask);
    theMasks.push_back(filter_mask);
    CV_Assert(sources.size()==theMasks.size());
    for(unsigned int i=0; i<sources.size(); ++i){
      CV_Assert(sources[i].cols==theMasks[i].cols && sources[i].rows==theMasks[i].rows);
    }
    detector_->match(sources, _threshold, nonconst_matches,vect_objs_to_pick, cv::noArray(), theMasks);
    /** Just to be sure it's not changed in the Rastafari loop */
    const std::vector<cv::linemod::Match>& matches=nonconst_matches;
    std::cout<<"Done: matches.size(): "<<matches.size()<<"\n";
    if(matches.size() == 0){
      return false;
    }

    cv::Mat_<cv::Vec3f> depth_real_ref_raw;
    cv::Mat_<float> K;
    K_depth_.convertTo(K, CV_32F);
    cv::rgbd::depthTo3d(depth_mm, K, depth_real_ref_raw);

    /** The buffer with detected objects and their info */
    std::vector <object_recognition_core::db::ObjData> objs_;
    int iter = 0;

    std::vector<std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > > vectVectPCPtr;

    vectVectPCPtr.resize(matches.size());
    for(unsigned int ix =0; ix<matches.size(); ++ix)
    {
      vectVectPCPtr[ix].resize(3); //1)Model,2)Ref,3)Aligned PC
    }

    BOOST_FOREACH(const cv::linemod::Match & match, matches) {

      // Fill the Pose object
      cv::Matx33d R_match = Rs_.at(match.class_id)[match.template_id].clone();
      cv::Vec3d T_match = Ts_.at(match.class_id)[match.template_id].clone();
      std::cout << "Rotation matrix: \n" << R_match << "\n";
      std::cout << "Translation vector:\n" << T_match << "\n";
      float D_match = distances_.at(match.class_id)[match.template_id];
      cv::Mat K_match = Ks_.at(match.class_id)[match.template_id];  

      //get the point cloud of the rendered object model
      cv::Mat mask;
      cv::Rect rect;
      cv::Matx33d R_temp(R_match.inv());
      cv::Vec3d up(-R_temp(0,1), -R_temp(1,1), -R_temp(2,1));
      std::cout<<"***match.class_id: ***"<<match.class_id<<"\n";
      //      RendererIterator* it_r = renderer_iterators_.at(match.class_id);
      std::shared_ptr<RendererIterator> it_r = renderer_iterators_.at(match.class_id);
      cv::Mat depth_ref_;
      it_r->renderDepthOnly(depth_ref_, mask, rect, -T_match, up);

      //cv::imshow("depth_ref_",depth_ref_);
      cv::Mat mask_copy; mask.copyTo(mask_copy);
      cvtColor(mask_copy,mask_copy,CV_GRAY2BGR);
      cv::imshow("mask",mask_copy);
      cv::waitKey(1);

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

      //plot Red Rect around the match in the full image
      cv::rectangle(rgb,rect_ref,cv::Scalar(0,0,255),2);

      cv::imshow("rgb",rgb);

      cv::waitKey(1);
      //prepare the model data: from the match
      cv::Mat_<cv::Vec3f> depth_real_model = depth_real_model_raw(rect_model);

      //      cv::rectangle(mask_copy,rect_model,cv::Scalar(0,0,255),2);
      //      cv::imshow("mask",mask_copy);
      //      cv::waitKey();

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
      for(unsigned int ii=0;ii<modelCloudPtr->points.size();++ii)
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
      for(unsigned int ii=0;ii<refCloudPtr->points.size();++ii)
      {
        refCloudPtr->points[ii].x =  pts_real_ref_temp[ii][0];
        refCloudPtr->points[ii].y =  pts_real_ref_temp[ii][1];
        refCloudPtr->points[ii].z =  pts_real_ref_temp[ii][2];

        refCloudPtr->points[ii].r = 255;
        refCloudPtr->points[ii].g = 0;
        refCloudPtr->points[ii].b = 0;

        //         std::cout<<"x "<<refCloudPtr->points[ii].x<<"\n";
        //        std::cout<<"y "<<refCloudPtr->points[ii].y<<"\n";
        //         std::cout<<"z "<<refCloudPtr->points[ii].z<<"\n";
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
      for(unsigned int ii=0;ii<finalModelCloudPtr->points.size();++ii)
      {


        finalModelCloudPtr->points[ii].r = 0;
        finalModelCloudPtr->points[ii].g = 0;
        finalModelCloudPtr->points[ii].b = 255;

      } 

      vectVectPCPtr[iter][2] = finalModelCloudPtr;


      /*** ***/
      /*****END PCL ICP *****/ 



      //keep the object match
      //objs_.push_back(object_recognition_core::db::ObjData(pts_real_ref_temp, pts_real_model_temp, match.class_id, match.similarity, icp_dist, px_ratio_match_inliers, R_real_icp, T_crop));

      ++iter;      
    }//END BOOST_FOREACH


    /** Take the best match and return it as a position */
    const cv::linemod::Match& match=matches[0];
    cv::Mat R_match = Rs_.at(match.class_id)[match.template_id].clone();
    cv::Mat T_match = Ts_.at(match.class_id)[match.template_id].clone();
    Pose=cv::Mat::zeros(4,4, CV_64F);
    R_match.copyTo(Pose(cv::Rect(0,0,3,3)));
    T_match.copyTo(Pose(cv::Rect(3,0,1,3)));
    Pose.at<double>(3,3)=1;
    return true;
  }

  const std::string RecognitionData::DEFAULT_OBJFOLDER_PATH("./objs");

  RecognitionData& RecognitionData::getInstance(){
    static RecognitionData x;
    return x;
  }

  RecognitionData::RecognitionData()
    :
      dfx_ (598.197811409705),
      dfy_ (593.372644257344),
      dcx_ (309.086101744595),
      dcy_ (243.497417130779),
      px_match_min_(0.25f),
      renderer_n_points_ (150),
      renderer_angle_step_ (10),
      renderer_radius_min_ (0.6),
      renderer_radius_max_ (1.1),
      renderer_radius_step_ (0.4),
      renderer_width_ (640),
      renderer_height_ (480),
      renderer_near_ (0.1),
      renderer_far_ (1000.0),
      renderer_focal_length_x_ (525.0),
      renderer_focal_length_y_ (525.0),
      icp_dist_min_ (0.06f),
      th_obj_dist_(0.04f),
      objsfolder_path(current_default_path),
      detector_ (cv::linemod::getDefaultLINEMOD()),
      _threshold(91.0f)
  {

    GLUTInit::init();

    /** Don't know a better way */
    double inputData[3][3]= {{ dfx_, 0, dcx_},{ 0, dfy_, dcy_},{ 0, 0, 1 }};
    K_depth_=cv::Mat(3,3 ,CV_32F, inputData);
    /*
       for(int i=0; i<3; ++i){
       for(int j=0; j<3; ++k){
       K_depth_.at<double>(i,j)=inputData[i][j];
       }
       }*/
    /** GOD FORGIVE ME */
    ::_threshold=_threshold;
    ::K_depth_=K_depth_;
    ::px_match_min_=px_match_min_;
    ::icp_dist_min_ = icp_dist_min_;
    ::th_obj_dist_=th_obj_dist_;
    //entry point simulate RGB and DEPTH images given by Simo
    //Read martrices from yml file
    cv::Mat depth_meters, depth_gray, rgb_img;

    namespace fs=boost::filesystem;
    /*
       cv::FileStorage file(argv[2], cv::FileStorage::READ);
       file["depth_mm"] >> depth_meters;
       file["depth_gray_img"] >> depth_gray;//for visualization purposes
       file["rgb"] >> rgb_img;

       file.release();

    //Debug Shows images
    cv::imshow("rgb",rgb_img);
    cv::imshow("depth_gray",depth_gray);

    cv::waitKey();//Hit enter to continue
    */

    /*** Init Process at Start-Up: only Once !!! ***/

    fs::path objNamesPath=fs::path(objsfolder_path) / fs::path("names.txt");
    if(!fs::exists(objNamesPath) || !fs::is_regular_file(objNamesPath)){
      throw objNamesPath.string() + " does not exist";
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
        throw std::string("Could not read model names.");
      }
    }

    //Load the detector_ with the objects in the DB
    //iterate over the Training Database
    fs::path targetDir(objsfolder_path); 

    fs::directory_iterator it(targetDir), eod;
    BOOST_FOREACH(fs::path const &p, std::make_pair(it, eod))   
    { 
      if(is_directory(p))
      {
        std::cout<<p.filename().string()<<"\n";
        //TODO: For Now only Crayola is Loaded...Remove it in the future
        if(objNames.find(p.filename().string()) == objNames.end()){
          continue;
        }

        //object_id
        std::string object_id_ = p.filename().string();

        std::cout<<"-object_id_: "<<object_id_<<"\n";

        //Load the yml of that class obtained in the Training phase
        fs::path saveLinemodPath = p / fs::path(object_id_+"_Linemod.yml");


        std::string mesh_file_path;
        std::string saveLinemodPathString(saveLinemodPath.string());
        //reading...
        std::shared_ptr<cv::linemod::Detector> detector = readLinemodAndPoses(saveLinemodPathString,mesh_file_path,_Rmap,_Tmap,_Kmap,_distMap,_hueHistMap);
        //DEBUG
        std::cout<<"\tnumTemplates: "<<detector->numTemplates()<<"\n";
        std::cout<<"\tmesh_file_path_debug: "<< mesh_file_path<<"\n";
        //add class template to detector_
        for (size_t template_id = 0; template_id < detector->numTemplates(); ++template_id) {
          const std::vector<cv::linemod::Template> &templates_original = detector->getTemplates(object_id_, template_id);
          detector_->addSyntheticTemplate(templates_original, object_id_);
        }
        //std::cout<<"new *renderer"<<"\n";
        std::shared_ptr<Renderer3d> renderer_ (new Renderer3d(mesh_file_path));
        renderer_->set_parameters(renderer_width_, renderer_height_, renderer_focal_length_x_, renderer_focal_length_y_, renderer_near_, renderer_far_);
        std::cout<<"new *renderer_iterator_"<<"\n";
        //initiaization of the renderer with the same parameters as used for learning
        std::shared_ptr<RendererIterator> renderer_iterator_ (new RendererIterator(renderer_, renderer_n_points_));
        renderer_iterator_->angle_step_ = renderer_angle_step_;
        renderer_iterator_->radius_min_ = float(renderer_radius_min_);
        renderer_iterator_->radius_max_ = float(renderer_radius_max_);
        renderer_iterator_->radius_step_ = float(renderer_radius_step_);
        renderer_iterators_.insert(std::pair<std::string,decltype(renderer_iterator_) >(object_id_, renderer_iterator_));

        //delete renderer_;
        //delete renderer_iterator_;
        std::cout<<"delete"<<"\n";


      }//end if(is_directory(p))
    }//end BOOST_FOREACH

  }

  C5G::Pose RecognitionData::recognize(const Img::ImageWMask& frame, std::string what){

    std::vector<std::string> vect_objs_to_pick(1);
    vect_objs_to_pick[0]=what;
    cv::Mat pose;
    updateGiorgio(frame.rgb, frame.depth, frame.mask, detector_, renderer_iterators_, _Rmap, _Tmap, _Kmap, _distMap, pose, vect_objs_to_pick);
    return matrixToPose(pose);
  }

  C5G::Pose RecognitionData::matrixToPose(cv::Mat m){
    /** Implements the refined algorithm from extracting euler angles from rotation matrix
     * see Mike Day, Insomniac Games, "Extracting Euler Angles from a Rotation Matrix"
     */
    CV_Assert(m.cols==4 && m.rows==4);
    double theta1=atan2(m.at<double>(1,2),m.at<double>(2,2));
    double c2=hypot(m.at<double>(0,0),m.at<double>(0,1));
    double theta2=atan2(-m.at<double>(0,2),c2);
    double s1=sin(theta1);
    double c1=cos(theta1);
    double theta3=atan2(s1*m.at<double>(2,0)-c1*m.at<double>(1,1),c1*m.at<double>(1,1)-s1*m.at<double>(2,1));
    return C5G::Pose(m.at<double>(0,3),m.at<double>(1,3),m.at<double>(2,3), theta1, theta2, theta3);
  }

  void RecognitionData::setModelPath(const std::string& path){
    current_default_path=path;
  }
}
