#include <Recognition/RecognitionData.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
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

#include <Recognition/GiorgioUtils.h>
#include <Recognition/GLUTInit.h>

namespace Recognition{


  /***************************************************************************************************************************
   *      AS THIS WILL BE POSTED TO GITHUB SOME DAY: DEAR PERSON-WHO-WANTS-TO-HIRE-ME, READ THIS BEFORE BURNING MY CV        *
   * I'm not responsible for this static functions. It's not me who made it. Naming is horrible. Pointers do kill people.    *
   * I know this. But it's 3 days to the deadline and I don't have enough time to explain to the "developer" who made        *
   * this crap how crappy his crap is. Just don't touch it and it shall work.                                                *
   ***************************************************************************************************************************/
  bool RecognitionData::updateGiorgio(const cv::Mat& const_rgb, const cv::Mat& depth_m, const cv::Mat& filter_mask, 
      cv::Mat& pose, const std::vector<std::string>& vect_objs_to_pick) const
  {

    cv::Mat rgb=const_rgb.clone();
    CV_Assert(filter_mask.depth() == CV_8UC1);
    /** The depth_ matrix is given in m, but we want to use it in mm now */
    CV_Assert(depth_m.type() == CV_32FC1);
    cv::Mat depth_mm;
    depth_m.convertTo(depth_mm, CV_16UC1, 1000);

    std::vector<cv::Mat> sources;
    sources.push_back(rgb);
    sources.push_back(depth_mm);

    std::vector<cv::linemod::Match> nonconst_matches;
    std::vector<cv::Mat> theMasks;
    theMasks.push_back(filter_mask);
    theMasks.push_back(filter_mask);

    /** Check consistent input have been provided */
    CV_Assert(sources.size()==theMasks.size());
    for(unsigned int i=0; i<sources.size(); ++i){
      CV_Assert(sources[i].cols==theMasks[i].cols && sources[i].rows==theMasks[i].rows);
    }

    /** Create LINE-MOD detector with templates built from the object */
    cv::Ptr<cv::linemod::Detector> detector (cv::linemod::getDefaultLINEMOD());
    for(auto& object_id_ : vect_objs_to_pick){
      _objectModels.at(object_id_).addAllTemplates(*detector);
    }
    std::cout << "#Templates: " << detector->numTemplates() << "\n";

    std::cout<<"Matching..."<<"\n";
    detector->match(sources, _threshold, nonconst_matches,vect_objs_to_pick, cv::noArray(), theMasks);

    /** Just to be sure it's not changed in the Rastafari loop */
    const std::vector<cv::linemod::Match>& matches=nonconst_matches;
    std::cout<<"Done: matches.size(): "<<matches.size()<<"\n";
    if(matches.size() == 0){
      return false;
    }

    cv::Mat depth_real_ref_raw;
    cv::rgbd::depthTo3d(depth_mm, _cameraModel.getIntrinsic(), depth_real_ref_raw);
    assert(depth_real_ref_raw.type()==CV_32FC3);

    /** The buffer with detected objects and their info */
    std::vector <object_recognition_core::db::ObjData> objs_;

    /** Keep the point cloud of the best match */
    std::array< pcl::PointCloud<pcl::PointXYZRGB>::Ptr , 3 > resultPointClouds;

    for(const auto& match : matches) {

      using Eigen::Affine3f;

      // Fill the pose object
      int tId=match.template_id;
      std::cout << "Template # " << tId << " matches.\n";
      auto& obj=_objectModels.at(match.class_id);
      cv::Matx33f R_match = obj.getR(tId);
      cv::Vec3f T_match = obj.getT(tId);
      std::cout << "Rotation matrix: \n" << R_match << "\n";
      std::cout << "Translation vector:\n" << T_match << "\n";
      float D_match = obj.getDist(tId);
      const auto& K_match = obj.getK(tId);

      //get the point cloud of the rendered object model
      cv::Mat mask;
      cv::Rect rect;
      cv::Matx33f R_temp(R_match.inv());
      cv::Vec3d up(-R_temp(0,1), -R_temp(1,1), -R_temp(2,1));
      auto it_r = _objectModels.at(match.class_id).getRenderer();
      cv::Mat depth_ref_;
      it_r->renderDepthOnly(depth_ref_, mask, rect, -T_match, up);

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
      double px_ratio_missing = matToVec(depth_real_model, depth_real_ref, pts_real_model_temp, pts_real_ref_temp);
      std::cout<<"px_ratio_missing > px_match_min_ ?: "<<px_ratio_missing<<" > " <<  px_match_min_ <<"\n";
      if (px_ratio_missing > px_match_min_)
        continue;

      Eigen::Matrix4f finalTransformationMatrix;
      //TODO if(!pclICP(pts_real_model_temp, pts_real_ref_temp, finalTransformationMatrix, resultPointClouds)){
      //TODO   continue;
      //TODO }

      /** Take the best match and return it as a position */

      /** Fill the transformation matrix */
      Eigen::Matrix3f eRot;
      Eigen::Vector3f eTras;

      cv2eigen(R_match, eRot);
      cv2eigen(T_match, eTras);

      Eigen::Affine3f matchTrans=Eigen::Affine3f::Identity();
      
      matchTrans*=eRot;
      std::cout << "MatrixR: \n" <<
        matchTrans.matrix() << "\n";

      Eigen::Affine3f miasorella=Eigen::Affine3f::Identity();
      miasorella*=Eigen::Translation3f(eTras);
      std::cout << "MatrixT: \n" <<
        miasorella.matrix() << "\n";

      matchTrans=miasorella*matchTrans;
      std::cout << "MatrixM: \n" <<
        matchTrans.matrix() << "\n";

      matchTrans.translate(Eigen::Vector3f(0.5,0,0.2));
      const auto& Cam_match = obj.getCam(tId);
      Eigen::Affine3f cameraToWorld=Cam_match.getExtrinsic().inverse();

      /** Move to global positions */
      std::cout << "Matrix0: \n" <<
        matchTrans.matrix() << "\n";
      Eigen::Affine3d mattiaEUnTrans(matchTrans);
      matchTrans = cameraToWorld * matchTrans;

      std::cout << "Matrix: \n" <<
        matchTrans.matrix() << "\n";
      cv::Mat sticazzi, stimazzi, stimaski;
      cv::Mat newFrame=const_rgb.clone();
      cv::Rect stiretti;
      obj.render(mattiaEUnTrans, stimazzi, sticazzi, stimaski, stiretti);
      stimazzi.copyTo(newFrame.rowRange(stiretti.y, stiretti.y+stiretti.height).colRange(stiretti.x,stiretti.x+stiretti.width));

      eigen2cv(matchTrans.matrix(), pose);

      return true;
    }
    return false;
  }

  bool RecognitionData::pclICP(const std::vector<cv::Vec3f>& pointsFromModel, const std::vector<cv::Vec3f>& pointsFromReference, Eigen::Matrix4f& finalTransformationMatrix, std::array< PCloud::Ptr , 3 >& resultPointClouds) const {

    /** Fills model and reference pointClouds with points taken from (X,Y,Z) coordinates */
    PCloud::Ptr modelCloudPtr (new pcl::PointCloud<pcl::PointXYZRGB>);
    PCloud::Ptr refCloudPtr (new pcl::PointCloud<pcl::PointXYZRGB>);

    /** Model PointCloud*/
    modelCloudPtr->points.resize(pointsFromModel.size());
    modelCloudPtr->width =  modelCloudPtr->points.size();
    modelCloudPtr->height = 1;
    modelCloudPtr->is_dense = true;

    for(unsigned int ii=0;ii<modelCloudPtr->points.size();++ii)
    {
      modelCloudPtr->points[ii].x = pointsFromModel[ii][0];
      modelCloudPtr->points[ii].y = pointsFromModel[ii][1];
      modelCloudPtr->points[ii].z = pointsFromModel[ii][2];

      modelCloudPtr->points[ii].r = 0;
      modelCloudPtr->points[ii].g = 255;
      modelCloudPtr->points[ii].b = 0;

    }

    /*Ref PointCloud*/
    refCloudPtr->points.resize(pointsFromReference.size());
    refCloudPtr->width =  refCloudPtr->points.size();
    refCloudPtr->height = 1;
    refCloudPtr->is_dense = true;
    for(unsigned int ii=0;ii<refCloudPtr->points.size();++ii)
    {
      refCloudPtr->points[ii].x = pointsFromReference[ii][0];
      refCloudPtr->points[ii].y = pointsFromReference[ii][1];
      refCloudPtr->points[ii].z = pointsFromReference[ii][2];

      refCloudPtr->points[ii].r = 255;
      refCloudPtr->points[ii].g = 0;
      refCloudPtr->points[ii].b = 0;
    }


    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaximumIterations (20);
    icp.setInputSource (modelCloudPtr);//Model
    icp.setInputTarget (refCloudPtr);//Ref scene
    PCloud::Ptr finalModelCloudPtr (new PCloud);

    icp.align (*finalModelCloudPtr);
    finalTransformationMatrix = icp.getFinalTransformation().cast<float>();

    if(!icp.hasConverged()){
      return false;
    }

    resultPointClouds[0] = modelCloudPtr;
    resultPointClouds[1] = refCloudPtr;

    //Color the aligned PC
    for(unsigned int ii=0;ii<finalModelCloudPtr->points.size();++ii)
    {


      finalModelCloudPtr->points[ii].r = 0;
      finalModelCloudPtr->points[ii].g = 0;
      finalModelCloudPtr->points[ii].b = 255;

    } 

    resultPointClouds[2] = finalModelCloudPtr;

    return true;
  }


  RecognitionData::RecognitionData(const std::string& trainPath, const CameraModel& m)
    :
      _cameraModel(m),
      px_match_min_(0.25f),
      th_obj_dist_(0.04f),
      objsfolder_path(trainPath),
      _threshold(91.0f)
  {

    cv::Mat depth_meters, depth_gray, rgb_img;

    namespace fs=boost::filesystem;

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

    fs::path targetDir(objsfolder_path); 

    fs::directory_iterator it(targetDir), eod;
    BOOST_FOREACH(fs::path const &p, std::make_pair(it, eod))   
    { 
      if(is_directory(p))
      {
        std::cout<<p.filename().string()<<"\n";
        if(objNames.find(p.filename().string()) == objNames.end()) {
          continue;
        }

        /** Loads a model from the trained ones */
        std::string object_id_ = p.filename().string();
        std::cout<<"Loading object: " << object_id_<<"\n";
        _objectModels.emplace(std::make_pair(object_id_, Model(object_id_, objsfolder_path)));

      }

    }
  }

  C5G::Pose RecognitionData::recognize(const Img::ImageWMask& frame, std::string what){

    std::vector<std::string> vect_objs_to_pick(1);
    vect_objs_to_pick[0]=what;
    cv::Mat pose;
    if(!updateGiorgio(frame.rgb, frame.depth, frame.mask, pose, vect_objs_to_pick)){
      throw std::string("Could not match anything :(");
    }
    return matrixToPose(pose);
  }

  RecognitionData::PCloud::Ptr RecognitionData::objectPointCloud(const std::string& objectID, const C5G::Pose& pose) const {
    const Model& m = _objectModels.at(objectID);
    return m.getPointCloud(pose);
  }

  C5G::Pose RecognitionData::matrixToPose(cv::Mat m){
    /** Implements the refined algorithm from extracting euler angles from rotation matrix
     * see Mike Day, Insomniac Games, "Extracting Euler Angles from a Rotation Matrix"
     */
    assert(m.cols==4 && m.rows==4 && m.type()==CV_32FC1);
    float theta1=atan2(m.at<float>(1,2),m.at<float>(2,2));
    float c2=hypot(m.at<float>(0,0),m.at<float>(0,1));
    float theta2=atan2(-m.at<float>(0,2),c2);
    float s1=sin(theta1);
    float c1=cos(theta1);
    float theta3=atan2(s1*m.at<float>(2,0)-c1*m.at<float>(1,1),c1*m.at<float>(1,1)-s1*m.at<float>(2,1));
    return C5G::Pose(m.at<float>(0,3),m.at<float>(1,3),m.at<float>(2,3), theta1, theta2, theta3);
  }

  const Model& RecognitionData::getModel(const std::string& name) const {
    return _objectModels.at(name);
  }
}
