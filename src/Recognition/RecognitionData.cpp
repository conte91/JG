#include <unordered_set>
#include <stdexcept>
#include <cmath>
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ia_ransac.h>

#include <Recognition/GLUTInit.h>
#include <Recognition/Utils.h>
#include <Recognition/ColorGradientPyramidFull.h>

namespace Recognition{

  static void turnBlackWhiteToBlueYellow(const cv::Mat& hsv_in, cv::Mat& hsv_out, double ts, double tv){
    using cv::Mat;
    Mat whiteMask, blackMask;
    Mat allY(hsv_in.size(), CV_8UC3);
    allY.setTo(cv::Scalar{30,255,255});
    Mat allB(hsv_in.size(), CV_8UC3);
    allB.setTo(cv::Scalar{120,255,255});
    /** Turn black points to blue points and white points to yellow points*/
    cv::inRange(hsv_in, cv::Scalar{0,0,0}, cv::Scalar{255, 255, tv}, blackMask);
    cv::inRange(hsv_in, cv::Scalar{0,0,tv}, cv::Scalar{255, ts, 255}, whiteMask);

    hsv_out=hsv_in.clone();
    allY.copyTo(hsv_out, whiteMask);
    allB.copyTo(hsv_out, blackMask);
  }

  static double matchingHuePercentage(const cv::Mat& matchingTemplate, const cv::Mat& possibleMatch, const cv::Mat& mask, const cv::Size& matchSize, double cutPercentage, double acceptThreshold, double scaleFactor, size_t& matchingArea){
    using cv::Mat;
    using cv::Rect;
    using cv::Size;

    /** Blank the background */
    Mat blankedTemplate(matchSize, CV_8UC3), blankedMatch(matchSize, CV_8UC3);
    constexpr int erosion_size = 1; 
    cv::Mat erosion_element = cv::getStructuringElement(cv::MORPH_CROSS,
                                                        cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                        cv::Point(erosion_size, erosion_size) );
    cv::Mat erodedMask;
    // Apply erosion or dilation on the image
    cv::erode(mask,erodedMask,erosion_element);
    blankedTemplate.setTo(cv::Scalar{0,0,0});
    blankedMatch.setTo(cv::Scalar{0,0,0});

    matchingTemplate.copyTo(blankedTemplate, erodedMask);
    possibleMatch.copyTo(blankedMatch, erodedMask);
    /** Remove a small border from the match, in order to compensate wrong-by-little positioning */
    cv::Rect section;
    section.width=matchSize.width;
    section.height=matchSize.height;
    section.y=0;
    section.x=0;
    Mat templatePart=blankedTemplate(section);
    Mat matchPart=blankedMatch(section);
    Mat maskPart=mask(section);
    Mat hsvTemplate, hsvMatch, filteredTemplate, filteredMatch;
    cv::cvtColor(templatePart, hsvTemplate, CV_BGR2HSV);
    cv::cvtColor(matchPart, hsvMatch, CV_BGR2HSV);
    //hsvTemplate.setTo(cv::Scalar{0,0,0}, ~maskPart);
    //hsvMatch.setTo(cv::Scalar{0,0,0}, ~maskPart);
    turnBlackWhiteToBlueYellow(hsvTemplate, filteredTemplate, 5, 5);
    turnBlackWhiteToBlueYellow(hsvMatch, filteredMatch, 20, 20);

    size_t totalPoints=0;
    matchingArea=0;
    Mat scaledTemplate, scaledMatch, scaledMask, scaledHsvTemplate, scaledHsvMatch;
    cv::resize(hsvTemplate, scaledHsvTemplate, cv::Size(0,0), 1.0/scaleFactor, 1.0/scaleFactor);
    cv::resize(hsvMatch, scaledHsvMatch, cv::Size(0,0), 1.0/scaleFactor, 1.0/scaleFactor);
    cv::resize(filteredTemplate, scaledTemplate, cv::Size(0,0), 1.0/scaleFactor, 1.0/scaleFactor);
    cv::resize(filteredMatch, scaledMatch, cv::Size(0,0), 1.0/scaleFactor, 1.0/scaleFactor);
    cv::resize(erodedMask, scaledMask, cv::Size(0,0), 1.0/scaleFactor, 1.0/scaleFactor);
    Mat displayT, displayM;
    cv::cvtColor(scaledTemplate, displayT, CV_HSV2BGR);
    cv::cvtColor(scaledMatch, displayM, CV_HSV2BGR);
    Mat matchingDraw(scaledTemplate.size(), CV_8UC1);
    matchingDraw.setTo(130);
    for(int i=0; i<scaledTemplate.rows; ++i){
      for(int j=0; j<scaledTemplate.cols; ++j){
        if(scaledMask.at<unsigned char>(i,j)){
          totalPoints++;
          cv::Vec3b tVec=scaledTemplate.at<cv::Vec3b>(i,j), mVec=scaledMatch.at<cv::Vec3b>(i,j);
          cv::Vec3b white{30,255,255};
          cv::Vec3b black{120,255,255};

          cv::Vec3b tVecOrig=scaledHsvTemplate.at<cv::Vec3b>(i,j), mVecOrig=scaledHsvMatch.at<cv::Vec3b>(i,j);
          if((fabs(tVecOrig[0]-mVecOrig[0])<acceptThreshold) || (tVec==white && mVec==white) || (tVec==black && mVec==black)){
            matchingArea++;
            matchingDraw.at<unsigned char>(i,j)=255;
          }
          else{
            matchingDraw.at<unsigned char>(i,j)=0;
          }
        }
      }
    }

    double percentage=(double) matchingArea/(double) totalPoints;
    return (double) matchingArea/(double) totalPoints;
  }

  std::vector<RecognitionData::FirstPassFoundItems> RecognitionData::makeAFirstPassRecognition(const cv::Mat& const_rgb, const cv::Mat& depth_m, const cv::Mat& filter_mask, const std::vector<std::string>& whatToSee) const {

    using cv::Mat;
    using cv::Rect;

    /** Some checks because you'll never know */
    assert(filter_mask.depth() == CV_8UC1 && "Filtering mask should be CV_8UC1" );
    assert(depth_m.type() == CV_32FC1 && "Depth should be CV_32FC1 (in meters)");
    assert(const_rgb.size()==depth_m.size() && "Inconsistent RGB and depth image provided");
    assert(const_rgb.size()==filter_mask.size() && "Inconsisten RGB/depth and mask provided");

    /** The depth_ matrix is given in m, but we want to use it in mm now */
    Mat depth_mm;
    depth_m.convertTo(depth_mm, CV_16UC1, 1000.0);

    /** Build inputs for Line-MOD */
    Mat rgb=const_rgb.clone();
    std::vector<Mat> sources;
    sources.push_back(rgb);
    sources.push_back(depth_mm);

    std::vector<Mat> theMasks;
    theMasks.push_back(filter_mask);
    theMasks.push_back(filter_mask);


    /** Here we will save all matching parts */
    std::vector<FirstPassFoundItems> found;
    /** To avoid re-evaluating the same match in different steps */
    std::unordered_set<cv::linemod::Match> foundMatches;
    /** Here we flag each object into the list to see what we encountered so far */

    /** Create LINE-MOD detector with templates built from the object */

    double currentThreshold=_threshold;
    while(1){
      /** Fill an object list with every object which has not been found (or is not valid) so far */
      std::map<std::string, bool> iHaveFound;
      for(const auto& x : whatToSee){
        iHaveFound[x]=false;
      }
      for(const auto& x : found){
        iHaveFound.at(x.match.class_id)=true;
      }
      std::vector<std::string> whatsMissing;
      for(const auto& x:whatToSee){
        if(!iHaveFound.at(x)){
          whatsMissing.push_back(x);
        }
      }
      if(whatsMissing.empty()){
        /** Everything was fine :) */
        break;
      }

      /** Create detector with every object which has to be found */
      /*** TODO TODO TODO TODO TODO Change this */
      std::unique_ptr<Model::Detector> detector (new Model::Detector(*cv::linemod::getFullObjectLINEMOD()));
      for(auto& object_id_ : whatToSee){
        _objectModels.at(object_id_).addAllTemplates(*detector);
      }
      std::cout << "#Templates: " << detector->numTemplates() << "\n";

      /** Search for every object now */
      size_t nDone=0;
      std::cout<<"Matching with threshold " << currentThreshold << "..."<<"\n";
      std::vector<cv::linemod::Match> matches;
      detector->match(sources, currentThreshold, matches,whatToSee, cv::noArray(), theMasks);
      std::cout<<"Done: matches.size(): "<<matches.size()<<"\n";

      /** Now, filter until something good is found for every object */
      for(const auto& match : matches) {

        using Eigen::Affine3d;

        /** Skip already evaluated matches */
        int tId=match.template_id;
        if(foundMatches.find(match)!=foundMatches.end()){
          continue;
        }
        foundMatches.insert(match);

        const auto& obj=_objectModels.at(match.class_id);

        /** Renders the match to check for hue correctness (drops some false positives) */
        Mat rgb, d, m;
        Rect section;
        auto mPose=obj.matchToObjectPose(match);
        obj.render(mPose, rgb, d, m, section);
        assert(!rgb.empty());
        const Mat matchingPart=const_rgb(section);
        assert(matchingPart.size()==rgb.size() && "Non coherent rgb and mask output from render");
        size_t matchingArea;
        double percentage=matchingHuePercentage(rgb, matchingPart, m, section.size(), 0.1,30,1, matchingArea);
        if(percentage < 0.6) {
          continue;
        }
        found.push_back({match, mPose, rgb, d, m, section, percentage});
      }
      currentThreshold *= 0.9;
      if(currentThreshold < 0.8){
        break;
      }
    }
    return found;
  }

  bool RecognitionData::updateGiorgio(const cv::Mat& const_rgb, const cv::Mat& depth_m, const cv::Mat& filter_mask, 
                                      ObjectMatches& result, const std::vector<std::string>& vect_objs_to_pick) const
  {

    using cv::Rect;
    using cv::Mat;

    /** TODO remove me when multiple objects are considered */
    assert(vect_objs_to_pick.size()==1);

    /** First of all, we match with decreasing thresholds until at least 1 not-so-badly-matching templates has been found for each object */
    auto found=makeAFirstPassRecognition(const_rgb, depth_m, filter_mask, vect_objs_to_pick);

    /** Now we will construct, for each match, the corresponding point cloud. We then apply ICP in order to refine the pose estimation and drop other false positives */


    result=ObjectMatches{};
    /** Scan each item and align it */
    for(const auto& x:found){
      /** Obtain the match's original image */
      const auto& obj=_objectModels.at(x.match.class_id);
      /** First, build the PC of the whole scene on which the template must be aligned */
      typedef pcl::PointXYZRGB PointType;
      typedef pcl::PointCloud<PointType> Cloud;
      Cloud::Ptr scenePC(new Cloud);
      {
        auto scenePCNotRegular=obj.getCam().sceneToCameraPointCloud(const_rgb, depth_m, filter_mask);
        scenePCNotRegular->is_dense=true;
        std::vector<int> dumbIgnoredValue;
        pcl::removeNaNFromPointCloud(*scenePCNotRegular, *scenePCNotRegular, dumbIgnoredValue);
        pcl::VoxelGrid<PointType> sceneVox;
        sceneVox.setInputCloud (scenePCNotRegular);
        sceneVox.setLeafSize (0.005f, 0.005f, 0.005f);
        sceneVox.filter (*scenePC);
      }
      auto matchImage=imageFromRender(x.rgb, x.depth, x.mask, x.rect, obj.getCam());
      /** Obtain the PCs relative to the parts to be aligned */
      Cloud::Ptr templatePC(new Cloud);
      {
        auto templatePCNotRegular=obj.getCam().sceneToCameraPointCloud(matchImage.rgb, matchImage.depth, matchImage.mask);
        templatePCNotRegular->is_dense=true;
        templatePCNotRegular->is_dense=true;
        std::vector<int> dumbIgnoredValue;
        pcl::removeNaNFromPointCloud(*templatePCNotRegular, *templatePCNotRegular, dumbIgnoredValue);
        /* Create the filtering object: downsample the dataset using a leaf size of 1cm */
        pcl::VoxelGrid<PointType> templateVox;
        templateVox.setInputCloud (templatePCNotRegular);
        templateVox.setLeafSize (0.005f, 0.005f, 0.005f);
        templateVox.filter (*templatePC);
      }
      /** Use ICP to refine the pose estimation of the object */
      pcl::IterativeClosestPoint<PointType, PointType> icp;
      icp.setMaximumIterations (100);
      icp.setInputSource (templatePC);//Model
      icp.setInputTarget (scenePC);//Ref scene
      Cloud::Ptr finalModelCloudPtr(new Cloud);
      icp.align (*finalModelCloudPtr);
      if(!icp.hasConverged()){
        continue;
      }
      std::cout << "Score: " << icp.getFitnessScore() << "\n";
      if(icp.getFitnessScore()>0.001){
        continue;
      }
      Eigen::Matrix4d finalTransformationMatrix  = icp.getFinalTransformation().cast<double>();
      auto finalPose = finalTransformationMatrix*x.objPose.matrix();
      result[x.match.class_id].push_back({icp.getFitnessScore(), Eigen::Affine3d{finalPose}});
    }

    for(auto& x : vect_objs_to_pick){
      std::sort(result[x].begin(), result[x].end(),
                [](const Match& a, const Match& b) -> bool{
                  return (a.matchScore < b.matchScore);
                });
    }
    return true;
  }

  RecognitionData::RecognitionData(const std::string& trainPath, const CameraModel& m)
    :
    _cameraModel(m),
    objsfolder_path(trainPath),
    px_match_min_(0.05),
    th_obj_dist_(0.04f), //"th_obj_dist", "Threshold on minimal distance between detected objects.", 0.04f);
    _threshold(91.0f)
  {

    cv::Mat depth_meters, depth_gray, rgb_img;

    namespace fs=boost::filesystem;

    /*** Init Process at Start-Up: only Once !!! ***/
    fs::path objNamesPath=fs::path(objsfolder_path) / fs::path("names.txt");
    if(!fs::exists(objNamesPath) || !fs::is_regular_file(objNamesPath)){
      throw std::runtime_error(objNamesPath.string() + " does not exist");
    }
    std::set<std::string> objNames;
    std::cout << "Reading model names from " << objNamesPath << "..\n";
    {
      std::ifstream names(objNamesPath.string());
      while(1){
        std::string s;
        names >> s;
        if(!names.good()){
          break;
        }
        if(s[0]=='#'){
          continue;
        }
        std::cout << "Will elaborate directory: " << s << "\n";
        objNames.insert(s);
      }
      if(!names.eof()){
        throw std::runtime_error("Could not read model names.");
      }
    }

    fs::path targetDir(objsfolder_path); 

    for(auto& name : objNames){

      fs::path p=objsfolder_path / fs::path(name);
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
      else{
        std::cout << "\n\n\n####################\n\n\nInvalid object was put for recognition!!!\n\n\n####################\n\n\n";
      }
    }
  }

  RecognitionData::ObjectMatches RecognitionData::recognize(const Img::ImageWMask& frame, const std::vector<std::string>& what){

    ObjectMatches result;
    if(!updateGiorgio(frame.rgb, frame.depth, frame.mask, result, what)){
      throw std::runtime_error("Could not match anything :(");
    }
    return result;
  }

  RecognitionData::PCloud::ConstPtr RecognitionData::objectPointCloud(const std::string& objectID, const Eigen::Affine3d& pose) const {
    const Model& m = _objectModels.at(objectID);
    return m.getPointCloud(pose);
  }

  RecognitionData::PCloud::ConstPtr RecognitionData::objectPointCloud(const std::string& objectID) const {
    const Model& m = _objectModels.at(objectID);
    return m.getPointCloud();
  }


  const Model& RecognitionData::getModel(const std::string& name) const {
    return _objectModels.at(name);
  }
}
