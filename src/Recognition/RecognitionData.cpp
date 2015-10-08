#include <unordered_set>
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

  static double matchingHuePercentage(const cv::Mat& matchingTemplate, const cv::Mat& possibleMatch, const cv::Mat& mask, const cv::Size& matchSize, double cutPercentage, double acceptThreshold, double scaleFactor){
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
    turnBlackWhiteToBlueYellow(hsvMatch, filteredMatch, 5, 5);
    imshow("Blue template", filteredTemplate);
    imshow("Blue match", filteredMatch);
    while((cv::waitKey() & 0xFF)!= 'q');
    Mat displayT, displayM;
    cv::cvtColor(filteredTemplate, displayT, CV_HSV2BGR);
    cv::cvtColor(filteredMatch, displayM, CV_HSV2BGR);

    int totalPoints=0, matchingPoints=0;
    Mat scaledTemplate, scaledMatch, scaledMask;
    cv::resize(filteredTemplate, scaledTemplate, cv::Size(0,0), 1.0/scaleFactor, 1.0/scaleFactor);
    cv::resize(filteredMatch, scaledMatch, cv::Size(0,0), 1.0/scaleFactor, 1.0/scaleFactor);
    cv::resize(erodedMask, scaledMask, cv::Size(0,0), 1.0/scaleFactor, 1.0/scaleFactor);
    Mat matchingDraw(scaledTemplate.size(), CV_8UC1);
    matchingDraw.setTo(130);
    for(int i=0; i<scaledTemplate.rows; ++i){
      for(int j=0; j<scaledTemplate.cols; ++j){
        if(scaledMask.at<unsigned char>(i,j)){
          totalPoints++;
          if(fabs(scaledTemplate.at<cv::Vec3b>(i,j)[0]-scaledMatch.at<cv::Vec3b>(i,j)[0])<acceptThreshold){
            matchingPoints++;
            matchingDraw.at<unsigned char>(i,j)=255;
          }
          else{
            matchingDraw.at<unsigned char>(i,j)=0;
          }
        }
      }
    }

    return (double) matchingPoints/(double) totalPoints;
  }

  bool RecognitionData::updateGiorgio(const cv::Mat& const_rgb, const cv::Mat& depth_m, const cv::Mat& filter_mask, 
      ObjectMatches& result, const std::vector<std::string>& vect_objs_to_pick) const
  {

    using cv::Rect;
    using cv::Mat;

    /** TODO remove me when multiple objects are considered */
    assert(vect_objs_to_pick.size()==1);
    Mat rgb=const_rgb.clone();
    assert(filter_mask.depth() == CV_8UC1 && "Filtering mask should be CV_8UC1" );
    /** The depth_ matrix is given in m, but we want to use it in mm now */
    assert(depth_m.type() == CV_32FC1 && "Depth should be CV_32FC1 (in meters)");
    Mat depth_mm;
    depth_m.convertTo(depth_mm, CV_16UC1, 1000.0);

    std::vector<Mat> sources;
    sources.push_back(rgb);
    sources.push_back(depth_mm);

    std::vector<Mat> theMasks;
    theMasks.push_back(filter_mask);
    theMasks.push_back(filter_mask);

    /** Check consistent input have been provided */
    assert(sources.size()==theMasks.size() && "Masks and sources of different sizes!");
    for(unsigned int i=0; i<sources.size(); ++i){
      assert(sources[i].cols==theMasks[i].cols && sources[i].rows==theMasks[i].rows && "Masks and sources images of different sizes");
    }

    /** Create LINE-MOD detector with templates built from the object */
    cv::Ptr<Model::Detector> detector (new Model::Detector(*cv::linemod::getDefaultLINEMOD()));
    for(auto& object_id_ : vect_objs_to_pick){
      _objectModels.at(object_id_).addAllTemplates(*detector);
    }
    std::cout << "#Templates: " << detector->numTemplates() << "\n";

    /** First of all, we match with decreasing thresholds until at least 3 not-so-badly-matching templates have been found */
    struct FirstPassFoundItems{
      cv::linemod::Match match;
      Eigen::Affine3d objPose;
      Mat rgb;
      Mat depth;
      Mat mask;
      Rect rect;
      double matchPercentage;
    };
    /** Here we will save all matching parts */
    std::vector<FirstPassFoundItems> found;


    /** To avoid re-evaluating the same match in different steps */
    std::unordered_set<cv::linemod::Match> foundMatches;

    double currentThreshold=_threshold;
    while(found.size()==0){
    std::cout<<"Matching with threshold " << currentThreshold << "..."<<"\n";
    std::vector<cv::linemod::Match> nonconst_matches;
    detector->match(sources, currentThreshold, nonconst_matches,vect_objs_to_pick, cv::noArray(), theMasks);

    /** Just to be sure it's not changed in the Rastafari loop */
    const std::vector<cv::linemod::Match>& matches=nonconst_matches;
    std::cout<<"Done: matches.size(): "<<matches.size()<<"\n";
    if(matches.size() == 0){
      std::cout << "LineMOD matching returned no valid matches.\n";
      return false;
    }

    for(const auto& match : matches) {

      using Eigen::Affine3d;

      int tId=match.template_id;
      std::cout << "Template # " << tId << " matches.\n";
      if(foundMatches.find(match)!=foundMatches.end()){
        std::cout << "(skipped)\n";
        continue;
      }
      foundMatches.insert(match);
      const auto& obj=_objectModels.at(match.class_id);
      /** Renders the match to check for hue correctness (drops some false positives) */
      cv::Mat sticazzi, stimazzi, stimaski;
      cv::Rect stiretti;
      obj.renderMatch(match, stimazzi, sticazzi, stimaski, stiretti);
      cv::Mat newFrame=const_rgb.clone();
      Mat matchingPart=newFrame(stiretti);
      assert(matchingPart.size()==stimaski.size() && matchingPart.size()==stimaski.size());
      double percentage=matchingHuePercentage(stimazzi, matchingPart, stimaski, stiretti.size(), 0.1,30,1);

      if(percentage < 0.6) {
        std::cout << "Object rejected (percentage=" << percentage << "). Trying another template...\n";
        continue;
      }
      std::cout << "Object taken (percentage=" << percentage << ").\n";
      /** Check for false positives by valuating hues values on the downsampled image */
      found.push_back({match, obj.matchToObjectPose(match), stimazzi, sticazzi, stimaski, stiretti, percentage});

    }
      if(currentThreshold<70){
        break;
      }
      currentThreshold*=0.9;
    }
    if(found.size()==0){
      std::cout << "LineMOD matching filtering returned no valid matches.\n";
      return false;
    }

    for(auto& x : found){
        Mat stimazzi, sticazzi, stimaski;
        Rect stiretti;
        std::cout << "Object rendering pose:\n" << x.objPose.matrix() << "\n\n\n";
        const auto& obj=_objectModels.at(x.match.class_id);
        obj.render(x.objPose, stimazzi, sticazzi, stimaski, stiretti);
        if(stimazzi.empty()){
          std::cout << "Image is empty!!!\n";
          continue;
        }
        auto newFrame=const_rgb.clone();
        stimazzi.copyTo(newFrame(stiretti));
        imshow("Ciao!", newFrame);
        while((cv::waitKey() & 0xFF)!= 'q');
    }

    std::vector<Match> refound;

    /** Now we will construct, for each match, the corresponding point cloud. We then apply ICP in order to refine the pose estimation and drop other false positives */
    for(const auto& x:found){
      using cv::Rect;
      using cv::Mat;
      typedef pcl::PointXYZRGB PointType;
      typedef pcl::PointCloud<PointType> Cloud;
      using pcl::PointXYZ;
      using pcl::PointNormal;
      using pcl::FPFHSignature33;
      typedef pcl::PointCloud<FPFHSignature33> FeatureCloud;
      typedef pcl::PointCloud<PointNormal> CloudXYZ;

      /** Take the corresponding parts of the match into the main image */
      Rect imgRect=x.rect;
      imgRect.x-=x.rect.width*0.05;
      imgRect.y-=x.rect.height*0.05;
      imgRect.width*=1.1;
      imgRect.height*=1.1;
      if(imgRect.x<0){
        imgRect.width+=imgRect.x;
        imgRect.x=0;
      }
      if(imgRect.y<0){
        imgRect.height+=imgRect.y;
        imgRect.y=0;
      }
      int diff=imgRect.width+imgRect.x-const_rgb.cols;
      if(diff>0){
        imgRect.width-=diff;
      }
      diff=imgRect.height+imgRect.y-const_rgb.rows;
      if(diff>0){
        imgRect.height-=diff;
      }

      Mat matchingPart=const_rgb(imgRect);
      Mat matchingDepth=depth_m(imgRect);
      assert(depth_m.type()==CV_32FC1);
      Mat x_d_m;
      x.depth.convertTo(x_d_m, CV_32FC1, 1.0/1000.0);

      /** Obtain the PCs relative to the parts to be aligned */
      const auto& obj=_objectModels.at(x.match.class_id);
      auto templatePC=obj.getCam().sceneToCameraPointCloud(x.rgb, x_d_m, x.mask);
      templatePC->is_dense=true;
      auto scenePC=obj.getCam().sceneToCameraPointCloud(matchingPart, matchingDepth);
      scenePC->is_dense=true;

      /* Create the filtering object: downsample the dataset using a leaf size of 1cm */
      pcl::VoxelGrid<PointNormal> templateVox, sceneVox;
      CloudXYZ::Ptr templatePCDownsampled(new CloudXYZ), scenePCDownsampled(new CloudXYZ), templatePCXYZ(new CloudXYZ), scenePCXYZ(new CloudXYZ);
      pcl::copyPointCloud(*templatePC, *templatePCXYZ);
      pcl::copyPointCloud(*scenePC, *scenePCXYZ);

      templateVox.setInputCloud (templatePCXYZ);
      templateVox.setLeafSize (0.005f, 0.005f, 0.005f);
      templateVox.filter (*templatePCDownsampled);
      sceneVox.setInputCloud (scenePCXYZ);
      sceneVox.setLeafSize (0.005f, 0.005f, 0.005f);
      sceneVox.filter (*scenePCDownsampled);
      /* Estimate normals for clouds */
      pcl::console::print_highlight ("Estimating scene normals...\n");
      pcl::NormalEstimationOMP<PointNormal, PointNormal> nest;
      nest.setRadiusSearch (0.02);
      nest.setInputCloud (templatePCDownsampled);
      nest.compute (*templatePCDownsampled);
      nest.setInputCloud (scenePCDownsampled);
      nest.compute (*scenePCDownsampled);
      FeatureCloud::Ptr template_features(new FeatureCloud), scene_features(new FeatureCloud);
      pcl::FPFHEstimationOMP<PointNormal, PointNormal, pcl::FPFHSignature33> fest;
      fest.setRadiusSearch (0.05);
      fest.setInputCloud (templatePCDownsampled);
      fest.setInputNormals (templatePCDownsampled);
      fest.compute (*template_features);
      fest.setInputCloud (scenePCDownsampled);
      fest.setInputNormals (scenePCDownsampled);
      fest.compute (*scene_features);

      /* Perform alignment */
      //pcl::SampleConsensusPrerejective<PointNormal, PointNormal, FPFHSignature33> align;
      pcl::SampleConsensusInitialAlignment<PointNormal, PointNormal, FPFHSignature33> align;
      align.setInputSource (templatePCDownsampled  );
      align.setSourceFeatures (template_features);
      align.setInputTarget (scenePCDownsampled);
      align.setTargetFeatures (scene_features);
      align.setMaximumIterations (1000); // Number of RANSAC iterations
      align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
      align.setCorrespondenceRandomness (5); // Number of nearest features to use
      //align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
      //align.setMaxCorrespondenceDistance (2.5f * 0.005f); // Inlier threshold
      //align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
      CloudXYZ::Ptr alignModelCloudPtr (new CloudXYZ);
      align.align (*alignModelCloudPtr);

      if(!align.hasConverged()){
        std::cout << "Align did not converge\n";
        continue;
      }

      CloudXYZ::Ptr finalModelCloudPtr(new CloudXYZ);
      Eigen::Affine3d finalTransformationMatrix;
      finalTransformationMatrix.matrix()  = align.getFinalTransformation().cast<double>();
      std::cout << "Transform (align):" << finalTransformationMatrix.matrix() << "\n";
      auto middlePose = finalTransformationMatrix;
      std::cout << "ObjPose: " << x.objPose.matrix() << "\n";
      std::cout << "Middle pose: " << middlePose.matrix() << "\n";
      /** Use ICP to refine the pose estimation of the object */
      pcl::IterativeClosestPoint<PointNormal, PointNormal> icp;
      icp.setMaximumIterations (100);
      icp.setInputSource (alignModelCloudPtr);//Model
      icp.setInputTarget (scenePCDownsampled);//Ref scene
      icp.align (*finalModelCloudPtr);
      if(!icp.hasConverged()){
        continue;
      }
      std::cout << "Score: " << icp.getFitnessScore() << "\n";

      if(icp.getFitnessScore()>0.001){
        continue;
      }
      finalTransformationMatrix.matrix()  = icp.getFinalTransformation().cast<double>();
      std::cout << "Transform (icp):" << finalTransformationMatrix.matrix() << "\n";
      auto finalPose = finalTransformationMatrix*middlePose;
      std::cout << "ObjPose: " << x.objPose.matrix() << "\n";
      std::cout << "Final pose: " << x.objPose*finalPose.matrix() << "\n";
      refound.push_back({icp.getFitnessScore(), finalPose});
      Eigen::Vector3d origin{0,0,0};
      auto noOrigin=finalPose*origin;
      std::cout << "Final position guess: " << noOrigin << "\n";
      
      /*

      pcl::visualization::PCLVisualizer viewer("Scene's PCL");
      viewer.addCoordinateSystem(0.1);
      //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> templatecolors (templatePC);
      pcl::visualization::PointCloudColorHandlerCustom<PointNormal> templatecolors (templatePCDownsampled, 255, 0, 0);
      viewer.addPointCloud<PointNormal>(templatePCDownsampled, templatecolors, "template");
      //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> finalModelColors (finalModelCloudPtr);
      pcl::visualization::PointCloudColorHandlerCustom<PointNormal> finalModelColors (finalModelCloudPtr, 0, 255, 0);
      viewer.addPointCloud<PointNormal>(finalModelCloudPtr, finalModelColors, "aligned");
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> scenecolors (scenePC);
      viewer.addPointCloud<pcl::PointXYZRGB>(scenePC, scenecolors, "scene");
      while(!viewer.wasStopped()){
        viewer.spinOnce(100);
      }
      viewer.close();
      */
      /** Visualization of the result */
      Mat stimazzi, sticazzi, stimaski;
      Rect stiretti;
      std::cout << "Final rendering pose:\n" << finalPose.matrix() << "\n\n\n";
      obj.render(finalPose, stimazzi, sticazzi, stimaski, stiretti);
      if(stimazzi.empty()){
        std::cout << "Image is empty!!!\n";
        continue;
      }
      auto newFrame=const_rgb.clone();
      stimazzi.copyTo(newFrame(stiretti));
      imshow("Wow!", newFrame);
      while((cv::waitKey() & 0xFF)!= 'q');
    }

    if(refound.size()==0){
      std::cout << "Result alignment returned no valid matches.\n";
      return false;
    }

    std::sort(refound.begin(), refound.end(),
        [](const Match& a, const Match& b) -> bool{
        return (a.matchScore < b.matchScore);
        });

    result[vect_objs_to_pick[0]]=refound;
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
        throw std::string("Could not read model names.");
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

  RecognitionData::ObjectMatches RecognitionData::recognize(const Img::ImageWMask& frame, std::string what){

    std::vector<std::string> vect_objs_to_pick(1);
    vect_objs_to_pick[0]=what;
    cv::Mat pose;

    ObjectMatches result;
    if(!updateGiorgio(frame.rgb, frame.depth, frame.mask, result, vect_objs_to_pick)){
      throw std::string("Could not match anything :(");
    }
    return result;
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
