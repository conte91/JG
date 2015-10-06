#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

using pcl::PointNormal;
typedef pcl::PointNormal Point;
typedef pcl::PointCloud<Point> Cloud;

Cloud::Ptr getCube(double width, double height, double depth, double cx, double cy, double cz, int nPoints){
  Cloud::Ptr result(new Cloud);

  double stepx=width/(nPoints-1);
  double stepy=height/(nPoints-1);
  double stepz=depth/(nPoints-1);

  double startx=cx-(width/2);
  double starty=cy-(height/2);
  double startz=cz-(depth/2);

  /** Front and back */
  for(int i=0; i<nPoints; ++i){
    for(int j=0; j<nPoints; ++j){
      pcl::PointNormal pt1, pt2;
      pt1.x=startx+i*stepx;
      pt1.y=starty+j*stepy;
      pt1.z=startz;
      pt2=pt1;
      pt2.z=startz+depth;
      result->push_back(pt1);
      result->push_back(pt2);
    }
  }
  /** Top and bottom */
  for(int i=0; i<nPoints; ++i){
    for(int j=0; j<nPoints; ++j){
      pcl::PointNormal pt1, pt2;
      pt1.x=startx+i*stepx;
      pt1.y=starty;
      pt1.z=startz+j*stepz;
      pt2=pt1;
      pt2.y=starty+height;
      result->push_back(pt1);
      result->push_back(pt2);
    }
  }
  /** Left and right */
  for(int i=0; i<nPoints; ++i){
    for(int j=0; j<nPoints; ++j){
      pcl::PointNormal pt1, pt2;
      pt1.x=startx;
      pt1.y=starty+i*stepy;
      pt1.z=startz+j*stepz;
      pt2=pt1;
      pt2.x=startx+width;
      result->push_back(pt1);
      result->push_back(pt2);
    }
  }

  return result;

}


/** Same code used for template matching registration, just to test how getFinalTransform() works */
int main(){
  typedef Cloud CloudXYZ;
  using pcl::FPFHSignature33;
  typedef pcl::PointCloud<FPFHSignature33> FeatureCloud;
  /** Obtain the PCs relative to the parts to be aligned */
  auto templatePC=getCube(1, 0.5, 0.2, 0, 0, 100, 1000);
  templatePC->is_dense=true;
  auto scenePC=getCube(1, 0.5, 0.2, 0, 0, -20, 1000);
  scenePC->is_dense=true;
  Eigen::Affine3d initialPose=Eigen::Affine3d::Identity();
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
  pcl::SampleConsensusPrerejective<PointNormal, PointNormal, FPFHSignature33> align;
  align.setInputSource (templatePCDownsampled  );
  align.setSourceFeatures (template_features);
  align.setInputTarget (scenePCDownsampled);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (20000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * 0.005f); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  CloudXYZ::Ptr alignModelCloudPtr (new CloudXYZ);
  align.align (*alignModelCloudPtr);

  if(!align.hasConverged()){
    std::cerr << "Not aligned\n";
    return -1;
  }
  pcl::visualization::PCLVisualizer viewerAligned("Scene's PCL");
  viewerAligned.addCoordinateSystem(0.1);
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> templatecolors (templatePC);
  pcl::visualization::PointCloudColorHandlerCustom<PointNormal> templatecolors (templatePCDownsampled, 255, 0, 0);
  viewerAligned.addPointCloud<PointNormal>(templatePCDownsampled, templatecolors, "template");
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> finalModelColors (finalModelCloudPtr);
  pcl::visualization::PointCloudColorHandlerCustom<PointNormal> alignModelColors (alignModelCloudPtr, 0, 255, 0);
  viewerAligned.addPointCloud<PointNormal>(alignModelCloudPtr, alignModelColors, "aligned");
  pcl::visualization::PointCloudColorHandlerCustom<PointNormal> scenecolors (scenePC, 255, 255, 255);
  viewerAligned.addPointCloud<PointNormal>(scenePC, scenecolors, "scene");
  while(!viewerAligned.wasStopped()){
    viewerAligned.spinOnce(100);
  }
  viewerAligned.close();

  CloudXYZ::Ptr finalModelCloudPtr(new CloudXYZ);
  Eigen::Affine3d finalTransformationMatrix;
  std::cout << "Final transform (align): " << align.getFinalTransformation().matrix() << "\n";
  finalTransformationMatrix.matrix()  = align.getFinalTransformation().cast<double>();
  std::cout << "Transform (align):" << finalTransformationMatrix.matrix() << "\n";
  //finalTransformationMatrix=finalTransformationMatrix.inverse();
  std::cout << "Inverse (align):" << finalTransformationMatrix.matrix() << "\n";
  auto middlePose = finalTransformationMatrix*initialPose;
  std::cout << "Middle pose: " << middlePose.matrix() << "\n";
  /** Use ICP to refine the pose estimation of the object */
  pcl::IterativeClosestPoint<PointNormal, PointNormal> icp;
  icp.setMaximumIterations (100);
  icp.setInputSource (alignModelCloudPtr);//Model
  icp.setInputTarget (scenePCDownsampled);//Ref scene
  icp.align (*finalModelCloudPtr);
  if(!icp.hasConverged()){
    std::cerr << "Not icped\n";
    return -1;
  }
  std::cout << "Score: " << icp.getFitnessScore() << "\n";

  if(icp.getFitnessScore()>0.001){
    std::cerr << "Not icped\n";
    return -1;
  }
  std::cout << "Final transform (icp): " << align.getFinalTransformation().matrix() << "\n";
  finalTransformationMatrix.matrix()  = icp.getFinalTransformation().cast<double>();
  std::cout << "Transform (icp):" << finalTransformationMatrix.matrix() << "\n";
  //finalTransformationMatrix=finalTransformationMatrix.inverse();
  std::cout << "Inverse (align):" << finalTransformationMatrix.matrix() << "\n";
  auto finalPose = finalTransformationMatrix*middlePose;
  std::cout << "Final pose: " << finalPose.matrix() << "\n";

  Eigen::Vector3d origin{0,0,100};
  std::cout << "Initial pose: " << origin << "\n";
  auto noOrigin=finalPose*origin;
  std::cout << "Final pose guess: " << noOrigin << "\n";



  pcl::visualization::PCLVisualizer viewer("Scene's PCL");
  viewer.addCoordinateSystem(0.1);
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> templatecolors (templatePC);
  pcl::visualization::PointCloudColorHandlerCustom<PointNormal> templatecolors2 (templatePCDownsampled, 255, 0, 0);
  viewer.addPointCloud<PointNormal>(templatePCDownsampled, templatecolors2, "template");
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> finalModelColors (finalModelCloudPtr);
  pcl::visualization::PointCloudColorHandlerCustom<PointNormal> finalModelColors2 (finalModelCloudPtr, 0, 255, 0);
  viewer.addPointCloud<PointNormal>(finalModelCloudPtr, finalModelColors2, "aligned");
  pcl::visualization::PointCloudColorHandlerCustom<PointNormal> scenecolors2 (scenePC, 255, 255, 255);
  viewer.addPointCloud<PointNormal>(scenePC, scenecolors2, "scene");
  while(!viewer.wasStopped()){
    viewer.spinOnce(100);
  }
  viewer.close();
}
