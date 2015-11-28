#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/core/operations.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Geometry>
#include <array>
#include <functional>
#include <Eigen/Core>

namespace Gripper{
  struct GraspPose{

    /** Thirds axis == Z is always true for now, but you'll never know */
    std::array<bool,3> constraints;

    /** Containes the vertical axis over which each axis of the gripper is constrained */
    std::array<Eigen::Vector3d, 3> axis;
    
    Eigen::Vector3d pickPose;

    double preferenceScore;

    int toolNumber;

    GraspPose(const decltype(constraints)& , const decltype(axis)&, const decltype(pickPose)&, decltype(preferenceScore), decltype(toolNumber));

    GraspPose();

    GraspPose constrain(const std::function<double(Eigen::Affine3d&)>& f);

    bool operator<(const GraspPose& other);

    void drawToViewer(pcl::visualization::PCLVisualizer& viewer) const;

  };
}

namespace cv{
  void write( FileStorage& fs, const std::string& name, const Gripper::GraspPose& pose);
  void read(const FileNode& node, Gripper::GraspPose& x, const Gripper::GraspPose& default_value = Gripper::GraspPose{} );
}
