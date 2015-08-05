#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>


namespace C5G {
  struct Pose{
    /* In meters */
    double x;
    double y;
    double z;

    /* In radians */
    double alpha;
    double beta;
    double gamma;

    Pose whichIsRelativeTo(const Pose& origin) const ;
    Pose andIWantItRelativeTo(const Pose& origin) const;

    Eigen::Affine3d toTransform() const;

    static Eigen::Affine3d poseToTransform(const Pose& x);
    static Pose transform2Pose(const Eigen::Affine3d& x);
    /** Basic operations */
    Pose operator+(const Pose& o) const;
    Pose(/* In meters */ double x, double y, double z, /* In radians */ double alpha, double beta, double gamma, /* Override default units */bool useMMDeg=false);
    Pose();
  };
}

std::ostream& operator<< (std::ostream& os, const C5G::Pose& x);
