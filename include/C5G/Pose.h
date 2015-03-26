#pragma once
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

    /** Basic operations */
    Pose operator+(const Pose& o) const;
    Pose(/* In meters */ double x, double y, double z, /* In radians */ double alpha, double beta, double gamma, /* Override default units */bool useMMDeg=false);
    Pose();
  };
}

std::ostream& operator<< (std::ostream& os, const C5G::Pose& x);
