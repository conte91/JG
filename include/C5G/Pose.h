#pragma once

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
    Pose operator+(const Pose& o);
  };
}
