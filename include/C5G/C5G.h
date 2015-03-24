#pragma once
#include <boost/thread.hpp>
#include "Pose.h"
#include "Grasp.h"
namespace C5G{
  class C5G{

    public:
      static const Pose safePose;
      void init();
      void standby();
      void moveCartesian(const Pose& p);
      void setZero();
      void setPosition(const Pose& p);
      void setGripping(double strength);
      void moveCartesianGlobal(const Pose& p);
      void moveAdditive();
      void executeGrasp(const Grasp&);

  };
}
