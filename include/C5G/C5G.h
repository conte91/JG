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
  }

  class C5G{

    void Init();
    void Standby();
    void MoveCartesian(const Pose& p);
    void SetZero();
    void SetPosition(const Pose& p);
    void MoveCartesianGlobal(const Pose& p);
    void MoveAdditive();

  }
}
