#pragma once
namespace C5G{
  class C5G{

    void init();
    void standby();
    void moveCartesian(const Pose& p);
    void setZero();
    void setPosition(const Pose& p);
    void moveCartesianGlobal(const Pose& p);
    void moveAdditive();

  };
}
