#pragma once
#include <boost/thread.hpp>
#include <boost/asio/ip/tcp.hpp>
#include "Pose.h"
#include "Grasp.h"
namespace C5G{
  class C5G{

    private:
      std::string _ip;
      std::string _sys_id;
      boost::asio::ip::tcp::iostream _connectionToRobot;
      static const std::string CONNECTION_PORT;

      enum MovementStatus {
        MOVING_GLOBAL,
        MOVING_RELATIVE
      };

      MovementStatus _currentMovementMode;
      Pose _lastGlobalPose;

    public:
      C5G(const std::string& ip, const std::string& sys_id, bool mustInit=true);
      ~C5G();

      static const Pose safePose();
      void init();
      void standby();
      void moveCartesian(const Pose& p);
      void setZero();
      void setPosition(const Pose& p);
      void setGripping(double strength);
      void moveCartesianGlobal(const Pose& p);
      void moveAdditive();

  };
}
