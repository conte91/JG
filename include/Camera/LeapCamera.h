#pragma once
#include <cv.hpp>
#include <Leap.h>
#include <Img/Image.h>

namespace Camera{
  class LeapCamera : public Leap::Listener {
    private:
      typedef Img::Image Image;
      Image _lastImage;
      Leap::Controller _controller;
      const std::string _myID;

    public:
      void start();
      void stop();
      const Image& getLastFrame();
      LeapCamera(const std::string& id="LEAP");
      virtual ~LeapCamera();
      virtual void onInit( const Leap::Controller& );
      virtual void onConnect( const Leap::Controller& );
      virtual void onDisconnect( const Leap::Controller& );
      virtual void onExit( const Leap::Controller& );
      virtual void onFrame( const Leap::Controller& );
      virtual void onFocusGained( const Leap::Controller& );
      virtual void onFocusLost( const Leap::Controller& );
      virtual void onDeviceChange( const Leap::Controller& );
      virtual void onServiceConnect( const Leap::Controller& );
      virtual void onServiceDisconnect( const Leap::Controller& );

  };
}
