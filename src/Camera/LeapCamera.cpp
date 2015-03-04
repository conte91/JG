#include <iostream>

#include <Camera/LeapCamera.h>

namespace Camera{

  LeapCamera::LeapCamera(const std::string& id)
    :
      _myID(id),
      _lastImage(cv::Mat(0, 0, CV_8UC1), cv::Mat(0, 0, CV_8UC3))

  {
  }

  void LeapCamera::start(){
    std::cout << _myID << ": starting listening for camera events...\n";
    _controller.addListener((*this));
  }

  void LeapCamera::stop(){
    _controller.removeListener((*this));
    std::cout << _myID << ": starting listening for camera events...\n";
  }

  const Image& LeapCamera::getLastFrame(){
    return _lastImage;
  }

  LeapCamera::~LeapCamera(){
    stop();
  }

  void LeapCamera::onInit( const Leap::Controller &controller)
  {
    std::cout << "Initialized" << std::endl;
  }

  void LeapCamera::onConnect( const Leap::Controller &controller)
  {
    std::cout << "Connected" << std::endl;
  }

  void LeapCamera::onDisconnect( const Leap::Controller &controller)
  {
    std::cout << "Disconnected" << std::endl;
  }

  void LeapCamera::onExit( const Leap::Controller &controller)
  {
    std::cout << "Exited" << std::endl;
  }

  void LeapCamera::onFrame( const Leap::Controller &controller)
  {

    const Leap::Frame frame = controller.frame();
    Leap::ImageList images = frame.images();

    Leap::Image image = images[0];
    cv::Mat opencvImg = cv::Mat( image.height(), image.width(), CV_8UC1 );
    opencvImg.data = (unsigned char*)image.data();

    cv::Mat rgb_black = cv::Mat( image.height(), image.width(), CV_8UC3 );
    opencvImg.data = (unsigned char*)image.data();
    _lastImage=Image(opencvImg, rgb_black);
  }

  void LeapCamera::onFocusGained( const Leap::Controller &controller)
  {
    std::cout << "Focus Gained" << std::endl;
  }

  void LeapCamera::onFocusLost( const Leap::Controller &controller)
  {
    std::cout << "Focus Lost" << std::endl;
  }

  void LeapCamera::onDeviceChange( const Leap::Controller &controller)
  {
    std::cout << "Device Changed" << std::endl;
    const Leap::DeviceList devices = controller.devices();

    for (int i = 0; i < devices.count(); ++i) {
      std::cout << "id: " << devices[i].toString() << std::endl;
      std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
    }
  }

  void LeapCamera::onServiceConnect( const Leap::Controller &controller)
  {
    std::cout << "Service Connected" << std::endl;
  }

  void LeapCamera::onServiceDisconnect( const Leap::Controller &controller)
  {
    std::cout << "Service Disconnected" << std::endl;
  }

}
