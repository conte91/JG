#include <cv.hpp>
#include <highgui.h>
#include "leaplistener.h"
#include <algorithm>


void leapListener::onInit( const Leap::Controller &controller)
{
    std::cout << "Initialized" << std::endl;
}

void leapListener::onConnect( const Leap::Controller &controller)
{
    std::cout << "Connected" << std::endl;
    controller.enableGesture(Leap::Gesture::TYPE_CIRCLE);
    controller.enableGesture(Leap::Gesture::TYPE_KEY_TAP);
    controller.enableGesture(Leap::Gesture::TYPE_SCREEN_TAP);
    controller.enableGesture(Leap::Gesture::TYPE_SWIPE);
}

void leapListener::onDisconnect( const Leap::Controller &controller)
{
    std::cout << "Disconnected" << std::endl;
}

void leapListener::onExit( const Leap::Controller &controller)
{
    std::cout << "Exited" << std::endl;
}

void leapListener::onFrame( const Leap::Controller &controller)
{

    const Leap::Frame frame = controller.frame();
    Leap::ImageList images = frame.images();
    std::cout << "# of images: " << images.count() << "\n";
    std::cout << "# of fingers: " << frame.fingers().count()  << "\n";
    std::cout << "Bytes per image: " << images[1].bytesPerPixel() << "\n";

    const Leap::Image& image = images[0];

    if(image.height()>0 && image.width()>0){
      cv::Mat opencvImg = cv::Mat( image.height(), image.width(), CV_8UC1 );
      opencvImg.data = (unsigned char*)image.data();

       cv::imshow("Image show", opencvImg);
       std::cout <<"Image Information"<<image.width()<<std::endl;
    }

    cv::waitKey(30);

}

void leapListener::onFocusGained( const Leap::Controller &controller)
{
    std::cout << "Focus Gained" << std::endl;
}

void leapListener::onFocusLost( const Leap::Controller &controller)
{
    std::cout << "Focus Lost" << std::endl;
}

void leapListener::onDeviceChange( const Leap::Controller &controller)
{
    std::cout << "Device Changed" << std::endl;
    const Leap::DeviceList devices = controller.devices();

    for (int i = 0; i < devices.count(); ++i) {
        std::cout << "id: " << devices[i].toString() << std::endl;
        std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
    }
}

void leapListener::onServiceConnect( const Leap::Controller &controller)
{
    std::cout << "Service Connected" << std::endl;
}

void leapListener::onServiceDisconnect( const Leap::Controller &controller)
{
    std::cout << "Service Disconnected" << std::endl;
}
