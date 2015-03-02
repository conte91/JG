#include "leaplistener.h"


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

    Leap::Image image = images[0];
    cv::Mat opencvImg = cv::Mat( image.height(), image.width(), CV_8UC1 );
    opencvImg.data = (unsigned char*)image.data();

    cv::Mat resetImg = cv::Mat( image.height(), image.width(), CV_8UC1 );
    for( int i = 0; i < opencvImg.rows; i++){
        for( int j = 0; j < opencvImg.cols; j++){
            if( opencvImg.data[i*opencvImg.cols + j] < 100 ){
                resetImg.data[i*opencvImg.cols + j] = 0;
            }
            else{
                resetImg.data[i*opencvImg.cols + j] = 255;
            }
        }
    }


    cv::imshow("depth", opencvImg);
    cv::imshow("filter", resetImg);
    cv::waitKey(30);

    std::cout <<"Image Information"<<image.width()<<std::endl;
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
