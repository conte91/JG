#include <highgui.h>
#include <Leap.h>
#include "leaplistener.h"

int main(int argc, char *argv[])
{

    cv::namedWindow("Image show");

    leapListener listener;
    Leap::Controller controller;

    controller.addListener( listener );

    Leap::Controller::PolicyFlag addImagePolicy;
    addImagePolicy = (Leap::Controller::PolicyFlag) ((int) Leap::Controller::POLICY_IMAGES | (int) Leap::Controller::POLICY_BACKGROUND_FRAMES | (int) controller.policyFlags());

    controller.setPolicyFlags( addImagePolicy );
    std::cout << "Press Enter to quit..." << std::endl;
    std::cin.get();

    controller.removeListener( listener );

    return 0;
}
