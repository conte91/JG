#include <highgui/highgui.hpp>
#include "leaplistener.h"

int main(int argc, char *argv[])
{

    leapListener listener;
    Leap::Controller controller;

    controller.addListener( listener );

    Leap::Controller::PolicyFlag addImagePolicy;
    addImagePolicy = ( Leap::Controller::PolicyFlag )
            ( Leap::Controller::PolicyFlag::POLICY_IMAGES |
             controller.policyFlags());

    controller.setPolicyFlags( addImagePolicy );
    std::cout << "Press Enter to quit..." << std::endl;
    std::cin.get();

    controller.removeListener( listener );

    return app.exec();
}
