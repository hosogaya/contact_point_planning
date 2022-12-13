#include <joy_stick_controller/joy_stick_controller.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_stick_controller");
    Controller::JoyStickController joy_stick_controller;

    ros::spin();
    
    return EXIT_SUCCESS;
}