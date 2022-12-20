#include <micro_ros_interface/micro_ros_interface.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_stick_controller");
    interface::MicroRosInterface micro_ros_interface;
    ros::spin();
    return EXIT_SUCCESS;   
}