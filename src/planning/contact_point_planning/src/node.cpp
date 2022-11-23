#include <contact_point_planning/contact_point_planning.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "contact_point_planning_node");
    contact_point_planning::ContactPointPlanning cp_planning;
    ros::spin();
    return EXIT_SUCCESS;
}