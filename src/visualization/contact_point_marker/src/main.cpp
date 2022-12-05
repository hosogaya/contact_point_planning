#include <contact_point_marker/contact_point_marker.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "contact_point_marker_node");
    visualization::ContactPointMarker contact_point_marker;
    ros::spin();
    return EXIT_SUCCESS;
}