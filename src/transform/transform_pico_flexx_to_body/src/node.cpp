#include <transform_pico_flexx_to_body/transform_pico_flexx_to_body.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "transform_pico_flexx_to_body_node");
    transform_pico_flexx_to_body::TransformPicoFlexxToBody node;
    ros::spin();

    return 0;
}