#include <joy_stick_controller/joy_stick_controller.h>

namespace Controller{

JoyStickController::JoyStickController(): nh_(), pnh_("~") {
    
    /** parameters */
    // pnh_.getParam<double>("control_period", control_period_, 0.02f);

    /** publishers */
    yaw_rate_pub_ = nh_.advertise<std_msgs::Float32>("output_yaw_rate",1);
    attitude_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("output_attitude",1);
    behavior_pub_ = nh_.advertise<fcp_msgs::Behavior>("output_behavior", 1);
    // contact_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("output_cp", 1);
    contact_point_pub_ = nh_.advertise<fcp_msgs::PlannedContactPoint>("output_cp", 1);
    // joystick_pub_ = nh_.advertise<sensor_msgs::Joy>("output_joy", 1);

    /** subscribers */
    joy_cmd_sub_ = nh_.subscribe("joy", 10, &JoyStickController::joyCallback, this);
    contact_point_sub_ = nh_.subscribe("input_cp", 10, &JoyStickController::cpCallback, this);


    /** register timer callback */
    timer_ = nh_.createTimer(ros::Duration(0.1), &JoyStickController::timerCallback, this);

    /** set button // https://qiita.com/srs/items/9114bb3c27a148e0b855 */
    // pnh_.getParam<size_t>("triangle_button", triangle_, 12);
    // pnh_.getParam<size_t>("cricle_button", circle_, 13);
    // pnh_.getParam<size_t>("cross_button", cross_, 14);
    // pnh_.getParam<size_t>("rectangle_button", rectangle_, 15);

}

JoyStickController::~JoyStickController() {}

void JoyStickController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
    last_joy_cmd_ = *joy_msg;
    get_joy_ = true;
    // joystick_pub_.publish(last_joy_cmd_);
}


// void JoyStickController::cpCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
void JoyStickController::cpCallback(const fcp_msgs::PlannedContactPoint::ConstPtr& msg) {
    cp_msg_ = *msg;
    get_cp_ = true;
}


void JoyStickController::timerCallback(const ros::TimerEvent& e) {
    
    if (!get_joy_) return;
    /** set data of messages according to last_joy_cmd_ */
    if (last_joy_cmd_.buttons[circle_]) publishCP();
    
    /** yaw rate */
    float yaw_x = last_joy_cmd_.axes[LV_axis_];
    float yaw_y = last_joy_cmd_.axes[LH_axis_];
    if (std::sqrt(yaw_x*yaw_x + yaw_y*yaw_y) < 0.5f) {
        yaw_rate_msg_.data = 0.0f;
    }
    else yaw_rate_msg_.data = std::atan2(yaw_y, yaw_x);

    // attitude_msg_.data.resize(2); // roll pitch
    // attitude_msg_.data[0] = 0.0f; // roll
    // attitude_msg_.data[1] = 0.0f; // pitch


    behavior_msg_.type = 0; // behavior type
    behavior_msg_.header.stamp = ros::Time::now();
    

    /** publish messages */
    yaw_rate_pub_.publish(yaw_rate_msg_);
    attitude_pub_.publish(attitude_msg_);
    behavior_pub_.publish(behavior_msg_);

    get_joy_ = false;
}

void JoyStickController::publishCP() {
    if (get_cp_) 
        contact_point_pub_.publish(cp_msg_);
}

}