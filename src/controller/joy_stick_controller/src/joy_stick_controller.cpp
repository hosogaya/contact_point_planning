#include <joy_stick_controller/joy_stick_controller.h>

namespace Controller{

JoyStickController::JoyStickController(): nh_(), pnh_("~") {
    
    /** publishers */
    yaw_rate_pub_ = nh_.advertise<std_msgs::Float32>("yaw_rate",1);
    attitude_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("attitude",1);
    behavior_pub_ = nh_.advertise<fcp_msgs::Behavior>("behavior", 1);

    /** subscribers */
    joy_cmd_sub_ = nh_.subscribe("joy", 10, &JoyStickController::joyCallback, this);

    /** register timer callback */
    timer_ = nh_.createTimer(ros::Duration(0.1), &JoyStickController::timerCallback, this);

    /** set button // https://qiita.com/srs/items/9114bb3c27a148e0b855 */

}

JoyStickController::~JoyStickController() {}

void JoyStickController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
    last_joy_cmd_ = *joy_msg.get();
}

void JoyStickController::timerCallback(const ros::TimerEvent& e) {
    /** set data of messages according to last_joy_cmd_ */

    /** publish messages */
    yaw_rate_pub_.publish(yaw_rate_msg_);
    attitude_pub_.publish(attitue_msg_);
    behavior_pub_.publish(behavior_msg_);
}

}