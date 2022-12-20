#include <micro_ros_interface/micro_ros_interface.h>

namespace interface
{
MicroRosInterface::MicroRosInterface()
    : nh_(), pnh_("~")
{
    // to micro computer
    yaw_rate_pub_ = nh_.advertise<std_msgs::Float32>("output_yaw_rate", 1);
    next_cp_pub_ = nh_.advertise<geometry_msgs::Point>("output_next_cp", 1);

    // to ros
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("output_imu", 1);
    camera_yaw_pub_ = nh_.advertise<std_msgs::Float32>("output_camera_yaw", 1);

    // from ros
    yaw_rate_sub_ = nh_.subscribe("input_yaw_rate", 10, &MicroRosInterface::yawRateCallback, this);
    next_cp_sub_ = nh_.subscribe("input_next_cp", 10, &MicroRosInterface::nextCpCallback, this);

    // from micro computer
    imu_sub_ = nh_.subscribe("input_imu", 10, &MicroRosInterface::imuCallback, this);
    camera_yaw_sub_ = nh_.subscribe("input_camera_yaw", 10, &MicroRosInterface::cameraYawCallback, this);
}   


MicroRosInterface::~MicroRosInterface() {}

void MicroRosInterface::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    imu_msg_ = *msg;
    imu_msg_.header.stamp = ros::Time::now();
    imu_pub_.publish(imu_msg_);
}

void MicroRosInterface::nextCpCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    next_cp_msg_ = msg->point;
    next_cp_pub_.publish(next_cp_msg_);
}

void MicroRosInterface::yawRateCallback(const std_msgs::Float32::ConstPtr& msg) {
    yaw_rate_pub_.publish(*msg);
}

void MicroRosInterface::cameraYawCallback(const std_msgs::Float32::ConstPtr& msg) {
    camera_yaw_pub_.publish(*msg);   
}

} // namespace interface
