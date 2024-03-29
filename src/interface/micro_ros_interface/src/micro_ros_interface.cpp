#include <micro_ros_interface/micro_ros_interface.h>

namespace interface
{
MicroRosInterface::MicroRosInterface()
    : nh_(), pnh_("~")
{
    // to micro computer
    yaw_rate_pub_ = nh_.advertise<std_msgs::Float32>("output_yaw_rate", 1);
    next_cp_pub_ = nh_.advertise<geometry_msgs::Point>("output_next_cp", 1);
    planned_leg_id_pub_ = nh_.advertise<std_msgs::Int8>("output_planned_leg_id", 1);

    // to ros
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("output_imu", 1);
    camera_yaw_pub_ = nh_.advertise<std_msgs::Float32>("output_camera_yaw", 1);
    planning_leg_id_pub_ = nh_.advertise<std_msgs::Int8>("output_planning_leg_id",1);

    // from ros
    yaw_rate_sub_ = nh_.subscribe("input_yaw_rate", 10, &MicroRosInterface::yawRateCallback, this);
    next_cp_sub_ = nh_.subscribe("input_next_cp", 10, &MicroRosInterface::nextCpCallback, this);

    // from micro computer
    imu_sub_ = nh_.subscribe("input_imu", 10, &MicroRosInterface::imuCallback, this);
    camera_yaw_sub_ = nh_.subscribe("input_camera_yaw", 10, &MicroRosInterface::cameraYawCallback, this);
    planning_leg_id_sub_ = nh_.subscribe("input_planning_leg_id", 10, &MicroRosInterface::planningLegIdCallback, this);
}   


MicroRosInterface::~MicroRosInterface() {}

void MicroRosInterface::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    imu_msg_ = *msg;
    imu_msg_.header.stamp = ros::Time::now();
    imu_pub_.publish(imu_msg_);
}

// void MicroRosInterface::nextCpCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
void MicroRosInterface::nextCpCallback(const fcp_msgs::PlannedContactPoint::ConstPtr& msg) {
    next_cp_msg_ = msg->point;
    planned_leg_id_msg_.data = msg->leg_id;
    next_cp_pub_.publish(next_cp_msg_);
    planned_leg_id_pub_.publish(planned_leg_id_msg_);
}

void MicroRosInterface::yawRateCallback(const std_msgs::Float32::ConstPtr& msg) {
    yaw_rate_pub_.publish(*msg);
}

void MicroRosInterface::cameraYawCallback(const std_msgs::Float32::ConstPtr& msg) {
    camera_yaw_pub_.publish(*msg);   
}

void MicroRosInterface::planningLegIdCallback(const std_msgs::Int8::ConstPtr& msg) {
    planning_leg_id_pub_.publish(*msg);
}

} // namespace interface
