#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

#include <nodelet/nodelet.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

namespace transform_pico_flexx_to_body { 
    class TransformPicoFlexxToBody : public nodelet::Nodelet {
        private:
            ros::NodeHandle nh_, pnh_;
            ros::Subscriber camera_yaw_sub_;
            ros::Subscriber imu_sub_;

            std::string body_frame_;
            std::string pico_flexx_frame_;
            std::string pico_flexx_motor_frame_;
            std::string base_link_;
            float yaw_ = 0.0f;
            float height_;
            float radius_;

            ros::Timer timer_;

            tf2_ros::TransformBroadcaster tf_br_;
            tf2_ros::StaticTransformBroadcaster static_tf_br_;
            geometry_msgs::TransformStamped transform_pico_flexx_base_to_focus_;
            geometry_msgs::TransformStamped transform_body_to_pico_flexx_base_;
            geometry_msgs::TransformStamped transform_base_link_to_body_;

            sensor_msgs::Imu imu_msgs_;

        public:
            TransformPicoFlexxToBody();
            ~TransformPicoFlexxToBody();

            void onInit();
            void timerCallback(const ros::TimerEvent&);
            void cameraYawCallback(const std_msgs::Float32::ConstPtr &msg);
            void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    };
}