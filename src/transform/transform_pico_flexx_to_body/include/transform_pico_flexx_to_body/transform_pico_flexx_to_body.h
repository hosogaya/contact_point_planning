#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <nodelet/nodelet.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

namespace transform_pico_flexx_to_body { 
    class TransformPicoFlexxToBody : public nodelet::Nodelet {
        private:
            ros::NodeHandle nh_, pnh_;
            ros::Subscriber sub_pc_;
            ros::Subscriber sub_yaw_;
            ros::Publisher pub_pc_;
            ros::Timer timer_;

            pcl::PointCloud<pcl::PointXYZ> pcl_out_;

            std::string output_frame_;
            std::string input_frame_;
            std::string intermediate_frame_;
            float yaw_ = 0.0f;
            float height_;
            float radius_;

            tf2_ros::TransformBroadcaster tf_br_;
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf_listener_;
            geometry_msgs::TransformStamped transform_pico_flexx_base_to_focus_;
            geometry_msgs::TransformStamped transform_body_pico_flexx_base_;

        public:
            TransformPicoFlexxToBody();
            ~TransformPicoFlexxToBody();

            void onInit();
            void timerCallback(const ros::TimerEvent&);
            void callbackPC(const sensor_msgs::PointCloud2::ConstPtr &pc_msg);
            void callbackYaw(const std_msgs::Float32 &yaw_msg);
    };
}