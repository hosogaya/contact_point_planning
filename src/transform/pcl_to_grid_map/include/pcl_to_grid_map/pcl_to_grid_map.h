#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_pcl/helpers.hpp>
#include <grid_map_filters/MedianFillFilter.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/transforms.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <string>

namespace transform {
namespace pcl_to_grid_map {

class PCLToGridMap : public nodelet::Nodelet
{
private:
    ros::NodeHandle nh_,pnh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher grid_map_raw_pub_;
    ros::Publisher gird_map_filtered_pub_;
    ros::Publisher point_cloud_pub_;

    grid_map::GridMapPclLoader grid_map_pcl_loader_;
    grid_map::MedianFillFilter median_fill_filter_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string output_frame_; // param
    geometry_msgs::TransformStamped transform_;
    Eigen::Matrix4f transform_mat_;
    sensor_msgs::PointCloud2 transformed_pc2_;

public:
    PCLToGridMap(/* args */);
    ~PCLToGridMap();

    void onInit() override;
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
};

}
}