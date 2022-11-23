#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <grid_map_pcl/GridMapPclLoader.hpp>
#include <grid_map_pcl/helpers.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

namespace transform {
namespace pcl_to_grid_map {

class PCLToGridMap : public nodelet::Nodelet
{
private:
    ros::NodeHandle nh_,pnh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher grid_map_pub_;

    grid_map::GridMapPclLoader grid_map_pcl_loader_;

public:
    PCLToGridMap(/* args */);
    ~PCLToGridMap();

    void onInit() override;
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
};

}
}