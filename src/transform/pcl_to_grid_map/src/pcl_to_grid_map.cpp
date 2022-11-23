#include <pcl_to_grid_map/pcl_to_grid_map.h>

namespace gm = ::grid_map::grid_map_pcl;

namespace transform {
namespace pcl_to_grid_map {

PCLToGridMap::PCLToGridMap(/* args */):nh_(), pnh_("~"), grid_map_pcl_loader_() {}

void PCLToGridMap::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();    
    gm::setVerbosityLevelToDebugIfFlagSet(pnh_);

    grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("output", 1);
    point_cloud_sub_ = nh_.subscribe("input", 10, &PCLToGridMap::pointCloudCallback, this);

    grid_map_pcl_loader_.loadParameters(gm::getParameterPath(pnh_));
}

PCLToGridMap::~PCLToGridMap(){}

void PCLToGridMap::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){

    ROS_INFO("pointCloudCallback is called");    
    grid_map::grid_map_pcl::Pointcloud::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    grid_map::grid_map_pcl::Pointcloud::Ptr nanRemovedCloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*msg, *inputCloud);
    inputCloud->is_dense =false;

    boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*inputCloud, *nanRemovedCloud,*indices);
    nanRemovedCloud->is_dense =true;

    grid_map_pcl_loader_.setInputCloud(nanRemovedCloud);
    // gm::processPointcloud(&grid_map_pcl_loader_, pnh_);
    const auto start = std::chrono::high_resolution_clock::now();
    grid_map_pcl_loader_.initializeGridMapGeometryFromInputCloud();
    gm::printTimeElapsedToRosInfoStream(start, "Initialization took: ");
    grid_map_pcl_loader_.addLayerFromInputCloud(grid_map::grid_map_pcl::getMapLayerName(pnh_));
    gm::printTimeElapsedToRosInfoStream(start, "Total time: ");

    // ROS_ERROR("height%d", inputCloud->height);
    // ROS_ERROR("width%d", inputCloud->width);

    grid_map::GridMap gridMap = grid_map_pcl_loader_.getGridMap();
    gridMap.setFrameId(gm::getMapFrame(pnh_));
    // gridMap.setTimestamp(ros::Time::now());
    // publish grid map
    grid_map_msgs::GridMap grid_map_msg;
    grid_map::GridMapRosConverter::toMessage(gridMap, grid_map_msg);
    grid_map_msg.info.header.stamp=ros::Time::now();
    grid_map_pub_.publish(grid_map_msg);

}

}
}
PLUGINLIB_EXPORT_CLASS(transform::pcl_to_grid_map::PCLToGridMap, nodelet::Nodelet);