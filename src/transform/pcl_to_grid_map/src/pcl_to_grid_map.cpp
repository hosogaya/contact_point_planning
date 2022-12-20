#include <pcl_to_grid_map/pcl_to_grid_map.h>

namespace gm = ::grid_map::grid_map_pcl;

namespace transform {
namespace pcl_to_grid_map {

PCLToGridMap::PCLToGridMap(/* args */) :
    nh_(), pnh_("~"), grid_map_pcl_loader_(), tf_buffer_(), tf_listener_(tf_buffer_){}

void PCLToGridMap::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();    
    pnh_.param<std::string>("output_frame", output_frame_, "base_link");
    gm::setVerbosityLevelToDebugIfFlagSet(pnh_);

    grid_map_raw_pub_ = nh_.advertise<grid_map_msgs::GridMap>("output_raw", 1);
    gird_map_filtered_pub_ = nh_.advertise<grid_map_msgs::GridMap>("output_filtered", 1);
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output_pc2", 1);
    point_cloud_sub_ = nh_.subscribe("input", 10, &PCLToGridMap::pointCloudCallback, this);
    


    grid_map_pcl_loader_.loadParameters(gm::getParameterPath(pnh_));

    // Set up the parameters
    XmlRpc::XmlRpcValue config;
    config["name"] = "median";
    config["type"] = "gridMapFilters/MedianFillFilter";

    XmlRpc::XmlRpcValue params;
    pnh_.getParam("input_layer", params["input_layer"]);
    pnh_.getParam("output_layer", params["output_layer"]);
    params["fill_hole_radius"] = 0.02;
    params["filter_existing_values"] = true;
    params["existing_value_radius"] = 0.02;
    params["fill_mask_layer"] = "fill_mask";
    params["debug"] = false;
    params["num_erode_dilation_iterations"] = 4;

    config["params"] = params;
    median_fill_filter_.filters::FilterBase<grid_map::GridMap>::configure(config);
    median_fill_filter_.configure();
}

PCLToGridMap::~PCLToGridMap(){}

void PCLToGridMap::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){

    try {
        transform_ = tf_buffer_.lookupTransform(output_frame_, msg->header.frame_id, ros::Time(0));
        transform_mat_ = tf2::transformToEigen(transform_.transform).matrix().cast<float>();
        pcl_ros::transformPointCloud(transform_mat_, *msg, transformed_pc2_);
        transformed_pc2_.header.frame_id = output_frame_;

        point_cloud_pub_.publish(transformed_pc2_);
    }
    catch (tf2::TransformException & ex) {
        ROS_WARN("%s", ex.what());
        return;
    }

    ROS_INFO("pointCloudCallback is called");    
    grid_map::grid_map_pcl::Pointcloud::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    grid_map::grid_map_pcl::Pointcloud::Ptr nanRemovedCloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(transformed_pc2_, *inputCloud);
    inputCloud->is_dense =false;

    boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*inputCloud, *nanRemovedCloud,*indices);
    nanRemovedCloud->is_dense =true;

    /**
     *  Process the point cloud 
     *  Remove outliers, down sampling
    */
    grid_map_pcl_loader_.setInputCloud(nanRemovedCloud);
    gm::processPointcloud(&grid_map_pcl_loader_, pnh_);

    /**Create grid map from point cloud*/
    grid_map::GridMap gridMap_raw = grid_map_pcl_loader_.getGridMap();
    gridMap_raw.setFrameId(gm::getMapFrame(pnh_));

    grid_map::GridMap gridMap_filtered;
    median_fill_filter_.update(gridMap_raw, gridMap_filtered);

    /** convert grid map to message*/
    grid_map_msgs::GridMap grid_map_raw_msg;
    grid_map::GridMapRosConverter::toMessage(gridMap_raw, grid_map_raw_msg);
    grid_map_raw_msg.info.header.stamp=ros::Time::now();
    grid_map_raw_pub_.publish(grid_map_raw_msg);

    grid_map_msgs::GridMap grid_map_filtered_msg;
    grid_map::GridMapRosConverter::toMessage(gridMap_filtered, grid_map_filtered_msg);
    grid_map_filtered_msg.info.header.stamp=ros::Time::now();
    gird_map_filtered_pub_.publish(grid_map_filtered_msg);
}

}
}
PLUGINLIB_EXPORT_CLASS(transform::pcl_to_grid_map::PCLToGridMap, nodelet::Nodelet);