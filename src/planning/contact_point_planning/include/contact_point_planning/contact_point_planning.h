#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/PointCloud2.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GetGridMap.h>

namespace contact_point_planning {

class ContactPointPlanning : public nodelet::Nodelet
{
    private:
        ros::NodeHandle nh_, pnh_;
        ros::Subscriber sub_pc2_, sub_gm_;

        void callbackPC2(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void callbackGM(const grid_map_msgs::GridMap::ConstPtr& msg);

    public:
        ContactPointPlanning();
        ~ContactPointPlanning();
        virtual void onInit();
};

}