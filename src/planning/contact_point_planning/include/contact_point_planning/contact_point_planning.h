#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <sensor_msgs/PointCloud2.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GetGridMap.h>
#include <std_msgs/Int8.h>

namespace contact_point_planning {

class ContactPointPlanning : public nodelet::Nodelet
{
    private:
        ros::NodeHandle nh_, pnh_;
        ros::Subscriber sub_pc2_, sub_gm_;
        ros::Subscriber planning_leg_id_sub_;
        ros::Publisher pub_gm_, pub_contact_point_;

        std::string input_layer_;
        std::string variance_layer_;

        geometry_msgs::PointStamped planned_cp_msg_;

        int8_t planning_leg_id_;
        grid_map::Position searching_area_center_;
        float thres_variance_ = 0.5f;
        float searching_area_radius_ = 0.1f;
        float searching_area_center_x_; 
        float searching_area_center_y_; 

        void callbackPC2(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void callbackGM(const grid_map_msgs::GridMap::ConstPtr& msg);
        void planningLegIdCallback(const std_msgs::Int8::ConstPtr& msg);

    public:
        ContactPointPlanning();
        ~ContactPointPlanning();
        virtual void onInit();
};

}