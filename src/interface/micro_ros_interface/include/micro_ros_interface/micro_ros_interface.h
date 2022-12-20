#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

namespace interface {

class MicroRosInterface {

    private:
        ros::NodeHandle nh_, pnh_;  
        
        // to micro computer
        ros::Publisher next_cp_pub_;
        ros::Publisher yaw_rate_pub_;
        
        // to ros
        ros::Publisher imu_pub_;
        ros::Publisher camera_yaw_pub_;

        // from ros
        ros::Subscriber next_cp_sub_;
        ros::Subscriber yaw_rate_sub_;
        
        // from micro computer 
        ros::Subscriber imu_sub_;
        ros::Subscriber camera_yaw_sub_;

        // message 
        sensor_msgs::Imu imu_msg_;
        geometry_msgs::Point next_cp_msg_;

    public:
        MicroRosInterface();
        ~MicroRosInterface();

        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
        void nextCpCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
        void yawRateCallback(const std_msgs::Float32::ConstPtr& msg);
        void cameraYawCallback(const std_msgs::Float32::ConstPtr& msg);
};

}