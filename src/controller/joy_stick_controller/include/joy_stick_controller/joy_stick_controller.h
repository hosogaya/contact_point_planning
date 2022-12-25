/* ros depends */
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

/* custom messages */
#include <fcp_msgs/Behavior.h>
#include <fcp_msgs/PlannedContactPoint.h>


namespace Controller {

class JoyStickController{
    public:
        JoyStickController();
        ~JoyStickController();

    /** functions */
    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
        // void cpCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
        void cpCallback(const fcp_msgs::PlannedContactPoint::ConstPtr & msg);
        void timerCallback(const ros::TimerEvent& e);
        void publishCP();
    
    /* node handler and publisher*/
    private:
        ros::NodeHandle nh_, pnh_;
        ros::Timer timer_;
        ros::Publisher yaw_rate_pub_;
        ros::Publisher attitude_pub_;
        ros::Publisher behavior_pub_;
        ros::Publisher contact_point_pub_;
        ros::Subscriber joy_cmd_sub_;
        ros::Subscriber contact_point_sub_;

        // for debug
        // ros::Publisher joystick_pub_;

    /** messages */
    private:
        std_msgs::Float32 yaw_rate_msg_;
        std_msgs::Float32MultiArray attitude_msg_;
        fcp_msgs::Behavior behavior_msg_;
        // geometry_msgs::PointStamped cp_msg_;
        fcp_msgs::PlannedContactPoint cp_msg_;
        bool get_cp_ = false;
        bool get_joy_ = false;

        sensor_msgs::Joy last_joy_cmd_;

    /** buttons */
    private:
        int circle_ = 1;
        int triangle_ = 2;
        int cross_ = 0;
        int rectangle_ = 3;
        int LV_axis_ = 1;
        int LH_axis_ = 0;
        int RV_axis_ = 4;
        int RH_axis_ = 3;
};
}