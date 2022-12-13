/* ros depends */
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

/* custom messages */
#include <fcp_msgs/Behavior.h>


namespace Controller {

class JoyStickController{
    public:
        JoyStickController();
        ~JoyStickController();

    /** functions */
    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
        void timerCallback(const ros::TimerEvent& e);
    
    /* node handler and publisher*/
    private:
        ros::NodeHandle nh_, pnh_;
        ros::Timer timer_;
        ros::Publisher yaw_rate_pub_;
        ros::Publisher attitude_pub_;
        ros::Publisher behavior_pub_;
        ros::Subscriber joy_cmd_sub_;

    /** messages */
    private:
        std_msgs::Float32 yaw_rate_msg_;
        std_msgs::Float32MultiArray attitue_msg_;
        fcp_msgs::Behavior behavior_msg_;

        sensor_msgs::Joy last_joy_cmd_;
};
}