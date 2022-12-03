#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>


namespace visualization {

class ContactPointMarker {
    private:
        ros::NodeHandle nh_;
        ros::Publisher marker_pub_;
        ros::Subscriber contact_point_sub_;
        ros::Timer timer_;

        bool publish_dummy_;

        void callbackContactPointMarker(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void timerCallbackPublishDummy(const ros::TimerEvent& e);

    public:
        ContactPointMarker();
        ~ContactPointMarker();
};
}