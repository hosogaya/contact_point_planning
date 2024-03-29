#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

#include <fcp_msgs/PlannedContactPoint.h>

namespace visualization {

class ContactPointMarker {
    private:
        ros::NodeHandle nh_;
        ros::Publisher marker_pub_;
        ros::Subscriber contact_point_sub_;
        ros::Timer timer_;

        visualization_msgs::Marker marker_msg_;

        bool publish_dummy_;

        // void callbackContactPointMarker(const geometry_msgs::PointStamped::ConstPtr& msg);
        void callbackContactPointMarker(const fcp_msgs::PlannedContactPoint::ConstPtr& msg);
        void timerCallbackPublishDummy(const ros::TimerEvent& e);

    public:
        ContactPointMarker();
        ~ContactPointMarker();
};
}