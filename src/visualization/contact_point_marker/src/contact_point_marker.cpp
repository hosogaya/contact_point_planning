#include <contact_point_marker/contact_point_marker.h>

namespace visualization {

ContactPointMarker::ContactPointMarker() : nh_("~") {
    contact_point_sub_ = nh_.subscribe("/input", 1, &ContactPointMarker::callbackContactPointMarker, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/output", 1);

    nh_.param<bool>("publish_dummy", publish_dummy_, false);
    if (publish_dummy_) {
        timer_ = nh_.createTimer(ros::Duration(0.1), &ContactPointMarker::timerCallbackPublishDummy, this);
    }
}

ContactPointMarker::~ContactPointMarker() {}

void ContactPointMarker::callbackContactPointMarker(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    visualization_msgs::Marker marker;

    marker.header = msg->header;
    marker.pose = msg->pose;
    marker.ns = "Planned_contact_point";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    /** color */
    marker.color.r = 0.0f;
    marker.color.b = 0.0f;
    marker.color.g = 1.0f;
    marker.color.a = 1.0f;

    float radius = 0.050f; // 10mm  
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;

    marker_pub_.publish(marker);
}

void ContactPointMarker::timerCallbackPublishDummy(const ros::TimerEvent& e) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "Planned_contact_point";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    /** color */
    marker.color.r = 0.0f;
    marker.color.b = 0.0f;
    marker.color.g = 1.0f;
    marker.color.a = 1.0f;

    float radius = 1.0f; 
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;

    marker.pose.position.x = 0.0f;
    marker.pose.position.y = 0.0f;
    marker.pose.position.z = 0.0f;

    marker.pose.orientation.x = 0.0f;
    marker.pose.orientation.y = 0.0f;
    marker.pose.orientation.z = 0.0f;
    marker.pose.orientation.w = 1.0f;

    marker_pub_.publish(marker);    
}

}