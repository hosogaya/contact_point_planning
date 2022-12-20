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

void ContactPointMarker::callbackContactPointMarker(const geometry_msgs::PointStamped::ConstPtr& msg) {

    marker_msg_.header = msg->header;
    marker_msg_.pose.position.x = msg->point.x;
    marker_msg_.pose.position.y = msg->point.y;
    marker_msg_.pose.position.z = msg->point.z;
    marker_msg_.pose.orientation.x = 0.0f;
    marker_msg_.pose.orientation.y = 0.0f;
    marker_msg_.pose.orientation.z = 0.0f;
    marker_msg_.pose.orientation.w = 1.0f;
    marker_msg_.ns = "Planned_contact_point";
    marker_msg_.id = 0;

    marker_msg_.type = visualization_msgs::Marker::SPHERE;
    marker_msg_.action = visualization_msgs::Marker::ADD;
    marker_msg_.lifetime = ros::Duration();

    /** color */
    marker_msg_.color.r = 0.0f;
    marker_msg_.color.b = 0.0f;
    marker_msg_.color.g = 1.0f;
    marker_msg_.color.a = 1.0f;

    float radius = 0.050f; // 10mm  
    marker_msg_.scale.x = radius;
    marker_msg_.scale.y = radius;
    marker_msg_.scale.z = radius;

    marker_pub_.publish(marker_msg_);
}

void ContactPointMarker::timerCallbackPublishDummy(const ros::TimerEvent& e) {

    marker_msg_.header.frame_id = "map";
    marker_msg_.header.stamp = ros::Time::now();
    marker_msg_.ns = "Planned_contact_point";
    marker_msg_.id = 0;

    marker_msg_.type = visualization_msgs::Marker::SPHERE;
    marker_msg_.action = visualization_msgs::Marker::ADD;
    marker_msg_.lifetime = ros::Duration();

    /** color */
    marker_msg_.color.r = 0.0f;
    marker_msg_.color.b = 0.0f;
    marker_msg_.color.g = 1.0f;
    marker_msg_.color.a = 1.0f;

    float radius = 1.0f; 
    marker_msg_.scale.x = radius;
    marker_msg_.scale.y = radius;
    marker_msg_.scale.z = radius;

    marker_msg_.pose.position.x = 0.0f;
    marker_msg_.pose.position.y = 0.0f;
    marker_msg_.pose.position.z = 0.0f;

    marker_msg_.pose.orientation.x = 0.0f;
    marker_msg_.pose.orientation.y = 0.0f;
    marker_msg_.pose.orientation.z = 0.0f;
    marker_msg_.pose.orientation.w = 1.0f;

    marker_pub_.publish(marker_msg_);    
}

}