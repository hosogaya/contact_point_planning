#include <transform_pico_flexx_to_body/transform_pico_flexx_to_body.h>

namespace transform_pico_flexx_to_body {
    TransformPicoFlexxToBody::TransformPicoFlexxToBody() 
        : nh_(), pnh_("~"), tf_buffer_(), tf_listener_(tf_buffer_) {}
    TransformPicoFlexxToBody::~TransformPicoFlexxToBody(){}


    void TransformPicoFlexxToBody::onInit() {
        NODELET_INFO("pico flexx to body init");
        nh_ = getNodeHandle();
        pnh_ = getPrivateNodeHandle();

        pub_pc_ = nh_.advertise<sensor_msgs::PointCloud2>("/output_pc2", 1);
        sub_pc_ = nh_.subscribe("/input_pc2", 1, &TransformPicoFlexxToBody::callbackPC, this);
        sub_yaw_ = nh_.subscribe("/input_yaw", 1, &TransformPicoFlexxToBody::callbackYaw, this);
        // parameter name, variable, default value
        pnh_.param<std::string>("output_frame", output_frame_, "output_frame");
        pnh_.param<std::string>("intermediate_frame", intermediate_frame_, "intermediate_frame");
        pnh_.param<std::string>("input_frame", input_frame_, "input_frame");
        pnh_.param<float>("radius", radius_, 83.549*1e-3);
        pnh_.param<float>("height", height_, 124.489*1e-3);


        /* tf from base to focus of pico flexx*/
        transform_pico_flexx_base_to_focus_.transform.translation.x = radius_;
        transform_pico_flexx_base_to_focus_.transform.translation.y = 0.0;
        transform_pico_flexx_base_to_focus_.transform.translation.z = 0.0;
        tf2::Quaternion q_pico_to_base;
        q_pico_to_base.setRPY(0.0, M_PI/4.0, 0.0);        
        transform_pico_flexx_base_to_focus_.transform.rotation.x = q_pico_to_base.getX();
        transform_pico_flexx_base_to_focus_.transform.rotation.y = q_pico_to_base.getY();
        transform_pico_flexx_base_to_focus_.transform.rotation.z = q_pico_to_base.getZ();
        transform_pico_flexx_base_to_focus_.transform.rotation.w = q_pico_to_base.getW();
        transform_pico_flexx_base_to_focus_.child_frame_id = input_frame_;
        transform_pico_flexx_base_to_focus_.header.frame_id = intermediate_frame_;
        transform_pico_flexx_base_to_focus_.header.stamp = ros::Time::now();

        /* tf from body to the base of pico flexx */
        transform_body_pico_flexx_base_.transform.translation.x = 0.0;
        transform_body_pico_flexx_base_.transform.translation.y = 0.0;
        transform_body_pico_flexx_base_.transform.translation.z = height_;
        tf2::Quaternion q_base_to_body;
        q_base_to_body.setRPY(0.0, 0.0, yaw_);
        transform_body_pico_flexx_base_.transform.rotation.x = q_base_to_body.getX();
        transform_body_pico_flexx_base_.transform.rotation.y = q_base_to_body.getY();
        transform_body_pico_flexx_base_.transform.rotation.z = q_base_to_body.getZ();
        transform_body_pico_flexx_base_.transform.rotation.w = q_base_to_body.getW();
        transform_body_pico_flexx_base_.child_frame_id = intermediate_frame_;
        transform_body_pico_flexx_base_.header.frame_id = output_frame_;
        transform_body_pico_flexx_base_.header.stamp = ros::Time::now();

        /* broadcast tf */
        tf_br_.sendTransform(transform_pico_flexx_base_to_focus_);
        tf_br_.sendTransform(transform_body_pico_flexx_base_);
    
        timer_ = nh_.createTimer(ros::Duration(0.1), &TransformPicoFlexxToBody::timerCallback, this);
    }

    void TransformPicoFlexxToBody::timerCallback(const ros::TimerEvent&) {
        transform_body_pico_flexx_base_.header.stamp = ros::Time::now();
        transform_pico_flexx_base_to_focus_.header.stamp = ros::Time::now();
        tf_br_.sendTransform(transform_pico_flexx_base_to_focus_);
        tf_br_.sendTransform(transform_body_pico_flexx_base_);
    }

    void TransformPicoFlexxToBody::callbackPC(const sensor_msgs::PointCloud2::ConstPtr &pc2_msg) {
        /* transform point cloud from pico flexx to body frame */
        sensor_msgs::PointCloud2 pc2_trans;
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(output_frame_, pc2_msg->header.frame_id, ros::Time(0));
            Eigen::Matrix4f mat = tf2::transformToEigen(transform.transform).matrix().cast<float>();
            pcl_ros::transformPointCloud(mat, *pc2_msg, pc2_trans);
            pc2_trans.header.frame_id = output_frame_;
            // pcl::PointCloud<pcl::PointXYZ> pcl_out_ptr(pcl_out_);
            // pcl::fromROSMsg(pc2_trans, pcl_out_ptr);

            pub_pc_.publish(pc2_trans);
        }
        catch (tf2::TransformException & ex) {
            ROS_WARN("%s", ex.what());
            return;
        }
    }

    void TransformPicoFlexxToBody::callbackYaw(const std_msgs::Float32 &yaw_msg) {
        yaw_ = yaw_msg.data;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_);
        transform_body_pico_flexx_base_.transform.rotation.x = q.getX();
        transform_body_pico_flexx_base_.transform.rotation.y = q.getY();
        transform_body_pico_flexx_base_.transform.rotation.z = q.getZ();
        transform_body_pico_flexx_base_.transform.rotation.z = q.getW();
        transform_body_pico_flexx_base_.header.stamp = ros::Time::now();
    }
}

PLUGINLIB_EXPORT_CLASS(transform_pico_flexx_to_body::TransformPicoFlexxToBody,
                       nodelet::Nodelet);