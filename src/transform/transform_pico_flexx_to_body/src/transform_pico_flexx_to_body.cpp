#include <transform_pico_flexx_to_body/transform_pico_flexx_to_body.h>

namespace transform_pico_flexx_to_body {
    TransformPicoFlexxToBody::TransformPicoFlexxToBody() 
        : nh_(), pnh_("~") {}
    
    
    TransformPicoFlexxToBody::~TransformPicoFlexxToBody(){}


    void TransformPicoFlexxToBody::onInit() {
        NODELET_INFO("pico flexx to body init");
        nh_ = getNodeHandle();
        pnh_ = getPrivateNodeHandle();

        camera_yaw_sub_ = nh_.subscribe("input_yaw", 1, &TransformPicoFlexxToBody::cameraYawCallback, this);
        imu_sub_ = nh_.subscribe("input_imu", 1, &TransformPicoFlexxToBody::imuCallback, this);
        
        // parameter name, variable, default value
        pnh_.param<std::string>("body_frame", body_frame_, "body_frame");
        pnh_.param<std::string>("pico_flexx_motor_frame", pico_flexx_motor_frame_, "pico_flexx_motor_frame");
        pnh_.param<std::string>("pico_flexx_frame", pico_flexx_frame_, "pico_flexx_frame");
        pnh_.param<std::string>("base_link", base_link_, "base_link");
        pnh_.param<float>("radius", radius_, 83.549*1e-3);
        pnh_.param<float>("height", height_, 124.489*1e-3);


        /* tf from base to focus of pico flexx*/
        transform_pico_flexx_base_to_focus_.transform.translation.x = radius_;
        transform_pico_flexx_base_to_focus_.transform.translation.y = 0.0;
        transform_pico_flexx_base_to_focus_.transform.translation.z = 0.0;
        tf2::Quaternion q_pico_to_base;
        q_pico_to_base.setRPY(0.0, 55.0f*M_PI/180.0f, 0.0);        
        transform_pico_flexx_base_to_focus_.transform.rotation.x = q_pico_to_base.getX();
        transform_pico_flexx_base_to_focus_.transform.rotation.y = q_pico_to_base.getY();
        transform_pico_flexx_base_to_focus_.transform.rotation.z = q_pico_to_base.getZ();
        transform_pico_flexx_base_to_focus_.transform.rotation.w = q_pico_to_base.getW();
        transform_pico_flexx_base_to_focus_.child_frame_id = pico_flexx_frame_;
        transform_pico_flexx_base_to_focus_.header.frame_id = pico_flexx_motor_frame_;
        transform_pico_flexx_base_to_focus_.header.stamp = ros::Time::now();

        /* tf from body to the base of pico flexx */
        transform_body_to_pico_flexx_base_.transform.translation.x = 0.0;
        transform_body_to_pico_flexx_base_.transform.translation.y = 0.0;
        transform_body_to_pico_flexx_base_.transform.translation.z = height_;
        tf2::Quaternion q_base_to_body;
        q_base_to_body.setRPY(0.0, 0.0, yaw_);
        transform_body_to_pico_flexx_base_.transform.rotation.x = q_base_to_body.getX();
        transform_body_to_pico_flexx_base_.transform.rotation.y = q_base_to_body.getY();
        transform_body_to_pico_flexx_base_.transform.rotation.z = q_base_to_body.getZ();
        transform_body_to_pico_flexx_base_.transform.rotation.w = q_base_to_body.getW();
        transform_body_to_pico_flexx_base_.child_frame_id = pico_flexx_motor_frame_;
        transform_body_to_pico_flexx_base_.header.frame_id = body_frame_;
        transform_body_to_pico_flexx_base_.header.stamp = ros::Time::now();

        transform_base_link_to_body_.transform.translation.x = 0.0f;
        transform_base_link_to_body_.transform.translation.y = 0.0f;
        transform_base_link_to_body_.transform.translation.z = 0.0f;
        transform_base_link_to_body_.transform.rotation.x = 0.0f;
        transform_base_link_to_body_.transform.rotation.y = 0.0f;
        transform_base_link_to_body_.transform.rotation.z = 0.0f;
        transform_base_link_to_body_.transform.rotation.w = 1.0f;
        transform_base_link_to_body_.child_frame_id = body_frame_;
        transform_base_link_to_body_.header.frame_id = base_link_;
        transform_base_link_to_body_.header.stamp = ros::Time::now();

        /* broadcast tf */
        static_tf_br_.sendTransform(transform_pico_flexx_base_to_focus_);
        static_tf_br_.sendTransform(transform_body_to_pico_flexx_base_);
        static_tf_br_.sendTransform(transform_base_link_to_body_);
    }

    void TransformPicoFlexxToBody::cameraYawCallback(const std_msgs::Float32::ConstPtr &yaw_msg) {
        yaw_ = yaw_msg->data;

        tf2::Quaternion q_body_to_pico_base;
        q_body_to_pico_base.setRPY(0.0f, 0.0f, yaw_);
        transform_body_to_pico_flexx_base_.transform.rotation.x = q_body_to_pico_base.getX();
        transform_body_to_pico_flexx_base_.transform.rotation.y = q_body_to_pico_base.getY();
        transform_body_to_pico_flexx_base_.transform.rotation.z = q_body_to_pico_base.getZ();
        transform_body_to_pico_flexx_base_.transform.rotation.w = q_body_to_pico_base.getW();
        // transform_body_to_pico_flexx_base_.transform.rotation.x = 0.0f;
        // transform_body_to_pico_flexx_base_.transform.rotation.y = 0.0f;
        // transform_body_to_pico_flexx_base_.transform.rotation.z = std::sin(yaw_*0.5f);
        // transform_body_to_pico_flexx_base_.transform.rotation.z = std::cos(yaw_*0.5f);
        transform_body_to_pico_flexx_base_.header.stamp = ros::Time::now();
        transform_body_to_pico_flexx_base_.header.frame_id = body_frame_;
        transform_body_to_pico_flexx_base_.child_frame_id = pico_flexx_motor_frame_;
        
        tf_br_.sendTransform(transform_body_to_pico_flexx_base_);
    }

    void TransformPicoFlexxToBody::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        transform_base_link_to_body_.transform.rotation = msg->orientation;
        transform_base_link_to_body_.header.stamp = ros::Time::now();
        tf_br_.sendTransform(transform_base_link_to_body_);
    }
}

PLUGINLIB_EXPORT_CLASS(transform_pico_flexx_to_body::TransformPicoFlexxToBody,
                       nodelet::Nodelet);