#include <contact_point_planning/contact_point_planning.h>

namespace contact_point_planning {

ContactPointPlanning::ContactPointPlanning(): nh_("~"), pnh_ () {}

ContactPointPlanning::~ContactPointPlanning(){}

void ContactPointPlanning::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    sub_pc2_ = nh_.subscribe("/input_pc2", 1, &ContactPointPlanning::callbackPC2, this);
    sub_gm_ = nh_.subscribe("/input_gm", 1, &ContactPointPlanning::callbackGM, this);
    planning_leg_id_sub_ =  nh_.subscribe("input_planning_leg_id", 1, &ContactPointPlanning::planningLegIdCallback, this);
    pub_gm_ = nh_.advertise<grid_map_msgs::GridMap>("/output_gm", 1);
    pub_contact_point_ = nh_.advertise<geometry_msgs::PointStamped>("/output_cp", 1);

    pnh_.param<std::string>("variance_layer", variance_layer_, "variance");
    pnh_.param<std::string>("input_layer", input_layer_, "elevation_filtered");
    pnh_.param<float>("searching_area_radius", searching_area_radius_, 0.1f);
    pnh_.param<float>("center_x", searching_area_center_x_, 0.230f);
    pnh_.param<float>("center_y", searching_area_center_y_, 0.170f);

    ROS_INFO("radius: %f", searching_area_radius_);
    ROS_INFO("center_x: %f", searching_area_center_x_);
    ROS_INFO("center_y: %f", searching_area_center_y_);
}

void ContactPointPlanning::callbackPC2(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        
}

void ContactPointPlanning::callbackGM(const grid_map_msgs::GridMap::ConstPtr& msg) {
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(*msg, map);

    /** Create reference of Marix for variance layer */
    if (!map.exists(input_layer_)) return; // 

    // check whether the searching are is in the grid map
    try {
        map.atPosition(input_layer_, searching_area_center_);
    }
    catch (std::out_of_range& e) {
        ROS_INFO("%s", e.what());
        return;
    }

    /** create variance layer */
    /** add layer for variance */
    if (!map.exists(variance_layer_)) {
        map.add(variance_layer_);
    }

    const grid_map::Matrix& height_map{map[input_layer_]};
    grid_map::Matrix& variance_mat{map[variance_layer_]};

    /** fill variance to each cell */
    size_t window_size = 5;
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
        auto& variance{variance_mat(index(0), index(1))};
        if (!std::isfinite(height_map(index(0), index(1)))) continue;
        variance = 0.0f;
        int num = 0;
        bool is_finite = true;
        for (int i = -floor(window_size/2); i <=floor(window_size/2); ++i) {
            for (int j = -floor(window_size/2); j <=floor(window_size/2); ++j) {
                if (index(0)+i >= 0 && index(0)+i < map.getSize()(0) && index(1)+j >=0 && index(1)+j < map.getSize()(1)) {
                    auto& height = height_map(index(0)+i, index(1)+j);
                    if (!std::isfinite(height)) {
                        is_finite = false;
                        break;
                    }
                    else {
                        num++;
                        variance += std::pow(height_map(index(0)+i, index(1)+j) - height_map(index(0), index(1)), 2.0f);
                    }
                } // end if    
            }// end for j
            if (!is_finite) break;
        }// end for i 

        if (num == window_size*window_size) {
            variance /= num;
            // variance *= 1e6; // m->mm
        }
        else variance = NAN;
    }

    // normalize 
    double max,min;
    bool initialize_flag = false;
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
        auto& variance{variance_mat(index(0), index(1))};
        if (!std::isfinite(variance_mat(index(0), index(1)))) continue;
        if (!initialize_flag) {
            max = variance;
            min = variance;
            initialize_flag = true;
        }
        else {
            if (variance > max) {
                max = variance;
            }
            if (variance < min) {
                min = variance;
            }
        }
    }

    /** add layer for variance */
    std::string normalized_variance_layer = variance_layer_+"normalized";
    if (!map.exists(normalized_variance_layer)) {
        map.add(normalized_variance_layer);
    }

     for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
        auto& variance{variance_mat(index(0), index(1))};
        auto& normalized_variance{map[normalized_variance_layer](index(0), index(1))};
        
        if (!std::isfinite(variance_mat(index(0), index(1)))) {
            normalized_variance = NAN;
        }
        else {
            normalized_variance = (variance - min)/(max - min);        
        }

    }

    /* publish variance layer*/
    grid_map_msgs::GridMap grid_map_msg;
    grid_map::GridMapRosConverter::toMessage(map, grid_map_msg);
    grid_map_msg.info.header.stamp = ros::Time::now();
    pub_gm_.publish(grid_map_msg);

    /** extract the data of the layer -> variance_mat*/

    /* decide searching area */
    ROS_INFO("variance map is published");
    /**
     *  create iterator for the searching area, (spiral iterator)
    */
    grid_map::Index solution_ind;
    grid_map::Position pos;
    float height;
    float solution_variance;
    solution_variance = thres_variance_;
    ROS_INFO("Containars are prepared");
    for (grid_map::SpiralIterator iterator(map, searching_area_center_, searching_area_radius_); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
        auto& variance{map[normalized_variance_layer](index(0), index(1))};
        ROS_INFO("Information of the cell is extracted");

        if (!std::isfinite(variance)) continue;
        if (variance < solution_variance) {
            solution_variance = variance;
            solution_ind = index;
            map.getPosition(index, pos);
            auto& h{height_map(index(0), index(1))};
            height = h;
            ROS_INFO("update variables");
        }
    }
    ROS_INFO("Iteration is finished");
    
    if (solution_variance < thres_variance_) {
        planned_cp_msg_.header.frame_id = map.getFrameId();
        planned_cp_msg_.header.stamp = ros::Time::now();

        planned_cp_msg_.point.x = pos(0);
        planned_cp_msg_.point.y = pos(1);
        planned_cp_msg_.point.z = height;

        pub_contact_point_.publish(planned_cp_msg_);
        ROS_INFO("Publish pose message");
    }

}

void ContactPointPlanning::planningLegIdCallback(const std_msgs::Int8::ConstPtr& msg) {
    planning_leg_id_ = msg->data;
    if (planning_leg_id_ == 0) {
        searching_area_center_.x() = searching_area_center_x_;
        searching_area_center_.y() =-searching_area_center_y_;
    }
    else if (planning_leg_id_ == 5) {
        searching_area_center_.x() = searching_area_center_x_;
        searching_area_center_.y() = searching_area_center_y_;
    }
    ROS_INFO("searching are center: %f, %f", searching_area_center_.x(), searching_area_center_.y());
}


}

PLUGINLIB_EXPORT_CLASS(contact_point_planning::ContactPointPlanning, nodelet::Nodelet);