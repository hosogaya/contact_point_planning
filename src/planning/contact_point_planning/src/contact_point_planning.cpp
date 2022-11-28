#include <contact_point_planning/contact_point_planning.h>

namespace contact_point_planning {

ContactPointPlanning::ContactPointPlanning(): nh_("~"), pnh_ () {}

ContactPointPlanning::~ContactPointPlanning(){}

void ContactPointPlanning::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    sub_pc2_ = nh_.subscribe("/input_pc2", 1, &ContactPointPlanning::callbackPC2, this);
    sub_gm_ = nh_.subscribe("/grid_map_filtered", 1, &ContactPointPlanning::callbackGM, this);
    pub_gm_ = nh_.advertise<grid_map_msgs::GridMap>("/output_gm", 1);
}

void ContactPointPlanning::callbackPC2(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        
}

void ContactPointPlanning::callbackGM(const grid_map_msgs::GridMap::ConstPtr& msg) {
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(*msg, map);

    /** create variance layer */
    /** add layer for variance */
    if (!map.exists(variance_layer_)) {
        map.add(variance_layer_);
    }
    /** Create reference of Marix for variance layer */
    if (!map.exists(input_layer_)) return; // 

    const grid_map::Matrix& height_map{map[input_layer_]};
    grid_map::Matrix& variance_mat{map[variance_layer_]};

    /** fill variance to each cell */
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index(*iterator);
        auto& variance{variance_mat(index(0), index(1))};
        if (!std::isfinite(height_map(index(0), index(1)))) continue;
        variance = 0.0f;
        int num = 0;
        for (int i = -1; i <=1; ++i) 
            for (int j = -1; j <=1; ++j) {
                if (index(0)+i >= 0 && index(0)+i < map.getSize()(0) && index(1)+j >=0 && index(1)+j < map.getSize()(1)) {
                    auto& height = height_map(index(0)+i, index(1)+j);
                    if (std::isfinite(height)) {
                        num++;
                        variance += std::pow(height_map(index(0)+i, index(1)+j) - height_map(index(0), index(1)), 2.0f);
                    }
                }   
            }
        if (num != 0) {
            variance /= num;
            variance *= 1e6;
        }
        else variance = NAN;
    }

    /* publish variance layer*/
    grid_map_msgs::GridMap grid_map_msg;
    grid_map::GridMapRosConverter::toMessage(map, grid_map_msg);
    grid_map_msg.info.header.stamp = ros::Time::now();
    pub_gm_.publish(grid_map_msg);

    /** extract the data of the layer */

    /* decide searching area */

    /**
     *  create iterator for the searching area, (spiral iterator)
    */

   /**
    * evaluate cells in the searching are, 
    * and get a feasible ground contact point 
   */
}

}

PLUGINLIB_EXPORT_CLASS(contact_point_planning::ContactPointPlanning, nodelet::Nodelet);