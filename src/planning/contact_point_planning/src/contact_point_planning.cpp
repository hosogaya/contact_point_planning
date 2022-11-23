#include <contact_point_planning/contact_point_planning.h>

namespace contact_point_planning {

ContactPointPlanning::ContactPointPlanning(): nh_("~"), pnh_ () {}

ContactPointPlanning::~ContactPointPlanning(){}

void ContactPointPlanning::onInit() {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    sub_pc2_ = nh_.subscribe("/input_pc2", 1, &ContactPointPlanning::callbackPC2, this);
    sub_gm_ = nh_.subscribe("/input_gm", 1, &ContactPointPlanning::callbackGM, this);
}

void ContactPointPlanning::callbackPC2(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        
}

void ContactPointPlanning::callbackGM(const grid_map_msgs::GridMap::ConstPtr& msg) {
    grid_map::GridMap map, filtered_map;
    grid_map::GridMapRosConverter::fromMessage(*msg, map);

    /** fill value of cells whose valuse is Nan */
    /** extract the data of the layer */
    std::string filtered_layer = filtered_map.getLayers().back();
    grid_map::Matrix& data = filtered_map.get(filtered_layer);

    /* decide searching area */

    /**
     *  create iterator for the searching area, 
     * or create another map whose area correspoinds to the searching are
    */

   /**
    * evaluate cells in the searching are, 
    * and get a feasible ground contact point 
   */

    // for (grid_map::GridMapIterator iterator(filtered_map); !iterator.isPastEnd(); ++iterator) {
    //     const grid_map::Index index(*iterator);
    //     /* extract position and value of the cell */
    //     /* evaluete the cell */
    // }
}

}

PLUGINLIB_EXPORT_CLASS(contact_point_planning::ContactPointPlanning, nodelet::Nodelet);