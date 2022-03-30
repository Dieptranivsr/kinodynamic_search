#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <kinodynamic_search/path_replan.h>

#include <kinodynamic_search/backward.hpp>

namespace backward {
    backward::SignalHandling sh;
}

using namespace remake_planner;

int main(int argc, char** argv){
    ros::init(argc, argv, "remake_planner_node");
    ros::NodeHandle nh("~");

    MakePlan remake_plan;
    remake_plan.init(nh);

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}


// MakePlan - main_node.cpp -> path_replan.cpp --> search_manager.cpp ----> kinodynamic_search.cpp