#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <kinodynamic_path/path_replan.h>

#include <plan_manage/backward.hpp>
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