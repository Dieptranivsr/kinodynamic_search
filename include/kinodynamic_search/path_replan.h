#ifndef _MAKE_PLANNER_H_
#define _MAKE_PLANNER_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <kinodynamic_search/search_manager.h>
#include <traj_utils/planning_visualization.h>

#include <Eigen/Eigen>

using std::vector;

namespace remake_planner{

class MakePlan {
private:
    /* ------------- flag ------------ */
    enum PLAN_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
    enum TARGET_TYPE { MANUAL = 1, PRESENT = 2 };

    /* planning utils */
    RemakePlannerManager::Ptr search_manager;
    PlanningVisualization::Ptr visual;

    /* parameter */
    int target_type;        // 1 manual select, 2 hard code
    double no_replan_thresh, replan_thresh;
    double waypoints[50][3];
    int waypoint_num;

    /* planning data */
    bool trigger, have_target, have_odom;
    PLAN_EXEC_STATE exec_state;

    /* ----------- Variable ----------- */
    Eigen::Vector3d odom_pos, odom_vel;     // odometry state
    Eigen::Quaterniond odom_orient;
    
    Eigen::Vector3d start_pt, start_vel, start_acc, start_yaw;      // start state
    Eigen::Vector3d end_pt, end_vel;                                // target state
    int current_wp;

    /* ROS utils */
    ros::NodeHandle node;
    ros::Timer exec_timer, safety_timer;
    ros::Subscriber waypoint_sub, odom_sub;
    ros::Publisher replan_pub;

    /* helper functions */
    bool callKinodynamicReplan();                       // front-end and back-end method
    
    void changePLANExecState(PLAN_EXEC_STATE new_state, string pos_call);
    void printPLANExecState();

    /* Ros functions */
    void execCallback(const ros::TimerEvent& e);
    //void checkCollisionCallback(const ros::TimerEvent& e);
    void waypointsCallback(const nav_msgs::PathConstPtr& msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
public:
    /* ------------------------------------- */
    MakePlan() {
    }
    ~MakePlan() {
    }

    void init(ros::NodeHandle& nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}   // namespace remake_planner

#endif