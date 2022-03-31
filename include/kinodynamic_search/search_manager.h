#ifndef _SEARCH_MANAGER_H_
#define _SEARCH_MANAGER_H_

#include <ros/ros.h>

//#include <path_searching/astar.h>
#include <kinodynamic_search/kinodynamic_search.h>

#include <plan_env/edt_environment.h>

#include <kinodynamic_search/plan_container.hpp>

namespace remake_planner{
    // Remake Planner Manager
    // Key algorithms of mapping and planning are called
    class RemakePlannerManager
    {
    public:
        RemakePlannerManager(/* args */);
        ~RemakePlannerManager();

        bool kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                         Eigen::Vector3d end_pt, Eigen::Vector3d end_vel);
        void initPlanModules(ros::NodeHandle& nh);
        void setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints);

        PlanParameters pp;
        LocalTrajData local_data;
        //LocalTrajData global_data;
        MidPlanData plan_data;
        EDTEnvironment::Ptr edt_environment;

    private:
        /* data */
        SDFMap::Ptr sdf_map;
        unique_ptr<KinodynamicSearch> kino_path_finder;

    public:
        typedef unique_ptr<RemakePlannerManager> Ptr;
    };
}   // namespace remake_planner

#endif