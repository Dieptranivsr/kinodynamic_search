#ifndef _SEARCH_MANAGER_H_
#define _SEARCH_MANAGER_H_

#include <ros/ros.h>

#include <path_searching/astar.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_env/plan_container.hpp>

#include <plan_env/edt_environment.h>
#include <kinodynamic_path/plan_container.h>

namespace remake_planner{
    // Remake Planner Manager
    // Key algorithms of mapping and planning are called
    class RemakePlannerManager
    {
    public:
        RemakePlannerManager(/* args */);
        ~RemakePlannerManager();

        void initPlanModules(ros::NodeHandle& nh);


        PlanParameters local_data;
        LocalTrajData local_data;
        LocalTrajData global_data;
        LocalTrajData plan_data;
        LocalTrajData edt_environment;

    private:
        /* data */
        SDFMap::Ptr sdf_map;
        unique_ptr<KinodynamicAstar> kino_path_finder;
        

    public:
        typedef unique_ptr<RemakePlannerManager> Ptr;
    };
}   // namespace remake_planner

#endif