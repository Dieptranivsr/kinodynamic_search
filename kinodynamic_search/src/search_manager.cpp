#include <kinodynamic_search/search_manager.h>

namespace remake_planner {

RemakePlannerManager::RemakePlannerManager(/* args */) { }
    
RemakePlannerManager::~RemakePlannerManager() { }
    std::cout << "Des RemakePlannerManager" << std::endl;
}

void RemakePlannerManage::initPlanModules(ros::NodeHandle& nh) {
    // read algorithm parameters
    //nh.param("manager/max_vel", )

    nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
    nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
    nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);

    sdf_map.reset(new SDFMap);
    sdf_map->initMap(nh);
    edt_environment->setMap(sdf_map);

    kino_path_finder.reset(new KinodynamicAstar);
    kino_path_finder->setParam(nh);
    kino_path_finder->setEnvironment(edt_environment);
    kino_path_finder->init();
}

void RemakePlannerManage::setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints){
    
}

}   // remake_planner