#include <kinodynamic_search/search_manager.h>

namespace remake_planner {

RemakePlannerManager::RemakePlannerManager(/* args */) { }
    
RemakePlannerManager::~RemakePlannerManager() {
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

bool RemakePlannerManager::kinodynamicReplan(Eigen::Vector3d _start_pt, Eigen::Vector3d _start_vel,
                                           Eigen::Vector3d _start_acc, Eigen::Vector3d _end_pt,
                                           Eigen::Vector3d _end_vel) {
    std::cout << "[kino replan]: -----------------------" << std::endl;
    cout << "start: " << _start_pt.transpose() << ", " << _start_vel.transpose() << ", "
         << _start_acc.transpose() << "\ngoal:" << _end_pt.transpose() << ", " << _end_vel.transpose()
         << endl;

    if ((_start_pt - _end_pt).norm() < 0.2) {
        cout << "Close goal" << endl;
        return false;
    }

    ros::Time t1, t2;

    local_data_.start_time_ = ros::Time::now();
    double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

    Eigen::Vector3d init_pos = _start_pt;
    Eigen::Vector3d init_vel = _start_vel;
    Eigen::Vector3d init_acc = _start_acc;

    // kinodynamic path searching

    t1 = ros::Time::now();

    kino_path_finder_->reset();

    int status = kino_path_finder_->search(_start_pt, _start_vel, _start_acc, _end_pt, _end_vel, true);

    if (status == KinodynamicAstar::NO_PATH) {
        cout << "[kino replan]: kinodynamic search fail!" << endl;

        // retry searching with discontinuous initial state
        kino_path_finder_->reset();
        status = kino_path_finder_->search(_start_pt, _start_vel, _start_acc, _end_pt, _end_vel, false);

        if (status == KinodynamicAstar::NO_PATH) {
            cout << "[kino replan]: Can't find path." << endl;
            return false;
        } else {
        cout << "[kino replan]: retry search success." << endl;
        }

    } else {
        cout << "[kino replan]: kinodynamic search success." << endl;
    }

    return true;
}

}   // remake_planner