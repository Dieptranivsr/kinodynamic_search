#include <ros/ros.h>

#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>

#include <Eigen/Eigen>

#include <kinodynamic_path/path_replan.h>

namespace remake_planner {

void MakePlan::init(ros::NodeHandle& nh){
  current_wp  = 0;
  exec_state  = PLAN_EXEC_STATE::INIT;
  have_target = false;
  have_odom   = false;

  /* MakePlan parameter */
  nh.param("makePlan/flyight_type", target_type, -1);
  nh.param("makePlan/thresh_replan", replan_thresh_, -1.0);
  nh.param("makePlan/thresh_no_replan", no_replan_thresh_, -1.0);

  nh.param("makePlan/waypoint_num", waypoint_num, -1);
  for (int i = 0; i < waypoint_num, i++){
    nh.param("makePlan/waypoint" + to_string(i) + "_x", waypoints[i][0], -1.0);
    nh.param("makePlan/waypoint" + to_string(i) + "_y", waypoints[i][1], -1.0);
    nh.param("makePlan/waypoint" + to_string(i) + "_z", waypoints[i][2], -1.0);
  }

  /* initialize main modules */
  search_manager.reset(new RemakePlannerManager);
  search_manager->initPlanModules(nh);
  visual.reset(new PlanningVisualization(nh));

  /* callback */
  exec_timer = nh.createTimer(ros::Duration(0.01), &MakePlan::execCallback, this);
  //safety_timer = nh.createTimer(ros::Duration(0.01), &MakePlan::checkCollisionCallback, this);

  waypoint_sub = nh.subscriber("/waypoint_generator/waypoints", 1, MakePlan::waypointsCallback, this);
  odom_sub = nh.subscriber("/odom_world", 1, MakePlan::odomCallback, this);

  replan_pub  = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
}

void MakePlan::waypointsCallback(const nav_msgs::PathConstPtr& msg){
  if (msg->pose[0].pose.position.z < -0.1) return;

  cout << "Triggered by dieptuantran!" << endl;
  //trigger = true;

  if (target_type == TARGET_TYPE::MANUAL){
    end_pt << msg->pose[0].pose.position.x, msg->pose[0].pose.position.y, 1.0;
  }
  else if (target_type == TARGET_TYPE::MANUAL){
    end_pt(0) = waypoints[current_wp][0];
    end_pt(1) = waypoints[current_wp][1];
    end_pt(2) = waypoints[current_wp][2];
    current_wp = (current_wp + 1) % waypoint_num;
  }

  visual->drawGoal(end_pt, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel.setZero();
  have_target = true;

  if (exec_state == WAIT_TARGET)
    changePLANExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state == EXEC_TRAJ)
    changePLANExecState(REPLAN_TRAJ, "TRIG");
}

void MakePlan::odomCallback(const nav_msgs::OdometryConstPtr& msg){
  odom_pos(0) = msg->pose.pose.position.x;
  odom_pos(1) = msg->pose.pose.position.y;
  odom_pos(2) = msg->pose.pose.position.z;

  odom_vel(0) = msg->twist.twist.linear.x;
  odom_vel(1) = msg->twist.twist.linear.y;
  odom_vel(2) = msg->twist.twist.linear.z;

  odom_orient.w() = msg->pose.pose.orientation.w;
  odom_orient.x() = msg->pose.pose.orientation.x;
  odom_orient.y() = msg->pose.pose.orientation.y;
  odom_orient.z() = msg->pose.pose.orientation.z;
  
  have_odom = true;
}

void MakePlan::changePLANExecState(PLAN_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int    pre_s        = int(exec_state);
  exec_state         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void MakePlan::printPLANExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };

  cout << "[PLAN]: state: " + state_str[int(exec_state_)] << endl;
}

void MakePlan::execCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printPLANExecState();
    if (!have_odom) cout << "no odom." << endl;
    if (!trigger) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state) {
    case INIT: {
      if (!have_odom) {
        return;
      }
      if (!trigger) {
        return;
      }
      changePLANExecState(WAIT_TARGET, "PLAN");
      break;
    }

    case WAIT_TARGET: {
      if (!have_target)
        return;
      else {
        changePLANExecState(GEN_NEW_TRAJ, "PLAN");
      }
      break;
    }

    case GEN_NEW_TRAJ: {
      start_pt  = odom_pos;
      start_vel = odom_vel;
      start_acc.setZero();

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      start_yaw(0)         = atan2(rot_x(1), rot_x(0));
      start_yaw(1) = start_yaw(2) = 0.0;

      bool success = callKinodynamicReplan();
      if (success) {
        changePLANExecState(EXEC_TRAJ, "PLAN");
      } else {
        // have_target = false;
        // changePLANExecState(WAIT_TARGET, "PLAN");
        changePLANExecState(GEN_NEW_TRAJ, "PLAN");
      }
      break;
    }

    case EXEC_TRAJ: {
      // <full version: have local_traj>
      if ((end_pt - pos).norm() < no_replan_thresh_) {
        // cout << "near end" << endl;
        return;

      } else if ((start_pt - pos).norm() < replan_thresh_) {
        // cout << "near start" << endl;
        return;

      } else {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ: {
      start_pt  = odom_pos;
      start_vel = odom_vel;
      start_acc.setZero();

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      std_msgs::Empty replan_msg;
      replan_pub.publish(replan_msg);

      bool success = callKinodynamicReplan();
      if (success) {
        changePLANExecState(EXEC_TRAJ, "PLAN");
      } else {
        changePLANExecState(GEN_NEW_TRAJ, "PLAN");
      }
      break;
    }
  }
}

bool MakePlan::callKinodynamicReplan() {
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);

  if (plan_success) {

    planner_manager_->planYaw(start_yaw_);

    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;
    visual->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));

    return true;

  } else {
    cout << "generate new traj fail." << endl;
    return false;
  }
}
}