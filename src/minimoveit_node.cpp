#include "ros/ros.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionFK.h>

static const std::string IK_SERVICE_NAME = "compute_ik"; // name of ik service
static const std::string FK_SERVICE_NAME = "compute_fk"; // name of fk service

static const std::string SCENE_TOPIC = "planning_scene";
static const std::string ROBOT_DESCRIPTION = "robot_description";

planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor;

bool performTransform(geometry_msgs::PoseStamped &pose_msg, const std::string &target_frame);
bool isIKSolutionValid(const planning_scene::PlanningScene *planning_scene,
                       const kinematic_constraints::KinematicConstraintSet *constraint_set,
                       robot_state::RobotState *state,
                       const robot_model::JointModelGroup *jmg,
                       const double *ik_solution);

bool computeIKService(moveit_msgs::GetPositionIK::Request &req, moveit_msgs::GetPositionIK::Response &res);
bool computeFKService(moveit_msgs::GetPositionFK::Request &req, moveit_msgs::GetPositionFK::Response &res);

void computeIK(moveit_msgs::PositionIKRequest &req, moveit_msgs::RobotState &solution, moveit_msgs::MoveItErrorCodes &error_code,
               robot_state::RobotState &rs, const robot_state::GroupStateValidityCallbackFn &constraint = robot_state::GroupStateValidityCallbackFn());


bool performTransform(geometry_msgs::PoseStamped &pose_msg, const std::string &target_frame)
{
  if (!scene_monitor || !scene_monitor->getTFClient())
    return false;
  if (pose_msg.header.frame_id == target_frame)
    return true;
  if (pose_msg.header.frame_id.empty())
  {
    pose_msg.header.frame_id = target_frame;
    return true;
  }

  try
  {
    std::string error;
    ros::Time common_time;
    scene_monitor->getTFClient()->getLatestCommonTime(pose_msg.header.frame_id, target_frame, common_time, &error);
    if (!error.empty())
      ROS_ERROR("TF Problem: %s", error.c_str());

    tf::Stamped<tf::Pose> pose_tf, pose_tf_out;
    tf::poseStampedMsgToTF(pose_msg, pose_tf);
    pose_tf.stamp_ = common_time;
    scene_monitor->getTFClient()->transformPose(target_frame, pose_tf, pose_tf_out);
    tf::poseStampedTFToMsg(pose_tf_out, pose_msg);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("TF Problem: %s", ex.what());
    return false;
  }
  return true;
}


bool isIKSolutionValid(const planning_scene::PlanningScene *planning_scene,
                       const kinematic_constraints::KinematicConstraintSet *constraint_set,
                       robot_state::RobotState *state,
                       const robot_model::JointModelGroup *jmg,
                       const double *ik_solution)
{
  state->setJointGroupPositions(jmg, ik_solution);
  state->update();
  return (!planning_scene || !planning_scene->isStateColliding(*state, jmg->getName())) &&
    (!constraint_set || constraint_set->decide(*state).satisfied);
}

bool computeFKService(moveit_msgs::GetPositionFK::Request &req, moveit_msgs::GetPositionFK::Response &res)
{
  if (req.fk_link_names.empty())
  {
    ROS_ERROR("No links specified for FK request");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
    return true;
  }

  scene_monitor->updateFrameTransforms();

  const std::string &default_frame = scene_monitor->getRobotModel()->getModelFrame();
  bool do_transform = !req.header.frame_id.empty() && !robot_state::Transforms::sameFrame(req.header.frame_id, default_frame)
    && scene_monitor->getTFClient();
  bool tf_problem = false;

  robot_state::RobotState rs = planning_scene_monitor::LockedPlanningSceneRO(scene_monitor)->getCurrentState();
  robot_state::robotStateMsgToRobotState(req.robot_state, rs);
  for (std::size_t i = 0 ; i < req.fk_link_names.size() ; ++i)
    if (rs.getRobotModel()->hasLinkModel(req.fk_link_names[i]))
    {
      res.pose_stamped.resize(res.pose_stamped.size() + 1);
      tf::poseEigenToMsg(rs.getGlobalLinkTransform(req.fk_link_names[i]), res.pose_stamped.back().pose);
      res.pose_stamped.back().header.frame_id = default_frame;
      res.pose_stamped.back().header.stamp = ros::Time::now();
      if (do_transform)
        if (!performTransform(res.pose_stamped.back(), req.header.frame_id))
          tf_problem = true;
      res.fk_link_names.push_back(req.fk_link_names[i]);
    }
  if (tf_problem)
    res.error_code.val = moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
  else
    if (res.fk_link_names.size() == req.fk_link_names.size())
      res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    else
      res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
  return true;
}


bool computeIKService(moveit_msgs::GetPositionIK::Request &req, moveit_msgs::GetPositionIK::Response &res)
{
  scene_monitor->updateFrameTransforms();

  // check if the planning scene needs to be kept locked; if so, call computeIK() in the scope of the lock
  if (req.ik_request.avoid_collisions || !kinematic_constraints::isEmpty(req.ik_request.constraints))
  {
    planning_scene_monitor::LockedPlanningSceneRO ls(scene_monitor);
    kinematic_constraints::KinematicConstraintSet kset(ls->getRobotModel());
    robot_state::RobotState rs = ls->getCurrentState();
    kset.add(req.ik_request.constraints, ls->getTransforms());
    computeIK(req.ik_request, res.solution, res.error_code, rs, boost::bind(&isIKSolutionValid, req.ik_request.avoid_collisions ?
                                                                            static_cast<const planning_scene::PlanningSceneConstPtr&>(ls).get() : NULL,
                                                                            kset.empty() ? NULL : &kset, _1, _2, _3));
  }
  else
  {
    // compute unconstrained IK, no lock to planning scene maintained
    robot_state::RobotState rs = planning_scene_monitor::LockedPlanningSceneRO(scene_monitor)->getCurrentState();
    computeIK(req.ik_request, res.solution, res.error_code, rs);
  }

  return true;
}

void computeIK(moveit_msgs::PositionIKRequest &req,
                                                       moveit_msgs::RobotState &solution,
                                                       moveit_msgs::MoveItErrorCodes &error_code,
                                                       robot_state::RobotState &rs,
                                                       const robot_state::GroupStateValidityCallbackFn &constraint)
{
  const robot_state::JointModelGroup *jmg = rs.getJointModelGroup(req.group_name);
  if (jmg)
  {
    robot_state::robotStateMsgToRobotState(req.robot_state, rs);
    const std::string &default_frame = scene_monitor->getRobotModel()->getModelFrame();

    if (req.pose_stamped_vector.empty() || req.pose_stamped_vector.size() == 1)
    {
      geometry_msgs::PoseStamped req_pose = req.pose_stamped_vector.empty() ? req.pose_stamped : req.pose_stamped_vector[0];
      std::string ik_link = req.pose_stamped_vector.empty() ? (req.ik_link_names.empty() ? "" : req.ik_link_names[0]) : req.ik_link_name;

      if (performTransform(req_pose, default_frame))
      {
        bool result_ik = false;
        if (ik_link.empty())
          result_ik = rs.setFromIK(jmg, req_pose.pose, req.attempts, req.timeout.toSec(), constraint);
        else
          result_ik = rs.setFromIK(jmg, req_pose.pose, ik_link, req.attempts, req.timeout.toSec(), constraint);

        if(result_ik)
        {
          robot_state::robotStateToRobotStateMsg(rs, solution, false);
          error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        }
        else
          error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
      }
      else
        error_code.val = moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
    }
    else
    {
      if (req.pose_stamped_vector.size() != req.ik_link_names.size())
        error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
      else
      {
        bool ok = true;
        EigenSTL::vector_Affine3d req_poses(req.pose_stamped_vector.size());
        for (std::size_t k = 0 ; k < req.pose_stamped_vector.size() ; ++k)
        {
          geometry_msgs::PoseStamped msg = req.pose_stamped_vector[k];
          if (performTransform(msg, default_frame))
            tf::poseMsgToEigen(msg.pose, req_poses[k]);
          else
          {
            error_code.val = moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE;
            ok = false;
            break;
          }
        }
        if (ok)
        {
          if (rs.setFromIK(jmg, req_poses, req.ik_link_names, req.attempts, req.timeout.toSec(), constraint))
          {
            robot_state::robotStateToRobotStateMsg(rs, solution, false);
            error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
          }
          else
            error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
        }
      }
    }
  }
  else
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
}


int main(int argc, char **argv)
{

  //init ROS node
  ros::init (argc, argv, "minimoveit");
  ros::WallDuration sleep_t(0.5);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle root_node_handle;
  ros::ServiceServer fk_service;
  ros::ServiceServer ik_service;

  ROS_INFO("Started minimoveit.");

  ROS_INFO("Getting robot model.");
  robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION);
  robot_model::RobotModel model_copy(*((robot_model_loader.getModel()).get()));

  robot_model::RobotModelConstPtr kinematic_model(&model_copy);

  ROS_INFO("Setting up a copy of the robot's planning scene: ");
  ROS_INFO("    Getting a transform listener.");
  boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener());
  ROS_INFO("    Starting a planning scene monitor.");
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, tf);
  psm.startStateMonitor();
  psm.startWorldGeometryMonitor(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC, planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC);
  psm.startSceneMonitor(SCENE_TOPIC);

  scene_monitor.reset(&psm);


  fk_service = root_node_handle.advertiseService(FK_SERVICE_NAME, &computeFKService);
  ik_service = root_node_handle.advertiseService(IK_SERVICE_NAME, &computeIKService);

  ros::spin();

  return 0;
}
