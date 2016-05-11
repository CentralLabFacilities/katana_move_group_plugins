#include <moveit/katana_pick_place/reachable_valid_pose_filter.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/bind.hpp>
#include <ros/console.h>

pick_place::ReachableAndValidPoseFilter::ReachableAndValidPoseFilter(const planning_scene::PlanningSceneConstPtr &scene,
                                                                     const collision_detection::AllowedCollisionMatrixConstPtr &collision_matrix,
                                                                     const constraint_samplers::ConstraintSamplerManagerPtr &constraints_sampler_manager) :
  ManipulationStage("reachable & valid pose filter"),
  planning_scene_(scene),
  collision_matrix_(collision_matrix),
  constraints_sampler_manager_(constraints_sampler_manager),
  position_tolerance_(0.01), // 1 cm
  orientation_tolerance_(0.2) // 11 degree
{
}

namespace
{
bool isStateCollisionFree(const planning_scene::PlanningScene *planning_scene,
                          const collision_detection::AllowedCollisionMatrix *collision_matrix,
                          bool verbose,
                          const pick_place::ManipulationPlan *manipulation_plan,
                          robot_state::RobotState *state,
                          const robot_model::JointModelGroup *group,
                          const double *joint_group_variable_values) 
{ 
  collision_detection::CollisionRequest req;
  req.verbose = verbose;
  req.group_name = manipulation_plan->shared_data_->planning_group_->getName();
  
  state->setJointGroupPositions(group, joint_group_variable_values);
  
  if (manipulation_plan->approach_posture_.points.empty())
  {  
    state->update();
    collision_detection::CollisionResult res;
    planning_scene->checkCollision(req, res, *state, *collision_matrix);
    if (res.collision)
      return false;
  }
  else 
    // apply approach posture for the end effector (we always apply it here since it could be the case the sampler changes this posture)
    for (std::size_t i = 0 ; i < manipulation_plan->approach_posture_.points.size() ; ++i)
    {
      state->setVariablePositions(manipulation_plan->approach_posture_.joint_names, manipulation_plan->approach_posture_.points[i].positions);
      state->update();
      collision_detection::CollisionResult res;
      planning_scene->checkCollision(req, res, *state, *collision_matrix);
      if (res.collision)
        return false;
    }
  return planning_scene->isStateFeasible(*state);
}
}

bool pick_place::ReachableAndValidPoseFilter::isEndEffectorFree(const ManipulationPlanPtr &plan, robot_state::RobotState &token_state) const
{
  tf::poseMsgToEigen(plan->goal_pose_.pose, plan->transformed_goal_pose_);
  plan->transformed_goal_pose_ = planning_scene_->getFrameTransform(token_state, plan->goal_pose_.header.frame_id) * plan->transformed_goal_pose_;
  token_state.updateStateWithLinkAt(plan->shared_data_->ik_link_, plan->transformed_goal_pose_);
  collision_detection::CollisionRequest req;
  req.verbose = verbose_;
  collision_detection::CollisionResult res;
  req.group_name = plan->shared_data_->end_effector_group_->getName();
  planning_scene_->checkCollision(req, res, token_state, *collision_matrix_);
  return res.collision == false;
}

bool pick_place::ReachableAndValidPoseFilter::evaluate(const ManipulationPlanPtr &plan) const
{
  // initialize with scene state
  robot_state::RobotStatePtr token_state(new robot_state::RobotState(planning_scene_->getCurrentState()));
  if (isEndEffectorFree(plan, *token_state))
  {
    // update the goal pose message if anything has changed; this is because the name of the frame in the input goal pose
    // can be that of objects in the collision world but most components are unaware of those transforms,
    // so we convert to a frame that is certainly known
    if (robot_state::Transforms::sameFrame(planning_scene_->getPlanningFrame(), plan->goal_pose_.header.frame_id))
    {
      tf::poseEigenToMsg(plan->transformed_goal_pose_, plan->goal_pose_.pose);
      plan->goal_pose_.header.frame_id = planning_scene_->getPlanningFrame();
    }

    // convert the pose we want to reach to a set of constraints
    // lziegler: plan->goal_constraints_ = kinematic_constraints::constructGoalConstraints(plan->shared_data_->ik_link_->getName(), plan->goal_pose_);
    plan->goal_constraints_ = kinematic_constraints::constructGoalConstraints(plan->shared_data_->ik_link_->getName(), plan->goal_pose_, position_tolerance_, orientation_tolerance_);

    const std::string &planning_group = plan->shared_data_->planning_group_->getName();

    ROS_INFO_STREAM("Planning group: " << planning_group);
    // construct a sampler for the specified constraints; this can end up calling just IK, but it is more general
    // and allows for robot-specific samplers, producing samples that also change the base position if needed, etc
    plan->goal_sampler_ = constraints_sampler_manager_->selectSampler(planning_scene_, planning_group, plan->goal_constraints_);
    if (plan->goal_sampler_)
    {
      plan->goal_sampler_->setGroupStateValidityCallback(boost::bind(&isStateCollisionFree, planning_scene_.get(), collision_matrix_.get(),
                                                                     verbose_, plan.get(), _1, _2, _3));
      plan->goal_sampler_->setVerbose(verbose_);
      if (plan->goal_sampler_->sample(*token_state, plan->shared_data_->max_goal_sampling_attempts_))
      {
        plan->possible_goal_states_.push_back(token_state);
        return true;
      }
      else
        if (verbose_)
          ROS_INFO_NAMED("manipulation", "Sampler failed to produce a state");
    }
    else
      ROS_ERROR_THROTTLE_NAMED(1, "manipulation", "No sampler was constructed");
  }
  plan->error_code_.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
  return false;
}

