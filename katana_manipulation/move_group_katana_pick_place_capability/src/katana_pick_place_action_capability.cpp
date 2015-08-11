#include "katana_pick_place_action_capability.h"
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/move_group_katana_pick_place_capability/capability_names.h>

#include <manipulation_msgs/GraspPlanning.h>
#include <eigen_conversions/eigen_msg.h>

move_group::MoveGroupKatanaPickPlaceAction::MoveGroupKatanaPickPlaceAction() :
  MoveGroupCapability("KatanaPickPlaceAction"),
  pickup_state_(IDLE)
{
}

void move_group::MoveGroupKatanaPickPlaceAction::initialize()
{
  pick_place_.reset(new pick_place::PickPlace(context_->planning_pipeline_));
  pick_place_->displayComputedMotionPlans(true);

  ROS_INFO_STREAM_NAMED("manipulation", "Pick&place debug mode: " << context_->debug_);
  if (context_->debug_ || true)
    pick_place_->displayProcessedGrasps(true);

  // start the pickup action server
  pickup_action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::PickupAction>(root_node_handle_, KATANA_PICKUP_ACTION,
                                                                                           boost::bind(&MoveGroupKatanaPickPlaceAction::executePickupCallback, this, _1), false));
  pickup_action_server_->registerPreemptCallback(boost::bind(&MoveGroupKatanaPickPlaceAction::preemptPickupCallback, this));
  pickup_action_server_->start();

  // start the place action server
  place_action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::PlaceAction>(root_node_handle_, KATANA_PLACE_ACTION,
                                                                                         boost::bind(&MoveGroupKatanaPickPlaceAction::executePlaceCallback, this, _1), false));
  place_action_server_->registerPreemptCallback(boost::bind(&MoveGroupKatanaPickPlaceAction::preemptPlaceCallback, this));
  place_action_server_->start();

  grasp_planning_service_ = root_node_handle_.serviceClient<manipulation_msgs::GraspPlanning>("database_grasp_planning");
}

void move_group::MoveGroupKatanaPickPlaceAction::startPickupExecutionCallback()
{
  setPickupState(MONITOR);
}

void move_group::MoveGroupKatanaPickPlaceAction::startPickupLookCallback()
{
  setPickupState(LOOK);
}

void move_group::MoveGroupKatanaPickPlaceAction::startPlaceExecutionCallback()
{
  setPlaceState(MONITOR);
}

void move_group::MoveGroupKatanaPickPlaceAction::startPlaceLookCallback()
{
  setPlaceState(LOOK);
}

void move_group::MoveGroupKatanaPickPlaceAction::executePickupCallback_PlanOnly(const moveit_msgs::PickupGoalConstPtr& goal, moveit_msgs::PickupResult &action_res)
{
  pick_place::PickPlanPtr plan;
  try
  {
    planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);
    plan = pick_place_->planPick(ps, *goal);
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR_NAMED("manipulation", "Pick&place threw an exception: %s", ex.what());
  }
  catch(...)
  {
    ROS_ERROR_NAMED("manipulation", "Pick&place threw an exception");
  }

  if (plan)
  {
    const std::vector<pick_place::ManipulationPlanPtr> &success = plan->getSuccessfulManipulationPlans();
    if (success.empty())
    {
      action_res.error_code = plan->getErrorCode();
    }
    else
    {
      const pick_place::ManipulationPlanPtr &result = success.back();
      convertToMsg(result->trajectories_, action_res.trajectory_start, action_res.trajectory_stages);
      action_res.trajectory_descriptions.resize(result->trajectories_.size());
      for (std::size_t i = 0 ; i < result->trajectories_.size() ; ++i)
        action_res.trajectory_descriptions[i] = result->trajectories_[i].description_;
      if (result->id_ < goal->possible_grasps.size())
        action_res.grasp = goal->possible_grasps[result->id_];
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
  }
  else
  {
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
}

void move_group::MoveGroupKatanaPickPlaceAction::executePlaceCallback_PlanOnly(const moveit_msgs::PlaceGoalConstPtr& goal, moveit_msgs::PlaceResult &action_res)
{
  pick_place::PlacePlanPtr plan;
  try
  {
    planning_scene_monitor::LockedPlanningSceneRO ps(context_->planning_scene_monitor_);
    plan = pick_place_->planPlace(ps, *goal);
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR_NAMED("manipulation", "Pick&place threw an exception: %s", ex.what());
  }
  catch(...)
  {
    ROS_ERROR_NAMED("manipulation", "Pick&place threw an exception");
  }

  if (plan)
  {
    const std::vector<pick_place::ManipulationPlanPtr> &success = plan->getSuccessfulManipulationPlans();
    if (success.empty())
    {
      action_res.error_code = plan->getErrorCode();
    }
    else
    {
      const pick_place::ManipulationPlanPtr &result = success.back();
      convertToMsg(result->trajectories_, action_res.trajectory_start, action_res.trajectory_stages);
      action_res.trajectory_descriptions.resize(result->trajectories_.size());
      for (std::size_t i = 0 ; i < result->trajectories_.size() ; ++i)
        action_res.trajectory_descriptions[i] = result->trajectories_[i].description_;
      if (result->id_ < goal->place_locations.size())
        action_res.place_location = goal->place_locations[result->id_];
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
  }
  else
  {
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
}

bool move_group::MoveGroupKatanaPickPlaceAction::planUsingPickPlace_Pickup(const moveit_msgs::PickupGoal& goal, moveit_msgs::PickupResult *action_res,
                                                                     plan_execution::ExecutableMotionPlan &plan)
{
  setPickupState(PLANNING);

  planning_scene_monitor::LockedPlanningSceneRO ps(plan.planning_scene_monitor_);

  pick_place::PickPlanPtr pick_plan;
  try
  {
    pick_plan = pick_place_->planPick(plan.planning_scene_, goal);
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR_NAMED("manipulation", "Pick&place threw an exception: %s", ex.what());
  }
  catch(...)
  {
    ROS_ERROR_NAMED("manipulation", "Pick&place threw an exception");
  }

  if (pick_plan)
  {
    const std::vector<pick_place::ManipulationPlanPtr> &success = pick_plan->getSuccessfulManipulationPlans();
    if (success.empty())
    {
      plan.error_code_ = pick_plan->getErrorCode();
    }
    else
    {
      const pick_place::ManipulationPlanPtr &result = success.back();
      plan.plan_components_ = result->trajectories_;
      if (result->id_ < goal.possible_grasps.size())
        action_res->grasp = goal.possible_grasps[result->id_];
      plan.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
  }
  else
  {
    plan.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }

  return plan.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
}

bool move_group::MoveGroupKatanaPickPlaceAction::planUsingPickPlace_Place(const moveit_msgs::PlaceGoal& goal, moveit_msgs::PlaceResult *action_res,
                                                                    plan_execution::ExecutableMotionPlan &plan)
{
  setPlaceState(PLANNING);

  planning_scene_monitor::LockedPlanningSceneRO ps(plan.planning_scene_monitor_);

  pick_place::PlacePlanPtr place_plan;
  try
  {
    place_plan = pick_place_->planPlace(plan.planning_scene_, goal);
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR_NAMED("manipulation", "Pick&place threw an exception: %s", ex.what());
  }
  catch(...)
  {
    ROS_ERROR_NAMED("manipulation", "Pick&place threw an exception");
  }

  if (place_plan)
  {
    const std::vector<pick_place::ManipulationPlanPtr> &success = place_plan->getSuccessfulManipulationPlans();
    if (success.empty())
    {
      plan.error_code_ = place_plan->getErrorCode();
    }
    else
    {
      const pick_place::ManipulationPlanPtr &result = success.back();
      plan.plan_components_ = result->trajectories_;
      if (result->id_ < goal.place_locations.size())
        action_res->place_location = goal.place_locations[result->id_];
      plan.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
  }
  else
  {
    plan.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }

  return plan.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
}

void move_group::MoveGroupKatanaPickPlaceAction::executePickupCallback_PlanAndExecute(const moveit_msgs::PickupGoalConstPtr& goal, moveit_msgs::PickupResult &action_res)
{
  plan_execution::PlanExecution::Options opt;

  opt.replan_ = goal->planning_options.replan;
  opt.replan_attempts_ = goal->planning_options.replan_attempts;
  opt.replan_delay_ = goal->planning_options.replan_delay;
  opt.before_execution_callback_ = boost::bind(&MoveGroupKatanaPickPlaceAction::startPickupExecutionCallback, this);

  opt.plan_callback_ = boost::bind(&MoveGroupKatanaPickPlaceAction::planUsingPickPlace_Pickup, this, boost::cref(*goal), &action_res, _1);
  if (goal->planning_options.look_around && context_->plan_with_sensing_)
  {
    opt.plan_callback_ = boost::bind(&plan_execution::PlanWithSensing::computePlan, context_->plan_with_sensing_.get(), _1, opt.plan_callback_,
                                     goal->planning_options.look_around_attempts, goal->planning_options.max_safe_execution_cost);
    context_->plan_with_sensing_->setBeforeLookCallback(boost::bind(&MoveGroupKatanaPickPlaceAction::startPickupLookCallback, this));
  }

  plan_execution::ExecutableMotionPlan plan;
  context_->plan_execution_->planAndExecute(plan, goal->planning_options.planning_scene_diff, opt);

  convertToMsg(plan.plan_components_, action_res.trajectory_start, action_res.trajectory_stages);
  action_res.trajectory_descriptions.resize(plan.plan_components_.size());
  for (std::size_t i = 0 ; i < plan.plan_components_.size() ; ++i)
    action_res.trajectory_descriptions[i] = plan.plan_components_[i].description_; 
  action_res.error_code = plan.error_code_;
}

void move_group::MoveGroupKatanaPickPlaceAction::executePlaceCallback_PlanAndExecute(const moveit_msgs::PlaceGoalConstPtr& goal, moveit_msgs::PlaceResult &action_res)
{
  plan_execution::PlanExecution::Options opt;

  opt.replan_ = goal->planning_options.replan;
  opt.replan_attempts_ = goal->planning_options.replan_attempts;
  opt.replan_delay_ = goal->planning_options.replan_delay;
  opt.before_execution_callback_ = boost::bind(&MoveGroupKatanaPickPlaceAction::startPlaceExecutionCallback, this);
  opt.plan_callback_ = boost::bind(&MoveGroupKatanaPickPlaceAction::planUsingPickPlace_Place, this, boost::cref(*goal), &action_res, _1);
  if (goal->planning_options.look_around && context_->plan_with_sensing_)
  {
    opt.plan_callback_ = boost::bind(&plan_execution::PlanWithSensing::computePlan, context_->plan_with_sensing_.get(), _1, opt.plan_callback_,
                                     goal->planning_options.look_around_attempts, goal->planning_options.max_safe_execution_cost);
    context_->plan_with_sensing_->setBeforeLookCallback(boost::bind(&MoveGroupKatanaPickPlaceAction::startPlaceLookCallback, this));
  }

  plan_execution::ExecutableMotionPlan plan;
  context_->plan_execution_->planAndExecute(plan, goal->planning_options.planning_scene_diff, opt);

  convertToMsg(plan.plan_components_, action_res.trajectory_start, action_res.trajectory_stages);
  action_res.trajectory_descriptions.resize(plan.plan_components_.size());
  for (std::size_t i = 0 ; i < plan.plan_components_.size() ; ++i)
    action_res.trajectory_descriptions[i] = plan.plan_components_[i].description_;
  action_res.error_code = plan.error_code_;
}

void move_group::MoveGroupKatanaPickPlaceAction::executePickupCallback(const moveit_msgs::PickupGoalConstPtr& input_goal)
{
  setPickupState(PLANNING);

  context_->planning_scene_monitor_->updateFrameTransforms();

  moveit_msgs::PickupGoalConstPtr goal;
  if (input_goal->possible_grasps.empty())
  {
    moveit_msgs::PickupGoal* copy(new moveit_msgs::PickupGoal(*input_goal));
    goal.reset(copy);
    fillGrasps(*copy);
  }
  else
    goal = input_goal;

  moveit_msgs::PickupResult action_res;

  if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
  {
    if (!goal->planning_options.plan_only)
      ROS_WARN_NAMED("manipulation", "This instance of MoveGroup is not allowed to execute trajectories but the pick goal request has plan_only set to false. Only a motion plan will be computed anyway.");
    executePickupCallback_PlanOnly(goal, action_res);
  }
  else
    executePickupCallback_PlanAndExecute(goal, action_res);

  bool planned_trajectory_empty = action_res.trajectory_stages.empty();
  std::string response = getActionResultString(action_res.error_code, planned_trajectory_empty, goal->planning_options.plan_only);
  if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    pickup_action_server_->setSucceeded(action_res, response);
  else
  {
    if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
      pickup_action_server_->setPreempted(action_res, response);
    else
      pickup_action_server_->setAborted(action_res, response);
  }

  setPickupState(IDLE);
}

void move_group::MoveGroupKatanaPickPlaceAction::executePlaceCallback(const moveit_msgs::PlaceGoalConstPtr& goal)
{
  setPlaceState(PLANNING);

  context_->planning_scene_monitor_->updateFrameTransforms();

  moveit_msgs::PlaceResult action_res;

  if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
  {
    if (!goal->planning_options.plan_only)
      ROS_WARN_NAMED("manipulation", "This instance of MoveGroup is not allowed to execute trajectories but the place goal request has plan_only set to false. Only a motion plan will be computed anyway.");
    executePlaceCallback_PlanOnly(goal, action_res);
  }
  else
    executePlaceCallback_PlanAndExecute(goal, action_res);

  bool planned_trajectory_empty = action_res.trajectory_stages.empty();
  std::string response = getActionResultString(action_res.error_code, planned_trajectory_empty, goal->planning_options.plan_only);
  if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    place_action_server_->setSucceeded(action_res, response);
  else
  {
    if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
      place_action_server_->setPreempted(action_res, response);
    else
      place_action_server_->setAborted(action_res, response);
  }

  setPlaceState(IDLE);
}

void move_group::MoveGroupKatanaPickPlaceAction::preemptPickupCallback()
{
}

void move_group::MoveGroupKatanaPickPlaceAction::preemptPlaceCallback()
{
}

void move_group::MoveGroupKatanaPickPlaceAction::setPickupState(MoveGroupState state)
{
  pickup_state_ = state;
  pickup_feedback_.state = stateToStr(state);
  pickup_action_server_->publishFeedback(pickup_feedback_);
}

void move_group::MoveGroupKatanaPickPlaceAction::setPlaceState(MoveGroupState state)
{
  place_state_ = state;
  place_feedback_.state = stateToStr(state);
  place_action_server_->publishFeedback(place_feedback_);
}

void move_group::MoveGroupKatanaPickPlaceAction::fillGrasps(moveit_msgs::PickupGoal& goal)
{
  planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);

  if (grasp_planning_service_ && lscene->hasObjectType(goal.target_name) && !lscene->getObjectType(goal.target_name).key.empty())
  {
    manipulation_msgs::GraspPlanning::Request request;
    manipulation_msgs::GraspPlanning::Response response;
    bool valid = true;

    collision_detection::World::ObjectConstPtr object = lscene->getWorld()->getObject(goal.target_name);
    if (object && !object->shape_poses_.empty())
    {
      request.arm_name = goal.group_name;
      request.target.reference_frame_id = lscene->getPlanningFrame();

      household_objects_database_msgs::DatabaseModelPose dbp;
      dbp.pose.header.frame_id = lscene->getPlanningFrame();
      dbp.pose.header.stamp = ros::Time::now();
      tf::poseEigenToMsg(object->shape_poses_[0], dbp.pose.pose);
      dbp.type = lscene->getObjectType(goal.target_name);
      try
      {
        dbp.model_id = boost::lexical_cast<int>(dbp.type.key);
        ROS_DEBUG_NAMED("manipulation", "Asking database for grasps for '%s' with model id: %d", dbp.type.key.c_str(), dbp.model_id);
        request.target.potential_models.push_back(dbp);
      }
      catch (boost::bad_lexical_cast &)
      {
        valid = false;
        ROS_ERROR_NAMED("manipulation", "Expected an integer object id, not '%s'", dbp.type.key.c_str());
      }
    }
    else
    {
      valid = false;
      ROS_ERROR_NAMED("manipulation", "Object has no geometry");
    }

    if (valid)
    {
      ROS_DEBUG_NAMED("manipulation", "Calling grasp planner...");
      if (grasp_planning_service_.call(request, response))
      {
        goal.possible_grasps.resize(response.grasps.size());
        for (std::size_t i = 0 ; i < response.grasps.size() ; ++i)
        {
          // construct a moveit grasp from a grasp planner grasp
          goal.possible_grasps[i].id = response.grasps[i].id;

          goal.possible_grasps[i].pre_grasp_posture.header = response.grasps[i].pre_grasp_posture.header;
          goal.possible_grasps[i].pre_grasp_posture.joint_names = response.grasps[i].pre_grasp_posture.name;
          goal.possible_grasps[i].pre_grasp_posture.points.resize(1);
          goal.possible_grasps[i].pre_grasp_posture.points[0].positions = response.grasps[i].pre_grasp_posture.position;
          goal.possible_grasps[i].pre_grasp_posture.points[0].velocities = response.grasps[i].pre_grasp_posture.velocity;
          goal.possible_grasps[i].pre_grasp_posture.points[0].effort = response.grasps[i].pre_grasp_posture.effort;

          goal.possible_grasps[i].grasp_posture.header = response.grasps[i].grasp_posture.header;
          goal.possible_grasps[i].grasp_posture.joint_names = response.grasps[i].grasp_posture.name;
          goal.possible_grasps[i].grasp_posture.points.resize(1);
          goal.possible_grasps[i].grasp_posture.points[0].positions = response.grasps[i].grasp_posture.position;
          goal.possible_grasps[i].grasp_posture.points[0].velocities = response.grasps[i].grasp_posture.velocity;
          goal.possible_grasps[i].grasp_posture.points[0].effort = response.grasps[i].grasp_posture.effort;

          goal.possible_grasps[i].grasp_pose = response.grasps[i].grasp_pose;
          goal.possible_grasps[i].grasp_quality = response.grasps[i].grasp_quality;

          goal.possible_grasps[i].pre_grasp_approach.direction = response.grasps[i].approach.direction;
          goal.possible_grasps[i].pre_grasp_approach.desired_distance = response.grasps[i].approach.desired_distance;
          goal.possible_grasps[i].pre_grasp_approach.min_distance = response.grasps[i].approach.min_distance;

          //  here we hard-code in the decision that after grasping, the object is lifted "up" (against gravity)
          //  we expect that in the planning frame Z points up
          goal.possible_grasps[i].post_grasp_retreat.direction.vector.x = 0.0;
          goal.possible_grasps[i].post_grasp_retreat.direction.vector.y = 0.0;
          goal.possible_grasps[i].post_grasp_retreat.direction.vector.z = 1.0;
          goal.possible_grasps[i].post_grasp_retreat.desired_distance = 0.1;
          goal.possible_grasps[i].post_grasp_retreat.min_distance = 0.0;
          goal.possible_grasps[i].post_grasp_retreat.direction.header.frame_id = lscene->getPlanningFrame();

          goal.possible_grasps[i].post_place_retreat.direction = response.grasps[i].retreat.direction;
          goal.possible_grasps[i].post_place_retreat.desired_distance = response.grasps[i].retreat.desired_distance;
          goal.possible_grasps[i].post_place_retreat.min_distance = response.grasps[i].retreat.min_distance;

          goal.possible_grasps[i].max_contact_force = response.grasps[i].max_contact_force;
          goal.possible_grasps[i].allowed_touch_objects = response.grasps[i].allowed_touch_objects;
        }
      }
    }
  }

  if (goal.possible_grasps.empty())
  {
    ROS_DEBUG_NAMED("manipulation", "Using default grasp poses");
    goal.minimize_object_distance = true;

    // add a number of default grasp points
    // \todo add more!
    moveit_msgs::Grasp g;
    g.grasp_pose.header.frame_id = goal.target_name;
    g.grasp_pose.pose.position.x = -0.2;
    g.grasp_pose.pose.position.y = 0.0;
    g.grasp_pose.pose.position.z = 0.0;
    g.grasp_pose.pose.orientation.x = 0.0;
    g.grasp_pose.pose.orientation.y = 0.0;
    g.grasp_pose.pose.orientation.z = 0.0;
    g.grasp_pose.pose.orientation.w = 1.0;

    g.pre_grasp_approach.direction.header.frame_id = lscene->getPlanningFrame();
    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.pre_grasp_approach.min_distance = 0.1;
    g.pre_grasp_approach.desired_distance = 0.2;

    g.post_grasp_retreat.direction.header.frame_id = lscene->getPlanningFrame();
    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.2;

    if (lscene->getRobotModel()->hasEndEffector(goal.end_effector))
    {
      g.pre_grasp_posture.joint_names = lscene->getRobotModel()->getEndEffector(goal.end_effector)->getJointModelNames();
      g.pre_grasp_posture.points.resize(1);
      g.pre_grasp_posture.points[0].positions.resize(g.pre_grasp_posture.joint_names.size(), std::numeric_limits<double>::max());

      g.grasp_posture.joint_names = g.pre_grasp_posture.joint_names;
      g.grasp_posture.points.resize(1);
      g.grasp_posture.points[0].positions.resize(g.grasp_posture.joint_names.size(), -std::numeric_limits<double>::max());
    }
    goal.possible_grasps.push_back(g);
  }
}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupKatanaPickPlaceAction, move_group::MoveGroupCapability)
