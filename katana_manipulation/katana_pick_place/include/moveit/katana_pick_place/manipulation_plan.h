#ifndef MOVEIT_PICK_PLACE_MANIPULATION_PLAN_
#define MOVEIT_PICK_PLACE_MANIPULATION_PLAN_

#include <boost/shared_ptr.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/constraint_samplers/constraint_sampler.h>
#include <moveit/plan_execution/plan_representation.h>
#include <moveit_msgs/GripperTranslation.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/Constraints.h>
#include <string>
#include <vector>

namespace pick_place
{

struct ManipulationPlanSharedData
{
  ManipulationPlanSharedData() 
    : planning_group_(NULL)
    , end_effector_group_(NULL)
    , ik_link_(NULL)
    , max_goal_sampling_attempts_(0)
    , minimize_object_distance_(false)
  {
  }
  
  const robot_model::JointModelGroup *planning_group_;
  const robot_model::JointModelGroup *end_effector_group_;
  const robot_model::LinkModel *ik_link_;

  unsigned int max_goal_sampling_attempts_;

  std::string planner_id_;

  bool minimize_object_distance_;

  moveit_msgs::Constraints path_constraints_;

  moveit_msgs::AttachedCollisionObject diff_attached_object_;

  ros::WallTime timeout_;
};

typedef boost::shared_ptr<ManipulationPlanSharedData> ManipulationPlanSharedDataPtr;
typedef boost::shared_ptr<const ManipulationPlanSharedData> ManipulationPlanSharedDataConstPtr;

struct ManipulationPlan
{
  ManipulationPlan(const ManipulationPlanSharedDataConstPtr &shared_data) :
    shared_data_(shared_data),
    processing_stage_(0)
  {
  }

  /// Restore this plan to a state that makes it look like it never was processed by the manipulation pipeline
  void clear()
  {
    goal_sampler_.reset();
    trajectories_.clear();
    approach_state_.reset();
    possible_goal_states_.clear();
    processing_stage_ = 0;
  }

  // Shared data between manipulation plans (set at initialization)
  ManipulationPlanSharedDataConstPtr shared_data_;

  // the approach motion towards the goal
  moveit_msgs::GripperTranslation approach_;

  // the retreat motion away from the goal
  moveit_msgs::GripperTranslation retreat_;

  // the kinematic configuration of the end effector when approaching the goal (an open gripper)
  trajectory_msgs::JointTrajectory approach_posture_;

  // the kinematic configuration of the end effector when retreating from the goal (a closed gripper)
  trajectory_msgs::JointTrajectory retreat_posture_;

  // -------------- computed data --------------------------
  geometry_msgs::PoseStamped goal_pose_;
  Eigen::Affine3d transformed_goal_pose_;

  moveit_msgs::Constraints goal_constraints_;

  // Allows for the sampling of a kineamtic state for a particular group of a robot
  constraint_samplers::ConstraintSamplerPtr goal_sampler_;

  std::vector<robot_state::RobotStatePtr> possible_goal_states_;

  robot_state::RobotStatePtr approach_state_;

  // The sequence of trajectories produced for execution
  std::vector<plan_execution::ExecutableTrajectory> trajectories_;

  // An error code reflecting what went wrong (if anything)
  moveit_msgs::MoveItErrorCodes error_code_;

  // The processing stage that was last working on this plan, or was about to work on this plan
  std::size_t processing_stage_;

  // An id for this plan; this is usually the index of the Grasp / PlaceLocation in the input request
  std::size_t id_;

};

typedef boost::shared_ptr<ManipulationPlan> ManipulationPlanPtr;
typedef boost::shared_ptr<const ManipulationPlan> ManipulationPlanConstPtr;

}

#endif
