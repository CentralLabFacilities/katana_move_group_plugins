#ifndef MOVEIT_PICK_PLACE_REACHABLE_VALID_POSE_FILTER_
#define MOVEIT_PICK_PLACE_REACHABLE_VALID_POSE_FILTER_

#include <moveit/katana_pick_place/manipulation_stage.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/planning_scene/planning_scene.h>

namespace pick_place
{

class ReachableAndValidPoseFilter : public ManipulationStage
{
public:

  ReachableAndValidPoseFilter(const planning_scene::PlanningSceneConstPtr &scene,
                              const collision_detection::AllowedCollisionMatrixConstPtr &collision_matrix,
                              const constraint_samplers::ConstraintSamplerManagerPtr &constraints_sampler_manager);

  virtual bool evaluate(const ManipulationPlanPtr &plan) const;

private:
  
  bool isEndEffectorFree(const ManipulationPlanPtr &plan, robot_state::RobotState &token_state) const;

  planning_scene::PlanningSceneConstPtr planning_scene_;
  collision_detection::AllowedCollisionMatrixConstPtr collision_matrix_;
  constraint_samplers::ConstraintSamplerManagerPtr constraints_sampler_manager_;

  double position_tolerance_;
  double orientation_tolerance_;
};

}

#endif
