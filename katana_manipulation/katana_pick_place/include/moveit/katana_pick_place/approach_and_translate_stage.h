#ifndef MOVEIT_PICK_PLACE_APPROACH_AND_TRANSLATE_STAGE_
#define MOVEIT_PICK_PLACE_APPROACH_AND_TRANSLATE_STAGE_

#include <moveit/katana_pick_place/manipulation_stage.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace pick_place
{

class ApproachAndTranslateStage : public ManipulationStage
{
public:

  ApproachAndTranslateStage(const planning_scene::PlanningSceneConstPtr &scene,
                            const collision_detection::AllowedCollisionMatrixConstPtr &collision_matrix);

  virtual bool evaluate(const ManipulationPlanPtr &plan) const;

private:

  planning_scene::PlanningSceneConstPtr planning_scene_;
  collision_detection::AllowedCollisionMatrixConstPtr collision_matrix_;
  trajectory_processing::IterativeParabolicTimeParameterization time_param_;

  unsigned int max_goal_count_;
  unsigned int max_fail_;
  double max_step_;
  double jump_factor_;

};

}

#endif
