#ifndef MOVEIT_PICK_PLACE_PLAN_STAGE_
#define MOVEIT_PICK_PLACE_PLAN_STAGE_

#include <moveit/katana_pick_place/manipulation_stage.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>

namespace pick_place
{

class PlanStage : public ManipulationStage
{
public:

  PlanStage(const planning_scene::PlanningSceneConstPtr &scene,
            const planning_pipeline::PlanningPipelinePtr &planning_pipeline);

  virtual void signalStop();

  virtual bool evaluate(const ManipulationPlanPtr &plan) const;

private:

  planning_scene::PlanningSceneConstPtr planning_scene_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;
};

}

#endif
