#ifndef MOVEIT_PICK_PLACE_PICK_PLACE_
#define MOVEIT_PICK_PLACE_PICK_PLACE_

#include <moveit/katana_pick_place/manipulation_pipeline.h>
#include <moveit/katana_pick_place/pick_place_params.h>
#include <moveit/constraint_sampler_manager_loader/constraint_sampler_manager_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <boost/noncopyable.hpp>
#include <boost/enable_shared_from_this.hpp>

namespace pick_place
{

class PickPlace;
typedef boost::shared_ptr<PickPlace> PickPlacePtr;
typedef boost::shared_ptr<const PickPlace> PickPlaceConstPtr;

class PickPlacePlanBase
{
public:

  PickPlacePlanBase(const PickPlaceConstPtr &pick_place, const std::string &name);
  ~PickPlacePlanBase();

  const std::vector<ManipulationPlanPtr>& getSuccessfulManipulationPlans() const
  {
    return pipeline_.getSuccessfulManipulationPlans();
  }
  const std::vector<ManipulationPlanPtr>& getFailedManipulationPlans() const
  {
    return pipeline_.getFailedManipulationPlans();
  }

  const moveit_msgs::MoveItErrorCodes& getErrorCode() const
  {
    return error_code_;
  }

protected:

  void initialize();
  void waitForPipeline(const ros::WallTime &endtime);
  void foundSolution();
  void emptyQueue();

  PickPlaceConstPtr pick_place_;
  ManipulationPipeline pipeline_;

  double last_plan_time_;
  bool done_;
  bool pushed_all_poses_;
  boost::condition_variable done_condition_;
  boost::mutex done_mutex_;
  moveit_msgs::MoveItErrorCodes error_code_;
};

class PickPlan : public PickPlacePlanBase
{
public:

  PickPlan(const PickPlaceConstPtr &pick_place);
  bool plan(const planning_scene::PlanningSceneConstPtr &planning_scene, const moveit_msgs::PickupGoal &goal);
};

typedef boost::shared_ptr<PickPlan> PickPlanPtr;
typedef boost::shared_ptr<const PickPlan> PickPlanConstPtr;

class PlacePlan : public PickPlacePlanBase
{
public:

  PlacePlan(const PickPlaceConstPtr &pick_place);
  bool plan(const planning_scene::PlanningSceneConstPtr &planning_scene, const moveit_msgs::PlaceGoal &goal);
};

typedef boost::shared_ptr<PlacePlan> PlacePlanPtr;
typedef boost::shared_ptr<const PlacePlan> PlacePlanConstPtr;

class PickPlace : private boost::noncopyable,
                  public boost::enable_shared_from_this<PickPlace>
{
public:

  static const std::string DISPLAY_PATH_TOPIC;
  static const std::string DISPLAY_GRASP_TOPIC;

  // the amount of time (maximum) to wait for achieving a grasp posture
  static const double DEFAULT_GRASP_POSTURE_COMPLETION_DURATION; // seconds

  PickPlace(const planning_pipeline::PlanningPipelinePtr &planning_pipeline);

  const constraint_samplers::ConstraintSamplerManagerPtr& getConstraintsSamplerManager() const
  {
    return constraint_sampler_manager_loader_->getConstraintSamplerManager();
  }

  const planning_pipeline::PlanningPipelinePtr& getPlanningPipeline() const
  {
    return planning_pipeline_;
  }

  const robot_model::RobotModelConstPtr& getRobotModel() const
  {
    return planning_pipeline_->getRobotModel();
  }

  /** \brief Plan the sequence of motions that perform a pickup action */
  PickPlanPtr planPick(const planning_scene::PlanningSceneConstPtr &planning_scene, const moveit_msgs::PickupGoal &goal) const;

  /** \brief Plan the sequence of motions that perform a placement action */
  PlacePlanPtr planPlace(const planning_scene::PlanningSceneConstPtr &planning_scene, const moveit_msgs::PlaceGoal &goal) const;

  void displayComputedMotionPlans(bool flag);
  void displayProcessedGrasps(bool flag);

  void visualizePlan(const ManipulationPlanPtr &plan) const;

  void visualizeGrasp(const ManipulationPlanPtr &plan) const;

  void visualizeGrasps(const std::vector<ManipulationPlanPtr>& plans) const;

private:

  ros::NodeHandle nh_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;
  bool display_computed_motion_plans_;
  bool display_grasps_;
  ros::Publisher display_path_publisher_;
  ros::Publisher grasps_publisher_;

  constraint_sampler_manager_loader::ConstraintSamplerManagerLoaderPtr constraint_sampler_manager_loader_;
};

}

#endif
