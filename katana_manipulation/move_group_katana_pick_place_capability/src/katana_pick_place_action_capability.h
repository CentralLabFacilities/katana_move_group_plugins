#ifndef MOVEIT_MOVE_GROUP_KATANA_PICK_PLACE_ACTION_CAPABILITY_
#define MOVEIT_MOVE_GROUP_KATANA_PICK_PLACE_ACTION_CAPABILITY_

#include <moveit/move_group/move_group_capability.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/pick_place/pick_place.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>

namespace move_group
{

class MoveGroupKatanaPickPlaceAction : public MoveGroupCapability
{
public:

	MoveGroupKatanaPickPlaceAction();
  virtual void initialize();

private:

  void executePickupCallback(const moveit_msgs::PickupGoalConstPtr& goal);
  void executePlaceCallback(const moveit_msgs::PlaceGoalConstPtr& goal);

  void executePickupCallback_PlanOnly(const moveit_msgs::PickupGoalConstPtr& goal, moveit_msgs::PickupResult &action_res);
  void executePickupCallback_PlanAndExecute(const moveit_msgs::PickupGoalConstPtr& goal, moveit_msgs::PickupResult &action_res);

  void executePlaceCallback_PlanOnly(const moveit_msgs::PlaceGoalConstPtr& goal, moveit_msgs::PlaceResult &action_res);
  void executePlaceCallback_PlanAndExecute(const moveit_msgs::PlaceGoalConstPtr& goal, moveit_msgs::PlaceResult &action_res);

  bool planUsingPickPlace_Pickup(const moveit_msgs::PickupGoal& goal, moveit_msgs::PickupResult *action_res, plan_execution::ExecutableMotionPlan &plan);
  bool planUsingPickPlace_Place(const moveit_msgs::PlaceGoal& goal, moveit_msgs::PlaceResult *action_res, plan_execution::ExecutableMotionPlan &plan);

  void preemptPickupCallback();
  void preemptPlaceCallback();

  void startPickupLookCallback();
  void startPickupExecutionCallback();

  void startPlaceLookCallback();
  void startPlaceExecutionCallback();

  void setPickupState(MoveGroupState state);
  void setPlaceState(MoveGroupState state);

  void fillGrasps(moveit_msgs::PickupGoal& goal);

  pick_place::PickPlacePtr pick_place_;

  boost::scoped_ptr<actionlib::SimpleActionServer<moveit_msgs::PickupAction> > pickup_action_server_;
  moveit_msgs::PickupFeedback pickup_feedback_;

  boost::scoped_ptr<actionlib::SimpleActionServer<moveit_msgs::PlaceAction> > place_action_server_;
  moveit_msgs::PlaceFeedback place_feedback_;

  boost::scoped_ptr<moveit_msgs::AttachedCollisionObject> diff_attached_object_;

  MoveGroupState pickup_state_;
  MoveGroupState place_state_;

  ros::ServiceClient grasp_planning_service_;

};

}

#endif
