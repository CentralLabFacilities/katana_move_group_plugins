#include <moveit/katana_pick_place/pick_place_params.h>
#include <dynamic_reconfigure/server.h>
#include <moveit_ros_manipulation/PickPlaceDynamicReconfigureConfig.h>

namespace pick_place
{

namespace
{
using namespace moveit_ros_manipulation;

class DynamicReconfigureImpl
{
public:

  DynamicReconfigureImpl() : dynamic_reconfigure_server_(ros::NodeHandle("~/pick_place"))
  {
    dynamic_reconfigure_server_.setCallback(boost::bind(&DynamicReconfigureImpl::dynamicReconfigureCallback, this, _1, _2));
  }

  const PickPlaceParams& getParams() const
  {
    return params_;
  }
  
private:
  PickPlaceParams params_;
  
  void dynamicReconfigureCallback(PickPlaceDynamicReconfigureConfig &config, uint32_t level)
  {
    params_.max_goal_count_ = config.max_attempted_states_per_pose;
    params_.max_fail_ = config.max_consecutive_fail_attempts;
    params_.max_step_ = config.cartesian_motion_step_size;
    params_.jump_factor_ = config.jump_factor;
  }

  dynamic_reconfigure::Server<PickPlaceDynamicReconfigureConfig> dynamic_reconfigure_server_;
};

}
}

pick_place::PickPlaceParams::PickPlaceParams() : max_goal_count_(5),
						 max_fail_(3),
						 max_step_(0.02),
						 jump_factor_(2.0)
{
}

const pick_place::PickPlaceParams& pick_place::GetGlobalPickPlaceParams() {
  static DynamicReconfigureImpl PICK_PLACE_PARAMS;
  return PICK_PLACE_PARAMS.getParams();
}
