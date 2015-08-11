#ifndef MOVEIT_PICK_PLACE_PICK_PLACE_PARAMS_
#define MOVEIT_PICK_PLACE_PICK_PLACE_PARAMS_

namespace pick_place
{

  struct PickPlaceParams
  {
    PickPlaceParams();

    unsigned int max_goal_count_;
    unsigned int max_fail_;
    double max_step_;
    double jump_factor_;
  };
  
  // Get access to a global variable that contains the pick & place params.
  const PickPlaceParams& GetGlobalPickPlaceParams();
}

#endif
