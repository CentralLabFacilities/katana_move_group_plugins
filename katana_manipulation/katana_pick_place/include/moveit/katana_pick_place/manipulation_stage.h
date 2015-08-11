#ifndef MOVEIT_PICK_PLACE_MANIPULATION_STAGE_
#define MOVEIT_PICK_PLACE_MANIPULATION_STAGE_

#include <moveit/katana_pick_place/manipulation_plan.h>
#include <string>

namespace pick_place
{

class ManipulationStage
{
public:

  ManipulationStage(const std::string &name) :
    name_(name),
    signal_stop_(false),
    verbose_(false)
  {
  }

  virtual ~ManipulationStage()
  {
  }

  const std::string& getName() const
  {
    return name_;
  }

  void setVerbose(bool flag)
  {
    verbose_ = flag;
  }

  virtual void resetStopSignal()
  {
    signal_stop_ = false;
  }

  virtual void signalStop()
  {
    signal_stop_ = true;
  }

  virtual bool evaluate(const ManipulationPlanPtr &plan) const = 0;

protected:

  std::string name_;
  bool signal_stop_;
  bool verbose_;
};

typedef boost::shared_ptr<ManipulationStage> ManipulationStagePtr;
typedef boost::shared_ptr<const ManipulationStage> ManipulationStageConstPtr;

}

#endif
