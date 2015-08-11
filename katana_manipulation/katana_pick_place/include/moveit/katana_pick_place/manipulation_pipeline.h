#ifndef MOVEIT_PICK_PLACE_MANIPULATION_PIPELINE_
#define MOVEIT_PICK_PLACE_MANIPULATION_PIPELINE_

#include <moveit/katana_pick_place/manipulation_stage.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <vector>
#include <deque>

namespace pick_place
{

/** \brief Represent the sequence of steps that are executed for a manipulation plan */
class ManipulationPipeline
{
public:
  
  ManipulationPipeline(const std::string &name, unsigned int nthreads);
  virtual ~ManipulationPipeline();
  
  const std::string& getName() const
  {
    return name_;
  }
  
  void setSolutionCallback(const boost::function<void()> &callback)
  {
    solution_callback_ = callback;
  }
  
  void setEmptyQueueCallback(const boost::function<void()> &callback)
  {
    empty_queue_callback_ = callback;
  }
  
  ManipulationPipeline& addStage(const ManipulationStagePtr &next);
  const ManipulationStagePtr& getFirstStage() const;
  const ManipulationStagePtr& getLastStage() const;
  void reset();
  
  void setVerbose(bool flag);
  
  void signalStop();
  void start();
  void stop();
  
  void push(const ManipulationPlanPtr &grasp);
  void clear();
  
  const std::vector<ManipulationPlanPtr>& getSuccessfulManipulationPlans() const
  {
    return success_;
  }
  
  const std::vector<ManipulationPlanPtr>& getFailedManipulationPlans() const
  {
    return failed_;
  }
  
  void reprocessLastFailure();
  
protected:
  
  void processingThread(unsigned int index);
  
  std::string name_;
  unsigned int nthreads_;
  bool verbose_;
  std::vector<ManipulationStagePtr> stages_;
  
  std::deque<ManipulationPlanPtr> queue_;
  std::vector<ManipulationPlanPtr> success_;
  std::vector<ManipulationPlanPtr> failed_;
  
  std::vector<boost::thread*> processing_threads_;
  boost::condition_variable queue_access_cond_;
  boost::mutex queue_access_lock_;
  boost::mutex result_lock_;
  
  boost::function<void()> solution_callback_;
  boost::function<void()> empty_queue_callback_;
  unsigned int empty_queue_threads_;
  
  bool stop_processing_;
  
};

}

#endif
