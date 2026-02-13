#ifndef _YOUR_PLANNER_H_
#define _YOUR_PLANNER_H_

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#include "RrtConConBase.h"

using namespace ::rl::plan;

/**
*	The implementation of your planner.
*	modify any of the existing methods to improve planning performance.
*/
class YourPlanner : public RrtConConBase
{
public:
  YourPlanner();

  virtual ~YourPlanner();

  virtual ::std::string getName() const;

  bool solve();

  bool useWorkspaceDistance = false;
  bool useKdTree = false;

protected:
  void choose(::rl::math::Vector& chosen);

  RrtConConBase::Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

  RrtConConBase::Vertex extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

  RrtConConBase::Neighbor nearest(const Tree& tree, const ::rl::math::Vector& chosen) override;


private:
  RrtConConBase::Neighbor nearestWithWorkspaceDistance(const Tree& tree, const ::rl::math::Vector& chosen);
  RrtConConBase::Neighbor nearestWithKdTree(const Tree& tree, const ::rl::math::Vector& chosen);
  Vertex addVertex(Tree& tree, const ::rl::plan::VectorPtr& q);
};

#endif // _YOUR_PLANNER_H_
