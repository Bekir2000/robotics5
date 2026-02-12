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

protected:
  void choose(::rl::math::Vector& chosen);

  RrtConConBase::Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

  RrtConConBase::Vertex extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

  bool areEqual(const ::rl::math::Vector& lhs, const ::rl::math::Vector& rhs) const override;

  RrtConConBase::Neighbor nearest(const Tree& tree, const ::rl::math::Vector& chosen) override;

private:

};

#endif // _YOUR_PLANNER_H_
