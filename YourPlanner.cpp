#include "YourPlanner.h"
#include <rl/plan/SimpleModel.h>

YourPlanner::YourPlanner() :
  RrtConConBase()
{
}

YourPlanner::~YourPlanner()
{
}

::std::string
YourPlanner::getName() const
{
  return "Weighted Distance Planner";
}

::rl::math::Real 
YourPlanner::weightedDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const {
  ::rl::math::Vector weights(this->model->getDof());
  // PUMA 560 weights: prioritizing base, shoulder, and elbow
  weights << 1.0, 0.8, 0.6, 0.2, 0.1, 0.1; 
  
  ::rl::math::Vector diff = q1 - q2;
  ::rl::math::Real sumSq = 0;
  for(int i = 0; i < diff.size(); ++i) {
    sumSq += weights(i) * diff(i) * diff(i);
  }
  return std::sqrt(sumSq);
}

bool
YourPlanner::areEqual(const ::rl::math::Vector& lhs, const ::rl::math::Vector& rhs) const
{
  // Using weighted distance for configuration comparison 
  if (this->weightedDistance(lhs, rhs) > this->epsilon)
  {
    return false;
  }
  else
  {
    return true;
  }
}

RrtConConBase::Neighbor
YourPlanner::nearest(const Tree& tree, const ::rl::math::Vector& chosen) {
  // Initialize with max possible value 
  Neighbor p(Vertex(), (::std::numeric_limits<::rl::math::Real>::max)());

  ::rl::math::Vector weights(this->model->getDof());
  weights << 1.0, 0.8, 0.6, 0.2, 0.1, 0.1; 

  for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first) {
    ::rl::math::Vector diff = chosen - *tree[*i.first].q;
    
    // Compute weighted squared distance (no sqrt in the loop for speed) 
    ::rl::math::Real dSq = 0;
    for(int j = 0; j < diff.size(); ++j) {
      dSq += weights(j) * diff(j) * diff(j);
    }

    if (dSq < p.second) {
      p.first = *i.first;
      p.second = dSq; // Store the squared distance
    }
  }

  // Compute square root only once for the winner 
  p.second = std::sqrt(p.second);
  return p;
}

void
YourPlanner::choose(::rl::math::Vector& chosen)
{
  //your modifications here
  RrtConConBase::choose(chosen);
}

RrtConConBase::Vertex 
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //your modifications here
  //Do first extend step

  ::rl::math::Real distance = nearest.second;
  ::rl::math::Real step = distance;

  bool reached = false;

  if (step <= this->delta)
  {
    reached = true;
  }
  else
  {
    step = this->delta;
  }

  ::rl::plan::VectorPtr last = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());

  // move "last" along the line q<->chosen by distance "step / distance"
  this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *last);

  this->model->setPosition(*last);
  this->model->updateFrames();

  if (this->model->isColliding())
  {
    return NULL;
  }

  ::rl::math::Vector next(this->model->getDof());

  while (!reached)
  {
    //Do further extend step
    distance = this->weightedDistance(*last, chosen);
    step = distance;

    if (step <= this->delta)
    {
      reached = true;
    }
    else
    {
      step = this->delta;
    }

    // move "next" along the line last<->chosen by distance "step / distance"
    this->model->interpolate(*last, chosen, step / distance, next);

    this->model->setPosition(next);
    this->model->updateFrames();

    if (this->model->isColliding())
    {
      break;
    }

    *last = next;
  }

  // "last" now points to the vertex where the connect step collided with the environment.
  // Add it to the tree
  Vertex connected = this->addVertex(tree, last);
  this->addEdge(nearest.first, connected, tree);
  return connected;
}

RrtConConBase::Vertex 
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //your modifications here
  ::rl::math::Real distance = nearest.second;
  ::rl::math::Real step = (::std::min)(distance, this->delta);

  ::rl::plan::VectorPtr next = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());

  this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *next);

  this->model->setPosition(*next);
  this->model->updateFrames();

  if (!this->model->isColliding())
  {
    Vertex extended = this->addVertex(tree, next);
    this->addEdge(nearest.first, extended, tree);
    return extended;
  }

  return NULL;
}

bool
YourPlanner::solve()
{
  //your modifications here
  return RrtConConBase::solve();
}
