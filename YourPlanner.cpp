#include "YourPlanner.h"
#include <rl/plan/SimpleModel.h>

YourPlanner::YourPlanner() :
  RrtConConBase()
{
  // Initialize weights for PUMA 560 (6 DOF)
  // Base, Shoulder, Elbow are high priority; Wrist is low priority.
  this->weights.resize(6);
  this->weights << 1.0, 0.8, 0.6, 0.2, 0.1, 0.1; 
}

YourPlanner::~YourPlanner() {}

::std::string YourPlanner::getName() const {
  return "PUMA 560 Weighted RRT";
}

::rl::math::Real 
YourPlanner::weightedDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const {
  ::rl::math::Real sumSq = 0;
  for(int i = 0; i < q1.size(); ++i) {
    // transformedDistance(v1, v2, joint_index) returns the SHORTEST squared distance
    // correctly handling the wraparound (circular) joints.
    sumSq += this->weights(i) * this->model->transformedDistance(q1(i), q2(i), i);
  }
  return std::sqrt(sumSq);
}

RrtConConBase::Neighbor
YourPlanner::nearest(const Tree& tree, const ::rl::math::Vector& chosen) {
  Neighbor p(Vertex(), (::std::numeric_limits<::rl::math::Real>::max)());

  for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first) {
    ::rl::math::Real dSq = 0;
    const ::rl::math::Vector& q_tree = *tree[*i.first].q;
    
    // Manual loop is faster here as it avoids vector subtraction allocations
    for(int j = 0; j < chosen.size(); ++j) {
      dSq += this->weights(j) * this->model->transformedDistance(chosen(j), q_tree(j), j);
    }

    if (dSq < p.second) {
      p.first = *i.first;
      p.second = dSq; 
    }
  }

  // Convert squared distance to real distance for 'delta' comparisons
  p.second = std::sqrt(p.second);
  return p;
}

bool
YourPlanner::areEqual(const ::rl::math::Vector& lhs, const ::rl::math::Vector& rhs) const {
  return this->weightedDistance(lhs, rhs) <= this->epsilon;
}

RrtConConBase::Vertex 
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen) {
  ::rl::math::Real dist = nearest.second;
  bool reached = false;
  ::rl::math::Real step = (dist <= this->delta) ? (reached = true, dist) : this->delta;

  ::rl::plan::VectorPtr last = ::std::make_shared<::rl::math::Vector>(this->model->getDof());
  this->model->interpolate(*tree[nearest.first].q, chosen, step / dist, *last);

  this->model->setPosition(*last);
  this->model->updateFrames();

  if (this->model->isColliding()) return NULL;

  ::rl::math::Vector next(this->model->getDof());
  while (!reached) {
    dist = this->weightedDistance(*last, chosen);
    step = (dist <= this->delta) ? (reached = true, dist) : this->delta;

    this->model->interpolate(*last, chosen, step / dist, next);
    this->model->setPosition(next);
    this->model->updateFrames();

    if (this->model->isColliding()) break;
    *last = next;
  }

  Vertex v = this->addVertex(tree, last);
  this->addEdge(nearest.first, v, tree);
  return v;
}

RrtConConBase::Vertex 
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen) {
  ::rl::math::Real dist = nearest.second;
  ::rl::math::Real step = (::std::min)(dist, this->delta);

  ::rl::plan::VectorPtr next = ::std::make_shared<::rl::math::Vector>(this->model->getDof());
  this->model->interpolate(*tree[nearest.first].q, chosen, step / dist, *next);

  this->model->setPosition(*next);
  this->model->updateFrames();

  if (!this->model->isColliding()) {
    Vertex v = this->addVertex(tree, next);
    this->addEdge(nearest.first, v, tree);
    return v;
  }
  return NULL;
}

bool YourPlanner::solve() {
  return RrtConConBase::solve();
}
