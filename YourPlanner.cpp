#include "YourPlanner.h"
#include <rl/plan/SimpleModel.h>
#include <rl/plan/KdtreeBoundingBoxNearestNeighbors.h>

YourPlanner::YourPlanner() :
  RrtConConBase()
{
  // Initialize weights for PUMA 560 (6 DOF)
  // Base, Shoulder, Elbow are high priority; Wrist is low priority.
  this->weights.resize(6);
  this->weights << 1.0, 0.8, 0.6, 0.2, 0.1, 0.1; 
}

YourPlanner::~YourPlanner()
{
  delete kdtrees[0];
  delete kdtrees[1];
  kdtrees[0] = nullptr;
  kdtrees[1] = nullptr;

}

::std::string
YourPlanner::getName() const
{
  if (this->useKdTree && this->useWorkspaceDistance) {
    return "Your Planner (Kd-Tree + Workspace Distance)";
  }
  if (this->useWorkspaceDistance) {
    return "Your Planner with Workspace Distance";
  }
  if (this->useKdTree) {
    return "Your Planner with Kd-Tree";
  }
  return "Your Planner";
}



void
YourPlanner::choose(::rl::math::Vector& chosen)
{
  //your modifications here
  RrtConConBase::choose(chosen);
}

YourPlanner::Vertex
YourPlanner::addVertex(Tree& tree, const ::rl::plan::VectorPtr& q)
{
  // Add vertex to the tree
  Vertex v = RrtConConBase::addVertex(tree, q);

  std::size_t idx = (&tree == &this->tree[0]) ? 0 : 1;
  
  // Add vertex to the vertexMap
  vertexMap[idx].push_back(v);

  // Create a Metric::Value as RobLib's kd-tree is designed to work only with this object type (does not work with boost vertices)
  // Metric::Value consists of the configuration q and an index
  std::size_t vertex_idx = vertexMap[idx].size() - 1;
  rl::plan::Metric::Value value(q.get(), reinterpret_cast<void*>(vertex_idx));

  // Add vertex to the kd-tree
  kdtrees[idx]->push(value);

  return v;
}


RrtConConBase::Neighbor 
YourPlanner::nearestWithWorkspaceDistance(const Tree& tree, const ::rl::math::Vector& chosen)
{
  //create an empty pair <Vertex, distance> to return
  Neighbor p(Vertex(), (::std::numeric_limits< ::rl::math::Real >::max)());

  // Calculate workspace frames for the 'chosen' configuration once
  this->model->setPosition(chosen);
  this->model->updateFrames(false);
  // Get the end-effector position for the sampled point
  ::rl::math::Vector3 chosenTcp = this->model->forwardPosition().translation();

  //Iterate through all vertices to find the nearest neighbour
  for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
  {
    // Calculate workspace position for the candidate node in the tree
    this->model->setPosition(*tree[*i.first].q);
    this->model->updateFrames();
    ::rl::math::Vector3 candidateTcp = this->model->forwardPosition().translation();

    // Displacement Metric: distance in the physical workspace (m) 
    // instead of joint space (rad)
    ::rl::math::Real d = (chosenTcp - candidateTcp).norm();

    if (d < p.second)
    {
      p.first = *i.first;
      p.second = d;
    }
  }


  // Compute the square root of distance
  //p.second = this->model->inverseOfTransformedDistance(p.second);

  return p;
}

RrtConConBase::Neighbor 
YourPlanner::nearestWithKdTree(const Tree& tree, const ::rl::math::Vector& chosen)
{ 
  // Use RobLib's KdtreeBoundingBoxNearestNeighbors datastructure for finding the nearest neighbor
  
  std::size_t idx = (&tree == &this->tree[0]) ? 0 : 1;
  
  // Create a query
  rl::plan::Metric::Value query(&chosen, nullptr);

  // Query the kd-tree to find the nearest neighbor (one could also query the kd-tree to find n-many neighbors)
  auto neighbors = this->kdtrees[idx]->nearest(query, 1, true);
  
  Neighbor result(Vertex(), 0);

  if (!neighbors.empty())
  {
  	// From the nearest neighbor (neighbors.front()), extract the Metric::Value object
	const rl::plan::Metric::Value& value = neighbors.front().second;

	// Get index for the vertexMap
  std::size_t vertex_idx = reinterpret_cast<std::size_t>(value.second);
	
	// Use the vertexMap to get the boost vertex for the corresponding Metric::Value object
	result.first = vertexMap[idx][vertex_idx];
	
	// Get the distance to the nearest neighbor
	result.second = neighbors.front().first;
	
  }

  // Return boost vertex and distance of the nearest neighbor
  return result;

}

RrtConConBase::Neighbor 
YourPlanner::nearest(const Tree& tree, const ::rl::math::Vector& chosen)
{
  if (this->useKdTree)
    return this->nearestWithKdTree(tree, chosen);
  else if(this->useWorkspaceDistance)
    return this->nearestWithWorkspaceDistance(tree, chosen);
  else
    return RrtConConBase::nearest(tree, chosen);  
  
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
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  //your modifications here
  return RrtConConBase::extend(tree, nearest, chosen);
}

bool
YourPlanner::solve()
{
  //your modifications here

  if(this->useKdTree){
    // Clear old kd-trees and vertex maps from previous execution
    for (std::size_t i = 0; i < 2; ++i)
    {
      delete kdtrees[i];
      kdtrees[i] = new rl::plan::KdtreeBoundingBoxNearestNeighbors(this->model);
      vertexMap[i].clear();
    }

  }

  return RrtConConBase::solve();
}
