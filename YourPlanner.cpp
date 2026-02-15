#include "YourPlanner.h"
#include "YourSampler.h"

#include <rl/plan/SimpleModel.h>
#include <rl/plan/KdtreeBoundingBoxNearestNeighbors.h>


YourPlanner::YourPlanner() :
  RrtConConBase()
{
  this->sampler = new YourSampler();

  // Prevent destructor crashes if solve() never ran
  kdtrees.assign(2, nullptr);

  if (useWeightedMetric)
  {
    std::size_t dof = this->model->getDof();
    weights.resize(dof);
    for (std::size_t i = 0; i < dof; ++i)
      weights[i] = static_cast<::rl::math::Real>(dof - i) / dof;
  }
}

YourPlanner::~YourPlanner()
{
  for (std::size_t i = 0; i < kdtrees.size(); ++i)
  {
    if (kdtrees[i] != nullptr)
    {
      delete kdtrees[i];
      kdtrees[i] = nullptr;
    }
  }
}


::std::string YourPlanner::getName() const
{
    std::vector<std::string> features;
    
    if (this->useKdTree) features.push_back("KdTree");
    if (this->useWorkspaceDistance) features.push_back("WorkspaceDistance");
    if (this->useExaustedNodePruning) features.push_back("exaustedNodePruning");


    YourSampler* mySampler = static_cast<YourSampler*>(this->sampler);
    if (mySampler && mySampler->useNormalDistribution) {
        features.push_back("Normaldistribution");
    }
    
    if (features.empty()) {
        return "Your Planner";
    }

    std::string result = "Your Planner (";
    for (size_t i = 0; i < features.size(); ++i) {
        result += features[i];
        if (i < features.size() - 1) result += " + ";
    }
    result += ")";
    
    return result;
}

void
YourPlanner::choose(::rl::math::Vector& chosen)
{
  RrtConConBase::choose(chosen);
}

YourPlanner::Vertex
YourPlanner::addVertex(Tree& tree, const ::rl::plan::VectorPtr& q)
{
  Vertex v = RrtConConBase::addVertex(tree, q);

  std::size_t idx = (&tree == &this->tree[0]) ? 0 : 1;

  // ALWAYS maintain mapping consistency
  vertexMap[idx].push_back(v);

  if (this->useKdTree && kdtrees[idx] != nullptr)
  {
    std::size_t vertex_idx = vertexMap[idx].size() - 1;

    rl::plan::Metric::Value value(q.get(), reinterpret_cast<void*>(vertex_idx));

    kdtrees[idx]->push(value);
  }

  return v;
}

RrtConConBase::Neighbor
YourPlanner::nearestWithKdTree(const Tree& tree, const ::rl::math::Vector& chosen)
{
  std::size_t idx = (&tree == &this->tree[0]) ? 0 : 1;

  if (kdtrees[idx] == nullptr || vertexMap[idx].empty())
    return RrtConConBase::nearest(tree, chosen);

  rl::plan::Metric::Value query(&chosen, nullptr);
  
  size_t k = this->useExaustedNodePruning ? this->exaustedThreshold : 1;
  auto neighbors = kdtrees[idx]->nearest(query, k);

  for (const auto& candidate : neighbors)
  {
    
    std::size_t v_idx = reinterpret_cast<std::size_t>(candidate.second.second);
    
    if (v_idx >= vertexMap[idx].size()) continue; // Safety check

    Vertex v = this->vertexMap[idx][v_idx];
    
    if (this->useExaustedNodePruning)
    {
      if (tree[v].failCount < this->exaustedThreshold)
      {
        return Neighbor(v, candidate.first); 
      }
    }
    else
    {
      return Neighbor(v, candidate.first);
    }
  }

  // If all candidates were pruned or search failed, use standard nearest
  return RrtConConBase::nearest(tree, chosen);
}

RrtConConBase::Neighbor
YourPlanner::nearestWithWorkspaceDistance(const Tree& tree, const ::rl::math::Vector& chosen)
{
    const ::rl::math::Real alpha = 1.0; 
    const ::rl::math::Real beta  = 1.0;   

    Neighbor best(Vertex(), std::numeric_limits< ::rl::math::Real >::max());

    // Precompute chosen TCP once
    this->model->setPosition(chosen);
    this->model->updateFrames(false);
    const ::rl::math::Vector3 chosenTcp =
        this->model->forwardPosition().translation();

    ::rl::math::Real bestScore =
        std::numeric_limits< ::rl::math::Real >::max();

    for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
    {
        const Vertex v = *i.first;
        const ::rl::math::Vector& q = *tree[v].q;

        // Joint-space distance (torus-safe inside model)
        const ::rl::math::Real jointD =
            this->model->distance(chosen, q);

        // Workspace distance
        this->model->setPosition(q);
        this->model->updateFrames(false);
        const ::rl::math::Vector3 tcp =
            this->model->forwardPosition().translation();

        const ::rl::math::Real workspaceD =
            (chosenTcp - tcp).norm();

        const ::rl::math::Real score =
            alpha * jointD + beta * workspaceD;

        if (score < bestScore)
        {
            bestScore = score;
            best.first = v;
            best.second = jointD; 
        }
    }

    return best;
}

RrtConConBase::Neighbor
YourPlanner::nearestWithSkippingExhaustedNodes(const Tree& tree, const ::rl::math::Vector& chosen)
{
  Neighbor p(Vertex(), (::std::numeric_limits<::rl::math::Real>::max)());

  for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
  {
    
    
    if (tree[*i.first].failCount >= this->exaustedThreshold) {
        continue; // Skip this node, it's stuck against a wall
    }

    ::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*i.first].q);

    if (d < p.second) {
      p.first = *i.first;
      p.second = d;
    }
  }

  p.second = this->model->inverseOfTransformedDistance(p.second);
  return p;
}

YourPlanner::nearestWithWeightingDistanceMetric(const Tree& tree, const ::rl::math::Vector& chosen){
  //create an empty pair <Vertex, distance> to return
  Neighbor p(Vertex(), (::std::numeric_limits< ::rl::math::Real >::max)());
  ::rl::math::Real bestRank = (::std::numeric_limits<::rl::math::Real>::max)();

  //Iterate through all vertices to find the nearest neighbour
  ::rl::math::Real rank;
  for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
  {
    ::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*i.first].q);

    for(int j = 0; j < chosen.size(); ++j){
      rank += weights[j] * d;
    }

    if (rank < bestRank)
    {
      bestRank = rank;
      p.first = *i.first;
      p.second = d;
    }
  }


  // Compute the square root of distance
  p.second = this->model->inverseOfTransformedDistance(p.second);

  return p;
}


RrtConConBase::Neighbor
YourPlanner::nearest(const Tree& tree, const ::rl::math::Vector& chosen)
{
  if (this->useWeightedMetric)
    return nearestWithWeightingDistanceMetric(tree, chosen);
  if (this->useKdTree)
    return nearestWithKdTree(tree, chosen);

  if (this->useWorkspaceDistance)
    return nearestWithWorkspaceDistance(tree, chosen);

  if (this->useExaustedNodePruning){
    return nearestWithSkippingExhaustedNodes(tree, chosen);
  }

  return RrtConConBase::nearest(tree, chosen);
}

RrtConConBase::Vertex
YourPlanner::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  Vertex result = RrtConConBase::connect(tree, nearest, chosen);

  // If result is NULL, it means we hit a wall immediately
  if (NULL == result)
  {
    tree[nearest.first].failCount += 1.0; 
  }
  
  return result;
}

RrtConConBase::Vertex
YourPlanner::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
{
  Vertex result = RrtConConBase::extend(tree, nearest, chosen);

  if (NULL == result)
  {
    tree[nearest.first].failCount += 1.0;
  }

  return RrtConConBase::extend(tree, nearest, chosen);
}

bool
YourPlanner::solve()
{
  if (this->useKdTree)
  {
    for (std::size_t i = 0; i < 2; ++i)
    {
      if (kdtrees[i] != nullptr)
      {
        delete kdtrees[i];
        kdtrees[i] = nullptr;
      }

      kdtrees[i] = new rl::plan::KdtreeBoundingBoxNearestNeighbors(this->model);
      vertexMap[i].clear();
    }
  }

  return RrtConConBase::solve();
}
