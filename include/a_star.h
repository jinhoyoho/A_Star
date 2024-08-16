
#ifndef A_STAR_H
#define A_STAR_H

#include "global_planner.h"

namespace global_planner
{
/**
 * @brief Class for objects that plan using the A* algorithm
 */
class AStar : public GlobalPlanner
{
public:
  /**
   * @brief Construct a new AStar object
   * @param costmap   the environment for path planning
   * @param dijkstra   using diksktra implementation
   * @param gbfs       using gbfs implementation
   */
  AStar(nav2_costmap_2d::Costmap2D* costmap, bool dijkstra = false, bool gbfs = false);

 
  bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

private:
  bool is_dijkstra_;  // using diksktra
  bool is_gbfs_;      // using greedy best first search(GBFS)
};
}  // namespace global_planner
#endif
