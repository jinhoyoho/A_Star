
#include "global_planner.h"
#include <iostream>

namespace global_planner
{
/**
 * @brief Set or reset obstacle factor
 * @param factor obstacle factor
 */
void GlobalPlanner::setFactor(float factor)
{
  factor_ = factor;
}

/**
 * @brief get the costmap
 * @return costmap costmap2d pointer
 */
nav2_costmap_2d::Costmap2D* GlobalPlanner::getCostMap() const
{
  return costmap_;
}

/**
 * @brief get the size of costmap
 * @return map_size the size of costmap
 */
int GlobalPlanner::getMapSize() const
{
  return map_size_;
}

/**
 * @brief Transform from grid map(x, y) to grid index(i)
 * @param x grid map x
 * @param y grid map y
 * @return index
 */
int GlobalPlanner::grid2Index(int x, int y)
{
  return x + static_cast<int>(costmap_->getSizeInCellsX() * y);
}

/**
 * @brief Transform from grid index(i) to grid map(x, y)
 * @param i grid index i
 * @param x grid map x
 * @param y grid map y
 */
void GlobalPlanner::index2Grid(int i, int& x, int& y)
{
  x = static_cast<int>(i % costmap_->getSizeInCellsX());
  y = static_cast<int>(i / costmap_->getSizeInCellsX());
}

/**
 * @brief Tranform from world map(x, y) to costmap(x, y)
 * @param mx costmap x
 * @param my costmap y
 * @param wx world map x
 * @param wy world map y
 * @return true if successfull, else false
 */
bool GlobalPlanner::world2Map(double wx, double wy, unsigned int& mx, unsigned int& my)
{
  return costmap_->worldToMap(wx, wy, mx, my);
}

/**
 * @brief Tranform from costmap(x, y) to world map(x, y)
 * @param mx costmap x
 * @param my costmap y
 * @param wx world map x
 * @param wy world map y
 */
void GlobalPlanner::map2World(unsigned int mx, unsigned int my, double& wx, double& wy)
{
  costmap_->mapToWorld(mx, my, wx, wy);
}

/**
 * @brief Inflate the boundary of costmap into obstacles to prevent cross planning
 */
void GlobalPlanner::outlineMap()
{
  auto nx = costmap_->getSizeInCellsX();
  auto ny = costmap_->getSizeInCellsY();
  auto pc = costmap_->getCharMap();
  for (int i = 0; i < nx; i++)
    *pc++ = nav2_costmap_2d::LETHAL_OBSTACLE;
  pc = costmap_->getCharMap() + (ny - 1) * nx;
  for (int i = 0; i < nx; i++)
    *pc++ = nav2_costmap_2d::LETHAL_OBSTACLE;
  pc = costmap_->getCharMap();
  for (int i = 0; i < ny; i++, pc += nx)
    *pc = nav2_costmap_2d::LETHAL_OBSTACLE;
  pc = costmap_->getCharMap() + nx - 1;
  for (int i = 0; i < ny; i++, pc += nx)
    *pc = nav2_costmap_2d::LETHAL_OBSTACLE;
}

std::vector<Node> GlobalPlanner::_convertClosedListToPath(std::unordered_map<int, Node>& closed_list, const Node& start, const Node& goal) {
    std::vector<Node> path;
    auto current_it = closed_list.end(); // iterator 초기화

    // 목표 노드를 closed_list에서 좌표로 찾기
    for (const auto& entry : closed_list) {
        if (entry.second.x() == goal.x() && entry.second.y() == goal.y()) {
            current_it = closed_list.find(entry.first); // ID로 찾기
            break;
        }
    }

    // 목표 노드가 closed_list에 존재하지 않는 경우
    if (current_it == closed_list.end()) {
        std::cerr << "Goal node not found in closed list!\n";
        return {};
    }

    // 현재 노드 설정
    Node current = current_it->second;

    while (current != start) {
        path.emplace_back(current.x(), current.y());
        
        // 부모 노드를 찾기 위해 ID로 closed_list에서 검색
        auto it = closed_list.find(current.pid());
        
        // 부모 노드가 closed_list에 없는 경우
        if (it != closed_list.end())
            current = it->second;
        else
            return {};
    }

    path.push_back(start);
    return path;
}


}  // namespace global_planner