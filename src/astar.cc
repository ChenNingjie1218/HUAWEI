#include "astar.h"

#include <algorithm>
#include <iostream>
#include <unordered_map>

#include "berth.h"
#include "boat.h"
#include "input_controller.h"
#include "map_controller.h"
#include "param.h"
#include "rent_controller.h"
extern int id;
// 方向数组
std::array<Location, 4> DIRS = {Location(0, 1), Location(0, -1),
                                Location(-1, 0), Location(1, 0)};
#ifdef SAVE_OLD_PATH
std::map<std::pair<Location, Location>, std::vector<Location>> Astar::old_path;
#endif

Location::Location(int x, int y, int direction)
    : x(x), y(y), boat_direction(direction) {}

bool operator==(const Location &a, const Location &b) {
  if (a.boat_direction == -1 || b.boat_direction == -1) {
    return a.x == b.x && a.y == b.y;
  }
  return a.x == b.x && a.y == b.y && a.boat_direction == b.boat_direction;
}
bool operator!=(const Location &a, const Location &b) { return !(a == b); }

bool operator<(Location a, Location b) {
  return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

bool operator==(const Point &a, const Point &b) { return a.loc == b.loc; }

// 这种构造函数不算邻居
Point::Point(int x, int y) : loc(x, y) {}

Point::Point(Location loc) {
  this->loc = loc;
  for (auto dir : DIRS) {
    Point next_point(loc.x + dir.x, loc.y + dir.y);
    if (next_point.CanReach()) {
      neighbors.push_back(next_point.loc);
    }
  }
}

bool Point::CanReach() {
  if (MapController::GetInstance()->busy_point[loc.x][loc.y] >
      DynamicParam::GetInstance()->GetBusyValve()) {
    return false;
  }
  if (MapController::GetInstance()->CanRobotReach(loc.x, loc.y)) {
    return true;
  }
  return false;
}

inline double heuristic(Location a, Location b) {
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

Astar::Astar(int start_x, int start_y, int end_x, int end_y, int direction)
    : start(start_x, start_y, direction), end(end_x, end_y) {}

// 找货物A*
bool Astar::AstarSearch(std::vector<Location> &path, int &astar_deep,
                        Goods *&find_goods) {
#ifdef SAVE_OLD_PATH
  std::pair<Location, Location> route = std::make_pair(start, end);
  if (old_path.find(route) != old_path.end()) {
    // 已经算过了
    path = old_path[std::make_pair(start, end)];
    return true;
  }
#endif
  PriorityQueue<Location, double> frontier;
  std::unordered_map<Location, Location> came_from;
  std::unordered_map<Location, double> cost_so_far;
  frontier.put(start, 0);
  came_from[start] = start;
  cost_so_far[start] = 0;

  while (!frontier.empty()) {
    Location current = frontier.get();
    if (current != start &&
        MapController::GetInstance()->busy_point[current.x][current.y] >
            DynamicParam::GetInstance()->GetBusyValve()) {
      continue;
    }
// 超过深度剪枝
#ifdef CUT_A_STAR
    if (cost_so_far[current] > astar_deep) {
      if (astar_deep < LIFETIME - 50) {
        astar_deep += 50;
      }
      return false;
    }
#endif

    // 如果是找货物
    if (MapController::GetInstance()->gds[current.x][current.y] &&
        MapController::GetInstance()->gds[current.x][current.y]->robot_id ==
            -1) {
#ifdef CHANGE_CLOSED_GOODS
      if (MapController::GetInstance()->gds[current.x][current.y]->money >
          DynamicParam::GetInstance()->GetValueableGoodsValve()) {
        // 只换值钱的货物
        find_goods = MapController::GetInstance()->gds[current.x][current.y];
      }
#endif

      if (find_goods->x == current.x && find_goods->y == current.y) {
        // 到达目标货物
#ifdef CUT_A_STAR
        if (astar_deep > DEFAULT_A_STAR_DEEP) {
          astar_deep -= 50;
        }
#endif
        Location temp = current;
        path.clear();
        // int count = 0;
        while (temp != start) {
          path.push_back(temp);
          // std::cerr << "(" << temp.x << "," << temp.y << ")" << std::endl;
          temp = came_from[temp];
          // ++count;
        }
        // std::cerr << count << std::endl;
        std::reverse(path.begin(), path.end());
#ifdef SAVE_OLD_PATH
        old_path[route] = path;
#endif
        return true;
      }
    }

    for (auto next : Point(current).neighbors) {
      double new_cost = cost_so_far[current] + 1;
      if (cost_so_far.find(next) == cost_so_far.end() ||
          new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        double priority = new_cost + heuristic(next, end);
        frontier.put(next, priority);
        came_from[next] = current;
      }
    }
  }
  return false;
}

// 找泊位A*
bool Astar::AstarSearch(std::vector<Location> &path, const int &berth_id) {
#ifdef SAVE_OLD_PATH
  std::pair<Location, Location> route = std::make_pair(start, end);
  if (old_path.find(route) != old_path.end()) {
    // 已经算过了
    path = old_path[route];
    return true;
  }
#endif
  PriorityQueue<Location, double> frontier;
  std::unordered_map<Location, Location> came_from;
  std::unordered_map<Location, double> cost_so_far;
  frontier.put(start, 0);
  came_from[start] = start;
  cost_so_far[start] = 0;
  while (!frontier.empty()) {
    Location current = frontier.get();
    if (MapController::GetInstance()->ch[current.x][current.y] == 'B' &&
        berth_id ==
            MapController::GetInstance()->location_to_berth_id[current]) {
      Location temp = current;
      path.clear();
      // int count = 0;
      while (temp != start) {
        path.push_back(temp);
        // std::cerr << "(" << temp.x << "," << temp.y << ")" << std::endl;
        temp = came_from[temp];
        // ++count;
      }

      // std::cerr << count << std::endl;
      std::reverse(path.begin(), path.end());
#ifdef SAVE_OLD_PATH
      old_path[route] = path;
#endif
      return true;
    }
    for (auto next : Point(current).neighbors) {
      double new_cost = cost_so_far[current] + 1;
      if (cost_so_far.find(next) == cost_so_far.end() ||
          new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        double priority = new_cost + heuristic(next, end);
        frontier.put(next, priority);
        came_from[next] = current;
      }
    }
  }
  return false;
}

// 船
void Astar::AstarSearch(std::vector<int> &path) {
  PriorityQueue<Location, double> frontier;
  std::unordered_map<Location, Location> came_from;
  std::unordered_map<Location, double> cost_so_far;
  frontier.put(start, 0);
  came_from[start] = start;
  cost_so_far[start] = 0;
  Location debug_point(198, 5, BOAT_DIRECTION_LEFT);
  while (!frontier.empty()) {
    Location current = frontier.get();
    if (current == end) {
      Location temp = current;
      path.clear();
      // int count = 0;
      while (temp != start) {
        // std::cerr << "(" << temp.x << "," << temp.y << ")" << std::endl;
        path.push_back(temp.boat_direction);
        temp = came_from[temp];
        // ++count;
      }
      // std::cerr << count << std::endl;
      std::reverse(path.begin(), path.end());
      return;
    }

    // 直行
    Location next(current.x + DIRS[current.boat_direction].x,
                  current.y + DIRS[current.boat_direction].y,
                  current.boat_direction);
    if (!CollisionBox(next.x, next.y, next.boat_direction).IsCollision()) {
      double new_cost = cost_so_far[current] + 1;
      auto it = cost_so_far.find(next);
      if (it == cost_so_far.end() || new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        double priority = new_cost + heuristic(next, end);
        frontier.put(next, priority);
        came_from[next] = current;
      } else if (it->first.boat_direction != next.boat_direction) {
        cost_so_far[next] = new_cost;
        double priority = new_cost + heuristic(next, end);
        frontier.put(next, priority);
        came_from[next] = current;
      }
    }

    // 顺时针转
    next = current.Clockwise();
    if (!CollisionBox(next.x, next.y, next.boat_direction).IsCollision()) {
      double new_cost = cost_so_far[current] + 1;
      auto it = cost_so_far.find(next);
      if (it == cost_so_far.end() || new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        double priority = new_cost + heuristic(next, end);
        frontier.put(next, priority);
        came_from[next] = current;
      } else if (it->first.boat_direction != next.boat_direction) {
        cost_so_far[next] = new_cost;
        double priority = new_cost + heuristic(next, end);
        frontier.put(next, priority);
        came_from[next] = current;
      }
    }

    // 逆时针转
    next = current.CounterClockwise();
    if (!CollisionBox(next.x, next.y, next.boat_direction).IsCollision()) {
      double new_cost = cost_so_far[current] + 1;
      auto it = cost_so_far.find(next);
      if (it == cost_so_far.end() || new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        double priority = new_cost + heuristic(next, end);
        frontier.put(next, priority);
        came_from[next] = current;
      } else if (it->first.boat_direction != next.boat_direction) {
        cost_so_far[next] = new_cost;
        double priority = new_cost + heuristic(next, end);
        frontier.put(next, priority);
        came_from[next] = current;
      }
    }
  }
  // std::cerr << "没路径" << std::endl;
}

// 顺时针转动
Location Location::Clockwise() {
  switch (boat_direction) {
    case BOAT_DIRECTION_RIGHT:
      return Location(x, y + 2, BOAT_DIRECTION_DOWN);
    case BOAT_DIRECTION_LEFT:
      return Location(x, y - 2, BOAT_DIRECTION_UP);
    case BOAT_DIRECTION_UP:
      return Location(x - 2, y, BOAT_DIRECTION_RIGHT);
    case BOAT_DIRECTION_DOWN:
      return Location(x + 2, y, BOAT_DIRECTION_LEFT);
    default:
      return Location();
      break;
  }
}

// 逆时针转动
Location Location::CounterClockwise() {
  switch (boat_direction) {
    case BOAT_DIRECTION_RIGHT:
      return Location(x + 1, y + 1, BOAT_DIRECTION_UP);
    case BOAT_DIRECTION_LEFT:
      return Location(x - 1, y - 1, BOAT_DIRECTION_DOWN);
    case BOAT_DIRECTION_UP:
      return Location(x - 1, y + 1, BOAT_DIRECTION_LEFT);
    case BOAT_DIRECTION_DOWN:
      return Location(x + 1, y - 1, BOAT_DIRECTION_RIGHT);
    default:
      return Location();
      break;
  }
}