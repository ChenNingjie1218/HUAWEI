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
  auto &map_instance = MapController::GetInstance();
  if (map_instance->busy_point[loc.x][loc.y] >
      DynamicParam::GetInstance()->GetBusyValve()) {
    return false;
  }
  if (map_instance->CanRobotReach(loc.x, loc.y)) {
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
bool Astar::AstarSearch(std::vector<Location> &path, Goods *&find_goods) {
  PriorityQueue<Location, double> frontier;
  std::unordered_map<Location, Location> came_from;
  frontier.put(start, 0);
  came_from[start] = start;
  double *cost_so_far = new double[1000000];
  cost_so_far[start.x * 1000 + start.y] = 1;
  while (!frontier.empty()) {
    Location current = frontier.get();
    int cur_hash_index = current.x * 1000 + current.y;
    if (current != start &&
        MapController::GetInstance()->busy_point[current.x][current.y] >
            DynamicParam::GetInstance()->GetBusyValve()) {
      continue;
    }

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
        return true;
      }
    }

    for (auto next : Point(current).neighbors) {
      double new_cost = cost_so_far[cur_hash_index] + 1;
      int hash_index = next.x * 1000 + next.y;
      if (!cost_so_far[hash_index]) {
        cost_so_far[hash_index] = new_cost;
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
  PriorityQueue<Location, double> frontier;
  std::unordered_map<Location, Location> came_from;
  frontier.put(start, 0);
  came_from[start] = start;
  double *cost_so_far = new double[1000000];
  cost_so_far[start.x * 1000 + start.y] = 1;
  int count = 0;
  while (!frontier.empty()) {
    Location current = frontier.get();
    ++count;
    // ++MapController::GetInstance()->astar_debug[current.x][current.y];
    int cur_hash_index = current.x * 1000 + current.y;
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

      std::cerr << count << std::endl;
      std::reverse(path.begin(), path.end());
      return true;
    }
    for (auto next : Point(current).neighbors) {
      double new_cost = cost_so_far[cur_hash_index] + 1;
      int hash_index = next.x * 1000 + next.y;
      if (!cost_so_far[hash_index]) {
        cost_so_far[hash_index] = new_cost;
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
  frontier.put(start, 0);
  came_from[start] = start;
  double *cost_so_far = new double[10000000];
  cost_so_far[start.boat_direction * 1000000 + start.x * 1000 + start.y] = 1;
  // int count = 0;
  // 方位朝向不对加高代价
  int located_left = end.y < start.y ? 1 : 0;  // 终点位于起点左边
  int located_up = end.x < start.x ? 1 : 0;    // 终点位于起点上边
  while (!frontier.empty()) {
    Location current = frontier.get();
    // ++count;
    // ++MapController::GetInstance()->astar_debug[current.x][current.y];
    int cur_hash_index =
        current.boat_direction * 1000000 + current.x * 1000 + current.y;
    if (current == end) {
      Location temp = current;
      path.clear();
      // int size = 0;
      while (temp != start) {
        // std::cerr << "(" << temp.x << "," << temp.y << ")" << std::endl;
        path.push_back(temp.boat_direction);
        temp = came_from[temp];
        // ++size;
      }
      // std::cerr << count << std::endl;
      // std::cerr << size << std::endl;
      std::reverse(path.begin(), path.end());
      return;
    }
    // 直行
    Location next = current.Ship();
    for (int i = 0; i < 3; ++i) {
      switch (i) {
        case 1:
          // 顺时针转
          next = current.Clockwise();
          break;
        case 2:
          // 逆时针转
          next = current.CounterClockwise();
          break;
        default:
          // 直行
          break;
      }
      CollisionBox boat_box = CollisionBox(next.x, next.y, next.boat_direction);
      if (!boat_box.IsCollision()) {
        double new_cost = cost_so_far[cur_hash_index] + 1;
        if (i == 2) {
          --new_cost;
        }
        int hash_index = next.boat_direction * 1000000 + next.x * 1000 + next.y;
        if (!cost_so_far[hash_index]) {
          // 主航道要减速
          if (boat_box.IsLocatedOnMainRoute()) {
            new_cost += 0.33;
          }

          cost_so_far[hash_index] = new_cost;
          double priority = new_cost + heuristic(next, end);

          // 往反方向走，代价加三
          if (next.boat_direction == BOAT_DIRECTION_LEFT ||
              next.boat_direction == BOAT_DIRECTION_RIGHT) {
            if (located_left ^
                (next.boat_direction == BOAT_DIRECTION_LEFT ? 1 : 0)) {
              // 横向反方向走
              priority += 3;
            }
          } else if (located_up ^
                     (next.boat_direction == BOAT_DIRECTION_UP ? 1 : 0)) {
            // 纵向反方向走
            priority += 3;
          }

          frontier.put(next, priority);
          came_from[next] = current;
        }
      }
    }
  }
#ifdef DEBUG
  std::cerr << "没路径" << std::endl;
#endif
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

// 直行
Location Location::Ship() {
  return Location(x + DIRS[boat_direction].x, y + DIRS[boat_direction].y,
                  boat_direction);
}