#include "astar.h"

#include <algorithm>
#include <iostream>

#include "berth.h"
#include "input_controller.h"
#include "param.h"
extern char ch[N][N];
extern Goods *gds[N][N];
extern int id;
extern Berth berth[berth_num + 10];
// 方向数组
std::array<Location, 4> DIRS = {Location(1, 0), Location(-1, 0), Location(0, 1),
                                Location(0, -1)};

Location::Location(int x, int y) {
  this->x = x;
  this->y = y;
}

bool operator==(const Location &a, const Location &b) {
  return a.x == b.x && a.y == b.y;
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
  if (ch[loc.x][loc.y] == '.' || ch[loc.x][loc.y] == 'A' ||
      ch[loc.x][loc.y] == 'B') {
    return true;
  }
  return false;
}

inline double heuristic(Location a, Location b) {
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

Astar::Astar(int start_x, int start_y, int end_x, int end_y)
    : start(start_x, start_y), end(end_x, end_y) {}

// 找货物A*
bool Astar::AstarSearch(std::vector<Location> &path, int &astar_deep,
                        Goods *&find_goods) {
  PriorityQueue<Location, double> frontier;
  std::unordered_map<Location, Location> came_from;
  std::unordered_map<Location, double> cost_so_far;
  frontier.put(start, 0);
  came_from[start] = start;
  cost_so_far[start] = 0;

  while (!frontier.empty()) {
    Location current = frontier.get();
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
    if (gds[current.x][current.y] &&
        gds[current.x][current.y]->robot_id == -1 &&
        cost_so_far[current] <
            LIFETIME - id + gds[current.x][current.y]->birth - TOLERANT_TIME) {
      // if (find_goods->money < VALUEABLE_GOODS_VALVE &&
      //     gds[current.x][current.y]->money > find_goods->money) {
      if (gds[current.x][current.y]->money > VALUEABLE_GOODS_VALVE) {
        // 只换值钱的货物
        find_goods = gds[current.x][current.y];
      }

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
bool Astar::AstarSearch(std::vector<Location> &path, int &berth_id,
                        int is_urgent) {
  PriorityQueue<Location, double> frontier;
  std::unordered_map<Location, Location> came_from;
  std::unordered_map<Location, double> cost_so_far;
  frontier.put(start, 0);
  came_from[start] = start;
  cost_so_far[start] = 0;

  while (!frontier.empty()) {
    Location current = frontier.get();
    if (ch[current.x][current.y] == 'B') {
      // 不紧急 找到更近的泊位可结束
      // 紧急   需要该泊位有船
      int temp_berth_id =
          InputController::GetInstance()->location_to_berth_id[current];
      bool can_finish =
          !is_urgent || (is_urgent && !berth[temp_berth_id].q_boat.empty());

      if (can_finish) {
#ifdef DEBUG
        if (berth_id == temp_berth_id) {
          std::cerr << "A*没有换新的目标泊位" << std::endl;
        } else {
          std::cerr << "A*换了新的目标泊位 " << berth_id << " to "
                    << temp_berth_id << std::endl;
        }
#endif
        berth_id = temp_berth_id;
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
// 判断泊位是否可达
bool Astar::AstarSearch(int berth_id) {
  PriorityQueue<Location, double> frontier;
  std::unordered_map<Location, Location> came_from;
  std::unordered_map<Location, double> cost_so_far;
  frontier.put(start, 0);
  came_from[start] = start;
  cost_so_far[start] = 0;

  std::map<Location, int> location_to_berth_id =
      InputController::GetInstance()->location_to_berth_id;
  while (!frontier.empty()) {
    Location current = frontier.get();
    if (ch[current.x][current.y] == 'B' &&
        location_to_berth_id.find(current) != location_to_berth_id.end() &&
        location_to_berth_id[current] == berth_id) {
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