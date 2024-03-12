#ifndef ASTAR_H_
#define ASTAR_H_
#include <array>
#include <list>
#include <queue>
#include <vector>

struct Location {
  int x, y;
  Location() = default;
  Location(int x, int y);
};

namespace std {
/* implement hash function so we can put GridLocation into an unordered_set */
template <>
struct hash<Location> {
  std::size_t operator()(const Location &id) const noexcept {
    // NOTE: better to use something like boost hash_combine
    return std::hash<int>()(id.x ^ (id.y << 16));
  }
};
}  // namespace std

struct Point {
  Location loc;
  std::vector<Location> neighbors;
  static std::array<Location, 4> DIR;
  Point() = default;
  Point(int x, int y);
  Point(Location loc);
  ~Point() = default;
  // 该点是否可以走动
  bool CanReach();
};

template <typename T, typename priority_t>
struct PriorityQueue {
  typedef std::pair<priority_t, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>,
                      std::greater<PQElement>>
      elements;

  inline bool empty() const { return elements.empty(); }

  inline void put(T item, priority_t priority) {
    elements.emplace(priority, item);
  }

  T get() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }
};

struct Astar {
  Location start;  // 起点坐标
  Location end;    // 终点坐标
  // Astar() = default;
  Astar(int start_x, int start_y, int end_x, int end_y);
  // void ReUse(int start_x, int start_y, int end_x, int end_y);
  bool AstarSearch(std::vector<Location> &path, bool is_berth = 0);
};
#endif