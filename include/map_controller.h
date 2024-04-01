#ifndef MAP
#define MAP
#include <map>
#include <utility>
#include <vector>

#include "astar.h"
#include "goods.h"
struct MapController {
 private:
  MapController() = default;
  MapController(const MapController &other) = default;
  static MapController *instance_;

 public:
  static MapController *&GetInstance();

  int parent[N * N];

  Goods *gds[N][N] = {{nullptr}};
  char ch[N][N];
  std::vector<std::pair<int, int>> robot_purchase_point;
  std::vector<std::pair<int, int>> boat_purchase_point;
  std::vector<std::pair<int, int>> delivery_point;

  // 坐标映射到泊位id
  std::map<Location, int> location_to_berth_id;

  // 用并查集分区
  // 找集合
  int FindArea(int id);
  // 合并集合
  void MergeArea(int id_1, int id_2);
};

#endif