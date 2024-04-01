#ifndef MAP
#define MAP
#include <map>
#include <utility>
#include <vector>

#include "astar.h"
#include "berth.h"
#include "goods.h"
struct MapController {
 private:
  MapController() = default;
  MapController(const MapController &other) = default;
  static MapController *instance_;

 public:
  static MapController *&GetInstance();

  int parent[N * N];               // 并查集区域
  int busy_point[N][N];            // 堵车点
  Goods *gds[N][N] = {{nullptr}};  // 货物地图
  int berth_num;                   // 泊位数量
  std::vector<Berth> berth;        // 泊位数据
  /*
   * - · 空地
   * - * 海洋
   * - # 障碍
   * - A 机器人起始位置，总共10个
   * - B 大小为4*4，标识泊位的位置
   */
  char ch[N][N];  // 地图数据
  std::vector<std::pair<int, int>> robot_purchase_point;
  std::vector<std::pair<int, int>> boat_purchase_point;
  std::vector<std::pair<int, int>> delivery_point;

  // 坐标映射到泊位id
  std::map<Location, int> location_to_berth_id;

  // 机器人可达
  bool CanRobotReach(int x, int y);

  // 初始化坐标映射到泊位id的map
  void InitBerthMap(int berth_id, int berth_x, int berth_y);

  // 用并查集分区
  // 找集合
  int FindArea(int id);
  // 合并集合
  void MergeArea(int id_1, int id_2);
};

#endif