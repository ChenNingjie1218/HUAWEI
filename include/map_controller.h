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

  int land_area[N * N];            // 陆地区域
  int sea_area[N * N];             // 海洋区域
  int busy_point[N][N];            // 堵车点
  Goods *gds[N][N] = {{nullptr}};  // 货物地图
  int nearest_berth[N][N];         // 距离该点最近的泊位id
  int berth_num;                   // 泊位数量
  std::vector<Berth> berth;        // 泊位数据
  /*
   ‘.’ ： 空地
   ‘>’ ： 陆地主干道
   ‘*’ ： 海洋
   ‘~’ ： 海洋主航道
   ‘#’ ： 障碍
   ‘R’ ： 机器人购买地块，同时该地块也是主干道
   ‘S’ ： 船舶购买地块，同时该地块也是主航道
   ‘B’ ： 泊位
   ‘K’ ： 靠泊区
   ‘C’ ： 海陆立体交通地块
   ‘c’ ： 海陆立体交通地块，同时为主干道和主航道
   ‘T’ ： 交货点
   */
  char ch[N][N];                                          // 地图数据
  std::vector<std::pair<int, int>> robot_purchase_point;  // 机器人购买点
  std::vector<std::pair<int, int>> boat_purchase_point;   // 船购买点
  std::vector<std::pair<int, int>> delivery_point;        // 交货点

  // 坐标映射到泊位id
  std::map<Location, int> location_to_berth_id;

  std::array<Location, 4> DIRS = {Location(1, 0), Location(-1, 0),
                                  Location(0, 1),
                                  Location(0, -1)};  // 上下左右方位

  // 初始化各项数据
  void InitMapData();

  // 初始化nearest_berth
  void InitNearestBerth(std::queue<std::pair<Location, int>> &q);
  // 机器人可达
  bool CanRobotReach(int x, int y);

  // 船可达
  bool CanBoatReach(int x, int y);

  // 初始化坐标映射到泊位id的map
  void InitBerthMap(int berth_id, int berth_x, int berth_y);

  // 用并查集分区
  // 找集合
  int FindArea(int id, bool is_land = true);
  // 合并集合
  void MergeArea(int id_1, int id_2, bool is_land = true);
};

#endif