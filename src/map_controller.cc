#include "map_controller.h"
MapController* MapController::instance_ = nullptr;
MapController*& MapController::GetInstance() {
  if (!instance_) {
    instance_ = new MapController();
  }
  return instance_;
}
// 用并查集分区
// 找集合
int MapController::FindArea(int id) {
  if (parent[id] != id) {
    parent[id] = FindArea(parent[id]);
  }
  return parent[id];
}
// 合并集合
void MapController::MergeArea(int id_1, int id_2) {
  int root_1 = FindArea(id_1);
  int root_2 = FindArea(id_2);
  if (root_1 != root_2) {
    parent[root_2] = root_1;
  }
}

// 机器人可达
bool MapController::CanRobotReach(int x, int y) {
  return ch[x][y] == '>' || ch[x][y] == 'R' || ch[x][y] == 'B' ||
         ch[x][y] == 'C' || ch[x][y] == 'c' || ch[x][y] == '.';
}

// 船可达
bool MapController::CanBoatReach(int x, int y) {
  return ch[x][y] == '~' || ch[x][y] == '*' || ch[x][y] == 'S' ||
         ch[x][y] == 'B' || ch[x][y] == 'K' || ch[x][y] == 'C' ||
         ch[x][y] == 'c' || ch[x][y] == 'T';
}

// 初始化坐标映射到泊位id的map
void MapController::InitBerthMap(int berth_id, int berth_x, int berth_y) {
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      location_to_berth_id[Location(berth_x + i, berth_y + j)] = berth_id;
    }
  }
}
