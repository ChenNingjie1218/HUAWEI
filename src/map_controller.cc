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