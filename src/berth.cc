#include "berth.h"

#include "map_controller.h"

Berth::Berth(int id, int x, int y, int loading_speed)
    : id_(id), x(x), y(y), loading_speed(loading_speed) {
  goods_num = 0;
  area_id = MapController::GetInstance()->FindArea(x * n + y, false);
}

// 获取该泊位的随机x坐标
int Berth::GetRandomX() {
  if (loc_index == static_cast<int>(loc.size())) {
    loc_index = 0;
  }
  return loc[loc_index++].x;
}

// 获取该泊位的随机y坐标
int Berth::GetRandomY() {
  if (loc_index == static_cast<int>(loc.size())) {
    loc_index = 0;
  }
  return loc[loc_index++].y;
}