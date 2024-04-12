#include "berth.h"

#include <cstdlib>

#include "map_controller.h"

Berth::Berth(int id, int x, int y, int loading_speed)
    : id_(id), x(x), y(y), loading_speed(loading_speed) {
  goods_num = 0;
  area_id = MapController::GetInstance()->FindArea(x * n + y, false);
  int delivery_index = MapController::GetInstance()->nearest_delivery[x][y];
  auto& delivery_point = MapController::GetInstance()->delivery_point;
  transport_time = std::abs(x - delivery_point[delivery_index].x) +
                   std::abs(y - delivery_point[delivery_index].y);
}

// 获取该泊位最近的x坐标
int Berth::GetNearestX(const int& x) {
  int size = loc.size();
  int min_x_i = -1;
  int min_x = 200;
  for (int i = 0; i < size; ++i) {
    int cal_x = std::abs(x - loc[i].x);
    if (cal_x < min_x) {
      min_x = cal_x;
      min_x_i = i;
    }
  }
  return loc[min_x_i].x;
}

// 获取该泊位最近的y坐标
int Berth::GetNearestY(const int& y) {
  int size = loc.size();
  int min_y_i = -1;
  int min_y = 200;
  for (int i = 0; i < size; ++i) {
    int cal_y = std::abs(y - loc[i].y);
    if (cal_y < min_y) {
      min_y = cal_y;
      min_y_i = i;
    }
  }
  return loc[min_y_i].y;
}

// 获取船到该泊位能装多少钱
int Berth::GetIdealMoney(const int& capacity) {
  int can_load = std::min(capacity, goods_num);
  int sum_money = 0;
  for (int i = 0; i < can_load; ++i) {
    sum_money += goods[first_index + i];
  }
  return sum_money;
}