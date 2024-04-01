#include "berth.h"

#include "map_controller.h"

Berth::Berth(int id, int x, int y, int loading_speed)
    : id_(id), x(x), y(y), loading_speed(loading_speed) {
  goods_num = 0;
  area_id = MapController::GetInstance()->FindArea(x * n + y);
}
