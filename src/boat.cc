#include "boat.h"

#include <algorithm>
#include <iostream>

#include "berth.h"
#include "decision.h"
#include "map_controller.h"
#include "param.h"
#include "rent_controller.h"
int Boat::boat_capacity = 0;
#ifdef DEBUG
char Boat::dir_str[4][10] = {"右", "左", "上", "下"};
#endif
extern int id;

Boat::Boat() { num = 0; }
Boat::Boat(int &id, int &goods_num, int &x, int &y, int &direction, int &status)
    : id_(id),
      num(goods_num),
      x(x),
      y(y),
      direction(direction),
      status(status) {
  area_id = MapController::GetInstance()->FindArea(x * n + y, false);
}

// 顺时针旋转
void Boat::DoClockwiseRotate() {
  Decision decision(DECISION_TYPE_BOAT_ROT, id_, DECISION_BOAT_ROT_CLOCKWISE);
  DecisionManager::GetInstance()->q_decision.push(decision);
  RemoveFirst();
#ifdef DEBUG
  std::cerr << id_ << " 船顺时针旋转" << std::endl;
#endif
}

// 逆时针旋转
void Boat::DoCounterclockwiseRotate() {
  Decision decision(DECISION_TYPE_BOAT_ROT, id_,
                    DECISION_BOAT_ROT_COUNTERCLOCKWISE);
  DecisionManager::GetInstance()->q_decision.push(decision);
  RemoveFirst();
#ifdef DEBUG
  std::cerr << id_ << " 船逆时针旋转" << std::endl;
#endif
}

// 往前走
void Boat::DoShip() {
  Decision decision(DECISION_TYPE_BOAT_SHIP, id_);
  DecisionManager::GetInstance()->q_decision.push(decision);
  RemoveFirst();
#ifdef DEBUG
  std::cerr << id_ << " 船往前走(向" << dir_str[direction] << ")" << std::endl;
#endif
}

// 靠泊
void Boat::DoBerth() {
  Decision decision(DECISION_TYPE_BOAT_BERTH, id_);
  DecisionManager::GetInstance()->q_decision.push(decision);
  path.clear();
#ifdef DEBUG
  std::cerr << id_ << " 船靠泊：" << pos << std::endl;
#endif
}

// 重置到主航道
void Boat::DoDept() {
  Decision decision(DECISION_TYPE_BOAT_DEPT, id_);
  DecisionManager::GetInstance()->q_decision.push(decision);
  path.clear();
#ifdef DEBUG
  std::cerr << id_ << " 船重置到主航道" << std::endl;
#endif
}

CollisionBox::CollisionBox(int core_x, int core_y, int direction) {
  switch (direction) {
    case BOAT_DIRECTION_RIGHT:
      l_x = core_x;
      l_y = core_y;
      r_x = core_x + 1;
      r_y = core_y + 2;
      break;
    case BOAT_DIRECTION_LEFT:
      l_x = core_x - 1;
      l_y = core_y - 2;
      r_x = core_x;
      r_y = core_y;
      break;
    case BOAT_DIRECTION_UP:
      l_x = core_x - 2;
      l_y = core_y;
      r_x = core_x;
      r_y = core_y + 1;
      break;
    case BOAT_DIRECTION_DOWN:
      l_x = core_x;
      l_y = core_y - 1;
      r_x = core_x + 2;
      r_y = core_y;
      break;
    default:
#ifdef DEBUG
      std::cerr << "创建CollisionBox对象:错误的direction" << direction
                << std::endl;
#endif
      break;
  }
}

// 是否撞边界
bool CollisionBox::IsCollision() {
  if (l_x < 1 || l_y < 1 || r_x > n || r_y > n) {
    // 出界了
    return true;
  }
  for (int i = l_x; i <= r_x; ++i) {
    for (int j = l_y; j <= r_y; ++j) {
      if (!MapController::GetInstance()->CanBoatReach(i, j)) {
        return true;
      }
    }
  }
  return false;
}

// 两个对象是否相撞
bool CollisionBox::JudgeCollision(const CollisionBox &first,
                                  const CollisionBox &second) {
  if (first.l_x > second.r_x || first.r_x < second.l_x ||
      first.l_y > second.r_y || first.r_y < second.l_y) {
    return false;
  }
  return true;
}

// 删除path的第一个点
void Boat::RemoveFirst() {
  if (!path.empty()) {
    path.erase(path.begin());
  }
}

// 判断能否交货
bool Boat::DeliveryCond() { return num == Boat::boat_capacity; }

// 寻找交货点
// 判断能否交货
void Boat::FindDeliveryPoint() {
  auto &berth = MapController::GetInstance()->berth;
  // 去交货
  auto &delivery_point = MapController::GetInstance()->delivery_point;
  int delivery_id = MapController::GetInstance()->nearest_delivery[x][y];

#ifdef DEBUG
  std::cerr << "------- start astar -------" << std::endl;
  std::cerr << "(" << x << "," << y << ")---->("
            << delivery_point[delivery_id].x << ","
            << delivery_point[delivery_id].y << ")"
            << "方向:" << dir_str[direction] << std::endl;
#endif

  Astar astar(x, y, delivery_point[delivery_id].x,
              delivery_point[delivery_id].y, direction);
  astar.AstarSearch(path);
  if (pos != -1) {
    // 重置老泊位
    berth[pos].boat_id = -1;
  }
  pos = -1;

#ifdef DEBUG
  std::cerr << id_ << " 船(" << x << "," << y << "),方向："
            << dir_str[direction] << "，准备去 ("
            << delivery_point[delivery_id].x << ","
            << delivery_point[delivery_id].y
            << ") 交货, path size:" << path.size() << std::endl;
#endif
}

// 船找泊位
void Boat::FindBerth() {
  auto &berth = MapController::GetInstance()->berth;
  int size = berth.size();
  int berth_id = -1;
  int max_goods_num = 0;

  // 遍历泊位
  for (int j = 0; j < size; ++j) {
    if (berth[j].area_id != area_id || berth[j].boat_id != -1 ||
        !berth[j].goods_num) {
      continue;
    }
    // 选货物最多的
    if (berth[j].goods_num > max_goods_num) {
      max_goods_num = berth[j].goods_num;
      berth_id = j;
    }
  }
  if (berth_id > -1) {
#ifdef DEBUG
    std::cerr << "------- start astar -------" << std::endl;
    std::cerr << "(" << x << "," << y << ")---->(" << berth[berth_id].x << ","
              << berth[berth_id].y << ")"
              << "方向:" << Boat::dir_str[direction] << std::endl;
#endif

    Astar astar(x, y, berth[berth_id].x, berth[berth_id].y, direction);
    astar.AstarSearch(path);
    if (pos != -1) {
      // 重置老泊位
      berth[pos].boat_id = -1;
    }
    berth[berth_id].boat_id = id_;
    pos = berth_id;

#ifdef DEBUG
    std::cerr << id_ << " 船寻路去泊位 " << pos << ", path size:" << path.size()
              << std::endl;
#endif
  }
}

// 判断是否处于主航道
bool CollisionBox::IsLocatedOnMainRoute() {
  auto &ch = MapController::GetInstance()->ch;
  for (int i = l_x; i <= r_x; ++i) {
    for (int j = l_y; j <= r_y; ++j) {
      if (ch[i][j] == '~' || ch[i][j] == 'c' || ch[i][j] == 'T' ||
          ch[i][j] == 'K') {
        return true;
      }
    }
  }
  return false;
}