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
bool Boat::DeliveryCond() {
  if (pos == -1) {
    return false;
  }
  std::vector<Berth> &berth = MapController::GetInstance()->berth;
  if (id > 15000 - berth[pos].transport_time -
               DynamicParam::GetInstance()->GetTolerantLeaveTime()) {
#ifdef DEBUG
    std::cerr << berth[pos].boat_id
              << " 船离开了，剩余货物数量: " << berth[pos].goods_num
              << " 船货物数量: " << num << std::endl;
#endif
    return true;
  }
  if (berth[pos].goods_num > 0) {
    // 能装满就装满了走
    return num >= boat_capacity;
  }
  // 微调快满的船又去换泊位
  return num >=
         boat_capacity - DynamicParam::GetInstance()->GetBoatCapacityReduce();
}

// 寻找交货点
// 判断能否交货
void Boat::FindDeliveryPoint() {
  // 去交货
  auto &berth = MapController::GetInstance()->berth;
  if (pos != -1 && berth[pos].path.find(-1) != berth[pos].path.end()) {
    path = berth[pos].path[-1];
  } else {
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
      berth[pos].path[-1] = path;
      berth[pos].transport_time = path.size();
    }
  }

  // 重置老泊位
  if (pos != -1) {
    berth[pos].boat_id = -1;
  }
  pos = -1;

#ifdef DEBUG
  std::cerr << id_ << " 船去交货, path size:" << path.size() << std::endl;
#endif
}

/*
 * 船找泊位
 * todo: 在非泊位点不准确的can_load也许会出问题
 */
void Boat::FindBerth() {
  auto &berth = MapController::GetInstance()->berth;
  int size = berth.size();
  int berth_id = -1;
  int max_goods_num = 0;

  Location loc(x, y);
  int now_berth_id = -1;
  auto &location_to_berth_id =
      MapController::GetInstance()->location_to_berth_id;
  if (location_to_berth_id.find(loc) != location_to_berth_id.end()) {
    now_berth_id = location_to_berth_id[loc];
  }
  // 先看有没有泊位能让自己填满
  for (int i = 0; i < size; ++i) {
    int time = 15000 - id - berth[i].transport_time -
               DynamicParam::GetInstance()->GetTolerantLeaveTime();
    if (now_berth_id != -1 &&
        berth[now_berth_id].path.find(i) != berth[now_berth_id].path.end()) {
      // 当前处于某泊位上 且 存了该泊位到泊位i的路径
      time -= berth[now_berth_id].path[i].size();  // 修正走过去的时间
    }

    int goods_num = std::min(time * berth[i].loading_speed, berth[i].goods_num);
    if (i != pos && goods_num >= boat_capacity - num &&
        berth[i].boat_id == -1) {
      // 泊位i可以把该船装满
      if (berth_id == -1 ||
          berth[i].transport_time < berth[berth_id].transport_time) {
        // 选离家近的
        berth_id = i;
      }
    }
  }

  if (berth_id == -1) {
    // 没有能装满该船的泊位
    for (int i = 0; i < size; ++i) {
      if (berth[i].area_id != area_id || berth[i].boat_id != -1 ||
          !berth[i].goods_num) {
        continue;
      }

      int time = 15000 - id - berth[i].transport_time -
                 DynamicParam::GetInstance()->GetTolerantLeaveTime();
      if (now_berth_id != -1 &&
          berth[now_berth_id].path.find(i) != berth[now_berth_id].path.end()) {
        // 当前处于某泊位上 且 存了该泊位到泊位i的路径
        time -= berth[now_berth_id].path[i].size();  // 修正走过去的时间
      }
      int can_load =
          std::min(time * berth[i].transport_time, berth[i].goods_num);

      // 选货物最多的
      if (can_load > max_goods_num) {
        max_goods_num = berth[i].goods_num;
        berth_id = i;
      }
    }
  }

  if (berth_id > -1) {
    // 判断是否处于泊位中
    auto &location_to_berth_id =
        MapController::GetInstance()->location_to_berth_id;
    Location loc(x, y);
    if (location_to_berth_id.find(loc) != location_to_berth_id.end()) {
      int now_berth_id = location_to_berth_id[loc];
      // 目前处于某泊位中
      if (berth[now_berth_id].path.find(berth_id) !=
          berth[now_berth_id].path.end()) {
        // 路径已经算过了
        path = berth[now_berth_id].path[berth_id];
      } else {
        // 路径没算过
#ifdef DEBUG
        std::cerr << "------- start astar -------" << std::endl;
        std::cerr << "(" << x << "," << y << ")---->(" << berth[berth_id].x
                  << "," << berth[berth_id].y << ")"
                  << "方向:" << Boat::dir_str[direction] << std::endl;
#endif
        Astar astar(x, y, berth[berth_id].x, berth[berth_id].y, direction);
        astar.AstarSearch(path);
        berth[now_berth_id].path[berth_id] = path;
      }
    } else {
      // 目前不处于泊位中

#ifdef DEBUG
      std::cerr << "------- start astar -------" << std::endl;
      std::cerr << "(" << x << "," << y << ")---->(" << berth[berth_id].x << ","
                << berth[berth_id].y << ")"
                << "方向:" << Boat::dir_str[direction] << std::endl;
#endif
      Astar astar(x, y, berth[berth_id].x, berth[berth_id].y, direction);
      astar.AstarSearch(path);
    }

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