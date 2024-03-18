#include "robot.h"

#include <cmath>
#include <iostream>

#include "berth.h"
#include "boat.h"
#include "goods.h"
#include "input_controller.h"
#include "param.h"
Robot robot[robot_num + 10];
extern Berth berth[berth_num + 10];
extern int id;
extern Boat boat[10];
Robot::Robot(int startX, int startY) {
  x = startX;
  y = startY;
}

// // 清除path
// void Robot::ClearPath() {
// #ifdef DEBUG
//   std::cerr << "ClearPath: path size ---- " << path.size() << std::endl;
// #endif
//   std::list<Location>::iterator it = path.begin();
//   while (it != path.end()) {
//     delete *it;
//   }
//   path.clear();
// }

// 删除path的第一个点
void Robot::RemoveFirst() {
  if (!path.empty()) {
    path.erase(path.begin());
  }
}

// 在头位置添加一个点
void Robot::AddFirst(int x, int y) {
  Location t(x, y);
  path.insert(path.begin(), t);
}

/*
 * 判断哪个机器人优先级高
 * @ret - 1 第一个优先级高
 * @ret - 2 第二个优先级高
 * 同等优先级默认第一个优先级高
 *
 * --- 优先级策略 ---
 * 优先级高到低：
 * - 都有货物价值高优先
 * - 有一个有货物，没货物优先
 * - 都没货物，目标货物生命周期少的优先
 * - 有人没目标货物，有目标货物的优先
 * - 都没目标货物，先判断的优先
 */
//
int Robot::JudgePriority(Robot *first, Robot *second) {
  // 如果都有货物，价值高的优先
  if (first->goods && second->goods) {
    if (first->goods_money < second->goods_money) {
      return 2;
    } else {
      return 1;
    }
  } else if (first->goods) {
    // 如果有一个有货物，没货物的优先
    // 第一个机器人有，第二个机器人没有
    return 2;
  } else if (second->goods) {
    // 如果有一个有货物，没货物的优先
    // 第一个机器人没有，第二个机器人有
    return 1;
  } else {
    // 如果都没有货物，目标货物生命低的优先
    if (first->target_goods && second->target_goods) {
      // 都有目标货物
      if (first->target_goods->birth <= second->target_goods->birth) {
        return 1;
      } else {
        return 2;
      }
    } else if (first->target_goods) {
      // 有一个有目标货物
      // 第一个机器人有目标货物,第二个机器人没有
      return 1;
    } else if (second->target_goods) {
      // 有一个有目标货物
      // 第一个机器人没有目标货物，第二个有
      return 2;
    } else {
      // 都没有目标货物
      return 1;
    }
  }
}

/*
 * 寻找目标货物
 */

void Robot::UpdateTargetGoods(int robot_id) {
  Goods *head_goods = GoodsManager::GetInstance()->head_goods;
  Goods *p_goods = GoodsManager::GetInstance()->first_free_goods;
  std::vector<Location> route;
  bool need_change_first_free_goods = true;
  Goods *find_goods = p_goods;
  int min_man = 99999;

  while (p_goods->next != head_goods) {
    if (p_goods->robot_id > -1) {
      // 该货物被选过了
      p_goods = p_goods->next;
      continue;
    }
    if (!p_goods->reachable[robot_id]) {
      // 该货物不可达
      p_goods = p_goods->next;
      continue;
    }
    int cal_man = std::abs(x - p_goods->x) + std::abs(y - p_goods->y);
    if (min_man > cal_man &&
        cal_man < LIFETIME - id + p_goods->birth - TOLERANT_TIME) {
      min_man = cal_man;
      find_goods = p_goods;
    }
    p_goods = p_goods->next;
  }
  if (find_goods->robot_id == -1) {
#ifdef DEBUG
    std::cerr << "------- start astar -------" << std::endl;
    std::cerr << "(" << x << "," << y << ")---->(" << find_goods->x << ","
              << find_goods->y << ")" << std::endl;
#endif
    Astar astar(x, y, find_goods->x, find_goods->y);
    Goods *temp_goods = find_goods;
    if (!astar.AstarSearch(route, astar_deep, temp_goods)) {
#ifdef DEBUG
      std::cerr << "route empty" << std::endl;
#endif
      //  该货物不可达
      p_goods->reachable[robot_id] = false;
    } else {
      target_goods = temp_goods;
#ifdef DEBUG
      if (target_goods == find_goods) {
        std::cerr << "same target goods" << std::endl;
      } else {
        std::cerr << "better target goods" << std::endl;
      }
#endif
      path = route;
      need_change_first_free_goods =
          find_goods == GoodsManager::GetInstance()->first_free_goods;
      if (need_change_first_free_goods) {
        while (GoodsManager::GetInstance()->first_free_goods->next !=
                   head_goods &&
               GoodsManager::GetInstance()->first_free_goods->robot_id > -1) {
          GoodsManager::GetInstance()->first_free_goods =
              GoodsManager::GetInstance()->first_free_goods->next;
        }
      }
    }
  }
}

// 存在移动后找泊位，这里不能直接用机器人的位置
void Robot::FindBerth(int start_x, int start_y) {
  int min_man_id = 0;  // 曼哈顿最小距离泊位id
  std::vector<Location> route;
  double min_man = 99999, cal_man;  // 曼哈顿距离
  bool is_final_sprint =
      id > 15000 - InputController::GetInstance()->max_transport_time -
               CHANGE_BERTH_TIME - FINAL_TOLERANT_TIME;
  // 寻找最近的泊位
  int size = berth_accessed.size();
  for (int j = 0; j < size; ++j) {
    if (is_final_sprint && berth[berth_accessed[j]].q_boat.empty() &&
        berth[berth_accessed[j]].goods_num <
            Boat::boat_capacity -
                boat[berth[berth_accessed[j]].q_boat.front()].num) {
      // 最后冲刺选有船的泊位
      continue;
    }
    cal_man = std::fabs(x - berth[berth_accessed[j]].x - 1.5) +
              std::fabs(y - berth[berth_accessed[j]].y - 1.5);
    if (min_man > cal_man) {
      min_man_id = berth_accessed[j];
      min_man = cal_man;
    }
  }

  Astar astar(start_x, start_y, berth[min_man_id].x + 1,
              berth[min_man_id].y + 1);
  astar.AstarSearch(path, berth_id, is_final_sprint);
}