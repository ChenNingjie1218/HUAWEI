#include "robot.h"

#include <iostream>

#include "berth.h"
#include "goods.h"
#include "param.h"
Robot robot[robot_num + 10];
extern Berth berth[berth_num + 10];
extern int id;
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

void Robot::UpdateTargetGoods() {
  double goods_weight = 0, cur_weight = 0;
  Goods *head_goods = GoodsManager::GetInstance()->head_goods;
  Goods *p_goods = GoodsManager::GetInstance()->first_free_goods;
  std::vector<Location> route;
  bool need_change_first_free_goods = true;
  // 遍历货物链表
  while (p_goods != head_goods) {
    if (p_goods->robot_id > -1) {
      // 该货物被选过了 可以改进弄一个全局指针
      // 但是有可能表前面的没有被选择，待定
      p_goods = p_goods->next;
      continue;
    }
// 调用a*算法获取路径及其长度：p_goods的坐标为终点，robot：x、y是起点
// 将长度和p_goods->money归一化加权作为权值，若大于当前权值则更新
#ifdef DEBUG
    std::cerr << "------- start astar -------" << std::endl;
    std::cerr << "(" << x << "," << y << ")---->(" << p_goods->x << ","
              << p_goods->y << ")" << std::endl;
#endif
    Astar astar(x, y, p_goods->x, p_goods->y);
    // Astar astar(36, 173, 137, 117);
    int size = route.size();
    Goods *find_goods = p_goods;
    if (!astar.AstarSearch(route, astar_deep, find_goods)) {
#ifdef DEBUG
      std::cerr << "route empty" << std::endl;
#endif
      p_goods = p_goods->next;
      need_change_first_free_goods = false;
      continue;
    } else if (size + id - p_goods->birth > LIFETIME) {
#ifdef DEBUG
      std::cerr << "can not get this good" << std::endl;
#endif
      p_goods = p_goods->next;
      need_change_first_free_goods = false;
      continue;
    }
    size = route.size();
#ifdef DEBUG
    std::cerr << "------- astar finished ------- route size:" << size
              << std::endl
              << std::endl;
    // std::vector<Location>::iterator it = route.begin();
    // for (int i = 0; i < size; ++i) {
    //   std::cerr << "(" << it->x << "," << it->y << ") -> ";
    //   ++it;
    // }
    // std::cerr << std::endl;
#endif
    cur_weight =
        0.5 * (p_goods->money - 1) / 999 - 0.5 * (size - 1) / 399.0 + 1;
    if (cur_weight > goods_weight) {
      if (p_goods == find_goods) {
        // 如果找到的货物与该货物是同一个货物
#ifdef DEBUG
        std::cerr << "same goods update path" << std::endl;
#endif
        target_goods = p_goods;
      } else {
#ifdef DEBUG
        std::cerr << "better goods update path" << std::endl;
#endif
        target_goods = find_goods;
        need_change_first_free_goods = false;
      }
      goods_weight = cur_weight;
      path = route;
      break;
    }
    p_goods = p_goods->next;
  }
  if (need_change_first_free_goods) {
    while (GoodsManager::GetInstance()->first_free_goods->next != head_goods &&
           GoodsManager::GetInstance()->first_free_goods->robot_id > -1) {
      GoodsManager::GetInstance()->first_free_goods =
          GoodsManager::GetInstance()->first_free_goods->next;
    }
  }
}

void Robot::FindBerth() {
  int length = 0, fin_length = 50000, fin_j = 0;
  std::vector<Location> route;

  // 寻找最近的泊位
  for (int j = 0; j < 10; j++) {
    if (berth[j].q_boat.empty()) {
      continue;
    }
    // Robot::ClearPath(route);  // 清空上一次计算的路径
    Astar astar(x, y, berth[j].x + 1, berth[j].y + 1);
    Goods *find_goods = nullptr;
    if (astar.AstarSearch(route, astar_deep, find_goods)) {
      length = route.size();
      if (length < fin_length) {
        fin_length = length;
        path = route;  //更新机器人的行动路径
        fin_j = j;
      }
    }
  }
  berth_id = fin_j;
}