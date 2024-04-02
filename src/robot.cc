#include "robot.h"

#include <cmath>
#include <iostream>

#include "berth.h"
#include "boat.h"
#include "goods.h"
#include "map_controller.h"
#include "param.h"
#include "rent_controller.h"
extern int id;
Robot::Robot(int &id, int &goods, int &startX, int &startY)
    : id_(id), x(startX), y(startY), goods(goods) {
  area_id = MapController::GetInstance()->FindArea(x * n + y);
}

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

bool Robot::UpdateTargetGoods(int robot_id) {
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
    if (p_goods->area_id != area_id) {
      // 不在一个分区
      p_goods = p_goods->next;
      continue;
    }
    int cal_man = std::abs(x - p_goods->x) + std::abs(y - p_goods->y);
    if (min_man > cal_man) {
      min_man = cal_man;
      find_goods = p_goods;
    }
    p_goods = p_goods->next;
  }
  if (min_man < 500 && find_goods->robot_id == -1 &&
      find_goods->reachable[robot_id]) {
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
      find_goods->reachable[robot_id] = false;
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
      return true;
    }
  }
  return false;
}

// 存在移动后找泊位，这里不能直接用机器人的位置
void Robot::FindBerth(int start_x, int start_y) {
  auto &berth = MapController::GetInstance()->berth;
  berth_id = MapController::GetInstance()->nearest_berth[start_x][start_y];
  if (berth_id != -1) {
    Astar astar(start_x, start_y, berth[berth_id].x, berth[berth_id].y);
    astar.AstarSearch(path, berth_id);
    std::cerr << "berth_id:" << berth_id << std::endl;
  } else {
#ifdef DEBUG
    std::cerr << "机器人找目标泊位id不合法" << berth_id << std::endl;
#endif
  }
}

/*
 * 机器人是否拦路
 * @ret next_points的下标
 */
int Robot::IsBlock(std::vector<NextPoint> &next_points) {
  int size = next_points.size();
  for (int i = 0; i < size; ++i) {
    if (next_points[i].count && x == next_points[i].x &&
        y == next_points[i].y) {
      return i;
    }
  }
  return -1;
}

/*
 * 让路
 * @ret 让路方向： DIR 数组下标
 * @bref 只考虑空闲位
 */
int Robot::GetAway(std::vector<NextPoint> &next_points, int ignore_id,
                   std::vector<int> &not_move_id) {
  // 可能存在不能让位的情况
  std::vector<Robot> &robot = RentController::GetInstance()->robot;
  int size = next_points.size();
  int ignore_x = robot[ignore_id].x, ignore_y = robot[ignore_id].y;

  // 看周围有没有不被占用的空位
  int first_dir;
  std::array<Location, 4> &DIRS = MapController::GetInstance()->DIRS;
  for (first_dir = 0; first_dir < 4; ++first_dir) {
    if (ignore_x + DIRS[first_dir].x == x &&
        ignore_y + DIRS[first_dir].y == y) {
      break;
    }
  }

#ifdef DEBUG
  std::vector<std::string> dir_string = {"右", "左", "上", "下"};
#endif
  char(&ch)[N][N] = MapController::GetInstance()->ch;
  for (int i = 1; i <= 4; ++i) {
    int index = (first_dir + i) % 4;
    if (x + DIRS[index].x == ignore_x && y + DIRS[index].y == ignore_y) {
      // 放弃ignore方向移动
      continue;
    }
    if (ch[x + DIRS[index].x][y + DIRS[index].y] == '.' ||
        ch[x + DIRS[index].x][y + DIRS[index].y] == 'A' ||
        ch[x + DIRS[index].x][y + DIRS[index].y] == 'B') {
      bool can_leave = true;
      for (int k = 0; k < size; ++k) {
        if (x + DIRS[index].x == next_points[k].x &&
            y + DIRS[index].y == next_points[k].y) {
          // 位置即将被某个机器人占用
          can_leave = false;
          break;
        }
      }
      if (can_leave) {
        // 检测是否被占用
        for (std::vector<int>::iterator it = not_move_id.begin();
             it != not_move_id.end(); ++it) {
          if (robot[*it].x == x + DIRS[index].x &&
              robot[*it].y == y + DIRS[index].y) {
            can_leave = false;
            break;
          }
        }
        if (can_leave) {
          // 有空位的情况
#ifdef DEBUG
          std::cerr << "向" << dir_string[index] << "让位" << std::endl;
#endif
          // 增加新落点
          NextPoint add_next_point =
              NextPoint(x + DIRS[index].x, y + DIRS[index].y, id_);
          next_points.push_back(add_next_point);
          // 将让的位加入让位机器的人path
          AddFirst(x, y);
          return index;
        }
      }
    }
  }
#ifdef DEBUG
  std::cerr << "!!!!!!!!!!!!!!!!!!!! Can not get away !!!!!!!!!!!!!!!!!!!!"
            << std::endl;
#endif
  return -1;
}
