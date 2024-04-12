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

bool Robot::FindTargetGoods() {
  auto &berth = MapController::GetInstance()->berth;
  Goods *head_goods = berth[berth_id].goods_manager.head_goods;
  Goods *p_goods = berth[berth_id].goods_manager.first_free_goods;
  std::vector<Location> route;
  bool need_change_first_free_goods = true;
  Goods *find_goods = nullptr;

#ifdef MONEY_FIRST
  double max_per_money = 0;
#else
  int min_man = 99999;
#endif
  if (is_sprint) {
    // 表明是冲刺阶段，要全局搜索货物
    // 循环遍历所有港口
    int berth_size = berth.size();
    for (int i = 0; i < berth_size; ++i) {
      if (berth[i].goods_manager.goods_num == 0) {
        // 港口货物列表为空
        continue;
      }
      head_goods = berth[i].goods_manager.head_goods;
      p_goods = berth[i].goods_manager.first_free_goods;
      while (p_goods->next != head_goods) {
        if (p_goods->robot_id > -1) {
          // 该货物被选过了
          p_goods = p_goods->next;
          continue;
        }
        if (p_goods->area_id != area_id) {
          // 不在一个分区
          p_goods = p_goods->next;
          continue;
        }
        int cal_man = std::abs(x - p_goods->x) + std::abs(y - p_goods->y);
        int total_cal_man =
            cal_man +
            std::abs(berth[berth_id].GetNearestX(p_goods->x) - p_goods->x) +
            std::abs(berth[berth_id].GetNearestY(p_goods->y) - p_goods->y);
        // total_cal_man = cal_man; // 屏蔽精确长度

        double per_money = 1.0 * p_goods->money / total_cal_man;
#ifdef MONEY_FIRST
        if (per_money > max_per_money &&
            cal_man < LIFETIME - id + p_goods->birth -
                          DynamicParam::GetInstance()->GetTolerantTime()) {
          max_per_money = per_money;
#else
        if (min_man > cal_man &&
            cal_man < LIFETIME - id + p_goods->birth -
                          DynamicParam::GetInstance()->GetTolerantTime()) {
          min_man = cal_man;
#endif

          find_goods = p_goods;
        }
        p_goods = p_goods->next;
      }
    }
  } else {
    while (p_goods != head_goods) {
      if (p_goods->robot_id > -1) {
        // 该货物被选过了
        p_goods = p_goods->next;
        continue;
      }
      if (p_goods->area_id != area_id) {
        // 不在一个分区
        p_goods = p_goods->next;
        continue;
      }
      int cal_man = std::abs(x - p_goods->x) + std::abs(y - p_goods->y);
      int total_cal_man =
          cal_man +
          std::abs(berth[berth_id].GetNearestX(p_goods->x) - p_goods->x) +
          std::abs(berth[berth_id].GetNearestY(p_goods->y) - p_goods->y);
      // total_cal_man = cal_man; // 屏蔽精确长度

      double per_money = 1.0 * p_goods->money / total_cal_man;
#ifdef MONEY_FIRST
      if (per_money > max_per_money &&
          cal_man < LIFETIME - id + p_goods->birth -
                        DynamicParam::GetInstance()->GetTolerantTime()) {
        max_per_money = per_money;
#else
      if (min_man > cal_man) {
        min_man = cal_man;
#endif
        find_goods = p_goods;
      }
      p_goods = p_goods->next;
    }
  }

  std::set<int> &neighbor = berth[berth_id].neighbor;
  //遍历邻居
  for (const auto &neighbor_id : neighbor) {
#ifdef DEBUG
    std::cerr << "邻居" << neighbor_id << std::endl;
#endif
    Goods *p_goods = berth[neighbor_id].goods_manager.first_free_goods;
    head_goods = berth[neighbor_id].goods_manager.head_goods;
    while (p_goods != head_goods) {
      if (p_goods->robot_id > -1) {
        // 该货物被选过了
        p_goods = p_goods->next;
        continue;
      }
      if (p_goods->area_id != area_id) {
        // 不在一个分区
        p_goods = p_goods->next;
        continue;
      }
#ifdef MONEY_FIRST
      int find_neighbor_max_robot =
          DynamicParam::GetInstance()->GetFindNeighborMaxRobot();
      double avr_money_differential =
          DynamicParam::GetInstance()->GetAvrMoneyDifferential();
      int cal_man = std::abs(x - p_goods->x) + std::abs(y - p_goods->y);
      int total_cal_man =
          cal_man +
          std::abs(berth[neighbor_id].GetNearestX(p_goods->x) - p_goods->x) +
          std::abs(berth[neighbor_id].GetNearestY(p_goods->y) - p_goods->y);

      double per_money = 1.0 * p_goods->money / total_cal_man;
      if ((per_money - max_per_money) >
              avr_money_differential &&              // 金钱/长度的差值
          cal_man < (1000 - id + p_goods->birth) &&  // 生存周期内
          berth[neighbor_id].robot.size() <
              find_neighbor_max_robot) {  // 前往的泊位机器人最多数量
#ifdef DEBUG
        std::cerr << max_per_money << "--升值-->"
                  << 1.0 * p_goods->money / cal_man << std::endl;
#endif
        max_per_money = p_goods->money;
        find_goods = p_goods;
      }
#else
      int cal_man = std::abs(x - p_goods->x) + std::abs(y - p_goods->y);
      if (min_man > cal_man) {
        min_man = cal_man;
        find_goods = p_goods;
      }
#endif
      p_goods = p_goods->next;
    }
  }

  if (find_goods != nullptr && find_goods->robot_id == -1) {
    // 若货物在邻居分区则先更换分区
    int goods_berth_id = MapController::GetInstance()
                             ->nearest_berth[find_goods->x][find_goods->y];
    if (goods_berth_id != berth_id) {
      ChangeBerth(goods_berth_id);
#ifdef DEBUG
      std::cerr << "找到邻居好货：(" << find_goods->x << "," << find_goods->y
                << "),价值：" << find_goods->money << std::endl;
#endif
    }
    head_goods = berth[goods_berth_id].goods_manager.head_goods;

    find_goods->robot_id = id_;
    if (FindPath(find_goods)) {
      need_change_first_free_goods =
          target_goods == berth[berth_id].goods_manager.first_free_goods;
      if (need_change_first_free_goods) {
        while (berth[berth_id].goods_manager.first_free_goods->next !=
                   head_goods &&
               berth[berth_id].goods_manager.first_free_goods->robot_id > -1) {
          berth[berth_id].goods_manager.first_free_goods =
              berth[berth_id].goods_manager.first_free_goods->next;
        }
      }

      return true;
    }
    find_goods->robot_id = -1;
  }

  return false;
}

// 存在移动后找泊位，这里不能直接用机器人的位置
void Robot::FindBerth(int start_x, int start_y) {
  // 判断机器人该不该冲刺，选择船中时间最大的
  auto &boat = RentController::GetInstance()->boat;
  auto &berth = MapController::GetInstance()->berth;
  auto size = RentController::GetInstance()->robot.size();

  // 冲刺
  if (id > 13000) {
    // SetDash();
  }
  // // 遍历所有的港口，寻找回家时间最短的港口
  // std::multimap<int, int> time_map;  // 第一个参数是时间，第二个参数是泊位id
  // if (!boat.empty() && !is_sprint) {
  //   int berth_size = berth.size();
  //   for (int i = 0; i < berth_size; ++i) {
  //     time_map.insert(std::make_pair(berth[i].transport_time, i));
  //   }
  //   for (auto it = time_map.begin(); it != time_map.end(); ++it) {
  //     if (berth[it->second].robot.size() < size / boat.size()) {
  //       // 可以选择这个泊位
  //       int sprint_time =
  //           it->first * 3 +
  //           DynamicParam::GetInstance()->GetFinalTolerantTime();
  //       if (15000 - id < sprint_time) {
  //         // 可以冲刺
  //         ChangeBerth(it->second);
  //         is_sprint = true;
  //         is_sprint = false;
  //       }
  //       break;
  //     }
  //   }
  // }
  if (berth_id != -1) {
#ifdef DEBUG
    std::cerr << "------- start astar -------" << std::endl;
    std::cerr << "(" << start_x << "," << start_y << ")---->("
              << berth[berth_id].x << "," << berth[berth_id].y << ")"
              << std::endl;
#endif
    Astar astar(start_x, start_y, berth[berth_id].GetNearestX(start_x),
                berth[berth_id].GetNearestY(start_y));
    astar.AstarSearch(path, berth_id);
#ifdef DEBUG
    std::cerr << "------- astar finished -------" << std::endl;
    std::cerr << "目标泊位：" << berth_id << " path size:" << path.size()
              << std::endl;
#endif
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
  for (int i = 1; i <= 4; ++i) {
    int index = (first_dir + i) % 4;
    if (x + DIRS[index].x == ignore_x && y + DIRS[index].y == ignore_y) {
      // 放弃ignore方向移动
      continue;
    }
    if (MapController::GetInstance()->CanRobotReach(x + DIRS[index].x,
                                                    y + DIRS[index].y)) {
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

// 机器人分区规划
bool Robot::ZonePlan() {
  if (is_sprint) return false;
#ifdef DEBUG
  std::cerr << "机器人" << id_ << "决定寻找负荷较大泊位" << std::endl;
#endif
  std::vector<Berth> &berth = MapController::GetInstance()->berth;
  if (berth[berth_id].robot.size() > 0) {
    int temp_berth_id = -1;
    double max_avg_goods_num = 0, temp;
    int size = MapController::GetInstance()->berth.size();

    // 寻找合适泊位
    for (int i = 0; i < size; ++i) {
      if (berth[berth_id].neighbor.find(i) == berth[berth_id].neighbor.end()) {
        // 不是邻居
        continue;
      }
      // 将机器人手上的货物也纳入考虑
      int count_hand = 0;
      std::vector<Robot> &robot = RentController::GetInstance()->robot;
      int robot_size = berth[i].robot.size();
      for (int k = 0; k < robot_size; k++) {
        if (robot[berth[i].robot[k]].goods == 1) {
          count_hand++;
        }
      }
      // 计算邻居泊位权值选择最优
      temp = (berth[i].goods_manager.goods_num + count_hand) * 1.0 /
             (berth[i].robot.size() ? berth[i].robot.size() : 0.01);
      if (temp > max_avg_goods_num &&
          (berth[i].goods_manager.goods_num - robot_size + count_hand >= 1)) {
        max_avg_goods_num = temp;
        temp_berth_id = i;
      }
    }

    // 机器人换去新泊位
    if (temp_berth_id != -1) {
      int old_berth_id = berth_id;
      for (std::vector<int>::iterator it = berth[old_berth_id].robot.begin();
           it != berth[old_berth_id].robot.end(); ++it) {
        if (*it == id_) {
          berth[old_berth_id].robot.erase(it);
          break;
        }
      }
      berth_id = temp_berth_id;
      berth[temp_berth_id].robot.push_back(id_);
#ifdef DEBUG
      std::cerr << "机器人" << id_ << "从泊位" << old_berth_id << "更换至泊位"
                << berth_id << std::endl;
#endif
      return true;
    }
#ifdef DEBUG
    std::cerr << "机器人" << id_ << "找不到负荷较大泊位" << std::endl;
#endif
  }
  return false;
}

// bfs寻找最近货物
void Robot::FindNeighborGoods() {}

// 机器人更换泊位
void Robot::ChangeBerth(int new_berth_id) {
  std::vector<Berth> &berth = MapController::GetInstance()->berth;
  int old_berth_id = berth_id;
  for (std::vector<int>::iterator it = berth[old_berth_id].robot.begin();
       it != berth[old_berth_id].robot.end(); ++it) {
    if (*it == id_) {
      berth[old_berth_id].robot.erase(it);
      break;
    }
  }
  berth_id = new_berth_id;
  berth[new_berth_id].robot.push_back(id_);
#ifdef DEBUG
  std::cerr << "机器人" << id_ << "从泊位" << old_berth_id << "更换至泊位"
            << berth_id << std::endl;
#endif
}

/*
 * 找路径
 */
bool Robot::FindPath(Goods *&find_goods) {
  Astar astar(x, y, find_goods->x, find_goods->y);
  Goods *temp_goods = find_goods;
  std::vector<Location> temp_path;
#ifdef DEBUG
  std::cerr << "------- start astar -------" << std::endl;
  std::cerr << "(" << x << "," << y << ")---->(" << temp_goods->x << ","
            << temp_goods->y << ")" << std::endl;
#endif
  if (astar.AstarSearch(temp_path, temp_goods)) {
#ifdef DEBUG
    std::cerr << "------- astar finished -------" << std::endl;
    if (find_goods == temp_goods) {
      std::cerr << "same target goods" << std::endl;
    } else {
      std::cerr << "better target goods" << std::endl;
    }
#endif
    target_goods = temp_goods;
    target_goods->robot_id = id_;
    path = temp_path;
    return true;
  } else {
#ifdef DEBUG
    std::cerr << "route empty" << std::endl;
#endif
    return false;
  }
}

// 设置机器人冲刺
void Robot::SetDash() {
  std::vector<std::pair<int, int>> dash_table =
      MapController::GetInstance()->dash_table;
  bool &is_dash = MapController::GetInstance()->is_dash;

  // 还未冲刺且达到时间点
  if (!is_sprint && id >= dash_table[id_].first) {
    ChangeBerth(dash_table[id_].second);
    is_sprint = true;

    // 只要有机器人冲刺就将在最终泊位的机器人也置为冲刺
    if (is_dash) {
      MapController::GetInstance()->KeepOrigin();
    }
  }
}