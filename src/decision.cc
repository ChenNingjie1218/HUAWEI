#include "decision.h"

#include <iostream>
#include <queue>
#include <vector>

#include "astar.h"
#include "berth.h"
#include "boat.h"
#include "param.h"
#include "robot.h"
extern Boat boat[10];
extern Robot robot[robot_num + 10];
extern Berth berth[berth_num + 10];
extern char ch[N][N];
extern Goods *gds[N][N];
extern int busy_point[N][N];
extern int id;
extern std::array<Location, 4> DIRS;
bool can_find_goods = true;  // 每次决策只找一次物品
DecisionManager *DecisionManager::instance_ = nullptr;

Decision::Decision(int type, int id, int param) {
  this->type = type;
  this->id = id;
  this->param = param;
}

DecisionManager *&DecisionManager::GetInstance() {
  if (!instance_) {
    instance_ = new DecisionManager();
  }
  return instance_;
}

// 清空决策队列
void DecisionManager::ClearQueue() {
  std::queue<Decision> empty;
  swap(empty, q_decision);
}

/*
 * 船做决策
 * 根据帧数据状态来决策
 */
void DecisionManager::DecisionBoat() {
#ifdef DEBUG
  std::cerr << "-----------------------------------Boat-----------"
               "--------------------------"
            << std::endl;
#endif
  //最大权重泊位，权重都为0就随机泊位
  // int rand_berth = 0;
  // 按船装的货物数量顺序来抉择
  int boat_id[5];
  boat_id[0] = 0;
  for (int i = 1; i < 5; ++i) {
    bool flag = true;
    for (int j = i - 1; j >= 0; --j) {
      if (boat[i].num > boat[boat_id[j]].num) {
        boat_id[j + 1] = boat_id[j];
      } else {
        boat_id[j + 1] = i;
        flag = false;
        break;
      }
    }
    if (flag) {
      boat_id[0] = i;
    }
  }

#ifdef DEBUG
  std::cerr << "船按货物数量排完序" << std::endl;
#endif

  for (int i = 0; i < 5; ++i) {
    // status 0 运输中 无需考虑决策
    if (boat[boat_id[i]].status == 1) {
      if (boat[boat_id[i]].pos == -1) {
        // 在虚拟点

        // 决策去哪个泊位
        boat[boat_id[i]].ChooseBerth3(boat_id[i]);
        if (boat[boat_id[i]].pos == -1) {
          continue;
        }
#ifdef DEBUG
        std::cerr << "boat " << boat_id[i]
                  << " choose berth:" << boat[boat_id[i]].pos << std::endl;
#endif
        Decision decision(DECISION_TYPE_BOAT_SHIP, boat_id[i],
                          boat[boat_id[i]].pos);
        q_decision.push(decision);  // 决策入队
      } else if (boat[boat_id[i]].LeaveCond()) {
        if (!berth[boat[boat_id[i]].pos].q_boat.empty()) {
          berth[boat[boat_id[i]].pos].q_boat.pop();
        }
        berth[boat[boat_id[i]].pos].boat_id = -1;
        // 决策是否驶离

#ifdef DEBUG
        std::cerr << "船 " << boat_id[i] << " 离开 " << boat[boat_id[i]].pos
                  << " 货物数量: " << boat[boat_id[i]].num << std::endl;
#endif
        Decision decision(DECISION_TYPE_BOAT_GO, boat_id[i], -1);
        q_decision.push(decision);
        boat[boat_id[i]].num = 0;  // 清空船中货物
      } else if (boat[boat_id[i]].ChangeBerth3(boat_id[i])) {  // 更换港口
        // 更换港口
        Decision decision(DECISION_TYPE_BOAT_SHIP, boat_id[i],
                          boat[boat_id[i]].pos);
        q_decision.push(decision);  // 决策入队
      }

    } else if (boat[boat_id[i]].status == 2) {
      // 在等待
      // 可以决策是否换船舶，换哪个船舶

#ifdef DEBUG
      std::cerr << "boat " << boat_id[i] << " is waiting "
                << boat[boat_id[i]].pos << std::endl;
#endif
    }
  }
}

// 让位
// 只考虑空闲位
bool DecisionManager::GetAway(int robot_id, std::vector<NextPoint> &next_points,
                              int ignore_id, std::vector<int> &not_move_id) {
#ifdef DEBUG
  std::cerr << "robot " << robot_id << " 让位给 " << ignore_id << std::endl;
#endif
  // 可能存在不能让位的情况

  int size = next_points.size();
  int x = robot[robot_id].x, y = robot[robot_id].y,
      ignore_x = robot[ignore_id].x, ignore_y = robot[ignore_id].y;

  // 1.看周围有没有不被占用的空位
  int first_dir;
  for (first_dir = 0; first_dir < 4; ++first_dir) {
    if (ignore_x + DIRS[first_dir].x == x &&
        ignore_y + DIRS[first_dir].y == y) {
      break;
    }
  }

#ifdef DEBUG
  std::vector<std::string> dir_string = {"下", "上", "右", "左"};
#endif
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
              NextPoint(x + DIRS[index].x, y + DIRS[index].y, robot_id);
          next_points.push_back(add_next_point);
          // 将让的位加入让位机器的人path
          robot[robot_id].AddFirst(x, y);
          return true;
        }
      }
    }
  }

#ifdef DEBUG
  std::cerr << "!!!!!!!!!!!!!!!!!!!! Can not get away !!!!!!!!!!!!!!!!!!!!"
            << std::endl;
#endif
  return false;
}

/*
 * 机器人做决策
 * 移动后的动作决策建立在成功移动后
 * 所以移动前动作和移动放一个循环判断
 * 移动后动作单独判断做了移动决策的机器人
 *
 */
void DecisionManager::DecisionRobot() {
  can_find_goods = true;
#ifdef DEBUG
  std::cerr << "-----------------------------------Robot-----------"
               "--------------------------"
            << std::endl;
#endif
  std::vector<NextPoint> next_points;
  std::vector<int> not_move_id;
  for (int i = 0; i < 10; ++i) {
    // --------- 移动前动作 ---------
    // if (robot[i].goods && ch[robot[i].x][robot[i].y] == 'B' &&
    //     !berth[robot[i].berth_id].q_boat.empty()) {
    if (robot[i].goods && ch[robot[i].x][robot[i].y] == 'B') {
#ifdef DEBUG
      std::cerr << "robot " << i << " 卸货：(" << robot[i].x << ","
                << robot[i].y << ")" << std::endl;
#endif
      // 卸货
      Decision decision(DECISION_TYPE_ROBOT_PULL, i, -1);
      q_decision.push(decision);

      // 增加泊位权重
      ++berth[robot->berth_id].weight;
#ifdef ONE_ROBOT_ONE_BERTH
      berth[robot->berth_id].robot_id = -1;
#endif
      // robot[i].berth_id = -1;
      // 下货的位置可能不是计算的位置，path里面还有内容
      robot[i].path.clear();
      // 决策，更新目标货物, 当前不持有货物
#ifdef DEBUG
      std::cerr << "robot " << i << " start UpdateTargetGoods" << std::endl;
#endif
      if (can_find_goods && robot[i].UpdateTargetGoods(i)) {
        can_find_goods = false;
      }

#ifdef DEBUG
      std::cerr << "robot " << i << " finished UpdateTargetGoods after pull"
                << std::endl;
#endif
      if (!robot[i].path.empty()) {
        // 有新的目标货物

#ifdef DEBUG
        std::cerr << "robot " << i << " start UpdateTargetGoods" << std::endl;
#endif
        robot[i].target_goods->robot_id = i;
      }
      robot[i].goods = false;
    } else if (!robot[i].goods && gds[robot[i].x][robot[i].y]) {
#ifdef DEBUG
      std::cerr << "地上有货物" << std::endl;
#endif
      bool is_get = false;
      // 机器人没拿货且地上有货物
      if (robot[i].target_goods &&
          gds[robot[i].x][robot[i].y] == robot[i].target_goods) {
        // 地上的货物为机器人准备捡的货物
        is_get = true;
#ifdef DEBUG
        std::cerr << "robot " << i << " 装目标货：(" << robot[i].x << ","
                  << robot[i].y << ")" << std::endl;
#endif
        // } else if (robot[i].target_goods &&
        //            gds[robot[i].x][robot[i].y]->money > VALUEABLE_GOODS_VALVE
        //            && gds[robot[i].x][robot[i].y]->robot_id == -1) {
      }
#ifdef CAN_GRAB_GOODS
      else if (robot[i].target_goods &&
               gds[robot[i].x][robot[i].y]->money > VALUEABLE_GOODS_VALVE) {
        // 有目标货物的情况下，路过的地上有贵重货物
        is_get = true;
        // 放弃原目标货物
        if (gds[robot[i].x][robot[i].y]->robot_id != -1) {
          // 有机器人选了这个目标货物
          // 给他抢了
          robot[gds[robot[i].x][robot[i].y]->robot_id].target_goods = nullptr;
        }
        robot[i].target_goods->robot_id = -1;
        robot[i].target_goods = gds[robot[i].x][robot[i].y];
        robot[i].target_goods->robot_id = i;

        // 还原first_free_goods指针
        Goods *head_goods = GoodsManager::GetInstance()->head_goods;
        GoodsManager::GetInstance()->first_free_goods = head_goods->next;
        while (GoodsManager::GetInstance()->first_free_goods->next !=
                   head_goods &&
               GoodsManager::GetInstance()->first_free_goods->robot_id > -1) {
          GoodsManager::GetInstance()->first_free_goods =
              GoodsManager::GetInstance()->first_free_goods->next;
        }

#ifdef DEBUG
        std::cerr << "robot " << i << " 装路过的高价货：(" << robot[i].x << ","
                  << robot[i].y << ")" << std::endl;
#endif
      }
#endif
      else if (!robot[i].target_goods) {
        // 机器人没目标货物，且该帧刚好生成了一个货物在脚底下
        is_get = true;
        robot[i].target_goods = gds[robot[i].x][robot[i].y];
        robot[i].target_goods->robot_id = i;
#ifdef DEBUG
        std::cerr << "robot " << i << " 装地上新生成的货：(" << robot[i].x
                  << "," << robot[i].y << ")" << std::endl;
#endif
      }
      if (is_get) {
        // 装货
        Decision decision(DECISION_TYPE_ROBOT_GET, i, -1);
        q_decision.push(decision);

        // 决策更新目标泊位和泊位权重
        robot[i].FindBerth(robot[i].x, robot[i].y);
        robot[i].out_maze = false;
        // berth_weight[robot[i].berth_id]++;
#ifdef DEBUG
        std::cerr << "成功更新目标泊位" << std::endl;
#endif
        // 捡到货物将其从链表删除
        GoodsManager::GetInstance()->DeleteGoods(robot[i].target_goods);

        //当前持有货物
        robot[i].goods = true;
        robot[i].pre_goods = true;
#ifdef ONE_ROBOT_ONE_BERTH
        if (!robot[i].path.empty()) {
          berth[robot[i].berth_id].robot_id = i;
        }
#endif
      }
    }
    if (robot[i].goods && robot[i].path.empty()) {
      // 如果有货物但是没路径
      robot[i].FindBerth(robot[i].x, robot[i].y);
      robot[i].out_maze = false;
#ifdef ONE_ROBOT_ONE_BERTH
      if (!robot[i].path.empty()) {
        berth[robot[i].berth_id].robot_id = i;
      }
#endif
    } else if (Robot::maze && !robot[i].out_maze &&
               (robot[i].y == 174 || robot[i].y == 175)) {
      // 右边出迷宫

#ifdef DEBUG
      std::cerr << "右边出迷宫" << std::endl;
#endif
      robot[i].FindBerth(robot[i].x, robot[i].y);
      robot[i].out_maze = true;
    }
    // 空闲机器人
    if (!robot[i].target_goods && !robot[i].goods) {
      robot[i].path.clear();
#ifdef DEBUG
      std::cerr << "robot " << i << " start UpdateTargetGoods" << std::endl;
#endif
      if (can_find_goods && robot[i].UpdateTargetGoods(i)) {
        can_find_goods = false;
      }
      if (!robot[i].path.empty()) {
// 有新的目标货物
#ifdef DEBUG
        std::cerr << "robot " << i << " 更新新的目标货物" << std::endl;
#endif
        robot[i].target_goods->robot_id = i;
      }
    }
#ifdef DEBUG
    std::cerr << "*************** robot " << i
              << " path size: " << robot[i].path.size() << " *****************"
              << std::endl;
#endif
    int next_x, next_y;
    // 存落点
    if (!robot[i].path.empty()) {
      std::vector<Location>::iterator iter = robot[i].path.begin();  //迭代器
      next_x = iter->x;
      next_y = iter->y;
      bool same_flag = false;
      for (int j = 0; j < next_points.size(); ++j) {
        if (next_points[j].x == next_x && next_points[j].y == next_y) {
          // 有相同落点
          same_flag = true;
          next_points[j].PushRobot(i, not_move_id);
#ifdef DEBUG
          std::cerr << "same next_point (" << next_x << "," << next_y << ")"
                    << std::endl;
#endif
          break;
        }
      }
      if (!same_flag) {
#ifdef DEBUG
        std::cerr << "add next_point (" << next_x << "," << next_y << ")"
                  << std::endl;
#endif
        next_points.push_back(NextPoint(next_x, next_y, i));
      }
    } else {
      // 集合中记录不动的robot
      not_move_id.push_back(i);
    }
  }
  int size = next_points.size();
  int not_move_size = not_move_id.size();
  //  让挡路的不动的机器人让路
  for (int i = 0; i < not_move_size; ++i) {
    // next_point的下标
    int block_id = -1;
    if ((block_id = IsBlock(not_move_id[i], next_points)) > -1) {
#ifdef DEBUG
      std::cerr << "机器人 " << not_move_id[i] << " 挡住了 "
                << next_points[block_id].list_robot[0] << std::endl;
#endif
      if (GetAway(not_move_id[i], next_points,
                  next_points[block_id].list_robot[0], not_move_id)) {
        // 可以安全让开
      } else {
        next_points[block_id].count = 0;
        not_move_id.push_back(next_points[block_id].list_robot[0]);
        not_move_size = not_move_id.size();
      }
    }
  }

#ifdef DEBUG
  std::cerr << "-- 开始死锁处理 --" << std::endl;
#endif
  // 死锁处理
  // size 不需要更新，因为新添的next_point都是空位
  for (int i = 0; i < size; ++i) {
    for (int j = i + 1; j < size; ++j) {
      if (IsDeadLock(next_points[i], next_points[j])) {
        // 发生死锁
        int getaway_id;  // 优先级低的机器人id
        int ignore_id;   // 优先级高的机器人id
        int giveup_id;   // 放弃的next_point id
        int save_id;     // 保护的next_point id
        if (Robot::JudgePriority(&robot[next_points[i].list_robot[0]],
                                 &robot[next_points[j].list_robot[0]])) {
          getaway_id = next_points[j].list_robot[0];
          ignore_id = next_points[i].list_robot[0];
          giveup_id = j;
          save_id = i;
        } else {
          getaway_id = next_points[i].list_robot[0];
          ignore_id = next_points[j].list_robot[0];
          giveup_id = i;
          save_id = j;
        }
        if (GetAway(getaway_id, next_points, ignore_id, not_move_id)) {
          next_points[giveup_id].count = 0;
        } else {
          // 没法让优先级低的没法让
          // 考虑优先级高的让
          if (GetAway(ignore_id, next_points, getaway_id, not_move_id)) {
            next_points[save_id].count = 0;
          } else {
            // 都没法让 停一步
#ifdef DEBUG
            std::cerr << "死锁都没法让" << std::endl;
#endif
            next_points[save_id].count = 0;
            next_points[giveup_id].count = 0;

            // 处理连锁反应
            // not_move_id.clear();
            not_move_id.push_back(getaway_id);
            not_move_id.push_back(ignore_id);
            int not_move_size = not_move_id.size();
            for (int i = not_move_size - 2; i < not_move_size; ++i) {
              int block_id = -1;
              if ((block_id = IsBlock(not_move_id[i], next_points)) > -1) {
#ifdef DEBUG
                std::cerr << "机器人 " << not_move_id[i] << " 挡住了 "
                          << next_points[block_id].list_robot[0] << std::endl;
#endif
                next_points[block_id].count = 0;
                not_move_id.push_back(next_points[block_id].list_robot[0]);
                not_move_size = not_move_id.size();
              }
            }
          }
        }
      }
    }
  }

#ifdef DEBUG
  std::cerr << "Start MOVE DECISION, next_points size:" << size << std::endl;
#endif
  // 下移动决策
  size = next_points.size();
  for (int i = 0; i < size; ++i) {
    next_points[i].OutPut(not_move_id);
  }

  // 记录拥堵点
  not_move_size = not_move_id.size();
  for (int i = 0; i < not_move_size; ++i) {
    int robot_id = not_move_id[i];
    busy_point[robot[robot_id].x][robot[robot_id].y]++;
    if (busy_point[robot[robot_id].x][robot[robot_id].y] > BUSY_VALVE) {
#ifdef DEBUG
      std::cerr << "机器人原地罚站超时" << std::endl;
#endif
      // 该机器人原地占了BUSY_VALVE次了
      if (robot[robot_id].goods) {
        // 在送货

#ifdef DEBUG
        std::cerr << "机器人更改新的路线，找泊位" << std::endl;
#endif
        robot[robot_id].path.clear();
        robot[robot_id].FindBerth(robot[robot_id].x, robot[robot_id].y);
        robot[robot_id].out_maze = false;
#ifdef ONE_ROBOT_ONE_BERTH
        if (!robot[robot_id].path.empty()) {
          berth[robot[robot_id].berth_id].robot_id = robot_id;
        }
#endif

#ifdef DEBUG
        std::cerr << "机器人更改完新的路线：" << robot[robot_id].path.size()
                  << std::endl;
#endif
      } else if (robot[robot_id].target_goods) {
        // 在拿货

#ifdef DEBUG
        std::cerr << "机器人更改新的路线，找货物" << std::endl;
#endif
        robot[robot_id].target_goods->robot_id = -1;
        robot[robot_id].target_goods = nullptr;
        // 还原first_free_goods指针
        Goods *head_goods = GoodsManager::GetInstance()->head_goods;
        GoodsManager::GetInstance()->first_free_goods = head_goods->next;
        while (GoodsManager::GetInstance()->first_free_goods->next !=
                   head_goods &&
               GoodsManager::GetInstance()->first_free_goods->robot_id > -1) {
          GoodsManager::GetInstance()->first_free_goods =
              GoodsManager::GetInstance()->first_free_goods->next;
        }

        // 找新的目标货物
        robot[robot_id].path.clear();
        robot[robot_id].UpdateTargetGoods(robot_id);
        if (!robot[robot_id].path.empty()) {
          robot[robot_id].target_goods->robot_id = robot_id;
#ifdef DEBUG
          std::cerr << "机器人成功更改新的路线" << std::endl;
#endif
        } else {
#ifdef DEBUG
          std::cerr << "机器人无法更改新的路线" << std::endl;
#endif
        }
      }
    }
  }
}

NextPoint::NextPoint(int x, int y, int robot_id) {
  this->x = x;
  this->y = y;
  list_robot[0] = robot_id;
  count = 1;
}

/*
 * 落点选择机器人决策
 *
 * - 插入算法：插入排序
 */
void NextPoint::PushRobot(int robot_id, std::vector<int> &not_move_id) {
  int i;
  for (i = count - 1; i >= 0; --i) {
    if (Robot::JudgePriority(&robot[list_robot[i]], &robot[robot_id]) == 1) {
      list_robot[i + 1] = robot_id;
      // 当前机器人被决策为不动
      not_move_id.push_back(robot_id);
      break;
    } else {
      list_robot[i + 1] = list_robot[i];
    }
  }
  // 新机器人优先级最高
  if (i < 0) {
    list_robot[0] = robot_id;
    if (count) {
      // 原来优先级最高的机器人被决策为不动
      not_move_id.push_back(list_robot[1]);
    }
  }
  ++count;
}

// 做移动决策
// 剔除not_move_robot_id中能够让位的机器人id
void NextPoint::OutPut(std::vector<int> &not_move_robot_id) {
  // 该落点有机器人决策落入
  if (count) {
    int robot_id = list_robot[0];
#ifdef DEBUG
    std::cerr << "robot " << robot_id << " decided to move from ("
              << robot[robot_id].x << "," << robot[robot_id].y << ") to (" << x
              << "," << y << ")" << std::endl;
#endif
    int param;
    if (this->x == robot[robot_id].x + 1) {
      param = DECISION_ROBOT_DOWN;
    } else if (this->x == robot[robot_id].x - 1) {
      param = DECISION_ROBOT_UP;
    } else if (this->y == robot[robot_id].y + 1) {
      param = DECISION_ROBOT_RIGHT;
    } else if (this->y == robot[robot_id].y - 1) {
      param = DECISION_ROBOT_LEFT;
    } else {
#ifdef DEBUG
      std::cerr << "robot can't move, robot id:" << robot_id << std::endl;
#endif
      return;
    }
    // 剔除not_move_robot_id中能够让位的机器人id
    int size = not_move_robot_id.size();
    for (int i = 0; i < size; ++i) {
      if (not_move_robot_id[i] == robot_id) {
        not_move_robot_id.erase(not_move_robot_id.begin() + i);
      }
    }
    busy_point[robot[robot_id].x][robot[robot_id].y] = 0;  // 该点不拥堵

    DecisionManager::GetInstance()->q_decision.push(
        Decision(DECISION_TYPE_ROBOT_MOVE, robot_id, param));

    // 如果走的是路径中的点，删除robot的path中走了的
    std::vector<Location>::const_iterator iter = robot[robot_id].path.begin();
    if (this->x == iter->x && this->y == iter->y) {
      robot[robot_id].RemoveFirst();
    }

    // 如果移动决策移动后动作
    // --------- 移动后动作 ---------
    if (robot[robot_id].goods && ch[x][y] == 'B') {
#ifdef DEBUG
      std::cerr << "robot " << robot_id << " 移动后卸货：(" << x << "," << y
                << ")" << std::endl;
#endif
      // 卸货
      Decision decision(DECISION_TYPE_ROBOT_PULL, robot_id, -1);
      DecisionManager::GetInstance()->q_decision.push(decision);
      // 增加泊位权重
      ++berth[robot[robot_id].berth_id].weight;
#ifdef ONE_ROBOT_ONE_BERTH
      berth[robot[robot_id].berth_id].robot_id = -1;
#endif
      // robot[robot_id].berth_id = -1;
    } else if (!robot[robot_id].goods && gds[x][y]) {
#ifdef DEBUG
      std::cerr << "地上有货物" << std::endl;
#endif
      bool is_get = false;
      // 机器人没拿货且地上有货物
      if (robot[robot_id].target_goods &&
          gds[x][y] == robot[robot_id].target_goods) {
        // 地上的货物为机器人准备捡的货物
        is_get = true;
#ifdef DEBUG
        std::cerr << "robot " << robot_id << " 移动后装目标货：(" << x << ","
                  << y << ")" << std::endl;
#endif
        // } else if (robot[robot_id].target_goods &&
        //            gds[x][y]->money > VALUEABLE_GOODS_VALVE &&
        //            gds[x][y]->robot_id == -1) {
      }
#ifdef CAN_GRAB_GOODS
      else if (robot[robot_id].target_goods &&
               gds[x][y]->money > VALUEABLE_GOODS_VALVE) {
        // 有目标货物的情况下，路过的地上有贵重货物
        is_get = true;
        // 放弃原目标货物
        if (gds[x][y]->robot_id != -1) {
          // 有机器人选了这个目标货物
          // 给他抢了
          robot[gds[x][y]->robot_id].target_goods = nullptr;
        }
        robot[robot_id].target_goods->robot_id = -1;
        robot[robot_id].target_goods = gds[x][y];
        gds[x][y]->robot_id = robot_id;

        // 还原first_free_goods指针
        Goods *head_goods = GoodsManager::GetInstance()->head_goods;
        GoodsManager::GetInstance()->first_free_goods = head_goods->next;
        while (GoodsManager::GetInstance()->first_free_goods->next !=
                   head_goods &&
               GoodsManager::GetInstance()->first_free_goods->robot_id > -1) {
          GoodsManager::GetInstance()->first_free_goods =
              GoodsManager::GetInstance()->first_free_goods->next;
        }

#ifdef DEBUG
        std::cerr << "robot " << robot_id << " 移动后装路过的高价货：(" << x
                  << "," << y << ")" << std::endl;
#endif
      }  // 区别于移动前动作，这里只有这两种情况
#endif
      if (is_get) {
        // 装货
        Decision decision(DECISION_TYPE_ROBOT_GET, robot_id, -1);
        DecisionManager::GetInstance()->q_decision.push(decision);

        // 决策更新目标泊位和泊位权重
        robot[robot_id].FindBerth(x, y);
        robot[robot_id].out_maze = false;
        // berth_weight[robot[i].berth_id]++;
#ifdef DEBUG
        std::cerr << "成功更新目标泊位" << std::endl;
#endif
        // 捡到货物将其从链表删除
        GoodsManager::GetInstance()->DeleteGoods(robot[robot_id].target_goods);

        //当前持有货物
        robot[robot_id].goods = true;
        robot[robot_id].pre_goods = true;
#ifdef ONE_ROBOT_ONE_BERTH
        if (!robot[robot_id].path.empty()) {
          berth[robot[robot_id].berth_id].robot_id = robot_id;
        }
#endif
      }
    }
  }
}

// 机器人是否拦路
int DecisionManager::IsBlock(int robot_id,
                             std::vector<NextPoint> &next_points) {
  int size = next_points.size();
  int robot_x = robot[robot_id].x;
  int robot_y = robot[robot_id].y;
  for (int i = 0; i < size; ++i) {
    if (next_points[i].count && robot_x == next_points[i].x &&
        robot_y == next_points[i].y) {
      return i;
    }
  }
  return -1;
}

// 检测是否存在面对面死锁
bool DecisionManager::IsDeadLock(NextPoint &first_point,
                                 NextPoint &second_point) {
  int first_x = robot[first_point.list_robot[0]].x;
  int first_y = robot[first_point.list_robot[0]].y;
  int second_x = robot[second_point.list_robot[0]].x;
  int second_y = robot[second_point.list_robot[0]].y;

  if (first_point.count && second_point.count && first_point.x == second_x &&
      first_point.y == second_y && second_point.x == first_x &&
      second_point.y == first_y) {
    return true;
  }
  return false;
}