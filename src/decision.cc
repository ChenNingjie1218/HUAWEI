#include "decision.h"

#include <iostream>
#include <queue>
#include <vector>

#include "berth.h"
#include "boat.h"
#include "param.h"
#include "robot.h"
extern Boat boat[10];
extern Robot robot[robot_num + 10];
extern Berth berth[berth_num + 10];
extern char ch[N][N];
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
  int rand_berth = 0;

  for (int i = 0; i < 5; ++i) {
    // status 0 运输中 无需考虑决策
    if (boat[i].status == 1) {
      if (boat[i].pos == -1) {
        // 在虚拟点

        // 决策去哪个泊位
        boat[i].ChooseBerth(rand_berth);
#ifdef DEBUG
        std::cerr << "boat " << i << " choose berth:" << boat[i].pos
                  << std::endl;
#endif
        Decision decision(DECISION_TYPE_BOAT_SHIP, i, boat[i].pos);
        q_decision.push(decision);  // 决策入队
      } else if (boat[i].LeaveCond()) {
        if (!berth[boat[i].pos].q_boat.empty()) {
          berth[boat[i].pos].q_boat.pop();
        }

// 决策是否驶离
#ifdef DEBUG
        std::cerr << "boat " << i << " leave" << boat[i].pos << std::endl;
#endif
        Decision decision(DECISION_TYPE_BOAT_GO, i, -1);
        q_decision.push(decision);
      }
    } else if (boat[i].status == 2) {
// 在等待
// 可以决策是否换船舶，换哪个船舶
#ifdef DEBUG
      std::cerr << "boat " << i << " is waiting" << boat[i].pos << std::endl;
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
  // // 要占领的next_point的id
  // int occupy_point_id = -1;
  // // 要占领的不动机器人的id
  // std::set<int>::iterator getaway_it = not_move_id.end();

  // 1.看周围有没有不被占用的空位
  // 下
  if ((x + 1 != ignore_x) &&
      (ch[x + 1][y] == '.' || ch[x + 1][y] == 'A' || ch[x + 1][y] == 'B')) {
    bool can_leave = true;
    for (int k = 0; k < size; ++k) {
      if (x + 1 == next_points[k].x && y == next_points[k].y) {
        // 下边位置即将被某个机器人占用
        can_leave = false;
        // occupy_point_id = -1;
        break;
      }
    }
    if (can_leave) {
      // 检测是否被占用
      for (std::vector<int>::iterator it = not_move_id.begin();
           it != not_move_id.end(); ++it) {
        if (robot[*it].x == x + 1 && robot[*it].y == y) {
          can_leave = false;
          // getaway_it = it;
          break;
        }
      }
      if (can_leave) {
        // 下边有空位的情况
#ifdef DEBUG
        std::cerr << "向下让位" << std::endl;
#endif
        // 增加新落点
        NextPoint add_next_point = NextPoint(x + 1, y, robot_id);
        next_points.push_back(add_next_point);
        // 将让的位加入让位机器的人path
        robot[robot_id].AddFirst(x, y);
        return true;
      }
    }
  }

  // 上
  if ((x - 1 != ignore_x) &&
      (ch[x - 1][y] == '.' || ch[x - 1][y] == 'A' || ch[x - 1][y] == 'B')) {
    bool can_leave = true;
    for (int k = 0; k < size; ++k) {
      if (x - 1 == next_points[k].x && y == next_points[k].y) {
        // 上边位置即将被某个机器人占用
        can_leave = false;
        // occupy_point_id = k;
        break;
      }
    }
    if (can_leave) {
      // 检测是否被占用
      for (std::vector<int>::iterator it = not_move_id.begin();
           it != not_move_id.end(); ++it) {
        if (robot[*it].x == x - 1 && robot[*it].y == y) {
          can_leave = false;
          // getaway_it = it;
          break;
        }
      }
      if (can_leave) {
        // 上边有空位的情况
#ifdef DEBUG
        std::cerr << "向上让位" << std::endl;
#endif
        // 增加新落点
        NextPoint add_next_point = NextPoint(x - 1, y, robot_id);
        next_points.push_back(add_next_point);
        // 将让的位加入让位机器的人path
        robot[robot_id].AddFirst(x, y);
        return true;
      }
    }
  }

  // 右
  if ((y + 1 != ignore_y) &&
      (ch[x][y + 1] == '.' || ch[x][y + 1] == 'A' || ch[x][y + 1] == 'B')) {
    bool can_leave = true;
    for (int k = 0; k < size; ++k) {
      if (x == next_points[k].x && y + 1 == next_points[k].y) {
        // 右边位置即将被某个机器人占用
        can_leave = false;
        // occupy_point_id = k;
        break;
      }
    }
    if (can_leave) {
      // 检测是否被占用
      std::vector<int>::iterator it;
      for (std::vector<int>::iterator it = not_move_id.begin();
           it != not_move_id.end(); ++it) {
        if (robot[*it].x == x && robot[*it].y == y + 1) {
          can_leave = false;
          // getaway_it = it;
          break;
        }
      }
      if (can_leave) {
        // 右边有空位的情况
#ifdef DEBUG
        std::cerr << "向右让位" << std::endl;
#endif
        // 增加新落点
        NextPoint add_next_point = NextPoint(x, y + 1, robot_id);
        next_points.push_back(add_next_point);
        // 将让的位加入让位机器的人path
        robot[robot_id].AddFirst(x, y);
        return true;
      }
    }
  }

  // 左
  if ((y - 1 != ignore_id) &&
      (ch[x][y - 1] == '.' || ch[x][y - 1] == 'A' || ch[x][y - 1] == 'B')) {
    bool can_leave = true;
    for (int k = 0; k < size; ++k) {
      if (x == next_points[k].x && y - 1 == next_points[k].y) {
        // 左边位置即将被某个机器人占用
        can_leave = false;
        // occupy_point_id = k;
        break;
      }
    }
    if (can_leave) {
      // 检测是否被占用
      std::vector<int>::iterator it;
      for (std::vector<int>::iterator it = not_move_id.begin();
           it != not_move_id.end(); ++it) {
        if (robot[*it].x == x && robot[*it].y == y - 1) {
          can_leave = false;
          // getaway_it = it;
          break;
        }
      }
      if (can_leave) {
#ifdef DEBUG
        std::cerr << "向左让位" << std::endl;
#endif
        // 增加新落点
        NextPoint add_next_point = NextPoint(x, y - 1, robot_id);
        next_points.push_back(add_next_point);
        // 将让的位加入让位机器的人path
        robot[robot_id].AddFirst(x, y);
        return true;
      }
    }
  }

  //   // 2.让周围不动的让位
  //   if (getaway_it != not_move_id.end()) {
  // #ifdef DEBUG
  //     std::cerr << "要占领让位" << std::endl;
  // #endif
  //     // 占领
  //     int getaway_id = *getaway_it;

  //     // 增加新落点
  //     if (x == robot[getaway_id].x && y - 1 == robot[getaway_id].y) {
  //       // 占领左
  //       NextPoint add_next_point = NextPoint(x, y - 1, robot_id);
  //       next_points.push_back(add_next_point);
  //     } else if (x == robot[getaway_id].x && y + 1 == robot[getaway_id].y) {
  //       // 占领右
  //       NextPoint add_next_point = NextPoint(x, y + 1, robot_id);
  //       next_points.push_back(add_next_point);
  //     } else if (x - 1 == robot[getaway_id].x && y == robot[getaway_id].y) {
  //       // 占领上
  //       NextPoint add_next_point = NextPoint(x - 1, y, robot_id);
  //       next_points.push_back(add_next_point);
  //     } else {
  //       // 占领下
  //       NextPoint add_next_point = NextPoint(x + 1, y, robot_id);
  //       next_points.push_back(add_next_point);
  //     }
  //     // 将让的位加入让位机器的人path
  //     robot[robot_id].AddFirst(x, y);

  //     not_move_id.erase(getaway_it);
  //     GetAway(getaway_id, next_points, robot_id, not_move_id);
  //     return;
  //   }

  //   // 3.抢占位置
  //   if (occupy_point_id > -1) {
  // #ifdef DEBUG
  //     std::cerr << "抢占位置" << std::endl;
  // #endif

  //     // 把当前机器人id插到最前面
  //     int count = next_points[occupy_point_id].count;
  //     for (int i = count; i > 0; --i) {
  //       next_points[occupy_point_id].list_robot[i] =
  //           next_points[occupy_point_id].list_robot[i - 1];
  //     }
  //     next_points[occupy_point_id].list_robot[0] = robot_id;
  //     not_move_id.insert(next_points[occupy_point_id].list_robot[1]);
  //     // 将让的位加入让位机器的人path
  //     robot[robot_id].AddFirst(x, y);

  //     // 产生了新的不动机器人需要判断
  //     return;
  // }

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
      robot[i].berth_id = -1;
      // 下货的位置可能不是计算的位置，path里面还有内容
      robot[i].path.clear();
      // 决策，更新目标货物, 当前不持有货物
#ifdef DEBUG
      std::cerr << "robot " << i << " start UpdateTargetGoods" << std::endl;
#endif
      robot[i].UpdateTargetGoods();
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
    } else if (!robot[i].goods && robot[i].target_goods &&
               robot[i].target_goods->x == robot[i].x &&
               robot[i].target_goods->y == robot[i].y) {
#ifdef DEBUG
      std::cerr << "robot " << i << " 装货：(" << robot[i].x << ","
                << robot[i].y << ")" << std::endl;
#endif
      // 装货
      Decision decision(DECISION_TYPE_ROBOT_GET, i, -1);
      q_decision.push(decision);

      // 捡到货物将其从链表删除
      GoodsManager::GetInstance()->DeleteGoods(robot[i].target_goods);

      // 决策更新目标泊位和泊位权重
      robot[i].FindBerth();
      // berth_weight[robot[i].berth_id]++;

      //当前持有货物
      robot[i].goods = true;
    }
    // if (!robot[i].goods) {
    // 空闲机器人
    if (!robot[i].target_goods && !robot[i].goods) {
      robot[i].path.clear();
#ifdef DEBUG
      std::cerr << "robot " << i << " start UpdateTargetGoods" << std::endl;
#endif
      robot[i].UpdateTargetGoods();
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
            not_move_id.clear();
            not_move_id.push_back(getaway_id);
            not_move_id.push_back(ignore_id);
            int not_move_size = not_move_id.size();
            for (int i = 0; i < not_move_size; ++i) {
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

  // 下移动决策
  size = next_points.size();
#ifdef DEBUG
  std::cerr << "Start MOVE DECISION, next_points size:" << size << std::endl;
#endif
  for (int i = 0; i < size; ++i) {
    next_points[i].OutPut();
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
void NextPoint::OutPut() {
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
      ++berth[robot->berth_id].weight;
      robot[robot_id].berth_id = -1;
    } else if (!robot[robot_id].goods && robot[robot_id].target_goods &&
               robot[robot_id].target_goods->x == robot[robot_id].x &&
               robot[robot_id].target_goods->y == robot[robot_id].y) {
#ifdef DEBUG
      std::cerr << "robot " << robot_id << " 移动后装货：(" << x << "," << y
                << ")" << std::endl;
#endif
      // 装货
      Decision decision(DECISION_TYPE_ROBOT_GET, robot_id, -1);
      DecisionManager::GetInstance()->q_decision.push(decision);

      // 捡到货物将其从链表删除
      GoodsManager::GetInstance()->DeleteGoods(robot[robot_id].target_goods);

      // 决策更新目标泊位和泊位权重
      robot[robot_id].FindBerth();
      // berth_weight[robot[i].berth_id]++;

      //当前持有货物
      robot[robot_id].goods = true;
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