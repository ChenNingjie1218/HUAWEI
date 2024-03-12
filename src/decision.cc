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

void DecisionManager::SolveFaceToFaceDeadLock(
    std::vector<NextPoint> &next_points) {
  int size = next_points.size();
  // 判断对碰死锁，更新next_points
  for (int i = 0; i < size; ++i) {
    for (int j = i + 1; j < size; ++j) {
      if (next_points[i].count && next_points[j].count &&
          next_points[i].x == robot[next_points[j].list_robot[0]].x &&
          next_points[i].y == robot[next_points[j].list_robot[0]].y &&
          robot[next_points[i].list_robot[0]].x == next_points[j].x &&
          robot[next_points[i].list_robot[0]].y && next_points[j].y) {
        // 给优先级高的机器人让位
        /*
         * 判断让位机器人能不能让位
         * @param leave 让位标志
         * - 0 无法让位
         * - 1 向右让位
         * - 2 向左让位
         * - 3 向上让位
         * - 4 向下让位
         */
        int leave = 0;
        int give_up_point;
        int save_point;
        if (Robot::JudgePriority(&robot[next_points[i].list_robot[0]],
                                 &robot[next_points[j].list_robot[0]]) == 1) {
          /*
           * 这里有点绕作如下解释：
           * next_points[i].list_robot[0]是站在next_points[j]上的机器人
           * next_points[j].list_robot[0]是站在next_points[i]上的机器人
           * 所以next_points[i].list_robot[0]优先级高，就要放弃位置为next_points[j]
           */
          save_point = i;
          give_up_point = j;
        } else {
          save_point = j;
          give_up_point = i;
        }

        //企图向右让位
        if (ch[next_points[save_point].x + 1][next_points[save_point].y] ==
            '*') {
          bool can_leave = true;
          for (int k = 0; k < size; ++k) {
            if (next_points[save_point].x + 1 == next_points[k].x) {
              // 右边位置即将被某个机器人占用
              can_leave = false;
              break;
            }
          }
          if (can_leave) {
            leave = 1;
            // 增加新落点
            NextPoint add_next_point = NextPoint(
                next_points[save_point].x + 1, next_points[save_point].y,
                next_points[give_up_point].list_robot[0]);
            // 将让的位加入让位机器的人path
            robot[next_points[give_up_point].list_robot[0]].AddFirst(
                next_points[save_point].x, next_points[save_point].y);
            // 废弃旧落点
            next_points[give_up_point].count = 0;
            next_points.push_back(add_next_point);
            // 更新size
            size = next_points.size();
            break;
          }
        }

        // 企图向左让位
        if (ch[next_points[save_point].x - 1][next_points[save_point].y] ==
            '*') {
          bool can_leave = true;
          for (int k = 0; k < size; ++k) {
            if (next_points[save_point].x - 1 == next_points[k].x) {
              // 右边位置即将被某个机器人占用
              can_leave = false;
              break;
            }
          }
          if (can_leave) {
            leave = 2;
            // 增加新落点
            NextPoint add_next_point = NextPoint(
                next_points[save_point].x - 1, next_points[save_point].y,
                next_points[give_up_point].list_robot[0]);
            // 将让的位加入让位机器的人path
            robot[next_points[give_up_point].list_robot[0]].AddFirst(
                next_points[save_point].x, next_points[save_point].y);
            // 废弃旧落点
            next_points[give_up_point].count = 0;
            next_points.push_back(add_next_point);
            // 更新size
            size = next_points.size();
            break;
          }
        }

        // 企图向上让位
        if (ch[next_points[save_point].x][next_points[save_point].y + 1] ==
            '*') {
          bool can_leave = true;
          for (int k = 0; k < size; ++k) {
            if (next_points[save_point].y + 1 == next_points[k].y) {
              // 上边位置即将被某个机器人占用
              can_leave = false;
              break;
            }
          }
          if (can_leave) {
            leave = 3;
            // 增加新落点
            NextPoint add_next_point = NextPoint(
                next_points[save_point].x, next_points[save_point].y + 1,
                next_points[give_up_point].list_robot[0]);
            // 将让的位加入让位机器的人path
            robot[next_points[give_up_point].list_robot[0]].AddFirst(
                next_points[save_point].x, next_points[save_point].y);
            // 废弃旧落点
            next_points[give_up_point].count = 0;
            next_points.push_back(add_next_point);
            // 更新size
            size = next_points.size();
            break;
          }
        }

        // 企图向下让位
        if (ch[next_points[save_point].x][next_points[save_point].y - 1] ==
            '*') {
          bool can_leave = true;
          for (int k = 0; k < size; ++k) {
            if (next_points[save_point].y - 1 == next_points[k].y) {
              // 下边位置即将被某个机器人占用
              can_leave = false;
              break;
            }
          }
          if (can_leave) {
            leave = 4;
            // 增加新落点
            NextPoint add_next_point = NextPoint(
                next_points[save_point].x, next_points[save_point].y - 1,
                next_points[give_up_point].list_robot[0]);
            // 将让的位加入让位机器的人path
            robot[next_points[give_up_point].list_robot[0]].AddFirst(
                next_points[save_point].x, next_points[save_point].y);
            // 废弃旧落点
            next_points[give_up_point].count = 0;
            next_points.push_back(add_next_point);
            // 更新size
            size = next_points.size();
            break;
          }
        }

        // 无法让位 两个机器人放弃移动
        next_points[i].count = 0;
        next_points[j].count = 0;
        break;
      }
    }
  }
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
  for (int i = 0; i < 10; ++i) {
    // --------- 移动前动作 ---------
    if (robot[i].goods && ch[robot[i].x][robot[i].y] == 'B' &&
        !berth[robot[i].berth_id].q_boat.empty()) {
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
      // 决策，更新目标货物, 当前不持有货物
#ifdef DEBUG
      std::cerr << "robot " << i << " start UpdateTargetGoods" << std::endl;
#endif
      robot[i].UpdateTargetGoods();
      if (!robot[i].path.empty()) {
        // 有新的目标货物
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
    if (!robot[i].target_goods && !robot[i].goods) {
#ifdef DEBUG
      std::cerr << "robot " << i << " start UpdateTargetGoods" << std::endl;
#endif
      robot[i].UpdateTargetGoods();
      if (!robot[i].path.empty()) {
        // 有新的目标货物
        robot[i].target_goods->robot_id = i;
      }
    }
#ifdef DEBUG
    std::cerr << "*************** robot " << i
              << " path size: " << robot[i].path.size() << " *****************"
              << std::endl;
#endif
    // 存落点
    if (!robot[i].path.empty()) {
      std::vector<Location>::iterator iter = robot[i].path.begin();  //迭代器
      bool same_flag = false;
      for (int j = 0; j < next_points.size(); ++j) {
        if (next_points[j].x == iter->x && next_points[j].y == iter->y) {
          // 有相同落点
          same_flag = true;
          next_points[j].PushRobot(i);
          break;
        }
      }
      if (!same_flag) {
#ifdef DEBUG
        std::cerr << "add next_point (" << iter->x << "," << iter->y << ")"
                  << std::endl;
#endif
        next_points.push_back(NextPoint(iter->x, iter->y, i));
      }
    }
  }
#ifdef DEBUG
  std::cerr << "Start SolveFaceToFaceDeadLock" << std::endl;
#endif
  // --------- 移动 ---------
  // 面对面死锁
  DecisionManager::SolveFaceToFaceDeadLock(next_points);
#ifdef DEBUG
  std::cerr << "Finish SolveFaceToFaceDeadLock" << std::endl;
#endif
  // 下移动决策
  int size = next_points.size();
#ifdef DEBUG
  std::cerr << "Start MOVE DECISION, next_points size:" << size << std::endl;
#endif
  for (int i = 0; i < size; ++i) {
    next_points[i].OutPut();
  }

  // 如果移动决策移动后动作
  // --------- 移动后动作 ---------
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
void NextPoint::PushRobot(int robot_id) {
  int i;
  for (i = count - 1; i >= 0; --i) {
    if (Robot::JudgePriority(&robot[list_robot[i]], &robot[robot_id]) == 1) {
      list_robot[i + 1] = robot_id;
      break;
    } else {
      list_robot[i + 1] = list_robot[i];
    }
  }
  // 新机器人优先级最高
  if (i < 0) {
    list_robot[0] = robot_id;
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
    } else {
      param = DECISION_ROBOT_LEFT;
    }
    DecisionManager::GetInstance()->q_decision.push(
        Decision(DECISION_TYPE_ROBOT_MOVE, robot_id, param));

    // 如果走的是路径中的点，删除robot的path中走了的
    std::vector<Location>::const_iterator iter = robot[robot_id].path.begin();
    if (this->x == iter->x && this->y == iter->y) {
      robot[robot_id].RemoveFirst();
    }
  }
}
