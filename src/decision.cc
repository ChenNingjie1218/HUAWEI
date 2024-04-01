#include "decision.h"

#include <iostream>
#include <queue>
#include <vector>

#include "astar.h"
#include "berth.h"
#include "boat.h"
#include "map_controller.h"
#include "param.h"
#include "rent_controller.h"
#include "robot.h"
// extern int busy_point[N][N];
extern int id;
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
  auto &robot = RentController::GetInstance()->robot;
  auto &ch = MapController::GetInstance()->ch;
  auto robot_num = robot.size();
  for (int i = 0; i < robot_num; ++i) {
    // --------- 移动前动作 ---------
    if (robot[i].goods && ch[robot[i].x][robot[i].y] == 'B') {
#ifdef DEBUG
      std::cerr << "robot " << i << " 卸货：(" << robot[i].x << ","
                << robot[i].y << ")" << std::endl;
#endif
      // 卸货
      Decision decision(DECISION_TYPE_ROBOT_PULL, i, -1);
      q_decision.push(decision);

#ifdef ONE_ROBOT_ONE_BERTH
      berth[robot->berth_id].robot_id = -1;
#endif
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
    } else if (!robot[i].goods &&
               MapController::GetInstance()->gds[robot[i].x][robot[i].y]) {
#ifdef DEBUG
      std::cerr << "地上有货物" << std::endl;
#endif
      bool is_get = false;
      // 机器人没拿货且地上有货物
      if (robot[i].target_goods &&
          MapController ::GetInstance()->gds[robot[i].x][robot[i].y] ==
              robot[i].target_goods) {
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
        GoodsManager::GetInstance()->ResetFirstFreeGoods();

#ifdef DEBUG
        std::cerr << "robot " << i << " 装路过的高价货：(" << robot[i].x << ","
                  << robot[i].y << ")" << std::endl;
#endif
      }
#endif
      else if (!robot[i].target_goods) {
        // 机器人没目标货物，且该帧刚好生成了一个货物在脚底下
        is_get = true;
        robot[i].target_goods =
            MapController::GetInstance()->gds[robot[i].x][robot[i].y];
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
#ifdef ONE_ROBOT_ONE_BERTH
      if (!robot[i].path.empty()) {
        berth[robot[i].berth_id].robot_id = i;
      }
#endif
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
      for (std::vector<NextPoint>::size_type j = 0; j < next_points.size();
           ++j) {
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
    if ((block_id = robot[not_move_id[i]].IsBlock(next_points)) > -1) {
#ifdef DEBUG
      std::cerr << "机器人 " << not_move_id[i] << " 挡住了 "
                << next_points[block_id].list_robot[0] << std::endl;
#endif
      int dir_index = -1;
      if ((dir_index = robot[not_move_id[i]].GetAway(
               next_points, next_points[block_id].list_robot[0],
               not_move_id)) != -1) {
        // 可以安全让开

#ifdef DEBUG
        std::cerr << "robot " << not_move_id[i] << " 让位给 "
                  << next_points[block_id].list_robot[0] << std::endl;
#endif
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
      if (next_points[i].IsDeadLock(next_points[j])) {
        // 发生死锁
        int getaway_id;  // 优先级低的机器人id
        int ignore_id;   // 优先级高的机器人id
        int giveup_id;   // 放弃的next_point id
        int save_id;     // 保护的next_point id
        if (Robot::JudgePriority(&robot[next_points[i].list_robot[0]],
                                 &robot[next_points[j].list_robot[0]]) == 1) {
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
        int dir_index = -1;
        if ((dir_index = robot[getaway_id].GetAway(next_points, ignore_id,
                                                   not_move_id)) != -1) {
#ifdef DEBUG
          std::cerr << "优先级低的robot " << getaway_id << " 让位给优先级高的 "
                    << ignore_id << std::endl;
#endif
          next_points[giveup_id].count = 0;
        } else {
          // 没法让优先级低的没法让
          // 考虑优先级高的让
          if ((dir_index = robot[ignore_id].GetAway(next_points, getaway_id,
                                                    not_move_id)) != -1) {
#ifdef DEBUG
            std::cerr << "优先级高的robot " << ignore_id << " 让位给优先级低的 "
                      << getaway_id << std::endl;
#endif
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
              if ((block_id = robot[not_move_id[i]].IsBlock(next_points)) >
                  -1) {
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
    MapController::GetInstance()
        ->busy_point[robot[robot_id].x][robot[robot_id].y]++;
    // busy_point[robot[robot_id].x][robot[robot_id].y]++;
    if (MapController::GetInstance()
            ->busy_point[robot[robot_id].x][robot[robot_id].y] >
        DynamicParam::GetInstance()->GetBusyValve()) {
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
        GoodsManager::GetInstance()->ResetFirstFreeGoods();

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

/*
 * 决策购买
 */
void DecisionManager::DecisionPurchase() {
  // 决策买机器人
  auto &robot_purchase_point =
      MapController::GetInstance()->robot_purchase_point;
  auto size = robot_purchase_point.size();
#ifdef DEBUG
  std::cerr << "当前机器人购买点数量：" << size << std::endl;
#endif
  for (std::vector<Location>::size_type i = 0; i < size; ++i) {
    RentController::GetInstance()->RentRobot(i);
  }
  // 决策买船
}