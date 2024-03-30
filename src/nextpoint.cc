#include "nextpoint.h"

#include <iostream>

#include "decision.h"
#include "param.h"
#include "robot.h"
extern Robot robot[robot_num + 10];
extern int busy_point[N][N];
extern char ch[N][N];
extern Goods *gds[N][N];
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

#ifdef ONE_ROBOT_ONE_BERTH
      berth[robot[robot_id].berth_id].robot_id = -1;
#endif
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

// 检测是否存在面对面死锁
bool NextPoint::IsDeadLock(NextPoint &other_point) {
  int first_x = robot[list_robot[0]].x;
  int first_y = robot[list_robot[0]].y;
  int second_x = robot[other_point.list_robot[0]].x;
  int second_y = robot[other_point.list_robot[0]].y;

  if (count && other_point.count && x == second_x && y == second_y &&
      other_point.x == first_x && other_point.y == first_y) {
    return true;
  }
  return false;
}