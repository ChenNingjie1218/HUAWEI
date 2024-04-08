#include "nextpoint.h"

#include <iostream>

#include "decision.h"
#include "map_controller.h"
#include "param.h"
#include "rent_controller.h"
#include "robot.h"
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
  std::vector<Robot> &robot = RentController::GetInstance()->robot;
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
    std::vector<Robot> &robot = RentController::GetInstance()->robot;
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
    MapController::GetInstance()
        ->busy_point[robot[robot_id].x][robot[robot_id].y] = 0;  // 该点不拥堵

    DecisionManager::GetInstance()->q_decision.push(
        Decision(DECISION_TYPE_ROBOT_MOVE, robot_id, param));

    // 如果走的是路径中的点，删除robot的path中走了的
    std::vector<Location>::const_iterator iter = robot[robot_id].path.begin();
    if (this->x == iter->x && this->y == iter->y) {
      robot[robot_id].RemoveFirst();
    }

    // 如果移动决策移动后动作
    // --------- 移动后动作 ---------
    if (robot[robot_id].goods &&
        MapController::GetInstance()->ch[x][y] == 'B') {
#ifdef DEBUG
      std::cerr << "robot " << robot_id << " 移动后卸货：(" << x << "," << y
                << ")" << std::endl;
#endif
      // 卸货
      Decision decision(DECISION_TYPE_ROBOT_PULL, robot_id, -1);
      DecisionManager::GetInstance()->q_decision.push(decision);
    } else if (!robot[robot_id].goods &&
               MapController::GetInstance()->gds[x][y]) {
#ifdef DEBUG
      std::cerr << "地上有货物" << std::endl;
#endif
      bool is_get = false;
      // 机器人没拿货且地上有货物
      if (robot[robot_id].target_goods &&
          MapController::GetInstance()->gds[x][y] ==
              robot[robot_id].target_goods) {
        // 地上的货物为机器人准备捡的货物
        is_get = true;
#ifdef DEBUG
        std::cerr << "robot " << robot_id << " 移动后装目标货：(" << x << ","
                  << y << ")" << std::endl;
#endif
      }
      if (is_get) {
        // 装货
        Decision decision(DECISION_TYPE_ROBOT_GET, robot_id, -1);
        DecisionManager::GetInstance()->q_decision.push(decision);

        // 决策更新目标泊位和泊位权重
        robot[robot_id].FindBerth(x, y);
#ifdef DEBUG
        std::cerr << "成功更新目标泊位" << std::endl;
        robot[robot_id].money += robot[robot_id].target_goods->money;
        MapController::GetInstance()->get_money +=
            robot[robot_id].target_goods->money;
#endif
        //当前持有货物
        robot[robot_id].goods = true;
      }
    }
  }
}

// 检测是否存在面对面死锁
bool NextPoint::IsDeadLock(NextPoint &other_point) {
  std::vector<Robot> &robot = RentController::GetInstance()->robot;
  int first_x = robot[list_robot[0]].x;
  int first_y = robot[list_robot[0]].y;
  int second_x = robot[other_point.list_robot[0]].x;
  int second_y = robot[other_point.list_robot[0]].y;
  if (MapController::GetInstance()->IsMainChannel(first_x, first_y) &&
      MapController::GetInstance()->IsMainChannel(second_x, second_y)) {
    // 都位于主干道的
    return false;
  }
  if (count && other_point.count && x == second_x && y == second_y &&
      other_point.x == first_x && other_point.y == first_y) {
    return true;
  }
  return false;
}