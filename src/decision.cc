#include "decision.h"

#include <algorithm>
#include <chrono>
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
bool can_astar = true;  // 每次决策只用一次astar
DecisionManager *DecisionManager::instance_ = nullptr;

Decision::Decision(int type, int param_1, int param_2) {
  this->type = type;
  this->param_1 = param_1;
  this->param_2 = param_2;
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
  auto &boat = RentController::GetInstance()->boat;
  int size = boat.size();
  std::vector<int> move_id;
  for (int i = 0; i < size; ++i) {
    if (boat[i].status == BOAT_STATUS_RESTORING) {
// 恢复状态不决策
#ifdef DEBUG
      std::cerr << boat[i].id_ << " 船处于恢复状态" << std::endl;
#endif
      continue;
    }
    auto &berth = MapController::GetInstance()->berth;
    // 靠泊
    if (boat[i].status != BOAT_STATUS_LOADING &&
        MapController::GetInstance()->ch[boat[i].x][boat[i].y] == 'K' &&
        boat[i].num < Boat::boat_capacity) {
      if (boat[i].pos > -1) {
        int berth_id =
            MapController::GetInstance()
                ->location_to_berth_id[Location(boat[i].x, boat[i].y)];
        if (berth[berth_id].goods_num) {
          if (boat[i].pos != berth_id) {
#ifdef DEBUG
            std::cerr << i << " 船路过泊位 " << berth_id
                      << " 货物数量:" << berth[berth_id].goods_num << std::endl;
#endif
            berth[boat[i].pos].boat_id = -1;
            boat[i].old_pos = berth_id;
            boat[i].pos = berth_id;
          }
          //&&
          // boat[i].pos ==
          //     MapController::GetInstance()
          //         ->location_to_berth_id[Location(boat[i].x, boat[i].y)]
          boat[i].DoBerth();
          continue;
        }
      }
    }

    if (boat[i].path.empty()) {
      if (boat[i].DeliveryCond()) {
        // 去交货
        boat[i].FindDeliveryPoint();
      } else {
        if (boat[i].status == BOAT_STATUS_LOADING &&
            berth[boat[i].pos].goods_num && boat[i].num < Boat::boat_capacity) {
#ifdef DEBUG
          std::cerr << boat[i].id_ << " 船正在装货" << std::endl;
#endif
          continue;
        }
        // 找船舶上货
        boat[i].FindBerth();
      }
    }

    if (!boat[i].path.empty()) {
      move_id.push_back(i);
    }
#ifdef DEBUG
    std::cerr << "*************** " << boat[i].id_
              << " 船 path size:" << boat[i].path.size() << "*****************"
              << std::endl;
#endif
  }
  // 移动决策
  size = move_id.size();
  for (int i = 0; i < size; ++i) {
    int first_id = move_id[i];
    CollisionBox first_now(boat[first_id].x, boat[first_id].y,
                           boat[first_id].direction);  // 第一艘船当前位置
    if (boat[first_id].stuck_times >
        DynamicParam::GetInstance()->GetBusyValve()) {
#ifdef DEBUG
      std::cerr << first_id << " 船罚站超时" << std::endl;
#endif
      boat[first_id].DoDept();
      int boat_size = boat.size();
      // 重置所有船堵塞次数，即只dept一艘船
      for (int j = 0; j < boat_size; ++j) {
        boat[j].stuck_times = 0;
      }
      continue;
    }
    CollisionBox first_next(boat[first_id].x, boat[first_id].y,
                            boat[first_id].direction,
                            boat[first_id].path[0]);  // 第一艘船下一步位置
    // 如果当前有一搜船完全在泊位上，不做碰撞检测
    if (first_next.IsCompletelyLocatedOnMainRoute()) {
      boat[first_id].DoMove();
      continue;
    }
    for (int j = i + 1; j < size; ++j) {
      int second_id = move_id[j];
      CollisionBox second_now(boat[second_id].x, boat[second_id].y,
                              boat[second_id].direction);  // 第二艘船当前位置
      CollisionBox second_next(boat[second_id].x, boat[second_id].y,
                               boat[second_id].direction,
                               boat[second_id].path[0]);  // 第二艘船下一步位置
      if (second_next.IsCompletelyLocatedOnMainRoute()) {
        // 不去影响位于主航道的船
        continue;
      }
      if (CollisionBox::JudgeCollision(first_next, second_now) &&
          CollisionBox::JudgeCollision(first_now, second_next) &&
          boat[first_id].path[0] / 2 == boat[second_id].path[0] / 2) {
        // 会碰撞且方向互斥

#ifdef DEBUG
        std::cerr << first_id << " 与 " << second_id << " 船发生碰撞"
                  << std::endl;
#endif
        int man = std::abs(boat[first_id].y - boat[second_id].y) +
                  std::abs(boat[first_id].x -
                           boat[second_id].x);  // 两船核心点的曼哈顿距离
        std::vector<int> new_path;              // 解决路径
        std::queue<int> q;                      // 解决方式队列
        if (boat[first_id].direction == boat[first_id].path[0] &&
            boat[second_id].direction == boat[second_id].path[0]) {
          // 都是前行

#ifdef DEBUG
          std::cerr << "都是前行" << std::endl;
#endif
          if (man == 5) {
            // 错开的情况 1 其中一艘船顺、逆可解
#ifdef DEBUG
            std::cerr << "错开的情况 1" << std::endl;
#endif
            q.push(DECISION_BOAT_ROT_CLOCKWISE);
            q.push(DECISION_BOAT_ROT_COUNTERCLOCKWISE);
            if (boat[first_id].SolveCollision(q, new_path)) {
              boat[first_id].path = new_path;
#ifdef DEBUG
              std::cerr << first_id << " 船让行" << std::endl;
#endif
              continue;
            } else if (boat[second_id].SolveCollision(q, new_path)) {
              boat[second_id].path = new_path;
#ifdef DEBUG
              std::cerr << second_id << " 船让行" << std::endl;
#endif
              continue;
            }
          } else if (man == 7) {
            // 错开的情况 2 其中一艘船逆、顺可解

#ifdef DEBUG
            std::cerr << "错开的情况 1" << std::endl;
#endif
            q.push(DECISION_BOAT_ROT_COUNTERCLOCKWISE);
            q.push(DECISION_BOAT_ROT_CLOCKWISE);
            if (boat[first_id].SolveCollision(q, new_path)) {
              boat[first_id].path = new_path;
#ifdef DEBUG
              std::cerr << first_id << " 船让行" << std::endl;
#endif
              continue;
            } else if (boat[second_id].SolveCollision(q, new_path)) {
              boat[second_id].path = new_path;
#ifdef DEBUG
              std::cerr << second_id << " 船让行" << std::endl;
#endif
              continue;
            }
          } else if (man == 6) {
            // 没有错开的情况

#ifdef DEBUG
            std::cerr << "没有错开的情况" << std::endl;
#endif
            // 如果两艘船都能动 执行顺、逆可解
            q.push(DECISION_BOAT_ROT_CLOCKWISE);
            q.push(DECISION_BOAT_ROT_COUNTERCLOCKWISE);
            std::vector<int> new_path_2;  // 同时解，另一艘船的解决路径
            if (boat[first_id].SolveCollision(q, new_path) &&
                boat[second_id].SolveCollision(q, new_path_2)) {
              boat[first_id].path = new_path;
              boat[second_id].path = new_path_2;
#ifdef DEBUG
              std::cerr << first_id << ", " << second_id << " 船同时让行"
                        << std::endl;
#endif
              continue;
            }

            // 两艘船同时顺逆无法解开 一艘能动的船顺、前行、逆可解开
            while (!q.empty()) {
              q.pop();
            }
            q.push(DECISION_BOAT_ROT_CLOCKWISE);
            q.push(DECISION_BOAT_SHIP);
            q.push(DECISION_BOAT_ROT_COUNTERCLOCKWISE);
            if (boat[first_id].SolveCollision(q, new_path)) {
              boat[first_id].path = new_path;
#ifdef DEBUG
              std::cerr << first_id << " 船让行" << std::endl;
#endif
              continue;
            } else if (boat[second_id].SolveCollision(q, new_path)) {
              boat[second_id].path = new_path;
#ifdef DEBUG
              std::cerr << second_id << " 船让行" << std::endl;
#endif
              continue;
            }
          } else {
#ifdef DEBUG
            std::cerr << "船直行，核心点曼哈顿距离错误" << std::endl;
#endif
          }
        } else if (boat[first_id].direction != boat[first_id].path[0] &&
                   boat[second_id].direction != boat[second_id].path[0]) {
          // 都是旋转

#ifdef DEBUG
          std::cerr << "都是旋转" << std::endl;
#endif
          bool is_clockwise =
              MoveType(boat[first_id].direction, boat[first_id].path[0]) ==
              DECISION_BOAT_ROT_CLOCKWISE;
          if (is_clockwise) {
#ifdef DEBUG
            std::cerr << first_id << " 船顺时针旋转" << std::endl;
#endif
            // 第一艘船是顺时针旋转，解决方案是逆时针旋转
            q.push(DECISION_BOAT_ROT_COUNTERCLOCKWISE);
            if (boat[first_id].SolveCollision(q, new_path)) {
              boat[first_id].path = new_path;
#ifdef DEBUG
              std::cerr << first_id << " 船让行" << std::endl;
#endif
              continue;
            }
            // 第一艘船逆时针解不开，用第二艘船解
            q.pop();
            // 判断第二艘船是不是顺时针旋转
            if (MoveType(boat[second_id].direction, boat[second_id].path[0]) ==
                DECISION_BOAT_ROT_CLOCKWISE) {
              // 第二艘船是顺时针旋转，解决方案是逆时针旋转
#ifdef DEBUG
              std::cerr << second_id << " 船顺时针旋转" << std::endl;
#endif
              q.push(DECISION_BOAT_ROT_COUNTERCLOCKWISE);
              if (boat[second_id].SolveCollision(q, new_path)) {
                boat[second_id].path = new_path;
#ifdef DEBUG
                std::cerr << second_id << " 船让行" << std::endl;
#endif
                continue;
              }
            } else {
#ifdef DEBUG
              std::cerr << second_id << " 船逆时针旋转" << std::endl;
#endif
              // 第二艘船是逆时针旋转，解决方案是顺时针旋转

              q.push(DECISION_BOAT_ROT_CLOCKWISE);
              if (boat[second_id].SolveCollision(q, new_path)) {
                boat[second_id].path = new_path;
#ifdef DEBUG
                std::cerr << second_id << " 船让行" << std::endl;
#endif
                continue;
              }
            }
            // 两艘船都旋转无法解开，那就看直行能不能解开，通过曼哈顿距离判断
            q.pop();
            int man = std::abs(boat[first_id].y - boat[second_id].y) +
                      std::abs(boat[first_id].x - boat[second_id].x);
            if (man == 5) {
              // 直行四步
              q.push(DECISION_BOAT_SHIP);
              q.push(DECISION_BOAT_SHIP);
              q.push(DECISION_BOAT_SHIP);
              q.push(DECISION_BOAT_SHIP);
              if (boat[first_id].SolveCollision(q, new_path)) {
                boat[first_id].path = new_path;
#ifdef DEBUG
                std::cerr << first_id << " 船让行" << std::endl;
#endif
                continue;
              }
              if (boat[second_id].SolveCollision(q, new_path)) {
                boat[second_id].path = new_path;
#ifdef DEBUG
                std::cerr << second_id << " 船让行" << std::endl;
#endif
                continue;
              }
            } else if (man == 4) {
              // 直行三步
              q.push(DECISION_BOAT_SHIP);
              q.push(DECISION_BOAT_SHIP);
              q.push(DECISION_BOAT_SHIP);
              if (boat[first_id].SolveCollision(q, new_path)) {
                boat[first_id].path = new_path;
#ifdef DEBUG
                std::cerr << first_id << " 船让行" << std::endl;
#endif
                continue;
              }
              if (boat[second_id].SolveCollision(q, new_path)) {
                boat[second_id].path = new_path;
#ifdef DEBUG
                std::cerr << second_id << " 船让行" << std::endl;
#endif
                continue;
              }
            } else if (man == 3) {
              // 直行两步
              q.push(DECISION_BOAT_SHIP);
              q.push(DECISION_BOAT_SHIP);
              if (boat[first_id].SolveCollision(q, new_path)) {
                boat[first_id].path = new_path;
#ifdef DEBUG
                std::cerr << first_id << " 船让行" << std::endl;
#endif
                continue;
              }
              if (boat[second_id].SolveCollision(q, new_path)) {
                boat[second_id].path = new_path;
#ifdef DEBUG
                std::cerr << second_id << " 船让行" << std::endl;
#endif
                continue;
              }
            } else if (man == 2) {
              // 直行一步
              q.push(DECISION_BOAT_SHIP);
              if (boat[first_id].SolveCollision(q, new_path)) {
                boat[first_id].path = new_path;
#ifdef DEBUG
                std::cerr << first_id << " 船让行" << std::endl;
#endif
                continue;
              }
              if (boat[second_id].SolveCollision(q, new_path)) {
                boat[second_id].path = new_path;
#ifdef DEBUG
                std::cerr << second_id << " 船让行" << std::endl;
#endif
                continue;
              }
            }
            // 其他解法
          } else {
#ifdef DEBUG
            std::cerr << first_id << " 船逆时针旋转" << std::endl;
#endif
            // 第一艘船是逆时针旋转，解决方案是顺时针旋转
            q.push(DECISION_BOAT_ROT_CLOCKWISE);
            if (boat[first_id].SolveCollision(q, new_path)) {
              boat[first_id].path = new_path;
#ifdef DEBUG
              std::cerr << first_id << " 船让行" << std::endl;
#endif
              continue;
            }
            // 第一艘船顺时针解不开，用第二艘船解
            q.pop();
            // 判断第二艘船是不是顺时针旋转
            if (MoveType(boat[second_id].direction, boat[second_id].path[0]) ==
                DECISION_BOAT_ROT_CLOCKWISE) {
#ifdef DEBUG
              std::cerr << second_id << " 船顺时针旋转" << std::endl;
#endif
              // 第二艘船是顺时针旋转，解决方案是逆时针旋转
              q.push(DECISION_BOAT_ROT_COUNTERCLOCKWISE);
              if (boat[second_id].SolveCollision(q, new_path)) {
                boat[second_id].path = new_path;
#ifdef DEBUG
                std::cerr << second_id << " 船让行" << std::endl;
#endif
                continue;
              }
            } else {
#ifdef DEBUG
              std::cerr << second_id << " 船逆时针旋转" << std::endl;
#endif
              // 第二艘船是逆时针旋转，解决方案是顺时针旋转
              q.push(DECISION_BOAT_ROT_CLOCKWISE);
              if (boat[second_id].SolveCollision(q, new_path)) {
#ifdef DEBUG
                std::cerr << second_id << " 船让行" << std::endl;
#endif
                boat[second_id].path = new_path;
                continue;
              }
            }
            // 两艘船都旋转无法解开，那就看直行能不能解开，直行四步
            q.pop();
            int man = std::abs(boat[first_id].y - boat[second_id].y) +
                      std::abs(boat[first_id].x - boat[second_id].x);
            if (man == 5) {
              // 直行四步
              q.push(DECISION_BOAT_SHIP);
              q.push(DECISION_BOAT_SHIP);
              q.push(DECISION_BOAT_SHIP);
              q.push(DECISION_BOAT_SHIP);
              if (boat[first_id].SolveCollision(q, new_path)) {
                boat[first_id].path = new_path;
#ifdef DEBUG
                std::cerr << first_id << " 船让行" << std::endl;
#endif
                continue;
              }
              if (boat[second_id].SolveCollision(q, new_path)) {
                boat[second_id].path = new_path;
#ifdef DEBUG
                std::cerr << second_id << " 船让行" << std::endl;
#endif
                continue;
              }
            } else if (man == 4) {
              // 直行三步
              q.push(DECISION_BOAT_SHIP);
              q.push(DECISION_BOAT_SHIP);
              q.push(DECISION_BOAT_SHIP);
              if (boat[first_id].SolveCollision(q, new_path)) {
                boat[first_id].path = new_path;
#ifdef DEBUG
                std::cerr << first_id << " 船让行" << std::endl;
#endif
                continue;
              }
              if (boat[second_id].SolveCollision(q, new_path)) {
                boat[second_id].path = new_path;
#ifdef DEBUG
                std::cerr << second_id << " 船让行" << std::endl;
#endif
                continue;
              }
            } else if (man == 3) {
              // 直行两步
              q.push(DECISION_BOAT_SHIP);
              q.push(DECISION_BOAT_SHIP);
              if (boat[first_id].SolveCollision(q, new_path)) {
                boat[first_id].path = new_path;
#ifdef DEBUG
                std::cerr << first_id << " 船让行" << std::endl;
#endif
                continue;
              }
              if (boat[second_id].SolveCollision(q, new_path)) {
                boat[second_id].path = new_path;
#ifdef DEBUG
                std::cerr << second_id << " 船让行" << std::endl;
#endif
                continue;
              }
            } else if (man == 2) {
              // 直行一步
              q.push(DECISION_BOAT_SHIP);
              if (boat[first_id].SolveCollision(q, new_path)) {
                boat[first_id].path = new_path;
#ifdef DEBUG
                std::cerr << first_id << " 船让行" << std::endl;
#endif
                continue;
              }
              if (boat[second_id].SolveCollision(q, new_path)) {
                boat[second_id].path = new_path;
#ifdef DEBUG
                std::cerr << second_id << " 船让行" << std::endl;
#endif
                continue;
              }
            }
            // 其他解法
          }
        } else {
          // 一个旋转 一个直行

#ifdef DEBUG
          std::cerr << "一个旋转 一个直行" << std::endl;
#endif
          int ship_id = boat[first_id].direction == boat[first_id].path[0]
                            ? first_id
                            : second_id;  // 直行的id
          int rot_id = boat[first_id].direction != boat[first_id].path[0]
                           ? first_id
                           : second_id;  // 旋转的id
#ifdef DEBUG
          std::cerr << ship_id << " 旋转," << rot_id << " 直行" << std::endl;
#endif
          bool is_clockwise =
              MoveType(boat[rot_id].direction, boat[rot_id].path[0]) ==
              DECISION_BOAT_ROT_CLOCKWISE;
          if (man == 6) {
            if (is_clockwise) {
              // 旋转的是顺时针的情况
              // 直行的顺逆可解
              q.push(DECISION_BOAT_ROT_CLOCKWISE);
              q.push(DECISION_BOAT_ROT_COUNTERCLOCKWISE);
            } else {
              // 旋转的是逆时针的情况
              // 直行的逆顺可解
              q.push(DECISION_BOAT_ROT_COUNTERCLOCKWISE);
              q.push(DECISION_BOAT_ROT_CLOCKWISE);
            }
            if (boat[ship_id].SolveCollision(q, new_path)) {
              boat[ship_id].path = new_path;
#ifdef DEBUG
              std::cerr << ship_id << " 船让行" << std::endl;
#endif
              continue;
            }
            // 直行的船解不开就用旋转的船解
            q.pop();
            q.pop();
            if (is_clockwise) {
              // 船是顺时针旋转，解法，逆时针旋转，然后直行一步
              q.push(DECISION_BOAT_ROT_COUNTERCLOCKWISE);
              q.push(DECISION_BOAT_SHIP);
              if (boat[rot_id].SolveCollision(q, new_path)) {
                boat[rot_id].path = new_path;
#ifdef DEBUG
                std::cerr << rot_id << " 船让行" << std::endl;
#endif
                continue;
              }
            } else {
              // 船是逆时针旋转，解法，顺时针旋转，然后直行一步
              q.push(DECISION_BOAT_ROT_CLOCKWISE);
              q.push(DECISION_BOAT_SHIP);
              if (boat[rot_id].SolveCollision(q, new_path)) {
                boat[rot_id].path = new_path;
#ifdef DEBUG
                std::cerr << rot_id << " 船让行" << std::endl;
#endif
                continue;
              }
            }

          } else if (man == 5) {
            if (is_clockwise) {
              // 旋转的是顺时针的情况
              // 直行的逆、直、直可解
              q.push(DECISION_BOAT_ROT_COUNTERCLOCKWISE);
            } else {
              // 旋转的是逆时针的情况
              // 直行的顺、直、直可解
              q.push(DECISION_BOAT_ROT_CLOCKWISE);
            }
            q.push(DECISION_BOAT_SHIP);
            q.push(DECISION_BOAT_SHIP);
            if (boat[ship_id].SolveCollision(q, new_path)) {
              boat[ship_id].path = new_path;
#ifdef DEBUG
              std::cerr << ship_id << " 船让行" << std::endl;
#endif
              continue;
            }
            // 直行的船解不开就用旋转的船解
            q.pop();
            q.pop();
            q.pop();
            if (is_clockwise) {
              // 船是顺时针旋转，解法，逆时针旋转，然后直行一步
              q.push(DECISION_BOAT_ROT_COUNTERCLOCKWISE);
              q.push(DECISION_BOAT_SHIP);
              if (boat[rot_id].SolveCollision(q, new_path)) {
                boat[rot_id].path = new_path;
#ifdef DEBUG
                std::cerr << rot_id << " 船让行" << std::endl;
#endif
                continue;
              }
            } else {
              // 船是逆时针旋转，解法，顺时针旋转，然后直行一步
              q.push(DECISION_BOAT_ROT_CLOCKWISE);
              q.push(DECISION_BOAT_SHIP);
              if (boat[rot_id].SolveCollision(q, new_path)) {
                boat[rot_id].path = new_path;
#ifdef DEBUG
                std::cerr << rot_id << " 船让行" << std::endl;
#endif
                continue;
              }
            }
          } else if (man == 4) {
            if (is_clockwise) {
              // 旋转的是顺时针的情况
              // 直行的逆、直可解
              q.push(DECISION_BOAT_ROT_COUNTERCLOCKWISE);
            } else {
              // 旋转的是逆时针的情况
              // 直行的顺、直可解
              q.push(DECISION_BOAT_ROT_CLOCKWISE);
            }
            q.push(DECISION_BOAT_SHIP);
            if (boat[ship_id].SolveCollision(q, new_path)) {
              boat[ship_id].path = new_path;
#ifdef DEBUG
              std::cerr << ship_id << " 船让行" << std::endl;
#endif
              continue;
            }
            // 直行的船解不开就用旋转的船解
            q.pop();
            q.pop();
            if (is_clockwise) {
              // 船是顺时针旋转，解法，逆时针旋转，然后直行一步
              q.push(DECISION_BOAT_ROT_COUNTERCLOCKWISE);
              q.push(DECISION_BOAT_SHIP);
              if (boat[rot_id].SolveCollision(q, new_path)) {
                boat[rot_id].path = new_path;
#ifdef DEBUG
                std::cerr << rot_id << " 船让行" << std::endl;
#endif
                continue;
              }
            } else {
              // 船是逆时针旋转，解法，顺时针旋转，然后直行一步
              q.push(DECISION_BOAT_ROT_CLOCKWISE);
              q.push(DECISION_BOAT_SHIP);
              if (boat[rot_id].SolveCollision(q, new_path)) {
                boat[rot_id].path = new_path;
#ifdef DEBUG
                std::cerr << rot_id << " 船让行" << std::endl;
#endif
                continue;
              }
            }

          } else {
#ifdef DEBUG
            std::cerr << "一船直行、另一船旋转，核心点曼哈顿距离错误"
                      << std::endl;
#endif
          }
        }

        // 不可解

#ifdef DEBUG
        std::cerr << "不可解" << std::endl;
#endif
      }
    }
    boat[first_id].DoMove();
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
  can_astar = true;
#ifdef DEBUG
  std::cerr << "-----------------------------------Robot-----------"
               "--------------------------"
            << std::endl;
#endif
  std::vector<NextPoint> next_points;
  std::vector<int> not_move_id;
  auto &robot = RentController::GetInstance()->robot;
  auto &ch = MapController::GetInstance()->ch;
  int robot_num = robot.size();
  for (int i = 0; i < robot_num; ++i) {
    // --------- 移动前动作 ---------
    if (robot[i].goods && ch[robot[i].x][robot[i].y] == 'B' &&
        robot[i].berth_id ==
            MapController::GetInstance()
                ->location_to_berth_id[Location(robot[i].x, robot[i].y)]) {
#ifdef DEBUG
      std::cerr << "robot " << i << " 卸货：(" << robot[i].x << ","
                << robot[i].y << ")" << std::endl;
#endif
      // 卸货
      Decision decision(DECISION_TYPE_ROBOT_PULL, i, -1);
      q_decision.push(decision);

      // 把货物金钱入队列
      auto &berth = MapController::GetInstance()->berth;
      berth[robot[i].berth_id].berth_goods_value.push(robot[i].goods_money);

      berth[robot[i].berth_id].total_value += robot[i].goods_money;

      // 下货的位置可能不是计算的位置，path里面还有内容
      robot[i].path.clear();
      // 决策，更新目标货物, 当前不持有货物
#ifdef DEBUG
      std::cerr << "robot " << i << " start FindTargetGoods" << std::endl;
#endif
      if (can_astar && robot[i].FindTargetGoods()) {
        // 有新的目标货物
        can_astar = DynamicParam::GetInstance()->GetMultipleAstar();
      }

#ifdef DEBUG
      std::cerr << "robot " << i << " finished FindTargetGoods after pull"
                << std::endl;
#endif
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
      } else if (!robot[i].target_goods) {
        // 机器人没目标货物，且该帧刚好生成了一个货物在脚底下
        is_get = true;
        robot[i].target_goods =
            MapController::GetInstance()->gds[robot[i].x][robot[i].y];
#ifdef DEBUG
        std::cerr << "robot " << i << " 装地上新生成的货：(" << robot[i].x
                  << "," << robot[i].y << ")" << std::endl;
#endif
      }
      if (is_get) {
        // 装货
        Decision decision(DECISION_TYPE_ROBOT_GET, i, -1);
        q_decision.push(decision);
        robot[i].goods_money = robot[i].target_goods->money;

        // 决策更新目标泊位和泊位权重
        if (can_astar) {
          robot[i].FindBerth(robot[i].x, robot[i].y);
          can_astar = DynamicParam::GetInstance()->GetMultipleAstar();
#ifdef DEBUG
          std::cerr << "成功更新目标泊位" << std::endl;
#endif
        }
        //当前持有货物
        robot[i].goods = true;
      }
    }
    // 空闲机器人
    if (robot[i].path.empty() && can_astar) {
      if (robot[i].goods) {
        // 如果有货物但是没路径
        robot[i].FindBerth(robot[i].x, robot[i].y);
      } else if (robot[i].target_goods) {
        // 有目标货物但是没路径
        Goods *target_goods = robot[i].target_goods;
        if (robot[i].FindPath(target_goods)) {
#ifdef DEBUG
          std::cerr << "robot " << i << " 为目标货物找到路径" << std::endl;
#endif
        } else {
#ifdef DEBUG
          std::cerr << "robot " << i << " 没有为目标货物找到路径" << std::endl;
#endif
        }

      } else if (robot[i].FindTargetGoods()) {
        // 有新的目标货物
#ifdef DEBUG
        std::cerr << "robot " << i << " 更新新的目标货物" << std::endl;
#endif
      } else {
        robot[i].ZonePlan();
#ifdef DEBUG
        std::cerr << "robot " << i << " 没有更新新的目标货物" << std::endl;
#endif
      }
      can_astar = DynamicParam::GetInstance()->GetMultipleAstar();
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
      if (!MapController::GetInstance()->IsMainRoad(next_x,
                                                    next_y)) {  // 不是主干道
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
    if (MapController::GetInstance()->IsMainRoad(robot[not_move_id[i]].x,
                                                 robot[not_move_id[i]].y)) {
      // 位于主干道不挡路
      continue;
    }
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
  // std::vector<Berth> &berth = MapController::GetInstance()->berth;
  for (int i = 0; i < not_move_size; ++i) {
    int robot_id = not_move_id[i];
    if (!MapController::GetInstance()->IsMainRoad(robot[robot_id].x,
                                                  robot[robot_id].y)) {
      ++MapController::GetInstance()
            ->busy_point[robot[robot_id].x][robot[robot_id].y];
    }
    if (can_astar &&
        MapController::GetInstance()
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
        MapController::GetInstance()
            ->berth[robot[robot_id].berth_id]
            .goods_manager.ResetFirstFreeGoods();

        // 找新的目标货物
        robot[robot_id].path.clear();
        if (robot[robot_id].FindTargetGoods()) {
#ifdef DEBUG
          std::cerr << "机器人成功更改新的路线" << std::endl;
#endif
        } else {
          robot[robot_id].ZonePlan();
#ifdef DEBUG
          std::cerr << "机器人无法更改新的路线" << std::endl;
#endif
        }
      } else {
        // 没有目标货物罚站，更换所属泊位

#ifdef DEBUG
        auto start = std::chrono::high_resolution_clock::now();
#endif
        // robot[robot_id].FindNeighborGoods();
        robot[robot_id].ZonePlan();
#ifdef DEBUG
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;
        std::cerr << "机器人" << robot_id << "换泊位耗时：" << duration.count()
                  << " ms" << std::endl;
#endif
      }
    }
  }
}

/*
 * 决策购买
 */
void DecisionManager::DecisionPurchase() {
  // 决策买机器人
  auto &rent_instance = RentController::GetInstance();
  auto &map_instance = MapController::GetInstance();
  auto &param_instance = DynamicParam::GetInstance();
  int rest_num = param_instance->GetMaxRobotNum() -
                 rent_instance->robot.size();  // 还可以租的机器人数量
  auto &berth = map_instance->berth;
  int berth_size = berth.size();
  extern int money;        // 全局金额
  int rest_money = money;  // 剩余金额
  if (rest_num) {
    if (rent_instance->boat.empty() && money < 10000) {
#ifdef DEBUG
      std::cerr << "还需要购买机器人，但是要留钱至少买一艘船" << std::endl;
#endif
    } else if (rest_money < 2000) {
#ifdef DEBUG
      std::cerr << "没钱买机器人了" << std::endl;
#endif
    } else {
      auto &goods = rent_instance->goods;
      // 每帧清空机器人需求
      while (!goods.empty()) {
        goods.pop();
      }
      std::vector<std::pair<double, Goods *>> target_goods;
      auto &purchase_point = map_instance->robot_purchase_point;
      // 遍历需要新增机器人捡的货物
      for (int i = 0; i < berth_size; ++i) {
        Goods *p_goods = berth[i].goods_manager.first_free_goods;
        Goods *head_goods = berth[i].goods_manager.head_goods;
#ifdef DEBUG
        std::cerr << "泊位 " << i << " 货物数量"
                  << berth[i].goods_manager.goods_num << std::endl;
#endif

        while (p_goods != head_goods) {
          if (p_goods->robot_id == -1) {
            int r_id = map_instance
                           ->nearest_r[p_goods->x]
                                      [p_goods->y];  // 距离该货物最近的购买点id
            if (r_id == -1) {
              // 夹缝中的货物
              p_goods = p_goods->next;
              continue;
            }
            int cal_man = std::abs(purchase_point[r_id].x - p_goods->x) +
                          std::abs(purchase_point[r_id].y - p_goods->y);
            if (cal_man > LIFETIME - id + p_goods->birth -
                              DynamicParam::GetInstance()->GetTolerantTime()) {
              p_goods = p_goods->next;
              continue;
            }
            int total_man =
                cal_man +
                std::abs(berth[i].GetNearestX(p_goods->x) - p_goods->x) +
                std::abs(berth[i].GetNearestY(p_goods->y) - p_goods->y);
            double per_money = 1.0 * p_goods->money / total_man;
            target_goods.push_back(std::make_pair(per_money, p_goods));
          }
          p_goods = p_goods->next;
        }
      }
      sort(target_goods.begin(), target_goods.end(),
           std::greater<std::pair<double, Goods *>>());
      int size = target_goods.size();
#ifdef DEBUG

      for (int i = 0; i < size; ++i) {
        std::cerr << " " << target_goods[i].first << std::endl;
      }
#endif
      for (int i = 0; i < size; ++i) {
        Goods *p_goods = target_goods[i].second;
        goods.push(p_goods);
        rent_instance->RentRobot(
            map_instance->nearest_r[p_goods->x][p_goods->y]);
#ifdef DEBUG
        std::cerr << "为货物(" << p_goods->x << "," << p_goods->y
                  << ")购买机器人" << std::endl;
#endif
        --rest_num;
        rest_money -= 2000;
        if (!rest_num) {
#ifdef DEBUG
          std::cerr << "已经买够机器人了" << std::endl;
#endif
          break;
        } else if (rent_instance->boat.empty() && rest_money < 10000) {
#ifdef DEBUG
          std::cerr << "还需要购买机器人，但是要留钱至少买一艘船" << std::endl;
#endif
          break;
        } else if (rest_money < 2000) {
#ifdef DEBUG
          std::cerr << "没钱买机器人了" << std::endl;
#endif
          break;
        }
      }
    }
  } else {
    // 机器人数量已经够了，不需要再购买
    auto &boat = rent_instance->boat;
    for (int i = 0; i < boat.size(); ++i) {
      boat[i].is_buy = false;
    }
  }

  rest_num = param_instance->GetMaxBoatNum() - rent_instance->boat.size();
  if (rest_num && rest_money > 8000) {
    // ------- 目前遇到的情况: 最多一帧买一艘船 -----------
    int berth_id = -1;  // 需要购买船的泊位
    int max_goods_num = 0;
    for (int i = 0; i < berth_size; ++i) {
      if (berth[i].boat_id == -1 && berth[i].goods_num > max_goods_num) {
        max_goods_num = berth[i].goods_num;
        berth_id = i;
      }
    }
    if (berth_id != -1) {
      int s_id = map_instance->nearest_s[berth[berth_id].x][berth[berth_id].y];
      rent_instance->RentBoat(s_id);
    }
  }
  // 决策买船

  // auto &boat_purchase_point =
  // MapController::GetInstance()->boat_purchase_point; size =
  // boat_purchase_point.size();

  // #ifdef DEBUG
  //   std::cerr << "当前船购买点数量：" << size << std::endl;
  // #endif
}