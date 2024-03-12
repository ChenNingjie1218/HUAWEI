#include "output_controller.h"

#include <cstdio>
#include <iostream>
#include <queue>

#include "decision.h"
#include "param.h"
#ifdef DEBUG
extern FILE* debug_command_file;
#endif
OutputController* OutputController::instance_ = nullptr;
OutputController*& OutputController::GetInstance() {
  if (!instance_) {
    instance_ = new OutputController();
  }
  return instance_;
}

/*
 * 机器人如何移动
 * @param move_tag - 往哪移动
 * 可选值：
 * - 0:右
 * - 1:左
 * - 2:上
 * - 3:下
 */
void OutputController::SendMove(int robot_id, int move_tag) {
  printf("move %d %d\n", robot_id, move_tag);
  fflush(stdout);
}

/*
 * 获取货物
 */
void OutputController::SendGet(int robot_id) {
  printf("get %d\n", robot_id);
  fflush(stdout);
}

/*
 * 卸货
 */
void OutputController::SendPull(int robot_id) {
  printf("pull %d\n", robot_id);
  fflush(stdout);
}

/*
 * 某船移动到某泊位
 */
void OutputController::SendShip(int boat_id, int berth_id) {
  printf("ship %d %d\n", boat_id, berth_id);
  fflush(stdout);
}

/*
 * 某船从泊位驶出至虚拟点运输货物
 */
void OutputController::SendGo(int boat_id) {
  printf("go %d\n", boat_id);
  fflush(stdout);
}

// 将决策发送给判题器
void OutputController::Output() {
#ifdef DEBUG
  std::cerr << "-----------------------------------Output-----------"
               "--------------------------"
            << std::endl;
  std::cerr << DecisionManager::GetInstance()->q_decision.size() << std::endl;
  fprintf(
      debug_command_file,
      "-----------------------------OUTPUT:%d------------------------------\n",
      DecisionManager::GetInstance()->q_decision.size());

#endif
  // 根据决策表输出
  while (!DecisionManager::GetInstance()->q_decision.empty()) {
    Decision next_decision = DecisionManager::GetInstance()->q_decision.front();
    DecisionManager::GetInstance()->q_decision.pop();
    switch (next_decision.type) {
      case DECISION_TYPE_ROBOT_MOVE:
        SendMove(next_decision.id, next_decision.param);
#ifdef DEBUG
        fprintf(debug_command_file, "move %d %d\n", next_decision.id,
                next_decision.param);
#endif
        break;
      case DECISION_TYPE_ROBOT_GET:
        SendGet(next_decision.id);
#ifdef DEBUG
        fprintf(debug_command_file, "get %d\n", next_decision.id);
#endif
        break;
      case DECISION_TYPE_ROBOT_PULL:
        SendPull(next_decision.id);
#ifdef DEBUG
        fprintf(debug_command_file, "pull %d\n", next_decision.id);
#endif
        break;
      case DECISION_TYPE_BOAT_SHIP:
        SendShip(next_decision.id, next_decision.param);
#ifdef DEBUG
        fprintf(debug_command_file, "ship %d %d\n", next_decision.id,
                next_decision.param);
#endif
        break;
      case DECISION_TYPE_BOAT_GO:
        SendGo(next_decision.id);
#ifdef DEBUG
        fprintf(debug_command_file, "go %d\n", next_decision.id);
#endif
        break;
      default:
#ifdef DEBUG
        std::cerr << "ERROR DECISION TYPE!" << std::endl;
#endif
        break;
    }
  }

  puts("OK");
#ifdef DEBUG
  fprintf(debug_command_file, "OK\n");
#endif
  fflush(stdout);
}