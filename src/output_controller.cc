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

/**
 * @brief 购买机器人指令
 * @param x - 机器人的x坐标
 * @param y - 机器人的y坐标
 */
void OutputController::BuyRobot(int x, int y) {
  printf("lbot %d %d\n", x, y);
  fflush(stdout);
}

/**
 * @brief 机器人如何移动
 * @param robot_id - 机器人id
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

/**
 * @brief 获取货物
 * @param robot_id - 机器人id
 */
void OutputController::SendGet(int robot_id) {
  printf("get %d\n", robot_id);
  fflush(stdout);
}

/**
 * @brief 卸货
 * @param robot_id - 机器人id
 */
void OutputController::SendPull(int robot_id) {
  printf("pull %d\n", robot_id);
  fflush(stdout);
}

/**
 * @brief 购买船指令
 * @param x - 船的x坐标
 * @param y - 船的y坐标
 */
void OutputController::BuyBoat(int x, int y){
  printf("lboat %d %d\n", x, y);
  fflush(stdout);

}

/**
  * @brief 尝试将对应船位置重置到主航道上，会导致船进入恢复状态。
  * @param boat_id - 船的id
  */
void OutputController::SendReset(int boat_id){
  printf("reset %d\n", boat_id);
  fflush(stdout);
}

/**
  * @brief 尝试将对应船靠泊到泊位上，会导致船进入恢复状态。
  * @param boat_id - 船的id 
*/
void OutputController::SendDock(int boat_id){
  printf("dock %d\n", boat_id);
  fflush(stdout);
}

/**
  * @brief 船的旋转
  * @param boat_id - 船的id
  * @param rotate_tag - 旋转的方向 0:顺时针 1:逆时针
*/
void OutputController::SendRotate(int boat_id, int rotate_tag){
  printf("rotate %d %d\n", boat_id, rotate_tag);
  fflush(stdout);
}

/**
  *@brief 向正方向前进1格（主航道）
  *@param boat_id - 船的id
*/
void OutputController::SendForward(int boat_id){
  printf("forward %d\n", boat_id);
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
      "-----------------------------OUTPUT:%ld------------------------------\n",
      DecisionManager::GetInstance()->q_decision.size());

#endif
  // 根据决策表输出
  while (!DecisionManager::GetInstance()->q_decision.empty()) {
    Decision next_decision = DecisionManager::GetInstance()->q_decision.front();
    DecisionManager::GetInstance()->q_decision.pop();
    switch (next_decision.type) {
      case DECISION_TYPE_ROBOT_BUY:
        BuyRobot(next_decision.id, next_decision.param);
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
      case DECISION_TYPE_BOAT_BUY:
        BuyBoat(next_decision.id, next_decision.param);
      case DECISION_TYPE_BOAT_RESET:
        SendReset(next_decision.id);
      case DECISION_TYPE_BOAT_SHIP:
        SendDock(next_decision.id);
        break;
      case DECISION_TYPE_BOAT_ROTATE:
        SendRotate(next_decision.id, next_decision.param);
      case DECISION_TYPE_BOAT_FORWARD:
        SendForward(next_decision.id);
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