#include <bits/stdc++.h>

#include "decision.h"
#include "goods.h"
#include "input_controller.h"
#include "output_controller.h"

// #define DEBUG
#ifdef DEBUG
FILE *debug_map_file = fopen("./debug_map.txt", "w");
FILE *debug_command_file = fopen("./debug.txt", "w");
#endif

int main() {
  InputController::GetInstance()->Init();
  for (int i = 0; i < 15000; ++i) {
    // #ifdef DEBUG
    //     if (i == 10) fclose(debug_command_file);
    // #endif
    InputController::GetInstance()->Input();
    // --------- 准备阶段 ----------
    GoodsManager::GetInstance()->FreshGoodsLists();  // 刷新货物链表
    DecisionManager::GetInstance()->ClearQueue();    // 清空决策队列

    // --------- 决策阶段 ----------
    DecisionManager::GetInstance()->DecisionRobot();
    DecisionManager::GetInstance()->DecisionBoat();

    // --------- 输出阶段 ----------
    OutputController::GetInstance()->Output();
  }
#ifdef DEBUG
  fclose(debug_command_file);
#endif
  return 0;
}
