#include <bits/stdc++.h>
#include <stdio.h>
#include <unistd.h>

#include <cstdio>

#include "berth.h"
#include "decision.h"
#include "goods.h"
#include "input_controller.h"
#include "map_controller.h"
#include "output_controller.h"

#ifdef DEBUG
FILE *debug_map_file = fopen("./debug/debug_map.txt", "w");
FILE *debug_command_file = fopen("./debug/debug.txt", "w");
FILE *debug_output_file = fopen("./debug/cerr.txt", "w");
#endif

int main() {
#ifdef DEBUG
  setvbuf(debug_command_file, nullptr, _IONBF, 0);
  setvbuf(debug_map_file, nullptr, _IONBF, 0);
  int output_fd = fileno(debug_output_file);
  dup2(output_fd, STDERR_FILENO);
#endif
  InputController::GetInstance()->Init();
  for (int i = 0; i < 15000; ++i) {
    InputController::GetInstance()->Input();
    // --------- 准备阶段 ----------
    DecisionManager::GetInstance()->ClearQueue();  // 清空决策队列

    // 维护每个泊位的货物生命周期
    auto &berth = MapController::GetInstance()->berth;
    int berth_size = berth.size();
    for (int j = 0; j < berth_size; ++j) {
      berth[j].goods_manager.FreshGoodsLists();
    }

    // --------- 决策阶段 ----------
    DecisionManager::GetInstance()->DecisionRobot();
    DecisionManager::GetInstance()->DecisionBoat();
    DecisionManager::GetInstance()->DecisionPurchase();

    // --------- 输出阶段 ----------
    OutputController::GetInstance()->Output();
  }
#ifdef DEBUG
  fclose(debug_command_file);
  std::vector<Berth> &berth = MapController::GetInstance()->berth;
  // 泊位上残留货物数量
  int size = berth.size();
  for (int i = 0; i < size; ++i) {
    std::cerr << "船泊残留货物:" << berth[i].goods_num << std::endl;
  }

#endif
  return 0;
}
