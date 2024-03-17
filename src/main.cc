#include <bits/stdc++.h>
#include <stdio.h>
#include <unistd.h>

#include <cstdio>

#include "astar.h"
#include "berth.h"
#include "decision.h"
#include "goods.h"
#include "input_controller.h"
#include "output_controller.h"

#ifdef DEBUG
FILE *debug_map_file = fopen("./debug/debug_map.txt", "w");
FILE *debug_command_file = fopen("./debug/debug.txt", "w");
FILE *debug_output_file = fopen("./debug/cerr.txt", "w");
extern Berth berth[berth_num + 10];
#endif

int main() {
#ifdef DEBUG
  setvbuf(debug_command_file, nullptr, _IONBF, 0);
  int output_fd = fileno(debug_output_file);
  dup2(output_fd, STDERR_FILENO);
#endif
  InputController::GetInstance()->Init();
  for (int i = 0; i < 15000; ++i) {
    // #ifdef DEBUG
    //     if (i == 1) fclose(debug_command_file);
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

  // 泊位上残留货物数量
  for (int i = 0; i < 10; ++i) {
    std::cerr << "船上残留货物:" << berth[i].goods_num << std::endl;
  }

  // 地上残留货物
  Goods *head_goods = GoodsManager::GetInstance()->head_goods;
  Goods *cur = head_goods->next;
  while (cur != head_goods) {
    std::cerr << "地上残留货物: " << cur->money << std::endl;
    cur = cur->next;
  }
#endif
  return 0;
}
