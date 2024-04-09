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
extern int money;
extern int id;

int main(int argc, char *argv[]) {
  int FLAG = 0, index;
  index = std::stoi(argv[1]);
  std::string path;
  if (index != -1) {
    path = "./gasa/data/data" + std::to_string(index) + ".txt";
  } else {
    path = "./gasa/data.txt";
  }
  std::ifstream inputFile(path.c_str());  // 打开文件用于读取
  if (inputFile.is_open()) {
    std::string line;
    std::getline(inputFile, line);
    DynamicParam::GetInstance()->SetTolerantTime(std::stoi(line));
    std::getline(inputFile, line);
    DynamicParam::GetInstance()->SetTolerantLeaveTime(std::stoi(line));
    std::getline(inputFile, line);
    DynamicParam::GetInstance()->SetGoodsValueValve(std::stoi(line));
    std::getline(inputFile, line);
    DynamicParam::GetInstance()->SetGoodsFilterValveNum(std::stoi(line));
    std::getline(inputFile, line);
    DynamicParam::GetInstance()->SetValueableGoodsValve(std::stoi(line));
    std::getline(inputFile, line);
    DynamicParam::GetInstance()->SetFinalTolerantTime(std::stoi(line));
    std::getline(inputFile, line);
    DynamicParam::GetInstance()->SetBusyValve(std::stoi(line));
    std::getline(inputFile, line);
    DynamicParam::GetInstance()->SetBoatCapacityReduce(std::stoi(line));
    std::getline(inputFile, line);
    DynamicParam::GetInstance()->SetMaxRobotNum(std::stoi(line));
    std::getline(inputFile, line);
    DynamicParam::GetInstance()->SetMaxBoatNum(std::stoi(line));
    std::getline(inputFile, line);
    FLAG = std::stoi(line);
    inputFile.close();  // 关闭文件
    std::cerr << "读取data成功" << std::endl;
  } else {
    std::cerr << "sdg无法打开文件进行读取操作。" << std::endl;
  }
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
    if (id >= 14950) {
      if (FLAG == 1) {
        std::string s;
        if (index == -1) {
          s = "./gasa/money.txt";
        } else {
          s = "./gasa/money/money" + std::to_string(index) + ".txt";
        }
        std::ofstream outputFile1(s.c_str(), std::ios::out | std::ios::trunc);
        // std::cerr << "123" << std::endl;
        if (outputFile1.is_open()) {
          // std::cerr << "456yyy" << std::endl;
          outputFile1 << money << std::endl;
          outputFile1.close();  // 关闭文件
          // std::cerr << "写入money成功" << std::endl;
        } else {
          std::cerr << "无法打开文件进行写入操作。" << std::endl;
        }
      }
    }
  }
  return 0;
}
