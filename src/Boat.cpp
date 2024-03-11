#include "Boat.h"

extern Boat boat[10];
extern int boat_capacity;

/*
 * 船做决策
 * 根据帧数据状态来决策
 */
void DecisionBoat() {
  //最大权重泊位，权重都为0就随机泊位
  int rand_berth = 0;

  for (int i = 0; i < 5; ++i) {
    // status 0 运输中 无需考虑决策
    if (boat[i].status == 1) {
      if (boat[i].pos == -1) {
        // 在虚拟点

        // 决策去哪个泊位
        Boat::ChooseBerth(i, rand_berth);
      } else {
        // 决策是否驶离
        Boat::LeaveCond(i);
      }
    } else if (boat[i].status == 2) {
      // 在等待
      // 可以决策是否换船舶，换哪个船舶
    }
  }
}

/*
 * 船在虚拟点选择泊位
 * 依据1 前往泊位的机器人数量
 */
void Boat::ChooseBerth(int i, int rand_berth) {
  int max_berth = 0;
  for (int j = 0; j < 10; j++) {
    if (berth_weight[j] > max_berth) {
      max_berth = j;
    }
  }
  if (max_berth == 0) {
    boat[i].pos = ++rand_berth % 10;
  } else {
    boat[i].pos = max_berth;
    berth_weight[max_berth] -= BERTH_WEIGHT_AFTER_BOAT_CHOOSE;  //权重减少
  }
  Decision decision(DECISION_TYPE_BOAT_SHIP, i, boat[i].pos);
  q_decision.push(decision);  // 决策入队
}

/*
 * 船在泊位何时离开
 * 决策依据：
 * 1 船的容量和当前装载量
 * 2 船在泊位停留的时间 ？
 * 3 正在前往该泊位的机器人数量 ？
 */
void Boat::LeaveCond(int i) {
  // 容量达到80%就走
  if (boat[i].num >= boat_capacity * 0.8) {
    Decision decision(DECISION_TYPE_BOAT_GO, i, -1);
    q_decision.push(decision);
  }
}