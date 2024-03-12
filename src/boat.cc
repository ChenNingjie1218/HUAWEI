#include "boat.h"

#include "berth.h"
#include "param.h"
Boat boat[10];
extern Berth berth[berth_num + 10];
extern int boat_capacity;
int Boat::boat_capacity = 0;

Boat::Boat() { num = 0; }

/*
 * 船在虚拟点选择泊位
 * 依据1 前往泊位的机器人数量
 */
void Boat::ChooseBerth(int &rand_berth) {
  int max_berth = 0;
  for (int j = 0; j < 10; j++) {
    if (berth[j].weight > max_berth) {
      max_berth = j;
    }
  }
  if (max_berth == 0) {
    pos = ++rand_berth % 10;
  } else {
    pos = max_berth;
    berth[max_berth].weight -= BERTH_WEIGHT_AFTER_BOAT_CHOOSE;  //权重减少
  }
}

/*
 * 船在泊位何时离开
 * 决策依据：
 * 1 船的容量和当前装载量
 * 2 船在泊位停留的时间 ？
 * 3 正在前往该泊位的机器人数量 ？
 */
bool Boat::LeaveCond() {
  // 容量达到80%就走
  return num >= boat_capacity * 0.8;
}