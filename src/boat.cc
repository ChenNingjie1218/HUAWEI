#include "boat.h"

#include "berth.h"
#include "param.h"
Boat boat[10];
extern Berth berth[berth_num + 10];
int Boat::boat_capacity = 0;
extern int id;
// 随机泊位
int rand_berth = 0;

Boat::Boat() {
  num = 0;
  final_count = 0;
  waiting_time = 0;
}

/*
 * version:1.0
 * 船在虚拟点选择泊位
 * 依据1 前往泊位的机器人数量
 */
// void Boat::ChooseBerth(int rand_berth) {
void Boat::ChooseBerth() {
  int max_berth = 0, cur_berth = 0;
  // for (int j = 0; j < 10; j++) {
  //   if (berth[j].weight > max_berth) {
  //     max_berth = j;
  //   }
  // }
  if (max_berth == 0) {
    // cur_berth = ++rand_berth % 10;

    pos = ++rand_berth % 10;
    // if (berth[cur_berth].q_boat.empty() && cur_berth >= 0) {
    //   pos = cur_berth;
    // }

    // for (; rand_berth < 10; rand_berth++) {
    //   if (berth[rand_berth].q_boat.empty()) {
    //     pos = rand_berth;
    //   }
    // }
  } else {
    pos = max_berth;
    berth[max_berth].weight -= BERTH_WEIGHT_AFTER_BOAT_CHOOSE;  //权重减少
  }
}

/*
 * version:2.0
 * 船在虚拟点选择泊位
 * 依据1 前往泊位的机器人数量
 */
// void Boat::ChooseBerth(int &rand_berth) {
//   int max_berth = -1;
//   int cur_weight = 0, max_weight = 0;
//   for (int j = 0; j < 10; j++) {
//     cur_weight = berth[j].weight / boat_capacity + 1;
//     if (berth[j].weight > max_weight) {
//       max_berth = j;
//       max_weight = berth[j].weight;
//     }
//   }
//   if (max_berth == -1) {
//     pos = ++rand_berth % 10;
//   } else {
//     pos = max_berth;
//     berth[max_berth].weight -= boat_capacity;  // 权重减少
//     berth[max_berth].weight =
//         berth[max_berth].weight > 0 ? berth[max_berth].weight : 0;  //
//         权重非负
//   }
// }

/*
 * version:3.0
 * 船在虚拟点选择泊位
 * 依据1 泊位货物数量
 */
void Boat::ChooseBerth3(int i) {
  int target_berth = i * 2;
  int max_berth =
      berth[target_berth].goods_num >= berth[target_berth + 1].goods_num
          ? target_berth
          : (target_berth + 1);
  pos = max_berth;
}

/*
 * 船在泊位何时离开
 * 决策依据：
 * 1 船的容量和当前装载量
 * 2 船在泊位停留的时间
 * 3 正在前往该泊位的机器人数量 ？
 */
bool Boat::LeaveCond() {
  if (pos != -1 &&
      id > 15000 - berth[pos].transport_time - TOLERANT_LEAVE_TIME) {
    return true;
  }
  // 容量达到80%就走
  return num >= boat_capacity;
}

/*
 * 船何时更换泊位
 * 决策依据：
 * 1 船的容量和当前装载量
 * 2 船在泊位停留的时间
 * 3 另一个泊位的货物数量
 */
bool Boat::ChangeBerth3(int i) {
  int target_berth = i * 2;  // 负责的第一个泊位
  int other_berth = (pos == target_berth) ? target_berth + 1
                                          : target_berth;  // 船负责的另一个泊位
  if (id > 15000 - berth[target_berth].transport_time - CHANGE_BERTH_TIME -
               TOLERANT_LEAVE_TIME) {
    // 防止船换泊位后，不能回虚拟点了
    return false;
  }
  // 该船舶没货物了
  // 船还有很多容量且隔壁有很多货物
  if (berth[pos].goods_num < berth[pos].loading_speed &&
      boat[i].num < (boat_capacity * 0.3) &&
      berth[other_berth].goods_num > (boat_capacity * 0.8)) {
    pos = other_berth;  // 更新目标泊位
    return true;
  }
  return false;
}