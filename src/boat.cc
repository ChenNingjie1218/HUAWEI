#include "boat.h"

#include <algorithm>
#include <iostream>

#include "berth.h"
#include "decision.h"
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
void Boat::ChooseBerth3(int boat_id) {
  int max_goods = 0;
  int target_pos = -1;
  for (int i = 0; i < 10; ++i) {
    // 剩余装载时间
    int time = 15000 - id - TOLERANT_LEAVE_TIME - 2 * berth[i].transport_time;
    if (berth[i].boat_id == -1 && time > 0) {
      // 在剩余时间内能装多少货物
      int can_load =
          std::min(time * berth[i].loading_speed, berth[i].goods_num);
      if (can_load > max_goods) {
        target_pos = i;
        max_goods = can_load;
      }
    }
  }
  if (target_pos == -1) {
    return;
  }
  berth[target_pos].boat_id = boat_id;
  pos = target_pos;
#ifdef DEBUG
  std::cerr << boat_id << " 船从虚拟点选择泊位: " << target_pos << " 货物数量："
            << berth[target_pos].goods_num << std::endl;
#endif
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
#ifdef DEBUG
    std::cerr << berth[pos].boat_id
              << " 船离开了，剩余货物数量: " << berth[pos].goods_num
              << " 船货物数量: " << num << std::endl;
#endif
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
bool Boat::ChangeBerth3(int boat_id, bool force) {
  if (berth[pos].goods_num > berth[pos].loading_speed && !force) {
    return false;
  }
  // 先看有没有船舶能让自己填满
  int target_pos = pos;
  for (int i = 0; i < 10; ++i) {
    int time = 15000 - id - berth[i].transport_time - CHANGE_BERTH_TIME -
               TOLERANT_LEAVE_TIME;
    int goods_num = std::min(time * berth[i].loading_speed, berth[i].goods_num);
    if (i != pos && goods_num >= boat_capacity - num) {
      if (!berth[i].q_boat.empty() &&
          berth[i].goods_num >= boat_capacity - boat[berth[i].boat_id].num) {
        // 该泊位有船 且它自己能装满
        continue;
      } else if (berth[i].boat_id == -1) {
        if (target_pos == pos) {
          // 找到的第一个能把该船填满的船舶
          target_pos = i;
        } else if (berth[i].transport_time < berth[target_pos].transport_time) {
          // 在都能把该船填满的情况下，选回家最快的
          target_pos = i;
        }
      }
    }
  }
  if (target_pos == pos) {
    // 没有能把该船填满的船舶
    int max_goods = berth[pos].goods_num;
    for (int i = 0; i < 10; ++i) {
      if (i != pos && berth[i].boat_id == -1) {
        int time = 15000 - id - berth[i].transport_time - CHANGE_BERTH_TIME -
                   TOLERANT_LEAVE_TIME;
        int can_load =
            std::min(time * berth[i].transport_time, berth[i].goods_num);
        if (can_load > max_goods) {
          target_pos = i;
          max_goods = can_load;
        }
      }
    }
  } else {
    if (!berth[target_pos].q_boat.empty() &&
        boat[berth[target_pos].boat_id].ChangeBerth3(berth[target_pos].boat_id,
                                                     true)) {
      // 如果该泊位有船
      // 让它换个坑位

#ifdef DEBUG
      std::cerr << boat_id
                << " 船抢占，该船舶有船: " << berth[target_pos].boat_id
                << std::endl;
#endif
      // 更换港口
      Decision decision(DECISION_TYPE_BOAT_SHIP, berth[target_pos].boat_id,
                        boat[berth[target_pos].boat_id].pos);
      DecisionManager::GetInstance()->q_decision.push(decision);  // 决策入队
    }
#ifdef DEBUG
    std::cerr << boat_id << " 船找到了能填满船的泊位，选择回家最快的泊位 "
              << target_pos << std::endl;
#endif
  }

  if (target_pos == pos) {
    return false;
  }

  // if (id > 15000 - berth[target_pos].transport_time - CHANGE_BERTH_TIME -
  //              TOLERANT_LEAVE_TIME) {
  //   // 防止船换泊位后，不能回虚拟点了
  //   return false;
  // }
#ifdef DEBUG
  std::cerr << boat_id << " 船更换泊位 " << pos << " -> " << target_pos
            << " 当前货物数量：" << num << std::endl;
#endif
  // 出队
  if (!berth[pos].q_boat.empty()) {
    berth[pos].q_boat.pop();
  }
  berth[pos].boat_id = -1;
  pos = target_pos;
  berth[target_pos].boat_id = boat_id;
  return true;
}