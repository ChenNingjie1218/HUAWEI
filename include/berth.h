#ifndef BERTH_H_
#define BERTH_H_
#include <queue>
// 泊位
struct Berth {
  int x;
  int y;

  // 到虚拟点的时间
  int transport_time;

  // 每帧可以装载的物品数
  int loading_speed;

  // 权重
  int weight;

  //到泊位的船队列
  std::queue<int> q_boat;
  Berth() {}
  Berth(int x, int y, int transport_time, int loading_speed);
};
#endif