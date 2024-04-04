#ifndef BERTH_H_
#define BERTH_H_
#include <map>
#include <queue>

#include "goods.h"
#include "param.h"
// 泊位
struct Berth {
  int id_;
  int x;
  int y;

  // 到交货点的时间
  int transport_time = 500;

  // 每帧可以装载的物品数
  int loading_speed;

  // 货物数量
  int goods_num;

  // 目标为这个泊位的boat id
  int boat_id = -1;

  // 所处区号
  int area_id = -1;

  /*
   * 已经算过的路径
   * first: 泊位id，交货点是-1
   * second: 路径
   */
  std::map<int, std::vector<int> > path;

  // 货物管理器
  GoodsManager goods_manager;

  //到泊位的船队列
  std::queue<int> q_boat;
  Berth() {}
  Berth(int id, int x, int y, int loading_speed);
};
#endif