#ifndef BERTH_H_
#define BERTH_H_
#include <map>
#include <queue>
#include <set>

#include "goods.h"
#include "param.h"
#include "robot.h"
// 泊位
struct Berth {
  int id_;
  int x;
  int y;

  // 到交货点的时间
  int transport_time;

  // 每帧可以装载的物品数
  int loading_speed;

  // 货物数量
  int goods_num;

  // 目标为这个泊位的boat id
  int boat_id = -1;

  // 所处区号
  int area_id = -1;

  // 维护一个队列，用于记录泊位每个货物的金钱
  std::queue<int> berth_goods_value;

  // 维护泊位的实时总金额
  int total_value = 0;

  /*
   * 已经算过的路径
   * first: 泊位id，交货点是-1
   * second: 路径
   */
  std::map<int, std::vector<int> > path;

  // 货物管理器
  GoodsManager goods_manager;

  // 机器人队列
  std::vector<int> robot;  // 当前的机器人

  // 邻居
  std::set<int> neighbor;

  //到泊位的船队列
  std::queue<int> q_boat;
  Berth() {}
  Berth(int id, int x, int y, int loading_speed);

  // 'B'的坐标集合
  std::vector<Location> loc;
  int loc_index = 0;

  // 获取该泊位最近的x坐标
  int GetNearestX(const int &x);

  // 获取该泊位最近的y坐标
  int GetNearestY(const int &y);
};
#endif