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

  // 放在该泊位的货物
  std::vector<int> goods;
  // 指向第一个没被装上船的货物下标
  int first_index = 0;

  Berth() {}
  Berth(int id, int x, int y, int loading_speed);

  // 'B'的坐标集合
  std::vector<Location> loc;
  int loc_index = 0;

  // 获取该泊位最近的x坐标
  int GetNearestX(const int &x);

  // 获取该泊位最近的y坐标
  int GetNearestY(const int &y);

  // 获取船到该泊位能装多少钱
  int GetIdealMoney(const int &capacity);
};
#endif