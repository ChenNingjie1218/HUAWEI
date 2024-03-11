#ifndef BOAT_H_
#define BOAT_H_
// 船
struct Boat {
  // 货物数量
  int num;

  // 目标泊位，虚拟点为-1
  int pos;

  // 船的最大容量
  static int boat_capacity;
  /*
   * 状态
   * 可能值：
   * - 0 运输中
   * - 1 运行状态
   * - 2 泊位外等待
   */
  int status;
  Boat();

  //虚拟点选择泊位
  void ChooseBerth(int rand_berth);

  //离开港口的条件
  bool LeaveCond();
};

#endif