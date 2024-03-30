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

  // version:3.0: 选择固定的两个泊位
  void ChooseBerth3(int boat_id);

  // 离开港口的条件
  bool LeaveCond();

  // 判断是否更换港口
  bool ChangeBerth3(int boat_id, bool force = false);
};

#endif