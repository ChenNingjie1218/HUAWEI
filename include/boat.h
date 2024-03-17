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

  // 船舶停留时间 v2.0
  int waiting_time;

  /*
   * 状态
   * 可能值：
   * - 0 运输中
   * - 1 运行状态
   * - 2 泊位外等待
   */
  int status;

  //大于13000帧后用该变量保证船再捡一会货物
  int final_count;

  Boat();

  // 虚拟点选择泊位
  void ChooseBerth();

  // version:3.0: 选择固定的两个泊位
  void ChooseBerth3(int boat_id);

  // 离开港口的条件
  bool LeaveCond();

  // 判断是否更换港口
  bool ChangeBerth3(int boat_id);
};

#endif