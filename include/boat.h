#ifndef BOAT_H_
#define BOAT_H_
// 船
struct Boat {
  // 核心点坐标
  int x, y;

  /*
   * 方向
   * - 0 右
   * - 1 左
   * - 2 上
   * - 3 下
   */
  int direction;

  // 货物数量
  int num;

  // 目标泊位，虚拟点为-1
  int pos;

  // 船的最大容量
  static int boat_capacity;

  /*
   * 状态
   * 可能值：
   * - 0 正常行驶状态
   * - 1 恢复状态
   * - 2 装载状态
   */
  int status;

  Boat();
  Boat(int x, int y);

  // version:3.0: 选择固定的两个泊位
  void ChooseBerth3(int boat_id);

  // 离开港口的条件
  bool LeaveCond();

  // 判断是否更换港口
  bool ChangeBerth3(int boat_id, bool force = false);
};

#endif