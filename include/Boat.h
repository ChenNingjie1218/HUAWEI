// 船
struct Boat {
  // 货物数量
  int num;

  // 目标泊位，虚拟点为-1
  int pos;

  /*
   * 状态
   * 可能值：
   * - 0 运输中
   * - 1 运行状态
   * - 2 泊位外等待
   */
  int status;
  Boat() { num = 0; }

  //虚拟点选择泊位
  static void ChooseBerth(int i, int rand_berth);

  //离开港口的条件
  static void LeaveCond(int i);
};

extern Boat boat[10];
extern int boat_capacity;

#define DECISION_TYPE_BOAT_SHIP 4
#define DECISION_TYPE_BOAT_GO 5
#define BERTH_WEIGHT_AFTER_BOAT_CHOOSE 1  //船选择泊位后，泊位权重的减少
