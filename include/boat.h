#ifndef BOAT_H_
#define BOAT_H_
#include <vector>

#include "param.h"
/*
 * AABB最小边界框
 *
 */
class CollisionBox {
 public:
  CollisionBox() = delete;
  CollisionBox(int core_x, int core_y, int direction);
  // 是否撞边界
  bool IsCollision();
  // 判断是否处于主航道
  bool IsLocatedOnMainRoute();
  // 两个对象是否相撞
  static bool JudgeCollision(const CollisionBox &first,
                             const CollisionBox &second);

 private:
  int l_x, l_y;  // 左上角坐标
  int r_x, r_y;  // 右上角坐标
};

// 船
struct Boat {
  // 船id
  int id_;

  // 货物数量
  int num;

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

  // 所处区号
  int area_id;

#ifdef DEBUG
  static char dir_str[4][10];
#endif

  // 目标泊位，虚拟点为-1
  int pos = -1;

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

  // 船的路径，只用存方向
  std::vector<int> path;

  Boat();
  Boat(int &id, int &goods_num, int &x, int &y, int &direction, int &status);

  void ChooseBerth3(int boat_id);

  // 顺时针旋转
  void DoClockwiseRotate();

  // 逆时针旋转
  void DoCounterclockwiseRotate();

  // 往前走
  void DoShip();

  // 靠泊
  void DoBerth();

  // 重置到主航道
  void DoDept();

  // 离开港口的条件
  bool LeaveCond();

  // 判断是否更换港口
  bool ChangeBerth3(int boat_id, bool force = false);

  // 删除path的第一个点
  void RemoveFirst();

  // 判断交货条件
  bool DeliveryCond();

  // 寻找交货点
  void FindDeliveryPoint();

  // 寻找泊位
  void FindBerth();
};

#endif