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

  // 判断某一个点是否在这个矩形内
  bool IsPointInBox(int x, int y);

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

  // 这一帧，船需不需要动
  bool need_move = true;

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
  Boat operator=(const Boat &boat) {
    this->id_ = boat.id_;
    this->num = boat.num;
    this->x = boat.x;
    this->y = boat.y;
    this->direction = boat.direction;
    this->area_id = boat.area_id;
    this->status = boat.status;
    this->path = boat.path;
    return *this;
  }

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

  // 删除path的第一个点
  void RemoveFirst();

  // 判断交货条件
  bool DeliveryCond();

  // 寻找交货点
  void FindDeliveryPoint();

  // 寻找泊位
  void FindBerth();

  // 获取船的下一个位置, 返回Boat
  Boat GetNextLocation();

  // 判断船的左右两边是不是空的，返回int ,
  // -1:表示左边是空的，1:表示右边是空的，0表示两边都不是空的
  int JudgeLeftRight(std::vector<Boat> &boat);

  // 船判断出可以让步之后，添加策略,flag 表示左边是空的还是右边是空的
  void AddStrategyintoPath(int flag);
};

#endif