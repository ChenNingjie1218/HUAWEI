#ifndef BOAT_H_
#define BOAT_H_
#include <map>
#include <queue>
#include <vector>

#include "astar.h"
#include "param.h"
/*
 * AABB最小边界框
 *
 */
class CollisionBox {
 public:
  CollisionBox() = delete;
  CollisionBox(int core_x, int core_y, int direction);
  CollisionBox(int core_x, int core_y, int direction, int next_location);
  // 是否撞边界
  bool IsCollision();
  // 判断是否处于主航道
  bool IsLocatedOnMainRoute();
  // 判断是否完全处于主航道
  bool IsCompletelyLocatedOnMainRoute();
  // 是否包含某点
  bool IsInclude(int x, int y);

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
  static char dir_str[4][10];  // 输出方向
  int total_money = 0;
#endif

  // 目标泊位，虚拟点为-1，购买点为-2
  int pos = -2;

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

  // 船堵塞的次数
  int stuck_times = 0;

  // 交货点到泊位的路径
  static std::map<std::pair<Location, int>, std::vector<int>> d_path;

  // 船上装的货物价值
  int money = 0;

  // 船走过的路程
  int total_transport_time = 0;

  Boat();
  Boat(int &id, int &goods_num, int &x, int &y, int &direction, int &status);

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

  // 给该船下移动指令
  void DoMove();

  /*
   * 解决碰撞
   * @param q 解决步骤
   * @param new_path 新路径
   * @return 是否成功解决
   */
  bool SolveCollision(std::queue<int> q, std::vector<int> &path);
};

/*
 * 行动类型
 * @param origin_direction 初始方向
 * @param next_direction 下一步的方向
 * @return 行动类型
 */
int MoveType(int origin_direction, int next_direction);
#endif