#ifndef ROBOT_H_
#define ROBOT_H_

#include "astar.h"
#include "goods.h"

// 机器人
struct Robot {
  int x, y;

  // 是否携带货物
  int goods;

  //保存上一刻的货物状态
  int pre_goods;

  // 手里拿的物品的价值
  int goods_money;

  // 是否是正常运行状态
  int status;
  int mbx, mby;  //什么意思？
  // A*的深度
  int astar_deep = DEFAULT_A_STAR_DEEP;

  // 目标港口
  int berth_id = -1;
  Robot() = default;
  Robot(int startX, int startY);

  Goods *target_goods;
  std::vector<Location> path;

  // 可到达的港口
  std::vector<int> berth_accessed;

  // 更新目标货物
  void UpdateTargetGoods(int robot_id);

  // 更新路线
  void UpdateTargetRoot();

  // 清目标货物，重置容忍次数
  void ResetTargetGoods();

  // 清除path
  void ClearPath();

  // 删除path的第一个点
  void RemoveFirst();

  // 在头位置添加一个点
  void AddFirst(int x, int y);

  /*
   * 判断哪个机器人优先级高
   * @ret - 1 第一个优先级高
   * @ret - 2 第二个优先级高
   * 同等优先级默认第一个优先级高
   *
   * --- 优先级策略 ---
   * 优先级高到低：
   * - 都有货物价值高优先
   * - 有一个有货物，没货物优先
   * - 都没货物，目标货物生命周期少的优先
   * - 有人没目标货物，有目标货物的优先
   * - 都没目标货物，先判断的优先
   */
  //
  static int JudgePriority(Robot *first, Robot *second);

  //拿到货物后寻找港口
  void FindBerth(int start_x, int start_y);
};

#endif