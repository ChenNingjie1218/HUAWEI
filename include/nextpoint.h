#ifndef NEXTPOINT_H_
#define NEXTPOINT_H_
#include <vector>
/*
 * 落点节点
 * 下一步准备走这个点
 */
struct NextPoint {
  int x, y;
  // 机器人数量
  int count;
  // 要走这个点的机器人
  int list_robot[4];
  NextPoint() {}
  NextPoint(int x, int y, int robot_id);
  /*
   * 落点选择机器人决策
   *
   * - 插入算法：插入排序
   */
  void PushRobot(int robot_id, std::vector<int>& not_move_id);

  // 做决策
  // 剔除not_move_robot_id中能够让位的机器人id
  void OutPut(std::vector<int>& not_move_robot_id);

  // 检测是否存在死锁
  bool IsDeadLock(NextPoint& other_point);
};
#endif