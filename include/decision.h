#ifndef DECISION_H_
#define DECISION_H_
#include <queue>
#include <vector>
// 决策
struct Decision {
  /*
   * - 1 机器人move
   * - 2 机器人get
   * - 3 机器人pull
   * - 4 船ship
   * - 5 船go
   */
  int type;

  // 机器人或者船的id
  int id;

  // 第二参数
  int param;
  Decision(int type, int id, int param);
};

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
};

// 决策管理
struct DecisionManager {
 private:
  DecisionManager() = default;
  DecisionManager(const DecisionManager& other) = default;
  static DecisionManager* instance_;

 public:
  static DecisionManager*& GetInstance();
  // 清空决策队列
  void ClearQueue();
  /*
   * 船做决策
   * 根据帧数据状态来决策
   */
  void DecisionBoat();

  /*
   * 机器人做决策
   * 移动后的动作决策建立在成功移动后
   * 所以移动前动作和移动放一个循环判断
   * 移动后动作单独判断做了移动决策的机器人
   *
   */
  void DecisionRobot();

  /*
   * 机器人是否拦路
   * @ret next_points的下标
   */
  int IsBlock(int robot_id, std::vector<NextPoint>& next_points);

  // 让路
  bool GetAway(int robot_id, std::vector<NextPoint>& next_points, int ignore_id,
               std::vector<int>& not_move_id);
  // 检测是否存在死锁
  bool IsDeadLock(NextPoint& first_point, NextPoint& second_point);

  // 决策队列
  std::queue<Decision> q_decision;
};

#endif
