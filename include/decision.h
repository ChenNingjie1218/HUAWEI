#ifndef DECISION_H_
#define DECISION_H_
#include <queue>
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
  NextPoint(int x, int y);
  /*
   * 落点选择机器人决策
   *
   * - 插入算法：插入排序
   */
  void PushRobot(int robot_id);

  // 做决策
  void OutPut();
};

// 决策管理
struct DecisionManager {
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

  // 解决面对面死锁
  void SolveFaceToFaceDeadLock(std::vector<NextPoint>& next_points);

  // 决策队列
  std::queue<Decision> q_decision;
  static DecisionManager* instance_;
};

#endif
