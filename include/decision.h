#ifndef DECISION_H_
#define DECISION_H_
#include <queue>
// 决策
struct Decision {
  /*
   * - 0 机器人lbot
   * - 1 机器人move
   * - 2 机器人get
   * - 3 机器人pull
   * - 4 船lboat
   * - 5 船dept
   * - 6 船berth
   * - 7 船rot
   * - 8 船ship
   */
  int type;

  // 第一个参数
  int param_1;

  // 第二参数
  int param_2;

  // 第三个参数
  int param_3;

  Decision(int type, int param_1, int param_2 = -1, int param_3 = -1);
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
   * 决策购买
   */
  void DecisionPurchase();

  // 决策队列
  std::queue<Decision> q_decision;

  bool boat_first = false;
};

#endif
