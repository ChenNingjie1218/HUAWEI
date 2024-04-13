#ifndef RENT_CONTROLLER_H_
#define RENT_CONTROLLER_H_

#include <vector>

#include "boat.h"
#include "robot.h"

// 租赁模块
class RentController {
 private:
  RentController() = default;
  RentController(const RentController &other) = default;
  static RentController *instance_;

 public:
  static RentController *&GetInstance();

  // 租船
  void RentBoat(int purchase_point_id);

  /*
   * 租机器人
   * type 0 容量1
   * type 1 容量2
   */
  void RentRobot(int purchase_point_id, int type = 0);

  std::vector<Robot> robot;  // 当前的机器人

  std::vector<Boat> boat;  // 当前的船

  std::queue<Goods *> goods;  // 这些货物需要新的机器人来捡
};

#endif