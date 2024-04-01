#ifndef RENT_CONTROLLER_H_
#define RENT_CONTROLLER_H_

#include <vector>

#include "boat.h"
#include "robot.h"

// 租赁模块
class RentController {
 private:
  RentController() = default;
  RentController(const RentController& other) = default;
  static RentController* instance_;

 public:
  static RentController*& GetInstance();

  // 租船
  void RentBoat(int x, int y);

  // 租机器人
  void RentRobot(int x, int y);

  std::vector<Robot> robot;  // 当前的机器人

  std::vector<Boat> boat;  // 当前的船
};

#endif