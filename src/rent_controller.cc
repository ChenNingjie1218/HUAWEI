#include "rent_controller.h"

RentController* RentController::instance_ = nullptr;

RentController*& RentController::GetInstance() {
  if (!instance_) {
    instance_ = new RentController();
  }
  return instance_;
}

// 租船
void RentController::RentBoat(int x, int y) { boat.push_back(Boat(x, y)); }

// 租机器人
void RentController::RentRobot(int x, int y) { robot.push_back(Robot(x, y)); }