#include "rent_controller.h"

RentController *RentController::instance_ = nullptr;

RentController *&RentController::GetInstance() {
  if (!instance_) {
    instance_ = new RentController();
  }
  return instance_;
}

// 租船
void RentController::RentBoat(int &id, int &goods_num, int &x, int &y,
                              int &direction, int &status) {
  boat.push_back(Boat(id, goods_num, x, y, direction, status));
}

// 租机器人
void RentController::RentRobot(int &id, int &goods, int &x, int &y) {
  robot.push_back(Robot(id, goods, x, y));
}