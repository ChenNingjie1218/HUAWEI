#include "rent_controller.h"

#include "decision.h"
#include "map_controller.h"
#include "output_controller.h"
RentController *RentController::instance_ = nullptr;

RentController *&RentController::GetInstance() {
  if (!instance_) {
    instance_ = new RentController();
  }
  return instance_;
}

// 租船
void RentController::RentBoat(int purchase_point_id) {
  auto &boat_purchase_point = MapController::GetInstance()->boat_purchase_point;
  DecisionManager::GetInstance()->q_decision.push(Decision(
      DECISION_TYPE_BOAT_LBOAT, boat_purchase_point[purchase_point_id].x - 1,
      boat_purchase_point[purchase_point_id].y - 1));
}

// 租机器人
void RentController::RentRobot(int purchase_point_id, int type) {
  auto &robot_purchase_point =
      MapController::GetInstance()->robot_purchase_point;
  DecisionManager::GetInstance()->q_decision.push(Decision(
      DECISION_TYPE_ROBOT_LBOT, robot_purchase_point[purchase_point_id].x - 1,
      robot_purchase_point[purchase_point_id].y - 1, type));
}