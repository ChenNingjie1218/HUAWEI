#ifndef INPUT_CONTROLLER_H_
#define INPUT_CONTROLLER_H_
#include <map>

#include "astar.h"
struct InputController {
 private:
  InputController() = default;
  InputController(const InputController& other) = default;
  static InputController* instance_;

 public:
  static InputController*& GetInstance();
  // 初始化
  void Init();

  // 每帧的数据
  void Input();

  // 初始化坐标映射到泊位id的map
  void InitBerthMap(int berth_id, int berth_x, int berth_y);

  // 坐标映射到泊位id
  std::map<Location, int> location_to_berth_id;
};
#endif