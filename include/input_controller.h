#ifndef INPUT_CONTROLLER_H_
#define INPUT_CONTROLLER_H_
#include "map_controller.h"
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

  // 填地图, 最后一个参数是二维数组的引用
  void FillMap(int x, int y, char (&chpy)[N][N]);
};
#endif