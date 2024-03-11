#ifndef INPUT_CONTROLLER_H_
#define INPUT_CONTROLLER_H_
struct InputController {
  static InputController*& GetInstance();
  // 初始化
  void Init();

  // 每帧的数据
  void Input();
  static InputController* instance_;
};
#endif