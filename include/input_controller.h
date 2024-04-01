#ifndef INPUT_CONTROLLER_H_
#define INPUT_CONTROLLER_H_
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

  // 船到虚拟点的最大耗时
  int max_transport_time = 0;
};
#endif