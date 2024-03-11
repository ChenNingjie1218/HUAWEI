#ifndef OUTPUT_CONTROLLER_H
#define OUTPUT_CONTROLLER_H
/*
 * 输出
 *
 */
struct OutputController {
  static OutputController*& GetInstance();
  /*
   * 机器人如何移动
   * @param move_tag - 往哪移动
   * 可选值：
   * - 0:右
   * - 1:左
   * - 2:上
   * - 3:下
   */
  void SendMove(int robot_id, int move_tag);
  /*
   * 获取货物
   */
  void SendGet(int robot_id);
  /*
   * 卸货
   */
  void SendPull(int robot_id);
  /*
   * 某船移动到某泊位
   */
  void SendShip(int boat_id, int berth_id);

  /*
   * 某船从泊位驶出至虚拟点运输货物
   */
  void SendGo(int boat_id);

  // 将决策发送给判题器
  void Output();
  static OutputController* instance_;
};
#endif