#ifndef OUTPUT_CONTROLLER_H
#define OUTPUT_CONTROLLER_H
/*
 * 输出
 *
 */
struct OutputController {
 private:
  OutputController() = default;
  OutputController(const OutputController& other) = default;
  static OutputController* instance_;

 public:
  static OutputController*& GetInstance();
  /**
   * @brief 购买机器人指令
   * @param x - 机器人的x坐标
   * @param y - 机器人的y坐标
   */
  void SendLbot(int x, int y);

  /**
   * @brief 机器人如何移动
   * @param robot_id - 机器人id
   * @param move_tag - 往哪移动
   * 可选值：
   * - 0:右
   * - 1:左
   * - 2:上
   * - 3:下
   */
  void SendMove(int robot_id, int move_tag);
  /**
   * @brief 获取货物
   * @param robot_id - 机器人id
   */
  void SendGet(int robot_id);
  /**
   * @brief 卸货
   * @param robot_id - 机器人id
   */
  void SendPull(int robot_id);

  /**
   * @brief 购买船指令
   * @param x - 船的x坐标
   * @param y - 船的y坐标
   */
  void SendLboat(int x, int y);

  /**
   * @brief 尝试将对应船位置重置到主航道上，会导致船进入恢复状态。
   * @param boat_id - 船的id
   */
  void SendDept(int boat_id);

  /**
   * @brief 尝试将对应船靠泊到泊位上，会导致船进入恢复状态。
   * @param boat_id - 船的id
   */
  void SendBerth(int boat_id);

  /**
   * @brief 船的旋转
   * @param boat_id - 船的id
   * @param rotate_tag - 旋转的方向 0:顺时针 1:逆时针
   */
  void SendRot(int boat_id, int rotate_tag);

  /**
   *@brief 向正方向前进1格（主航道）
   *@param boat_id - 船的id
   */
  void SendShip(int boat_id);

  // 将决策发送给判题器
  void Output();
};
#endif