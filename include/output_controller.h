#ifndef OUTPUT_CONTROLLER_H
#define OUTPUT_CONTROLLER_H
#include <stdio.h>
/*
 * 输出
 *
 */
struct OutputController {
  /*
   * 机器人如何移动
   * @param move_tag - 往哪移动
   * 可选值：
   * - 0:右
   * - 1:左
   * - 2:上
   * - 3:下
   */
  void SendMove(int robot_id, int move_tag) {
    printf("move %d %d", robot_id, move_tag);
    fflush(stdout);
  }

  /*
   * 获取货物
   */
  void SendGet(int robot_id) {
    printf("get %d", robot_id);
    fflush(stdout);
  }

  /*
   * 卸货
   */
  void SendPull(int robot_id) {
    printf("pull %d", robot_id);
    fflush(stdout);
  }

  /*
   * 某船移动到某泊位
   */
  void SendShip(int boat_id, int berth_id) {
    printf("ship %d %d", boat_id, berth_id);
    fflush(stdout);
  }

  /*
   * 某船从泊位驶出至虚拟点运输货物
   */
  void SendGo(int boat_id) {
    printf("go %d", boat_id);
    fflush(stdout);
  }
};
#endif