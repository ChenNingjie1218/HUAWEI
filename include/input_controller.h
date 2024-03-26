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

  // 机器人初始位置 该位置还无法对应机器人id 可以用来辨别地图
  std::vector<Location> robot_initial_position;

  // 坐标映射到泊位id
  std::map<Location, int> location_to_berth_id;

  // 船到虚拟点的最大耗时
  int max_transport_time = 0;

  // 用并查集分区
  // 找集合
  int FindArea(int id);
  // 合并集合
  void MergeArea(int id_1, int id_2);
};
#endif