#include "input_controller.h"

#include <cstdio>
#include <iostream>

#include "berth.h"
#include "boat.h"
#include "goods.h"
#include "map_controller.h"
#include "param.h"
#include "rent_controller.h"
#ifdef DEBUG
extern FILE* debug_command_file;
extern FILE* debug_map_file;
#endif
InputController* InputController::instance_ = nullptr;

InputController*& InputController::GetInstance() {
  if (!instance_) {
    instance_ = new InputController();
  }
  return instance_;
}
int money;  // 得分
int id;     // 帧号

// 初始化
void InputController::Init() {
  char(&ch)[N][N] = MapController::GetInstance()->ch;
  // 地图数据
  for (int i = 1; i <= n; i++) scanf("%s", ch[i] + 1);

#ifdef DEBUG
  for (int i = 1; i <= n; i++) {
    for (int j = 1; j <= n; j++) {
      fprintf(debug_map_file, "%c", ch[i][j]);
    }
    fprintf(debug_map_file, "\n");
  }
#endif

  MapController::GetInstance()->InitMapData();

#ifdef DEBUG
  fprintf(debug_map_file, "初始化地图数据完毕！\n");
#endif

  // 泊位数据
  std::vector<Berth>& berth = MapController::GetInstance()->berth;
  std::queue<std::pair<Location, int>> q;
  int berth_num;
  scanf("%d", &berth_num);
  for (int i = 0; i < berth_num; ++i) {
    int id, x, y, loading_speed;
    scanf("%d%d%d%d", &id, &x, &y, &loading_speed);
    berth.push_back(Berth(id, ++x, ++y, loading_speed));
    MapController::GetInstance()->InitBerthMap(id, x, y);

    // 处理nearest_berth
    MapController::GetInstance()->nearest_berth[x][y] = id;
    q.push(std::make_pair(Location(x, y), id));

#ifdef DEBUG
    fprintf(debug_map_file,
            "泊位 %d: x = %d, y = %d, loading_speed = "
            "%d\n",
            id, berth[id].x, berth[id].y, berth[id].loading_speed);
#endif
  }

#ifdef DEBUG
  fprintf(debug_map_file, "泊位数据处理完毕！\n");
#endif

  // 初始化离每个点最近的泊位id
  MapController::GetInstance()->InitNearestBerth(q);

#ifdef DEBUG
  fprintf(debug_map_file, "最近泊位处理完毕！\n");
#endif

  // 船容积
  scanf("%d", &Boat::boat_capacity);
#ifdef DEBUG
  fprintf(debug_map_file, "船容量：%d\n", Boat::boat_capacity);
#endif
  char okk[100];
  scanf("%s", okk);
#ifdef DEBUG
  fprintf(debug_map_file, "%s", okk);
  fclose(debug_map_file);
#endif
  printf("OK\n");
  fflush(stdout);

  // --------- 其他初始化 ----------
}

// 每帧的数据
void InputController::Input() {
#ifdef DEBUG
  fprintf(debug_command_file,
          "-----------------------------INPUT--------------------------\n");
#endif
  int temp_id = 0;
  std::vector<Berth>& berth = MapController::GetInstance()->berth;
  if (scanf("%d%d", &temp_id, &money) == EOF) {
#ifdef DEBUG
    fclose(debug_command_file);

    // 泊位上残留货物数量
    for (int i = 0; i < 10; ++i) {
      std::cerr << "船泊残留货物:" << berth[i].goods_num << std::endl;
    }
#endif
    exit(0);
  }
  // 计算往船上装了多少货物
  std::vector<Boat>& boat = RentController::GetInstance()->boat;
  int dis_id = temp_id - id;
  //   for (int i = 0; i < 10; i++) {
  // #ifdef DEBUG
  //     if (!berth[i].q_boat.empty() && boat[berth[i].q_boat.front()].status ==
  //     1 &&
  //         berth[i].goods_num == 0) {
  //       std::cerr << i << "空等" << std::endl;
  //     }
  // #endif

  //     if (!berth[i].q_boat.empty() && berth[i].goods_num > 0 &&
  //         boat[berth[i].q_boat.front()].status == 1) {
  //       int load_num = berth[i].loading_speed * dis_id;
  //       if (load_num >= berth[i].goods_num) {
  //         boat[berth[i].q_boat.front()].num += berth[i].goods_num;
  //         berth[i].goods_num = 0;
  //       } else {
  //         boat[berth[i].q_boat.front()].num += load_num;
  //         berth[i].goods_num -= load_num;
  //       }
  //       if (boat[berth[i].q_boat.front()].num > Boat::boat_capacity) {
  //         // 装载速度大于1的时候出现超额
  //         berth[i].goods_num +=
  //             boat[berth[i].q_boat.front()].num - Boat::boat_capacity;
  //         boat[berth[i].q_boat.front()].num = Boat::boat_capacity;
  //       }
  //     }
  //   }

  // 跳帧
#ifdef DEBUG
  if (dis_id > 1) {
    std::cerr << "跳帧：" << dis_id << std::endl;
  }
#endif

  id = temp_id;
#ifdef DEBUG
  std::cerr << "------------------帧数id: " << id << "------------------"
            << std::endl;
  fprintf(debug_command_file, "帧数 id = %d, 分数 money = %d\n", id, money);
#endif
  // 货物变化
  int num;
  scanf("%d", &num);
#ifdef DEBUG
  fprintf(debug_command_file, "change goods num = %d\n", num);
#endif
  for (int i = 1; i <= num; i++) {
    int x, y, val;
    scanf("%d%d%d", &x, &y, &val);
    ++x;
    ++y;
#ifdef DEBUG
    fprintf(debug_command_file, "goods %d info: x = %d, y = %d, money = %d\n",
            i, x, y, val);
#endif
    if (money > 0 && MapController::GetInstance()->CanRobotReach(x, y)) {
#ifdef GOODS_FILTER
      if (val < GoodsManager::GetInstance()->value_valve) {
#ifdef DEBUG
        std::cerr << "货物价值过低被抛弃: " << val << std::endl;
#endif
        // 忽略不值钱的货物
        continue;
      }
#endif

      Goods* new_goods = new Goods(x, y, val, id);
      GoodsManager::GetInstance()->PushGoods(new_goods);
      new_goods->area_id = MapController::GetInstance()->FindArea(x * n + y);
    } else if (!money) {
      GoodsManager::GetInstance()->RemoveExpiredGoods(x, y);
    }
  }

  // 机器人实时数据
  std::vector<Robot>& robot = RentController::GetInstance()->robot;
  int(&busy_point)[N][N] = MapController::GetInstance()->busy_point;
  int robot_num;
  scanf("%d", &robot_num);

#ifdef DEBUG
  fprintf(debug_command_file, "robot num:  %d\n", robot_num);
#endif

  for (int i = 0; i < robot_num; i++) {
    int robot_id, temp_goods, x, y;
    scanf("%d%d%d%d", &robot_id, &temp_goods, &x, &y);
    if (i >= robot.size()) {
      // 新增机器人
      robot.push_back(Robot(robot_id, temp_goods, ++x, ++y));
      robot[i].InitAccessedBerth();
    } else {
      // 旧机器人更新坐标
      robot[i].x = ++x;
      robot[i].y = ++y;
    }
    ++busy_point[robot[i].x][robot[i].y];
#ifdef DEBUG
    fprintf(debug_command_file, "robot %d info: goods = %d, x = %d, y = %d\n",
            robot_id, temp_goods, robot[i].x, robot[i].y);

#endif
    // 放置成功港口货物加一
    if (robot[i].pre_goods - temp_goods == 1) {
      berth[robot[i].berth_id].goods_num++;
      robot[i].berth_id = -1;
    }
    robot[i].goods = temp_goods;
    robot[i].pre_goods = temp_goods;
  }

  // 船的实时数据
  int boat_num;
  scanf("%d", &boat_num);

#ifdef DEBUG
  fprintf(debug_command_file, "boat num:  %d\n", boat_num);
#endif

  for (int i = 0; i < boat_num; i++) {
    int boat_id, goods_num, x, y, direction, temp_status;
    scanf("%d%d%d%d%d%d\n", &boat_id, &goods_num, &x, &y, &direction,
          &temp_status);
    if (i > RentController::GetInstance()->boat.size()) {
      boat.push_back(
          Boat(boat_id, goods_num, ++x, ++y, direction, temp_status));
    } else {
      boat[i].x = ++x;
      boat[i].y = ++y;
      boat[i].num = goods_num;
      boat[i].direction = direction;
    }
    // 到达泊位入队
    if (temp_status != boat[i].status && temp_status == 0 &&
        boat[i].pos != -1) {
      berth[boat[i].pos].q_boat.push(i);
    }

    boat[i].status = temp_status;
#ifdef DEBUG
    fprintf(debug_command_file, "boat %d info: status = %d, pos = %d\n", i,
            temp_status, boat[i].pos);
#endif
  }
  char okk[100];
  scanf("%s", okk);
#ifdef DEBUG
  fprintf(debug_command_file, "%s\n", okk);
#endif
}
