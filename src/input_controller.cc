#include "input_controller.h"

#include <cstdio>
#include <iostream>

#include "berth.h"
#include "boat.h"
#include "goods.h"
#include "param.h"
#include "robot.h"
extern Berth berth[berth_num + 10];
extern Robot robot[robot_num + 10];
extern Boat boat[10];
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
/*
 * - · 空地
 * - * 海洋
 * - # 障碍
 * - A 机器人起始位置，总共10个
 * - B 大小为4*4，标识泊位的位置
 */
char ch[N][N];
int money;
int id;                // 帧号
int busy_point[N][N];  // 堵车点

// 初始化
void InputController::Init() {
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

  // 记录机器人初始位置
  for (int i = 1; i <= n; i++) {
    for (int j = 1; j <= n; j++) {
      if (ch[i][j] == 'A') {
        robot_initial_position.push_back(Location(i, j));
      }
      busy_point[i][j] = 0;
    }
  }

  // 泊位数据
  for (int i = 0; i < berth_num; i++) {
    int id;
    scanf("%d", &id);
    scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time,
          &berth[id].loading_speed);
    ++berth[id].x;
    ++berth[id].y;
    InitBerthMap(id, berth[id].x, berth[id].y);
    InitReachableBerth(id, berth[id].x, berth[id].y);
#ifdef DEBUG
    fprintf(
        debug_map_file,
        "泊位 %d: x = %d, y = %d, transport_time = %d, loading_speed = %d\n",
        id, berth[id].x, berth[id].y, berth[id].transport_time,
        berth[id].loading_speed);
#endif
    if (berth[id].transport_time > max_transport_time) {
      max_transport_time = berth[id].transport_time;
    }
  }
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
  // if (scanf("%d%d", &id, &money) != EOF) {
  // }
  int temp_id = 0;
  if (scanf("%d%d", &temp_id, &money) == EOF) {
#ifdef DEBUG
    fclose(debug_command_file);

    // 泊位上残留货物数量
    for (int i = 0; i < 10; ++i) {
      std::cerr << "船泊残留货物:" << berth[i].goods_num << std::endl;
    }

    // 地上残留货物
    Goods* head_goods = GoodsManager::GetInstance()->head_goods;
    Goods* cur = head_goods->next;
    while (cur != head_goods) {
      std::cerr << "地上残留货物: " << cur->money << std::endl;
      cur = cur->next;
    }
#endif
    exit(0);
  }
  // 计算往船上装了多少货物
  int dis_id = temp_id - id;
  for (int i = 0; i < 10; i++) {
#ifdef DEBUG
    if (!berth[i].q_boat.empty() && boat[berth[i].q_boat.front()].status == 1 &&
        berth[i].goods_num == 0) {
      std::cerr << i << "空等" << std::endl;
    }
#endif

    if (!berth[i].q_boat.empty() && berth[i].goods_num > 0 &&
        boat[berth[i].q_boat.front()].status == 1) {
      int load_num = berth[i].loading_speed * dis_id;
      if (load_num >= berth[i].goods_num) {
        boat[berth[i].q_boat.front()].num += berth[i].goods_num;
        berth[i].goods_num = 0;
      } else {
        boat[berth[i].q_boat.front()].num += load_num;
        berth[i].goods_num -= load_num;
      }
    }
  }

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
  // 新增货物
  int num;
  scanf("%d", &num);
#ifdef DEBUG
  fprintf(debug_command_file, "new goods num = %d\n", num);
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
    if (ch[x][y] == '.' || ch[x][y] == 'A' || ch[x][y] == 'B') {
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
    }
  }

  // 机器人实时数据
  for (int i = 0; i < robot_num; i++) {
    int temp_goods = 0;
    scanf("%d%d%d%d", &temp_goods, &robot[i].x, &robot[i].y, &robot[i].status);
    ++robot[i].x;
    ++robot[i].y;
    ++busy_point[robot[i].x][robot[i].y];
#ifdef DEBUG
    fprintf(debug_command_file,
            "robot %d info: goods = %d, x = %d, y = %d, status = %d\n", i,
            temp_goods, robot[i].x, robot[i].y, robot[i].status);
    if (!robot[i].status) {
      fprintf(debug_command_file, "碰撞\n");
    }
#endif
    // 放置成功港口货物加一
    if (robot[i].pre_goods - temp_goods == 1) {
      berth[robot[i].berth_id].goods_num++;
      berth[robot[i].berth_id].weight++;
      robot[i].berth_id = -1;
      // boat[berth[robot[i].berth_id].q_boat.front()].num++;
    }
    robot[i].goods = temp_goods;
    robot[i].pre_goods = temp_goods;

    // 第一帧更新机器人可达的泊位
    if (id == 1) {
      robot[i].berth_accessed =
          reachable_berths[Location(robot[i].x, robot[i].y)];
    }
  }

  // 船的实时数据
  for (int i = 0; i < 5; i++) {
    int temp_status = 0;
    scanf("%d%d\n", &temp_status, &boat[i].pos);

    // 到达泊位入队
    if (temp_status != boat[i].status && temp_status == 0 &&
        boat[i].pos != -1) {
      berth[boat[i].pos].q_boat.push(i);
    }

    // 离开泊位出队
    // if (temp_status != boat[i].status && boat[i].pos == -1) {
    //   // 先到港口先出队，还未考虑到港口后在去别的港口的情况
    //   if (!berth[boat[i].pos].q_boat.empty()) {
    //     berth[boat[i].pos].q_boat.pop();
    //   }
    // }
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

// 初始化坐标映射到泊位id的map
void InputController::InitBerthMap(int berth_id, int berth_x, int berth_y) {
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      location_to_berth_id[Location(berth_x + i, berth_y + j)] = berth_id;
    }
  }
}

// 初始化每个机器人可达的泊位
void InputController::InitReachableBerth(int berth_id, int berth_x,
                                         int berth_y) {
  int size = robot_initial_position.size();
  for (int i = 0; i < size; ++i) {
    Astar astar(robot_initial_position[i].x, robot_initial_position[i].y,
                berth_x + 1, berth_y + 1);
    if (astar.AstarSearch(berth_id)) {
      reachable_berths[robot_initial_position[i]].push_back(berth_id);
    }
  }
}