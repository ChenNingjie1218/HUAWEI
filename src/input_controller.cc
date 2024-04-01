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
  int(&busy_point)[N][N] = MapController::GetInstance()->busy_point;
  int(&parent)[N * N] = MapController::GetInstance()->parent;
  for (int i = 1; i <= n; ++i) {
    for (int j = 1; j <= n; ++j) {
      // 记录购买点、交货点
      if (ch[i][j] == 'R')
        MapController::GetInstance()->robot_purchase_point.push_back(
            std::make_pair(i, j));
      else if (ch[i][j] == 'S')
        MapController::GetInstance()->boat_purchase_point.push_back(
            std::make_pair(i, j));
      else if (ch[i][j] == 'T')
        MapController::GetInstance()->delivery_point.push_back(
            std::make_pair(i, j));

      // 初始化堵车标记
      busy_point[i][j] = 0;

      // 初始化区域号
      parent[i * n + j] = i * n + j;
    }
  }

  for (int i = 1; i <= n; ++i) {
    for (int j = 1; j <= n; ++j) {
      if (ch[i][j] == '.' || ch[i][j] == 'A' || ch[i][j] == 'B') {
        if (ch[i - 1][j] == '.' || ch[i - 1][j] == 'A' || ch[i - 1][j] == 'B') {
          MapController::GetInstance()->MergeArea(i * n + j, (i - 1) * n + j);
        }
        if (ch[i][j - 1] == '.' || ch[i][j - 1] == 'A' || ch[i][j - 1] == 'B') {
          MapController::GetInstance()->MergeArea(i * n + j, i * n + j - 1);
        }
      }
    }
  }

  // 泊位数据
  int& berth_num = MapController::GetInstance()->berth_num;
  std::vector<Berth>& berth = MapController::GetInstance()->berth;
  for (int i = 0; i < berth_num; i++) {
    int id;
    scanf("%d", &id);
    scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time,
          &berth[id].loading_speed);
    ++berth[id].x;
    ++berth[id].y;
    MapController::GetInstance()->InitBerthMap(id, berth[id].x, berth[id].y);
    berth[id].area_id =
        MapController::GetInstance()->FindArea(berth[id].x * n + berth[id].y);
#ifdef DEBUG
    fprintf(debug_map_file,
            "泊位 %d: x = %d, y = %d, transport_time = %d, loading_speed = "
            "%d\n",
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
      if (boat[berth[i].q_boat.front()].num > Boat::boat_capacity) {
        // 装载速度大于1的时候出现超额
        berth[i].goods_num +=
            boat[berth[i].q_boat.front()].num - Boat::boat_capacity;
        boat[berth[i].q_boat.front()].num = Boat::boat_capacity;
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
  int robot_num = RentController::GetInstance()->robot.size();
  int(&busy_point)[N][N] = MapController::GetInstance()->busy_point;
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
      robot[i].berth_id = -1;
      // boat[berth[robot[i].berth_id].q_boat.front()].num++;
    }
    robot[i].goods = temp_goods;
    robot[i].pre_goods = temp_goods;

    // 第一帧更新机器人可达的泊位、区号
    if (id == 1) {
      robot[i].id_ = i;  // 初始化robot的id
      robot[i].area_id =
          MapController::GetInstance()->FindArea(robot[i].x * n + robot[i].y);
      robot[i].InitAccessedBerth();
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
