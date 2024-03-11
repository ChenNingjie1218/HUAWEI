#include <bits/stdc++.h>

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <list>

#include "Astar.h"
#include "output_controller.h"
#include "param.h"
// #define DEBUG
#ifdef DEBUG
FILE *debug_map_file = fopen("./debug_map.txt", "w");
FILE *debug_command_file = fopen("./debug.txt", "w");
#endif
/*
 * - · 空地
 * - * 海洋
 * - # 障碍
 * - A 机器人起始位置，总共10个
 * - B 大小为4*4，标识泊位的位置
 */
char ch[N][N];
bool gds[N][N] = {false};  // 该点是否有货物
int money, boat_capacity;
int id;  // 帧号
// 港口权重
int berth_weight[10];
// 货物
struct Goods {
  Goods() {
    this->pre = this;
    this->next = this;
  }
  Goods(int x, int y, int money, int birth) {
    this->x = x;
    this->y = y;
    this->money = money;
    this->birth = birth;
  }
  int birth;  // 生成帧
  int money;  // 价值
  int x;
  int y;
  Goods *pre;  // 双向链表连接货物
  Goods *next;  // 按生存周期排列的，具有队列性质，又可随机删除
};

// 机器人
struct Robot {
  int x, y;

  // 是否携带货物
  int goods;
  // 手里拿的物品的价值
  int goods_money;

  // 是否是正常运行状态
  int status;
  int mbx, mby;  //什么意思？

  // 目标港口
  int berth_id;
  Robot() {}
  Robot(int startX, int startY) {
    x = startX;
    y = startY;
  }

  Goods *target_goods;
  std::list<Point *> path;
  // 更新目标货物
  // void UpdateTargetGoods(Goods* goods);
  static void UpdateTargetGoods(int i);

  // 更新路线
  void UpdateTargetRoot();

  // 清目标货物，重置容忍次数
  void ResetTargetGoods();

  // 清除path
  void ClearPath() {
    std::cerr << "ClearPath: path size ---- " << path.size() << std::endl;
    std::list<Point *>::iterator it = path.begin();
    while (it != path.end()) {
      delete *it;
    }
    path.clear();
  }

  // 删除path的第一个点
  void RemoveFirst() {
    std::list<Point *>::iterator it = path.begin();
    path.remove(*it);
    free(*it);
  }

  // 在头位置添加一个点
  void AddFirst(int x, int y) {
    Point t;
    t.x = x;
    t.y = y;
    path.push_front(&t);
  }

  /*
   * 判断哪个机器人优先级高
   * @ret - 1 第一个优先级高
   * @ret - 2 第二个优先级高
   * 同等优先级默认第一个优先级高
   *
   * --- 优先级策略 ---
   * 优先级高到低：
   * - 都有货物价值高优先
   * - 有一个有货物，没货物优先
   * - 都没货物，目标货物生命周期少的优先
   * - 有人没目标货物，有目标货物的优先
   * - 都没目标货物，先判断的优先
   */
  //
  static int JudgePriority(Robot *first, Robot *second) {
    // 如果都有货物，价值高的优先
    if (first->goods && second->goods) {
      if (first->goods_money < second->goods_money) {
        return 2;
      } else {
        return 1;
      }
    } else if (first->goods) {
      // 如果有一个有货物，没货物的优先
      // 第一个机器人有，第二个机器人没有
      return 2;
    } else if (second->goods) {
      // 如果有一个有货物，没货物的优先
      // 第一个机器人没有，第二个机器人有
      return 1;
    } else {
      // 如果都没有货物，目标货物生命低的优先
      if (first->target_goods && second->target_goods) {
        // 都有目标货物
        if (first->target_goods->birth <= second->target_goods->birth) {
          return 1;
        } else {
          return 2;
        }
      } else if (first->target_goods) {
        // 有一个有目标货物
        // 第一个机器人有目标货物,第二个机器人没有
        return 1;
      } else if (second->target_goods) {
        // 有一个有目标货物
        // 第一个机器人没有目标货物，第二个有
        return 2;
      } else {
        // 都没有目标货物
        return 1;
      }
    }
  }

  //拿到货物后寻找港口
  static int FindBerth(int i);
} robot[robot_num + 10];

// 泊位
struct Berth {
  int x;
  int y;

  // 到虚拟点的时间
  int transport_time;

  // 每帧可以装载的物品数
  int loading_speed;

  //到泊位的船队列
  std::queue<int> q_boat;
  Berth() {}
  Berth(int x, int y, int transport_time, int loading_speed) {
    this->x = x;
    this->y = y;
    this->transport_time = transport_time;
    this->loading_speed = loading_speed;
  }
} berth[berth_num + 10];

// 船
struct Boat {
  // 货物数量
  int num;

  // 目标泊位，虚拟点为-1
  int pos;

  /*
   * 状态
   * 可能值：
   * - 0 运输中
   * - 1 运行状态
   * - 2 泊位外等待
   */
  int status;
  Boat() { num = 0; }

  //虚拟点选择泊位
  static void ChooseBerth(int i, int rand_berth);

  //离开港口的条件
  static void LeaveCond(int i);
} boat[10];

// 货物管理器
struct GoodsManager {
  Goods *head_goods = new Goods();
  /*
   * 将货物放入链表
   */
  void PushGoods(Goods *new_goods) {
    gds[new_goods->x][new_goods->y] = true;
    if (head_goods->next == head_goods) {
      // 空链表
      head_goods->next = new_goods;
      head_goods->pre = new_goods;
      new_goods->next = head_goods;
      new_goods->pre = head_goods;

    } else {
      new_goods->pre = head_goods->pre;
      head_goods->pre->next = new_goods;
      head_goods->pre = new_goods;
      new_goods->next = head_goods;
    }
  };

  // 删除货物
  void DeleteGoods(Goods *&goods) {
    gds[goods->x][goods->y] = false;
    goods->pre->next = goods->next;
    goods->next->pre = goods->pre;
    delete goods;
    goods = NULL;
  }

  // 刷新货物链表
  void FreshGoodsLists() {
    Goods *cur = head_goods->next;
    while (cur != head_goods) {
      if (id - cur->birth == LIFETIME) {
        Goods *temp = cur->next;
        DeleteGoods(cur);
        cur = temp;
      } else {
        // 剪枝
        break;
      }
    }
  }
} g_goodsmanager;

// 决策
struct Decision {
  /*
   * - 1 机器人move
   * - 2 机器人get
   * - 3 机器人pull
   * - 4 船ship
   * - 5 船go
   */
  int type;

  // 机器人或者船的id
  int id;

  // 第二参数
  int param;
  Decision(int type, int id, int param) {
    this->type = type;
    this->id = id;
    this->param = param;
  }
};

// 决策队列
std::queue<Decision> q_decision;
// 清空决策队列
void ClearQueue(std::queue<Decision> &q) {
  std::queue<Decision> empty;
  swap(empty, q);
}

/*
 * 落点节点
 * 下一步准备走这个点
 */
struct NextPoint {
  int x, y;
  // 机器人数量
  int count;
  // 要走这个点的机器人
  int list_robot[4];
  NextPoint() {}
  NextPoint(int x, int y) {
    this->x = x;
    this->y = y;
    count = 0;
  }
  /*
   * 落点选择机器人决策
   *
   * - 插入算法：插入排序
   */
  void PushRobot(int robot_id) {
    int i;
    for (i = count - 1; i >= 0; --i) {
      if (Robot::JudgePriority(&robot[list_robot[i]], &robot[robot_id]) == 1) {
        list_robot[i + 1] = robot_id;
        break;
      } else {
        list_robot[i + 1] = list_robot[i];
      }
    }
    // 新机器人优先级最高
    if (i < 0) {
      list_robot[0] = robot_id;
    }
    ++count;
  }

  // 做决策
  void OutPut() {
    // 该落点有机器人决策落入
    if (count) {
      int robot_id = list_robot[0];
      int param;
      if (this->x == robot[robot_id].x + 1) {
        param = DECISION_ROBOT_RIGHT;
      } else if (this->x == robot[robot_id].x - 1) {
        param = DECISION_ROBOT_LEFT;
      } else if (this->y == robot[robot_id].y + 1) {
        param = DECISION_ROBOT_UP;
      } else {
        param = DECISION_ROBOT_DOWN;
      }
      q_decision.push(Decision(DECISION_TYPE_ROBOT_MOVE, robot_id, param));

      // 如果走的是路径中的点，删除robot的path中走了的
      std::list<Point *>::const_iterator iter = robot[robot_id].path.begin();
      if (this->x == (*iter)->x && this->y == (*iter)->y) {
        robot[robot_id].RemoveFirst();
      }
    }
  }
};

// 初始化
void Init() {
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
  // 泊位数据
  for (int i = 0; i < berth_num; i++) {
    int id;
    scanf("%d", &id);
    scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time,
          &berth[id].loading_speed);
#ifdef DEBUG
    fprintf(
        debug_map_file,
        "泊位 %d: x = %d, y = %d, transport_time = %d, loading_speed = %d\n",
        id, berth[id].x, berth[id].y, berth[id].transport_time,
        berth[id].loading_speed);
#endif
  }
  // 船容积
  scanf("%d", &boat_capacity);
#ifdef DEBUG
  fprintf(debug_map_file, "船容量：%d\n", boat_capacity);
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
bool Input() {
  // if (scanf("%d%d", &id, &money) != EOF) {
  scanf("%d%d", &id, &money);
#ifdef DEBUG
  fprintf(debug_command_file, "id = %d, money = %d\n", id, money);
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
#ifdef DEBUG
    fprintf(debug_command_file, "goods %d info: x = %d, y = %d, money = %d\n",
            i, x, y, val);
#endif
    Goods *new_goods = new Goods(x, y, val, id);
    g_goodsmanager.PushGoods(new_goods);
  }

  // 机器人实时数据
  for (int i = 0; i < robot_num; i++) {
    int temp_goods = 0;
    scanf("%d%d%d%d", &robot[i].goods, &robot[i].x, &robot[i].y,
          &robot[i].status);
#ifdef DEBUG
    fprintf(debug_command_file,
            "robot %d info: goods = %d, x = %d, y = %d, status = %d\n", i,
            robot[i].goods, robot[i].x, robot[i].y, robot[i].status);
#endif
    //放置成功船上货物加一
    if (robot[i].goods - temp_goods == 1) {
      boat[berth[robot[i].berth_id].q_boat.front()].num++;
    }
  }

  // 船的实时数据
  for (int i = 0; i < 5; i++) {
    int temp_status = 0;
    scanf("%d%d\n", &temp_status, &boat[i].pos);

    // 到达泊位入队
    if (temp_status != boat[i].status && boat[i].pos != -1) {
      berth[boat[i].pos].q_boat.push(i);
    }

    // 离开泊位出队
    if (temp_status != boat[i].status && boat[i].pos == -1) {
      // 先到港口先出队，还未考虑到港口后在去别的港口的情况
      berth[boat[i].pos].q_boat.pop();
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
  //   return 1;
  // }
  // return 0;
}

/*
 * 机器人做决策
 * 移动后的动作决策建立在成功移动后
 * 所以移动前动作和移动放一个循环判断
 * 移动后动作单独判断做了移动决策的机器人
 *
 */
void DecisionRobot() {
  std::vector<NextPoint> next_points;
  for (int i = 0; i < 10; ++i) {
    // --------- 移动前动作 ---------
    if (robot[i].goods && ch[robot[i].x][robot[i].y] == 'B' &&
        !berth[robot[i].berth_id].q_boat.empty()) {
      // 卸货
      Decision decision(DECISION_TYPE_ROBOT_PULL, i, -1);
      q_decision.push(decision);

      // 增加泊位权重
      berth_weight[robot->berth_id]++;
      robot[i].berth_id = -1;
      // 决策，更新目标货物, 当前不持有货物
      Robot::UpdateTargetGoods(i);
      robot[i].goods = false;
    } else if (!robot[i].goods && robot[i].target_goods &&
               robot[i].target_goods->x == robot[i].x &&
               robot[i].target_goods->y == robot[i].y) {
      // 装货
      Decision decision(DECISION_TYPE_ROBOT_GET, i, -1);
      q_decision.push(decision);

      // 捡到货物将其从链表删除
      g_goodsmanager.DeleteGoods(robot[i].target_goods);

      // 决策更新目标泊位和泊位权重
      robot[i].berth_id = Robot::FindBerth(i);
      // berth_weight[robot[i].berth_id]++;

      //当前持有货物
      robot[i].goods = true;
    }
    if (!robot[i].goods) {
      Robot::UpdateTargetGoods(i);
    }
    std::cerr << "robot " << i << " path size:" << robot[i].path.size()
              << std::endl;
    // 存落点
    if (!robot[i].path.empty()) {
      std::list<Point *>::iterator iter = robot[i].path.begin();  //迭代器
      bool same_flag = false;
      for (int j = 0; j < next_points.size(); ++j) {
        if (next_points[j].x == (*iter)->x && next_points[j].y == (*iter)->y) {
          // 有相同落点
          same_flag = true;
          next_points[i].PushRobot(i);
          break;
        }
      }
      if (!same_flag) {
        next_points.push_back(NextPoint((*iter)->x, (*iter)->y));
      }
    }
  }

  // --------- 移动 ---------

  // 决策移动
  int size = next_points.size();
  // 判断对碰死锁，更新next_points
  for (int i = 0; i < size; ++i) {
    for (int j = i + 1; j < size; ++j) {
      if (next_points[i].count && next_points[j].count &&
          next_points[i].x == robot[next_points[j].list_robot[0]].x &&
          next_points[i].y == robot[next_points[j].list_robot[0]].y &&
          robot[next_points[i].list_robot[0]].x == next_points[j].x &&
          robot[next_points[i].list_robot[0]].y && next_points[j].y) {
        // 给优先级高的机器人让位
        /*
         * 判断让位机器人能不能让位
         * @param leave 让位标志
         * - 0 无法让位
         * - 1 向右让位
         * - 2 向左让位
         * - 3 向上让位
         * - 4 向下让位
         */
        int leave = 0;
        int give_up_point;
        int save_point;
        if (Robot::JudgePriority(&robot[next_points[i].list_robot[0]],
                                 &robot[next_points[j].list_robot[0]]) == 1) {
          /*
           * 这里有点绕作如下解释：
           * next_points[i].list_robot[0]是站在next_points[j]上的机器人
           * next_points[j].list_robot[0]是站在next_points[i]上的机器人
           * 所以next_points[i].list_robot[0]优先级高，就要放弃位置为next_points[j]
           */
          save_point = i;
          give_up_point = j;
        } else {
          save_point = j;
          give_up_point = i;
        }

        //企图向右让位
        if (ch[next_points[save_point].x + 1][next_points[save_point].y] ==
            '*') {
          bool can_leave = true;
          for (int k = 0; k < size; ++k) {
            if (next_points[save_point].x + 1 == next_points[k].x) {
              // 右边位置即将被某个机器人占用
              can_leave = false;
              break;
            }
          }
          if (can_leave) {
            leave = 1;
            // 增加新落点
            NextPoint add_next_point = NextPoint(next_points[save_point].x + 1,
                                                 next_points[save_point].y);
            // 给机器人安排新落点
            add_next_point.PushRobot(next_points[give_up_point].list_robot[0]);
            // 将让的位加入让位机器的人path
            robot[next_points[give_up_point].list_robot[0]].AddFirst(
                next_points[save_point].x, next_points[save_point].y);
            // 废弃旧落点
            next_points[give_up_point].count = 0;
            next_points.push_back(add_next_point);
            // 更新size
            size = next_points.size();
            break;
          }
        }

        // 企图向左让位
        if (ch[next_points[save_point].x - 1][next_points[save_point].y] ==
            '*') {
          bool can_leave = true;
          for (int k = 0; k < size; ++k) {
            if (next_points[save_point].x - 1 == next_points[k].x) {
              // 右边位置即将被某个机器人占用
              can_leave = false;
              break;
            }
          }
          if (can_leave) {
            leave = 2;
            // 增加新落点
            NextPoint add_next_point = NextPoint(next_points[save_point].x - 1,
                                                 next_points[save_point].y);
            // 给机器人安排新落点
            add_next_point.PushRobot(next_points[give_up_point].list_robot[0]);
            // 将让的位加入让位机器的人path
            robot[next_points[give_up_point].list_robot[0]].AddFirst(
                next_points[save_point].x, next_points[save_point].y);
            // 废弃旧落点
            next_points[give_up_point].count = 0;
            next_points.push_back(add_next_point);
            // 更新size
            size = next_points.size();
            break;
          }
        }

        // 企图向上让位
        if (ch[next_points[save_point].x][next_points[save_point].y + 1] ==
            '*') {
          bool can_leave = true;
          for (int k = 0; k < size; ++k) {
            if (next_points[save_point].y + 1 == next_points[k].y) {
              // 上边位置即将被某个机器人占用
              can_leave = false;
              break;
            }
          }
          if (can_leave) {
            leave = 3;
            // 增加新落点
            NextPoint add_next_point = NextPoint(next_points[save_point].x,
                                                 next_points[save_point].y + 1);
            // 给机器人安排新落点
            add_next_point.PushRobot(next_points[give_up_point].list_robot[0]);
            // 将让的位加入让位机器的人path
            robot[next_points[give_up_point].list_robot[0]].AddFirst(
                next_points[save_point].x, next_points[save_point].y);
            // 废弃旧落点
            next_points[give_up_point].count = 0;
            next_points.push_back(add_next_point);
            // 更新size
            size = next_points.size();
            break;
          }
        }

        // 企图向下让位
        if (ch[next_points[save_point].x][next_points[save_point].y - 1] ==
            '*') {
          bool can_leave = true;
          for (int k = 0; k < size; ++k) {
            if (next_points[save_point].y - 1 == next_points[k].y) {
              // 下边位置即将被某个机器人占用
              can_leave = false;
              break;
            }
          }
          if (can_leave) {
            leave = 4;
            // 增加新落点
            NextPoint add_next_point = NextPoint(next_points[save_point].x,
                                                 next_points[save_point].y - 1);
            // 给机器人安排新落点
            add_next_point.PushRobot(next_points[give_up_point].list_robot[0]);
            // 将让的位加入让位机器的人path
            robot[next_points[give_up_point].list_robot[0]].AddFirst(
                next_points[save_point].x, next_points[save_point].y);
            // 废弃旧落点
            next_points[give_up_point].count = 0;
            next_points.push_back(add_next_point);
            // 更新size
            size = next_points.size();
            break;
          }
        }

        // 无法让位 两个机器人放弃移动
        next_points[i].count = 0;
        next_points[j].count = 0;
        break;
      }
    }
  }
  // 下决策
  size = next_points.size();
  for (int i = 0; i < size; ++i) {
    next_points[i].OutPut();
  }

  // 如果移动决策移动后动作
  // --------- 移动后动作 ---------
}

/*
 * 寻找目标货物
 */
void Robot::UpdateTargetGoods(int i) {
  std::cerr << "UpdateTargetGoods" << std::endl;
  double goods_weight = 0, cur_weight = 0;
  Goods *p_goods = g_goodsmanager.head_goods->next;
  Goods *cur_goods = p_goods;
  std::list<Point *> route, path;
  // 遍历货物链表
  while (p_goods != g_goodsmanager.head_goods) {
    // 调用a*算法获取路径及其长度：p_goods的坐标为终点，robot：x、y是起点
    // 将长度和p_goods->money归一化加权作为权值，若大于当前权值则更新
    route = astar(ch, robot[i].x, robot[i].y, p_goods->x, p_goods->y);
    std::cerr << "route size:" << route.size() << std::endl;
    if (route.empty()) {
      std::cerr << "route empty" << std::endl;
      p_goods = p_goods->next;
      continue;
    }
    cur_weight =
        0.5 * (p_goods->money - 1) / 999 - 0.5 * (route.size() - 1) / 399.0 + 1;
    if (cur_weight > goods_weight) {
      std::cerr << "update" << std::endl;
      cur_goods = p_goods;
      goods_weight = cur_weight;
      // robot[i].ClearPath();  // 清空上一次计算的路径
      path = route;
    } else {
      // Robot::ClearPath(route);
    }
    p_goods = p_goods->next;
  }
  robot[i].target_goods = cur_goods;
  robot[i].path = path;
  std::cerr << "robot " << i << "update path, path size: " << path.size()
            << std::endl;
}

int Robot::FindBerth(int i) {
  int length = 0, fin_length = 50000, fin_j = 0;
  std::list<Point *> route;

  // 寻找最近的泊位
  for (int j = 0; j < 10; j++) {
    // Robot::ClearPath(route);  // 清空上一次计算的路径
    route = astar(ch, robot[i].x, robot[i].y, berth[j].x + 1, berth[j].y + 1);
    length = route.size();
    if (length < fin_length) {
      fin_length = length;
      robot[i].path = route;  //更新机器人的行动路径
      fin_j = j;
    }
  }
  return fin_j;
}

/*
 * 船做决策
 * 根据帧数据状态来决策
 */
void DecisionBoat() {
  //最大权重泊位，权重都为0就随机泊位
  int rand_berth = 0;

  for (int i = 0; i < 5; ++i) {
    // status 0 运输中 无需考虑决策
    if (boat[i].status == 1) {
      if (boat[i].pos == -1) {
        // 在虚拟点

        // 决策去哪个泊位
        Boat::ChooseBerth(i, rand_berth);
      } else {
        // 决策是否驶离
        Boat::LeaveCond(i);
      }
    } else if (boat[i].status == 2) {
      // 在等待
      // 可以决策是否换船舶，换哪个船舶
    }
  }
}

/*
 * 船在虚拟点选择泊位
 * 依据1 前往泊位的机器人数量
 */
void Boat::ChooseBerth(int i, int rand_berth) {
  int max_berth = 0;
  for (int j = 0; j < 10; j++) {
    if (berth_weight[j] > max_berth) {
      max_berth = j;
    }
  }
  if (max_berth == 0) {
    boat[i].pos = ++rand_berth % 10;
  } else {
    boat[i].pos = max_berth;
    berth_weight[max_berth] -= BERTH_WEIGHT_AFTER_BOAT_CHOOSE;  //权重减少
  }
  Decision decision(DECISION_TYPE_BOAT_SHIP, i, boat[i].pos);
  q_decision.push(decision);  // 决策入队
}

/*
 * 船在泊位何时离开
 * 决策依据：
 * 1 船的容量和当前装载量
 * 2 船在泊位停留的时间 ？
 * 3 正在前往该泊位的机器人数量 ？
 */
void Boat::LeaveCond(int i) {
  // 容量达到80%就走
  if (boat[i].num >= boat_capacity * 0.8) {
    Decision decision(DECISION_TYPE_BOAT_GO, i, -1);
    q_decision.push(decision);
  }
}

int main() {
  Init();
  for (int i = 0; i < 15000; ++i) {
    // #ifdef DEBUG
    //     if (i == 10) fclose(debug_command_file);
    // #endif
    Input();
    // --------- 准备阶段 ----------
    g_goodsmanager.FreshGoodsLists();  // 刷新货物链表
    ClearQueue(q_decision);            // 清空决策队列

    // --------- 决策阶段 ----------
    DecisionRobot();
    DecisionBoat();

    // --------- 输出阶段 ----------
    // 根据决策表输出
    while (!q_decision.empty()) {
      Decision next_decision = q_decision.front();
      q_decision.pop();
      switch (next_decision.type) {
        case DECISION_TYPE_ROBOT_MOVE:
          output_controller.SendMove(next_decision.id, next_decision.param);
#ifdef DEBUG
          fprintf(debug_command_file, "move %d %d\n", next_decision.id,
                  next_decision.param);
#endif
          break;
        case DECISION_TYPE_ROBOT_GET:
          output_controller.SendGet(next_decision.id);
#ifdef DEBUG
          fprintf(debug_command_file, "get %d\n", next_decision.id);
#endif
          break;
        case DECISION_TYPE_ROBOT_PULL:
          output_controller.SendPull(next_decision.id);
#ifdef DEBUG
          fprintf(debug_command_file, "pull %d\n", next_decision.id);
#endif
          break;
        case DECISION_TYPE_BOAT_SHIP:
          output_controller.SendShip(next_decision.id, next_decision.param);
#ifdef DEBUG
          fprintf(debug_command_file, "ship %d %d\n", next_decision.id,
                  next_decision.param);
#endif
          break;
        case DECISION_TYPE_BOAT_GO:
          output_controller.SendGo(next_decision.id);
#ifdef DEBUG
          fprintf(debug_command_file, "go %d\n", next_decision.id);
#endif
          break;
        default:
          std::cerr << "ERROR DECISION TYPE!" << std::endl;
          break;
      }
    }

    puts("OK");
#ifdef DEBUG
    fprintf(debug_command_file, "OK\n");
#endif
    fflush(stdout);
  }
//   while (Input()) {
//     // --------- 准备阶段 ----------
//     g_goodsmanager.FreshGoodsLists();  // 刷新货物链表
//     ClearQueue(q_decision);            // 清空决策队列

//     // --------- 决策阶段 ----------
//     DecisionRobot();
//     DecisionBoat();

//     // --------- 输出阶段 ----------
//     // 根据决策表输出
//     while (!q_decision.empty()) {
//       Decision next_decision = q_decision.front();
//       q_decision.pop();
//       switch (next_decision.type) {
//         case DECISION_TYPE_ROBOT_MOVE:
//           output_controller.SendMove(next_decision.id, next_decision.param);
// #ifdef DEBUG
//           fprintf(debug_command_file, "move %d %d", next_decision.id,
//                   next_decision.param);
// #endif
//           break;
//         case DECISION_TYPE_ROBOT_GET:
//           output_controller.SendGet(next_decision.id);
// #ifdef DEBUG
//           fprintf(debug_command_file, "get %d", next_decision.id);
// #endif
//           break;
//         case DECISION_TYPE_ROBOT_PULL:
//           output_controller.SendPull(next_decision.id);
// #ifdef DEBUG
//           fprintf(debug_command_file, "pull %d", next_decision.id);
// #endif
//           break;
//         case DECISION_TYPE_BOAT_SHIP:
//           output_controller.SendShip(next_decision.id, next_decision.param);
// #ifdef DEBUG
//           fprintf(debug_command_file, "ship %d %d", next_decision.id,
//                   next_decision.param);
// #endif
//           break;
//         case DECISION_TYPE_BOAT_GO:
//           output_controller.SendGo(next_decision.id);
// #ifdef DEBUG
//           fprintf(debug_command_file, "go %d", next_decision.id);
// #endif
//           break;
//         default:
//           std::cerr << "ERROR DECISION TYPE!" << std::endl;
//           break;
//       }
//     }
//     puts("OK");
//     fflush(stdout);
//   }
#ifdef DEBUG
  fclose(debug_command_file);
#endif
  return 0;
}
