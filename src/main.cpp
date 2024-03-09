#include <bits/stdc++.h>

#include <cstdio>
#include <string>

#include "Astar.h"
#include "output_controller.h"

using namespace std;
// 货物生存周期
const int LIFETIME = 20;

// 地图长度
const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;

int money, boat_capacity;
int id; // 帧号

#define BERTH_WEIGHT_AFTER_BOAT_CHOOSE 1 //船选择泊位后，泊位权重的减少

/*
 * - · 空地
 * - * 海洋
 * - # 障碍
 * - A 机器人起始位置，总共10个
 * - B 大小为4*4，标识泊位的位置
 */
char ch[N][N];
bool gds[N][N] = {false}; // 该点是否有货物

// 港口权重
int berth_weight[10];

// 货物
struct Goods {
  Goods(int x, int y, int money, int birth) {
    this->x = x;
    this->y = y;
    this->money = money;
    this->birth = birth;
  }
  int birth; // 生成帧
  int money; // 价值
  int x;
  int y;
  Goods* pre; // 双向链表连接货物
  Goods* next; // 按生存周期排列的，具有队列性质，又可随机删除
};

// 机器人
struct Robot {
  int x, y;

  // 是否携带货物
  int goods;

  // 是否是正常运行状态
  int status;
  int mbx, mby; //什么意思？

  // 目标港口
  int berth_id;
  Robot() {}
  Robot(int startX, int startY) {
    x = startX;
    y = startY;
  }
  Goods* target_goods;

  // 更新目标货物
  // void UpdateTargetGoods(Goods* goods);
  static void UpdateTargetGoods(int i);

  // 更新路线
  void UpdateTargetRoot();

  // 清目标货物，重置容忍次数
  void ResetTargetGoods();

  //拿到货物后寻找港口
  int FindBerth();
} robot[robot_num + 10];

// 泊位
struct Berth {
  int x;
  int y;
  // 到虚拟点的时间
  int transport_time;
  // 每帧可以装载的物品数
  int loading_speed;
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
  Goods* head_goods;
  /*
   * 将货物放入链表
   */
  void PushGoods(Goods*& new_goods) {
    gds[new_goods->x][new_goods->y] = true;
    if (head_goods->next == NULL) {
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
  void DeleteGoods(Goods*& goods) {
    gds[goods->x][goods->y] = false;
    goods->pre->next = goods->next;
    goods->next->pre = goods->pre;
    delete goods;
    goods = NULL;
  }

  // 刷新货物链表
  void FreshGoodsLists() {
    Goods* cur = head_goods->next;
    while (cur != head_goods) {
      if (id - cur->birth == LIFETIME) {
        Goods* temp = cur->next;
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
queue<Decision> q_decision;
// 清空决策队列
void ClearQueue(queue<Decision>& q) {
  queue<Decision> empty;
  swap(empty, q);
}
// 初始化
void Init() {
  // 地图数据
  for (int i = 1; i <= n; i++)
    scanf("%s", ch[i] + 1);
  // 泊位数据
  for (int i = 0; i < berth_num; i++) {
    int id;
    scanf("%d", &id);
    scanf("%d%d%d%d", &berth[id].x, &berth[id].y, &berth[id].transport_time,
          &berth[id].loading_speed);
  }
  // 船容积
  scanf("%d", &boat_capacity);
  char okk[100];
  scanf("%s", okk);
  printf("OK\n");
  fflush(stdout);

  // --------- 其他初始化 ----------
}

// 每帧的数据
bool Input() {
  if (scanf("%d%d", &id, &money) != EOF) {
    // 新增货物
    int num;
    scanf("%d", &num);
    for (int i = 1; i <= num; i++) {
      int x, y, val;
      scanf("%d%d%d", &x, &y, &val);
      Goods* new_goods = new Goods(x, y, val, id);
      g_goodsmanager.PushGoods(new_goods);
    }

    // 机器人实时数据
    for (int i = 0; i < robot_num; i++) {
      scanf("%d%d%d%d", &robot[i].goods, &robot[i].x, &robot[i].y,
            &robot[i].status);
    }

    // 船的实时数据
    for (int i = 0; i < 5; i++)
      scanf("%d%d\n", &boat[i].status, &boat[i].pos);

    char okk[100];
    scanf("%s", okk);
    return 1;
  }
  return 0;
}

/*
 * 机器人做决策
 * 移动后的动作决策建立在成功移动后
 * 所以移动前动作和移动放一个循环判断
 * 移动后动作单独判断做了移动决策的机器人
 *
 */
void DecisionRobot() {
  for (int i = 0; i < 10; ++i) {
    // --------- 移动前动作 ---------
    if (robot[i].goods && ch[robot[i].x][robot[i].y] == 'B') {
      // 卸货
      Decision decision(3, i, -1);
      q_decision.push(decision);

      // 增加泊位权重
      berth_weight[robot->berth_id]++;
      robot[i].berth_id = -1;
    }

    // 决策，更新目标货物
    Robot::UpdateTargetGoods(i);

    if (!robot[i].goods && robot[i].target_goods &&
        robot[i].target_goods->x == robot[i].x &&
        robot[i].target_goods->y == robot[i].y) {
      // 装货
      Decision decision(2, i, -1);
      q_decision.push(decision);

      // 决策更新目标泊位和泊位权重
      robot[i].berth_id = robot[i].FindBerth();
      berth_weight[robot[i].berth_id]++;
    }
  }

  // --------- 移动 ---------
  // 决策是否移动

  // 如果移动决策移动后动作
  // --------- 移动后动作 ---------
}

void Robot::UpdateTargetGoods(int i) {
  double goods_weight = 0, cur_weight = 0;
  Goods* p_goods = g_goodsmanager.head_goods->next;
  Goods* cur_goods = p_goods;

  // 遍历货物链表
  while (p_goods) {
    // 调用a*算法获取路径及其长度：p_goods的坐标为终点，robot：x、y是起点
    // 将长度和p_goods->money归一化加权作为权值，若大于当前权值则更新
    if (cur_weight > goods_weight) {
      cur_goods = p_goods;
      goods_weight = cur_weight;
    }
    p_goods = p_goods->next;
  }
  robot[i].target_goods = cur_goods;
  g_goodsmanager.DeleteGoods(cur_goods); //别人不能再来抢这个货物
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
    berth_weight[max_berth] -= BERTH_WEIGHT_AFTER_BOAT_CHOOSE; //权重减少
  }
  Decision decision(4, i, boat[i].pos);
  q_decision.push(decision); // 决策入队
}

/*
 * 船在泊位何时离开
 * 决策依据：
 * 1 船的容量和当前装载量
 * 2 船在泊位停留的时间 ？
 * 3 正在前往该泊位的机器人数量 ？
 */
void Boat::LeaveCond(int i) {
  // 容量达到70%就走
  if (boat[i].num > boat_capacity * 0.7) {
    Decision decision(5, i, -1);
    q_decision.push(decision);
  }
}

int main() {
  Init();
  while (Input()) {
    // --------- 准备阶段 ----------
    g_goodsmanager.FreshGoodsLists(); // 刷新货物链表
    ClearQueue(q_decision);           // 清空决策队列

    // --------- 决策阶段 ----------
    DecisionRobot();
    DecisionBoat();

    // --------- 输出阶段 ----------
    // 根据决策表输出
    puts("OK");
    fflush(stdout);
  }
  return 0;
}
