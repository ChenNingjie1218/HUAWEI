#include <bits/stdc++.h>

#include <cstdio>
#include <string>

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
int id;  // 帧号
char ch[N][N];
int gds[N][N];  // 暂时不知道干嘛的？

// 机器人
struct Robot {
  int x, y, goods;
  int status;
  int mbx, mby;  //什么意思？
  Robot() {}
  Robot(int startX, int startY) {
    x = startX;
    y = startY;
  }
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
} boat[10];

// 货物
struct Goods {
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
  Goods* pre;  // 双向链表连接货物
  Goods* next;  // 按生存周期排列的，具有队列性质，又可随机删除
};

// 货物管理器
struct GoodsManager {
  Goods* head_goods;
  /*
   * 将货物放入链表
   */
  void PushGoods(Goods*& new_goods) {
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
struct decision {
  /*
   * - 0 机器人
   * - 1 船
   */
  int type;

  // 机器人或者船的id
  int id;

  // 第二参数
  int param;
};

// 决策队列
queue<decision> q_decision;
// 清空决策队列
void ClearQueue(queue<decision>& q) {
  queue<decision> empty;
  swap(empty, q);
}
// 初始化
void Init() {
  // 地图数据
  for (int i = 1; i <= n; i++) scanf("%s", ch[i] + 1);
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
    for (int i = 0; i < 5; i++) scanf("%d%d\n", &boat[i].status, &boat[i].pos);

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

    // --------- 移动 ---------
  }

  // --------- 移动后动作 ---------
}
/*
 * 船做决策
 * 根据帧数据状态来决策
 */
void DecisionBoat() {
  for (int i = 0; i < 5; ++i) {
    // status 0 运输中 无需考虑决策
    if (boat[i].status == 1) {
      if (boat[i].pos == -1) {
        // 在虚拟点
        // 决策去哪个泊位
      } else {
        // 决策是否驶离
      }
    } else if (boat[i].status == 2) {
      // 在等待
      // 可以决策是否换船舶，换哪个船舶
    }
  }
}

int main() {
  Init();
  while (Input()) {
    // --------- 准备阶段 ----------
    g_goodsmanager.FreshGoodsLists();  // 刷新货物链表
    ClearQueue(q_decision);            // 清空决策队列

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
