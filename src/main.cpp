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
  int transport_time;
  int loading_speed;
  Berth() {}
  Berth(int x, int y, int transport_time, int loading_speed) {
    this->x = x;
    this->y = y;
    this->transport_time = transport_time;
    this->loading_speed = loading_speed;
  }
} berth[berth_num + 10];

struct Boat {
  int num, pos, status;
} boat[10];

// 货物
struct Goods {
  Goods(int x, int y, int money, int birth) {
    this->x = x;
    this->y = y;
    this->money = money;
    this->birth = birth;
  }
  int birth;
  int money;
  int x;
  int y;
  Goods* pre;
  Goods* next;
} * head_goods;

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
      PushGoods(new_goods);
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

// 更新货物

int main() {
  Init();
  OutputController output_controller;
  while (Input()) {
    // 决策

    // 输出
    puts("OK");
    fflush(stdout);
  }
  return 0;
}
