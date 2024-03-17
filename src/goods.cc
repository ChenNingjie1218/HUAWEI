#include "goods.h"

#include <iostream>

#include "robot.h"

Goods *gds[N][N] = {nullptr};
extern int id;
extern Robot robot[robot_num + 10];
GoodsManager *GoodsManager::instance_ = nullptr;
Goods::Goods() {
  this->pre = this;
  this->next = this;
}
Goods::Goods(int x, int y, int money, int birth) {
  this->x = x;
  this->y = y;
  this->money = money;
  this->birth = birth;
}

void GoodsManager::PushGoods(Goods *new_goods) {
  ++goods_num;
  if (goods_num > 15) {
    UpdateValueValve(true);
  }
  gds[new_goods->x][new_goods->y] = new_goods;
  if (head_goods->next == head_goods) {
    // 空链表
    head_goods->next = new_goods;
    head_goods->pre = new_goods;
    new_goods->next = head_goods;
    new_goods->pre = head_goods;
    // 从头节点下一个节点开始遍历寻找路径
    first_free_goods = new_goods;
  } else {
    new_goods->pre = head_goods->pre;
    head_goods->pre->next = new_goods;
    head_goods->pre = new_goods;
    new_goods->next = head_goods;
  }
};
// 删除货物
void GoodsManager::DeleteGoods(Goods *&goods, bool is_timeout) {
  --goods_num;
  if (goods_num < 15) {
    UpdateValueValve(false);
  }
  gds[goods->x][goods->y] = nullptr;
  goods->pre->next = goods->next;
  goods->next->pre = goods->pre;

#ifdef DEBUG
  if (is_timeout) {
    std::cerr << "损失货物 money: " << goods->money << std::endl;
  }
#endif

  if (is_timeout && goods->robot_id > -1) {
#ifdef DEBUG
    std::cerr << "货物失效 robot " << goods->robot_id << "失去目标"
              << std::endl;
#endif
    robot[goods->robot_id].target_goods = nullptr;
  }
  delete goods;
  if (goods == first_free_goods) {
    // first_free_goods 失效，从头节点的下一个节点开始
    first_free_goods = head_goods->next;
  }
  goods = nullptr;
}

// 刷新货物链表
void GoodsManager::FreshGoodsLists() {
  Goods *cur = head_goods->next;
  while (cur != head_goods) {
    if (id - cur->birth >= LIFETIME) {
      Goods *temp = cur->next;
      DeleteGoods(cur, true);
      cur = temp;
    } else {
      // 剪枝
      break;
    }
  }
}

// 更新价值域值
void GoodsManager::UpdateValueValve(bool is_plus) {
  if (is_plus) {
    value_valve = VALUEABLE_GOODS_VALVE;
  } else {
    value_valve = GOODS_VALUE_VALVE;
  }
}