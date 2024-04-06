#include "goods.h"

#include <iostream>

#include "map_controller.h"
#include "rent_controller.h"
extern int id;
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
  // #ifdef GOODS_FILTER
  ++goods_num;
  if (goods_num > DynamicParam::GetInstance()->GetGoodsFilterValveNum()) {
    UpdateValueValve(true);
  }
  // #endif
  MapController::GetInstance()->gds[new_goods->x][new_goods->y] = new_goods;
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

// 移除过期货物
void GoodsManager::RemoveExpiredGoods(int &x, int &y) {
  Goods *cur = head_goods->next;
  while (cur != head_goods) {
    if (cur->x == x && cur->y == y) {
      DeleteGoods(cur, true);
      return;
    }
    cur = cur->next;
  }
}
// 删除货物
void GoodsManager::DeleteGoods(Goods *&goods, bool is_timeout) {
  // #ifdef GOODS_FILTER
  --goods_num;
  if (goods_num < DynamicParam::GetInstance()->GetGoodsFilterValveNum()) {
    UpdateValueValve(false);
  }
  // #endif
  MapController::GetInstance()->gds[goods->x][goods->y] = nullptr;
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
    RentController::GetInstance()->robot[goods->robot_id].target_goods =
        nullptr;
  }
  delete goods;
  if (goods == first_free_goods) {
    // first_free_goods 失效，从头节点的下一个节点开始
    first_free_goods = head_goods->next;
  }
  goods = nullptr;

#ifdef DEBUG
  if (first_free_goods->next == head_goods) {
    std::cerr << "货物链表空了" << std::endl;
  }
#endif
}

// 更新价值域值
void GoodsManager::UpdateValueValve(bool is_plus) {
  if (is_plus) {
    value_valve = DynamicParam::GetInstance()->GetValueableGoodsValve();
  } else {
    value_valve = DynamicParam::GetInstance()->GetGoodsValueValve();
  }
}

// 重置first_free_goods
void GoodsManager::ResetFirstFreeGoods() {
  first_free_goods = head_goods->next;
  while (first_free_goods->next != head_goods &&
         first_free_goods->robot_id > -1) {
    first_free_goods = first_free_goods->next;
  }
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