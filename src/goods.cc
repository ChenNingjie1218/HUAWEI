#include "goods.h"
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
void GoodsManager::DeleteGoods(Goods *&goods) {
  gds[goods->x][goods->y] = false;
  goods->pre->next = goods->next;
  goods->next->pre = goods->pre;
  delete goods;
  goods = nullptr;
}

// 刷新货物链表
void GoodsManager::FreshGoodsLists() {
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