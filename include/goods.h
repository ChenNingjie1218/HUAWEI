#ifndef GOODS_H_
#define GOODS_H_
#include "param.h"
// 货物
struct Goods {
  Goods();
  Goods(int x, int y, int money, int birth);
  int birth;  // 生成帧
  int money;  // 价值
  int x;
  int y;
  Goods *pre;  // 双向链表连接货物
  Goods *next;  // 按生存周期排列的，具有队列性质，又可随机删除
  int robot_id = -1;  // 选择该货物的机器人
};

// 货物管理器
struct GoodsManager {
 private:
  GoodsManager() = default;
  GoodsManager(const GoodsManager &other) = default;
  static GoodsManager *instance_;

 public:
  static GoodsManager *&GetInstance() {
    if (!instance_) {
      instance_ = new GoodsManager();
    }
    return instance_;
  }
  /*
   * 将货物放入链表
   */
  void PushGoods(Goods *new_goods);

  // 删除货物
  void DeleteGoods(Goods *&goods, bool is_timeout = false);

  // 刷新货物链表
  void FreshGoodsLists();

  // 货物链表头节点
  Goods *head_goods = new Goods();

  // 货物链表起始节点
  // 每次找路径从该节点开始
  Goods *first_free_goods = head_goods;
};
#endif