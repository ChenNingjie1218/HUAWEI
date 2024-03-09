#ifndef PARAM_H_
#define PARAM_H_
const int k_Cost1 = 10;  //走一格消耗10
const int k_Cost2 = 14;  //斜移走一个消耗14
#define COL 210
#define ROW 210
// 货物生存周期
const int LIFETIME = 20;

// 地图长度
const int n = 200;
const int robot_num = 10;
const int berth_num = 10;
const int N = 210;

int money, boat_capacity;
int id;  // 帧号


#define BERTH_WEIGHT_AFTER_BOAT_CHOOSE 1  //船选择泊位后，泊位权重的减少

/*
 * - · 空地
 * - * 海洋
 * - # 障碍
 * - A 机器人起始位置，总共10个
 * - B 大小为4*4，标识泊位的位置
 */
char ch[N][N];
bool gds[N][N] = {false};  // 该点是否有货物

// 港口权重
int berth_weight[10];
#endif
