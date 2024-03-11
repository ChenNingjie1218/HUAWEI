#ifndef ASTAR_H_
#define ASTAR_H_
#include <string.h>
#include <unistd.h>

#include <cstddef>
#include <cstdio>
#include <list>  //链表
#include <string>
#include <vector>

#include "param.h"
typedef struct _Point {
  int x, y;                      // x为行，y为列
  int F = 0, G, H;               // F=G+H;
  struct _Point* parent = NULL;  //父节点的坐标
} Point;
void clearAll();
//分配一个节点
Point* AllocPoint(int x, int y);

// 地图数据转换
void Translatedata(char (*start)[N], int* m);

//通过A*算法寻找路径
std::list<Point*> GetPath(const Point* startPoint, const Point* endPoint,
                          int flag);

//查找路径的小方法,返回一个终点，根据终点可以回溯到起点
Point* findPath(const Point* startPoint, const Point* endPoint,
                int flag);  //用static是为了只能在函数内部调用而不能单独的使用

//返回开放列表中F的最小值的点
Point* getLeastFPoint(int flag);

std::list<Point*> findAllCurPoint(Point* cur);

//获取周围的节点
std::vector<Point*> getSurroundPoint(const Point* point, int flag);

//某个点是否可达
bool isCanreach(const Point* point, const Point* target, int flag);

//是否存在某个list中，这里用作是否存在closeList/openList中
Point* isInList(const std::list<Point*>& list, const Point* point);

//获取G，H，F
int caloG(const Point* temp_start, const Point* point);
int caloH(const Point* point, const Point* end);
int caloF(const Point* point);

//清理资源
void clearAstarMaze(int flag);

/**
  @brief 接口函数
  @param map 地图数据 start_x: 起始点的横坐标，start_y: 起始点的纵坐标，end_x:
  终止点的横坐标，end_y: 终止点的纵坐标
  @return list
*/
void astar(int start_x, int start_y, int end_x, int end_y, int flag);
std::list<Point*> AstarThreads(int start_x, int start_y, int end_x, int end_y);

static std::list<Point*> openList1;   //开放列表
static std::list<Point*> closeList1;  //关闭列表
static std::list<Point*> openList2;
static std::list<Point*> closeList2;

// 地图数据转换
void Translatedata(char (*start)[N], int* m);

/**
  @brief 把路径输出到文件, 检测路径是不是对的
*/
void PutIntoFile(std::string pathname, std::list<Point*> path);
#endif