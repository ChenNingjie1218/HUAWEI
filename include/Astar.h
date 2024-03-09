#ifndef ASTAR_H_
#define ASTAR_H_
#include <string.h>

#include <cmath>
#include <iostream>
#include <list>  //链表
#include <vector>
// extern int k_Cost1;
// extern int k_Cost2;
const int k_Cost1 = 10;  //走一格消耗10
const int k_Cost2 = 14;  //斜移走一个消耗14
#define COL 210
#define ROW 210

typedef struct _Point {
  int x, y;               // x为行，y为列
  int F, G, H;            // F=G+H;
  struct _Point* parent;  //父节点的坐标
} Point;

//分配一个节点
Point* AllocPoint(int x, int y);

// 地图数据转换
void Translatedata(char (*start)[COL], int* m);

//初始化地图，地图，行，列
void InitAstarMaze(int* _maze, int _line, int _colums);

//通过A*算法寻找路径
std::list<Point*> GetPath(const Point* startPoint, const Point* endPoint);

//查找路径的小方法,返回一个终点，根据终点可以回溯到起点
static Point* findPath(
    const Point* startPoint,
    const Point* endPoint);  //用static是为了只能在函数内部调用而不能单独的使用

//返回开放列表中F的最小值的点
static Point* getLeastFPoint();

//获取周围的节点
static std::vector<Point*> getSurroundPoint(const Point* point);

//某个点是否可达
static bool isCanreach(const Point* point, const Point* target);

//是否存在某个list中，这里用作是否存在closeList/openList中
static Point* isInList(const std::list<Point*>& list, const Point* point);

//获取G，H，F
static int caloG(const Point* temp_start, const Point* point);
static int caloH(const Point* point, const Point* end);
static int caloF(const Point* point);

//清理资源
void clearAstarMaze();

/**
  @brief 接口函数
  @param map 地图数据 start_x: 起始点的横坐标，start_y: 起始点的纵坐标，end_x:
  终止点的横坐标，end_y: 终止点的纵坐标
  @return list
*/
std::list<Point*> astar(char (*map)[COL], int start_x, int start_y, int end_x,
                        int end_y);

static int* maze;  //初始化迷宫
static int cols;   //二维数组对应的列
static int lines;  //二维数组对应的行

static std::list<Point*> openList;   //开放列表
static std::list<Point*> closeList;  //关闭列表

// 地图数据转换
void Translatedata(char (*start)[COL], int* m) {
  // std::cout << "123" << std::endl;
  // std::cout << "123" << std::endl;
  for (int i = 0; i < ROW; i++) {
    // std::cout << "123" << std::endl;
    for (int j = 0; j < COL; j++) {
      // std::cout << col << std::endl;
      if (start[i][j] == '.') {
        // std::cout << row << std::endl;
        m[i * COL + j] = 0;
      } else {
        m[i * COL + j] = 1;
      }
      // std::cout << "sdf" << std::endl;
    }
  }
  return;
}

/*初始化地图*/
void InitAstarMaze(int* _maze, int _line, int colums) {  //一级指针保存二维数组
  maze = _maze;
  lines = _line;
  cols = colums;
}

/*分配节点*/
Point* AllocPoint(int x, int y) {
  Point* temp = new Point;
  memset(temp, 0, sizeof(Point));  //清理养成好习惯

  temp->x = x;
  temp->y = y;
  return temp;
}

/*通过A*算法寻找路径*/
std::list<Point*> GetPath(const Point* startPoint, const Point* endPoint) {
  Point* result = findPath(startPoint, endPoint);
  std::list<Point*> path;

  //返回路径
  while (result) {
    path.push_front(result);  //这样起点就是在这个链表的最前面了
    result = result->parent;
  }

  return path;
}

/*查找路径的小方法,返回一个终点，根据终点可以回溯到起点*/
static Point* findPath(const Point* startPoint, const Point* endPoint) {
  openList.push_back(
      AllocPoint(startPoint->x, startPoint->y));  //重新分配更加的安全，置入起点

  while (!openList.empty()) {
    // 1、获取开放表中最小的F值
    Point* curPoint = getLeastFPoint();

    // 2、把当前节点放到closeList中
    openList.remove(curPoint);
    closeList.push_back(curPoint);

    // 3、找到当前节点周围可到达的节点并计算F值
    std::vector<Point*> surroundPoints = getSurroundPoint(curPoint);

    std::vector<Point*>::const_iterator iter;
    for (iter = surroundPoints.begin(); iter != surroundPoints.end(); iter++) {
      Point* target = *iter;

      //如果没在开放列表中就加入到开放列表，设置当前节点为父节点
      Point* exist = isInList(openList, target);
      if (!exist) {
        target->parent = curPoint;
        target->G =
            caloG(curPoint, target);  //父节点的G加上某个数就好（自己设计的）
        target->H = caloH(target, endPoint);
        target->F = caloF(target);

        openList.push_back(target);
      } else {  //如果已存在就重新计算G值看要不要替代
        int tempG = caloG(curPoint, target);
        if (tempG < target->G) {  //更新
          exist->parent = curPoint;
          exist->G = tempG;
          exist->F = caloF(target);
        }

        delete *iter;
      }

    }  // end for循环

    surroundPoints.clear();
    Point* resPoint = isInList(openList, endPoint);  //终点是否在openList上
    if (resPoint) {
      return resPoint;
    }
  }

  return NULL;
}

//返回开放列表中F的最小值的点
static Point* getLeastFPoint() {
  //遍历
  if (!openList.empty()) {
    Point* resPoint = openList.front();

    std::list<Point*>::const_iterator itor;  //定义迭代器，用于遍历链表

    //迭代器遍历，C++特性,直接理解成平时我们用的for
    for (itor = openList.begin(); itor != openList.end(); itor++) {
      Point* p = *itor;  //把元素拿出来
      if (p->F < resPoint->F) {
        resPoint = p;
      }
    }
    return resPoint;
  }
  return NULL;
}

/*获取周围的节点*/
static std::vector<Point*> getSurroundPoint(const Point* point) {
  std::vector<Point*> surroundPoints;
  //周围九个点都会进行检测
  for (int x = point->x - 1; x <= point->x + 1; x++) {
    for (int y = point->y - 1; y <= point->y + 1; y++) {
      Point* temp = AllocPoint(x, y);
      if (isCanreach(point, temp)) {
        surroundPoints.push_back(temp);
      } else {
        delete temp;
      }
    }
  }
  return surroundPoints;
}

/*某个点是否可达*/
static bool isCanreach(const Point* point, const Point* target) {
  if (target->x < 0 || target->x > (lines - 1) || target->y < 0 ||
      target->y > (cols - 1) ||
      (maze[target->x * cols + target->y] ==
       1)  //找到对应的二维数组中的位置-》障碍物
      || (maze[target->x * cols + target->y] == 2) ||
      (target->x == point->x && target->y == point->y) ||
      isInList(closeList, target)) {
    return false;
  }

  if (abs(point->x - target->x) + abs(point->y - target->y) ==
      1) {  //我们现在只考虑上下左右4个点
    return true;
  } else {
    return false;
  }
}

/*是否存在某个list中，这里用作是否存在closeList/openList中*/
static Point* isInList(const std::list<Point*>& list, const Point* point) {
  std::list<Point*>::const_iterator itor;
  for (itor = list.begin(); itor != list.end(); itor++) {
    if ((*itor)->x == point->x && (*itor)->y == point->y) {
      return *itor;  //存在返回该节点
    }
  }
  return NULL;
}

static int caloG(const Point* temp_start, const Point* point) {
  int extraG =
      (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1
          ? k_Cost1
          : k_Cost2;  //周围的点与扩散点的差值,是否为斜边
  int parentG =
      (point->parent == NULL
           ? NULL
           : point->parent->G);  //如果是初始值为null，其他的点是父类的G值
  return parentG + extraG;  //返回两个量相加
}

static int caloH(const Point* point, const Point* end) {
  //欧几里得求斜边
  return (int)sqrt((double)(end->x - point->x) * (double)(end->x - point->x)) +
         (double)(end->y - point->y) * (double)(end->y - point->y) * k_Cost1;
}

static int caloF(const Point* point) { return point->G + point->H; }

/*清理资源*/
void clearAstarMaze() {
  maze = NULL;
  lines = 0;
  cols = 0;
  std::list<Point*>::iterator itor;
  for (itor = openList.begin(); itor != openList.end();) {
    delete *itor;
    itor = openList.erase(itor);  //获取到下一个节点
  }
  for (itor = closeList.begin(); itor != closeList.end();) {
    delete *itor;
    itor = closeList.erase(itor);  //获取到下一个节点
  }
}

std::list<Point*> astar(char (*map)[COL], int start_x, int start_y, int end_x,
                        int end_y) {
  int m[ROW][COL];

  Translatedata(map, &m[0][0]);

  InitAstarMaze(&m[0][0], ROW, COL);
  //设置
  Point* start = AllocPoint(start_x, start_y);
  Point* end = AllocPoint(end_x, end_y);

  //寻找路径
  std::list<Point*> path = GetPath(start, end);

  //设置迭代器遍历
  // std::list<Point*>::const_iterator iter;  //迭代器

  // cout << "(" << start->x << "," << start->y << ")"
  //      << "------>(" << end->x << "," << end->y << ")" << endl;
  // cout << "****************** 寻路结果 ********************************"
  //      << endl;
  //  int num = 0;
  // std::map<int, char> map_num;
  // map_num[0] = '0';
  // map_num[1] = '1';
  // map_num[2] = '2';
  // map_num[3] = '3';
  // map_num[4] = '4';
  // map_num[5] = '5';
  // map_num[6] = '6';
  // map_num[7] = '7';
  // map_num[8] = '8';
  // map_num[9] = '9';
  // for (iter = path.begin(); iter != path.end();) {
  //   Point *cur = *iter;
  //   cout << '(' << cur->x << "," << cur->y << ')' << endl;
  //   num %= 10;
  //   if (ch[cur->x][cur->y] == '.'){
  //     ch[cur->x][cur->y] = map_num[num];
  //   }else {
  //     exit(-1);
  //   }
  //   num++;
  // cur;//不用再进行释放了因为在openList和closeList链表中我们最后都有清理，如果再清理就会报错
  // iter = path.erase(iter);
  // sleep(1); //休眠
  // }
  // clearAstarMaze();
  return path;
}
#endif
