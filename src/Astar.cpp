#include "Astar.h"

#include <string.h>
#include <unistd.h>

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <iostream>
#include <list>  //链表
#include <map>
#include <vector>
extern int translate_map[N][N];
// 地图数据转换
void Translatedata(char (*start)[N], int* m) {
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
Point* findPath(const Point* startPoint, const Point* endPoint) {
  Point* t = AllocPoint(startPoint->x, startPoint->y);
  // InsertList(flag, open_strs, t);
  openList.push_back(t);
  while (!openList.empty()) {
    // 1、获取开放表中最小的F值，最小值F可能不唯一就会造成得到的解可能不是最优解
    Point* curPoint = getLeastFPoint();
    // 2、把当前节点放到closeList中
    openList.remove(curPoint);
    closeList.push_back(curPoint);

    // 3、找到当前节点周围可到达的节点并计算F值
    std::vector<Point*> surroundPoints;
    // for (auto it = res.begin(); it != res.end(); it++) {
    surroundPoints = getSurroundPoint(curPoint);

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
        // std::cout << "进去" << std::endl;
        int tempG = caloG(curPoint, target);
        if (tempG < target->G) {  //更新
          exist->parent = curPoint;
          exist->G = tempG;
          exist->F = caloF(target);
          // std::cout << "替换" << std::endl;
        }

        delete *iter;
      }
      // }
    }  // end for循环

    surroundPoints.clear();
    Point* resPoint = isInList(openList,
                               endPoint);  //终点是否在openList上
    if (resPoint) {
      return resPoint;
    }
  }

  return NULL;
}

//返回开放列表中F的最小值的点
Point* getLeastFPoint() {
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
std::vector<Point*> getSurroundPoint(const Point* point) {
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
bool isCanreach(const Point* point, const Point* target) {
  if (target->x < 0 || target->x > (ROW - 1) || target->y < 0 ||
      target->y > (COL - 1) ||
      (translate_map[target->x][target->y] ==
       1)  //找到对应的二维数组中的位置-》障碍物
      || (translate_map[target->x][target->y] == 2) ||
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
Point* isInList(const std::list<Point*>& list, const Point* point) {
  std::list<Point*>::const_iterator itor;
  for (itor = list.begin(); itor != list.end(); itor++) {
    if ((*itor)->x == point->x && (*itor)->y == point->y) {
      return *itor;  //存在返回该节点
    }
  }
  return NULL;
}

int caloG(const Point* temp_start, const Point* point) {
  int extraG =
      (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1
          ? k_Cost1
          : k_Cost2;  //周围的点与扩散点的差值,是否为斜边
  int parentG =
      (point->parent == NULL
           ? 0
           : point->parent->G);  //如果是初始值为null，其他的点是父类的G值
  return parentG + extraG;  //返回两个量相加
}

int caloH(const Point* point, const Point* end) {
  //欧几里得求斜边
  return (int)sqrt((double)(end->x - point->x) * (double)(end->x - point->x)) +
         (double)(end->y - point->y) * (double)(end->y - point->y) * k_Cost1;
}

int caloF(const Point* point) { return point->G + point->H; }

/*清理资源*/
void clearAstarMaze() {
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

/**
  清空list
*/
void clearList(std::list<Point*> l) {
  std::list<Point*>::iterator it;
  for (it = l.begin(); it != l.end(); it++) {
    free(*it);
  }
}

std::list<Point*> astar(int start_x, int start_y, int end_x, int end_y) {
  std::list<Point*> path;
  if ((start_x == end_x) && (start_y == end_y)) {
    Point* t = AllocPoint(start_x, start_y);
    path.push_back(t);

    return path;
  }
  //设置
  Point* start = AllocPoint(start_x, start_y);
  Point* end = AllocPoint(end_x, end_y);

  //寻找路径
  path = GetPath(start, end);
  // for (std::list<Point*>::iterator it = path.begin(); it != path.end(); it++)
  // {
  //   Point* cur = *it;
  //   std::cout << "(" << cur->x << "," << cur->y << ")" << std::endl;
  // }
  // clearAstarMaze();
  return path;
}

/**
  @brief 把路径输出到文件, 检测路径是不是对的
*/
void PutIntoFile(std::string pathname, std::list<Point*>& path) {
  FILE* fp = fopen(pathname.c_str(), "w");  // 打开文件，创建文件流对象
  char m[N][N];
  if (fp != NULL) {  // 确保文件成功打开
    int num = 0;
    std::map<int, char> map_num;
    map_num[0] = '0';
    map_num[1] = '1';
    map_num[2] = '2';
    map_num[3] = '3';
    map_num[4] = '4';
    map_num[5] = '5';
    map_num[6] = '6';
    map_num[7] = '7';
    map_num[8] = '8';
    map_num[9] = '9';
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        m[i][j] = ch[i][j];
      }
    }
    std::list<Point*>::iterator iter;
    for (iter = path.begin(); iter != path.end(); iter++) {
      Point* cur = *iter;
      std::cout << "(" << cur->x << "," << cur->y << ")" << std::endl;
      num %= 10;
      if (m[cur->x][cur->y] == '.') {
        m[cur->x][cur->y] = map_num[num];
      } else {
        exit(-1);
      }
      num++;
    }
    for (int i = 1; i <= n; i++) {
      for (int j = 1; j <= n; j++) {
        fprintf(fp, "%c", m[i][j]);
        // std::cout << m[i][j];
      }
      fprintf(fp, "\n");
      // std::cout << std::endl;
    }
    fclose(fp);  // 关闭文件
  } else {
    std::cout << "无法打开文件。" << std::endl;
  }
}
