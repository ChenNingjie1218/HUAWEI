#include "floyd.h"

#include <cmath>
#include <iostream>
#include <stack>
extern char ch[N][N];
bool PointCanMove(int i, int j) {
  if (i < 1 || i > n || j < 1 || j > n) {
    return false;
  }
  if (ch[i][j] == '.' || ch[i][j] == 'B' || ch[i][j] == 'A') {
    return true;
  }
  return false;
}

Floyd::Floyd() {
  const int SIZE = 40010;
  dist = new int*[SIZE];
  path = new int*[SIZE];
  for (int i = 0; i < SIZE; ++i) {
    dist[i] = new int[SIZE];
    path[i] = new int[SIZE];
  }
}

Floyd::~Floyd() {
  const int SIZE = 40010;
  for (int i = 0; i < SIZE; ++i) {
    delete[] dist[i];
    delete[] path[i];
  }
  delete[] dist;
  delete[] path;
}
// 初始化dist及path
void Floyd::Init() {
  const int MAX_DIST = 100000;
  int i_x, i_y;
  int j_x, j_y;
  for (int i = 1; i <= 40000; ++i) {
    for (int j = 1; j <= 40000; ++j) {
      if (i == j) {
        dist[i][j] = 0;
      } else if (((i_y = i % n) - 1) == (j_y = j % n) || i_y + 1 == j_y ||
                 ((i_x = ceil(1.0 * i / n)) - 1) == (j_x = ceil(1.0 * j / n)) ||
                 i_x + 1 == j_x) {
        // j是i周围的点
        if (PointCanMove(i_x, i_y) && PointCanMove(j_x, j_y)) {
          dist[i][j] = 1;
        } else {
          dist[i][j] = MAX_DIST;
        }
      } else {
        dist[i][j] = MAX_DIST;
      }
      path[i][j] = -1;
    }
  }
}
// 计算最短dist及path
void Floyd::Calc() {
  int point_num = n * n;
  for (int i = 1; i <= point_num; ++i) {
    for (int j = 1; j <= point_num; ++j) {
      for (int k = 1; k <= point_num; ++k) {
        if (dist[i][j] >= dist[i][k] + dist[k][j]) {
          dist[i][j] = dist[i][k] + dist[k][j];
          path[i][j] = k;
        }
      }
    }
  }
}

// 计算路径长度
int Floyd::GetPathLength(int start_x, int start_y, int end_x, int end_y) {
  return dist[(start_x - 1) * n + start_y][(end_x - 1) * n + end_y];
}

// 输出路径
void Floyd::OutputPath(int start_x, int start_y, int end_x, int end_y) {
  int start_id = (start_x - 1) * n + start_y;
  int end_id = (start_y - 1) * n + start_y;
  int pass_id = end_id;
  std::stack<int> st;
  while (pass_id != start_id) {
    st.push(pass_id);
    pass_id = path[start_id][pass_id];
  }

  while (!st.empty()) {
    pass_id = st.top();
    st.pop();

    std::cout << "(" << pass_id / n + 1 << "," << pass_id % n << ")"
              << std::endl;
  }
}