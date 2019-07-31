#ifndef PLANNER_GRID_INPUT_H
#define PLANNER_GRID_INPUT_H

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

typedef std::pair<int16_t, int16_t> Points;
const int8_t kFile_Path_Size = 50;  //当前目录绝对路径字符串长度
const int8_t kFile_Numbers = 20;    //需要读取的文件数目

class GrideInput {
 public:
  GrideInput(int16_t file_number) : file_num_(file_number) {}

  void GetOneGrid();  //得到一张地图
  void PrintMap();    //打印map

  inline int16_t get_grid_rows() const { return rows_; }
  inline int16_t get_grid_columns() const { return columns_; }
  inline Points get_start_pos() const { return start_pos_; }
  inline Points get_goal_pos() const { return goal_pos_; }
  inline std::vector<Points> get_obstacle_pos() const { return obstacle_list_; }

 private:
  int16_t rows_, columns_, file_num_;
  Points start_pos_, goal_pos_;
  std::vector<Points> obstacle_list_;
};

void GridInputOneMap(int16_t num);  //获得一个文本的map
void AllGridInput();            //输出全部的map

#endif