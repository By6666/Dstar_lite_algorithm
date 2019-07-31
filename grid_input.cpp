#include "include/grid_input.h"

// GrideInput::~GrideInput() { delete[] grid_; }

void GrideInput::GetOneGrid() {
  //   char file_path[kFile_Path_Size] = "";
  //   getcwd(file_path, kFile_Path_Size);  //获得当前目录的绝对路径
  //    std::cout << file_path << std::endl;

  //   char file_name[kFile_Numbers][kFile_Path_Size] = {""};

  //   for (int i = 0; i < kFile_Numbers; ++i) {
  //     sprintf(file_name[i], "%s/inputs/%d.txt\n", file_path, i + 1);
  //     printf("%s", file_name[i]);
  //   }
  //   std::ifstream read_grid;
  //   read_grid.open("inputs//10.txt", std::ios_base::in);
  //   if (!read_grid.is_open()) std::cout << "file open failed !!" <<
  //   std::endl;

  //   int crow = 0, columns = 0;
  //   read_grid >> crow >> columns;
  //   std::cout << crow << "  " << columns << std::endl;

  // first part ： grid input
  char file_name[kFile_Numbers][kFile_Path_Size] = {""};

  for (int8_t i = 0; i < kFile_Numbers; ++i) {
    sprintf(file_name[i], "inputs//%d.txt", i + 1);
    //  printf("%s\n", file_name[i]);
  }

  // file_num_:0~19   <----->  file.txt:1~20
  std::ifstream read_grid;
  read_grid.open(file_name[file_num_], std::ios_base::in);
  if (!read_grid.is_open()) std::cout << "file open failed !!" << std::endl;

  read_grid >> rows_ >> columns_;
  // std::cout << rows_ << "  " << columns_ << std::endl;

  std::string grid_buff = "";
  for (int16_t i = 0; i < rows_ + 1; ++i) {
    getline(read_grid, grid_buff);  //整行读取
    //获得起点、终点、障碍物坐标
    for (int16_t j = 0; j < grid_buff.length(); ++j) {
      if (grid_buff[j] == 's') {
        start_pos_.first = i - 1;
        start_pos_.second = j / 2;
      } else if (grid_buff[j] == 'g') {
        goal_pos_.first = i - 1;
        goal_pos_.second = j / 2;
      } else if (grid_buff[j] == 'x') {
        obstacle_list_.push_back(Points(i - 1, j / 2));
      }
    }
  }
  read_grid.close();
}

//打印一张地图
void GrideInput::PrintMap() {
  std::cout << "primal map :" << std::endl;
  for (int16_t i = 0; i < rows_; ++i) {
    for (int16_t j = 0; j < columns_; ++j) {
      if (start_pos_ == Points(i, j))
        std::cout << "s ";
      else if (goal_pos_ == Points(i, j))
        std::cout << "g ";
      else if (std::find(obstacle_list_.begin(), obstacle_list_.end(),
                         Points(i, j)) != obstacle_list_.end())
        std::cout << "x ";
      else
        std::cout << "_ ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl << std::endl;
}

// for testing
void GridInputOneMap(int16_t num) {  //获得一个文本的map
  GrideInput grid_info(num);
  grid_info.GetOneGrid();
  //输出map的行列数
  std::cout << "rows:" << grid_info.get_grid_rows()
            << " columns:" << grid_info.get_grid_columns() << std::endl;
  //输出起点与终点坐标
  std::cout << "start_pos:(" << grid_info.get_start_pos().first << ","
            << grid_info.get_start_pos().second << ")  goal_pos:("
            << grid_info.get_goal_pos().first << ","
            << grid_info.get_goal_pos().second << ")" << std::endl;
  grid_info.PrintMap();
}

void AllGridInput() {                           //输出全部的map
  for (int8_t i = 0; i < kFile_Numbers; ++i) {  //获得所有文本的map
    GridInputOneMap(i);
    std::cout << std::endl;
  }
}
