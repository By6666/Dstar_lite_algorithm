#include <time.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <queue>
#include <vector>

#include "include/Dstar_lite_algorithm.h"
#include "include/grid_input.h"

int main() {
  // clock_t start, end;
  // start = clock();
  // for (int i = 0; i < 20; ++i) {
  //   std::cout << "/*******************************************/" <<
  //   std::endl; std::cout << "/**********file——" << i + 1 << "**********/" <<
  //   std::endl; SearchOneMap(i); std::cout <<
  //   "/*******************************************/" << std::endl; std::cout
  //   << std::endl;
  // }
  // end = clock();

  // /* 输出运行的时间 */
  // printf("Spend time %.5f seconds!!\n", (float)(end - start) /
  // CLOCKS_PER_SEC); std::cout << std::endl << std::endl; PrintSumResult();

  int16_t map_num = 0;
  while (1) {
    std::cout << "Please input the map index (1~20): ";
    std::cin >> map_num;
    if (map_num < 1 || map_num > 20) {
      std::cout << "Input wrong, Please input again !!" << std::endl;
      continue;
    }
    std::cout << "/*******************************************/" << std::endl;
    std::cout << "/**********file——" << map_num << "**********/" << std::endl;
    SearchOneMap(map_num - 1);

    std::cout << "/*******************************************/" << std::endl;
    std::cout << std::endl;
    PrintSumResult();
  }

  return 0;
}
