# Dstar_lite algorithm

* Header file in the folder [include](./include).
 
* testing maps int the folder [inputs](./inputs).

* Code of reading map in the file [grid_input.cpp](./grid_input.cpp).

* Algorithm code file [Dstar_lite_algorithm.cpp](./Dstar_lite_algorithm.cpp).


  Dstar_lite buliding on LPAstar, that is a dynamic version of LPAstar. In this algorithm, we  need to switch the search direction of so that the g-values are estimates of the goal distances. We use variable Km to maintain lower bounds of priority queue.

