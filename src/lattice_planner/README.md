# latticePlanner
author:ming.ustb@outlook.com

## Requirements
   eigen3

sudo apt-get install libeigen3-dev

sudo cp -r /usr/local/include/eigen3/Eigen /usr/local/include 

or 

sudo cp -r /usr/include/eigen3/Eigen /usr/include 
## algorithms
1 calculate kappa

2 calculate heading

3 sampling in the frenet coordinate system

4 Dijkstra's algorithm

5 cubic polynomials in the frenet coordinate system

## how to run

you should change the coefficients.csv's path.

the current path is "/home/ming/coefficients_test.csv"

roslaunch reference_line refLine.launch 

roslaunch lattice_planner lattice_planner.launch 


