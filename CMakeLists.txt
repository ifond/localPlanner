cmake_minimum_required(VERSION 3.14)
project(latticePlanner)

set(CMAKE_CXX_STANDARD 14)

add_executable(latticePlanner main.cpp curves/cubicPolynomial.cpp curves/cubicPolynomial.h pathPlanner/latticePlanner.cpp pathPlanner/latticePlanner.h pathPlanner/costFunctions.cpp pathPlanner/costFunctions.h FrenetMath/calHeading.cpp FrenetMath/calHeading.h FrenetMath/calKappa.cpp FrenetMath/calKappa.h config/parameters.cpp config/parameters.h FrenetMath/cartesianToFrenet.cpp FrenetMath/cartesianToFrenet.h FrenetMath/frenetToCartesian.cpp FrenetMath/frenetToCartesian.h referenceLine/referenceLine.cpp referenceLine/referenceLine.h referenceLine/cubicSpline.cpp referenceLine/cubicSpline.h)