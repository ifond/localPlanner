/**
 * ==================================================================
 * 1
 * 对于碰撞检查，其中一种方式是基于代价地图，车辆的路径是栅格的，障碍物也是栅格的，
 * 这样对整个路径的栅格与地图中的栅格进行简单的运算即可作出快速地碰撞检查。
 * -------------------------
 * 2
 * 对于不依赖代价地图的碰撞检查，
 * 库：FCL，用于moveit
 * 库：apollo
 * 
 * 对于本文，使用三层圆进行检查，算法如下：
 * 
 * if(外接圆无碰撞) return no collision
 * else {
 *      if(内接圆有碰撞) return collision
 *      else{
 *          计算包络圆是否碰撞
 *          if(碰撞) return collison;
 *          else return nocollison;
 *      }
 *  
 * 
 * }
 *     
 * 对于碰撞的具体计算方式，本文使用
 * 
 */




#ifndef LATTICE_PLANNER_COLLISIONCHECKING_H
#define LATTICE_PLANNER_COLLISIONCHECKING_H

namespace lattice_planner{




}

#endif