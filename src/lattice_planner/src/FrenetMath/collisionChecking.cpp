#include "collisionChecking.h"






namespace lattice_planner{


CollisionChecker::CollisionChecker(complexPath & cpxpath, 
                                    PointsObstacle & obstacles,
                                    double rhoLeft,
                                    double rhoRight,
                                    nav_msgs::Path &refline, 
                                    std::vector<CubicCoefficients> &coefficients){
    cpxpath_ = cpxpath;
    obstacles_ = obstacles;
    rhoLeft_ = rhoLeft;
    rhoRight_ = rhoRight;

    refline_ = refline;
    coefficients_ = coefficients;

    generateDisksTrajectory();

}


void CollisionChecker::generateDisksTrajectory(){
    for(auto i:cpxpath_.cpxPath_){
        VehicleDisks disks(i.catPose_, &coefficients_, refline_);
        disksTrajectory_.disksTrajectory_.push_back(disks);
    }
}

bool CollisionChecker::performChecking(){
    CartesianPoint p0;
    p0.x = 0.0;
    p0.y = 1.0;

    for (auto m:disksTrajectory_.disksTrajectory_){

        VehicleDisk BigDisk = m.getBigDisk();


        double tmpDistance = pow(BigDisk.Catpoint.x - p0.x,2) + pow(BigDisk.Catpoint.y - p0.y, 2);
        if (tmpDistance > pow(BigDisk.radius,2))

            continue;

        else{
            VehicleDisk disk1 = m.getDisk1();
            VehicleDisk disk2 = m.getDisk2();
            VehicleDisk disk3 = m.getDisk3();
            tmpDistance = pow(disk1.Catpoint.x - p0.x,2) + pow(disk1.Catpoint.y - p0.y, 2);
            if (tmpDistance < pow(disk1.radius,2)) return false;

            tmpDistance = pow(disk2.Catpoint.x - p0.x,2) + pow(disk2.Catpoint.y - p0.y, 2);
            if (tmpDistance < pow(disk2.radius,2)) return false;

            tmpDistance = pow(disk3.Catpoint.x - p0.x,2) + pow(disk3.Catpoint.y - p0.y, 2);
            if (tmpDistance < pow(disk1.radius,2)) return false;

        }
    }

    return true;
}
    
}