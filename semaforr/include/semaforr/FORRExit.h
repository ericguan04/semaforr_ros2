/************************************************
FORRExit.h 
This file contains the class FORRExit which is intended to be a data structure which abstracts the notion of an exit

Written by Anoop Aroor, 2014
**********************************************/

#ifndef FORREXIT_H
#define FORREXIT_H

#include <iostream>
#include <FORRGeometry.h>
#include <FORRRegion.h>

class FORRExit{
 public:
  FORRExit(){};
  FORRExit(CartesianPoint point, CartesianPoint mid_point, CartesianPoint region_point, int region_id, double distance_between, int connection_path, vector<CartesianPoint> connection_points){
    exitPoint = point;
    middlePoint = mid_point;
    exitRegionPoint = region_point;
    exitRegion = region_id;
    exitDistance = distance_between;
    connectionPath = connection_path;
    connectionPoints = connection_points;
  }

  void print(){
    cout << exitPoint.get_x() << " " << exitPoint.get_y() << " " << exitRegion << endl;
  }
    
  double distance(CartesianPoint point1, CartesianPoint point2){
    double dy = point1.get_y() - point2.get_y();
    double dx = point1.get_x() - point2.get_x();
    return sqrt((dx*dx) + (dy*dy));
  }

  CartesianPoint getExitPoint(){ return exitPoint;}
  void setExitPoint(CartesianPoint point) { exitPoint = point;}

  CartesianPoint getMidPoint(){ return middlePoint;}
  void setMidPoint(CartesianPoint point) { middlePoint = point;}

  CartesianPoint getExitRegionPoint(){ return exitRegionPoint;}
  void setExitRegionPoint(CartesianPoint point){ exitRegionPoint = point;}

  void setExitRegion(int region_id) { exitRegion = region_id;}
  int getExitRegion(){ return exitRegion; }

  void setExitDistance(double dist){ exitDistance = dist;}
  double getExitDistance(){ return exitDistance; }

  void setConnectionPath(int cp){ connectionPath = cp;}
  int getConnectionPath(){ return connectionPath; }

  void setConnectionPoints(vector<CartesianPoint> cp){ connectionPoints = cp;}
  vector<CartesianPoint> getConnectionPoints(){ return connectionPoints; }

  bool operator < (const FORRExit &exit) const{
    return false;
  }

 private:
  CartesianPoint exitPoint;
  CartesianPoint middlePoint;
  CartesianPoint exitRegionPoint;
  int exitRegion;
  double exitDistance;
  int connectionPath;
  vector<CartesianPoint> connectionPoints;
};


#endif
