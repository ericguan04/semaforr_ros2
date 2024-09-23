/************************************************
FORRRegion.h 
This file contains the class FORRRegion which is intended to be a data structure which abstracts the notion of 
open spaces and doors in the maps as regions

Written by Anoop Aroor, 2014
**********************************************/

#ifndef FORRREGION_H
#define FORRREGION_H

#include <iostream>
#include "FORRGeometry.h"
#include "FORRExit.h"
#include <algorithm>
#include <vector>

class FORRRegion{
 public:
  FORRRegion(){};
  FORRRegion(CartesianPoint point, vector <CartesianPoint> lep, double r){
    // cout << "creating region with laser" << endl;
    center = point;
    // lasers.push_back(lep);
    for(int i = 0; i < 360; i++){
      min_visibility.push_back(-1.0);
      max_visibility.push_back(-1.0);
      avg_visibility.push_back(0.0);
      count_visibility.push_back(0.0);
      start_max_visibility.push_back(CartesianPoint());
    }
    this->adjustVisibility(point, lep);
    this->setRadius(r);
    // passage_value = -1;
  }
  FORRRegion(CartesianPoint point, double r){
    // cout << "creating region without laser" << endl;
    center = point;
    // lasers.push_back(lep);
    for(int i = 0; i < 360; i++){
      min_visibility.push_back(r);
      max_visibility.push_back(r);
      avg_visibility.push_back(r);
      count_visibility.push_back(1);
      start_max_visibility.push_back(center);
    }
    this->setRadius(r);
    // passage_value = -1;
  }

  bool doIntersect(FORRRegion test){
    int buffer = 0;
    if((test.getRadius() + this->getRadius() - distance(test.getCenter(), center)) > buffer)
      return true;
    else 
      return false;
  }

  bool operator==(FORRRegion b){
    bool equal = false;
    if(radius == b.getRadius() && center.get_x() == b.getCenter().get_x() && center.get_y() == b.getCenter().get_y()){
      equal = true;
    }
    return equal;
  }
  
  void clearExits(){
    exits.clear();
    ext_exits.clear();
    min_exits.clear();
  }

  void print(){
    cout << center.get_x() << " " << center.get_y() << " " << this->getRadius();
    for(int i = 0; i < exits.size() ; i++){
      cout << " " << exits[i].getExitPoint().get_x() << " "  << exits[i].getExitPoint().get_y() << " "  << exits[i].getExitRegion();
    }
    cout << endl;
  }

  void printVisibility(){
    cout << center.get_x() << " " << center.get_y() << " " << this->getRadius();
    for(int i = 0; i < max_visibility.size(); i++){
      cout << " " << max_visibility[i];
    }
    cout << endl;
  }

  bool equals(FORRRegion test) { 
    if(test.getCenter().get_x() == center.get_x() && test.getCenter().get_y() == center.get_y() && test.getRadius() == this->getRadius())
      return true;
    else 
      return false;
  }

  bool isExitAlreadyPresent(FORRExit exit){
    for(int i = 0; i < exits.size() ; i++){
      if(exit.getExitPoint().get_x() == exits[i].getExitPoint().get_x() && exit.getExitPoint().get_y() == exits[i].getExitPoint().get_y() && exit.getExitRegion() == exits[i].getExitRegion() && exit.getExitDistance() == exits[i].getExitDistance())
        return true;
    }
    return false;
  }

  void addMinDistanceExit(FORRExit exit){
    // cout << "Inside addMinDistanceExit " << min_exits.size() << endl;
    bool foundAny = false;
    for(int i = 0; i < min_exits.size() ; i++){
      // cout << "Exit region " << exit.getExitRegion() << " min_exit region " << min_exits[i].getExitRegion() << endl;
      if(exit.getExitRegion() == min_exits[i].getExitRegion()){
        foundAny = true;
        // cout << "Exit distance " << exit.getExitDistance() << " min_exit distance " << min_exits[i].getExitDistance() << endl;
        if(exit.getExitDistance() < min_exits[i].getExitDistance()){
          min_exits.erase(min_exits.begin()+i);
          min_exits.push_back(exit);
        }
        break;
      }
    }
    if(foundAny == false){
      // cout << "Exit not found, add new" << endl;
      min_exits.push_back(exit);
    }
    // cout << "End addMinDistanceExit " << min_exits.size() << endl;
  }

  bool inRegion(double x, double y){ return (distance(CartesianPoint(x,y),center) <= this->getRadius());}

  bool inRegion(CartesianPoint p){ return (distance(p,center) <= this->getRadius());}
    
  double distance(CartesianPoint point1, CartesianPoint point2){
    double dy = point1.get_y() - point2.get_y();
    double dx = point1.get_x() - point2.get_x();
    return sqrt((dx*dx) + (dy*dy));
  }

  CartesianPoint getCenter(){ return center;}
  void setCenter(CartesianPoint point) { center = point;}
  
  double getRadius(){ 
    return radius;
  }
  void setRadius(double r){ 
    radius = r;
  }

  vector<int> getPassageValues(){
    return passage_values;
  }
  void setPassageValue(int pv){
    passage_values.push_back(pv);
  }
  void resetPassageValues(){
    passage_values.clear();
  }

  void adjustVisibility(CartesianPoint point, vector <CartesianPoint> lep){
    // cout << "Inside Adjust Visibility " << point.get_x() << " " << point.get_y() << " " << lep.size() << endl;
    // cout << "center " << center.get_x() << " " << center.get_y() << endl;
    for(int i = 0; i < lep.size(); i++){
      // cout << lep[i].get_x() << " " << lep[i].get_y() << endl;
      double laser_direction = atan2((lep[i].get_y() - center.get_y()), (lep[i].get_x() - center.get_x()));
      double degrees = laser_direction * (180.0/3.141592653589793238463);
      // cout << "laser_direction " << laser_direction << " degrees " << degrees << endl;
      if(degrees < 0) degrees = degrees + 360;
      double dist_to_center = lep[i].get_distance(center);
      if(dist_to_center > 25.0) dist_to_center = 25.0;
      // cout << "dist_to_center " << dist_to_center << " degrees " << degrees << " (int)(degrees) " << (int)(degrees) << endl;
      // cout << "laser_direction " << laser_direction << " degrees " << degrees << " distance " << dist_to_center << endl;
      // cout << "min_visibility " << min_visibility[(int)(degrees)] << " max_visibility " << max_visibility[(int)(degrees)] << " avg_visibility " << avg_visibility[(int)(degrees)] << " count_visibility " << count_visibility[(int)(degrees)] << endl;
      if(min_visibility[(int)(degrees)] == -1.0 or min_visibility[(int)(degrees)] > dist_to_center){
        min_visibility[(int)(degrees)] = dist_to_center;
      }
      if(max_visibility[(int)(degrees)] == -1.0 or max_visibility[(int)(degrees)] < dist_to_center){
        max_visibility[(int)(degrees)] = dist_to_center;
        start_max_visibility[(int)(degrees)] = point;
      }
      avg_visibility[(int)(degrees)] = ((avg_visibility[(int)(degrees)] * count_visibility[(int)(degrees)]) + dist_to_center) / (count_visibility[(int)(degrees)] + 1);
      count_visibility[(int)(degrees)] ++;
      // cout << "min_visibility " << min_visibility[(int)(degrees)] << " max_visibility " << max_visibility[(int)(degrees)] << " avg_visibility " << avg_visibility[(int)(degrees)] << " count_visibility " << count_visibility[(int)(degrees)] << endl;
    }
    // cout << "Current visibility: ";
    // for(int i = 0; i < visibility.size(); i++){
    //   cout << visibility[i] << " ";
    // }
    // cout << endl;
  }

  void mergeVisibility(FORRRegion rg){
    // cout << "Inside Merge Visibility" << endl;
    // cout << "current region " << center.get_x() << " " << center.get_y() << " " << radius << endl;
    // cout << "merger region " << rg.getCenter().get_x() << " " << rg.getCenter().get_y() << " " << rg.getRadius() << endl;
    vector<double> vis = rg.getMaxVisibility();
    vector<CartesianPoint> visStart = rg.getStartMaxVisibility();
    for(int i = 0; i < vis.size(); i++){
      double dist = vis[i];
      double angle;
      if(i <= 180)
        angle = double(i) / (180.0/3.141592653589793238463);
      else
        angle = double(i - 360) / (180.0/3.141592653589793238463);
      // cout << "dist " << dist << " i " << i << " angle " << angle << endl;
      CartesianPoint end_point = CartesianPoint(rg.getCenter().get_x() + dist*cos(angle), rg.getCenter().get_y() + dist*sin(angle));
      // cout << "end_point " << end_point.get_x() << " " << end_point.get_y() << endl;
      double laser_direction = atan2((end_point.get_y() - center.get_y()), (end_point.get_x() - center.get_x()));
      double degrees = laser_direction * (180.0/3.141592653589793238463);
      if(degrees < 0) degrees = degrees + 360;
      double dist_to_center = end_point.get_distance(center);
      if(dist_to_center > 25.0) dist_to_center = 25.0;
      // cout << "laser_direction " << laser_direction << " degrees " << degrees << " distance " << dist_to_center << endl;
      // cout << "max_visibility " << max_visibility[(int)(degrees)] << endl;
      if(max_visibility[(int)(degrees)] == -1.0 or max_visibility[(int)(degrees)] < dist_to_center){
        max_visibility[(int)(degrees)] = dist_to_center;
        start_max_visibility[(int)(degrees)] = visStart[i];
      }
      // cout << "max_visibility " << max_visibility[(int)(degrees)] << endl;
    }
    vis = rg.getMinVisibility();
    for(int i = 0; i < vis.size(); i++){
      double dist = vis[i];
      double angle;
      if(i <= 180)
        angle = double(i) / (180.0/3.141592653589793238463);
      else
        angle = double(i - 360) / (180.0/3.141592653589793238463);
      CartesianPoint end_point = CartesianPoint(rg.getCenter().get_x() + dist*cos(angle), rg.getCenter().get_y() + dist*sin(angle));

      double laser_direction = atan2((end_point.get_y() - center.get_y()), (end_point.get_x() - center.get_x()));
      double degrees = laser_direction * (180.0/3.141592653589793238463);
      if(degrees < 0) degrees = degrees + 360;
      double dist_to_center = end_point.get_distance(center);
      if(dist_to_center > 25.0) dist_to_center = 25.0;
      // cout << "laser_direction " << laser_direction << " degrees " << degrees << " distance " << dist_to_center << endl;
      if(min_visibility[(int)(degrees)] == -1.0 or min_visibility[(int)(degrees)] > dist_to_center){
        min_visibility[(int)(degrees)] = dist_to_center;
      }
    }
    vis = rg.getAvgVisibility();
    for(int i = 0; i < vis.size(); i++){
      double dist = vis[i];
      double angle;
      if(i <= 180)
        angle = double(i) / (180.0/3.141592653589793238463);
      else
        angle = double(i - 360) / (180.0/3.141592653589793238463);
      CartesianPoint end_point = CartesianPoint(rg.getCenter().get_x() + dist*cos(angle), rg.getCenter().get_y() + dist*sin(angle));

      double laser_direction = atan2((end_point.get_y() - center.get_y()), (end_point.get_x() - center.get_x()));
      double degrees = laser_direction * (180.0/3.141592653589793238463);
      if(degrees < 0) degrees = degrees + 360;
      double dist_to_center = end_point.get_distance(center);
      if(dist_to_center > 25.0) dist_to_center = 25.0;
      // cout << "laser_direction " << laser_direction << " degrees " << degrees << " distance " << dist_to_center << endl;
      avg_visibility[(int)(degrees)] = ((avg_visibility[(int)(degrees)] * count_visibility[(int)(degrees)]) + dist_to_center) / (count_visibility[(int)(degrees)] + 1);
      count_visibility[(int)(degrees)] ++;
    }
    // cout << "Current visibility: ";
    // for(int i = 0; i < visibility.size(); i++){
    //   cout << visibility[i] << " ";
    // }
    // cout << endl;
  }

  // void addLaser(vector <CartesianPoint> lep){
  //   lasers.push_back(lep);
  // }

  // void addLasers(vector < vector <CartesianPoint> > leps){
  //   for(int i = 0; i < leps.size(); i++){
  //     lasers.push_back(leps[i]);
  //   }
  // }

  // vector < vector <CartesianPoint> > getLasers(){
  //   return lasers;
  // }

  vector<double> getMaxVisibility(){
    return max_visibility;
  }

  vector<double> getMinVisibility(){
    return min_visibility;
  }

  vector<double> getAvgVisibility(){
    return avg_visibility;
  }

  vector<double> getCountVisibility(){
    return count_visibility;
  }

  vector<CartesianPoint> getStartMaxVisibility(){
    return start_max_visibility;
  }

  vector<LineSegment> getVisibilityLineSegments(){
    vector<LineSegment> endPoints;
    for(int i = 0; i < max_visibility.size(); i++){
      if(max_visibility[i] == -1){
        endPoints.push_back(LineSegment(center, center));
      }
      else{
        double angle;
        if(i > 180){
          angle = (i - 360.0)/(180.0/3.141592653589793238463);
        }
        else{
          angle = (i - 0.0)/(180.0/3.141592653589793238463);
        }
        CartesianPoint end_point = CartesianPoint(center.get_x() + max_visibility[i]*cos(angle), center.get_y() + max_visibility[i]*sin(angle));
        endPoints.push_back(LineSegment(start_max_visibility[i], end_point));
      }
    }
    return endPoints;
  }


  bool visibleFromRegion(CartesianPoint point, double distanceLimit){
    // cout << "Inside Visible From Region" << endl;
    CartesianPoint laserPos = center;
    double distLaserPosToPoint = laserPos.get_distance(point);
    if(distLaserPosToPoint > distanceLimit){
      return false;
    }
    double point_direction = atan2((point.get_y() - laserPos.get_y()), (point.get_x() - laserPos.get_x()));
    double degrees = point_direction * (180.0/3.141592653589793238463);
    if(degrees < 0) degrees = degrees + 360;
    // cout << "point_direction " << point_direction << " degrees " << degrees << " distance " << distLaserPosToPoint << " current visibility " << max_visibility[(int)(degrees)] << endl;
    int index = (int)(degrees);
    while (index-2 < 0){
      index = index + 1;
    }
    while (index+2 > max_visibility.size()-1){
      index = index - 1;
    }
    int numFree = 0;
    for(int i = -2; i < 3; i++){
      if(max_visibility[index+i] >= distLaserPosToPoint){
        numFree++;
      }
    }
    if(numFree > 0){
      return true;
    }
    else{
      return false;
    }
  }

  LineSegment visibleLineSegmentFromRegion(CartesianPoint point, double distanceLimit){
    CartesianPoint laserPos = center;
    double distLaserPosToPoint = laserPos.get_distance(point);
    if(distLaserPosToPoint > distanceLimit){
      return LineSegment(point, point);
    }
    double point_direction = atan2((point.get_y() - laserPos.get_y()), (point.get_x() - laserPos.get_x()));
    double degrees = point_direction * (180.0/3.141592653589793238463);
    if(degrees < 0) degrees = degrees + 360;
    // cout << "point_direction " << point_direction << " degrees " << degrees << " distance " << distLaserPosToPoint << " current visibility " << max_visibility[(int)(degrees)] << endl;
    int index = (int)(degrees);
    while (index-2 < 0){
      index = index + 1;
    }
    while (index+2 > max_visibility.size()-1){
      index = index - 1;
    }
    int max_ind = 0;
    double max_ind_val = 0;
    for(int i = -2; i < 3; i++){
      if(max_visibility[index+i] > max_ind_val){
        max_ind_val = max_visibility[index+i];
        max_ind = index+i;
      }
    }
    double angle;
    if(max_ind > 180){
      angle = (max_ind - 360.0)/(180.0/3.141592653589793238463);
    }
    else{
      angle = (max_ind - 0.0)/(180.0/3.141592653589793238463);
    }
    CartesianPoint end_point = CartesianPoint(center.get_x() + max_visibility[max_ind]*cos(angle), center.get_y() + max_visibility[max_ind]*sin(angle));
    return LineSegment(start_max_visibility[max_ind], end_point);
  }

  vector<FORRExit> getExtExits() { return ext_exits; }

  vector<FORRExit> getExits() { return exits;}

  vector<FORRExit> getMinExits() { return min_exits;}

  void setExits(vector<FORRExit> exit_points) { exits = exit_points;}
  void addExit(FORRExit exit) {
    exits.push_back(exit);
    ext_exits.push_back(FORRExit(CartesianPoint(2*(exit.getExitPoint().get_x()) - center.get_x(), 2*(exit.getExitPoint().get_y()) - center.get_y()), exit.getMidPoint(), exit.getExitRegionPoint(), exit.getExitRegion(), exit.getExitDistance(), exit.getConnectionPath(), exit.getConnectionPoints()));
    this->addMinDistanceExit(exit);
  }

  void removeExitsToRegion(vector <int> inds){
    // cout << "Remove exits to region " << inds.size() << endl;
    vector<FORRExit> new_exits;
    for(int i = 0; i < exits.size(); i++){
      if(find(inds.begin(), inds.end(), exits[i].getExitRegion()) == inds.end()){
        new_exits.push_back(exits[i]);
      }
      // if(exits[i].getExitRegion() == ind){
      //   cout << "Found " << i << " " << exits[i].getExitRegion() << endl;
      //   exits.erase(exits.begin() + i);
      //   i--;
      // }
    }
    exits = new_exits;
    vector<FORRExit> new_ext_exits;
    for(int i = 0; i < ext_exits.size(); i++){
      if(find(inds.begin(), inds.end(), ext_exits[i].getExitRegion()) == inds.end()){
        new_ext_exits.push_back(ext_exits[i]);
      }
      // if(ext_exits[i].getExitRegion() == ind){
      //   ext_exits.erase(ext_exits.begin() + i);
      //   i--;
      // }
    }
    ext_exits = new_ext_exits;
    vector<FORRExit> new_min_exits;
    for(int i = 0; i < min_exits.size(); i++){
      if(find(inds.begin(), inds.end(), min_exits[i].getExitRegion()) == inds.end()){
        new_min_exits.push_back(min_exits[i]);
      }
      // if(min_exits[i].getExitRegion() == ind){
      //   min_exits.erase(min_exits.begin() + i);
      //   i--;
      // }
    }
    min_exits = new_min_exits;
  }

  void fixExitReferences(vector <int> inds){
    // cout << "Fixing exit references " << inds.size() << endl;
    for(int i = 0; i < exits.size(); i++){
      for(int j = 0; j < inds.size(); j++){
        if(exits[i].getExitRegion() > inds[j]){
          // cout << "Found " << i << " " << exits[i].getExitRegion() << endl;
          exits[i].setExitRegion(exits[i].getExitRegion() - 1);
        }
      }
      // if(exits[i].getExitRegion() > ind){
      //   cout << "Found " << i << " " << exits[i].getExitRegion() << endl;
      //   exits[i].setExitRegion(exits[i].getExitRegion() - 1);
      // }
    }
    for(int i = 0; i < ext_exits.size(); i++){
      for(int j = 0; j < inds.size(); j++){
        if(ext_exits[i].getExitRegion() > inds[j]){
          // cout << "Found " << i << " " << ext_exits[i].getExitRegion() << endl;
          ext_exits[i].setExitRegion(ext_exits[i].getExitRegion() - 1);
        }
      }
      // if(ext_exits[i].getExitRegion() > ind){
      //   ext_exits[i].setExitRegion(ext_exits[i].getExitRegion() - 1);
      // }
    }
    for(int i = 0; i < min_exits.size(); i++){
      for(int j = 0; j < inds.size(); j++){
        if(min_exits[i].getExitRegion() > inds[j]){
          // cout << "Found " << i << " " << min_exits[i].getExitRegion() << endl;
          min_exits[i].setExitRegion(min_exits[i].getExitRegion() - 1);
        }
      }
      // if(min_exits[i].getExitRegion() > ind){
      //   min_exits[i].setExitRegion(min_exits[i].getExitRegion() - 1);
      // }
    }
  }

  void setIsLeaf(bool leaf){isLeaf = leaf;}
  bool getIsLeaf() {return isLeaf;}

 private:
  CartesianPoint center;
  // vector < vector <CartesianPoint> > lasers;
  vector<double> min_visibility;
  vector<double> max_visibility;
  vector<double> avg_visibility;
  vector<double> count_visibility;
  vector<CartesianPoint> start_max_visibility;
  double radius;
  bool isLeaf;
  vector<int> passage_values;
  // each value in the list denotes a possible exit from the region
  vector<FORRExit> exits;
  vector<FORRExit> ext_exits;
  vector<FORRExit> min_exits;
};

struct RegionNode{
  FORRRegion region;
  int regionID;
  double nodeCost;
  vector<RegionNode> regionSequence;
  RegionNode(): region(FORRRegion()), regionID(-1), nodeCost(0), regionSequence(vector<RegionNode>()) { }
  RegionNode(FORRRegion r, int id, double c){
    region = r;
    regionID = id;
    nodeCost = c;
    vector<RegionNode> regions;
    regionSequence = regions;
  }
  void addToRegionSequence(RegionNode rn){
    regionSequence.push_back(rn);
  }
  void setRegionSequence(vector<RegionNode> rs){
    regionSequence = rs;
  }
  bool operator==(const RegionNode rn) {
    if(regionID == rn.regionID){
      return true;
    }
    else{
      return false;
    }
  }
  bool operator < (const RegionNode rn) const{
    if(nodeCost < rn.nodeCost){
      return true;
    }
    else{
      return false;
    }
  }
  bool operator > (const RegionNode rn) const{
    if(nodeCost > rn.nodeCost){
      return true;
    }
    else{
      return false;
    }
  }
};

#endif
