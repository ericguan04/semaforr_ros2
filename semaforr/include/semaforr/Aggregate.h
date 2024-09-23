/************************************************
Aggregate.h 
This file contains the class which contains information about the aggregates used to represent hallways that FORR learns and uses

Written by Raj Korpan, adapted from Sarah Mathew and Gil Dekel, 2018
**********************************************/

#ifndef AGGREGATE_H
#define AGGREGATE_H

#include <FORRGeometry.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <cmath>        //for atan2 and M_PI
#include <algorithm>

using namespace std;

//=========================================================//=========================================================//

struct Aggregate {
public:
  Aggregate(vector<CartesianPoint> coordinates, int id): points_(coordinates), hallway_type_(id) {}

  vector<CartesianPoint> getPoints() const {return points_;}

  int getHallwayType() const {return hallway_type_;}

  bool pointInAggregate(CartesianPoint point){
    //cout << "Inside pointInAggregate" << endl;
    std::vector<CartesianPoint>::iterator it;
    CartesianPoint roundedPoint = CartesianPoint((int)(point.get_x()),(int)(point.get_y()));
    //cout << "point x = " << point.get_x() << ", y = " << point.get_y() << "; Rounded point x = " << roundedPoint.get_x() << ", y = " << roundedPoint.get_y() << endl;
    it = find(points_.begin(), points_.end(), roundedPoint);
    if(it != points_.end()){
      //cout << "point found in aggregate" << endl;
      return true;
    }
    else{
      //cout << "point NOT found in aggregate" << endl;
      return false;
    }
  }

  double distanceToAggregate(CartesianPoint point){
    //cout << "Inside distanceToAggregate" << endl;
    std::vector<CartesianPoint>::iterator it;
    CartesianPoint roundedPoint = CartesianPoint((int)(point.get_x()),(int)(point.get_y()));
    //cout << "point x = " << point.get_x() << ", y = " << point.get_y() << "; Rounded point x = " << roundedPoint.get_x() << ", y = " << roundedPoint.get_y() << endl;
    it = find(points_.begin(), points_.end(), roundedPoint);
    double dist = 1000000.0;
    if(it != points_.end()){
      //cout << "point found in aggregate so distance = 0.0" << endl;
      dist = 0.0;
    }
    else{
      //cout << "point NOT found in aggregate" << endl;
      for(int i = 0; i < points_.size(); i++){
        double tempDist = point.get_distance(points_[i]);
        if(tempDist < dist){
          dist = tempDist;
        }
      }
    }
    //cout << "distance = " << dist << endl;
    return dist;
  }

  void findConnection(Aggregate &hlwy, int id1, int id2){
    //cout << "Inside findConnection" << endl;
    for(int i = 0; i < points_.size(); i++){
      if(hlwy.pointInAggregate(points_[i]) == true){
        connectedHallways.push_back(id2);
        hlwy.addConnection(id1);
        //cout << "Connection found between " << id1 << " and " << id2 << endl;
        break;
      }
    }
  }

  void addConnection(int id){
    connectedHallways.push_back(id);
  }

  int numConnections(){
    //cout << "Inside numConnections = " << connectedHallways.size() << endl;
    return connectedHallways.size();
  }

  bool isHallwayConnected(int id){
    //cout << "Inside isHallwayConnected" << endl;
    for(int i = 0; i < connectedHallways.size(); i++){
      if(connectedHallways[i] == id){
        // cout << "Hallways are connected" << endl;
        return true;
      }
    }
    // cout << "Hallways are NOT connected" << endl;
    return false;
  }

  double distanceBetweenAggregates(Aggregate hlwy){
    //cout << "Inside distanceBetweenAggregates" << endl;
    double dist = 1000000.0;
    for(int i = 0; i < points_.size(); i++){
      double tempDist = hlwy.distanceToAggregate(points_[i]);
      if(tempDist < dist){
        dist = tempDist;
      }
    }
    //cout << "Distance between = " << dist << endl;
    return dist;
  }

  vector<double> closestPointsBetweenAggregates(Aggregate hlwy){
    //cout << "Inside closestPointsBetweenAggregates" << endl;
    vector<CartesianPoint> other_points_ = hlwy.getPoints();
    double dist = 1000000.0;
    CartesianPoint currentHallwayPoint;
    CartesianPoint otherHallwayPoint;
    for(int i = 0; i < points_.size(); i++){
      for(int j = 0; j < other_points_.size(); j++){
        double tempDist = points_[i].get_distance(other_points_[j]);
        if(tempDist < dist){
          dist = tempDist;
          currentHallwayPoint = points_[i];
          otherHallwayPoint = other_points_[j];
        }
      }
    }
    vector<double> values;
    values.push_back(dist);
    values.push_back(currentHallwayPoint.get_x());
    values.push_back(currentHallwayPoint.get_y());
    values.push_back(otherHallwayPoint.get_x());
    values.push_back(otherHallwayPoint.get_y());
    return values;
  }

private:
  vector<CartesianPoint> points_;
  int hallway_type_;
  vector<int> connectedHallways;
};

#endif