#ifndef EDGE_H
#define EDGE_H

#include <vector>
#include <iostream>
#include "FORRGeometry.h"

using namespace std;

class Edge {
private:
  int from, to;  
  bool usable; 

protected:
  double costfromto, costtofrom, cost;
  vector<CartesianPoint> path;
  vector<CartesianPoint> reversepath;

public:
  Edge(): from (-1), to(-1), costfromto(0), costtofrom(0) { usable = true; cost = 0; } 
  Edge(int n1, int n2, double cft = 0, double ctf = 0): from(n1), to(n2), costfromto(cft), costtofrom(ctf)    { usable = true; }
  bool operator == ( Edge e ) {
    int ef = e.getFrom();
    int et = e.getTo(); 
    // double ecost = e.getDistCost();
    // return ( from == ef && to == et && cost == ecost);
    return ( from == ef && to == et);
  }

  double getDistCost(){
	return cost;
  }

  double getCost(bool direction) {
	if(direction) return costfromto; 
	else          return costtofrom;
  }

  void setCost(double cfromto, double ctofrom) { 
	costfromto = cfromto;
	costtofrom = ctofrom; 
  }

  void setDistCost(double c){
	costfromto = c;
	costtofrom = c;
	cost = c;
  }

  void setEdgePath(vector<CartesianPoint> p){
    path = p;
    for(int i = p.size()-1; i >= 0; i--){
      reversepath.push_back(p[i]);
    }
  }

  vector<CartesianPoint> getEdgePath(bool direction){
    if(direction)
      return path; 
    else
      return reversepath;
  }

  int getFrom(){ return from; }
  int getTo(){ return to; }

  void setUsable(bool b) { usable = b; }
  bool isUsable() { return usable; }

  void printEdge() const{
    cout << "<EDGE-From Node:" << from 
	 << " -To Node:" << to 
	 << " - Costfromto: " << costfromto 
	 << " - Costtofrom: " << costtofrom << " >" << endl;
  }
}; 

#endif
