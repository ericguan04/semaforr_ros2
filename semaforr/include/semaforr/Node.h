#ifndef NODE_H
#define NODE_H

#include "Edge.h"
#include <iostream>
#include <vector> 
#include "FORRGeometry.h"
using namespace std; 

class Node {
protected:

  int id; 

  int x, y; 

  double radius;

  double distToWall;

  int intersection_id;
  
  bool inBuffer;           // true if within a wall buffer ( too close to a wall ) 

  bool accessible;         // false if the edges of this node are temporarily disabled.

  vector<int> neighbors;   // this is the adjacency list of the node. populated from Graph.

  vector<Edge*> nodeEdges;  // vector of edges that goes out from this node ( or comes in if not digraph) 
  
public:

  Node(int i = invalid_node_index, int xt = 0, int yt = 0, double r = 0, bool ib = false, double dw = 0)
    : id(i), x(xt), y(yt), radius(r), inBuffer(ib), distToWall(dw), accessible(true)
    {} 

  bool operator == (const Node& n) const{
    return ( this->id == n.getID() && this->x == n.getX() && this->y == n.getY() );
  }

  bool operator != (const Node& n) const{
    return ( this->id != n.getID() ); 
  }

  bool operator<(const Node& n) const{
    return ( this->id < n.getID() );
  }

  void setID(int i) { id = i; }

  int getID() const { return id; }

  void setIntersectionID(int i) { intersection_id = i; }

  int getIntersectionID() const { return intersection_id; }

  void setX(int x) { this->x = x; }

  int getX() const { return x; }

  void setY(int y) { this->y = y; }

  int getY() const { return y; }

  void setRadius(double r) { this->radius = r; }

  double getRadius() const { return radius; }

  void setDistWall(double dw) { this->distToWall = dw; }

  double getDistWall() const { return distToWall; }

  bool getInBuffer() const { return inBuffer; }

  void printNode() const{
    cout << "<NODE: " << id << " :(" << x << "," << y << ") >" ;
  }
  
  void printNeighbors() const {
    cout << "Node " << id << " neighbors: " ; 
    for( unsigned int i = 0; i < neighbors.size(); i++ ) {
      cout << neighbors.at(i) << "\t" ; 
    }
    cout << endl ;
  }

  void printNodeEdges() const { 
    cout << "Node " << id << " nodeEdges: " << endl;
    for ( unsigned int i = 0 ; i < nodeEdges.size(); i++ )
      nodeEdges.at(i)->printEdge(); 
  }

  //void setAccessible(bool b) { accessible = b; }
  bool isAccessible() { return accessible; }

  void setAccessible(bool b) {
    accessible = b; 
    vector<Edge*>::iterator iter; 
    for( iter = nodeEdges.begin(); iter != nodeEdges.end(); iter++ )
      (*iter)->setUsable(b);
  }

  void addNeighbor(int i) { neighbors.push_back(i); }

  vector<int> getNeighbors() { return neighbors; }

  void addNodeEdge(Edge* e) { nodeEdges.push_back(e); }

  vector<Edge*>& getNodeEdges() { return nodeEdges; }

  vector<Edge> getUsableNodeEdges() {
    vector<Edge> e; 
    vector<Edge*>::iterator iter;
    for ( iter = nodeEdges.begin() ; iter != nodeEdges.end() ; iter++ ) 
      if ( (*iter)->isUsable() ) 
	e.push_back(*(*iter)); 
    return e; 
  }

  int numNeighbors() const { return neighbors.size(); }

  bool isNeighbor(Node n) {
    vector<int>::iterator iter; 
    for( iter = neighbors.begin(); iter != neighbors.end(); iter++ )
      if ( *iter == n.getID() ) 
	return true;
    return false; 
  }

  bool neighborEmpty(){
    return ( neighbors.size() == 0 );
  }

  double getCostTo(int nid)
  {
    double cost;
    vector<Edge*>::iterator it;
    for(it = nodeEdges.begin(); it != nodeEdges.end(); it++)
    {
      Edge* tmp = *it;
      if(tmp->getTo() == this->id && tmp->getFrom() == nid)
      {
	//get cost from nid to this node 
        cost = tmp->getCost(false);
      }
      else if(tmp->getFrom() == this->id && tmp->getTo() == nid)
      {
	//get cost from this node to nid
        cost = tmp->getCost(true);
      }
    }

    return cost;
  }

  static const int invalid_node_index = -1; 
}; 

#endif

