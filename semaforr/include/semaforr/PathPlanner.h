/*! \file PathPlanner.h

  \addtogroup PathPlanner
  @{
*/

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "astar.h"
#include "Position.h"
#include "FORRGeometry.h"
#include <math.h>
#include <vector>
#include "FORRConveyors.h"
#include "FORRRegion.h"
#include "FORRExit.h"
#include "FORRDoors.h"
#include "Aggregate.h"
#include <map>
#include <algorithm>
#include <queue>

//#include <semaforr/CrowdModel.h>

using namespace std;

/*! 
  \brief PathPlanner class in PathPlanner module

  This class manages the major navigational waypoints that construct a path from a source to a target. 
 */

class PathPlanner {
private: 
  Graph * navGraph;
  Graph * originalNavGraph;
  Map map;
  semaforr::CrowdModel crowdModel;
  Node source, target; 
  list<int> path;
  vector< list<int> > paths;
  double pathCost;
  list<int> origPath;
  list<int> origPath1;
  list<int> origPath2;
  list<int> origPath3;
  vector< list<int> > origPaths;
  double origPathCost;
  vector <double> pathCosts;
  vector <double> origPathCosts;
  string name;
  vector< vector<int> > posHistMap;
  vector< vector<double> > posHistMapNorm;
  double width;
  double height;
  int granularity;
  int boxes_width;
  int boxes_height;
  int map_height;
  int map_width;
  //SpatialModel* spatialModel;
  FORRConveyors* conveyors;
  vector<FORRRegion> regions;
  vector< vector<Door> > doors;
  vector< vector<CartesianPoint> > trails;
  vector<Aggregate> hallways;
  vector< vector<int> > passage_grid;
  std::map<int, vector< vector<int> > > passage_graph_nodes;
  vector< vector<int> > passage_average_values;
  vector< vector<int> > passage_graph;
  vector<Node> otherIntersection;
  vector<bool> usedOtherIntersection;
  vector< vector<int> > coverage_grid;
  bool use_coverage_grid;

  //list<int>::iterator head;
  Node waypoint; 
  bool objectiveSet;
  bool pathCompleted;
  bool pathCalculated;
  bool origObjectiveSet;
  bool origPathCompleted;
  bool origPathCalculated;

  void smoothPath(list<int>&, Node, Node);
  double computeCrowdFlow(Node s, Node d);
  double projection(double angle, double length, double xs, double ys, double xd, double yd);

public: 
  /*! \brief C'tor (only version) 
    \param Graph navigation graph 
    \param Node starting point (source)
    \param Node destination point (target)
  */
 PathPlanner(Graph * g, Map& m, Node s, Node t, string n): navGraph(g), map(m), source(s), target(t), name(n), pathCalculated(false), use_coverage_grid(false){}

 PathPlanner(Graph * g, Node s, Node t, string n): navGraph(g), source(s), target(t), name(n), pathCalculated(false), use_coverage_grid(false){}

  int calcPath(bool cautious = false);
  int calcOrigPath(bool cautious = false);

  /*! \return list of node indexes of waypoints */
  list<int> getPath(){ return path; }
  list<int> getOrigPath(){ return origPath; }

  vector< list<int> > getPaths(){ return paths; }
  vector< list<int> > getOrigPaths(){
    if(origPaths.size() < 3){
      origPaths.push_back(list<int>());
      origPaths.push_back(list<int>());
      origPaths.push_back(list<int>());
    }
    return origPaths;
  }

  void resetPath() { 
    path.clear();
    paths.clear();
    pathCompleted = true; 
    pathCalculated = false;
    otherIntersection.clear();
    usedOtherIntersection.clear();
    use_coverage_grid = false;
    // pathCost = 0;
    // pathCosts.clear();
  }

  void resetOrigPath() { 
    origPath.clear();
    origPath1.clear();
    origPath2.clear();
    origPath3.clear();
    origPaths.clear();
    origPathCompleted = true; 
    origPathCalculated = false;
    use_coverage_grid = false;
    // origPathCost = 0;
    // origPathCosts.clear();
  }

  void setCrowdModel(semaforr::CrowdModel c){ 
	crowdModel = c;
  }
  semaforr::CrowdModel getCrowdModel(){ return crowdModel;}

  void setOriginalNavGraph(Graph * navGraph){ 
    originalNavGraph = navGraph;
  }
  void setPosHistory(vector< vector<CartesianPoint> > all_trace){
    posHistMap.clear();
    posHistMapNorm.clear();
    if (name == "explore" or name == "combined"){
      width = navGraph->getMap()->getLength()/100;
      height = navGraph->getMap()->getHeight()/100;
      granularity = 10;
      boxes_width = width/granularity;
      boxes_height = height/granularity;
      map_height = height;
      map_width = width;
      for(int i = 0; i < boxes_width; i++){
        vector<int> col;
        for(int j = 0; j < boxes_height; j++){
          col.push_back(0);
        }
        posHistMap.push_back(col);
      }
      //cout << "width = " << width << " height = " << height << " granularity = " << granularity << " boxes_width = " << boxes_width << " boxes_height = " << boxes_height << " map_width = " << map_width << " map_height = " << map_height << endl;
      for(int i = 0; i < all_trace.size(); i++){
        for(int j = 0; j < all_trace[i].size(); j++) {
          //cout << "Pose " << i << ", " << j << " : x = " << all_trace[i][j].get_x() << " y = " << all_trace[i][j].get_y() << endl;
          //cout << "Modified x = " << (int)((all_trace[i][j].get_x()/(map_width*1.0)) * boxes_width) << " Modified y = " << (int)((all_trace[i][j].get_y()/(map_height*1.0)) * boxes_height) << endl;
          posHistMap[(int)((all_trace[i][j].get_x()/(map_width*1.0)) * boxes_width)][(int)((all_trace[i][j].get_y()/(map_height*1.0)) * boxes_height)] += 1;
        }
      }
      // double cmax=-1.0, cmin=1000000.0;
      // for(int i = 0; i < boxes_width; i++){
      //   for(int j = 0; j < boxes_height; j++){
      //     if(posHistMap[i][j]>cmax){
      //       cmax = posHistMap[i][j];
      //     }
      //     if(posHistMap[i][j]<cmin){
      //       cmin = posHistMap[i][j];
      //     }
      //   }
      // }
      //cout << "max = " << cmax << " min = " << cmin << endl;
      // for(int i = 0; i < boxes_width; i++){
      //   vector<double> colm;
      //   for(int j = 0; j < boxes_height; j++){
      //     //cout << "Cell val = " << posHistMap[i][j] << " Normed = " << ((double)posHistMap[i][j]-cmin)/(cmax-cmin) << endl;
      //     double normedCellVal = ((double)posHistMap[i][j]-cmin)/(cmax-cmin);
      //     //cout << "normedCellVal = " << normedCellVal << endl;
      //     colm.push_back(normedCellVal);
      //   }
      //   posHistMapNorm.push_back(colm);
      // }
      /*for(int i = 0; i < boxes_width; i++){
        for(int j = 0; j < boxes_height; j++){
          cout << posHistMapNorm[i][j] << " ";
        }
        cout << endl;
      }*/
    }
  }

  void setSpatialModel(FORRConveyors* cv, vector<FORRRegion> rgs, vector< vector<Door> > drs, vector< vector<CartesianPoint> > trl, vector<Aggregate> hlwys){
    conveyors = cv;
    regions = rgs;
    doors = drs;
    vector< vector<CartesianPoint> > interpolatedTrails;
    for(int i = 0; i < trl.size(); i++){
      vector<CartesianPoint> tempTrail;
      for(int j = 0; j < trl[i].size()-1; j++){
        tempTrail.push_back(trl[i][j]);
        tempTrail.push_back(CartesianPoint((trl[i][j].get_x()+trl[i][j+1].get_x())/2.0, (trl[i][j].get_y()+trl[i][j+1].get_y())/2.0));
        // double step_size = 0.1;
        // for(double step = 0; step < 1; step += step_size){
        //   double tx = (trl[i][j+1].get_x() * step) + (trl[i][j].get_x() * (1-step));
        //   double ty = (trl[i][j+1].get_y() * step) + (trl[i][j].get_y() * (1-step));
        //   tempTrail.push_back(CartesianPoint(tx,ty));
        // }
      }
      tempTrail.push_back(trl[i][trl[i].size()-1]);
      interpolatedTrails.push_back(tempTrail);
    }
    trails = interpolatedTrails;
    hallways = hlwys;
  }

  void setPassageGrid(vector< vector<int> > pg, std::map<int, vector< vector<int> > > pgn, vector< vector<int> > pgr, vector< vector<int> > ap){
    passage_grid = pg;
    passage_graph_nodes = pgn;
    passage_graph = pgr;
    passage_average_values = ap;
    otherIntersection.clear();
    usedOtherIntersection.clear();
  }

  void setCoverageGrid(vector< vector<int> > cg){
    coverage_grid = cg;
    use_coverage_grid = true;
  }

  void updateNavGraph();
  double computeNewEdgeCost(Node s, Node d, bool direction, double oldcost);

  Graph* getGraph(){ return navGraph; }
  void resetGraph(){
    navGraph->resetGraph();
    // int length = navGraph->getLength();
    // int height = navGraph->getHeight();
    // int proximity = navGraph->getProximity();
    // navGraph = new Graph(proximity, length, height);
  }
  Graph* getOrigGraph(){ return originalNavGraph; }
  void resetOrigGraph(){
    originalNavGraph->resetGraph();
  }

  Map* getMap() { return &map;}

  vector<FORRRegion> getRegions() { return regions; }

  Node getSource(){ return source; }

  void setSource(Node s){ source = s; } 

  Node getClosestNode(Node n, Node ref, bool isTarget);

  vector<Node> getClosestNodes(Node n, Node ref, bool findAny);

  Node getTarget(){ return target; }

  string getName(){ return name;}

  double cellCost(int sx, int sy, int buffer);

  double riskCost(int sx, int sy, int buffer);

  double novelCost(int sx, int sy);

  double computeConveyorCost(int sx, int sy);

  void setTarget(Node t){ target = t; }

  bool isAccessible(Node s, Node t);


  /*! \brief returns true if the line segment between \f$(x1,y1)\f$ and \f$(x2,y2)\f$ 
   *         is not obstructed by known obstacles represented in the map.
   */
  bool isInLineOfSight(int x1, int y1, int x2, int y2) { return !map.isPathObstructed(x1, y1, x2, y2); }


  //! Checks if a given \f$(x,y)\f$ is inside the buffer zones of the walls. 
  bool isPointCloseToWalls(int x, int y) { return map.isPointInBuffer(x, y); }


  /*! \brief returns a list of waypoints between arbitrary points, from source \f$(x1,y1)\f$ ending at target \f$(x2,y2)\f$.
   *         this function will not change the path that the robot is going to follow. it is used for checking if 
   *         paths exist between any two points, generally required for reasoning. Works a bit different than calcPath()
   *        
   *
   *  \param (x1,y1) represents the starting point or the source
   *  \param (x2,y2) represents the end point or the target
   *
   *  \return a list of \f$(x,y)\f$ pairs starting from the source ending with target
   *
   *  Initially if the \f$(x1,y1)\f$ and/or \f$(x2, y2)\f$ are not valid graph nodes (most of the cases are in this category)
   *  the function will call getClosestNode() which will return the closest valid node to the point, if one exists 
   *  within reasonable distance from it. If this fails the function will return the list containing a single pair
   *  <em> (source_error, target_error) </em>, which the values will be -1 if this is the case. 
   *  e.g. \f$(-1,0)\f$ means a valid source node was not found, while \f$(-1,-1)\f$ means neither a valid source or a target
   *       was found
   *
   *  If both source and target are valid but there is no path from one to the other the returned list will be empty.
   * 
   *  If a path is found it will be smoothed (first waypoint and last waypoint will be tested for relevance) 
   *
   *  \sa calcPath()
   *  \sa getClosestNode()
   *  \sa smoothPath()
   */
  list<pair<int,int> > getPathXYBetween(int, int, int, int);

  /*! \brief return the length of a given path in pairs of (x,y)
   */
  int getPathLength(list<pair<int,int> > path);

  /*! \brief this is used for checking if the source and target points on the map are within borders & not on a wall
   */
  bool isNodeValid(Node n){
    return (map.isWithinBorders(n.getX(), n.getY()) && map.isAccessible(n.getX(), n.getY()));
  }

  bool pathEmpty() { return path.empty(); }

  double getPathCost() { return pathCost; }
  double getOrigPathCost() { return origPathCost; }
  vector <double> getPathCosts() { return pathCosts; }
  vector <double> getOrigPathCosts() { return origPathCosts; }

  double getRemainingPathLength(double x, double y);  
    
  double calcPathCost(list<int>);
  double calcOrigPathCost(list<int>);
  double calcPathCost(vector<CartesianPoint> waypoints, Position source, Position target);

  double estimateCost(Node, Node, int); 

  double estimateCost(int x1, int y1, int x2, int y2);
  
  //! returns the first waypoint in the path
  Node getWaypoint() { 
    Node wp ; 
    ( !pathEmpty() ) ? wp = navGraph->getNode(path.front()) : wp = target; 
    return wp; 
  } 


  //! returns the second waypoint in the path
  Node getNextWaypoint() {
    Node wp; 
    if ( !pathEmpty() ) {
      list<int>::iterator iter=path.begin();
      iter++; 
      ( iter != path.end() ) ? wp = navGraph->getNode((*iter)) : wp = target; 
    }
    return wp;
  }

  bool isObjectiveSet() { return objectiveSet; }
 
  void waypointReached() {
    if ( !pathEmpty() )
      path.pop_front(); 
    objectiveSet = false;
  }
  
  // Double check: The change in Graph::addNode(Node&) returns a reference to the actual
  // node. This may cause it to be consumed when robot follows the actual path
  void waypointSet() {
    objectiveSet = true; 
    ( !pathEmpty() ) ? waypoint = navGraph->getNode(path.front()) : waypoint = target;
  }

  bool isPathCompleted() { 
    return pathCompleted;
  }

  bool isPathCalculated() { 
    return pathCalculated; 
  }

  /* Wrapper functions for adding 'hard' obstacles. These change the usability of edges */ 
  void clearGraph() { 
    navGraph->clearGraph(); 
  }
  
  void printPath();

  void printPath(list<int>);

  void printPath(list<pair<int,int> > p); 

  bool allWaypointsValid();
};

#endif

/*! @} */
