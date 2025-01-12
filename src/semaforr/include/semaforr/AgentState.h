/*!
 * AgentState.h
 *
 * \brief Represents the current state of an agent: its current position,
 *        its plans, etc.).
 *
 * \author Eric Schneider <esch@hunter.cuny.edu>
 * \date 11/11/2011 Created
 */
#ifndef AGENTSTATE_H
#define AGENTSTATE_H

#include "Task.h"
#include "FORRAction.h"
#include "Position.h"
#include "FORRGeometry.h"

#include <time.h>
#include <unistd.h>

#include <vector>
#include <list>
#include <deque>
#include <set>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;

class AgentState
{
public:
  /** \brief AgentState constructor */
  AgentState(double arrMove[], double arrRotate[], int moveArrMax, int rotateArrMax) : currentTask(NULL) {
    vetoedActions = new set<FORRAction>();
    action_set = new set<FORRAction>();
    forward_set = new set<FORRAction>();
    rotation_set = new set<FORRAction>();
    rotateMode = true;
    //rotateMode = false;
    numMoves = moveArrMax;
    numRotates = rotateArrMax;
    robotConfined = false;
    getOutTriggered = false;
    farthestPoint = CartesianPoint(0.0,0.0);
    intermediatePoint = CartesianPoint(0.0,0.0);
    currentDirection = -1;
    desiredDirection = -1;
    directionEnd = false;
    repositionTriggered = false;
    repositionPoint = CartesianPoint(0.0,0.0);
    repositionCount = 0;
    findAWayCount = 0;
    enforcerCount = 0;
    for(int i = 1; i < numRotates; i++){
      action_set->insert(FORRAction(LEFT_TURN, i));
      action_set->insert(FORRAction(RIGHT_TURN, i));
      rotation_set->insert(FORRAction(LEFT_TURN, i));
      rotation_set->insert(FORRAction(RIGHT_TURN, i));
    }
    for(int i = 1; i < numMoves; i++){
      action_set->insert(FORRAction(FORWARD, i));
      forward_set->insert(FORRAction(FORWARD, i));
    }
    action_set->insert(FORRAction(PAUSE,0));
    forward_set->insert(FORRAction(PAUSE,0));
    //rotation_set->insert(FORRAction(PAUSE,0));
    //double m[] = {0, 0.2, 0.4, 0.8, 1.6, 3.2};  
    //double r[] = {0, 0.25, 0.5, 1, 2};
    for(int i = 0 ; i < numMoves ; i++) move[i] = arrMove[i];
    for(int i = 0 ; i < numRotates ; i++) rotate[i] = arrRotate[i];
    all_position_trace = new vector<Position>();
    all_laser_history = new vector< vector<CartesianPoint> >();
    all_laserscan_history = new vector< sensor_msgs::msg::LaserScan >();
    killBecauseStuck = false;
  }
  
  // Best possible move towards the target
  FORRAction moveTowards(CartesianPoint target);

  set<FORRAction> *getActionSet(){return action_set;}
  set<FORRAction> *getForwardActionSet(){return forward_set;}
  set<FORRAction> *getRotationActionSet(){return rotation_set;}

  double getDistanceToTarget(double x, double y){ 
    double dx = x - currentTask->getX();
    double dy = y - currentTask->getY();
    return sqrt((dx*dx) + (dy*dy));
  }

  
  // returns the euclidiean distance to target from the current robot position 
  double getDistanceToTarget(){
     return currentPosition.getDistance(currentTask->getX(), currentTask->getY());
  }
  
  bool isDestinationReached(int epsilon){
    if(getDistanceToTarget() < epsilon) {
        return true;
    }
    return false;
  }

  Position getCurrentPosition() { return currentPosition; }
  vector<CartesianPoint> getCurrentLaserEndpoints() { return laserEndpoints; }

  void setCurrentSensor(Position p, sensor_msgs::msg::LaserScan scan) { 
    currentPosition = p;
    currentLaserScan = scan;
    transformToEndpoints();
    if(currentTask != NULL){
      //save the current position and laser endpoints 
      all_laserscan_history->push_back(scan);
      all_position_trace->push_back(p);
      all_laser_history->push_back(laserEndpoints);
      currentTask->saveSensor(p, laserEndpoints, scan);
    }
  }
  
  Task *getCurrentTask() { return currentTask; }
  /*void setCurrentTask(Task *task, Position current, PathPlanner *planner, bool aStarOn) { 
    currentTask = task; 
    if(aStarOn){
    	currentTask->generateWaypoints(current, planner);
    } 
  }*/

  void setCurrentTask(Task *task){
    currentTask = task;
    //// RCLCPP_DEBUG_STREAM(this->get_logger(), "Current task set");
  }

  // list<int> getWaypoints(Position current, PathPlanner *planner, bool aStarOn){
  //   if(aStarOn){
  //     // RCLCPP_DEBUG_STREAM(this->get_logger(), "Generating waypoints");
  //     currentTask->generateWaypoints(current, planner);
  //     return currentTask->getWaypointInds();
  //   }
  // }

  vector< list<int> > getPlansWaypoints(Position current, PathPlanner *planner, bool aStarOn){
    if(aStarOn){
      // RCLCPP_DEBUG_STREAM(this->get_logger(), "Generating multiple sets of waypoints");
      currentTask->generateWaypoints(current, planner);
      return currentTask->getPlansInds();
    }
  }

  void setCurrentWaypoints(Position current, vector<CartesianPoint> currentLaserEndpoints, PathPlanner *planner, bool aStarOn, list<int> indices, vector<FORRRegion> regions){
    if(aStarOn){
      currentTask->generateWaypointsFromInds(current, currentLaserEndpoints, planner, indices, regions);
      currentTask->generateOriginalWaypoints(current, planner);
    }
  }

  set<FORRAction> *getVetoedActions() { 
	//std::cout << "returning vetoed action list " << vetoedActions->size() << std::endl;
	return vetoedActions;
  }
  void clearVetoedActions() { vetoedActions->clear();}
  
  void addTask(float x, float y, int l, int h) {
    Task *task = new Task(x,y,l,h);
    agenda.push_back(task); 
    all_agenda.push_back(task);
  }
  
  Task *getNextTask() { return agenda.front(); }
  
  list<Task*>& getAgenda() { return agenda; }
  list<Task*>& getAllAgenda() { return all_agenda; }
  
  //Merge finish task and skip task they are both doing the same right now
  void finishTask(bool skipTask) {
    if (currentTask != NULL){
      //save the current task position into all_trace
      vector<CartesianPoint> trace;
      vector<Position> *pos_hist = currentTask->getPositionHistory();
      for(int i = 0 ; i < pos_hist->size() ; i++){
        trace.push_back(CartesianPoint((*pos_hist)[i].getX(),(*pos_hist)[i].getY()));
        // all_position_trace->push_back((*pos_hist)[i]);
      }
      all_trace.push_back(trace);
      vector< vector<CartesianPoint> > *laser_hist = currentTask->getLaserHistory();
      all_laser_trace.push_back(*laser_hist);
      task_decision_count.push_back(currentTask->getDecisionCount());
      if(skipTask){
        if((*pos_hist)[0] == (*pos_hist)[pos_hist->size()-1]){
          killBecauseStuck = true;
        }
        else{
          killBecauseStuck = false;
        }
      }
      agenda.remove(currentTask);
    }
    rotateMode = true;
    currentTask = NULL;
  }

  vector< vector<CartesianPoint> > getAllTrace(){return all_trace;}
  vector< vector < vector<CartesianPoint> > > getAllLaserTrace(){return all_laser_trace;}
  vector< Position > *getAllPositionTrace(){return all_position_trace;}
  vector< vector<CartesianPoint> > *getAllLaserHistory(){return all_laser_history;}
  vector< sensor_msgs::msg::LaserScan > *getAllLaserScanHistory(){return all_laserscan_history;}

  vector< vector<CartesianPoint> > getInitialExitTraces(){return initial_exit_traces;}
  void setInitialExitTraces(vector< vector<CartesianPoint> > exit_traces){initial_exit_traces = exit_traces;}

  vector<int> getTaskDecisionCount(){return task_decision_count;}

  void skipTask() {
    if (currentTask != NULL)
      agenda.remove(currentTask);

    currentTask = NULL;
  }

  bool isMissionComplete(){
	bool status = false; 
	if(getAgenda().size() == 0 && currentTask == NULL){
		status = true;
 	}
  else if(killBecauseStuck == true){
    status = true;
  }
	return status;
  }

  sensor_msgs::msg::LaserScan getCurrentLaserScan(){return currentLaserScan;}

  vector<CartesianPoint> transformToEndpoints(Position p, sensor_msgs::msg::LaserScan scan);
  
  Position getExpectedPositionAfterAction(FORRAction action);

  Position getExpectedPositionAfterAction(FORRAction action, vector<CartesianPoint> initialLaser, Position currPosition);

  // Returns distance from obstacle 
  double getDistanceToNearestObstacle(Position pos);

  // Returns nearest obstacle 
  Position getNearestObstacle(Position pos);

  // returns distance to obstacle in the direction of rotation
  double getDistanceToObstacle(double rotation_angle);
  double getDistanceToObstacle(Position initialPosition, vector<CartesianPoint> initialLaser, double rotation_angle);
  double getDistanceToForwardObstacle(){
    //// RCLCPP_DEBUG(this->get_logger(), "in getDistance to forward obstacle");
    if(currentLaserScan.ranges.size() == 0)
      return 25;
    double min_distance = 25;
    int mid_index = currentLaserScan.ranges.size()/2;
    for(int i = mid_index-2; i < mid_index+3; i++){
      if(currentLaserScan.ranges[i] < min_distance){
        min_distance = currentLaserScan.ranges[i];
      }
    }
    // return currentLaserScan.ranges[currentLaserScan.ranges.size()/2];
    return min_distance;
  }
  double getDistanceToForwardObstacle(Position initialPosition, vector<CartesianPoint> initialLaser){
    //// RCLCPP_DEBUG(this->get_logger(), "in getDistance to forward obstacle");
    if(initialLaser.size() == 0)
      return 25;
    double min_distance = 25;
    int mid_index = initialLaser.size()/2;
    for(int i = mid_index-2; i < mid_index+3; i++){
      if(initialPosition.getDistance(initialLaser[i].get_x(), initialLaser[i].get_y()) < min_distance){
        min_distance = initialPosition.getDistance(initialLaser[i].get_x(), initialLaser[i].get_y());
      }
    }
    // return initialPosition.getDistance(initialLaser[initialLaser.size()/2].get_x(), initialLaser[initialLaser.size()/2].get_y());
    return min_distance;
  }

  FORRAction maxForwardAction();
  FORRAction maxForwardAction(Position initialPosition, vector<CartesianPoint> initialLaser);

  bool getRobotConfined(int decisionLimit, double distanceLimit);
  bool getGetOutTriggered(){
    cout << "getOutTriggered " << getOutTriggered << endl;
    return getOutTriggered;
  }
  void setGetOutTriggered(bool status){
    cout << "setGetOutTriggered " << status << endl;
    getOutTriggered = status;
    if(status == false){
      farthestPoint = CartesianPoint(0.0,0.0);
      intermediatePoint = CartesianPoint(0.0,0.0);
      getOutGrid.clear(); 
    }
  }

  void setFarthestPoint(CartesianPoint farthest_point){
    cout << "setFarthestPoint " << farthest_point.get_x() << " " << farthest_point.get_y() << endl;
    farthestPoint = farthest_point;
  }

  void setIntermediatePoint(CartesianPoint intermediate_point){
    cout << "setIntermediatePoint " << intermediate_point.get_x() << " " << intermediate_point.get_y() << endl;
    intermediatePoint = intermediate_point;
  }
  CartesianPoint getFarthestPoint(){return farthestPoint;}
  CartesianPoint getIntermediatePoint(){return intermediatePoint;}
  
  void setGetOutGrid(vector< vector<int> > grid){
    cout << "setGetOutGrid" << endl;
    getOutGrid = grid;
  }

  vector< vector<int> > getGetOutGrid(){
    cout << "getGetOutGrid" << endl;
    return getOutGrid;
  }

  int getCurrentDirection(){
    return currentDirection;
  }

  int getDesiredDirection(){
    return desiredDirection;
  }

  void setCurrentDirection(int num){
    currentDirection = num;
  }

  void setDesiredDirection(int num){
    desiredDirection = num;
  }

  void resetDirections(){
    currentDirection = -1;
    desiredDirection = -1;
    directionEnd = false;
  }

  bool getDirectionEnd(){
    return directionEnd;
  }

  void setDirectionEnd(bool status){
    directionEnd = status;
  }

  bool getRepositionTriggered(){
    cout << "repositionTriggered " << repositionTriggered << endl;
    return repositionTriggered;
  }
  void setRepositionTriggered(bool status){
    cout << "setRepositionTriggered " << status << endl;
    repositionTriggered = status;
    if(status == false){
      repositionPoint = CartesianPoint(0.0,0.0);
    }
  }

  void setRepositionPoint(CartesianPoint reposition_point){
    cout << "setRepositionPoint " << reposition_point.get_x() << " " << reposition_point.get_y() << endl;
    repositionPoint = reposition_point;
  }

  CartesianPoint getRepositionPoint(){return repositionPoint;}

  int getRepositionCount() { return repositionCount; }

  void setRepositionCount(int num){
    repositionCount = num;
  }

  void increaseRepositionCount(){ repositionCount++; }

  int getFindAWayCount() { return findAWayCount; }

  void setFindAWayCount(int num){
    findAWayCount = num;
  }

  void increaseFindAWayCount(){ findAWayCount++; }

  int getEnforcerCount() { return enforcerCount; }

  void setEnforcerCount(int num){
    enforcerCount = num;
  }

  void increaseEnforcerCount(){ enforcerCount++; }

  // Can a robot see a segment or a point using its laser scan data?
  bool canSeeSegment(CartesianPoint point1, CartesianPoint point2);
  bool canSeeSegment(vector<CartesianPoint> givenLaserEndpoints, CartesianPoint laserPos, CartesianPoint point1, CartesianPoint point2);
  bool canSeePoint(vector<CartesianPoint> givenLaserEndpoints, CartesianPoint laserPos, CartesianPoint point, double distanceLimit);
  bool canSeePoint(CartesianPoint point, double distanceLimit);
  // bool canAccessPoint(vector<CartesianPoint> givenLaserEndpoints, CartesianPoint laserPos, CartesianPoint point, double distanceLimit);
  bool canSeeRegion(CartesianPoint center, double radius, double distanceLimit);

  std::pair < std::vector<CartesianPoint>, std::vector< vector<CartesianPoint> > > getCleanedTrailMarkers();

  double getMovement(int para){return move[para];}
  double getRotation(int para){return rotate[para];}

  bool getRotateMode(){return rotateMode;}
  void setRotateMode(bool mode){rotateMode = mode;}

  double getRobotFootPrint(){return robotFootPrint;}
 
  void setAgentStateParameters(double val1, double val2, double val3, double val4, double val5, double val6, double val7);
  
  geometry_msgs::msg::PoseArray getCrowdPose(){ return currentCrowd;}
  void setCrowdPose(geometry_msgs::msg::PoseArray crowdpose){
	currentCrowd = crowdpose;
  }

  vector <Position> getCrowdPositions(geometry_msgs::msg::PoseArray crowdpose);

  geometry_msgs::msg::PoseArray getCrowdPoseAll(){ return allCrowd;}
  void setCrowdPoseAll(geometry_msgs::msg::PoseArray crowdposeall){
	allCrowd = crowdposeall;
  }

  void setCrowdModel(semaforr__msg__CrowdModel c){ 
    crowdModel = c;
  }
  semaforr__msg__CrowdModel getCrowdModel(){ return crowdModel;}

  bool crowdModelLearned();
  bool riskModelLearned();
  bool flowModelLearned();
  double getGridValue(double x, double y);
  double getRiskValue(double x, double y);
  double getFlowValue(double x, double y, double theta);
  double getCrowdObservation(double x, double y);
  double getRiskExperience(double x, double y);
  double getFLowObservation(double x, double y);

  void dfs(int x, int y, int current_label, vector<int> dx, vector<int> dy, int row_count, int col_count, vector< vector<int> > *label, vector< vector<int> > *m);

  void setPassageValues(vector< vector<int> > pg, map<int, vector< vector<int> > > pgn, map<int, vector< vector<int> > > pge, vector< vector<int> > pgr, vector< vector<int> > ap, vector< vector<CartesianPoint> > gt, vector< vector<int> > gti, vector< vector<CartesianPoint> > git){
    passage_grid = pg;
    passage_graph_nodes = pgn;
    passage_graph_edges = pge;
    passage_graph = pgr;
    average_passage = ap;
    graph_trails = gt;
    graph_through_intersections = gti;
    graph_intersection_trails = git;
  }

  void setRemainingCandidates(vector< vector<Position> > rc){
    remaining_candidates = rc;
  }

  vector< vector<int> > getPassageGrid(){
    return passage_grid;
  }

  void setPassageGrid(vector< vector<int> > pg){
    passage_grid = pg;
  }

  map<int, vector< vector<int> > > getPassageGraphNodes(){
    return passage_graph_nodes;
  }

  map<int, vector< vector<int> > > getPassageGraphEdges(){
    return passage_graph_edges;
  }

  vector< vector<int> > getPassageGraph(){
    return passage_graph;
  }

  vector< vector<int> > getAveragePassage(){
    return average_passage;
  }

  vector< vector<CartesianPoint> > getGraphTrails(){
    return graph_trails;
  }

  vector< vector<int> > getGraphThroughIntersections(){
    return graph_through_intersections;
  }

  vector< vector<CartesianPoint> > getGraphIntersectionTrails(){
    return graph_intersection_trails;
  }

  vector< vector<Position> > getRemainingCandidates(){
    return remaining_candidates;
  }

 private:

  // Stores the move and rotate action values
  double move[300];  
  double rotate[300];
  int numMoves, numRotates;

  FORRAction get_max_allowed_forward_move();

  // Current position of the agent x, y, theta 
  Position currentPosition;

  // All position history of all targets
  vector< vector<CartesianPoint> > all_trace;
  vector< Position > *all_position_trace;
  vector < vector<CartesianPoint> > initial_exit_traces;

  // All laser history of all targets
  vector< vector < vector<CartesianPoint> > > all_laser_trace;
  vector< vector<CartesianPoint> > *all_laser_history;
  vector< sensor_msgs::msg::LaserScan > *all_laserscan_history;

  // Decision count by task
  vector<int> task_decision_count;

  // set of vetoed actions that the robot cant execute in its current state
  set<FORRAction> *vetoedActions;

  // Set of all actions that the robot has in its action set
  set<FORRAction> *action_set;

  // Set of all forward actions that the robot has in its action set
  set<FORRAction> *forward_set;

  // Set of all forward actions that the robot has in its action set
  set<FORRAction> *rotation_set;
  
  // aggregate decision making statistics of the agent
  // Total travel time
  double total_travel_time;

  // Total travel distance
  double total_travel_distance;

  // Total tier 1 decisions 
  int total_t1_decisions;

  // Total tier 2 decisions
  int total_t2_decisions;

  // Total tier 3 decisions
  int total_t3_decisions;
  
  /** \brief The agent's currnet list of tasks */
  list<Task*> agenda;

  // list of all targets;
  list<Task*> all_agenda;
  
  // Current task in progress
  Task *currentTask;

  // Currrent laser scan reading at the current position
  sensor_msgs::msg::LaserScan currentLaserScan;

  // Current laser scan data as endpoints in the x-y coordinate frame
  vector<CartesianPoint> laserEndpoints;

  //Converts current laser range scanner to endpoints
  void transformToEndpoints();

  // Nearby crowd positions
  geometry_msgs::msg::PoseArray currentCrowd;

  // All crowd positions
  geometry_msgs::msg::PoseArray allCrowd;

  // Current crowd model
  semaforr__msg__CrowdModel crowdModel;

  //Rotate mode tells if the t3 should rotate or move
  bool rotateMode;

  // Robot confined currently
  bool robotConfined;
  bool getOutTriggered;
  CartesianPoint farthestPoint;
  CartesianPoint intermediatePoint;
  vector< vector<int> > getOutGrid;

  // Circumnavigate
  int currentDirection;
  int desiredDirection;
  bool directionEnd;

  // Doorway
  CartesianPoint repositionPoint;
  bool repositionTriggered;
  int repositionCount;

  // FindAWay
  int findAWayCount;

  // Enforcer
  int enforcerCount;

  //after linear move
  Position afterLinearMove(Position initialPosition, double distance);
  Position afterAngularMove(Position initialPosition, double angle);
  Position afterLinearMove(Position initialPosition, vector<CartesianPoint> initialLaser, double distance);
  Position afterAngularMove(Position initialPosition, vector<CartesianPoint> initialLaser, double angle);

  // Parameters
  double canSeePointEpsilon, laserScanRadianIncrement, robotFootPrint, robotFootPrintBuffer, maxLaserRange, maxForwardActionBuffer, maxForwardActionSweepAngle;

  bool killBecauseStuck;

  // Passage Values
  vector< vector<int> > passage_grid;
  map<int, vector< vector<int> > > passage_graph_nodes;
  map<int, vector< vector<int> > > passage_graph_edges;
  vector< vector<int> > passage_graph;
  vector< vector<int> > average_passage;
  vector< vector<CartesianPoint> > graph_trails;
  vector< vector<int> > graph_through_intersections;
  vector< vector<CartesianPoint> > graph_intersection_trails;
  vector< vector<Position> > remaining_candidates;

};

#endif
