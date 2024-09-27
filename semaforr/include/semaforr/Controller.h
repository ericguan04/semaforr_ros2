#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <map>
#include <set>
#include <string>
#include <time.h>
#include <limits.h>
#include <cstdlib>
#include <time.h>
#include <math.h>
#include <vector>
#include <utility>

// SemaFORR
#include "Beliefs.h"
#include "Tier1Advisor.h"
#include "Tier3Advisor.h"
#include "FORRActionStats.h"
#include "PathPlanner.h"
#include "HighwayExplore.h"
#include "FrontierExplore.h"
#include "Circumnavigate.h"
#include "FORRPassages.h"

#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


// Forward-declare Controller so the typedef below can reference it
class Controller;
typedef std::vector<Tier3Advisor*>::iterator advisor3It;
typedef std::vector<PathPlanner*>::iterator planner2It;

// ROS Controller class 
class Controller {
public:

  Controller(string, string, string, string, string);
  
  //main sense decide loop, receives the input messages and calls the FORRDecision function
  FORRAction decide();

  FORRActionStats *getCurrentDecisionStats() { return decisionStats; }
  void clearCurrentDecisionStats() { decisionStats = new FORRActionStats();}

  //Update state of the agent using sensor readings 
  void updateState(Position current, sensor_msgs::msg::LaserScan laserscan, geometry_msgs::msg::PoseArray crowdpose, geometry_msgs::msg::PoseArray crowdposeall);

  //Returns the state of the robots mission (True 
  bool isMissionComplete();

  // getter for beliefs
  Beliefs *getBeliefs() { return beliefs; }

  // getter for planner
  PathPlanner *getPlanner() { return planner; }

  std::vector<PathPlanner*> getPlanners() { return tier2Planners; }

  void updatePlannersModels(semaforr__msg__CrowdModel c) {
    for (planner2It it = tier2Planners.begin(); it != tier2Planners.end(); it++){
      PathPlanner *planner = *it;
      planner->setCrowdModel(c);
    }
  }

  HighwayExplorer *gethighwayExploration() { return highwayExploration; }
  FrontierExplorer *getfrontierExploration() { return frontierExploration; }

  bool getHighwayFinished(){
    if(highwayFinished >= 1){
      return true;
    }
    else{
      return false;
    }
  }

  bool getFrontierFinished(){
    if(frontierFinished >= 1){
      return true;
    }
    else{
      return false;
    }
  }

  int getHighwaysOn(){
    if(highwaysOn){
      return 1;
    }
    else if(frontiersOn){
      return 2;
    }
    else{
      return 0;
    }
  }

private:

  //FORR decision loop and tiers
  FORRAction FORRDecision();

  FORRActionStats *decisionStats;
  
  //Tier 1 advisors are called here
  bool tierOneDecision(FORRAction *decision);

  //Tier 2 planners are called here
  void tierTwoDecision(Position current, bool selectNextTask);

  //Tier 3 advisors are called here
  void tierThreeDecision(FORRAction *decision);

  //Check influence of tier 3 Advisors
  void tierThreeAdvisorInfluence();

  // learns the spatial model and updates the beliefs
  void learnSpatialModel(AgentState *agentState, bool taskStatus, bool earlyLearning);
  void updateSkeletonGraph(AgentState* agentState);

  void initialize_advisors(std::string);
  void initialize_tasks(std::string, int length, int height);
  void initialize_params(std::string);
  void initialize_planner(std::string,std::string, int &l, int &h);
  void initialize_situations(std::string);
  void initialize_spatial_model(std::string);
  
  // Knowledge component of robot
  Beliefs *beliefs;

  HighwayExplorer *highwayExploration;
  FrontierExplorer *frontierExploration;
  Circumnavigate *circumnavigator;

  // An ordered list of advisors that are consulted by Controller::FORRDecision
  Tier1Advisor *tier1;
  PathPlanner *planner;
  std::vector<PathPlanner*> tier2Planners;
  std::vector<Tier3Advisor*> tier3Advisors;
  
  // Checks if a given advisor is active
  bool isAdvisorActive(string advisorName);

  double canSeePointEpsilon, laserScanRadianIncrement, robotFootPrint, robotFootPrintBuffer, maxLaserRange, maxForwardActionBuffer, maxForwardActionSweepAngle, highwayDistanceThreshold, highwayTimeThreshold, highwayDecisionThreshold;
  double arrMove[300];
  double arrRotate[300];
  int moveArrMax, rotateArrMax;
  int taskDecisionLimit;
  int planLimit;
  bool trailsOn;
  bool conveyorsOn;
  bool regionsOn;
  bool doorsOn;
  bool hallwaysOn;
  bool barrsOn;
  bool aStarOn;
  bool situationsOn;
  bool highwaysOn;
  bool frontiersOn;
  bool outofhereOn;
  bool doorwayOn;
  bool findawayOn;
  bool behindOn;
  bool dontgobackOn;
  bool firstTaskAssigned;
  int highwayFinished;
  int frontierFinished;
  bool distance, smooth, novel, density, risk, flow, combined, CUSUM, discount, explore, spatial, hallwayer, trailer, barrier, conveys, safe, skeleton, hallwayskel;
};
  
#endif /* CONTROLLER_H */
