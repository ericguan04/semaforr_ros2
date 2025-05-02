/*
 *
 * Implementation of Advisors 
 *
 * Created on: Jun. 27, 2013
 * Last modified: April 14, 2013
 * created by Slavisa Djukic <sdjukic@hunter.cuny.edu>
 */

# include "Tier3Advisor.h"
# include "FORRAction.h"
# include <cmath>
# include <iostream>
# include <cstdlib>
# include <time.h>
# include <utility>
# include <limits>
# include "sensor_msgs/msg/laser_scan.hpp"

using std::set;
  
// Constructor for Tier3Advisor
Tier3Advisor::Tier3Advisor(Beliefs *beliefs_para, string name_para, string description_para, double weight_para, double *magic_init, bool is_active)  { 
  beliefs = beliefs_para;
  name = name_para;
  description = description_para;
  weight = weight_para;
  active = is_active;
  // load all magic numbers for this advisor
  for (int i = 0; i < 4; ++i)
    auxiliary_constants[i] = magic_init[i];
  //cout << "after initializing auxilary constants" << endl;
}

// Destructor
Tier3Advisor::~Tier3Advisor() {};

// this function will produce comment strengths for all possible actions 
// robot can make at the certain moment
// it does it by successively calling actionComment method for each action
// No arguments are necessary because actions are stored as member variable
// in advisor 
// It returns map that maps action to comment strength
std::map <FORRAction, double> Tier3Advisor::allAdvice(){
  //cout << "IN All advisor t3: " << beliefs->getAgentState()->isMissionComplete() << endl;
  set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
  set<FORRAction> *action_set;
  //cout << "IN All advisor t3: " << endl;
  bool inRotateMode = beliefs->getAgentState()->getRotateMode();
  //cout << "Decision Count : " << beliefs->getAgentState()->getCurrentTask()->getDecisionCount() << endl;
  
  //cout << "Rotation mode : " << inRotateMode << endl;
  action_set = beliefs->getAgentState()->getActionSet();
  inRotateMode = true;
  /*if(inRotateMode){
	action_set = beliefs->getAgentState()->getRotationActionSet();
  }
  else{
	action_set = beliefs->getAgentState()->getForwardActionSet();
  }*/

  std::map <FORRAction, double> result;
 
  std::size_t foundr = (this->get_name()).find("Rotation");
  // If the advisor is linear and the agent is in rotation mode
  if((foundr == std::string::npos) and inRotateMode){
	//cout << "Advisor is linear and agent is in rotation mode" << endl; 
      return result;
  }
  // If the advisor is rotation and the agent is in linear mode
  if((foundr != std::string::npos) and !inRotateMode){
	//cout << "Advisor is rotation and agent is in linear mode" << endl;
      return result;
  }

  double adviceStrength;
  FORRAction forrAction;
  //std::cout << this->agent_name << ": in allAdvice function .. comments are as following:" << std::endl;
  set<FORRAction>::iterator actionIter;
  for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
    forrAction = *actionIter;
    //cout << forrAction.type << " " << forrAction.parameter << endl;
    if(vetoed_actions->find(forrAction) != vetoed_actions->end())// is this action vetoed
      continue;
    adviceStrength = this->actionComment(forrAction);
    // std::cout << "Advisor name :"  << this->get_name() << " Strength: " << adviceStrength << " Action Type:" << forrAction.type << " " << "Action intensity " << forrAction.parameter << std::endl;
    result[forrAction] = adviceStrength;
  } 
  //if(result.size() > 1){
  normalize(&result);
  //rank(&result);
  //standardize(&result);
  //}
  return result;
}


//normalizing from 0 to 10
void Tier3Advisor::
normalize(map <FORRAction, double> * result){
  double max = -std::numeric_limits<double>::infinity(), min = std::numeric_limits<double>::infinity();
  map<FORRAction, double>::iterator itr;
  for(itr = result->begin(); itr != result->end() ; itr++){
    if(max < itr->second)  max = itr->second;
    if(min > itr->second)  min = itr->second;
  } 
  //std::cout << "Inside normalize " << max << " " << min << endl;
  if(max != min and result->size() > 1 and max != -std::numeric_limits<double>::infinity() and min != std::numeric_limits<double>::infinity()){
    double norm_factor = (max - min)/10;
    for(itr = result->begin(); itr != result->end() ; itr++){
      //cout << "Before : " << itr->second << endl;
      itr->second = (itr->second - min)/norm_factor;
    }
  }
  else if(max == min and result->size() > 1){
    for(itr = result->begin(); itr != result->end() ; itr++){
      //cout << "Before : " << itr->second << endl;
      itr->second = 0;
    }
  }
  else {
    result->begin()->second = 0;
  }
}

//rank the comments 
void Tier3Advisor::
rank(map <FORRAction, double> *result){
  //cout << "start compute advisor preference function" << endl;
  std::set<double> distinctComments;
  map<FORRAction, double>::iterator iter1, iter2;
  for(iter1 = result->begin(); iter1 != result->end(); iter1++){
    distinctComments.insert(iter1->second);
  }
  for(iter2 = result->begin(); iter2 != result->end(); iter2++){
    double rank = result->size();
    double test = iter2->second;
    set<double>::iterator iter; 
    for(iter = distinctComments.begin(); iter != distinctComments.end(); iter++){
      if(test < (*iter)){
        rank--;
      }
    }
    iter2->second = rank;
  }
}

//standardize the comments by converting to a z-score
void Tier3Advisor::
standardize(map <FORRAction, double> * result){
  double mean = 0, count = 0, stdDev = 0;
  map<FORRAction, double>::iterator itr;
  for(itr = result->begin(); itr != result->end() ; itr++){
    mean += itr->second;
    count++;
  }
  mean = (mean / count);

  for(itr = result->begin(); itr != result->end() ; itr++){
    stdDev += pow((itr->second - mean), 2);
  }
  stdDev = sqrt(stdDev / count);
  if(stdDev != 0) {
    for(itr = result->begin(); itr != result->end() ; itr++){
      //cout << "Before : " << itr->second << endl;
      itr->second = ((itr->second - mean)/stdDev);
    }
  }
  else {
    for(itr = result->begin(); itr != result->end() ; itr++){
      //cout << "Before : " << itr->second << endl;
      itr->second = 0;
    }
  }
}

// factory definition
Tier3Advisor* Tier3Advisor::makeAdvisor(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active){
  if(name == "Greedy")
    return new Tier3Greedy(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ElbowRoom")
    return new Tier3ElbowRoom(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "CloseIn")
    //return new Tier3CloseIn(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "BigStep")
    return new Tier3BigStep(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "Unlikely")
    return new Tier3Unlikely(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "UnlikelyRotation")
    return new Tier3UnlikelyRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "UnlikelyField")
    return new Tier3UnlikelyField(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "UnlikelyFieldRotation")
    return new Tier3UnlikelyFieldRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "Explorer")
    return new Tier3Explorer(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExplorerEndPoints")
    return new Tier3ExplorerEndPoints(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "BaseLine")
    return new Tier3BaseLine(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "GreedyRotation")
    return new Tier3GreedyRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ElbowRoomRotation")
    return new Tier3ElbowRoomRotation(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "CloseInRotation")
    //return new Tier3CloseInRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "BigStepRotation")
    return new Tier3BigStepRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "GoAroundRotation")
    return new Tier3GoAroundRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExplorerRotation")
    return new Tier3ExplorerRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExplorerEndPointsRotation")
    return new Tier3ExplorerEndPointsRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "BaseLineRotation")
    return new Tier3BaseLineRotation(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "AvoidRobotRotation")
    //return new Tier3AvoidRobotRotation(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "AvoidRobot")
    //return new Tier3AvoidRobot(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitLinear")
    return new Tier3ExitLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitRotation")
    return new Tier3ExitRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitFieldLinear")
    return new Tier3ExitFieldLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitFieldRotation")
    return new Tier3ExitFieldRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitClosest")
    return new Tier3ExitClosest(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitClosestRotation")
    return new Tier3ExitClosestRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "RegionLeaverLinear")
    return new Tier3RegionLeaverLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "RegionLeaverRotation")
    return new Tier3RegionLeaverRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "EnterLinear")
    return new Tier3EnterLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "EnterRotation")
    return new Tier3EnterRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "EnterExit")
    return new Tier3EnterExit(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "EnterExitRotation")
    return new Tier3EnterExitRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ConveyLinear")
    return new Tier3ConveyLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ConveyRotation")
    return new Tier3ConveyRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "TrailerLinear")
    return new Tier3TrailerLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "TrailerRotation")
    return new Tier3TrailerRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "EnterDoorLinear")
    return new Tier3EnterDoorLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "EnterDoorRotation")
    return new Tier3EnterDoorRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitDoorLinear")
    return new Tier3ExitDoorLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ExitDoorRotation")
    return new Tier3ExitDoorRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "AccessLinear")
    return new Tier3AccessLinear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "AccessRotation")
    return new Tier3AccessRotation(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "NeighborDoorLinear")
    //return new Tier3NeighborDoorLinear(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "NeighborDoorRotation")
    //return new Tier3NeighborDoorRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "LearnSpatialModel")
    return new Tier3LearnSpatialModel(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "LearnSpatialModelRotation")
    return new Tier3LearnSpatialModelRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "Curiosity")
    return new Tier3Curiosity(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "CuriosityRotation")
    return new Tier3CuriosityRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "Enfilade")
    return new Tier3Enfilade(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "EnfiladeRotation")
    return new Tier3EnfiladeRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "Thigmotaxis")
    return new Tier3Thigmotaxis(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "ThigmotaxisRotation")
    return new Tier3ThigmotaxisRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "VisualScanRotation")
    return new Tier3VisualScanRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "LeastAngle")
    return new Tier3LeastAngle(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "LeastAngleRotation")
    return new Tier3LeastAngleRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "Interpersonal")
    return new Tier3Interpersonal(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "InterpersonalRotation")
    return new Tier3InterpersonalRotation(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "Formation")
  //  return new Tier3Formation(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "FormationRotation")
  //  return new Tier3FormationRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "Front")
    return new Tier3Front(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "FrontRotation")
    return new Tier3FrontRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "Rear")
    return new Tier3Rear(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "RearRotation")
    return new Tier3RearRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "Side")
    return new Tier3Side(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "SideRotation")
    return new Tier3SideRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "Visible")
    return new Tier3Visible(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "VisibleRotation")
    return new Tier3VisibleRotation(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "Wait")
  //  return new Tier3Wait(beliefs, name, description, weight, magic_init, is_active);
  //else if(name == "WaitRotation")
  //  return new Tier3WaitRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "CrowdAvoid")
    return new Tier3CrowdAvoid(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "CrowdAvoidRotation")
    return new Tier3CrowdAvoidRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "FindTheCrowd")
    return new Tier3FindTheCrowd(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "FindTheCrowdRotation")
    return new Tier3FindTheCrowdRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "RiskAvoid")
    return new Tier3RiskAvoid(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "RiskAvoidRotation")
    return new Tier3RiskAvoidRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "FindTheRisk")
    return new Tier3FindTheRisk(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "FindTheRiskRotation")
    return new Tier3FindTheRiskRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "FlowAvoid")
    return new Tier3FlowAvoid(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "FlowAvoidRotation")
    return new Tier3FlowAvoidRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "FindTheFlow")
    return new Tier3FindTheFlow(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "FindTheFlowRotation")
    return new Tier3FindTheFlowRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "Follow")
    return new Tier3Follow(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "FollowRotation")
    return new Tier3FollowRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "Crossroads")
    return new Tier3Crossroads(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "CrossroadsRotation")
    return new Tier3CrossroadsRotation(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "Stay")
    return new Tier3Stay(beliefs, name, description, weight, magic_init, is_active);
  else if(name == "StayRotation")
    return new Tier3StayRotation(beliefs, name, description, weight, magic_init, is_active);
  else 
    std::cout << "No such advisor " << std::endl;
}

// Constructors 
Tier3Greedy::Tier3Greedy (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3ElbowRoom::Tier3ElbowRoom(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
//Tier3CloseIn::Tier3CloseIn(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3BigStep::Tier3BigStep(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};  
Tier3Unlikely::Tier3Unlikely(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3UnlikelyField::Tier3UnlikelyField(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3Explorer::Tier3Explorer(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3ExplorerEndPoints::Tier3ExplorerEndPoints(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3BaseLine::Tier3BaseLine (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
//Tier3AvoidRobot::Tier3AvoidRobot (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 

Tier3GreedyRotation::Tier3GreedyRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3ElbowRoomRotation::Tier3ElbowRoomRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
//Tier3CloseInRotation::Tier3CloseInRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3BigStepRotation::Tier3BigStepRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3GoAroundRotation::Tier3GoAroundRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3UnlikelyRotation::Tier3UnlikelyRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3UnlikelyFieldRotation::Tier3UnlikelyFieldRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3ExplorerRotation::Tier3ExplorerRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3ExplorerEndPointsRotation::Tier3ExplorerEndPointsRotation(Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3BaseLineRotation::Tier3BaseLineRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
//Tier3AvoidRobotRotation::Tier3AvoidRobotRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3ExitLinear::Tier3ExitLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ExitRotation::Tier3ExitRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ExitFieldLinear::Tier3ExitFieldLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ExitFieldRotation::Tier3ExitFieldRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ExitClosest::Tier3ExitClosest (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ExitClosestRotation::Tier3ExitClosestRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3RegionLeaverLinear::Tier3RegionLeaverLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3RegionLeaverRotation::Tier3RegionLeaverRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3EnterLinear::Tier3EnterLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3EnterRotation::Tier3EnterRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3EnterExit::Tier3EnterExit (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3EnterExitRotation::Tier3EnterExitRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ConveyLinear::Tier3ConveyLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ConveyRotation::Tier3ConveyRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3TrailerLinear::Tier3TrailerLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3TrailerRotation::Tier3TrailerRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3EnterDoorLinear::Tier3EnterDoorLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3EnterDoorRotation::Tier3EnterDoorRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ExitDoorLinear::Tier3ExitDoorLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ExitDoorRotation::Tier3ExitDoorRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3AccessLinear::Tier3AccessLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3AccessRotation::Tier3AccessRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
//Tier3NeighborDoorLinear::Tier3NeighborDoorLinear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
//Tier3NeighborDoorRotation::Tier3NeighborDoorRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3LearnSpatialModel::Tier3LearnSpatialModel (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3LearnSpatialModelRotation::Tier3LearnSpatialModelRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3Curiosity::Tier3Curiosity (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3CuriosityRotation::Tier3CuriosityRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3Enfilade::Tier3Enfilade (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3EnfiladeRotation::Tier3EnfiladeRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3Thigmotaxis::Tier3Thigmotaxis (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3ThigmotaxisRotation::Tier3ThigmotaxisRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3VisualScanRotation::Tier3VisualScanRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3LeastAngle::Tier3LeastAngle (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3LeastAngleRotation::Tier3LeastAngleRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3Interpersonal::Tier3Interpersonal (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3InterpersonalRotation::Tier3InterpersonalRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
//Tier3Formation::Tier3Formation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
//Tier3FormationRotation::Tier3FormationRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3Front::Tier3Front (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3FrontRotation::Tier3FrontRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3Rear::Tier3Rear (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3RearRotation::Tier3RearRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3Side::Tier3Side (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3SideRotation::Tier3SideRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3Visible::Tier3Visible (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3VisibleRotation::Tier3VisibleRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
//Tier3Wait::Tier3Wait (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
//Tier3WaitRotation::Tier3WaitRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3CrowdAvoid::Tier3CrowdAvoid (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3CrowdAvoidRotation::Tier3CrowdAvoidRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3FindTheCrowd::Tier3FindTheCrowd (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3FindTheCrowdRotation::Tier3FindTheCrowdRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3RiskAvoid::Tier3RiskAvoid (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3RiskAvoidRotation::Tier3RiskAvoidRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3FindTheRisk::Tier3FindTheRisk (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3FindTheRiskRotation::Tier3FindTheRiskRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3FlowAvoid::Tier3FlowAvoid (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3FlowAvoidRotation::Tier3FlowAvoidRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3FindTheFlow::Tier3FindTheFlow (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3FindTheFlowRotation::Tier3FindTheFlowRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3Follow::Tier3Follow (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3FollowRotation::Tier3FollowRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3Crossroads::Tier3Crossroads (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3CrossroadsRotation::Tier3CrossroadsRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
Tier3Stay::Tier3Stay (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {};
Tier3StayRotation::Tier3StayRotation (Beliefs *beliefs, string name, string description, double weight, double *magic_init, bool is_active): Tier3Advisor(beliefs, name, description, weight, magic_init, is_active) {}; 
 


// Default dummy constructors
Tier3Greedy::Tier3Greedy(): Tier3Advisor() {};
Tier3ElbowRoom::Tier3ElbowRoom(): Tier3Advisor() {};
//Tier3CloseIn::Tier3CloseIn(): Tier3Advisor() {};
Tier3BigStep::Tier3BigStep(): Tier3Advisor() {};
Tier3Unlikely::Tier3Unlikely(): Tier3Advisor() {};
Tier3UnlikelyField::Tier3UnlikelyField(): Tier3Advisor() {};
Tier3Explorer::Tier3Explorer(): Tier3Advisor() {};
Tier3ExplorerEndPoints::Tier3ExplorerEndPoints(): Tier3Advisor() {};
//Tier3AvoidRobot::Tier3AvoidRobot(): Tier3Advisor() {};
Tier3BaseLine::Tier3BaseLine(): Tier3Advisor() {};

Tier3GreedyRotation::Tier3GreedyRotation(): Tier3Advisor() {};
Tier3ElbowRoomRotation::Tier3ElbowRoomRotation(): Tier3Advisor() {};
//Tier3CloseInRotation::Tier3CloseInRotation(): Tier3Advisor() {};
Tier3BigStepRotation::Tier3BigStepRotation(): Tier3Advisor() {};
Tier3GoAroundRotation::Tier3GoAroundRotation(): Tier3Advisor() {};
Tier3UnlikelyRotation::Tier3UnlikelyRotation(): Tier3Advisor() {};
Tier3UnlikelyFieldRotation::Tier3UnlikelyFieldRotation(): Tier3Advisor() {};
Tier3ExplorerEndPointsRotation::Tier3ExplorerEndPointsRotation(): Tier3Advisor() {};
//Tier3AvoidRobotRotation::Tier3AvoidRobotRotation(): Tier3Advisor() {};
Tier3BaseLineRotation::Tier3BaseLineRotation(): Tier3Advisor() {};

Tier3ExitLinear::Tier3ExitLinear(): Tier3Advisor() {};
Tier3ExitRotation::Tier3ExitRotation(): Tier3Advisor() {};
Tier3ExitFieldLinear::Tier3ExitFieldLinear(): Tier3Advisor() {};
Tier3ExitFieldRotation::Tier3ExitFieldRotation(): Tier3Advisor() {};
Tier3ExitClosest::Tier3ExitClosest(): Tier3Advisor() {};
Tier3ExitClosestRotation::Tier3ExitClosestRotation(): Tier3Advisor() {};
Tier3RegionLeaverLinear::Tier3RegionLeaverLinear(): Tier3Advisor() {};
Tier3RegionLeaverRotation::Tier3RegionLeaverRotation(): Tier3Advisor() {};
Tier3EnterLinear::Tier3EnterLinear(): Tier3Advisor() {};
Tier3EnterRotation::Tier3EnterRotation(): Tier3Advisor() {};
Tier3EnterExit::Tier3EnterExit(): Tier3Advisor() {};
Tier3EnterExitRotation::Tier3EnterExitRotation(): Tier3Advisor() {};
Tier3ConveyLinear::Tier3ConveyLinear(): Tier3Advisor() {};
Tier3ConveyRotation::Tier3ConveyRotation(): Tier3Advisor() {};
Tier3TrailerLinear::Tier3TrailerLinear(): Tier3Advisor() {};
Tier3TrailerRotation::Tier3TrailerRotation(): Tier3Advisor() {};
Tier3EnterDoorLinear::Tier3EnterDoorLinear(): Tier3Advisor() {};
Tier3EnterDoorRotation::Tier3EnterDoorRotation(): Tier3Advisor() {};
Tier3ExitDoorLinear::Tier3ExitDoorLinear(): Tier3Advisor() {};
Tier3ExitDoorRotation::Tier3ExitDoorRotation(): Tier3Advisor() {};
Tier3AccessLinear::Tier3AccessLinear(): Tier3Advisor() {};
Tier3AccessRotation::Tier3AccessRotation(): Tier3Advisor() {};
//Tier3NeighborDoorLinear::Tier3NeighborDoorLinear(): Tier3Advisor() {};
//Tier3NeighborDoorRotation::Tier3NeighborDoorRotation(): Tier3Advisor() {};
Tier3LearnSpatialModel::Tier3LearnSpatialModel(): Tier3Advisor() {};
Tier3LearnSpatialModelRotation::Tier3LearnSpatialModelRotation(): Tier3Advisor() {};
Tier3Curiosity::Tier3Curiosity(): Tier3Advisor() {};
Tier3CuriosityRotation::Tier3CuriosityRotation(): Tier3Advisor() {};
Tier3Enfilade::Tier3Enfilade(): Tier3Advisor() {};
Tier3EnfiladeRotation::Tier3EnfiladeRotation(): Tier3Advisor() {};
Tier3Thigmotaxis::Tier3Thigmotaxis(): Tier3Advisor() {};
Tier3ThigmotaxisRotation::Tier3ThigmotaxisRotation(): Tier3Advisor() {};
Tier3VisualScanRotation::Tier3VisualScanRotation(): Tier3Advisor() {};
Tier3LeastAngle::Tier3LeastAngle(): Tier3Advisor() {};
Tier3LeastAngleRotation::Tier3LeastAngleRotation(): Tier3Advisor() {};
Tier3Interpersonal::Tier3Interpersonal(): Tier3Advisor() {};
Tier3InterpersonalRotation::Tier3InterpersonalRotation(): Tier3Advisor() {};
//Tier3Formation::Tier3Formation(): Tier3Advisor() {};
//Tier3FormationRotation::Tier3FormationRotation(): Tier3Advisor() {};
Tier3Front::Tier3Front(): Tier3Advisor() {};
Tier3FrontRotation::Tier3FrontRotation(): Tier3Advisor() {};
Tier3Rear::Tier3Rear(): Tier3Advisor() {};
Tier3RearRotation::Tier3RearRotation(): Tier3Advisor() {};
Tier3Side::Tier3Side(): Tier3Advisor() {};
Tier3SideRotation::Tier3SideRotation(): Tier3Advisor() {};
Tier3Visible::Tier3Visible(): Tier3Advisor() {};
Tier3VisibleRotation::Tier3VisibleRotation(): Tier3Advisor() {};
//Tier3Wait::Tier3Wait(): Tier3Advisor() {};
//Tier3WaitRotation::Tier3WaitRotation(): Tier3Advisor() {};
Tier3CrowdAvoid::Tier3CrowdAvoid(): Tier3Advisor() {};
Tier3CrowdAvoidRotation::Tier3CrowdAvoidRotation(): Tier3Advisor() {};
Tier3FindTheCrowd::Tier3FindTheCrowd(): Tier3Advisor() {};
Tier3FindTheCrowdRotation::Tier3FindTheCrowdRotation(): Tier3Advisor() {};
Tier3RiskAvoid::Tier3RiskAvoid(): Tier3Advisor() {};
Tier3RiskAvoidRotation::Tier3RiskAvoidRotation(): Tier3Advisor() {};
Tier3FindTheRisk::Tier3FindTheRisk(): Tier3Advisor() {};
Tier3FindTheRiskRotation::Tier3FindTheRiskRotation(): Tier3Advisor() {};
Tier3FlowAvoid::Tier3FlowAvoid(): Tier3Advisor() {};
Tier3FlowAvoidRotation::Tier3FlowAvoidRotation(): Tier3Advisor() {};
Tier3FindTheFlow::Tier3FindTheFlow(): Tier3Advisor() {};
Tier3FindTheFlowRotation::Tier3FindTheFlowRotation(): Tier3Advisor() {};
Tier3Follow::Tier3Follow(): Tier3Advisor() {};
Tier3FollowRotation::Tier3FollowRotation(): Tier3Advisor() {};
Tier3Crossroads::Tier3Crossroads(): Tier3Advisor() {};
Tier3CrossroadsRotation::Tier3CrossroadsRotation(): Tier3Advisor() {};
Tier3Stay::Tier3Stay(): Tier3Advisor() {};
Tier3StayRotation::Tier3StayRotation(): Tier3Advisor() {};


// vote to go through an extrance to a region containing the target

double Tier3EnterLinear::actionComment(FORRAction action){
  // cout << "In enter linear " << endl;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  int targetRegion=-1;
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  double dist_to_target = expPosition.get_distance(targetPoint);
      
  vector<FORRExit> exits = regions[targetRegion].getExits();

  double metric = 1000;
  for(int i = 0; i < exits.size(); i++){
    double dist_to_exit = expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y());
    if(regions[targetRegion].inRegion(expPosition) == true){
      Circle region = Circle(regions[targetRegion].getCenter(), regions[targetRegion].getRadius());
      CartesianPoint intersectPoint = intersection_point(region, LineSegment(currentPosition, expPosition));
      double arc_length = acos(1 - ((intersectPoint.get_distance(exits[i].getExitPoint()) * intersectPoint.get_distance(exits[i].getExitPoint())) / (2 * regions[targetRegion].getRadius() * regions[targetRegion].getRadius()))) * regions[targetRegion].getRadius();
      if(arc_length <= 5 * M_PI * regions[targetRegion].getRadius() / 180){
        metric = dist_to_target;
        break;
      }
    }
    else{
      if(dist_to_target + dist_to_exit < metric){
        metric = dist_to_target + dist_to_exit;
      }
    }
  }
  if(doors.size() > 0){
    double comment_strength = 1000;
    // check if the expected position is in the target's region
    if(regions[targetRegion].inRegion(expPosition) == true and doors[targetRegion].size() > 0){
      Circle region = Circle(regions[targetRegion].getCenter(), regions[targetRegion].getRadius());
      CartesianPoint intersectPoint = intersection_point(region, LineSegment(currentPosition, expPosition));
      double intersectPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), intersectPoint.get_x(), intersectPoint.get_y());
      for(int i = 0; i < doors[targetRegion].size(); i++) {
        // check if the point that the robot crosses into the region goes through one of the doors
        double startPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), doors[targetRegion][i].startPoint.getExitPoint().get_x(), doors[targetRegion][i].startPoint.getExitPoint().get_y());
        double endPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), doors[targetRegion][i].endPoint.getExitPoint().get_x(), doors[targetRegion][i].endPoint.getExitPoint().get_y());
        if(intersectPointAngle <= endPointAngle and intersectPointAngle >= startPointAngle){
          comment_strength = expPosition.get_distance(targetPoint);
          break;
        }
      }
    }
    else{
      for(int i = 0; i < doors[targetRegion].size(); i++) {
        double dist_to_door = doors[targetRegion][i].distanceToDoor(expPosition, regions[targetRegion]);
        if(dist_to_target + dist_to_door < comment_strength){
          comment_strength = dist_to_target + dist_to_door;
        }
      }
    }
    // cout << "Metric " << metric << " Comment strength " << comment_strength << endl;
    if(metric < comment_strength){
      return -1 * metric;
    }
    else{
      return -1 * comment_strength;
    }
  }
  else{
    return -1 * metric;
  }
}


void Tier3EnterLinear::set_commenting(){
  // cout << "In enter linear set commenting " << endl;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool targetInRegionWithExit = false;
  bool targetInRegionWithDoor = false;
  bool robotRegTargetRegConnected = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
      if(doors.size() > 0){
        if(doors[i].size() >= 1){
          targetInRegionWithDoor = true;
        }
      }
      if((regions[i]).getExits().size() >= 1){
        targetInRegionWithExit = true;
      }
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }
  if(targetInRegionWithExit){
    for(int i = 0; i < (regions[targetRegion]).getExits().size(); i++){
      if(regions[targetRegion].getExits()[i].getExitRegion() == robotRegion){
        robotRegTargetRegConnected = true;
      }
    }
  }
  // cout << "Robot region " << robotRegion << " Target region " << targetRegion << " robotRegTargetRegConnected " << robotRegTargetRegConnected << " targetInRegionWithExit " << targetInRegionWithExit << " targetInRegionWithDoor " << targetInRegionWithDoor << endl;
  if(targetInRegion == true and robotRegion != targetRegion and robotRegTargetRegConnected == true and (targetInRegionWithExit == true or targetInRegionWithDoor == true))
    advisor_commenting = true;
  else
    advisor_commenting = false;
}


double Tier3EnterRotation::actionComment(FORRAction action){
  // cout << "In enter rotation " << endl;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  int targetRegion=-1;
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  double dist_to_target = expPosition.get_distance(targetPoint);
      
  vector<FORRExit> exits = regions[targetRegion].getExits();

  double metric = 1000;
  for(int i = 0; i < exits.size(); i++){
    double dist_to_exit = expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y());
    if(regions[targetRegion].inRegion(expPosition) == true){
      Circle region = Circle(regions[targetRegion].getCenter(), regions[targetRegion].getRadius());
      CartesianPoint intersectPoint = intersection_point(region, LineSegment(currentPosition, expPosition));
      double arc_length = acos(1 - ((intersectPoint.get_distance(exits[i].getExitPoint()) * intersectPoint.get_distance(exits[i].getExitPoint())) / (2 * regions[targetRegion].getRadius() * regions[targetRegion].getRadius()))) * regions[targetRegion].getRadius();
      if(arc_length <= 5 * M_PI * regions[targetRegion].getRadius() / 180){
        metric = dist_to_target;
        break;
      }
    }
    else{
      if(dist_to_target + dist_to_exit < metric){
        metric = dist_to_target + dist_to_exit;
      }
    }
  }
  if(doors.size() > 0){
    double comment_strength = 1000;
    // check if the expected position is in the target's region
    if(regions[targetRegion].inRegion(expPosition) == true and doors[targetRegion].size() > 0){
      Circle region = Circle(regions[targetRegion].getCenter(), regions[targetRegion].getRadius());
      CartesianPoint intersectPoint = intersection_point(region, LineSegment(currentPosition, expPosition));
      double intersectPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), intersectPoint.get_x(), intersectPoint.get_y());
      for(int i = 0; i < doors[targetRegion].size(); i++) {
        // check if the point that the robot crosses into the region goes through one of the doors
        double startPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), doors[targetRegion][i].startPoint.getExitPoint().get_x(), doors[targetRegion][i].startPoint.getExitPoint().get_y());
        double endPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), doors[targetRegion][i].endPoint.getExitPoint().get_x(), doors[targetRegion][i].endPoint.getExitPoint().get_y());
        if(intersectPointAngle <= endPointAngle and intersectPointAngle >= startPointAngle){
          comment_strength = expPosition.get_distance(targetPoint);
          break;
        }
      }
    }
    else{
      for(int i = 0; i < doors[targetRegion].size(); i++) {
        double dist_to_door = doors[targetRegion][i].distanceToDoor(expPosition, regions[targetRegion]);
        if(dist_to_target + dist_to_door < comment_strength){
          comment_strength = dist_to_target + dist_to_door;
        }
      }
    }
    // cout << "Metric " << metric << " Comment strength " << comment_strength << endl;
    if(metric < comment_strength){
      return -1 * metric;
    }
    else{
      return -1 * comment_strength;
    }
  }
  else{
    return -1 * metric;
  }
}

void Tier3EnterRotation::set_commenting(){
  // cout << "In enter rotation set commenting " << endl;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool targetInRegionWithExit = false;
  bool targetInRegionWithDoor = false;
  bool robotRegTargetRegConnected = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
      if(doors.size() > 0){
        if(doors[i].size() >= 1){
          targetInRegionWithDoor = true;
        }
      }
      if((regions[i]).getExits().size() >= 1){
        targetInRegionWithExit = true;
      }
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }
  if(targetInRegionWithExit){
    for(int i = 0; i < (regions[targetRegion]).getExits().size(); i++){
      if(regions[targetRegion].getExits()[i].getExitRegion() == robotRegion){
        robotRegTargetRegConnected = true;
      }
    }
  }
  // cout << "Robot region " << robotRegion << " Target region " << targetRegion << " robotRegTargetRegConnected " << robotRegTargetRegConnected << " targetInRegionWithExit " << targetInRegionWithExit << " targetInRegionWithDoor " << targetInRegionWithDoor << endl;
  if(targetInRegion == true and robotRegion != targetRegion and robotRegTargetRegConnected == true and (targetInRegionWithExit == true or targetInRegionWithDoor == true))
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3EnterExit::actionComment(FORRAction action){
  //cout << "In enter exit linear " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  int targetRegion=-1;
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
      
  vector<FORRExit> exits = regions[targetRegion].getExits();

  double metric = 1000;
  for(int i = 0; i < exits.size(); i++){
    if (expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()) < metric) {
      metric = expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y());
    }
  }
  return metric * (-1);
}


void Tier3EnterExit::set_commenting(){

  //cout << "In enter exit linear set commenting " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }

  if(targetInRegion == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}


double Tier3EnterExitRotation::actionComment(FORRAction action){
  //cout << "In enter exit rotation " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  int targetRegion=-1;
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
      
  vector<FORRExit> exits = regions[targetRegion].getExits();

  double metric = 1000;
  for(int i = 0; i < exits.size(); i++){
    if (expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()) < metric) {
      metric = expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y());
    }
  }
  return metric * (-1);
}

void Tier3EnterExitRotation::set_commenting(){

  //cout << "In region finder rotation set commenting " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }

  if(targetInRegion == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3ExitLinear::actionComment(FORRAction action){
  double result=0;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  
  int robotRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  //cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;
  vector<FORRExit> ext_exits = regions[robotRegion].getExtExits();

  for(int i = 0 ; i < ext_exits.size(); i++){
    result += expectedPosition.getDistance(ext_exits[i].getExitPoint().get_x(), ext_exits[i].getExitPoint().get_y());
  }

  return result*(-1);
}


void Tier3ExitLinear::set_commenting(){

  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}


double Tier3ExitRotation::actionComment(FORRAction action){
  double result=0;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  
  int robotRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }
  
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  //cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;
  vector<FORRExit> ext_exits = regions[robotRegion].getExtExits();

  for(int i = 0 ; i < ext_exits.size(); i++){
    result += expectedPosition.getDistance(ext_exits[i].getExitPoint().get_x(), ext_exits[i].getExitPoint().get_y());
  }

  return result*(-1);
}

void Tier3ExitRotation::set_commenting(){
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
  
}

double Tier3ExitFieldLinear::actionComment(FORRAction action){
  double result=0;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  
  int robotRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  //cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;
  vector<FORRExit> ext_exits = regions[robotRegion].getExtExits();

  for(int i = 0 ; i < ext_exits.size(); i++){
    result += (1 / expectedPosition.getDistance(ext_exits[i].getExitPoint().get_x(), ext_exits[i].getExitPoint().get_y()));
  }

  return result;
}


void Tier3ExitFieldLinear::set_commenting(){

  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}


double Tier3ExitFieldRotation::actionComment(FORRAction action){
  double result=0;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  
  int robotRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }
  
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  //cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;
  vector<FORRExit> ext_exits = regions[robotRegion].getExtExits();

  for(int i = 0 ; i < ext_exits.size(); i++){
    result += (1 / expectedPosition.getDistance(ext_exits[i].getExitPoint().get_x(), ext_exits[i].getExitPoint().get_y()));
  }

  return result;
}

void Tier3ExitFieldRotation::set_commenting(){
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
  
}

double Tier3ExitClosest::actionComment(FORRAction action){
  double result=1000;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  
  int robotRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  //cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;
  vector<FORRExit> exits = regions[robotRegion].getExits();

  for(int i = 0 ; i < exits.size(); i++){
    if (expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()) < result)
      result = expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y());
  }

  return -result;
}


void Tier3ExitClosest::set_commenting(){

  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}


double Tier3ExitClosestRotation::actionComment(FORRAction action){
  double result=1000;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  
  int robotRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  //cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;
  vector<FORRExit> exits = regions[robotRegion].getExits();

  for(int i = 0 ; i < exits.size(); i++){
    if (expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y()) < result)
      result = expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y());
  }

  return -result;
}

void Tier3ExitClosestRotation::set_commenting(){
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
  
}

double Tier3RegionLeaverLinear::actionComment(FORRAction action){
  // cout << "In region leaver " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  
  int robotRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }
  double robotRegionRadius = regions[robotRegion].getRadius();

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  
  // cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << " " << regions[robotRegion].getRadius() << endl;

  vector<FORRExit> exits = regions[robotRegion].getExits();
  double result = - 1000;
  for(int i = 0 ; i < exits.size(); i++){
    double expDistToExit = expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y());
    double expDistToRegion = expectedPosition.getDistance(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y());
    double value = expDistToRegion - robotRegionRadius - expDistToExit;
    // cout << "Exit " << exits[i].getExitPoint().get_x() << " " << exits[i].getExitPoint().get_y() << " expDistToExit " << expDistToExit << " expDistToRegion " << expDistToRegion << " value " << value << endl;
    if(value > result)
      result = value;
  }

  vector< vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  if(doors.size() > 0){
    for(int i = 0; i < doors[robotRegion].size(); i++) {
      double expDistToDoor = doors[robotRegion][i].distanceToDoor(expPosition, regions[robotRegion]);
      double expDistToRegion = expectedPosition.getDistance(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y());
      double value = expDistToRegion - robotRegionRadius - expDistToDoor;
      // cout << "Door " << i << " expDistToDoor " << expDistToDoor << " expDistToRegion " << expDistToRegion << " value " << value << endl;
      if(value > result){
        result = value;
      }
    }
  }
  // cout << "Action " << action.type << " " << action.parameter << " Result " << result << endl;
  return result;
}


void Tier3RegionLeaverLinear::set_commenting(){
  // cout << "In region leaver set commenting " << endl;
  vector< vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  bool currPosInRegionWithDoor = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
      if((regions[i]).getExits().size() >= 1){
        currPosInRegionWithExit = true;
      }
      if(doors.size() > 0){
        if(doors[i].size() >= 1){
          currPosInRegionWithDoor = true;
        }
      }
    }
  }
  // cout << "Robot region " << robotRegion << " Target region " << targetRegion << " Robot region with exit " << currPosInRegionWithExit << " Robot region with door " << currPosInRegionWithDoor << endl;
  if((currPosInRegionWithExit == true or currPosInRegionWithDoor == true) and robotRegion != targetRegion){
    advisor_commenting = true;
  }
  else{
    advisor_commenting = false;
  }
}


double Tier3RegionLeaverRotation::actionComment(FORRAction action){
  // cout << "In region leaver rotation " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  
  int robotRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
 
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }
  double robotRegionRadius = regions[robotRegion].getRadius();

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  
  // cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << " " << regions[robotRegion].getRadius() << endl;

  vector<FORRExit> exits = regions[robotRegion].getExits();
  double result = - 1000;
  for(int i = 0 ; i < exits.size(); i++){
    double expDistToExit = expectedPosition.getDistance(exits[i].getExitPoint().get_x(), exits[i].getExitPoint().get_y());
    double expDistToRegion = expectedPosition.getDistance(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y());
    double value = expDistToRegion - robotRegionRadius - expDistToExit;
    // cout << "Exit " << exits[i].getExitPoint().get_x() << " " << exits[i].getExitPoint().get_y() << " expDistToExit " << expDistToExit << " expDistToRegion " << expDistToRegion << " value " << value << endl;
    if(value > result)
      result = value;
  }

  vector< vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  if(doors.size() > 0){
    for(int i = 0; i < doors[robotRegion].size(); i++) {
      double expDistToDoor = doors[robotRegion][i].distanceToDoor(expPosition, regions[robotRegion]);
      double expDistToRegion = expectedPosition.getDistance(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y());
      double value = expDistToRegion - robotRegionRadius - expDistToDoor;
      // cout << "Door " << i << " expDistToDoor " << expDistToDoor << " expDistToRegion " << expDistToRegion << " value " << value << endl;
      if(value > result){
        result = value;
      }
    }
  }
  // cout << "Action " << action.type << " " << action.parameter << " Result " << result << endl;
  return result;
}

void Tier3RegionLeaverRotation::set_commenting(){
  // cout << "In region leaver rotation set commenting " << endl;
  vector< vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  bool currPosInRegionWithDoor = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
      if((regions[i]).getExits().size() >= 1){
        currPosInRegionWithExit = true;
      }
      if(doors.size() > 0){
        if(doors[i].size() >= 1){
          currPosInRegionWithDoor = true;
        }
      }
    }
  }
  // cout << "Robot region " << robotRegion << " Target region " << targetRegion << " Robot region with exit " << currPosInRegionWithExit << " Robot region with door " << currPosInRegionWithDoor << endl;
  if((currPosInRegionWithExit == true or currPosInRegionWithDoor == true) and robotRegion != targetRegion){
    advisor_commenting = true;
    // cout << "advisor_commenting " << advisor_commenting << endl;
  }
  else{
    advisor_commenting = false;
    // cout << "advisor_commenting " << advisor_commenting << endl;
  }
  // cout << "done with set_commenting" << endl;
}
/*
//CloseIn : When target is nearby, distance is within 80 units, go towards it!
double Tier3CloseIn::actionComment(FORRAction action){
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  double task_x = beliefs->getAgentState()->getCurrentTask()->getX();
  double task_y = beliefs->getAgentState()->getCurrentTask()->getY();

  double newDistance = expectedPosition.getDistance(task_x, task_y);
  
  return newDistance *(-1);
}


// this advisor is active only when robot is 80px away from the target
void Tier3CloseIn::set_commenting(){
  double distanceToTarget = beliefs->getAgentState()->getDistanceToTarget();
  if(distanceToTarget > 80)
    advisor_commenting = false;
  else
    advisor_commenting = true;
}


double Tier3CloseInRotation::actionComment(FORRAction action){
  double newDistance;

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  double task_x = beliefs->getAgentState()->getCurrentTask()->getX();
  double task_y = beliefs->getAgentState()->getCurrentTask()->getY();

  newDistance = expectedPosition.getDistance(task_x, task_y);

  return newDistance*(-1);
}


void Tier3CloseInRotation::set_commenting(){
  double distanceToTarget = beliefs->getAgentState()->getDistanceToTarget();
  if(distanceToTarget > 80)
    advisor_commenting = false;
  else
    advisor_commenting = true;
}
*/
/*
double Tier3AvoidRobot::actionComment(FORRAction action){
  double result;
  //max_len is used as range
  double range = 100;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double sumOfTeammateDistances = beliefs->getTeamState()->getSumOfTeammateDistances(expectedPosition, range);  
  return sumOfTeammateDistances;
}

// On when there are more than one robots in the field
void Tier3AvoidRobot::set_commenting(){
  if((beliefs->beliefs->getTeamState()->getTeamPose()).size() <= 1)
    advisor_commenting = false;
  else
    advisor_commenting = true;
}

double Tier3AvoidRobotRotation::actionComment(FORRAction action){
  double result;
  double range = 100;
  vector<FORRAction> actionList;
  actionList.push_back(action);
  FORRAction max_forward_move = FORRAction(FORWARD,5);
  actionList.push_back(max_forward_move);
  
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterActions(actionList);
  double sumOfTeammateDistances = beliefs->getTeamState()->getSumOfTeammateDistances(expectedPosition, range);
  return sumOfTeammateDistances;
}

void Tier3AvoidRobotRotation::set_commenting(){
  if((beliefs->getTeamState()->getTeamPose()).size() <= 1)
    advisor_commenting = false;
  else
    advisor_commenting = true;
}
*/

//Wants to make moves that keep the robot as far as possible from the obstacles
double Tier3ElbowRoom::actionComment(FORRAction action){

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double distanceToObstacle = beliefs->getAgentState()->getDistanceToNearestObstacle(expectedPosition);
  return distanceToObstacle;
}

// always on
void Tier3ElbowRoom::set_commenting(){
  advisor_commenting = true;
}

double Tier3ElbowRoomRotation::actionComment(FORRAction action){

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double obstacleDistance = beliefs->getAgentState()->getDistanceToNearestObstacle(expectedPosition);
  
  return obstacleDistance;
}

void Tier3ElbowRoomRotation::set_commenting(){
  advisor_commenting = true;
}


double Tier3Greedy::actionComment(FORRAction action){
  
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  double task_x = beliefs->getAgentState()->getCurrentTask()->getX();
  double task_y = beliefs->getAgentState()->getCurrentTask()->getY();

  double newDistance = expectedPosition.getDistance(task_x, task_y);
  
  return newDistance *(-1);
}

// always on
void Tier3Greedy::set_commenting(){
  advisor_commenting = true;
}

double Tier3GreedyRotation::actionComment(FORRAction action){
  double newDistance;

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  double task_x = beliefs->getAgentState()->getCurrentTask()->getX();
  double task_y = beliefs->getAgentState()->getCurrentTask()->getY();

  newDistance = expectedPosition.getDistance(task_x, task_y);

  return newDistance*(-1);
}

void Tier3GreedyRotation::set_commenting(){
  advisor_commenting = true;
}

// Comment strength function for BigStep advisor
double Tier3BigStep::actionComment(FORRAction action){
   
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  double cur_x = beliefs->getAgentState()->getCurrentPosition().getX();
  double cur_y = beliefs->getAgentState()->getCurrentPosition().getY();

  double distanceFromCurrentPosition = expectedPosition.getDistance(cur_x,cur_y);
  return distanceFromCurrentPosition;
}

// commenting only when robot is sufficiently far from the obstacle
void Tier3BigStep::set_commenting(){
  
  advisor_commenting = true;
  
}

double Tier3BigStepRotation::actionComment(FORRAction action){

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  double cur_x = beliefs->getAgentState()->getCurrentPosition().getX();
  double cur_y = beliefs->getAgentState()->getCurrentPosition().getY();

  double distanceFromCurrentPosition = expectedPosition.getDistance(cur_x,cur_y);
  return distanceFromCurrentPosition;
}

void Tier3BigStepRotation::set_commenting(){
  
  advisor_commenting = true;
  
}


// always on
void Tier3Unlikely::set_commenting(){
  //cout << "In avoid leaf set commenting " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }

  if(currPosInRegionWithExit == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3Unlikely::actionComment(FORRAction action){
  //cout << "In Avoid leaf " << endl;
  double result;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  int robotRegion=-1,targetRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }
  
  vector<FORRExit> exits = regions[robotRegion].getExits();
  std::vector<Door> robotRegionDoors = doors[robotRegion];
  //cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;

  vector<FORRRegion> nearRegions;
  for(int i = 0; i < exits.size() ; i++){
    FORRRegion test = regions[exits[i].getExitRegion()];
    std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
    if(nearRegions.empty() or (it == nearRegions.end())){
      nearRegions.push_back(test);
      //cout << "Neighbour Region : " << test.getCenter().get_x() << " " << test.getCenter().get_y() << endl;
    }
  }

  //cout << "#neighbours found" << nearRegions.size() << endl; 

  double metric = 0;
  for(int i = 0; i < nearRegions.size(); i++){
    if(beliefs->getSpatialModel()->getRegionList()->isLeaf(nearRegions[i], robotRegionDoors.size()) and !(nearRegions[i].inRegion(targetPoint.get_x(), targetPoint.get_y()))){
      //cout << "Avoid Leaf: Found deadend !" << endl;
      metric += expectedPosition.getDistance(nearRegions[i].getCenter().get_x(), nearRegions[i].getCenter().get_y());
    }
  }

  return metric;
}

void Tier3UnlikelyRotation::set_commenting(){
  //cout << "In region finder rotation set commenting " << endl;
   vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
   Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
   Task *task = beliefs->getAgentState()->getCurrentTask();
   CartesianPoint targetPoint (task->getX() , task->getY());
   CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
   bool targetInRegion = false;
   bool currPosInRegionWithExit = false;
   int robotRegion=-1, targetRegion = -1;
   
   // check the preconditions for activating the advisor
   for(int i = 0; i < regions.size() ; i++){
     // check if the target point is in region
     if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
       targetInRegion = true;
       targetRegion = i;
     }
     // check if the rob_pos is in a region and the region has atleast one exit
     if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
       currPosInRegionWithExit = true;
       robotRegion = i;
     }
   }
   
   if(currPosInRegionWithExit == true and robotRegion != targetRegion)
     advisor_commenting = true;
   else
     advisor_commenting = false;
}

double Tier3UnlikelyRotation::actionComment(FORRAction action){
  //cout << "In Avoid leaf Rotation" << endl;
  double result;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  int robotRegion=-1,targetRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }
  
  vector<FORRExit> exits = regions[robotRegion].getExits();
  std::vector<Door> robotRegionDoors = doors[robotRegion];
  //cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;

  vector<FORRRegion> nearRegions;
  for(int i = 0; i < exits.size() ; i++){
    FORRRegion test = regions[exits[i].getExitRegion()];
    std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
    if(nearRegions.empty() or (it == nearRegions.end())){
      nearRegions.push_back(test);
      //cout << "Neighbour Region : " << test.getCenter().get_x() << " " << test.getCenter().get_y() << endl;
    }
  }

  //cout << "#neighbours found" << nearRegions.size() << endl; 

  double metric = 0;
  for(int i = 0; i < nearRegions.size(); i++){
    if(beliefs->getSpatialModel()->getRegionList()->isLeaf(nearRegions[i], robotRegionDoors.size()) and !(nearRegions[i].inRegion(targetPoint.get_x(), targetPoint.get_y()))){
      //cout << "Avoid leaf Rotation: Found deadend !" << endl;
      metric += expectedPosition.getDistance(nearRegions[i].getCenter().get_x(), nearRegions[i].getCenter().get_y());
    }
  }

  return metric;
}

void Tier3UnlikelyField::set_commenting(){
  //cout << "In avoid leaf set commenting " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and regions[i].getExits().size() >= 1){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }

  if(currPosInRegionWithExit == true and robotRegion != targetRegion){
    advisor_commenting = true;
  }
  else{
    advisor_commenting = false;
  }
}

double Tier3UnlikelyField::actionComment(FORRAction action){
  //cout << "In Avoid leaf " << endl;
  double result;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  int robotRegion=-1,targetRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }
  
  vector<FORRExit> exits = regions[robotRegion].getExits();
  int numDoors = 0;
  if(doors.size() > 0){
    std::vector<Door> robotRegionDoors = doors[robotRegion];
    numDoors = robotRegionDoors.size();
  }
  //cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;

  vector<FORRRegion> nearRegions;
  for(int i = 0; i < exits.size() ; i++){
    FORRRegion test = regions[exits[i].getExitRegion()];
    std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
    if(nearRegions.empty() or (it == nearRegions.end())){
      nearRegions.push_back(test);
      //cout << "Neighbour Region : " << test.getCenter().get_x() << " " << test.getCenter().get_y() << endl;
    }
  }

  //cout << "#neighbours found" << nearRegions.size() << endl; 

  double metric = 0;
  for(int i = 0; i < nearRegions.size(); i++){
    if(beliefs->getSpatialModel()->getRegionList()->isLeaf(nearRegions[i], numDoors) and !(nearRegions[i].inRegion(targetPoint.get_x(), targetPoint.get_y()))){
      //cout << "Avoid Leaf: Found deadend !" << endl;
      metric += 1/(abs((expectedPosition.getDistance(nearRegions[i].getCenter().get_x(), nearRegions[i].getCenter().get_y())) - nearRegions[i].getRadius()));
    }
  }

  return metric * (-1);
}

void Tier3UnlikelyFieldRotation::set_commenting(){
  // cout << "In UnlikelyFieldRotation set commenting " << endl;
   vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
   Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
   Task *task = beliefs->getAgentState()->getCurrentTask();
   CartesianPoint targetPoint (task->getX() , task->getY());
   CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
   bool targetInRegion = false;
   bool currPosInRegionWithExit = false;
   int robotRegion=-1, targetRegion = -1;
   
   // check the preconditions for activating the advisor
   for(int i = 0; i < regions.size() ; i++){
     // check if the target point is in region
     if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
       targetInRegion = true;
       targetRegion = i;
     }
     // check if the rob_pos is in a region and the region has atleast one exit
     if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and regions[i].getExits().size() >= 1){
       currPosInRegionWithExit = true;
       robotRegion = i;
     }
   }
   // cout << "Robot region " << robotRegion << " Target region " << targetRegion << " Robot region with exit " << currPosInRegionWithExit << endl;
   if(currPosInRegionWithExit == true and robotRegion != targetRegion){
     advisor_commenting = true;
     // cout << "advisor_commenting " << advisor_commenting << endl;
   }
   else{
     advisor_commenting = false;
     // cout << "advisor_commenting " << advisor_commenting << endl;
   }
   // cout << "done with set_commenting" << endl;
}

double Tier3UnlikelyFieldRotation::actionComment(FORRAction action){
  // cout << "In UnlikelyFieldRotation" << endl;
  double result;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  int robotRegion=-1,targetRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }
  
  vector<FORRExit> exits = regions[robotRegion].getExits();
  int numDoors = 0;
  if(doors.size() > 0){
    std::vector<Door> robotRegionDoors = doors[robotRegion];
    numDoors = robotRegionDoors.size();
  }
  // cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << endl;

  vector<FORRRegion> nearRegions;
  for(int i = 0; i < exits.size() ; i++){
    FORRRegion test = regions[exits[i].getExitRegion()];
    std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
    if(nearRegions.empty() or (it == nearRegions.end())){
      nearRegions.push_back(test);
      // cout << "Neighbour Region : " << test.getCenter().get_x() << " " << test.getCenter().get_y() << endl;
    }
  }

  // cout << "#neighbours found" << nearRegions.size() << endl; 

  double metric = 0;
  for(int i = 0; i < nearRegions.size(); i++){
    if(beliefs->getSpatialModel()->getRegionList()->isLeaf(nearRegions[i], numDoors) and !(nearRegions[i].inRegion(targetPoint.get_x(), targetPoint.get_y()))){
      // cout << "Avoid leaf Rotation: Found deadend !" << endl;
      metric += 1/(abs((expectedPosition.getDistance(nearRegions[i].getCenter().get_x(), nearRegions[i].getCenter().get_y())) - nearRegions[i].getRadius()));
    }
  }

  return metric * (-1);
}

// always on
void Tier3Explorer::set_commenting(){
  advisor_commenting = true;
}

double Tier3Explorer::actionComment(FORRAction action){
 
  vector<Position> *positionHis = beliefs->getAgentState()->getCurrentTask()->getPositionHistory();
 
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  int beta = 0;
  double totalForce = 0, distance = 0; 

  for(int i = 0; i < positionHis->size(); i++){
    distance = expectedPosition.getDistance((*positionHis)[i]);
    if(distance < 1)     distance = 1;
    totalForce += (1/distance);
  }
  return totalForce * (-1);
  
}

void Tier3ExplorerEndPoints::set_commenting(){
  advisor_commenting = true;
}

double Tier3ExplorerEndPoints::actionComment(FORRAction action){
  vector< vector <CartesianPoint> > *laserHis = beliefs->getAgentState()->getCurrentTask()->getLaserHistory();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  int beta = 0;
  double totalForce = 0, distance = 0; 

  for(int i = 0; i < laserHis->size(); i++){
  	//cout << laserHis->size() << endl;
  	for (int j = 0; j< ((*laserHis)[i]).size(); j++) {
  		//cout << ((*laserHis)[i]).size() << endl;
  		distance = expectedPosition.getDistance(((*laserHis)[i])[j].get_x(), ((*laserHis)[i])[j].get_y());
    	if(distance < 1)     distance = 1;
    	if(distance < 25) {
        totalForce += (1/distance);
      }
  	}
  }
  return totalForce * (-1);
}

double Tier3BaseLine::actionComment(FORRAction action){
  //srand (time(NULL));
  //cout << "Baseline :::::::::::::::::::::::::::::::::::::::::::::::::::::::" << rand()%10 - 5 << endl;
  // cout << "Baseline" << endl;
  return rand()%10 - 5;
}

void Tier3BaseLine::set_commenting(){
  //srand (time(NULL));
  // if(rand()%2 == 0)
  //   advisor_commenting = true;
  // else
  //   advisor_commenting = false;
  advisor_commenting = true;
}

void Tier3ExplorerRotation::set_commenting(){
  advisor_commenting = true;
}

double Tier3ExplorerRotation::actionComment(FORRAction action){
 
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);

  vector<Position> *positionHis = beliefs->getAgentState()->getCurrentTask()->getPositionHistory();
  int beta = 0;
  double totalForce = 0, distance = 0;
 
  for(int i = 0; i < positionHis->size(); i++){
    distance = expectedPosition.getDistance((*positionHis)[i]);
    if(distance < 1)     distance = 1;
    totalForce += (1/distance);
  }
  return totalForce * (-1);
}

void Tier3ExplorerEndPointsRotation::set_commenting(){
  advisor_commenting = true;
}

double Tier3ExplorerEndPointsRotation::actionComment(FORRAction action){
  vector< vector <CartesianPoint> > *laserHis = beliefs->getAgentState()->getCurrentTask()->getLaserHistory();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  int beta = 0;
  double totalForce = 0, distance = 0; 

  for(int i = 0; i < laserHis->size(); i++){
  	//cout << laserHis->size() << endl;
  	for (int j = 0; j< ((*laserHis)[i]).size(); j++) {
  		//cout << ((*laserHis)[i]).size() << endl;
  		distance = expectedPosition.getDistance(((*laserHis)[i])[j].get_x(), ((*laserHis)[i])[j].get_y());
    	if(distance < 1)     distance = 1;
      if(distance < 25) {
        totalForce += (1/distance);
      }
  	}
  }
  return totalForce * (-1);
}

double Tier3BaseLineRotation::actionComment(FORRAction action){
  //srand (time(NULL));
  // cout << "BaselineRotation" << endl;
  return rand()%10 - 5;
}

void Tier3BaseLineRotation::set_commenting(){
  //srand (time(NULL));
  // if(rand()%2 == 0)
  //   advisor_commenting = true;
  // else
  //   advisor_commenting = false;
  advisor_commenting = true;
}



//votes for moves that will take it to an adjacent cell block, given by the overlay grid, 
//that has the most points in it.  
//The metric is the distance to the expected point times the grid value, because we want to pull
//the robot further distances, but at the same time want it to go to high-value grid cells
double Tier3ConveyLinear::actionComment(FORRAction action){
 
  //cout << "Entered Convey linear."<<endl;
  Position cur_pos = beliefs->getAgentState()->getCurrentPosition();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  
  int grid_value = beliefs->getSpatialModel()->getConveyors()->getGridValue(expectedPosition.getX(), expectedPosition.getY());
  
  double distance = cur_pos.getDistance(expectedPosition.getX(), expectedPosition.getY());
  
  //cout <<"Exit conveyor linear."<<endl;
  return distance * grid_value; //want larger grid values that are further away
}


void Tier3ConveyLinear::set_commenting(){
  set<int> grid_values;
  cout << "set commenting for conveylinear" << endl;
  set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
  cout << "after vetoactions" << endl;
  set<FORRAction> *action_set;
  action_set = beliefs->getAgentState()->getActionSet();
  cout << "after actionset" << endl;
  FORRAction forrAction;
  set<FORRAction>::iterator actionIter;
  for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
    forrAction = *actionIter;
    //cout << forrAction.type << " " << forrAction.parameter << endl;
    if(vetoed_actions->find(forrAction) != vetoed_actions->end())// is this action vetoed
      continue;
    Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
    cout << "after get expected position" << expectedPosition.getX() << " " << expectedPosition.getY() << endl;
    int grid_value = beliefs->getSpatialModel()->getConveyors()->getGridValue(expectedPosition.getX(), expectedPosition.getY());
    cout << "after get grid value" << endl;
    if(grid_value > 1)
      grid_values.insert(grid_value);
  }
  if(grid_values.size() <= 1){
    advisor_commenting = false;
  }
  else{
    advisor_commenting = true;
  }
}

double Tier3ConveyRotation::actionComment(FORRAction action){
  //cout <<" Entered Convey rotation." << endl;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  //cout <<" Expected position after action: " <<expectedPosition.getX() << " " << expectedPosition.getY() << endl;
  int grid_value = beliefs->getSpatialModel()->getConveyors()->getGridValue(expectedPosition.getX(), expectedPosition.getY());
  //cout << "grid value: "<<grid_value<<endl;

  Position cur_pos = beliefs->getAgentState()->getCurrentPosition();
  double distance = cur_pos.getDistance(expectedPosition.getX(), expectedPosition.getY());  
  return distance * grid_value;
}

void Tier3ConveyRotation::set_commenting(){
  set<int> grid_values;
  set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
  set<FORRAction> *action_set;
  action_set = beliefs->getAgentState()->getActionSet();
  FORRAction forrAction;
  set<FORRAction>::iterator actionIter;
  for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
    forrAction = *actionIter;
    //cout << forrAction.type << " " << forrAction.parameter << endl;
    if(vetoed_actions->find(forrAction) != vetoed_actions->end())// is this action vetoed
      continue;
    Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
    int grid_value = beliefs->getSpatialModel()->getConveyors()->getGridValue(expectedPosition.getX(), expectedPosition.getY());
    if(grid_value > 1)
      grid_values.insert(grid_value);
  }
  if(grid_values.size() <= 1){
    advisor_commenting = false;
  }
  else{
    advisor_commenting = true;
  }
}


double Tier3TrailerLinear::actionComment(FORRAction action){
   CartesianPoint target_trailmarker = beliefs->getSpatialModel()->getTrails()->getFurthestVisiblePointOnChosenTrail(beliefs->getAgentState());  
   //if out of line of sight of trail marker, vote the same value functionally turning off voting
   if(!beliefs->getSpatialModel()->getTrails()->canSeeTrail()){
     return 0;
   }

  Position target(target_trailmarker.get_x(),target_trailmarker.get_y(),0);

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double newDistance = expectedPosition.getDistance(target);
  //cout << (-1)* newDistance << endl;
  return newDistance *(-1); 
}

//only turn on if a trail is active
void Tier3TrailerLinear::set_commenting(){
  // Searches for a trail if not found
  beliefs->getSpatialModel()->getTrails()->findNearbyTrail(beliefs->getAgentState());
  if(beliefs->getSpatialModel()->getTrails()->getChosenTrail() == -1){
    advisor_commenting = false;
  }
  else{
	// If the advisor is commenting that means a trail has been choosen and advisor will stick with it till the end of task
      advisor_commenting = true;
  }
}

double Tier3TrailerRotation::actionComment(FORRAction action){
   CartesianPoint target_trailmarker = beliefs->getSpatialModel()->getTrails()->getFurthestVisiblePointOnChosenTrail(beliefs->getAgentState());  
   //if out of line of sight of trail marker, vote the same value in effect turning off voting
  //can_see_trail is set in getFurthestVisiblePointOnChosenTrail
   if(!beliefs->getSpatialModel()->getTrails()->canSeeTrail()){
     return 0;
   }

  Position target(target_trailmarker.get_x(),target_trailmarker.get_y(),0);

  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double newDistance = expectedPosition.getDistance(target);
  return newDistance *(-1); 
}


//only turn on if a trail is active
void Tier3TrailerRotation::set_commenting(){
  // Searches for a trail if not choosen yet
  beliefs->getSpatialModel()->getTrails()->findNearbyTrail(beliefs->getAgentState());
  if(beliefs->getSpatialModel()->getTrails()->getChosenTrail() == -1){
    advisor_commenting = false;
  }
  else{
    // If the advisor is commenting that means a trail has been choosen and advisor will stick with it till the end of task 
    advisor_commenting = true;
  }
}

double Tier3GoAroundRotation::actionComment(FORRAction action){
  // break up action into its components
  FORRActionType actionType = action.type;
  if(actionType == FORWARD){
    return 0;
  }
  int intensity = action.parameter;
  double comment_strength;
  double avgRightDistanceVector = 0, avgLeftDistanceVector = 0;
  sensor_msgs::msg::LaserScan laserScan = beliefs->getAgentState()->getCurrentLaserScan();
  // compute forward distance to obstacle
  double centerDistanceVector = ( laserScan.ranges[((laserScan.ranges.size()/2)-1)] + laserScan.ranges[((laserScan.ranges.size()/2))] ) / 2;
  if(centerDistanceVector < 0.1){
    centerDistanceVector = 0.1;
  }
  // find average length of distance vectors on right and left sides
  for(int i = 0; i < ((laserScan.ranges.size()/2)); i++){
    double length = laserScan.ranges[i];
    avgRightDistanceVector += length;
  }
  for(int i = (laserScan.ranges.size()/2); i < (laserScan.ranges.size()-1); i++){
    double length = laserScan.ranges[i];
    avgLeftDistanceVector += length;
  }

  avgRightDistanceVector = avgRightDistanceVector / (laserScan.ranges.size()/2);
  avgLeftDistanceVector = avgLeftDistanceVector / (laserScan.ranges.size()/2);
  // Comment strength is computed as the intensity of the rotation divided by the distance to the forward obstacle
  if(avgRightDistanceVector > avgLeftDistanceVector && actionType == RIGHT_TURN) {
    comment_strength = (intensity / centerDistanceVector);
  }
  else if(avgRightDistanceVector < avgLeftDistanceVector && actionType == RIGHT_TURN) {
    comment_strength = -1 * (intensity / centerDistanceVector);
  }
  else if(avgRightDistanceVector > avgLeftDistanceVector && actionType == LEFT_TURN) {
    comment_strength = -1 * (intensity / centerDistanceVector);
  }
  else if(avgRightDistanceVector < avgLeftDistanceVector && actionType == LEFT_TURN) {
    comment_strength = (intensity / centerDistanceVector);
  }
  else if(avgRightDistanceVector = avgLeftDistanceVector) {
    comment_strength = (intensity / centerDistanceVector);
  }

  return comment_strength;
}
  
void Tier3GoAroundRotation::set_commenting(){
  advisor_commenting = true;
}

double Tier3EnterDoorLinear::actionComment(FORRAction action){
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  int targetRegion = -1;
  double comment_strength = 0;

  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
  }
  // check if the expected position is in the target's region
  if(regions[targetRegion].inRegion(expPosition) == true){
    Circle region = Circle(regions[targetRegion].getCenter(), regions[targetRegion].getRadius());
    CartesianPoint intersectPoint = intersection_point(region, LineSegment(currentPosition, expPosition));
    double intersectPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), intersectPoint.get_x(), intersectPoint.get_y());
    for(int i = 0; i < doors[targetRegion].size(); i++) {
      // check if the point that the robot crosses into the region goes through one of the doors
      double startPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), doors[targetRegion][i].startPoint.getExitPoint().get_x(), doors[targetRegion][i].startPoint.getExitPoint().get_y());
      double endPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), doors[targetRegion][i].endPoint.getExitPoint().get_x(), doors[targetRegion][i].endPoint.getExitPoint().get_y());
      if(intersectPointAngle <= endPointAngle and intersectPointAngle >= startPointAngle){
        comment_strength = -expPosition.get_distance(targetPoint);
      }
    }
  }

  return comment_strength;
}

void Tier3EnterDoorLinear::set_commenting(){
  //cout << "In enter door linear set commenting " << endl;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegionWithDoor = false;
  bool currPosInTargetRegion = false;
  int targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y()) and (doors[i].size() >= 1)){
      targetInRegionWithDoor = true;
      targetRegion = i;
      currPosInTargetRegion = regions[targetRegion].inRegion(curr_pos.getX(), curr_pos.getY());
    }
  }

  if(targetInRegionWithDoor == true and currPosInTargetRegion == false)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3EnterDoorRotation::actionComment(FORRAction action){
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  int targetRegion = -1;
  double comment_strength = 0;

  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
  }
  // check if the expected position is in the target's region
  if(regions[targetRegion].inRegion(expPosition) == true){
    Circle region = Circle(regions[targetRegion].getCenter(), regions[targetRegion].getRadius());
    CartesianPoint intersectPoint = intersection_point(region, LineSegment(currentPosition, expPosition));
    double intersectPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), intersectPoint.get_x(), intersectPoint.get_y());
    for(int i = 0; i < doors[targetRegion].size(); i++) {
      // check if the point that the robot crosses into the region goes through one of the doors
      double startPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), doors[targetRegion][i].startPoint.getExitPoint().get_x(), doors[targetRegion][i].startPoint.getExitPoint().get_y());
      double endPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[targetRegion].getCenter().get_x(), regions[targetRegion].getCenter().get_y(), doors[targetRegion][i].endPoint.getExitPoint().get_x(), doors[targetRegion][i].endPoint.getExitPoint().get_y());
      if(intersectPointAngle <= endPointAngle and intersectPointAngle >= startPointAngle){
        comment_strength = -expPosition.get_distance(targetPoint);
      }
    }
  }

  return comment_strength;
}

void Tier3EnterDoorRotation::set_commenting(){
  //cout << "In enter door rotation set commenting " << endl;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegionWithDoor = false;
  bool currPosInTargetRegion = false;
  int targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y()) and (doors[i].size() >= 1)){
      targetInRegionWithDoor = true;
      targetRegion = i;
      currPosInTargetRegion = regions[targetRegion].inRegion(curr_pos.getX(), curr_pos.getY());
    }
  }

  if(targetInRegionWithDoor == true and currPosInTargetRegion == false)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3ExitDoorLinear::actionComment(FORRAction action){
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  double comment_strength = 0;
  int robotRegion=-1;
 
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }

  //cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << " " << regions[robotRegion].getRadius() << endl;

  double min_distance = 1000;
  for(int i = 0; i < doors[robotRegion].size(); i++) {
  	double doorDistance = doors[robotRegion][i].distanceToDoor(expPosition, regions[robotRegion]);
    if (doorDistance < min_distance){
      min_distance = doorDistance;
      comment_strength = ((double)(doors[robotRegion][i].str)) * (1 / doorDistance);
    }
  }

  // check if the robot gets out of the region
  /*if(expectedPosition.getDistance(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y()) > regions[robotRegion].getRadius()) {
    Circle region = Circle(regions[robotRegion].getCenter(), regions[robotRegion].getRadius());
    CartesianPoint intersectPoint = intersection_point(region, LineSegment(currentPosition, expPosition));
    double intersectPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), intersectPoint.get_x(), intersectPoint.get_y());
    for(int i = 0; i < doors[robotRegion].size(); i++) {
      // check if the point that the robot crosses out of the region goes through one of the doors
      double startPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), doors[robotRegion][i].startPoint.getExitPoint().get_x(), doors[robotRegion][i].startPoint.getExitPoint().get_y());
      double endPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), doors[robotRegion][i].endPoint.getExitPoint().get_x(), doors[robotRegion][i].endPoint.getExitPoint().get_y());
      if(intersectPointAngle <= endPointAngle and intersectPointAngle >= startPointAngle){
        comment_strength = ((double)(doors[robotRegion][i].str)) * (expectedPosition.getDistance(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y()) - regions[robotRegion].getRadius());
      }
    }
  }*/
  return comment_strength;
}

void Tier3ExitDoorLinear::set_commenting(){
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithDoor = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and (doors[i].size() >= 1)){
      currPosInRegionWithDoor = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithDoor == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3ExitDoorRotation::actionComment(FORRAction action){
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  double comment_strength = 0;
  int robotRegion=-1;
 
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY())){
      robotRegion = i;
    }
  }

  //cout << "Robot Region : " << regions[robotRegion].getCenter().get_x() << " " << regions[robotRegion].getCenter().get_y() << " " << regions[robotRegion].getRadius() << endl;

  double min_distance = 1000;
  for(int i = 0; i < doors[robotRegion].size(); i++) {
  	double doorDistance = doors[robotRegion][i].distanceToDoor(expPosition, regions[robotRegion]);
    if (doorDistance < min_distance){
      min_distance = doorDistance;
      comment_strength = ((double)(doors[robotRegion][i].str)) * (1 / doorDistance);
    }
  }

  // check if the robot gets out of the region
  /*if(expectedPosition.getDistance(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y()) > regions[robotRegion].getRadius()) {
    Circle region = Circle(regions[robotRegion].getCenter(), regions[robotRegion].getRadius());
    CartesianPoint intersectPoint = intersection_point(region, LineSegment(currentPosition, expPosition));
    double intersectPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), intersectPoint.get_x(), intersectPoint.get_y());
    for(int i = 0; i < doors[robotRegion].size(); i++) {
      // check if the point that the robot crosses out of the region goes through one of the doors
      double startPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), doors[robotRegion][i].startPoint.getExitPoint().get_x(), doors[robotRegion][i].startPoint.getExitPoint().get_y());
      double endPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), doors[robotRegion][i].endPoint.getExitPoint().get_x(), doors[robotRegion][i].endPoint.getExitPoint().get_y());
      if(intersectPointAngle <= endPointAngle and intersectPointAngle >= startPointAngle){
        comment_strength = ((double)(doors[robotRegion][i].str)) * (expectedPosition.getDistance(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y()) - regions[robotRegion].getRadius());
      }
    }
  }*/
  return comment_strength;
}

void Tier3ExitDoorRotation::set_commenting(){
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithDoor = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and (doors[i].size() >= 1)){
      currPosInRegionWithDoor = true;
      robotRegion = i;
    }
  }
  if(currPosInRegionWithDoor == true and robotRegion != targetRegion)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3AccessLinear::actionComment(FORRAction action){
  double result=0;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());

  for(int i = 0; i < doors.size() ; i++){
    double dist = (expPosition.get_distance(regions[i].getCenter()) - regions[i].getRadius());
    if(dist > 1)
      result += ((1 / dist ) * doors[i].size());
    else
      result += doors[i].size();
  }

  return result;
}

void Tier3AccessLinear::set_commenting(){
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  bool atLeastOneDoor = false;
  for(int i = 0; i < doors.size(); i++){
    if(doors[i].size() >= 1){
      atLeastOneDoor = true;
      break;
    }
  }
  if(atLeastOneDoor == true) 
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3AccessRotation::actionComment(FORRAction action){
  double result=0;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());

  for(int i = 0; i < doors.size() ; i++){
    double dist = (expPosition.get_distance(regions[i].getCenter()) - regions[i].getRadius());
    if(dist > 1)
      result += ((1 / dist ) * doors[i].size());
    else
      result += doors[i].size();
  }

  return result;
}

void Tier3AccessRotation::set_commenting(){
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  bool atLeastOneDoor = false;
  for(int i = 0; i < doors.size(); i++){
    if(doors[i].size() >= 1){
      atLeastOneDoor = true;
      break;
    }
  }
  if(atLeastOneDoor == true) 
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

/*double Tier3NeighborDoorLinear::actionComment(FORRAction action){
  double result=0;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  int targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
  }

  vector<FORRExit> exits = regions[targetRegion].getExits();
  vector<FORRRegion> nearRegions;
  for(int i = 0; i < exits.size() ; i++){
    FORRRegion test = regions[exits[i].getExitRegion()];
    std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
    if(nearRegions.empty() or (it == nearRegions.end())){
      nearRegions.push_back(test);
      result += ((1 / expPosition.get_distance(regions[exits[i].getExitRegion()].getCenter())) * doors[exits[i].getExitRegion()].size());
    }
  }

  return result;
}

void Tier3NeighborDoorLinear::set_commenting(){
  cout << "In neighbor door linear set commenting " << endl;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool targetHasNeighbors = false;
  int targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
  }
  if(targetInRegion == true) {
    vector<FORRExit> exits = regions[targetRegion].getExits();
    vector<FORRRegion> nearRegions;
    for(int i = 0; i < exits.size() ; i++){
      FORRRegion test = regions[exits[i].getExitRegion()];
      std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
      if(nearRegions.empty() or (it == nearRegions.end())){
        nearRegions.push_back(test);
        cout << "Neighbour Region : " << test.getCenter().get_x() << " " << test.getCenter().get_y() << endl;
      }
    }
    cout << "#Neighbors found :" << nearRegions.size() << endl;
    if(nearRegions.size() >= 1) {
      targetHasNeighbors = true;
    }
  }

  if(targetInRegion == true and targetHasNeighbors == true)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3NeighborDoorRotation::actionComment(FORRAction action){
  double result=0;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  int targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetRegion = i;
    }
  }

  vector<FORRExit> exits = regions[targetRegion].getExits();
  vector<FORRRegion> nearRegions;
  for(int i = 0; i < exits.size() ; i++){
    FORRRegion test = regions[exits[i].getExitRegion()];
    std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
    if(nearRegions.empty() or (it == nearRegions.end())){
      nearRegions.push_back(test);
      result += ((1 / expPosition.get_distance(regions[exits[i].getExitRegion()].getCenter())) * doors[exits[i].getExitRegion()].size());
    }
  }

  return result;
}

void Tier3NeighborDoorRotation::set_commenting(){
  cout << "In neighbor door linear set commenting " << endl;
  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool targetHasNeighbors = false;
  int targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
  }
  if(targetInRegion == true) {
    vector<FORRExit> exits = regions[targetRegion].getExits();
    vector<FORRRegion> nearRegions;
    for(int i = 0; i < exits.size() ; i++){
      FORRRegion test = regions[exits[i].getExitRegion()];
      std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
      if(nearRegions.empty() or (it == nearRegions.end())){
        nearRegions.push_back(test);
        cout << "Neighbour Region : " << test.getCenter().get_x() << " " << test.getCenter().get_y() << endl;
      }
    }
    cout << "#Neighbors found :" << nearRegions.size() << endl;
    if(nearRegions.size() >= 1) {
      targetHasNeighbors = true;
    }
  }

  if(targetInRegion == true and targetHasNeighbors == true)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}*/

void Tier3LearnSpatialModel::set_commenting(){
  advisor_commenting = true;
}

double Tier3LearnSpatialModel::actionComment(FORRAction action){
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  double result;
  int robotRegion = -1;

  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  for(int i = 0; i < regions.size() ; i++){
    // check if the expected position is in region
    if(regions[i].inRegion(expPosition.get_x(), expPosition.get_y())){
      robotRegion = i;
      break;
    }
  }

  vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
  bool expPosInHallway = false;
  for(int i = 0; i < hallways.size(); i++){
    if(hallways[i].pointInAggregate(expPosition)){
      expPosInHallway = true;
      break;
    }
  }
  if(robotRegion > (-1) or expPosInHallway == true){
    result = -1.0 * (beliefs->getSpatialModel()->getConveyors()->getMaxGridValue());
    //cout << "LearnSpatialModel INSIDE REGION or HALLWAY : " << result << endl;
  }
  else {
    result = -(beliefs->getSpatialModel()->getConveyors()->getAverageGridValue(expectedPosition.getX(), expectedPosition.getY()));
    //cout << "LearnSpatialModel OUTSIDE REGION or HALLWAY : " << result << endl;
  }
  return result;
}

void Tier3LearnSpatialModelRotation::set_commenting(){
  advisor_commenting = true;
}

double Tier3LearnSpatialModelRotation::actionComment(FORRAction action){
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  double result;
  int robotRegion = -1;

  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  for(int i = 0; i < regions.size() ; i++){
    // check if the expected position is in region
    if(regions[i].inRegion(expPosition.get_x(), expPosition.get_y())){
      robotRegion = i;
      break;
    }
  }

  vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
  bool expPosInHallway = false;
  for(int i = 0; i < hallways.size(); i++){
    if(hallways[i].pointInAggregate(expPosition)){
      expPosInHallway = true;
      break;
    }
  }
  if(robotRegion > (-1) or expPosInHallway == true){
    result = -1.0 * (beliefs->getSpatialModel()->getConveyors()->getMaxGridValue());
    //cout << "LearnSpatialModelRotation INSIDE REGION or HALLWAY : " << result << endl;
  }
  else {
    result = -(beliefs->getSpatialModel()->getConveyors()->getAverageGridValue(expectedPosition.getX(), expectedPosition.getY()));
    //cout << "LearnSpatialModelRotation OUTSIDE REGION or HALLWAY : " << result << endl;
  }
  return result;
}

void Tier3Curiosity::set_commenting(){
  advisor_commenting = true;
}

double Tier3Curiosity::actionComment(FORRAction action){
  vector< vector<CartesianPoint> > all_trace = beliefs->getAgentState()->getAllTrace();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  double totalForce = 0, distance = 0; 

  for(int i = 0; i < all_trace.size(); i++){
    for(int j = 0; j < all_trace[i].size(); j++) {
      distance = expPosition.get_distance(all_trace[i][j]);
      if(distance < 1)     distance = 1;
      totalForce += (1/distance);
    }
  }

  vector<Position> *positionHis = beliefs->getAgentState()->getCurrentTask()->getPositionHistory();
  for(int i = 0; i < positionHis->size(); i++){
    distance = expectedPosition.getDistance((*positionHis)[i]);
    if(distance < 1)     distance = 1;
    totalForce += (1/distance);
  }

  return totalForce * (-1);
}

void Tier3CuriosityRotation::set_commenting(){
  advisor_commenting = true;
}

double Tier3CuriosityRotation::actionComment(FORRAction action){
  vector< vector<CartesianPoint> > all_trace = beliefs->getAgentState()->getAllTrace();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  double totalForce = 0, distance = 0; 

  for(int i = 0; i < all_trace.size(); i++){
    for(int j = 0; j < all_trace[i].size(); j++) {
      distance = expPosition.get_distance(all_trace[i][j]);
      if(distance < 1)     distance = 1;
      totalForce += (1/distance);
    }
  }

  vector<Position> *positionHis = beliefs->getAgentState()->getCurrentTask()->getPositionHistory();
  for(int i = 0; i < positionHis->size(); i++){
    distance = expectedPosition.getDistance((*positionHis)[i]);
    if(distance < 1)     distance = 1;
    totalForce += (1/distance);
  }

  return totalForce * (-1);
}

void Tier3Enfilade::set_commenting(){
  vector<Position> *positionHis = beliefs->getAgentState()->getCurrentTask()->getPositionHistory();
  if(positionHis->size() > 1){
    advisor_commenting = true;
  }
  else{
    advisor_commenting = false;
  }
}

double Tier3Enfilade::actionComment(FORRAction action){
  //cout << "Inside Enfilade" << endl;
  vector<Position> *positionHis = beliefs->getAgentState()->getCurrentTask()->getPositionHistory();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double totalForce = 0, distance = 0;
  int startPosition = positionHis->size()-1;
  vector<Position> uniquePositions;
  uniquePositions.push_back((*positionHis)[startPosition]);
  while(uniquePositions.size() < 10 and startPosition >= 0){
    if((*positionHis)[startPosition].getDistance(uniquePositions[0]) >= 0.1){
      uniquePositions.insert(uniquePositions.begin(), (*positionHis)[startPosition]);
    }
    startPosition = startPosition - 1;
  }
  for(int i = 0; i < uniquePositions.size(); i++){
    distance = expectedPosition.getDistance(uniquePositions[i]);
    //cout << "distance = " << distance << endl;
    totalForce += distance;
  }

  // int startPosition = 0;
  // if (positionHis->size() > 10){
  //   startPosition = positionHis->size() - 10;
  // }
  // //cout << "startPosition = " << startPosition << " currentPosition = " << positionHis->size() << endl;
  // for(int i = startPosition; i < positionHis->size(); i++){
  //   distance = expectedPosition.getDistance((*positionHis)[i]);
  //   //cout << "distance = " << distance << endl;
  //   totalForce += distance;
  // }
  //cout << "totalForce = " << totalForce << endl;
  return totalForce * (-1);
}

void Tier3EnfiladeRotation::set_commenting(){
  vector<Position> *positionHis = beliefs->getAgentState()->getCurrentTask()->getPositionHistory();
  if(positionHis->size() > 1){
    advisor_commenting = true;
  }
  else{
    advisor_commenting = false;
  }
}

double Tier3EnfiladeRotation::actionComment(FORRAction action){
  //cout << "Inside EnfiladeRotation" << endl;
  vector<Position> *positionHis = beliefs->getAgentState()->getCurrentTask()->getPositionHistory();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double totalForce = 0, distance = 0; 
  int startPosition = positionHis->size()-1;
  vector<Position> uniquePositions;
  uniquePositions.push_back((*positionHis)[startPosition]);
  while(uniquePositions.size() < 10 and startPosition >= 0){
    if((*positionHis)[startPosition].getDistance(uniquePositions[0]) >= 0.1){
      uniquePositions.insert(uniquePositions.begin(), (*positionHis)[startPosition]);
    }
    startPosition = startPosition - 1;
  }
  for(int i = 0; i < uniquePositions.size(); i++){
    distance = expectedPosition.getDistance(uniquePositions[i]);
    //cout << "distance = " << distance << endl;
    totalForce += distance;
  }

  // int startPosition = 0;
  // if (positionHis->size() > 10){
  //   startPosition = positionHis->size() - 10;
  // }
  // //cout << "startPosition = " << startPosition << " currentPosition = " << positionHis->size() << endl;
  // for(int i = startPosition; i < positionHis->size(); i++){
  //   distance = expectedPosition.getDistance((*positionHis)[i]);
  //   //cout << "distance = " << distance << endl;
  //   totalForce += distance;
  // }
  //cout << "totalForce = " << totalForce << endl;
  return totalForce * (-1);
}

void Tier3Thigmotaxis::set_commenting(){
  advisor_commenting = true;
}

double Tier3Thigmotaxis::actionComment(FORRAction action){
  //cout << "Inside Thigmotaxis" << endl;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  Position nearestObstacle = beliefs->getAgentState()->getNearestObstacle(curr_pos);
  double distanceToObstacle = expectedPosition.getDistance(nearestObstacle);
  //cout << "distanceToObstacle = " << distanceToObstacle << endl;
  return distanceToObstacle * (-1);
}

void Tier3ThigmotaxisRotation::set_commenting(){
  advisor_commenting = true;
}

double Tier3ThigmotaxisRotation::actionComment(FORRAction action){
  //cout << "Inside ThigmotaxisRotation" << endl;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  Position nearestObstacle = beliefs->getAgentState()->getNearestObstacle(curr_pos);
  double distanceToObstacle = expectedPosition.getDistance(nearestObstacle);
  //cout << "distanceToObstacle = " << distanceToObstacle << endl;
  return distanceToObstacle * (-1);
}

void Tier3VisualScanRotation::set_commenting(){
  advisor_commenting = true;
}

double Tier3VisualScanRotation::actionComment(FORRAction action){
  //cout << "Inside VisualScanRotation" << endl;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  vector<Position> nearby_points;
  nearby_points.push_back(curr_pos);
  vector<Position> *positionHis = beliefs->getAgentState()->getCurrentTask()->getPositionHistory();

  for(int i = 0; i < positionHis->size(); i++){
    double distance = curr_pos.getDistance((*positionHis)[i]);
    if(distance <= 1){
      nearby_points.push_back((*positionHis)[i]);
    }
  }
  //cout << "nearby_points.size() = " << nearby_points.size() << endl;
  double totalX=0, totalY=0;
  for(int i = 0; i < nearby_points.size(); i++){
    totalX += cos(nearby_points[i].getTheta());
    totalY += sin(nearby_points[i].getTheta());
  }
  //cout << "totalX = " << totalX << " totalY = " << totalY << endl;
  double totalVectorAngle = atan2(totalY,totalX);
  //cout << "totalVectorAngle = " << totalVectorAngle << endl;
  double desiredangle = 0;
  if(totalVectorAngle>0){
    desiredangle = -(M_PI-totalVectorAngle);
  }
  else if(totalVectorAngle<0){
    desiredangle = M_PI+totalVectorAngle;
  }
  else if(totalVectorAngle==0){
    desiredangle = M_PI;
  }
  double angleDiff = min(abs(desiredangle - expectedPosition.getTheta()),(2*M_PI) - abs(desiredangle - expectedPosition.getTheta()));
  //cout << "desiredangle = " << desiredangle << " angleDiff = " << angleDiff << endl;
  return angleDiff * (-1);
}

void Tier3LeastAngle::set_commenting(){
  //cout << "In least angle set commenting " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }

  if(currPosInRegionWithExit == true and ((targetInRegion == true and robotRegion != targetRegion) or (targetInRegion == false)))
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3LeastAngle::actionComment(FORRAction action){
  //cout << "Inside LeastAngle" << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  int robotRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and (regions[i]).getExits().size() >= 1){
      robotRegion = i;
    }
  }
  //cout << "robotRegion = " << robotRegion << endl;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  vector<FORRExit> exits = regions[robotRegion].getExits();

  vector<FORRRegion> nearRegions;
  for(int i = 0; i < exits.size() ; i++){
    FORRRegion test = regions[exits[i].getExitRegion()];
    std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
    if(nearRegions.empty() or (it == nearRegions.end())){
      nearRegions.push_back(test);
    }
  }
  //cout << "nearRegions.size() = " << nearRegions.size() << endl;
  double target_direction = atan2((curr_pos.getY() - targetPoint.get_y()), (curr_pos.getX() - targetPoint.get_x()));
  //cout << "target_direction = " << target_direction << endl;
  double minAngleDiff = 0.2618, metric = 1000;
  for(int i = 0; i < nearRegions.size(); i++){
    double region_direction = atan2((curr_pos.getY() - nearRegions[i].getCenter().get_y()), (curr_pos.getX() - nearRegions[i].getCenter().get_x()));
    double angle_diff = region_direction - target_direction;
    if(angle_diff > M_PI)
      angle_diff = angle_diff - 2*M_PI;
    if(angle_diff < -M_PI)
      angle_diff = angle_diff + 2*M_PI;
    angle_diff = fabs(angle_diff);

    //cout << "region_direction = " << region_direction << " angle_diff = " << angle_diff << endl;
    if(angle_diff < minAngleDiff){
      minAngleDiff = angle_diff;
      metric = expectedPosition.getDistance(nearRegions[i].getCenter().get_x(), nearRegions[i].getCenter().get_y()) + nearRegions[i].getCenter().get_distance(targetPoint);
      //cout << "minAngleDiff = " << minAngleDiff << " metric = " << metric << endl;
    }
  }

  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  double comment_strength = 1000;
  if(doors.size() > 0){
    // check if the expected position is in the target's region
    Circle region = Circle(regions[robotRegion].getCenter(), regions[robotRegion].getRadius());
    CartesianPoint intersectPoint = intersection_point(region, LineSegment(CartesianPoint(curr_pos.getX(), curr_pos.getY()), CartesianPoint(expectedPosition.getX(), expectedPosition.getY())));
    double intersectPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), intersectPoint.get_x(), intersectPoint.get_y());
    double targetPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), targetPoint.get_x(), targetPoint.get_y());
    for(int i = 0; i < doors[robotRegion].size(); i++) {
      // check if the point that the robot crosses into the region goes through one of the doors
      double startPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), doors[robotRegion][i].startPoint.getExitPoint().get_x(), doors[robotRegion][i].startPoint.getExitPoint().get_y());
      double endPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), doors[robotRegion][i].endPoint.getExitPoint().get_x(), doors[robotRegion][i].endPoint.getExitPoint().get_y());
      if(intersectPointAngle <= endPointAngle and intersectPointAngle >= startPointAngle and targetPointAngle <= endPointAngle and targetPointAngle >= startPointAngle){
        comment_strength = expectedPosition.getDistance(Position(targetPoint.get_x(), targetPoint.get_y(), 0));
      }
    }
  }
  if(metric < comment_strength){
    return metric * (-1);
  }
  else{
    return comment_strength * (-1);
  }
}

void Tier3LeastAngleRotation::set_commenting(){
  //cout << "In least angle set commenting " << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  CartesianPoint currentPosition (curr_pos.getX(), curr_pos.getY());
  bool targetInRegion = false;
  bool currPosInRegionWithExit = false;
  int robotRegion=-1, targetRegion = -1;
  
  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the target point is in region
    if(regions[i].inRegion(targetPoint.get_x(), targetPoint.get_y())){
      targetInRegion = true;
      targetRegion = i;
    }
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and ((regions[i]).getExits().size() >= 1)){
      currPosInRegionWithExit = true;
      robotRegion = i;
    }
  }

  if(currPosInRegionWithExit == true and ((targetInRegion == true and robotRegion != targetRegion) or (targetInRegion == false)))
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3LeastAngleRotation::actionComment(FORRAction action){
  //cout << "Inside LeastAngleRotation" << endl;
  vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
  int robotRegion=-1, desiredRegion=-1;
  Position curr_pos = beliefs->getAgentState()->getCurrentPosition();
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());

  // check the preconditions for activating the advisor
  for(int i = 0; i < regions.size() ; i++){
    // check if the rob_pos is in a region and the region has atleast one exit
    if(regions[i].inRegion(curr_pos.getX(), curr_pos.getY()) and (regions[i]).getExits().size() >= 1){
      robotRegion = i;
    }
  }
  //cout << "robotRegion = " << robotRegion << endl;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  vector<FORRExit> exits = regions[robotRegion].getExits();

  vector<FORRRegion> nearRegions;
  for(int i = 0; i < exits.size() ; i++){
    FORRRegion test = regions[exits[i].getExitRegion()];
    std::vector<FORRRegion>::iterator it = std::find(nearRegions.begin(),nearRegions.end(), test);
    if(nearRegions.empty() or (it == nearRegions.end())){
      nearRegions.push_back(test);
    }
  }
  //cout << "nearRegions.size() = " << nearRegions.size() << endl;
  double target_direction = atan2((curr_pos.getY() - targetPoint.get_y()), (curr_pos.getX() - targetPoint.get_x()));
  //cout << "target_direction = " << target_direction << endl;
  double minAngleDiff = 0.2618, metric = 1000;
  for(int i = 0; i < nearRegions.size(); i++){
    double region_direction = atan2((curr_pos.getY() - nearRegions[i].getCenter().get_y()), (curr_pos.getX() - nearRegions[i].getCenter().get_x()));
    double angle_diff = region_direction - target_direction;
    if(angle_diff > M_PI)
      angle_diff = angle_diff - 2*M_PI;
    if(angle_diff < -M_PI)
      angle_diff = angle_diff + 2*M_PI;
    angle_diff = fabs(angle_diff);

    //cout << "region_direction = " << region_direction << " angle_diff = " << angle_diff << endl;
    if(angle_diff < minAngleDiff){
      minAngleDiff = angle_diff;
      metric = expectedPosition.getDistance(nearRegions[i].getCenter().get_x(), nearRegions[i].getCenter().get_y()) + nearRegions[i].getCenter().get_distance(targetPoint);
      //cout << "minAngleDiff = " << minAngleDiff << " metric = " << metric << endl;
    }
  }

  std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
  double comment_strength = 1000;
  if(doors.size() > 0){
    // check if the expected position is in the target's region
    Circle region = Circle(regions[robotRegion].getCenter(), regions[robotRegion].getRadius());
    CartesianPoint intersectPoint = intersection_point(region, LineSegment(CartesianPoint(curr_pos.getX(), curr_pos.getY()), CartesianPoint(expectedPosition.getX(), expectedPosition.getY())));
    double intersectPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), intersectPoint.get_x(), intersectPoint.get_y());
    double targetPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), targetPoint.get_x(), targetPoint.get_y());
    for(int i = 0; i < doors[robotRegion].size(); i++) {
      // check if the point that the robot crosses into the region goes through one of the doors
      double startPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), doors[robotRegion][i].startPoint.getExitPoint().get_x(), doors[robotRegion][i].startPoint.getExitPoint().get_y());
      double endPointAngle = beliefs->getSpatialModel()->getDoors()->calculateFixedAngle(regions[robotRegion].getCenter().get_x(), regions[robotRegion].getCenter().get_y(), doors[robotRegion][i].endPoint.getExitPoint().get_x(), doors[robotRegion][i].endPoint.getExitPoint().get_y());
      if(intersectPointAngle <= endPointAngle and intersectPointAngle >= startPointAngle and targetPointAngle <= endPointAngle and targetPointAngle >= startPointAngle){
        comment_strength = expectedPosition.getDistance(Position(targetPoint.get_x(), targetPoint.get_y(), 0));
      }
    }
  }
  if(metric < comment_strength){
    return metric * (-1);
  }
  else{
    return comment_strength * (-1);
  }
}

void Tier3Interpersonal::set_commenting(){
  //cout << "In Interpersonal set commenting " << endl;
  if(beliefs->getAgentState()->getCrowdPose().poses.size() > 0)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3Interpersonal::actionComment(FORRAction action){
  //cout << "Inside Interpersonal" << endl;
  vector <Position> crowdPositions = beliefs->getAgentState()->getCrowdPositions(beliefs->getAgentState()->getCrowdPose());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double metric = 0;
  for(int i = 0; i < crowdPositions.size(); i++){
    double distanceToPedestrian = expectedPosition.getDistance(crowdPositions[i]);
    //cout << "distance to pedestrian = " << distanceToPedestrian << endl;
    if(distanceToPedestrian<=3.6){
      metric += -log(-distanceToPedestrian+4.22)+0.44;
      //cout << "metric = " << metric << endl;
    }
    else{
      metric += 1;
      //cout << "metric = " << metric << endl;
    }
  }
  return metric;
}

void Tier3InterpersonalRotation::set_commenting(){
  //cout << "In InterpersonalRotation set commenting " << endl;
  if(beliefs->getAgentState()->getCrowdPose().poses.size() > 0)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3InterpersonalRotation::actionComment(FORRAction action){
  //cout << "Inside InterpersonalRotation" << endl;
  vector <Position> crowdPositions = beliefs->getAgentState()->getCrowdPositions(beliefs->getAgentState()->getCrowdPose());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double metric = 0;
  for(int i = 0; i < crowdPositions.size(); i++){
    double distanceToPedestrian = expectedPosition.getDistance(crowdPositions[i]);
    //cout << "distance to pedestrian = " << distanceToPedestrian << endl;
    if(distanceToPedestrian<=3.6){
      metric += -log(-distanceToPedestrian+4.22)+0.44;
      //cout << "metric = " << metric << endl;
    }
    else{
      metric += 1;
      //cout << "metric = " << metric << endl;
    }
  }
  return metric;
}

/*void Tier3Formation::set_commenting(){
  cout << "In Interpersonal set commenting " << endl;
  if(beliefs->getAgentState()->getCrowdPose().poses.size() > 0)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3Formation::actionComment(FORRAction action){
  cout << "Inside Interpersonal" << endl;
  vector <Position> crowdPositions = beliefs->getAgentState()->getCrowdPositions(beliefs->getAgentState()->getCrowdPose());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double metric = 0;
  for(int i = 0; i < crowdPositions.size(); i++){
    double distanceToPedestrian = expectedPosition.getDistance(crowdPositions[i]);
    cout << "distance to pedestrian = " << distanceToPedestrian << endl;
    if(distanceToPedestrian<=3.6){
      metric += -log(-distanceToPedestrian+4.22)+0.44;
      cout << "metric = " << metric << endl;
    }
  }
  return metric;
}

void Tier3FormationRotation::set_commenting(){
  cout << "In InterpersonalRotation set commenting " << endl;
  if(beliefs->getAgentState()->getCrowdPose().poses.size() > 0)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3FormationRotation::actionComment(FORRAction action){
  cout << "Inside InterpersonalRotation" << endl;
  vector <Position> crowdPositions = beliefs->getAgentState()->getCrowdPositions(beliefs->getAgentState()->getCrowdPose());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double metric = 0;
  for(int i = 0; i < crowdPositions.size(); i++){
    double distanceToPedestrian = expectedPosition.getDistance(crowdPositions[i]);
    cout << "distance to pedestrian = " << distanceToPedestrian << endl;
    if(distanceToPedestrian<=3.6){
      metric += -log(-distanceToPedestrian+4.22)+0.44;
      cout << "metric = " << metric << endl;
    }
  }
  return metric;
}*/

void Tier3Front::set_commenting(){
  //cout << "In Front set commenting " << endl;
  if(beliefs->getAgentState()->getCrowdPose().poses.size() > 0)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3Front::actionComment(FORRAction action){
  //cout << "Inside Front" << endl;
  vector <Position> crowdPositions = beliefs->getAgentState()->getCrowdPositions(beliefs->getAgentState()->getCrowdPose());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
  double facingAngle = 0;
  if(expectedPosition.getTheta()>0){
    facingAngle = -(M_PI-expectedPosition.getTheta());
  }
  else if(expectedPosition.getTheta()<0){
    facingAngle = M_PI+expectedPosition.getTheta();
  }
  else if(expectedPosition.getTheta()==0){
    facingAngle = M_PI;
  }
  //cout << "expected Theta = " << expectedPosition.getTheta() << " facingAngle = " << facingAngle << endl;
  double metric = 0;
  for(int i = 0; i < crowdPositions.size(); i++){
    double pedestrianTheta = crowdPositions[i].getTheta();
    double angleDiff = min(abs(pedestrianTheta - facingAngle),(2*M_PI) - abs(pedestrianTheta - facingAngle));
    //cout << "pedestrianTheta = " << pedestrianTheta << " angleDiff = " << angleDiff << endl;
    double currentDistPedestrian = currentPosition.getDistance(crowdPositions[i]);
    double expectedDistPedestrian = expectedPosition.getDistance(crowdPositions[i]);
    double changeDist = expectedDistPedestrian - currentDistPedestrian;
    //cout << "currentDistPedestrian = " << currentDistPedestrian << " expectedDistPedestrian = " << expectedDistPedestrian << " changeDist = " << changeDist << endl;
    if(angleDiff<=M_PI/4 and changeDist < 0){
      metric += changeDist;
      //cout << "metric = " << metric << endl;
    }
  }
  return metric;
}

void Tier3FrontRotation::set_commenting(){
  //cout << "In FrontRotation set commenting " << endl;
  if(beliefs->getAgentState()->getCrowdPose().poses.size() > 0)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3FrontRotation::actionComment(FORRAction action){
  //cout << "Inside FrontRotation" << endl;
  vector <Position> crowdPositions = beliefs->getAgentState()->getCrowdPositions(beliefs->getAgentState()->getCrowdPose());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
  double facingAngle = 0;
  if(expectedPosition.getTheta()>0){
    facingAngle = -(M_PI-expectedPosition.getTheta());
  }
  else if(expectedPosition.getTheta()<0){
    facingAngle = M_PI+expectedPosition.getTheta();
  }
  else if(expectedPosition.getTheta()==0){
    facingAngle = M_PI;
  }
  //cout << "expected Theta = " << expectedPosition.getTheta() << " facingAngle = " << facingAngle << endl;
  double metric = 0;
  for(int i = 0; i < crowdPositions.size(); i++){
    double pedestrianTheta = crowdPositions[i].getTheta();
    double angleDiff = min(abs(pedestrianTheta - facingAngle),(2*M_PI) - abs(pedestrianTheta - facingAngle));
    //cout << "pedestrianTheta = " << pedestrianTheta << " angleDiff = " << angleDiff << endl;
    double currentDistPedestrian = currentPosition.getDistance(crowdPositions[i]);
    double expectedDistPedestrian = expectedPosition.getDistance(crowdPositions[i]);
    double changeDist = expectedDistPedestrian - currentDistPedestrian;
    //cout << "currentDistPedestrian = " << currentDistPedestrian << " expectedDistPedestrian = " << expectedDistPedestrian << " changeDist = " << changeDist << endl;
    if(angleDiff<=M_PI/4 and changeDist < 0){
      metric += changeDist;
      //cout << "metric = " << metric << endl;
    }
  }
  return metric;
}

void Tier3Rear::set_commenting(){
  //cout << "In Rear set commenting " << endl;
  if(beliefs->getAgentState()->getCrowdPose().poses.size() > 0)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3Rear::actionComment(FORRAction action){
  //cout << "Inside Rear" << endl;
  vector <Position> crowdPositions = beliefs->getAgentState()->getCrowdPositions(beliefs->getAgentState()->getCrowdPose());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
  double facingAngle = currentPosition.getTheta();
  //cout << "facingAngle = " << facingAngle << endl;
  double metric = 0;
  for(int i = 0; i < crowdPositions.size(); i++){
    double pedestrianTheta = crowdPositions[i].getTheta();
    double angleDiff = min(abs(pedestrianTheta - facingAngle),(2*M_PI) - abs(pedestrianTheta - facingAngle));
    //cout << "pedestrianTheta = " << pedestrianTheta << " angleDiff = " << angleDiff << endl;
    double currentDistPedestrian = currentPosition.getDistance(crowdPositions[i]);
    double expectedDistPedestrian = expectedPosition.getDistance(crowdPositions[i]);
    double changeDist = expectedDistPedestrian - currentDistPedestrian;
    //cout << "currentDistPedestrian = " << currentDistPedestrian << " expectedDistPedestrian = " << expectedDistPedestrian << " changeDist = " << changeDist << endl;
    if(angleDiff<=M_PI/4 and changeDist < 0){
      metric += changeDist;
      //cout << "metric = " << metric << endl;
    }
  }
  return metric;
}

void Tier3RearRotation::set_commenting(){
  //cout << "In RearRotation set commenting " << endl;
  if(beliefs->getAgentState()->getCrowdPose().poses.size() > 0)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3RearRotation::actionComment(FORRAction action){
  //cout << "Inside RearRotation" << endl;
  vector <Position> crowdPositions = beliefs->getAgentState()->getCrowdPositions(beliefs->getAgentState()->getCrowdPose());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
  double facingAngle = currentPosition.getTheta();
  //cout << "facingAngle = " << facingAngle << endl;
  double metric = 0;
  for(int i = 0; i < crowdPositions.size(); i++){
    double pedestrianTheta = crowdPositions[i].getTheta();
    double angleDiff = min(abs(pedestrianTheta - facingAngle),(2*M_PI) - abs(pedestrianTheta - facingAngle));
    //cout << "pedestrianTheta = " << pedestrianTheta << " angleDiff = " << angleDiff << endl;
    double currentDistPedestrian = currentPosition.getDistance(crowdPositions[i]);
    double expectedDistPedestrian = expectedPosition.getDistance(crowdPositions[i]);
    double changeDist = expectedDistPedestrian - currentDistPedestrian;
    //cout << "currentDistPedestrian = " << currentDistPedestrian << " expectedDistPedestrian = " << expectedDistPedestrian << " changeDist = " << changeDist << endl;
    if(angleDiff<=M_PI/4 and changeDist < 0){
      metric += changeDist;
      //cout << "metric = " << metric << endl;
    }
  }
  return metric;
}

void Tier3Side::set_commenting(){
  //cout << "In Side set commenting " << endl;
  if(beliefs->getAgentState()->getCrowdPose().poses.size() > 0)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3Side::actionComment(FORRAction action){
  //cout << "Inside Side" << endl;
  vector <Position> crowdPositions = beliefs->getAgentState()->getCrowdPositions(beliefs->getAgentState()->getCrowdPose());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
  //cout << "current Theta = " << currentPosition.getTheta() << endl;
  double metric = 0;
  for(int i = 0; i < crowdPositions.size(); i++){
    double pedestrianTheta = crowdPositions[i].getTheta();
    double angleDiff = min(abs(pedestrianTheta - currentPosition.getTheta()),(2*M_PI) - abs(pedestrianTheta - currentPosition.getTheta()));
    //cout << "pedestrianTheta = " << pedestrianTheta << " angleDiff = " << angleDiff << endl;
    if(angleDiff <= 5*M_PI/8 and angleDiff >= 3*M_PI/8){
      LineSegment robotLineSegment(CartesianPoint(currentPosition.getX(),currentPosition.getY()),CartesianPoint(currentPosition.getX()+5*cos(currentPosition.getTheta()),currentPosition.getY()+5*sin(currentPosition.getTheta())));
      LineSegment pedestrianLineSegment(CartesianPoint(crowdPositions[i].getX(),crowdPositions[i].getY()),CartesianPoint(crowdPositions[i].getX()+5*cos(pedestrianTheta),crowdPositions[i].getY()+5*sin(pedestrianTheta)));
      CartesianPoint intersectionPoint;
      if(do_intersect(robotLineSegment, pedestrianLineSegment, intersectionPoint)){
        double distPedestrian = crowdPositions[i].getDistance(intersectionPoint.get_x(), intersectionPoint.get_y());
        double expectedDistRobot = expectedPosition.getDistance(intersectionPoint.get_x(), intersectionPoint.get_y());
        double currentDistRobot = currentPosition.getDistance(intersectionPoint.get_x(), intersectionPoint.get_y());
        if(currentDistRobot < expectedDistRobot){
          metric += sqrt(distPedestrian*distPedestrian + currentDistRobot*currentDistRobot);
          //cout << "distPedestrian = " << distPedestrian << " currentDistRobot = " << currentDistRobot << " metric = " << metric << endl;
        }
        else{
          metric += sqrt(distPedestrian*distPedestrian + expectedDistRobot*expectedDistRobot);
          //cout << "distPedestrian = " << distPedestrian << " expectedDistRobot = " << expectedDistRobot << " metric = " << metric << endl;
        }
        //metric += (-1) * min(distPedestrian,expectedDistRobot)/max(distPedestrian,expectedDistRobot);
        //cout << "distPedestrian = " << distPedestrian << " expectedDistRobot = " << expectedDistRobot << " metric = " << metric << endl;
      }
    }
  }
  return metric;
}

void Tier3SideRotation::set_commenting(){
  //cout << "In SideRotation set commenting " << endl;
  if(beliefs->getAgentState()->getCrowdPose().poses.size() > 0)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3SideRotation::actionComment(FORRAction action){
  //cout << "Inside SideRotation" << endl;
  vector <Position> crowdPositions = beliefs->getAgentState()->getCrowdPositions(beliefs->getAgentState()->getCrowdPose());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
  //cout << "current Theta = " << currentPosition.getTheta() << endl;
  double metric = 0;
  for(int i = 0; i < crowdPositions.size(); i++){
    double pedestrianTheta = crowdPositions[i].getTheta();
    double angleDiff = min(abs(pedestrianTheta - currentPosition.getTheta()),(2*M_PI) - abs(pedestrianTheta - currentPosition.getTheta()));
    //cout << "pedestrianTheta = " << pedestrianTheta << " angleDiff = " << angleDiff << endl;
    if(angleDiff <= 5*M_PI/8 and angleDiff >= 3*M_PI/8){
      LineSegment robotLineSegment(CartesianPoint(currentPosition.getX(),currentPosition.getY()),CartesianPoint(currentPosition.getX()+5*cos(currentPosition.getTheta()),currentPosition.getY()+5*sin(currentPosition.getTheta())));
      LineSegment pedestrianLineSegment(CartesianPoint(crowdPositions[i].getX(),crowdPositions[i].getY()),CartesianPoint(crowdPositions[i].getX()+5*cos(pedestrianTheta),crowdPositions[i].getY()+5*sin(pedestrianTheta)));
      CartesianPoint intersectionPoint;
      if(do_intersect(robotLineSegment, pedestrianLineSegment, intersectionPoint)){
        double distPedestrian = crowdPositions[i].getDistance(intersectionPoint.get_x(), intersectionPoint.get_y());
        double expectedDistRobot = expectedPosition.getDistance(intersectionPoint.get_x(), intersectionPoint.get_y());
        double currentDistRobot = currentPosition.getDistance(intersectionPoint.get_x(), intersectionPoint.get_y());
        if(currentDistRobot < expectedDistRobot){
          metric += sqrt(distPedestrian*distPedestrian + currentDistRobot*currentDistRobot);
          //cout << "distPedestrian = " << distPedestrian << " currentDistRobot = " << currentDistRobot << " metric = " << metric << endl;
        }
        else{
          metric += sqrt(distPedestrian*distPedestrian + expectedDistRobot*expectedDistRobot);
          //cout << "distPedestrian = " << distPedestrian << " expectedDistRobot = " << expectedDistRobot << " metric = " << metric << endl;
        }
        //metric += (-1) * min(distPedestrian,expectedDistRobot)/max(distPedestrian,expectedDistRobot);
        //cout << "distPedestrian = " << distPedestrian << " expectedDistRobot = " << expectedDistRobot << " metric = " << metric << endl;
      }
    }
  }
  return metric;
}

void Tier3Visible::set_commenting(){
  //cout << "In Visible set commenting " << endl;
  if(beliefs->getAgentState()->getCrowdPose().poses.size() > 0)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3Visible::actionComment(FORRAction action){
  //cout << "Inside Visible" << endl;
  vector <Position> crowdPositions = beliefs->getAgentState()->getCrowdPositions(beliefs->getAgentState()->getCrowdPose());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
  double metric = 0, currentVisibility = 0, expectedVisibility = 0;
  for(int i = 0; i < crowdPositions.size(); i++){
    double pedestrianTheta = crowdPositions[i].getTheta();
    double pedestrianExpectedRobotAngle = atan2(crowdPositions[i].getY()-expectedPosition.getY(),crowdPositions[i].getX()-expectedPosition.getX());
    double angleDiffExpected = min(abs(pedestrianTheta - pedestrianExpectedRobotAngle),(2*M_PI) - abs(pedestrianTheta - pedestrianExpectedRobotAngle));
    //cout << "pedestrianTheta = " << pedestrianTheta << " pedestrianExpectedRobotAngle = " << pedestrianExpectedRobotAngle << " angleDiffExpected = " << angleDiffExpected << endl;
    if(angleDiffExpected <= M_PI/3){
      expectedVisibility += 1;
      //cout << "expectedVisibility = " << expectedVisibility << endl;
    }
    else{
      expectedVisibility += 0;
      //cout << "expectedVisibility = " << expectedVisibility << endl;
    }
    double pedestrianCurrentRobotAngle = atan2(crowdPositions[i].getY()-currentPosition.getY(),crowdPositions[i].getX()-currentPosition.getX());
    double angleDiffCurrent = min(abs(pedestrianTheta - pedestrianCurrentRobotAngle),(2*M_PI) - abs(pedestrianTheta - pedestrianCurrentRobotAngle));
    //cout << "pedestrianTheta = " << pedestrianTheta << " pedestrianCurrentRobotAngle = " << pedestrianCurrentRobotAngle << " angleDiffCurrent = " << angleDiffCurrent << endl;
    if(angleDiffCurrent <= M_PI/3){
      currentVisibility += 1;
      //cout << "currentVisibility = " << currentVisibility << endl;
    }
    else{
      currentVisibility += 0;
      //cout << "currentVisibility = " << currentVisibility << endl;
    }
  }
  metric = expectedVisibility - currentVisibility;
  return metric;
}

void Tier3VisibleRotation::set_commenting(){
  //cout << "In VisibleRotation set commenting " << endl;
  if(beliefs->getAgentState()->getCrowdPose().poses.size() > 0)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3VisibleRotation::actionComment(FORRAction action){
  //cout << "Inside VisibleRotation" << endl;
  vector <Position> crowdPositions = beliefs->getAgentState()->getCrowdPositions(beliefs->getAgentState()->getCrowdPose());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
  double metric = 0, currentVisibility = 0, expectedVisibility = 0;
  for(int i = 0; i < crowdPositions.size(); i++){
    double pedestrianTheta = crowdPositions[i].getTheta();
    double pedestrianExpectedRobotAngle = atan2(crowdPositions[i].getY()-expectedPosition.getY(),crowdPositions[i].getX()-expectedPosition.getX());
    double angleDiffExpected = min(abs(pedestrianTheta - pedestrianExpectedRobotAngle),(2*M_PI) - abs(pedestrianTheta - pedestrianExpectedRobotAngle));
    //cout << "pedestrianTheta = " << pedestrianTheta << " pedestrianExpectedRobotAngle = " << pedestrianExpectedRobotAngle << " angleDiffExpected = " << angleDiffExpected << endl;
    if(angleDiffExpected <= M_PI/3){
      expectedVisibility += 1;
      //cout << "expectedVisibility = " << expectedVisibility << endl;
    }
    else{
      expectedVisibility += 0;
      //cout << "expectedVisibility = " << expectedVisibility << endl;
    }
    double pedestrianCurrentRobotAngle = atan2(crowdPositions[i].getY()-currentPosition.getY(),crowdPositions[i].getX()-currentPosition.getX());
    double angleDiffCurrent = min(abs(pedestrianTheta - pedestrianCurrentRobotAngle),(2*M_PI) - abs(pedestrianTheta - pedestrianCurrentRobotAngle));
    //cout << "pedestrianTheta = " << pedestrianTheta << " pedestrianCurrentRobotAngle = " << pedestrianCurrentRobotAngle << " angleDiffCurrent = " << angleDiffCurrent << endl;
    if(angleDiffCurrent <= M_PI/3){
      currentVisibility += 1;
      //cout << "currentVisibility = " << currentVisibility << endl;
    }
    else{
      currentVisibility += 0;
      //cout << "currentVisibility = " << currentVisibility << endl;
    }
  }
  metric = expectedVisibility - currentVisibility;
  return metric;
}

/*void Tier3Wait::set_commenting(){
  cout << "In Wait set commenting " << endl;
  if(beliefs->getAgentState()->getCrowdPose().poses.size() > 0)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3Wait::actionComment(FORRAction action){
  cout << "Inside Wait" << endl;
  vector <Position> crowdPositions = beliefs->getAgentState()->getCrowdPositions(beliefs->getAgentState()->getCrowdPose());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
  cout << "current Theta = " << currentPosition.getTheta() << endl;
  double metric = 0;
  for(int i = 0; i < crowdPositions.size(); i++){
    double pedestrianTheta = crowdPositions[i].getTheta();
    double angleDiff = min(abs(pedestrianTheta - currentPosition.getTheta()),(2*M_PI) - abs(pedestrianTheta - currentPosition.getTheta()));
    cout << "pedestrianTheta = " << pedestrianTheta << " angleDiff = " << angleDiff << endl;
    if(angleDiff <= 5*M_PI/8 and angleDiff >= 3*M_PI/8){
      LineSegment robotLineSegment(CartesianPoint(currentPosition.getX(),currentPosition.getY()),CartesianPoint(currentPosition.getX()+5*cos(currentPosition.getTheta()),currentPosition.getY()+5*sin(currentPosition.getTheta())));
      LineSegment pedestrianLineSegment(CartesianPoint(crowdPositions[i].getX(),crowdPositions[i].getY()),CartesianPoint(crowdPositions[i].getX()+5*cos(pedestrianTheta),crowdPositions[i].getY()+5*sin(pedestrianTheta)));
      CartesianPoint intersectionPoint;
      if(do_intersect(robotLineSegment, pedestrianLineSegment, intersectionPoint)){
        double distPedestrian = crowdPositions[i].getDistance(intersectionPoint.get_x(), intersectionPoint.get_y());
        double expectedDistRobot = expectedPosition.getDistance(intersectionPoint.get_x(), intersectionPoint.get_y());
        metric += (-1) * min(distPedestrian,expectedDistRobot)/max(distPedestrian,expectedDistRobot);
        cout << "distPedestrian = " << distPedestrian << " expectedDistRobot = " << expectedDistRobot << " metric = " << metric << endl;
      }
    }
  }
  return metric;
}

void Tier3WaitRotation::set_commenting(){
  cout << "In WaitRotation set commenting " << endl;
  if(beliefs->getAgentState()->getCrowdPose().poses.size() > 0)
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3WaitRotation::actionComment(FORRAction action){
  cout << "Inside WaitRotation" << endl;
  vector <Position> crowdPositions = beliefs->getAgentState()->getCrowdPositions(beliefs->getAgentState()->getCrowdPose());
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
  cout << "current Theta = " << currentPosition.getTheta() << endl;
  double metric = 0;
  for(int i = 0; i < crowdPositions.size(); i++){
    double pedestrianTheta = crowdPositions[i].getTheta();
    double angleDiff = min(abs(pedestrianTheta - currentPosition.getTheta()),(2*M_PI) - abs(pedestrianTheta - currentPosition.getTheta()));
    cout << "pedestrianTheta = " << pedestrianTheta << " angleDiff = " << angleDiff << endl;
    if(angleDiff <= 5*M_PI/8 and angleDiff >= 3*M_PI/8){
      LineSegment robotLineSegment(CartesianPoint(currentPosition.getX(),currentPosition.getY()),CartesianPoint(currentPosition.getX()+5*cos(currentPosition.getTheta()),currentPosition.getY()+5*sin(currentPosition.getTheta())));
      LineSegment pedestrianLineSegment(CartesianPoint(crowdPositions[i].getX(),crowdPositions[i].getY()),CartesianPoint(crowdPositions[i].getX()+5*cos(pedestrianTheta),crowdPositions[i].getY()+5*sin(pedestrianTheta)));
      CartesianPoint intersectionPoint;
      if(do_intersect(robotLineSegment, pedestrianLineSegment, intersectionPoint)){
        double distPedestrian = crowdPositions[i].getDistance(intersectionPoint.get_x(), intersectionPoint.get_y());
        double expectedDistRobot = expectedPosition.getDistance(intersectionPoint.get_x(), intersectionPoint.get_y());
        metric += (-1) * min(distPedestrian,expectedDistRobot)/max(distPedestrian,expectedDistRobot);
        cout << "distPedestrian = " << distPedestrian << " expectedDistRobot = " << expectedDistRobot << " metric = " << metric << endl;
      }
    }
  }
  return metric;
}*/

void Tier3CrowdAvoid::set_commenting(){
  //cout << "In CrowdAvoid set commenting " << endl;
  if(beliefs->getAgentState()->crowdModelLearned())
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3CrowdAvoid::actionComment(FORRAction action){
  //cout << "Inside CrowdAvoid" << endl;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double grid_value = beliefs->getAgentState()->getGridValue(expectedPosition.getX(), expectedPosition.getY());
  return (-1) * grid_value;
}

void Tier3CrowdAvoidRotation::set_commenting(){
  //cout << "In CrowdAvoidRotation set commenting " << endl;
  if(beliefs->getAgentState()->crowdModelLearned())
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3CrowdAvoidRotation::actionComment(FORRAction action){
  //cout << "Inside CrowdAvoidRotation" << endl;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double grid_value = beliefs->getAgentState()->getGridValue(expectedPosition.getX(), expectedPosition.getY());
  return (-1) * grid_value;
}

void Tier3RiskAvoid::set_commenting(){
  //cout << "In RiskAvoid set commenting " << endl;
  if(beliefs->getAgentState()->riskModelLearned())
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3RiskAvoid::actionComment(FORRAction action){
  //cout << "Inside RiskAvoid" << endl;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double risk_value = beliefs->getAgentState()->getRiskValue(expectedPosition.getX(), expectedPosition.getY());
  return (-1) * risk_value;
}

void Tier3RiskAvoidRotation::set_commenting(){
  //cout << "In RiskAvoidRotation set commenting " << endl;
  if(beliefs->getAgentState()->riskModelLearned())
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3RiskAvoidRotation::actionComment(FORRAction action){
  //cout << "Inside RiskAvoidRotation" << endl;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double risk_value = beliefs->getAgentState()->getRiskValue(expectedPosition.getX(), expectedPosition.getY());
  return (-1) * risk_value;
}

void Tier3FlowAvoid::set_commenting(){
  //cout << "In FlowAvoid set commenting " << endl;
  if(beliefs->getAgentState()->flowModelLearned())
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3FlowAvoid::actionComment(FORRAction action){
  //cout << "Inside FlowAvoid" << endl;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double flow_value = beliefs->getAgentState()->getFlowValue(expectedPosition.getX(), expectedPosition.getY(), expectedPosition.getTheta());
  return flow_value;
}

void Tier3FlowAvoidRotation::set_commenting(){
  //cout << "In FlowAvoidRotation set commenting " << endl;
  if(beliefs->getAgentState()->flowModelLearned())
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3FlowAvoidRotation::actionComment(FORRAction action){
  //cout << "Inside FlowAvoidRotation" << endl;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double flow_value = beliefs->getAgentState()->getFlowValue(expectedPosition.getX(), expectedPosition.getY(), expectedPosition.getTheta());
  return flow_value;
}

void Tier3FindTheCrowd::set_commenting(){
  //cout << "In FindTheCrowd set commenting " << endl;
  if(beliefs->getAgentState()->crowdModelLearned())
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3FindTheCrowd::actionComment(FORRAction action){
  //cout << "Inside FindTheCrowd" << endl;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double grid_value = beliefs->getAgentState()->getCrowdObservation(expectedPosition.getX(), expectedPosition.getY());
  return (-1) * grid_value;
}

void Tier3FindTheCrowdRotation::set_commenting(){
  //cout << "In FindTheCrowdRotation set commenting " << endl;
  if(beliefs->getAgentState()->crowdModelLearned())
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3FindTheCrowdRotation::actionComment(FORRAction action){
  //cout << "Inside FindTheCrowdRotation" << endl;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double grid_value = beliefs->getAgentState()->getCrowdObservation(expectedPosition.getX(), expectedPosition.getY());
  return (-1) * grid_value;
}

void Tier3FindTheRisk::set_commenting(){
  //cout << "In FindTheRisk set commenting " << endl;
  if(beliefs->getAgentState()->riskModelLearned())
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3FindTheRisk::actionComment(FORRAction action){
  //cout << "Inside FindTheRisk" << endl;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double risk_value = beliefs->getAgentState()->getRiskExperience(expectedPosition.getX(), expectedPosition.getY());
  return (-1) * risk_value;
}

void Tier3FindTheRiskRotation::set_commenting(){
  //cout << "In FindTheRiskRotation set commenting " << endl;
  if(beliefs->getAgentState()->riskModelLearned())
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3FindTheRiskRotation::actionComment(FORRAction action){
  //cout << "Inside FindTheRiskRotation" << endl;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double risk_value = beliefs->getAgentState()->getRiskExperience(expectedPosition.getX(), expectedPosition.getY());
  return (-1) * risk_value;
}

void Tier3FindTheFlow::set_commenting(){
  //cout << "In FindTheFlow set commenting " << endl;
  if(beliefs->getAgentState()->flowModelLearned())
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3FindTheFlow::actionComment(FORRAction action){
  //cout << "Inside FindTheFlow" << endl;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double flow_value = beliefs->getAgentState()->getFLowObservation(expectedPosition.getX(), expectedPosition.getY());
  return (-1) * flow_value;
}

void Tier3FindTheFlowRotation::set_commenting(){
  //cout << "In FindTheFlowRotation set commenting " << endl;
  if(beliefs->getAgentState()->flowModelLearned())
    advisor_commenting = true;
  else
    advisor_commenting = false;
}

double Tier3FindTheFlowRotation::actionComment(FORRAction action){
  //cout << "Inside FindTheFlowRotation" << endl;
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  double flow_value = beliefs->getAgentState()->getFLowObservation(expectedPosition.getX(), expectedPosition.getY());
  return (-1) * flow_value;
}

double Tier3Follow::actionComment(FORRAction action){
  //cout << "Inside Follow" << endl;
  double result=0;
  vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  vector<int> targetClosestHallway;
  vector<int> expPosClosestHallway;
  vector<double> targetDistances;
  vector<double> expPosDistances;
  double targetDistToHallway = 1000000.0, expDistToHallway = 1000000.0;
  //cout << "Number of hallways = " << hallways.size() << endl;
  // Get distances between all hallways and the target and expected position
  for(int i = 0; i < hallways.size() ; i++){
    targetDistances.push_back(hallways[i].distanceToAggregate(targetPoint));
    expPosDistances.push_back(hallways[i].distanceToAggregate(expPosition));
  }
  // Get minimum distance between the target and expected position and a hallway
  for(int i = 0; i < hallways.size(); i++){
    //cout << "targetDistances[" << i << "] == " << targetDistances[i] << "; expPosDistances[" << i << "] == " << expPosDistances[i] << endl;
    if(targetDistances[i] < targetDistToHallway){
      targetDistToHallway = targetDistances[i];
    }
    if(expPosDistances[i] < expDistToHallway){
      expDistToHallway = expPosDistances[i];
    }
  }
  //cout << "targetDistToHallway = " << targetDistToHallway << "; expDistToHallway = " << expDistToHallway << endl;
  // Get all hallways with minimum distance to the target and expected position
  for(int i = 0; i < hallways.size(); i++){
    if(targetDistances[i] == targetDistToHallway){
      targetClosestHallway.push_back(i);
      //cout << "targetClosestHallway = " << i << endl;
    }
    if(expPosDistances[i] == expDistToHallway){
      expPosClosestHallway.push_back(i);
      //cout << "expPosClosestHallway = " << i << endl;
    }
  }
  // Check whether the target and expected position are in the same or different hallways
  bool sameHallway = false, expCloseToSame = false, diffHallwayConnected = false, diffHallwayUnconnected = false;
  double targetDist, expPosDist;
  int targetHallwayConn, expPosHallwayConn, targetHallwayNoConn, expPosHallwayNoConn;
  for(int i = 0; i < targetClosestHallway.size(); i++){
    for(int j = 0; j < expPosClosestHallway.size(); j++){
      //cout << "targetClosestHallway[" << i << "] = " << targetClosestHallway[i] << "; expPosClosestHallway[" << j << "] = " << expPosClosestHallway[j] << endl;
      if(targetClosestHallway[i] == expPosClosestHallway[j] and targetDistances[targetClosestHallway[i]] == 0 and expPosDistances[expPosClosestHallway[j]] == 0){
        sameHallway = true;
        //cout << "sameHallway = true" << endl;
      }
      else if(targetClosestHallway[i] == expPosClosestHallway[j] and targetDistances[targetClosestHallway[i]] >= 0 and expPosDistances[expPosClosestHallway[j]] >= 0){
        expCloseToSame = true;
        targetDist = targetDistances[targetClosestHallway[i]];
        expPosDist = expPosDistances[expPosClosestHallway[j]];
        //cout << "expCloseToSame = true; targetDist = " << targetDist << "; expPosDist = " << expPosDist << endl;
      }
      // else if(targetClosestHallway[i] != expPosClosestHallway[j]){
      //   if(hallways[targetClosestHallway[i]].isHallwayConnected(expPosClosestHallway[j]) == true){
      //     diffHallwayConnected = true;
      //     targetHallwayConn = targetClosestHallway[i];
      //     expPosHallwayConn = expPosClosestHallway[j];
      //     //cout << "diffHallwayConnected = true; targetHallwayConn = " << targetHallwayConn << "; expPosHallwayConn = " << expPosHallwayConn << endl;
      //   }
      //   else{
      //     diffHallwayUnconnected = true;
      //     targetHallwayNoConn = targetClosestHallway[i];
      //     expPosHallwayNoConn = expPosClosestHallway[j];
      //     //cout << "diffHallwayUnconnected = true; targetHallwayNoConn = " << targetHallwayNoConn << "; expPosHallwayNoConn = " << expPosHallwayNoConn << endl;
      //   }
      // }
    }
  }

  if(sameHallway == true){
    result = (-1) * expPosition.get_distance(targetPoint); //distance from expected position to target
  }
  else if(expCloseToSame == true){
    result = (-1) * (expPosition.get_distance(targetPoint) + expPosDist + targetDist); //distance from expected position to target plus distance from expected position to hallway plus distance from target to hallway
  }
  // else if(diffHallwayConnected == true){
  //   result = (-1) * (expPosDistances[targetHallwayConn] + targetDistances[expPosHallwayConn]); //distance from expected position to target's hallway plus distance from target to expected position's hallway
  // }
  // else if(diffHallwayUnconnected == true){
  //   result = (-1) * (hallways[targetHallwayNoConn].distanceBetweenAggregates(hallways[expPosHallwayNoConn]) + expPosDistances[expPosHallwayNoConn] + targetDistances[targetHallwayNoConn]); //distance between target's hallway and expected position's hallway plus distance from expected position to hallway plus distance from target to hallway
  // }
  //cout << "result = " << result << endl;
  return result;
}

void Tier3Follow::set_commenting(){
  //cout << "In Follow set commenting " << endl;
  vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
  Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
  CartesianPoint currPosition (currentPosition.getX(), currentPosition.getY());
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  bool currPosInHallway = false, targetInHallway = false;
  //cout << "Number of hallways = " << hallways.size() << endl;
  if(hallways.size() == 0){
    advisor_commenting = false;
  }
  else{
    for(int i = 0; i < hallways.size(); i++){
      if(hallways[i].pointInAggregate(currPosition) and hallways[i].pointInAggregate(targetPoint)){
        currPosInHallway = true;
        targetInHallway = true;
        break;
      }
    }
    // for(int i = 0; i < hallways.size(); i++){
    //   if(hallways[i].pointInAggregate(targetPoint)){
    //     targetInHallway = true;
    //     break;
    //   }
    // }
    if(currPosInHallway == true and targetInHallway == true){
      //cout << "currPosInHallway = true and targetInHallway = true" << endl;
      advisor_commenting = true;
    }
    else{
      //cout << "currPosInHallway = false and/or targetInHallway = false" << endl;
      advisor_commenting = false;
    }
  }
}

double Tier3FollowRotation::actionComment(FORRAction action){
  //cout << "Inside FollowRotation" << endl;
  double result=0;
  vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  vector<int> targetClosestHallway;
  vector<int> expPosClosestHallway;
  vector<double> targetDistances;
  vector<double> expPosDistances;
  double targetDistToHallway = 1000000.0, expDistToHallway = 1000000.0;
  //cout << "Number of hallways = " << hallways.size() << endl;
  // Get distances between all hallways and the target and expected position
  for(int i = 0; i < hallways.size() ; i++){
    targetDistances.push_back(hallways[i].distanceToAggregate(targetPoint));
    expPosDistances.push_back(hallways[i].distanceToAggregate(expPosition));
  }
  // Get minimum distance between the target and expected position and a hallway
  for(int i = 0; i < hallways.size(); i++){
    //cout << "targetDistances[" << i << "] == " << targetDistances[i] << "; expPosDistances[" << i << "] == " << expPosDistances[i] << endl;
    if(targetDistances[i] < targetDistToHallway){
      targetDistToHallway = targetDistances[i];
    }
    if(expPosDistances[i] < expDistToHallway){
      expDistToHallway = expPosDistances[i];
    }
  }
  //cout << "targetDistToHallway = " << targetDistToHallway << "; expDistToHallway = " << expDistToHallway << endl;
  // Get all hallways with minimum distance to the target and expected position
  for(int i = 0; i < hallways.size(); i++){
    if(targetDistances[i] == targetDistToHallway){
      targetClosestHallway.push_back(i);
      //cout << "targetClosestHallway = " << i << endl;
    }
    if(expPosDistances[i] == expDistToHallway){
      expPosClosestHallway.push_back(i);
      //cout << "expPosClosestHallway = " << i << endl;
    }
  }
  // Check whether the target and expected position are in the same or different hallways
  bool sameHallway = false, expCloseToSame = false, diffHallwayConnected = false, diffHallwayUnconnected = false;
  double targetDist, expPosDist;
  int targetHallwayConn, expPosHallwayConn, targetHallwayNoConn, expPosHallwayNoConn;
  for(int i = 0; i < targetClosestHallway.size(); i++){
    for(int j = 0; j < expPosClosestHallway.size(); j++){
      //cout << "targetClosestHallway[" << i << "] = " << targetClosestHallway[i] << "; expPosClosestHallway[" << j << "] = " << expPosClosestHallway[j] << endl;
      if(targetClosestHallway[i] == expPosClosestHallway[j] and targetDistances[targetClosestHallway[i]] == 0 and expPosDistances[expPosClosestHallway[j]] == 0){
        sameHallway = true;
        //cout << "sameHallway = true" << endl;
      }
      else if(targetClosestHallway[i] == expPosClosestHallway[j] and targetDistances[targetClosestHallway[i]] >= 0 and expPosDistances[expPosClosestHallway[j]] >= 0){
        expCloseToSame = true;
        targetDist = targetDistances[targetClosestHallway[i]];
        expPosDist = expPosDistances[expPosClosestHallway[j]];
        //cout << "expCloseToSame = true; targetDist = " << targetDist << "; expPosDist = " << expPosDist << endl;
      }
      // else if(targetClosestHallway[i] != expPosClosestHallway[j]){
      //   if(hallways[targetClosestHallway[i]].isHallwayConnected(expPosClosestHallway[j]) == true){
      //     diffHallwayConnected = true;
      //     targetHallwayConn = targetClosestHallway[i];
      //     expPosHallwayConn = expPosClosestHallway[j];
      //     //cout << "diffHallwayConnected = true; targetHallwayConn = " << targetHallwayConn << "; expPosHallwayConn = " << expPosHallwayConn << endl;
      //   }
      //   else{
      //     diffHallwayUnconnected = true;
      //     targetHallwayNoConn = targetClosestHallway[i];
      //     expPosHallwayNoConn = expPosClosestHallway[j];
      //     //cout << "diffHallwayUnconnected = true; targetHallwayNoConn = " << targetHallwayNoConn << "; expPosHallwayNoConn = " << expPosHallwayNoConn << endl;
      //   }
      // }
    }
  }

  if(sameHallway == true){
    result = (-1) * expPosition.get_distance(targetPoint); //distance from expected position to target
  }
  else if(expCloseToSame == true){
    result = (-1) * (expPosition.get_distance(targetPoint) + expPosDist + targetDist); //distance from expected position to target plus distance from expected position to hallway plus distance from target to hallway
  }
  // else if(diffHallwayConnected == true){
  //   result = (-1) * (expPosDistances[targetHallwayConn] + targetDistances[expPosHallwayConn]); //distance from expected position to target's hallway plus distance from target to expected position's hallway
  // }
  // else if(diffHallwayUnconnected == true){
  //   result = (-1) * (hallways[targetHallwayNoConn].distanceBetweenAggregates(hallways[expPosHallwayNoConn]) + expPosDistances[expPosHallwayNoConn] + targetDistances[targetHallwayNoConn]); //distance between target's hallway and expected position's hallway plus distance from expected position to hallway plus distance from target to hallway
  // }
  //cout << "result = " << result << endl;
  return result;
}

void Tier3FollowRotation::set_commenting(){
  //cout << "In FollowRotation set commenting " << endl;
  vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
  Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
  CartesianPoint currPosition (currentPosition.getX(), currentPosition.getY());
  Task *task = beliefs->getAgentState()->getCurrentTask();
  CartesianPoint targetPoint (task->getX() , task->getY());
  bool currPosInHallway = false, targetInHallway = false;
  //cout << "Number of hallways = " << hallways.size() << endl;
  if(hallways.size() == 0){
    advisor_commenting = false;
  }
  else{
    for(int i = 0; i < hallways.size(); i++){
      if(hallways[i].pointInAggregate(currPosition) and hallways[i].pointInAggregate(targetPoint)){
        currPosInHallway = true;
        targetInHallway = true;
        break;
      }
    }
    // for(int i = 0; i < hallways.size(); i++){
    //   if(hallways[i].pointInAggregate(targetPoint)){
    //     targetInHallway = true;
    //     break;
    //   }
    // }
    if(currPosInHallway == true and targetInHallway == true){
      //cout << "currPosInHallway = true and targetInHallway = true" << endl;
      advisor_commenting = true;
    }
    else{
      //cout << "currPosInHallway = false and/or targetInHallway = false" << endl;
      advisor_commenting = false;
    }
  }
}

double Tier3Crossroads::actionComment(FORRAction action){
  //cout << "Inside Crossroads" << endl;
  double result=0;
  vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  //cout << "Number of hallways = " << hallways.size() << endl;
  for(int i = 0; i < hallways.size() ; i++){
    double tempDist = hallways[i].distanceToAggregate(expPosition);
    //cout << "tempDist = " << tempDist << endl;
    if(tempDist > 1){
      result += ((1 / tempDist) * hallways[i].numConnections());
    }
    else{
      result += (hallways[i].numConnections());
    }
    //cout << "result = " << result << endl;
  }
  //cout << "Final result = " << result << endl;
  return result;
}

void Tier3Crossroads::set_commenting(){
  //cout << "In Crossroads set commenting " << endl;
  vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
  if(hallways.size() > 0){
    bool atLeastOneConnection = false;
    for(int i = 0; i < hallways.size() ; i++){
      if(hallways[i].numConnections() > 0){
        atLeastOneConnection = true;
        break;
      }
    }
    if(atLeastOneConnection)
      advisor_commenting = true;
    else
      advisor_commenting = false;
  }
  else{
    advisor_commenting = false;
  }
}

double Tier3CrossroadsRotation::actionComment(FORRAction action){
  //cout << "Inside CrossroadsRotation" << endl;
  double result=0;
  vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  //cout << "Number of hallways = " << hallways.size() << endl;
  for(int i = 0; i < hallways.size() ; i++){
    double tempDist = hallways[i].distanceToAggregate(expPosition);
    //cout << "tempDist = " << tempDist << endl;
    if(tempDist > 1){
      result += ((1 / tempDist) * hallways[i].numConnections());
    }
    else{
      result += (hallways[i].numConnections());
    }
    //cout << "result = " << result << endl;
  }
  //cout << "Final result = " << result << endl;
  return result;
}

void Tier3CrossroadsRotation::set_commenting(){
  //cout << "In CrossroadsRotation set commenting " << endl;
  vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
  if(hallways.size() > 0){
    bool atLeastOneConnection = false;
    for(int i = 0; i < hallways.size() ; i++){
      if(hallways[i].numConnections() > 0){
        atLeastOneConnection = true;
        break;
      }
    }
    if(atLeastOneConnection)
      advisor_commenting = true;
    else
      advisor_commenting = false;
  }
  else{
    advisor_commenting = false;
  }
}

double Tier3Stay::actionComment(FORRAction action){
  //cout << "Inside Stay" << endl;
  vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  double minDistance = 1000000.0;
  //cout << "Number of hallways = " << hallways.size() << endl;
  for(int i = 0; i < hallways.size() ; i++){
    double tempDist = hallways[i].distanceToAggregate(expPosition);
    //cout << "tempDist = " << tempDist << endl;
    if(tempDist < minDistance){
      minDistance = tempDist;
    }
    if(minDistance == 0){
      break;
    }
  }
  //cout << "minDistance = " << minDistance << endl;
  return ((-1) * minDistance);
}

void Tier3Stay::set_commenting(){
  //cout << "In Stay set commenting " << endl;
  vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
  Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
  CartesianPoint currPosition (currentPosition.getX(), currentPosition.getY());
  bool currPosInHallway = false;
  //cout << "Number of hallways = " << hallways.size() << endl;
  if(hallways.size() == 0){
    advisor_commenting = false;
  }
  else{
    for(int i = 0; i < hallways.size(); i++){
      if(hallways[i].pointInAggregate(currPosition)){
        currPosInHallway = true;
        break;
      }
    }
    if(currPosInHallway == true){
      //cout << "currPosInHallway = true" << endl;
      advisor_commenting = true;
    }
    else{
      //cout << "currPosInHallway = false" << endl;
      advisor_commenting = false;
    }
  }
}

double Tier3StayRotation::actionComment(FORRAction action){
  //cout << "Inside StayRotation" << endl;
  vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
  Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(action);
  CartesianPoint expPosition (expectedPosition.getX(), expectedPosition.getY());
  double minDistance = 1000000.0;
  //cout << "Number of hallways = " << hallways.size() << endl;
  for(int i = 0; i < hallways.size() ; i++){
    double tempDist = hallways[i].distanceToAggregate(expPosition);
    //cout << "tempDist = " << tempDist << endl;
    if(tempDist < minDistance){
      minDistance = tempDist;
    }
    if(minDistance == 0){
      break;
    }
  }
  //cout << "minDistance = " << minDistance << endl;
  return ((-1) * minDistance);
}

void Tier3StayRotation::set_commenting(){
  //cout << "In StayRotation set commenting " << endl;
  vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
  Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
  CartesianPoint currPosition (currentPosition.getX(), currentPosition.getY());
  bool currPosInHallway = false;
  //cout << "Number of hallways = " << hallways.size() << endl;
  if(hallways.size() == 0){
    advisor_commenting = false;
  }
  else{
    for(int i = 0; i < hallways.size(); i++){
      if(hallways[i].pointInAggregate(currPosition)){
        currPosInHallway = true;
        break;
      }
    }
    if(currPosInHallway == true){
      //cout << "currPosInHallway = true" << endl;
      advisor_commenting = true;
    }
    else{
      //cout << "currPosInHallway = false" << endl;
      advisor_commenting = false;
    }
  }
}