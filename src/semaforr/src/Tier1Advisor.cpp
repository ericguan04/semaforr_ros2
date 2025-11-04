
#include "Tier1Advisor.h"


void Tier1Advisor::advisorNotOpposite(){
  // RCLCPP_DEBUG(this->get_logger(), "COntroller::advisorNotOpposite > Entering function");
  vector<FORRAction> actions = beliefs->getAgentState()->getCurrentTask()->getPreviousDecisions();
  set<FORRAction> *rotation_set = beliefs->getAgentState()->getRotationActionSet();
  int size = actions.size();
  if(actions.size() < 2){
    // RCLCPP_DEBUG(this->get_logger(), "actions list less than 2. Exiting not opposite");
    return;
  }
  FORRAction lastAction = actions[size - 1];
  FORRAction lastlastAction = actions[size - 2];
  FORRAction lastlastlastAction = actions[size - 3];
  // RCLCPP_DEBUG_STREAM(this->get_logger(), "Controller::advisorNotOpposite > " << lastAction.type << " " << lastAction.parameter << ", " << lastlastAction.type << " " << lastlastAction.parameter << ", " << lastlastlastAction.type << " " << lastlastlastAction.parameter); 
  if(lastlastAction.type == RIGHT_TURN and lastAction.type == PAUSE){
    // RCLCPP_DEBUG(this->get_logger(), "Not opposite active ");
    (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(LEFT_TURN, lastlastAction.parameter)));
    // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << FORRAction(LEFT_TURN, lastlastAction.parameter).type << " " << lastlastAction.parameter);
    if(lastlastAction.parameter > 1){
      for(int i = 1; i < lastlastAction.parameter; i++){
        (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(LEFT_TURN, i)));
        // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << FORRAction(LEFT_TURN, i).type << " " << i);
      }
    }
  }
  else if(lastlastAction.type == RIGHT_TURN and lastAction.type == RIGHT_TURN){
    // RCLCPP_DEBUG(this->get_logger(), "Not opposite active ");
    double angle_rotated = beliefs->getAgentState()->getRotation(lastlastAction.parameter) + beliefs->getAgentState()->getRotation(lastAction.parameter);
    for(int i = 1; i < rotation_set->size()/2+1 ; i++){
      if(angle_rotated + beliefs->getAgentState()->getRotation(i) - M_PI > 0){
        (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(RIGHT_TURN, i)));
        // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << FORRAction(RIGHT_TURN, i).type << " " << i);
      }
    }
    (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(LEFT_TURN, lastlastAction.parameter)));
    // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << FORRAction(LEFT_TURN, lastlastAction.parameter).type << " " << lastlastAction.parameter);
    if(beliefs->getAgentState()->getRotation(lastlastAction.parameter) + beliefs->getAgentState()->getRotation(lastAction.parameter) == beliefs->getAgentState()->getRotation(lastlastAction.parameter + lastAction.parameter)){
      (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(LEFT_TURN, lastlastAction.parameter+lastAction.parameter)));
      // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << FORRAction(LEFT_TURN, lastlastAction.parameter).type << " " << lastlastAction.parameter+lastAction.parameter);
    }
  }
  if(lastAction.type == RIGHT_TURN){
    // RCLCPP_DEBUG(this->get_logger(), "Not opposite active ");
    (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(LEFT_TURN, lastAction.parameter)));
    // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << FORRAction(LEFT_TURN, lastAction.parameter).type << " " << lastAction.parameter);
    if(lastAction.parameter > 1){
      for(int i = 1; i < lastAction.parameter; i++){
        (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(LEFT_TURN, i)));
        // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << FORRAction(LEFT_TURN, i).type << " " << i);
      }
    }
  }


  if(lastlastAction.type == LEFT_TURN and lastAction.type == PAUSE){
    // RCLCPP_DEBUG(this->get_logger(), "Not opposite active ");
    (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(RIGHT_TURN, lastlastAction.parameter)));
    // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << FORRAction(RIGHT_TURN, lastlastAction.parameter).type << " " << lastlastAction.parameter);
    if(lastlastAction.parameter > 1){
      for(int i = 1; i < lastlastAction.parameter; i++){
        (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(RIGHT_TURN, i)));
        // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << FORRAction(RIGHT_TURN, i).type << " " << i);
      }
    }
  }
  else if(lastlastAction.type == LEFT_TURN and lastAction.type == LEFT_TURN){
    // RCLCPP_DEBUG(this->get_logger(), "Not opposite active ");
    double angle_rotated = beliefs->getAgentState()->getRotation(lastlastAction.parameter) + beliefs->getAgentState()->getRotation(lastAction.parameter);
    for(int i = 1; i < rotation_set->size()/2+1 ; i++){
      if(angle_rotated + beliefs->getAgentState()->getRotation(i) - M_PI > 0){
        (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(LEFT_TURN, i)));
        // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << FORRAction(LEFT_TURN, i).type << " " << i);
      }
    }
    (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(RIGHT_TURN, lastlastAction.parameter)));
    // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << FORRAction(RIGHT_TURN, lastlastAction.parameter).type << " " << lastlastAction.parameter);
    if(beliefs->getAgentState()->getRotation(lastlastAction.parameter) + beliefs->getAgentState()->getRotation(lastAction.parameter) == beliefs->getAgentState()->getRotation(lastlastAction.parameter + lastAction.parameter)){
      (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(RIGHT_TURN, lastlastAction.parameter+lastAction.parameter)));
      // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << FORRAction(RIGHT_TURN, lastlastAction.parameter).type << " " << lastlastAction.parameter+lastAction.parameter);
    }
  }
  if(lastAction.type == LEFT_TURN){
    // RCLCPP_DEBUG(this->get_logger(), "Not opposite active ");
    (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(RIGHT_TURN, lastAction.parameter)));
    // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << FORRAction(RIGHT_TURN, lastAction.parameter).type << " " << lastAction.parameter);
    if(lastAction.parameter > 1){
      for(int i = 1; i < lastAction.parameter; i++){
        (beliefs->getAgentState()->getVetoedActions()->insert(FORRAction(RIGHT_TURN, i)));
        // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << FORRAction(RIGHT_TURN, i).type << " " << i);
      }
    }
  }
  // RCLCPP_DEBUG(this->get_logger(), "leaving notOpposite");
  return;
}

bool Tier1Advisor::advisorVictory(FORRAction *decision) {
  // RCLCPP_DEBUG(this->get_logger(), "Begin Victory advisor");
  // if the robot is oriented towards the goal and the robot actions which are not vetoed allows the robot to reach the goal then take that action.
  bool decisionMade = false;
  CartesianPoint task(beliefs->getAgentState()->getCurrentTask()->getTaskX(),beliefs->getAgentState()->getCurrentTask()->getTaskY());
  // RCLCPP_DEBUG(this->get_logger(), "Check if target can be spotted using laser scan");
  cout << "Target = " << task.get_x() << " " << task.get_y() << endl;
  bool targetInSight = beliefs->getAgentState()->canSeePoint(task, 20);
  if(targetInSight == false){
    // RCLCPP_DEBUG(this->get_logger(), "Target not in sight, Victory advisor skipped");
  }
  else{
    // RCLCPP_DEBUG(this->get_logger(), "Target in sight , Victory advisor active");
    (*decision) = beliefs->getAgentState()->moveTowards(task);
    if(decision->parameter != 0){
      Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
      if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
        if(decision->type == RIGHT_TURN or decision->type == LEFT_TURN){
          // RCLCPP_DEBUG(this->get_logger(), "Target in sight and no obstacles, Victory advisor to take decision");
          decisionMade = true;
        }
        else{
          FORRAction forward = beliefs->getAgentState()->maxForwardAction();
          if(forward.parameter >= decision->parameter){
            // RCLCPP_DEBUG(this->get_logger(), "Target in sight and no obstacles, Victory advisor to take decision");
            decisionMade = true;
          }
        }
      }
    }
  }
  if(decisionMade == true){
    beliefs->getAgentState()->setRepositionTriggered(false);
    beliefs->getAgentState()->setRepositionCount(0);
  }
  if(localExploration->getAlreadyStarted()){
    double search_radius = 7.0;
    CartesianPoint current(beliefs->getAgentState()->getCurrentPosition().getX(), beliefs->getAgentState()->getCurrentPosition().getY());
    vector< LineSegment > potential_exploration;
    vector<CartesianPoint> laserEndpoints = beliefs->getAgentState()->getCurrentLaserEndpoints();
    for(int i = 0; i < laserEndpoints.size(); i++){
      LineSegment pair = LineSegment(current, laserEndpoints[i]);
      if(distance(task, pair) < search_radius){
        potential_exploration.push_back(pair);
      }
    }
    localExploration->addToQueue(potential_exploration);
    localExploration->updateCoverage(current);
  }
  return decisionMade;
}

bool Tier1Advisor::advisorEnforcer(FORRAction *decision) {
  // RCLCPP_DEBUG(this->get_logger(), "Begin Enforcer advisor");
  // if the robot is oriented towards the goal and the robot actions which are not vetoed allows the robot to reach the goal then take that action.
  bool decisionMade = false;
  // RCLCPP_DEBUG(this->get_logger(), "Check if waypoint can be spotted using laser scan");
  cout << "PlannerName " << beliefs->getAgentState()->getCurrentTask()->getPlannerName() << " PlanSize " << beliefs->getAgentState()->getCurrentTask()->getPlanSize() << endl;
  if((beliefs->getAgentState()->getCurrentTask()->getPlannerName() == "skeleton" or beliefs->getAgentState()->getCurrentTask()->getPlannerName() == "hallwayskel") and beliefs->getAgentState()->getCurrentTask()->getPlanSize() > 0){
    if(beliefs->getAgentState()->getEnforcerCount() >= 4 and beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getType() == 1){
      beliefs->getAgentState()->getCurrentTask()->skipWaypoint();
    }
  }
  if(beliefs->getAgentState()->getCurrentTask()->getPlannerName() != "skeleton" and beliefs->getAgentState()->getCurrentTask()->getPlannerName() != "hallwayskel" and beliefs->getAgentState()->getCurrentTask()->getPlanSize() > 0){
    CartesianPoint waypoint(beliefs->getAgentState()->getCurrentTask()->getX(),beliefs->getAgentState()->getCurrentTask()->getY()); // THIS GIVES -nan -nan FOR SOME REASON
    cout << "Waypoint = " << waypoint.get_x() << " " << waypoint.get_y() << endl;
    bool waypointInSight = beliefs->getAgentState()->canSeePoint(waypoint, 20);
    if(waypointInSight == true){
      cout << "Waypoint in sight, Enforcer advisor active" << endl;
      // RCLCPP_DEBUG(this->get_logger(), "Waypoint in sight , Enforcer advisor active");
      (*decision) = beliefs->getAgentState()->moveTowards(waypoint);
      if(decision->parameter != 0){
        set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
        if(vetoed_actions->find(*decision) != vetoed_actions->end()){
          decisionMade = false;
        }
        else{
          Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
          if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
            if(decision->type == RIGHT_TURN or decision->type == LEFT_TURN){
              cout << "Waypoint in sight and no obstacles and not vetoed, Enforcer advisor to take decision" << endl;
              // RCLCPP_DEBUG(this->get_logger(), "Waypoint in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
              decisionMade = true;
            }
            else{
              FORRAction forward = beliefs->getAgentState()->maxForwardAction();
              if(forward.parameter >= decision->parameter){
                cout << "Waypoint in sight and no obstacles and not vetoed, Enforcer advisor to take decision" << endl;
                // RCLCPP_DEBUG(this->get_logger(), "Waypoint in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                decisionMade = true;
              }
            }
          }
        }
      }
    }
    else{
      // RCLCPP_DEBUG(this->get_logger(), "Waypoint not in sight, Enforcer advisor skipped");
    }
  }
  else if(beliefs->getAgentState()->getCurrentTask()->getPlannerName() == "skeleton" and beliefs->getAgentState()->getCurrentTask()->getPlanSize() > 0){
    cout << "Waypoint Region = " << beliefs->getAgentState()->getCurrentTask()->getX() << " " << beliefs->getAgentState()->getCurrentTask()->getY() << endl;
    bool waypointRegionInSight = false;
    int regionID = -1;
    bool nextWaypointRegionInSight = false;
    int nextRegionID = -1;
    bool waypointPathInSight = false;
    int pathID = -1;
    double waypointDistThreshold = 20;
    if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getType() == 0){
      waypointRegionInSight = beliefs->getAgentState()->canSeeRegion(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getRegion().getCenter(), beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getRegion().getRadius(), waypointDistThreshold);
      regionID = 0;
      if(beliefs->getAgentState()->getCurrentTask()->getPlanSize() >= 2){
        vector<CartesianPoint> pathBetween = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getPath();
        for(int i = 0; i < pathBetween.size(); i++){
          if(beliefs->getAgentState()->canSeePoint(pathBetween[i], waypointDistThreshold)){
            waypointPathInSight = true;
            pathID = 1;
            break;
          }
        }
      }
      if(beliefs->getAgentState()->getCurrentTask()->getPlanSize() >= 3){
        nextWaypointRegionInSight = beliefs->getAgentState()->canSeeRegion(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[2].getRegion().getCenter(), beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[2].getRegion().getRadius(), waypointDistThreshold);
        nextRegionID = 2;
      }
    }
    else{
      vector<CartesianPoint> pathBetween = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getPath();
      for(int i = 0; i < pathBetween.size(); i++){
        if(beliefs->getAgentState()->canSeePoint(pathBetween[i], waypointDistThreshold)){
          waypointPathInSight = true;
          pathID = 0;
          break;
        }
      }
      if(beliefs->getAgentState()->getCurrentTask()->getPlanSize() >= 2){
        waypointRegionInSight = beliefs->getAgentState()->canSeeRegion(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getRegion().getCenter(), beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getRegion().getRadius(), waypointDistThreshold);
        regionID = 1;
      }
      if(beliefs->getAgentState()->getCurrentTask()->getPlanSize() >= 4){
        nextWaypointRegionInSight = beliefs->getAgentState()->canSeeRegion(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[3].getRegion().getCenter(), beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[3].getRegion().getRadius(), waypointDistThreshold);
        nextRegionID = 3;
      }
      if(waypointPathInSight == false){
        beliefs->getAgentState()->increaseEnforcerCount();
      }
    }
    // cout << "waypointRegionInSight " << waypointRegionInSight << " regionID " << regionID << " nextWaypointRegionInSight " << nextWaypointRegionInSight << " nextRegionID " << nextRegionID << " waypointPathInSight " << waypointPathInSight << " pathID " << pathID << endl;
    if(nextWaypointRegionInSight == true){
      // RCLCPP_DEBUG(this->get_logger(), "Next Waypoint Region in sight, Enforcer advisor active");
      set<FORRAction> *vetoedActions = beliefs->getAgentState()->getVetoedActions();
      set<FORRAction> *action_set = beliefs->getAgentState()->getActionSet();
      set<FORRAction> possible_set;
      set<FORRAction>::iterator actionIter;
      FORRAction closest_action;
      double dist_to_region = 100000;
      vector<FORRAction> actions = beliefs->getAgentState()->getCurrentTask()->getPreviousDecisions();
      FORRAction lastAction;
      int size = actions.size();
      // cout << size << " " << (size - 1) << " " << ((size - 1) >= 0) << endl;
      if(size > 0){
        // cout << "first" << endl;
        lastAction = actions[size - 1];
        // cout << "after" << endl;
      }
      else{
        // cout << "second" << endl;
        lastAction = FORRAction(PAUSE, 0);
        // cout << "after" << endl;
      }
      // cout << "lastAction " << lastAction.type << " " << lastAction.parameter << endl;
      for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
        FORRAction forrAction = *actionIter;
        if(std::find(vetoedActions->begin(), vetoedActions->end(), forrAction) != vetoedActions->end()){
          // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << forrAction.type << " " << forrAction.parameter);
          continue;
        }
        else if(forrAction.type == PAUSE or forrAction.parameter == 0){
          // RCLCPP_DEBUG_STREAM(this->get_logger(), "Pause action : " << forrAction.type << " " << forrAction.parameter);
          continue;
        }
        else if((lastAction.type == RIGHT_TURN or lastAction.type == LEFT_TURN) and (forrAction.type == RIGHT_TURN or forrAction.type == LEFT_TURN)){
          // RCLCPP_DEBUG_STREAM(this->get_logger(), "Skip action : " << forrAction.type << " " << forrAction.parameter);
          continue;
        }
        else{
          Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
          if(nextRegionID == 2){
            if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[2].getRegion().inRegion(expectedPosition.getX(), expectedPosition.getY())){
              FORRAction a(forrAction.type,forrAction.parameter);
              // RCLCPP_DEBUG_STREAM(this->get_logger(), "Potential action : " << a.type << " " << a.parameter);
              possible_set.insert(a);
            }
            else{
              // RCLCPP_DEBUG_STREAM(this->get_logger(), "Get closer action : " << forrAction.type << " " << forrAction.parameter);
              double dist_from_exp = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[2].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
              // cout << dist_from_exp << endl;
              if(dist_from_exp < dist_to_region){
                dist_to_region = dist_from_exp;
                closest_action = forrAction;
              }
            }
          }
          else if(nextRegionID == 3){
            if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[3].getRegion().inRegion(expectedPosition.getX(), expectedPosition.getY())){
              FORRAction a(forrAction.type,forrAction.parameter);
              // RCLCPP_DEBUG_STREAM(this->get_logger(), "Potential action : " << a.type << " " << a.parameter);
              possible_set.insert(a);
            }
            else{
              // RCLCPP_DEBUG_STREAM(this->get_logger(), "Get closer action : " << forrAction.type << " " << forrAction.parameter);
              double dist_from_exp = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[3].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
              // cout << dist_from_exp << endl;
              if(dist_from_exp < dist_to_region){
                dist_to_region = dist_from_exp;
                closest_action = forrAction;
              }
            }
          }
        }
      }
      if(possible_set.size() == 1){
        actionIter = possible_set.begin();
        (*decision) = (*actionIter);
        // RCLCPP_DEBUG(this->get_logger(), "Only one action to get to Next Waypoint Region, Enforcer advisor to take decision");
        decisionMade = true;
        shortcut = true;
      }
      else if(possible_set.size() > 1){
        // RCLCPP_DEBUG(this->get_logger(), "More than one action to get to Next Waypoint Region, Enforcer advisor to take decision");
        if(nextRegionID == 2){
          double minDistance = 1000000;
          for(actionIter = possible_set.begin(); actionIter != possible_set.end(); actionIter++){
            FORRAction forrAction = *actionIter;
            // RCLCPP_DEBUG_STREAM(this->get_logger(), "Potential action : " << forrAction.type << " " << forrAction.parameter);
            Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
            if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[2].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY())) < minDistance){
              minDistance = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[2].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
              // cout << "New minDistance " << minDistance << endl;
              (*decision) = forrAction;
              decisionMade = true;
              shortcut = true;
            }
          }
        }
        else if(nextRegionID == 3){
          double minDistance = 1000000;
          for(actionIter = possible_set.begin(); actionIter != possible_set.end(); actionIter++){
            FORRAction forrAction = *actionIter;
            // RCLCPP_DEBUG_STREAM(this->get_logger(), "Potential action : " << forrAction.type << " " << forrAction.parameter);
            Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
            if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[3].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY())) < minDistance){
              minDistance = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[3].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
              // cout << "New minDistance " << minDistance << endl;
              (*decision) = forrAction;
              decisionMade = true;
              shortcut = true;
            }
          }
        }
      }
      // else if(dist_to_region < 100000){
      //   // RCLCPP_DEBUG(this->get_logger(), "Action to get closest to visible Next Waypoint Region, Enforcer advisor to take decision");
      //   (*decision) = closest_action;
      //   decisionMade = true;
      // }
    }
    else{
      // RCLCPP_DEBUG(this->get_logger(), "Next Waypoint Region not in sight");
    }
    if(decisionMade == false and waypointRegionInSight == true){
      // RCLCPP_DEBUG(this->get_logger(), "Waypoint Region in sight, Enforcer advisor active");
      set<FORRAction> *vetoedActions = beliefs->getAgentState()->getVetoedActions();
      set<FORRAction> *action_set = beliefs->getAgentState()->getActionSet();
      set<FORRAction> possible_set;
      set<FORRAction>::iterator actionIter;
      FORRAction closest_action;
      double dist_to_region = 100000;
      vector<FORRAction> actions = beliefs->getAgentState()->getCurrentTask()->getPreviousDecisions();
      FORRAction lastAction;
      int size = actions.size();
      // cout << size << " " << (size - 1) << " " << ((size - 1) >= 0) << endl;
      if(size > 0){
        // cout << "first" << endl;
        lastAction = actions[size - 1];
        // cout << "after" << endl;
      }
      else{
        // cout << "second" << endl;
        lastAction = FORRAction(PAUSE, 0);
        // cout << "after" << endl;
      }
      // cout << "lastAction " << lastAction.type << " " << lastAction.parameter << endl;
      for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
        FORRAction forrAction = *actionIter;
        if(std::find(vetoedActions->begin(), vetoedActions->end(), forrAction) != vetoedActions->end()){
          // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << forrAction.type << " " << forrAction.parameter);
          continue;
        }
        else if(forrAction.type == PAUSE or forrAction.parameter == 0){
          // RCLCPP_DEBUG_STREAM(this->get_logger(), "Pause action : " << forrAction.type << " " << forrAction.parameter);
          continue;
        }
        else if((lastAction.type == RIGHT_TURN or lastAction.type == LEFT_TURN) and (forrAction.type == RIGHT_TURN or forrAction.type == LEFT_TURN)){
          // RCLCPP_DEBUG_STREAM(this->get_logger(), "Skip action : " << forrAction.type << " " << forrAction.parameter);
          continue;
        }
        else{
          Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
          if(regionID == 0){
            if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getRegion().inRegion(expectedPosition.getX(), expectedPosition.getY())){
              FORRAction a(forrAction.type,forrAction.parameter);
              // RCLCPP_DEBUG_STREAM(this->get_logger(), "Potential action : " << a.type << " " << a.parameter);
              possible_set.insert(a);
            }
            else{
              // RCLCPP_DEBUG_STREAM(this->get_logger(), "Get closer action : " << forrAction.type << " " << forrAction.parameter);
              double dist_from_exp = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
              // cout << dist_from_exp << endl;
              if(dist_from_exp < dist_to_region){
                dist_to_region = dist_from_exp;
                closest_action = forrAction;
              }
            }
          }
          else if(regionID == 1){
            if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getRegion().inRegion(expectedPosition.getX(), expectedPosition.getY())){
              FORRAction a(forrAction.type,forrAction.parameter);
              // RCLCPP_DEBUG_STREAM(this->get_logger(), "Potential action : " << a.type << " " << a.parameter);
              possible_set.insert(a);
            }
            else{
              // RCLCPP_DEBUG_STREAM(this->get_logger(), "Get closer action : " << forrAction.type << " " << forrAction.parameter);
              double dist_from_exp = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
              // cout << dist_from_exp << endl;
              if(dist_from_exp < dist_to_region){
                dist_to_region = dist_from_exp;
                closest_action = forrAction;
              }
            }
          }
        }
      }
      if(possible_set.size() == 1){
        actionIter = possible_set.begin();
        (*decision) = (*actionIter);
        // RCLCPP_DEBUG(this->get_logger(), "Only one action to get to Waypoint Region, Enforcer advisor to take decision");
        decisionMade = true;
      }
      else if(possible_set.size() > 1){
        // RCLCPP_DEBUG(this->get_logger(), "More than one action to get to Waypoint Region, Enforcer advisor to take decision");
        if(regionID == 0){
          double minDistance = 1000000;
          for(actionIter = possible_set.begin(); actionIter != possible_set.end(); actionIter++){
            FORRAction forrAction = *actionIter;
            // RCLCPP_DEBUG_STREAM(this->get_logger(), "Potential action : " << forrAction.type << " " << forrAction.parameter);
            Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
            if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY())) < minDistance){
              minDistance = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
              // cout << "New minDistance " << minDistance << endl;
              (*decision) = forrAction;
              decisionMade = true;
            }
          }
        }
        else if(regionID == 1){
          double minDistance = 1000000;
          for(actionIter = possible_set.begin(); actionIter != possible_set.end(); actionIter++){
            FORRAction forrAction = *actionIter;
            // RCLCPP_DEBUG_STREAM(this->get_logger(), "Potential action : " << forrAction.type << " " << forrAction.parameter);
            Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
            if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY())) < minDistance){
              minDistance = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
              // cout << "New minDistance " << minDistance << endl;
              (*decision) = forrAction;
              decisionMade = true;
            }
          }
        }
      }
      else if(dist_to_region < 100000){
        // RCLCPP_DEBUG(this->get_logger(), "Action to get closest to visible Waypoint Region, Enforcer advisor to take decision");
        (*decision) = closest_action;
        decisionMade = true;
      }
    }
    else{
      // RCLCPP_DEBUG(this->get_logger(), "Waypoint Region not in sight");
    }
    if(decisionMade == false and waypointPathInSight == true){
      // RCLCPP_DEBUG(this->get_logger(), "Waypoint Region not in sight or no action can get there");
      vector<CartesianPoint> pathBetweenWaypoints;
      if(pathID == 0){
        pathBetweenWaypoints = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getPath();
      }
      else if(pathID == 1){
        pathBetweenWaypoints = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[1].getPath();
      }
      // cout << "pathBetweenWaypoints " << pathBetweenWaypoints.size() << endl;
      CartesianPoint farthestVisible;
      int farthest = -1;
      for(int i = pathBetweenWaypoints.size()-1; i >= 0; i--){
        if(beliefs->getAgentState()->canSeePoint(pathBetweenWaypoints[i], waypointDistThreshold)){
          farthestVisible = pathBetweenWaypoints[i];
          // cout << "farthestVisible " << i << endl;
          farthest = i;
          break;
        }
      }
      if(farthest >= 0){
        (*decision) = beliefs->getAgentState()->moveTowards(farthestVisible);
        if(decision->parameter != 0){
          set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
          if(vetoed_actions->find(*decision) != vetoed_actions->end()){
            decisionMade = false;
          }
          else{
            Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
            if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
              if(decision->type == RIGHT_TURN or decision->type == LEFT_TURN){
                // RCLCPP_DEBUG(this->get_logger(), "Waypoint in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                decisionMade = true;
              }
              else{
                FORRAction forward = beliefs->getAgentState()->maxForwardAction();
                if(forward.parameter >= decision->parameter){
                  // RCLCPP_DEBUG(this->get_logger(), "Waypoint in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                  decisionMade = true;
                  // if(decision->type == FORWARD)
                  //   beliefs->getAgentState()->setGetOutTriggered(false);
                }
              }
            }
          }
        }
      }
    }
  }
  else if(beliefs->getAgentState()->getCurrentTask()->getPlannerName() == "hallwayskel" and beliefs->getAgentState()->getCurrentTask()->getPlanSize() > 0){
    cout << "Passage Waypoint = " << beliefs->getAgentState()->getCurrentTask()->getX() << " " << beliefs->getAgentState()->getCurrentTask()->getY() << endl;
    bool waypointRegionInSight = false;
    int regionID = -1;
    bool nextWaypointRegionInSight = false;
    int nextRegionID = -1;
    bool waypointPathInSight = false;
    int pathID = -1;
    bool intersectionInSight = false;
    int intersectionID = -1;
    bool nextIntersectionInSight = false;
    int nextIntersectionID = -1;
    bool passageInSight = false;
    int passageID = -1;
    int lookAhead = 3;
    if(beliefs->getAgentState()->getCurrentTask()->getPlanSize() < lookAhead){
      lookAhead = beliefs->getAgentState()->getCurrentTask()->getPlanSize();
    }
    for(int i = lookAhead-1; i >=0; i--){
      if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[i].getType() == 0){
        regionID = i;
        waypointRegionInSight = beliefs->getAgentState()->canSeeRegion(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[regionID].getRegion().getCenter(), beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[regionID].getRegion().getRadius(), 25);
        if(waypointRegionInSight){
          // RCLCPP_DEBUG(this->get_logger(), "Waypoint Region in sight, Enforcer advisor active");
          set<FORRAction> *vetoedActions = beliefs->getAgentState()->getVetoedActions();
          set<FORRAction> *action_set = beliefs->getAgentState()->getActionSet();
          set<FORRAction> possible_set;
          set<FORRAction>::iterator actionIter;
          FORRAction closest_action;
          double dist_to_region = 100000;
          vector<FORRAction> actions = beliefs->getAgentState()->getCurrentTask()->getPreviousDecisions();
          FORRAction lastAction;
          int size = actions.size();
          // cout << size << " " << (size - 1) << " " << ((size - 1) >= 0) << endl;
          if(size > 0){
            // cout << "first" << endl;
            lastAction = actions[size - 1];
            // cout << "after" << endl;
          }
          else{
            // cout << "second" << endl;
            lastAction = FORRAction(PAUSE, 0);
            // cout << "after" << endl;
          }
          // cout << "lastAction " << lastAction.type << " " << lastAction.parameter << endl;
          for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
            FORRAction forrAction = *actionIter;
            if(std::find(vetoedActions->begin(), vetoedActions->end(), forrAction) != vetoedActions->end()){
              // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << forrAction.type << " " << forrAction.parameter);
              continue;
            }
            else if(forrAction.type == PAUSE or forrAction.parameter == 0){
              // RCLCPP_DEBUG_STREAM(this->get_logger(), "Pause action : " << forrAction.type << " " << forrAction.parameter);
              continue;
            }
            else if((lastAction.type == RIGHT_TURN or lastAction.type == LEFT_TURN) and (forrAction.type == RIGHT_TURN or forrAction.type == LEFT_TURN)){
              // RCLCPP_DEBUG_STREAM(this->get_logger(), "Skip action : " << forrAction.type << " " << forrAction.parameter);
              continue;
            }
            else{
              Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
              if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[regionID].getRegion().inRegion(expectedPosition.getX(), expectedPosition.getY())){
                FORRAction a(forrAction.type,forrAction.parameter);
                // RCLCPP_DEBUG_STREAM(this->get_logger(), "Potential action : " << a.type << " " << a.parameter);
                possible_set.insert(a);
              }
              else{
                // RCLCPP_DEBUG_STREAM(this->get_logger(), "Get closer action : " << forrAction.type << " " << forrAction.parameter);
                double dist_from_exp = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[regionID].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
                // cout << dist_from_exp << endl;
                if(dist_from_exp < dist_to_region){
                  dist_to_region = dist_from_exp;
                  closest_action = forrAction;
                }
              }
            }
          }
          if(possible_set.size() == 1){
            actionIter = possible_set.begin();
            (*decision) = (*actionIter);
            // RCLCPP_DEBUG(this->get_logger(), "Only one action to get to Waypoint Region, Enforcer advisor to take decision");
            decisionMade = true;
          }
          else if(possible_set.size() > 1){
            // RCLCPP_DEBUG(this->get_logger(), "More than one action to get to Waypoint Region, Enforcer advisor to take decision");
            double minDistance = 1000000;
            for(actionIter = possible_set.begin(); actionIter != possible_set.end(); actionIter++){
              FORRAction forrAction = *actionIter;
              // RCLCPP_DEBUG_STREAM(this->get_logger(), "Potential action : " << forrAction.type << " " << forrAction.parameter);
              Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
              if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[regionID].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY())) < minDistance){
                minDistance = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[regionID].getRegion().getCenter().get_distance(CartesianPoint(expectedPosition.getX(), expectedPosition.getY()));
                // cout << "New minDistance " << minDistance << endl;
                (*decision) = forrAction;
                decisionMade = true;
              }
            }
          }
          else if(dist_to_region < 100000){
            // RCLCPP_DEBUG(this->get_logger(), "Action to get closest to visible Waypoint Region, Enforcer advisor to take decision");
            (*decision) = closest_action;
            decisionMade = true;
          }
          if(decisionMade){
            if(i > 0){
              shortcut = true;
            }
            break;
          }
        }
      }
      else if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[i].getType() == 1){
        pathID = i;
        vector<CartesianPoint> pathBetween = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[pathID].getPath();
        for(int j = 0; j < pathBetween.size(); j++){
          if(beliefs->getAgentState()->canSeePoint(pathBetween[j], 20)){
            waypointPathInSight = true;
            break;
          }
        }
        if(waypointPathInSight == false){
          beliefs->getAgentState()->increaseEnforcerCount();
        }
      }
      else if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[i].getType() == 2){
        intersectionID = i;
        vector< vector<int> > passagePoints = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[intersectionID].getPassagePoints();
        for(int j = 0; j < passagePoints.size(); j++){
          if(beliefs->getAgentState()->canSeePoint(CartesianPoint(passagePoints[j][0], passagePoints[j][1]), 20)){
            intersectionInSight = true;
            break;
          }
        }
        if(intersectionInSight){
          // RCLCPP_DEBUG(this->get_logger(), "Waypoint Intersection in sight, Enforcer advisor active");
          CartesianPoint intersectionCentroid = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[intersectionID].getPassageCentroid();
          bool intersectionCentroidInSight = beliefs->getAgentState()->canSeePoint(intersectionCentroid, 20);
          if(intersectionCentroidInSight){
            // RCLCPP_DEBUG(this->get_logger(), "Waypoint Intersection Centroid in sight");
            (*decision) = beliefs->getAgentState()->moveTowards(intersectionCentroid);
            if(decision->parameter != 0){
              set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
              if(vetoed_actions->find(*decision) != vetoed_actions->end()){
                decisionMade = false;
              }
              else{
                Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
                if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
                  if(decision->type == RIGHT_TURN or decision->type == LEFT_TURN){
                    // RCLCPP_DEBUG(this->get_logger(), "Waypoint centroid in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                    decisionMade = true;
                  }
                  else{
                    FORRAction forward = beliefs->getAgentState()->maxForwardAction();
                    if(forward.parameter >= decision->parameter){
                      // RCLCPP_DEBUG(this->get_logger(), "Waypoint centroid in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                      decisionMade = true;
                    }
                  }
                }
              }
            }
          }
          // if(decisionMade){
          //   break;
          // }
        }
      }
      else if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[i].getType() == 3){
        passageID = i;
        vector<CartesianPoint> pathBetween = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[passageID].getPath();
        for(int j = 0; j < pathBetween.size(); j++){
          if(beliefs->getAgentState()->canSeePoint(pathBetween[j], 20)){
            passageInSight = true;
            break;
          }
        }
        if(passageInSight){
          // RCLCPP_DEBUG(this->get_logger(), "Waypoint Passage in sight, Enforcer advisor active");
          CartesianPoint farthestVisible;
          int farthest = -1;
          for(int i = pathBetween.size()-1; i >= 0; i--){
            if(beliefs->getAgentState()->canSeePoint(pathBetween[i], 20)){
              farthestVisible = pathBetween[i];
              // cout << "farthestVisible " << i << endl;
              farthest = i;
              break;
            }
          }
          if(farthest >= 0){
            (*decision) = beliefs->getAgentState()->moveTowards(farthestVisible);
            if(decision->parameter != 0){
              set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
              if(vetoed_actions->find(*decision) != vetoed_actions->end()){
                decisionMade = false;
              }
              else{
                Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
                if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
                  if(decision->type == RIGHT_TURN or decision->type == LEFT_TURN){
                    // RCLCPP_DEBUG(this->get_logger(), "Waypoint Passage path in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                    decisionMade = true;
                  }
                  else{
                    FORRAction forward = beliefs->getAgentState()->maxForwardAction();
                    if(forward.parameter >= decision->parameter){
                      // RCLCPP_DEBUG(this->get_logger(), "Waypoint Passage path in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                      decisionMade = true;
                    }
                  }
                }
              }
            }
          }
        }
        // if(decisionMade){
        //   break;
        // }
      }
    }

    // cout << "waypointRegionInSight " << waypointRegionInSight << " regionID " << regionID << " nextWaypointRegionInSight " << nextWaypointRegionInSight << " nextRegionID " << nextRegionID << " waypointPathInSight " << waypointPathInSight << " pathID " << pathID << " intersectionInSight " << intersectionInSight << " intersectionID " << intersectionID << " nextIntersectionInSight " << nextIntersectionInSight << " nextIntersectionID " << nextIntersectionID << " passageInSight " << passageInSight << " passageID " << passageID << endl;
    if(decisionMade == false and pathID > -1 and waypointPathInSight){
      // RCLCPP_DEBUG(this->get_logger(), "Waypoint Region or Intersection not in sight or no action can get there, try waypoint path");
      vector<CartesianPoint> pathBetweenWaypoints = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoints()[pathID].getPath();
      // cout << "pathBetweenWaypoints " << pathBetweenWaypoints.size() << endl;
      CartesianPoint farthestVisible;
      int farthest = -1;
      for(int i = pathBetweenWaypoints.size()-1; i >= 0; i--){
        if(beliefs->getAgentState()->canSeePoint(pathBetweenWaypoints[i], 20)){
          farthestVisible = pathBetweenWaypoints[i];
          // cout << "farthestVisible " << i << endl;
          farthest = i;
          break;
        }
      }
      if(farthest >= 0){
        (*decision) = beliefs->getAgentState()->moveTowards(farthestVisible);
        if(decision->parameter != 0){
          set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
          if(vetoed_actions->find(*decision) != vetoed_actions->end()){
            decisionMade = false;
          }
          else{
            Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
            if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
              if(decision->type == RIGHT_TURN or decision->type == LEFT_TURN){
                // RCLCPP_DEBUG(this->get_logger(), "Waypoint path in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                decisionMade = true;
              }
              else{
                FORRAction forward = beliefs->getAgentState()->maxForwardAction();
                if(forward.parameter >= decision->parameter){
                  // RCLCPP_DEBUG(this->get_logger(), "Waypoint path in sight and no obstacles and not vetoed, Enforcer advisor to take decision");
                  decisionMade = true;
                }
              }
            }
          }
        }
      }
    }
    else{
      // RCLCPP_DEBUG(this->get_logger(), "Waypoint Path not in sight");
    }
    if(decisionMade == false){
      // RCLCPP_DEBUG(this->get_logger(), "No Plan element in sight");
    }

  }
  if(decisionMade == true){
    beliefs->getAgentState()->setRepositionTriggered(false);
    beliefs->getAgentState()->setRepositionCount(0);
    beliefs->getAgentState()->setFindAWayCount(0);
    beliefs->getAgentState()->setEnforcerCount(0);
    Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
    beliefs->getAgentState()->getCurrentTask()->updatePlanPositions(currentPosition.getX(), currentPosition.getY());
  }
  return decisionMade;
}


/*
 * This advisor has to do prevent all the actions that will result in robot hitting the obstacles.
 * Straight forward thing is to check for collsion in the orientation.
 */
bool Tier1Advisor::advisorAvoidObstacles(){
  // RCLCPP_DEBUG(this->get_logger(), "In advisor avoid obstacles");
  cout << "Advisor Avoid Obstacles called" << endl;
  FORRAction max_forward = beliefs->getAgentState()->maxForwardAction();
  // RCLCPP_DEBUG_STREAM(this->get_logger(), "Max allowed forward action : " << max_forward.type << " " << max_forward.parameter);
  cout << "Max allowed forward action : " << max_forward.type << " " << max_forward.parameter << endl;
  int intensity = max_forward.parameter;
  set<FORRAction> *vetoedActions = beliefs->getAgentState()->getVetoedActions();
  set<FORRAction> *forward_set = beliefs->getAgentState()->getForwardActionSet();
  cout << "vetoedActions and forward_set obtained" << endl;
  for(int i = forward_set->size()-1 ; i > 0; i--){
    FORRAction a(FORWARD,i);
    if(i > intensity){
      // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << a.type << " " << a.parameter);
      vetoedActions->insert(a);
    }
    // Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(a);
    // cout << "expectedPosition " << expectedPosition.getX() << " " << expectedPosition.getY() << " Action : " << a.type << " " << a.parameter << endl;
  }
  cout << "for loop for forward actions done" << endl;
  set<FORRAction> *rotation_set = beliefs->getAgentState()->getRotationActionSet();
  cout << "rotation_set obtained" << endl;
  for(int i = 1; i < rotation_set->size()/2+1 ; i++){
    FORRAction forrAction = FORRAction(RIGHT_TURN, i);
    cout << "Start getting expected position for RIGHT_TURN " << endl;
    Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
    cout << "Got expected position for RIGHT_TURN " << endl;
    // cout << "expectedPosition " << expectedPosition.getX() << " " << expectedPosition.getY() << " Action : " << forrAction.type << " " << forrAction.parameter << endl;
    if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) < 0.1){
      (beliefs->getAgentState()->getVetoedActions()->insert(forrAction));
      // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << forrAction.type << " " << forrAction.parameter);
    }
    forrAction = FORRAction(LEFT_TURN, i);
    expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
    // cout << "expectedPosition " << expectedPosition.getX() << " " << expectedPosition.getY() << " Action : " << forrAction.type << " " << forrAction.parameter << endl;
    if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) < 0.1){
      (beliefs->getAgentState()->getVetoedActions()->insert(forrAction));
      // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << forrAction.type << " " << forrAction.parameter);
    }
  }
  return false; 
}

bool Tier1Advisor::advisorDontGoBack(){
  // RCLCPP_DEBUG(this->get_logger(), "In advisor don't go back");
  set<FORRAction> *vetoedActions = beliefs->getAgentState()->getVetoedActions();
  set<FORRAction> *action_set = beliefs->getAgentState()->getActionSet();
  set<FORRAction>::iterator actionIter;
  int numOfVetoes = 0;
  for(actionIter = action_set->begin(); actionIter != action_set->end(); actionIter++){
    FORRAction forrAction = *actionIter;
    if(std::find(vetoedActions->begin(), vetoedActions->end(), forrAction) != vetoedActions->end()){
      continue;
    }
    else if(forrAction.type == PAUSE or forrAction.type == FORWARD){
      continue;
    }
    else{
      Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction(forrAction);
      if(beliefs->getAgentState()->getCurrentTask()->getPlanPositionValue(expectedPosition.getX(), expectedPosition.getY())){
        FORRAction a(forrAction.type,forrAction.parameter);
        // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << a.type << " " << a.parameter);
        vetoedActions->insert(a);
        numOfVetoes++;
      }
    }
  }
  cout << "Don't go back number of vetoes " << numOfVetoes << " " << action_set->size() << endl;
  if(numOfVetoes == beliefs->getAgentState()->getRotationActionSet()->size()){
    beliefs->getAgentState()->getCurrentTask()->resetPlanPositions();
  }
  return false; 
}

bool Tier1Advisor::advisorGetOut(FORRAction *decision) {
  // RCLCPP_DEBUG(this->get_logger(), "Begin get out advisor");
  // if the robot is in a confined space and it sees a way out then take the action that does that.
  bool decisionMade = false;
  set<FORRAction> *rotation_set = beliefs->getAgentState()->getRotationActionSet();
  int decisionLimit = 10 + (int)(beliefs->getAgentState()->getCurrentTask()->getPositionHistory()->size()/50);
  if(beliefs->getAgentState()->getGetOutTriggered() == false and beliefs->getAgentState()->getRobotConfined(decisionLimit, 1)){
    vector<FORRAction> actions = beliefs->getAgentState()->getCurrentTask()->getPreviousDecisions();
    int size = actions.size();
    // cout << "actions size " << size << " rotation size " << rotation_set->size()/2 << endl;
    if(size >= 5){
      FORRAction lastAction = actions[size - 1];
      FORRAction lastlastAction = actions[size - 2];
      FORRAction lastlastlastAction = actions[size - 3];
      FORRAction lastlastlastlastAction = actions[size - 4];
      // cout << "lastAction " << lastAction.type << " " << lastAction.parameter << " lastlastAction " <<  lastlastAction.type << " " << lastlastAction.parameter << " lastlastlastAction " <<  lastlastlastAction.type << " " << lastlastlastAction.parameter << " lastlastlastlastAction " <<  lastlastlastlastAction.type << " " << lastlastlastlastAction.parameter << endl;
      if(lastlastAction.type == RIGHT_TURN and lastlastAction.parameter == rotation_set->size()/2 and lastAction.type == RIGHT_TURN and lastAction.parameter == rotation_set->size()/2 and lastlastlastAction.type == RIGHT_TURN and lastlastlastAction.parameter == rotation_set->size()/2 and lastlastlastlastAction.type == RIGHT_TURN and lastlastlastlastAction.parameter == rotation_set->size()/2){
        vector<Position> *positionHis = beliefs->getAgentState()->getAllPositionTrace();
        vector< vector <CartesianPoint> > *laserHis = beliefs->getAgentState()->getAllLaserHistory();
        CartesianPoint current_position = CartesianPoint(positionHis->at(positionHis->size()-1).getX(), positionHis->at(positionHis->size()-1).getY());
        // cout << "current_position " << current_position.get_x() << " " << current_position.get_y() << endl;
        vector<Position> last_positions;
        vector< vector<CartesianPoint> > last_endpoints;
        int previous_count = 4;
        for(int i = 1; i < previous_count+1; i++){
          last_positions.push_back(positionHis->at(positionHis->size()-i));
          last_endpoints.push_back(laserHis->at(laserHis->size()-i));
        }
        // cout << last_positions.size() << " " << last_lasers.size() << " " << last_endpoints.size() << endl;
        // cout << last_positions.size() << " " << last_endpoints.size() << endl;
        int last_invisible = 1;
        int last_visible = 1;
        for(int i = 2; i < positionHis->size(); i++){
          bool anyVisible = false;
          for(int j = 0; j < last_positions.size(); j++){
            if(beliefs->getAgentState()->canSeePoint(last_endpoints[j], CartesianPoint(last_positions[j].getX(), last_positions[j].getY()), CartesianPoint(positionHis->at(positionHis->size()-i).getX(), positionHis->at(positionHis->size()-i).getY()), 25)){
              anyVisible = true;
              break;
            }
          }
          if(anyVisible == false){
            last_invisible = i;
            break;
          }
          last_visible = i;
        }
        // cout << "last_visible " << last_visible << " last_invisible " << last_invisible << endl;
        CartesianPoint last_invisible_point = CartesianPoint(positionHis->at(positionHis->size()-last_invisible).getX(), positionHis->at(positionHis->size()-last_invisible).getY());
        CartesianPoint last_visible_point = CartesianPoint(positionHis->at(positionHis->size()-last_visible).getX(), positionHis->at(positionHis->size()-last_visible).getY());
        // cout << "last_invisible_point " << last_invisible_point.get_x() << " " << last_invisible_point.get_y() << endl;
        // cout << "last_visible_point " << last_visible_point.get_x() << " " << last_visible_point.get_y() << endl;
        beliefs->getAgentState()->setGetOutTriggered(true);
        // cout << "after setGetOutTriggered" << endl;
        beliefs->getAgentState()->setFarthestPoint(last_invisible_point);
        // cout << "after setFarthestPoint" << endl;
        beliefs->getAgentState()->setIntermediatePoint(last_visible_point);
        // cout << "after setIntermediatePoint" << endl;
        // (*decision) = beliefs->getAgentState()->moveTowards(last_visible_point);
        // // RCLCPP_DEBUG(this->get_logger(), "last_visible_point in sight and no obstacles, get out advisor to take decision");
        // decisionMade = true;
        // cout << "after decisionMade" << endl;
        int end_waypoint = last_invisible + 2;
        if(end_waypoint > positionHis->size()){
          end_waypoint = positionHis->size();
        }
        // cout << "end_waypoint " << end_waypoint << endl;
        std::vector<CartesianPoint> trailPositions;
        trailPositions.push_back(CartesianPoint(positionHis->at(positionHis->size()-end_waypoint).getX(), positionHis->at(positionHis->size()-end_waypoint).getY()));
        // Find the furthest point on path that can be seen from current position, push that point to trail and then move to that point
        for(int i = end_waypoint; i >= 1; i--){
          for(int j = 1; j < i; j++){
            if(canAccessPoint(laserHis->at(positionHis->size()-i), CartesianPoint(positionHis->at(positionHis->size()-i).getX(), positionHis->at(positionHis->size()-i).getY()), CartesianPoint(positionHis->at(positionHis->size()-j).getX(), positionHis->at(positionHis->size()-j).getY()), 3)){
              trailPositions.push_back(CartesianPoint(positionHis->at(positionHis->size()-j).getX(), positionHis->at(positionHis->size()-j).getY()));
              i = j+1;
            }
          }
          // cout << i << endl;
        }
        for(int i = 0; i < trailPositions.size(); i++){
          beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(trailPositions[i], 2);
        }
      }
      else{
        // cout << "else make turn" << endl;
        (*decision) = FORRAction(RIGHT_TURN, rotation_set->size()/2);
        // RCLCPP_DEBUG(this->get_logger(), "Rotate in place, get out advisor to take decision");
        decisionMade = true;
      }
    }
  }
  // cout << "decisionMade " << decisionMade << endl;
  return decisionMade;
}

bool Tier1Advisor::advisorDoorway(FORRAction *decision){
  // RCLCPP_DEBUG(this->get_logger(), "In advisor doorway");
  bool decisionMade = false;
  set<FORRAction> *vetoedActions = beliefs->getAgentState()->getVetoedActions();
  if(beliefs->getAgentState()->getRepositionTriggered()){
    cout << "Reposition already triggered, move towards point" << endl;
    (*decision) = beliefs->getAgentState()->moveTowards(beliefs->getAgentState()->getRepositionPoint());
    if(decision->parameter != 0){
      Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
      if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
        if(decision->type == RIGHT_TURN or decision->type == LEFT_TURN){
          FORRAction forrAction = *decision;
          if(std::find(vetoedActions->begin(), vetoedActions->end(), forrAction) != vetoedActions->end()){
            // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << forrAction.type << " " << forrAction.parameter);
          }
          else{
            // RCLCPP_DEBUG(this->get_logger(), "Reposition point in sight and no obstacles, Doorway advisor to take decision");
            decisionMade = true;
          }
        }
        else{
          FORRAction forward = beliefs->getAgentState()->maxForwardAction();
          if(forward.parameter >= decision->parameter){
            if(decision->parameter > 4){
              (*decision) = FORRAction(FORWARD, 4);
            }
            // RCLCPP_DEBUG(this->get_logger(), "Reposition point in sight and no obstacles, Doorway advisor to take decision");
            decisionMade = true;
          }
        }
      }
    }
    if(decisionMade == false){
      beliefs->getAgentState()->increaseRepositionCount();
    }
  }
  else{
    CartesianPoint task(beliefs->getAgentState()->getCurrentTask()->getTaskX(),beliefs->getAgentState()->getCurrentTask()->getTaskY());
    // RCLCPP_DEBUG(this->get_logger(), "Check if target or waypoint can be spotted using laser scan");
    cout << "Target = " << task.get_x() << " " << task.get_y() << endl;
    bool targetInSight = beliefs->getAgentState()->canSeePoint(task, 20);
    CartesianPoint waypoint(beliefs->getAgentState()->getCurrentTask()->getX(),beliefs->getAgentState()->getCurrentTask()->getY());
    cout << "Waypoint = " << waypoint.get_x() << " " << waypoint.get_y() << endl;
    bool waypointInSight = beliefs->getAgentState()->canSeePoint(waypoint, 20);
    cout << "targetInSight " << targetInSight << " waypointInSight " << waypointInSight << endl;
    CartesianPoint subgoal;
    if(targetInSight){
      subgoal = task;
    }
    else if(waypointInSight){
      subgoal = waypoint;
    }
    if(targetInSight or waypointInSight){
      std::vector<CartesianPoint> givenLaserEndpoints = beliefs->getAgentState()->getCurrentLaserEndpoints();
      CartesianPoint laserPos = CartesianPoint(beliefs->getAgentState()->getCurrentPosition().getX(), beliefs->getAgentState()->getCurrentPosition().getY());
      double point_direction = atan2((subgoal.get_y() - laserPos.get_y()), (subgoal.get_x() - laserPos.get_x()));
      cout << "Robot position " << laserPos.get_x() << " " << laserPos.get_y() << " " << beliefs->getAgentState()->getCurrentPosition().getTheta() << " point_direction " << point_direction << endl;
      int index = 0;
      double min_angle = 100000;
      for(int i = 0; i < givenLaserEndpoints.size(); i++){
        double laser_direction = atan2((givenLaserEndpoints[i].get_y() - laserPos.get_y()), (givenLaserEndpoints[i].get_x() - laserPos.get_x()));
        double angle_diff = laser_direction - point_direction;
        if(angle_diff > M_PI)
          angle_diff = angle_diff - 2*M_PI;
        if(angle_diff < -M_PI)
          angle_diff = angle_diff + 2*M_PI;
        angle_diff = fabs(angle_diff);
        // cout << "point_direction " << point_direction << " laser_direction " << laser_direction << " angle_diff " << angle_diff << endl;
        if(angle_diff < min_angle){
          // cout << "Laser Direction : " << laser_direction << ", Point Direction : " << point_direction << endl;
          min_angle = angle_diff;
          index = i;
        }
      }
      if(index + 1 >= givenLaserEndpoints.size()){
        index = index - 1;
      }
      cout << "min_angle " << min_angle << " index " << index << " endpoint " << givenLaserEndpoints[index].get_x() << " " << givenLaserEndpoints[index].get_y() << endl;
      double distLeft = 0, xLeft = 0, yLeft = 0, countLeft = 0;
      double distRight = 0, xRight = 0, yRight = 0, countRight = 0;
      for(int i = 0; i < index; i++){
        double dist_to_endpoint = givenLaserEndpoints[i].get_distance(laserPos);
        distLeft += dist_to_endpoint;
        countLeft++;
        xLeft += givenLaserEndpoints[i].get_x();
        yLeft += givenLaserEndpoints[i].get_y();
      }
      for(int i = index + 1; i < givenLaserEndpoints.size(); i++){
        double dist_to_endpoint = givenLaserEndpoints[i].get_distance(laserPos);
        distRight += dist_to_endpoint;
        countRight++;
        xRight += givenLaserEndpoints[i].get_x();
        yRight += givenLaserEndpoints[i].get_y();
      }
      cout << "distLeft " << distLeft << " xLeft " << xLeft << " yLeft " << yLeft << " countLeft " << countLeft << " distRight " << distRight << " xRight " << xRight << " yRight " << yRight << " countRight " << countRight << endl;
      distLeft = distLeft / countLeft;
      xLeft = xLeft / countLeft;
      yLeft = yLeft / countLeft;
      distRight = distRight / countRight;
      xRight = xRight / countRight;
      yRight = yRight / countRight;
      cout << "distLeft " << distLeft << " xLeft " << xLeft << " yLeft " << yLeft << " countLeft " << countLeft << " distRight " << distRight << " xRight " << xRight << " yRight " << yRight << " countRight " << countRight << endl;
      if(distLeft > distRight){
        double tx, ty;
        for(double j = 0; j <= 1; j += 0.1){
          tx = (laserPos.get_x() * j) + (xLeft * (1 - j));
          ty = (laserPos.get_y() * j) + (yLeft * (1 - j));
          if(beliefs->getAgentState()->canSeePoint(CartesianPoint(tx, ty), 20)){
            break;
          }
        }
        beliefs->getAgentState()->setRepositionPoint(CartesianPoint(tx, ty));
        beliefs->getAgentState()->setRepositionTriggered(true);
        beliefs->getAgentState()->setRepositionCount(0);
      }
      else{
        double tx, ty;
        for(double j = 0; j <= 1; j += 0.1){
          tx = (laserPos.get_x() * j) + (xRight * (1 - j));
          ty = (laserPos.get_y() * j) + (yRight * (1 - j));
          if(beliefs->getAgentState()->canSeePoint(CartesianPoint(tx, ty), 20)){
            break;
          }
        }
        beliefs->getAgentState()->setRepositionPoint(CartesianPoint(tx, ty));
        beliefs->getAgentState()->setRepositionTriggered(true);
        beliefs->getAgentState()->setRepositionCount(0);
      }
      if(beliefs->getAgentState()->getRepositionTriggered()){
        cout << "Reposition triggered, move towards point" << endl;
        (*decision) = beliefs->getAgentState()->moveTowards(beliefs->getAgentState()->getRepositionPoint());
        if(decision->parameter != 0){
          Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
          if(expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
            if(decision->type == RIGHT_TURN or decision->type == LEFT_TURN){
              FORRAction forrAction = *decision;
              if(std::find(vetoedActions->begin(), vetoedActions->end(), forrAction) != vetoedActions->end()){
                // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << forrAction.type << " " << forrAction.parameter);
              }
              else{
                // RCLCPP_DEBUG(this->get_logger(), "Reposition point in sight and no obstacles, Doorway advisor to take decision");
                decisionMade = true;
              }
            }
            else{
              FORRAction forward = beliefs->getAgentState()->maxForwardAction();
              if(forward.parameter >= decision->parameter){
                if(decision->parameter > 4){
                  (*decision) = FORRAction(FORWARD, 4);
                }
                // RCLCPP_DEBUG(this->get_logger(), "Reposition point in sight and no obstacles, Doorway advisor to take decision");
                decisionMade = true;
              }
            }
          }
        }
      }
    }
  }
  return decisionMade;
}

bool Tier1Advisor::advisorBehindYou(FORRAction *decision){
  // RCLCPP_DEBUG(this->get_logger(), "In advisor BehindYou");
  bool decisionMade = false;
  set<FORRAction> *vetoedActions = beliefs->getAgentState()->getVetoedActions();
  double behind_radius = 1.5;
  // RCLCPP_DEBUG(this->get_logger(), "Check if waypoint can be spotted using laser scan");
  CartesianPoint waypoint(beliefs->getAgentState()->getCurrentTask()->getX(),beliefs->getAgentState()->getCurrentTask()->getY());
  cout << "Waypoint = " << waypoint.get_x() << " " << waypoint.get_y() << endl;
  bool waypointInSight = beliefs->getAgentState()->canSeePoint(waypoint, 20);
  cout << "waypointInSight " << waypointInSight << endl;
  CartesianPoint robotPos = CartesianPoint(beliefs->getAgentState()->getCurrentPosition().getX(), beliefs->getAgentState()->getCurrentPosition().getY());
  vector<FORRAction> actions = beliefs->getAgentState()->getCurrentTask()->getPreviousDecisions();
  set<FORRAction> *rotation_set = beliefs->getAgentState()->getRotationActionSet();
  int size = actions.size();
  FORRAction lastAction;
  vector<Position> *positionHis = beliefs->getAgentState()->getAllPositionTrace();
  vector< vector <CartesianPoint> > *laserHis = beliefs->getAgentState()->getAllLaserHistory();
  CartesianPoint last_position;
  vector<CartesianPoint> last_endpoint;
  if(size > 0){
    lastAction = actions[size - 1];
    last_position = CartesianPoint(positionHis->at(positionHis->size()-2).getX(), positionHis->at(positionHis->size()-2).getY());
    last_endpoint = laserHis->at(laserHis->size()-2);
  }
  else{
    lastAction = FORRAction(PAUSE, 0);
    last_position = robotPos;
    last_endpoint = beliefs->getAgentState()->getCurrentLaserEndpoints();
  }
  if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getType() == 0){
    if(beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getRegion().getRadius() > behind_radius){
      behind_radius = beliefs->getAgentState()->getCurrentTask()->getSkeletonWaypoint().getRegion().getRadius() + behind_radius;
    }
  }
  if(robotPos.get_distance(waypoint) <= behind_radius and !waypointInSight and !(lastAction == FORRAction(RIGHT_TURN, rotation_set->size()/2)) and !(lastAction == FORRAction(LEFT_TURN, rotation_set->size()/2)) and !canAccessPoint(last_endpoint, last_position, waypoint, 20)){
    FORRAction forrAction = FORRAction(RIGHT_TURN, rotation_set->size()/2);
    if(std::find(vetoedActions->begin(), vetoedActions->end(), forrAction) != vetoedActions->end()){
      // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << forrAction.type << " " << forrAction.parameter);
      forrAction = FORRAction(LEFT_TURN, rotation_set->size()/2);
      if(std::find(vetoedActions->begin(), vetoedActions->end(), forrAction) != vetoedActions->end()){
        // RCLCPP_DEBUG_STREAM(this->get_logger(), "Vetoed action : " << forrAction.type << " " << forrAction.parameter);
      }
      else{
        (*decision) = FORRAction(LEFT_TURN, rotation_set->size()/2);
        // RCLCPP_DEBUG(this->get_logger(), "Waypoint not in sight but close, BehindYou advisor to turn around");
        decisionMade = true;
      }
    }
    else{
      (*decision) = FORRAction(RIGHT_TURN, rotation_set->size()/2);
      // RCLCPP_DEBUG(this->get_logger(), "Waypoint not in sight but close, BehindYou advisor to turn around");
      decisionMade = true;
    }
  }
  return decisionMade;
}


bool Tier1Advisor::advisorFindAWay(FORRAction *decision){
  // RCLCPP_DEBUG(this->get_logger(), "In advisor FindAWay");
  bool decisionMade = false;
  double search_radius = 7.0;
  localExploration->setPathPlanner(beliefs->getAgentState()->getCurrentTask()->getPathPlanner());
  if(beliefs->getAgentState()->getCurrentTask()->getPlanSize() == 0 or !beliefs->getAgentState()->getCurrentTask()->getIsPlanActive() or localExploration->getAlreadyStarted()){
    cout << "No active plan, try to do local exploration" << endl;
    if(localExploration->getAlreadyStarted()){
      cout << "Exploration already started" << endl;
      CartesianPoint current(beliefs->getAgentState()->getCurrentPosition().getX(), beliefs->getAgentState()->getCurrentPosition().getY());
      CartesianPoint task(beliefs->getAgentState()->getCurrentTask()->getTaskX(),beliefs->getAgentState()->getCurrentTask()->getTaskY());
      if(!beliefs->getAgentState()->canSeePoint(CartesianPoint(beliefs->getAgentState()->getCurrentTask()->getX(), beliefs->getAgentState()->getCurrentTask()->getY()), 25)){
        beliefs->getAgentState()->increaseFindAWayCount();
      }
      if((localExploration->atEndOfPotential(current) or beliefs->getAgentState()->getFindAWayCount() >= 4 or beliefs->getAgentState()->getCurrentTask()->getPlanSize() == 0) and beliefs->getAgentState()->getGetOutTriggered() == false){
        cout << "At end of current potential " << localExploration->atEndOfPotential(current) << " cannot see next waypoint of potential " << beliefs->getAgentState()->getFindAWayCount() << endl;
        beliefs->getAgentState()->getCurrentTask()->clearWaypoints(3);
        cout << "waypoints cleared" << endl;
        beliefs->getAgentState()->setFindAWayCount(0);
        // vector< LineSegment > potential_exploration;
        // vector<CartesianPoint> laserEndpoints = beliefs->getAgentState()->getCurrentLaserEndpoints();
        // for(int i = 0; i < laserEndpoints.size(); i++){
        //   LineSegment pair = LineSegment(current, laserEndpoints[i]);
        //   if(distance(task, pair) < search_radius){
        //     potential_exploration.push_back(pair);
        //   }
        // }
        // localExploration->addToQueue(potential_exploration);
        cout << "generate new potentials" << endl;
        if(!localExploration->getFinishedPotentials()){
          cout << "finished current potential, go to next" << endl;
          localExploration->atStartOfPotential(CartesianPoint(beliefs->getAgentState()->getCurrentPosition().getX(), beliefs->getAgentState()->getCurrentPosition().getY()));
          if(localExploration->getAtStartOfPotential()){
            cout << "go to end of current potential" << endl;
            vector<CartesianPoint> waypoints = localExploration->getPathToEnd();
            for(int i = waypoints.size()-1; i >= 0; i--){
              // cout << "waypoint " << waypoints[i].get_x() << " " << waypoints[i].get_y() << endl;
              beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(waypoints[i], 3);
            }
          }
          else{
            cout << "get to start of current potential 1" << endl;
            vector<CartesianPoint> end_waypoints = localExploration->getPathToEnd();
            cout << "end_waypoints size " << end_waypoints.size() << endl;
            for(int i = end_waypoints.size()-1; i >= 0; i--){
              cout << "waypoint " << end_waypoints[i].get_x() << " " << end_waypoints[i].get_y() << endl;
              beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(end_waypoints[i], 3);
            }
            vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
            bool currently_in_region = false;
            bool start_in_region = false;
            for(int i = 0; i < regions.size(); i++){
              if(regions[i].inRegion(current)){
                currently_in_region = true;
              }
              if(regions[i].inRegion(localExploration->getStartOfPotential())){
                start_in_region = true;
              }
              if(currently_in_region and start_in_region){
                break;
              }
            }
            // cout << "currently_in_region " << currently_in_region << " start_in_region " << start_in_region << endl;
            if(currently_in_region and start_in_region){
              vector<CartesianPoint> waypoints = localExploration->getPathToStart(current);
              for(int i = waypoints.size()-1; i >= 0; i--){
                // cout << "waypoint " << waypoints[i].get_x() << " " << waypoints[i].get_y() << endl;
                beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(waypoints[i], 3);
              }
            }
            else{
              vector<Position> *positionHis = beliefs->getAgentState()->getAllPositionTrace();
              bool found_recent_in_region = false;
              CartesianPoint new_start_region;
              bool found_recent_nearby = false;
              CartesianPoint new_start_nearby;
              int new_start_region_ind = -1;
              int new_start_nearby_ind = -1;
              // cout << "positionHis " << positionHis->size() << endl;
              for(int i = 1; i < positionHis->size(); i++){
                if(found_recent_in_region == false){
                  for(int j = 0; j < regions.size(); j++){
                    if(regions[j].inRegion(CartesianPoint(positionHis->at(positionHis->size()-i).getX(), positionHis->at(positionHis->size()-i).getY()))){
                      new_start_region = CartesianPoint(positionHis->at(positionHis->size()-i).getX(), positionHis->at(positionHis->size()-i).getY());
                      new_start_region_ind = positionHis->size()-i;
                      found_recent_in_region = true;
                      break;
                    }
                  }
                }
                if(found_recent_nearby == false){
                  if(current.get_distance(localExploration->getStartOfPotential()) < 0.75){
                    new_start_nearby = CartesianPoint(positionHis->at(positionHis->size()-i).getX(), positionHis->at(positionHis->size()-i).getY());
                    new_start_nearby_ind = positionHis->size()-i;
                    found_recent_nearby = true;
                  }
                }
                if(found_recent_in_region == true and found_recent_nearby == true){
                  break;
                }
              }
              // cout << "found_recent_in_region " << found_recent_in_region << " found_recent_nearby " << found_recent_nearby << " new_start_region_ind " << new_start_region_ind << " new_start_nearby_ind "<< new_start_nearby_ind << endl;
              if(found_recent_in_region == true and (new_start_region_ind >= new_start_nearby_ind or found_recent_nearby == false)){
                vector<CartesianPoint> waypoints = localExploration->getPathToStart(new_start_region);
                for(int i = waypoints.size()-1; i >= 0; i--){
                  // cout << "waypoint " << waypoints[i].get_x() << " " << waypoints[i].get_y() << endl;
                  beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(waypoints[i], 3);
                }
                vector< vector <CartesianPoint> > *laserHis = beliefs->getAgentState()->getAllLaserHistory();
                std::vector<CartesianPoint> trailPositions;
                trailPositions.push_back(new_start_region);
                // Find the furthest point on path that can be seen from current position, push that point to trail and then move to that point
                for(int i = new_start_region_ind; i < positionHis->size(); i++){
                  for(int j = positionHis->size()-1; j > i; j--){
                    if(canAccessPoint(laserHis->at(i), CartesianPoint(positionHis->at(i).getX(), positionHis->at(i).getY()), CartesianPoint(positionHis->at(j).getX(), positionHis->at(j).getY()), 2) or j == i+1){
                      trailPositions.push_back(CartesianPoint(positionHis->at(j).getX(), positionHis->at(j).getY()));
                      i = j;
                    }
                  }
                  // cout << i << endl;
                }
                trailPositions.push_back(current);
                // cout << "trailPositions " << trailPositions.size() << endl;
                for(int i = trailPositions.size()-1; i >= 0; i--){
                  // cout << "waypoint " << trailPositions[i].get_x() << " " << trailPositions[i].get_y() << endl;
                  beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(trailPositions[i], 3);
                }
                // std::vector<CartesianPoint> trailPositions;
                // trailPositions.push_back(CartesianPoint(positionHis->at(positionHis->size()-new_start_region_ind).getX(), positionHis->at(positionHis->size()-new_start_region_ind).getY()));
                // // Find the furthest point on path that can be seen from current position, push that point to trail and then move to that point
                // for(int i = new_start_region_ind; i >= 1; i--){
                //   for(int j = 1; j < i; j++){
                //     if(canAccessPoint(laserHis->at(positionHis->size()-i), CartesianPoint(positionHis->at(positionHis->size()-i).getX(), positionHis->at(positionHis->size()-i).getY()), CartesianPoint(positionHis->at(positionHis->size()-j).getX(), positionHis->at(positionHis->size()-j).getY()), 3)){
                //       trailPositions.push_back(CartesianPoint(positionHis->at(positionHis->size()-j).getX(), positionHis->at(positionHis->size()-j).getY()));
                //       i = j+1;
                //     }
                //   }
                //   // cout << i << endl;
                // }
                // cout << "trailPositions " << trailPositions.size() << endl;
                // for(int i = 0; i < trailPositions.size(); i++){
                //   cout << "waypoint " << trailPositions[i].get_x() << " " << trailPositions[i].get_y() << endl;
                //   beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(trailPositions[i], 3);
                // }
              }
              else if(found_recent_nearby == true and (new_start_region_ind <= new_start_nearby_ind or found_recent_in_region == false)){
                vector< vector <CartesianPoint> > *laserHis = beliefs->getAgentState()->getAllLaserHistory();
                std::vector<CartesianPoint> trailPositions;
                trailPositions.push_back(new_start_nearby);
                // Find the furthest point on path that can be seen from current position, push that point to trail and then move to that point
                for(int i = new_start_nearby_ind; i < positionHis->size(); i++){
                  for(int j = positionHis->size()-1; j > i; j--){
                    if(canAccessPoint(laserHis->at(i), CartesianPoint(positionHis->at(i).getX(), positionHis->at(i).getY()), CartesianPoint(positionHis->at(j).getX(), positionHis->at(j).getY()), 2) or j == i+1){
                      trailPositions.push_back(CartesianPoint(positionHis->at(j).getX(), positionHis->at(j).getY()));
                      i = j;
                    }
                  }
                  // cout << i << endl;
                }
                trailPositions.push_back(current);
                // cout << "trailPositions " << trailPositions.size() << endl;
                for(int i = trailPositions.size()-1; i >= 0; i--){
                  // cout << "waypoint " << trailPositions[i].get_x() << " " << trailPositions[i].get_y() << endl;
                  beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(trailPositions[i], 3);
                }
                // std::vector<CartesianPoint> trailPositions;
                // trailPositions.push_back(CartesianPoint(positionHis->at(positionHis->size()-new_start_nearby_ind).getX(), positionHis->at(positionHis->size()-new_start_nearby_ind).getY()));
                // // Find the furthest point on path that can be seen from current position, push that point to trail and then move to that point
                // for(int i = new_start_nearby_ind; i >= 1; i--){
                //   for(int j = 1; j < i; j++){
                //     if(canAccessPoint(laserHis->at(positionHis->size()-i), CartesianPoint(positionHis->at(positionHis->size()-i).getX(), positionHis->at(positionHis->size()-i).getY()), CartesianPoint(positionHis->at(positionHis->size()-j).getX(), positionHis->at(positionHis->size()-j).getY()), 3)){
                //       trailPositions.push_back(CartesianPoint(positionHis->at(positionHis->size()-j).getX(), positionHis->at(positionHis->size()-j).getY()));
                //       i = j+1;
                //     }
                //   }
                //   // cout << i << endl;
                // }
                // cout << "trailPositions " << trailPositions.size() << endl;
                // for(int i = 0; i < trailPositions.size(); i++){
                //   cout << "waypoint " << trailPositions[i].get_x() << " " << trailPositions[i].get_y() << endl;
                //   beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(trailPositions[i], 3);
                // }
              }
            }
          }
        }
        else{
          cout << "no more potentials, randomly explore" << endl;
          vector<CartesianPoint> end_waypoints = localExploration->randomExploration(current, beliefs->getAgentState()->getCurrentLaserEndpoints(), task);
          for(int i = end_waypoints.size()-1; i >= 0; i--){
            // cout << "waypoint " << end_waypoints[i].get_x() << " " << end_waypoints[i].get_y() << endl;
            beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(end_waypoints[i], 3);
          }
        }
      }
    }
    else{
      CartesianPoint task(beliefs->getAgentState()->getCurrentTask()->getTaskX(),beliefs->getAgentState()->getCurrentTask()->getTaskY());
      cout << "Target = " << task.get_x() << " " << task.get_y() << endl;
      CartesianPoint current(beliefs->getAgentState()->getCurrentPosition().getX(), beliefs->getAgentState()->getCurrentPosition().getY());
      vector< vector<Position> > remaining_candidates = beliefs->getAgentState()->getRemainingCandidates();
      vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
      cout << "remaining_candidates " << remaining_candidates.size() << " regions " << regions.size() << endl;
      vector< LineSegment > potential_exploration;
      for(int i = 0; i < remaining_candidates.size(); i++){
        LineSegment pair = LineSegment(CartesianPoint(remaining_candidates[i][0].getX(), remaining_candidates[i][0].getY()), CartesianPoint(remaining_candidates[i][1].getX(), remaining_candidates[i][1].getY()));
        if(distance(task, pair) < search_radius){
          potential_exploration.push_back(pair);
          if(pair.get_length() > 25){
            // cout << "remaining_candidates " << i << " length " << pair.get_length() << endl;
          }
        }
        // if(task.get_distance(remaining_candidates[i][0].getX(), remaining_candidates[i][0].gety()) < search_radius or task.get_distance(remaining_candidates[i][1].getX(), remaining_candidates[i][1].gety()) < search_radius or task.get_distance((remaining_candidates[i][0].getX() + remaining_candidates[i][1].getX())/2, (remaining_candidates[i][0].gety() + remaining_candidates[i][1].gety())/2) < search_radius){
        //   vector<CartesianPoint> potential;
        //   potential.push_back(CartesianPoint(remaining_candidates[i][0].getX(), remaining_candidates[i][0].gety()));
        //   potential.push_back(CartesianPoint(remaining_candidates[i][1].getX(), remaining_candidates[i][1].gety()));
        //   potential_exploration.push_back(potential);
        // }
      }
      int dimension = beliefs->getAgentState()->getCurrentTask()->getDimension();
      vector< vector<int> > coverage_grid;
      for(int i = 0; i < dimension; i++){
        vector<int> col;
        for(int j = 0; j < dimension; j++){
          col.push_back(-1);
        }
        coverage_grid.push_back(col);
      }
      bool currently_in_region = false;
      bool start_in_region = false;
      for(int i = 0; i < regions.size(); i++){
        if(current.get_distance(regions[i].getCenter()) - regions[i].getRadius() <= 25){
          vector<LineSegment> vis_segments = regions[i].getVisibilityLineSegments();
          // double min_distance = search_radius;
          // LineSegment min_segment;
          for(int j = 0; j < vis_segments.size(); j++){
            double dist_to_target = distance(task, vis_segments[j]);
            // if(dist_to_target < search_radius and dist_to_target < min_distance){
            if(dist_to_target < search_radius){
              potential_exploration.push_back(vis_segments[j]);
              // min_segment = vis_segments[j];
              // min_distance = dist_to_target;
              if(vis_segments[j].get_length() > 25){
                // cout << "region " << i << " vis_segments " << j << " length " << vis_segments[j].get_length() << endl;
              }
            }
          }
          // if(min_distance < search_radius){
          //   potential_exploration.push_back(min_segment);
          // }
        }
        if(regions[i].inRegion(current)){
          currently_in_region = true;
        }
        if(regions[i].inRegion(localExploration->getStartOfPotential())){
          start_in_region = true;
        }
        double cx = regions[i].getCenter().get_x();
        double cy = regions[i].getCenter().get_y();
        double cr = regions[i].getRadius();
        for(double x = cx - cr; x <= cx + cr; x += cr/10){
          for(double y = cy - cr; y <= cy + cr; y += cr/10){
            coverage_grid[(int)(x)][(int)(y)] = i+1;
          }
        }
        vector<FORRExit> exits = regions[i].getExits();
        for(int j = 0; j < exits.size(); j++){
          vector<CartesianPoint> path = exits[j].getConnectionPoints();
          for(int k = 0; k < path.size()-1; k++){
            double tx, ty;
            for(double j = 0; j <= 1; j += 0.1){
              tx = (path[k+1].get_x() * j) + (path[k].get_x() * (1 - j));
              ty = (path[k+1].get_y() * j) + (path[k].get_y() * (1 - j));
              coverage_grid[(int)(tx)][(int)(ty)] = i+1;
            }
          }
        }
      }
      localExploration->setCoverage(coverage_grid);
      vector<CartesianPoint> laserEndpoints = beliefs->getAgentState()->getCurrentLaserEndpoints();
      for(int i = 0; i < laserEndpoints.size(); i++){
        LineSegment pair = LineSegment(current, laserEndpoints[i]);
        if(distance(task, pair) < search_radius){
          potential_exploration.push_back(pair);
          if(pair.get_length() > 25){
            // cout << "laserEndpoints " << i << " length " << pair.get_length() << endl;
          }
        }
      }
      vector<Position> *taskPositionHis = beliefs->getAgentState()->getCurrentTask()->getPositionHistory();
      vector< vector <CartesianPoint> > *taskLaserHis = beliefs->getAgentState()->getCurrentTask()->getLaserHistory();
      for(int i = 1; i < taskPositionHis->size(); i++){
        vector<CartesianPoint> taskLaserEndpoints = taskLaserHis->at(taskLaserHis->size()-i);
        for(int j = 0; j < taskLaserEndpoints.size(); j++){
          LineSegment pair = LineSegment(CartesianPoint(taskPositionHis->at(taskPositionHis->size()-i).getX(), taskPositionHis->at(taskPositionHis->size()-i).getY()), taskLaserEndpoints[j]);
          if(distance(task, pair) < search_radius){
            potential_exploration.push_back(pair);
            if(pair.get_length() > 25){
              // cout << "decisionpoint " << i << " laserEndpoints " << j << " length " << pair.get_length() << endl;
            }
          }
        }
      }
      cout << "potential_exploration: " << potential_exploration.size() << endl;
      if(potential_exploration.size() > 0){
        localExploration->setQueue(task, potential_exploration);
        localExploration->atStartOfPotential(current);
        if(localExploration->getAtStartOfPotential()){
          // cout << "go to end of current potential" << endl;
          vector<CartesianPoint> waypoints = localExploration->getPathToEnd();
          for(int i = waypoints.size()-1; i >= 0; i--){
            // cout << "waypoint " << waypoints[i].get_x() << " " << waypoints[i].get_y() << endl;
            beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(waypoints[i], 3);
          }
        }
        else{
          cout << "get to start of current potential 2" << endl;
          vector<CartesianPoint> end_waypoints = localExploration->getPathToEnd();
          cout << "end_waypoints size " << end_waypoints.size() << endl;
          for(int i = end_waypoints.size()-1; i >= 0; i--){
            cout << "waypoint " << end_waypoints[i].get_x() << " " << end_waypoints[i].get_y() << endl;
            beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(end_waypoints[i], 3);
          }
          //CHECK IF CURRENT IN REGION OTHERWISE FOLLOW PATH TRAIL BACK TO REGION
          cout << "currently_in_region " << currently_in_region << " start_in_region " << start_in_region << endl;
          if(currently_in_region == true and start_in_region == true){
            vector<CartesianPoint> waypoints = localExploration->getPathToStart(current);
            for(int i = waypoints.size()-1; i >= 0; i--){
              cout << "waypoint " << waypoints[i].get_x() << " " << waypoints[i].get_y() << endl;
              beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(waypoints[i], 3);
            }
          }
          else{
            vector<Position> *positionHis = beliefs->getAgentState()->getAllPositionTrace();
            bool found_recent_in_region = false;
            CartesianPoint new_start_region;
            bool found_recent_nearby = false;
            CartesianPoint new_start_nearby;
            int new_start_region_ind = -1;
            int new_start_nearby_ind = -1;
            cout << "positionHis " << positionHis->size() << endl;
            for(int i = 1; i < positionHis->size(); i++){
              if(found_recent_in_region == false){
                for(int j = 0; j < regions.size(); j++){
                  if(regions[j].inRegion(CartesianPoint(positionHis->at(positionHis->size()-i).getX(), positionHis->at(positionHis->size()-i).getY()))){
                    new_start_region = CartesianPoint(positionHis->at(positionHis->size()-i).getX(), positionHis->at(positionHis->size()-i).getY());
                    new_start_region_ind = positionHis->size()-i;
                    found_recent_in_region = true;
                    break;
                  }
                }
              }
              if(found_recent_nearby == false){
                if(current.get_distance(localExploration->getStartOfPotential()) < 0.75){
                  new_start_nearby = CartesianPoint(positionHis->at(positionHis->size()-i).getX(), positionHis->at(positionHis->size()-i).getY());
                  new_start_nearby_ind = positionHis->size()-i;
                  found_recent_nearby = true;
                }
              }
              if(found_recent_in_region == true or found_recent_nearby == true){
                break;
              }
            }
            cout << "found_recent_in_region " << found_recent_in_region << " found_recent_nearby " << found_recent_nearby << " new_start_region_ind " << new_start_region_ind << " new_start_nearby_ind "<< new_start_nearby_ind << endl;
            if(found_recent_in_region == true and (new_start_region_ind >= new_start_nearby_ind or found_recent_nearby == false)){
              vector<CartesianPoint> waypoints = localExploration->getPathToStart(new_start_region);
              for(int i = waypoints.size()-1; i >= 0; i--){
                cout << "waypoint " << waypoints[i].get_x() << " " << waypoints[i].get_y() << endl;
                beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(waypoints[i], 3);
              }
              vector< vector <CartesianPoint> > *laserHis = beliefs->getAgentState()->getAllLaserHistory();
              std::vector<CartesianPoint> trailPositions;
              trailPositions.push_back(new_start_region);
              // Find the furthest point on path that can be seen from current position, push that point to trail and then move to that point
              for(int i = new_start_region_ind; i < positionHis->size(); i++){
                for(int j = positionHis->size()-1; j > i; j--){
                  if(canAccessPoint(laserHis->at(i), CartesianPoint(positionHis->at(i).getX(), positionHis->at(i).getY()), CartesianPoint(positionHis->at(j).getX(), positionHis->at(j).getY()), 2) or j == i+1){
                    trailPositions.push_back(CartesianPoint(positionHis->at(j).getX(), positionHis->at(j).getY()));
                    i = j;
                  }
                }
                // cout << i << endl;
              }
              trailPositions.push_back(current);
              // cout << "trailPositions " << trailPositions.size() << endl;
              for(int i = trailPositions.size()-1; i >= 0; i--){
                // cout << "waypoint " << trailPositions[i].get_x() << " " << trailPositions[i].get_y() << endl;
                beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(trailPositions[i], 3);
              }
              // std::vector<CartesianPoint> trailPositions;
              // trailPositions.push_back(CartesianPoint(positionHis->at(positionHis->size()-new_start_region_ind).getX(), positionHis->at(positionHis->size()-new_start_region_ind).getY()));
              // // Find the furthest point on path that can be seen from current position, push that point to trail and then move to that point
              // for(int i = new_start_region_ind; i >= 1; i--){
              //   for(int j = 1; j < i; j++){
              //     if(canAccessPoint(laserHis->at(positionHis->size()-i), CartesianPoint(positionHis->at(positionHis->size()-i).getX(), positionHis->at(positionHis->size()-i).getY()), CartesianPoint(positionHis->at(positionHis->size()-j).getX(), positionHis->at(positionHis->size()-j).getY()), 3)){
              //       trailPositions.push_back(CartesianPoint(positionHis->at(positionHis->size()-j).getX(), positionHis->at(positionHis->size()-j).getY()));
              //       i = j+1;
              //     }
              //   }
              //   // cout << i << endl;
              // }
              // cout << "trailPositions " << trailPositions.size() << endl;
              // for(int i = 0; i < trailPositions.size(); i++){
              //   cout << "waypoint " << trailPositions[i].get_x() << " " << trailPositions[i].get_y() << endl;
              //   beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(trailPositions[i], 3);
              // }
            }
            else if(found_recent_nearby == true and (new_start_region_ind <= new_start_nearby_ind or found_recent_in_region == false)){
              vector< vector <CartesianPoint> > *laserHis = beliefs->getAgentState()->getAllLaserHistory();
              std::vector<CartesianPoint> trailPositions;
              trailPositions.push_back(new_start_nearby);
              // Find the furthest point on path that can be seen from current position, push that point to trail and then move to that point
              for(int i = new_start_nearby_ind; i < positionHis->size(); i++){
                for(int j = positionHis->size()-1; j > i; j--){
                  if(canAccessPoint(laserHis->at(i), CartesianPoint(positionHis->at(i).getX(), positionHis->at(i).getY()), CartesianPoint(positionHis->at(j).getX(), positionHis->at(j).getY()), 2) or j == i+1){
                    trailPositions.push_back(CartesianPoint(positionHis->at(j).getX(), positionHis->at(j).getY()));
                    i = j;
                  }
                }
                // cout << i << endl;
              }
              trailPositions.push_back(current);
              // cout << "trailPositions " << trailPositions.size() << endl;
              for(int i = trailPositions.size()-1; i >= 0; i--){
                // cout << "waypoint " << trailPositions[i].get_x() << " " << trailPositions[i].get_y() << endl;
                beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(trailPositions[i], 3);
              }
              // std::vector<CartesianPoint> trailPositions;
              // trailPositions.push_back(CartesianPoint(positionHis->at(positionHis->size()-new_start_nearby_ind).getX(), positionHis->at(positionHis->size()-new_start_nearby_ind).getY()));
              // // Find the furthest point on path that can be seen from current position, push that point to trail and then move to that point
              // for(int i = new_start_nearby_ind; i >= 1; i--){
              //   for(int j = 1; j < i; j++){
              //     if(canAccessPoint(laserHis->at(positionHis->size()-i), CartesianPoint(positionHis->at(positionHis->size()-i).getX(), positionHis->at(positionHis->size()-i).getY()), CartesianPoint(positionHis->at(positionHis->size()-j).getX(), positionHis->at(positionHis->size()-j).getY()), 3)){
              //       trailPositions.push_back(CartesianPoint(positionHis->at(positionHis->size()-j).getX(), positionHis->at(positionHis->size()-j).getY()));
              //       i = j+1;
              //     }
              //   }
              //   // cout << i << endl;
              // }
              // cout << "trailPositions " << trailPositions.size() << endl;
              // for(int i = 0; i < trailPositions.size(); i++){
              //   cout << "waypoint " << trailPositions[i].get_x() << " " << trailPositions[i].get_y() << endl;
              //   beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(trailPositions[i], 3);
              // }
            }
          }
        }
        beliefs->getAgentState()->setFindAWayCount(0);
      }
      else{
        cout << "no available potential places" << endl;
        vector<CartesianPoint> end_waypoints = localExploration->randomExploration(current, beliefs->getAgentState()->getCurrentLaserEndpoints(), task);
        cout << "end_waypoints size " << end_waypoints.size() << endl;
        for(int i = end_waypoints.size()-1; i >= 0; i--){
          // cout << "waypoint " << end_waypoints[i].get_x() << " " << end_waypoints[i].get_y() << endl;
          beliefs->getAgentState()->getCurrentTask()->createNewWaypoint(end_waypoints[i], 3);
        }
      }
    }
  }
  return decisionMade;
}