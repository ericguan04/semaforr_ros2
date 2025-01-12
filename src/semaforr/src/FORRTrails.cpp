#include<vector>
#include<FORRGeometry.h>
#include<FORRTrails.h>
#include<fstream>
#include<sstream>


using namespace std;

#define EPSILON 20


//if it does find a trail marker that sees, return the index.  otherwise, return -1
int FORRTrails::doesTrailHaveVisiblePointToTarget(CartesianPoint target_point, int trail_index, AgentState *agentState){
  for(int j = 0; j < trails[trail_index].size(); j++){
    // if(agentState->canAccessPoint(trails[trail_index][j].wallVectorEndpoints, trails[trail_index][j].coordinates, target_point, 5)){
    if(canAccessPoint(trails[trail_index][j].wallVectorEndpoints, trails[trail_index][j].coordinates, target_point, 5)){
      //target point is visible along trail at trail marker trails[trail_index][j]
      //cout << "Trail at coordinate "<<trails[trail_index][j].coordinates.get_x()<<","<< trails[trail_index][j].coordinates.get_y()<<" can see target"<<endl;
      //cout<< "Target located at: ("<<target_point.get_x()<<","<<target_point.get_y()<<endl;
      //trail_marker_sees_point = j;
      return j;
      }
  }
  //else nothing visible, return false
  return -1;
}



void FORRTrails::updateTrails(AgentState *agentState){
    vector<TrailMarker> trail;
    std::pair < std::vector<CartesianPoint>, std::vector<vector<CartesianPoint> > > cleantrail = agentState->getCleanedTrailMarkers();
    //collect the wall distance vector endpoints after the initial x,y trail coordinates

    for(int i = 0; i < cleantrail.first.size(); i++){
	TrailMarker t = TrailMarker(cleantrail.first[i],cleantrail.second[i]);
	trail.push_back(t);
    }
  
  //Finally, push the full trail into the trails vector in FORRTrails.h
  trails.push_back(trail);
}


void FORRTrails::findNearbyTrail(AgentState *agentState){

  // If trail in place return
  if(chosen_trail != -1){
	return;
  }
  //first, check to see if there is a trail that is near your current position.
  //could also do this the other way around and check to see first if there are any points along the 
  //trail that are "seeable" (within some epsilon to a distance vector) to the target.
  //either way, these both need to be satisfied before a trail is "found"
  int trail_marker_seen_target = 0;
  CartesianPoint target(agentState->getCurrentTask()->getX(),agentState->getCurrentTask()->getY());
  //loop through all your trails and see if there is any trail point along those that you can see from where the robot is currently
  for(unsigned int i = 0; i < trails.size(); i++){
    trail_marker_seen_target = doesTrailHaveVisiblePointToTarget(target,i, agentState);
    
    //if not -1, then has a trail marker that can see the target
    if(trail_marker_seen_target > 0){

      //size-1 because we're checking pairs of trailmarkers now that generate a line
      for(unsigned int j = 0; j < trails[i].size()-1; j++){
	//for checking to see our current vector, we use the version of ispointvisibletoanotherpoint() 
	//that takes the distance vectors as distance doubles, as that's what's getting passed
	//from robotcontroller
	//check to see if any of our robot's current (i.e. the location the robot is at right now) wall vectors
	//are within epsilon to the point on the trail
	//say epsilon distance units for now as the epsilon
	
	//looks at the intersection between segments
	if(agentState->canSeeSegment(trails[i][j].coordinates, trails[i][j+1].coordinates)){
	  /*cout << "Trail marker at coordinates ("<<trails[i][j].coordinates.get_x()<<","<<
	    trails[i][j].coordinates.get_y()<<") visible to robot"<<endl;
	  cout << "----------TRAIL FOUND!---------- i ="<<i<<endl;*/
	  chosen_trail = i;
	  
	  //if the trail marker that saw the target is greater along the sequence than what you're seeing,
	  //then you're following the trail in the same direction it was created
	  //otherwise, you need to follow the trail in reverse
	  if(trail_marker_seen_target > j){
	    setDirection(POSITIVE);
	  }
	  else{
	    setDirection(NEGATIVE);
	  }
	}
      }
    }
  }
  
  //otherwise, chosen_trail remains -1
}


CartesianPoint FORRTrails::getFurthestVisiblePointOnChosenTrail(AgentState *agentState){
  //if no trail found, return a dummy point
  if(chosen_trail == -1) return CartesianPoint(-1,-1);

  if(dir == POSITIVE) {
  //go backwards if direction is positive to grab a further point first if possible
    for(int j = trails[chosen_trail].size()-1; j >= 1; j--){
      if(agentState->canSeePoint(trails[chosen_trail][j].coordinates, 20)) {
        can_see_trail = true;
        return trails[chosen_trail][j].coordinates;
      }
    }
  }
  else if(dir == NEGATIVE){
  //go forwards if direction is negative to grab a further point first if possible
    for(int j = 0; j < trails[chosen_trail].size(); j++){
      if(agentState->canSeePoint(trails[chosen_trail][j].coordinates, 20)) {
        can_see_trail = true;
        return trails[chosen_trail][j].coordinates;
      }
    }
  }
/*
  //go backwards to grab a further point first if possible
  for(int j = trails[chosen_trail].size(); j >= 1; j--){
    if(agentState->canSeeSegment(trails[chosen_trail][j].coordinates,
    			      trails[chosen_trail][j-1].coordinates)){
      can_see_trail = true;
      return trails[chosen_trail][j].coordinates;
    }
  }
  */
  can_see_trail = false;
  return CartesianPoint(-1,-1);
}


void FORRTrails::printTrails(){
  for(int i = 0; i < trails.size(); i++){
    for(int j = 0; j < trails[i].size(); j++){
      cout << "("<<trails[i][j].coordinates.get_x()<<","<<trails[i][j].coordinates.get_y()<<") ";
    }
    cout <<endl;
  }
}

vector< vector< CartesianPoint > > FORRTrails::getTrailsPoints(){
  vector< vector< CartesianPoint > > trailsPoints;
  for(int i = 0; i < trails.size(); i++){
    vector< CartesianPoint > trailPoints;
    for(int j = 0; j < trails[i].size(); j++){
      trailPoints.push_back(trails[i][j].coordinates);
    }
    trailsPoints.push_back(trailPoints);
  }
  return trailsPoints;
}
