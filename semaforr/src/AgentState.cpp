#include <AgentState.h>
#include <iostream>

/*
Position AgentState::getExpectedPositionAfterActions(Position initialPosition, vector<FORRAction> actionList){
  Position currentPosition = initialPosition;
  Position nextPosition;
  for(int i = 0 ; i < actionList.size(); i++){
    FORRAction action = actionList[i];
    nextPosition = getExpectedPositionAfterAction(currentPosition, action);
    // cout << "AgentState :: getExpectedPositionAfterActions >> nextposition " << nextPosition.getX() << " " << nextPosition.getY() << endl;
    currentPosition = nextPosition;
  }
  return nextPosition;
}
*/

Position AgentState::getExpectedPositionAfterAction(FORRAction action){
  Position result; 
  int intensity = action.parameter;
  FORRActionType type = action.type;
  Position initialPosition = getCurrentPosition();
  
  switch(type){
  case FORWARD:
    result = afterLinearMove(initialPosition, move[intensity]);
    break;
  case RIGHT_TURN:
    result = afterAngularMove(initialPosition, -rotate[intensity]);
    break;
  case LEFT_TURN:
    result = afterAngularMove(initialPosition, rotate[intensity]);
    break;
  case PAUSE:
    result = initialPosition;
    break;
  }
  return result;
}


Position AgentState::afterLinearMove(Position initialPosition, double distance){
  if(distance > getDistanceToForwardObstacle()) distance = getDistanceToForwardObstacle();

  double new_x = initialPosition.getX() + (distance * cos(initialPosition.getTheta()));
  double new_y = initialPosition.getY() + (distance * sin(initialPosition.getTheta()));
  
  return Position(new_x, new_y, initialPosition.getTheta());
}


Position AgentState::afterAngularMove(Position initialPosition, double angle){
  // cout << "inside afterAngularMove theta " << initialPosition.getTheta() << " angle " << angle << endl;
  double new_angle = (angle + initialPosition.getTheta());
  
  double distance = getDistanceToObstacle(angle);
  // cout << "new_angle " << new_angle << " distance " << distance << endl;
  // max is the maximum look ahead in meters
  double max;
  if(numMoves-2 > 0){
    max = move[numMoves-2];
  }
  else{
    max = move[1];
  }
  if(distance > max) distance = max;
  // cout << "max " << max << " distance " << distance << endl;
  double new_x = initialPosition.getX() + (distance * cos(new_angle));
  double new_y = initialPosition.getY() + (distance * sin(new_angle));
  
  return Position(new_x, new_y, new_angle);
  // return Position(initialPosition.getX(), initialPosition.getY(), new_angle);
}


Position AgentState::getExpectedPositionAfterAction(FORRAction action, vector<CartesianPoint> initialLaser, Position currPosition){
  Position result; 
  int intensity = action.parameter;
  FORRActionType type = action.type;
  Position initialPosition = currPosition;
  
  switch(type){
  case FORWARD:
    result = afterLinearMove(initialPosition, initialLaser, move[intensity]);
    break;
  case RIGHT_TURN:
    result = afterAngularMove(initialPosition, initialLaser, -rotate[intensity]);
    break;
  case LEFT_TURN:
    result = afterAngularMove(initialPosition, initialLaser, rotate[intensity]);
    break;
  case PAUSE:
    result = initialPosition;
    break;
  }
  return result;
}
  

Position AgentState::afterLinearMove(Position initialPosition, vector<CartesianPoint> initialLaser, double distance){
  if(distance > getDistanceToForwardObstacle(initialPosition, initialLaser)) distance = getDistanceToForwardObstacle(initialPosition, initialLaser);

  double new_x = initialPosition.getX() + (distance * cos(initialPosition.getTheta()));
  double new_y = initialPosition.getY() + (distance * sin(initialPosition.getTheta()));
  
  return Position(new_x, new_y, initialPosition.getTheta());
}


Position AgentState::afterAngularMove(Position initialPosition, vector<CartesianPoint> initialLaser, double angle){

  double new_angle = (angle + initialPosition.getTheta());
  
  double distance = getDistanceToObstacle(initialPosition, initialLaser, angle);
  // max is the maximum look ahead in meters 
  double max;
  if(numMoves-2 > 0){
    max = move[numMoves-2];
  }
  else{
    max = move[1];
  }
  if(distance > max) distance = max;

  double new_x = initialPosition.getX() + (distance * cos(new_angle));
  double new_y = initialPosition.getY() + (distance * sin(new_angle));
  
  return Position(new_x, new_y, new_angle);
  // return Position(initialPosition.getX(), initialPosition.getY(), new_angle);
}


//return the distance to obstacle when the robot is in POS using the current laser scan
double AgentState::getDistanceToNearestObstacle(Position pos){ 
   double min_dis = 1000000;
   for(int i = 0; i < laserEndpoints.size(); i++){
	double x = laserEndpoints[i].get_x();
 	double y = laserEndpoints[i].get_y();
	double dist = pos.getDistance(Position(x,y,0));
	if(dist < min_dis)
		min_dis = dist; 
   }
   return min_dis;
}

//return the nearest obstacle when the robot is in POS using the current laser scan
Position AgentState::getNearestObstacle(Position pos){ 
  double min_dis = 1000000;
  Position min_pos = Position(0,0,0);
  for(int i = 0; i < laserEndpoints.size(); i++){
    double x = laserEndpoints[i].get_x();
    double y = laserEndpoints[i].get_y();
    double dist = pos.getDistance(Position(x,y,0));
    if(dist < min_dis){
      min_dis = dist;
      min_pos = Position(x,y,0);
    }
  }
  return min_pos;
}

//Sees if the laser scan intersects with a segment created by 2
//trailpoints
bool AgentState::canSeeSegment(CartesianPoint point1, CartesianPoint point2){
  CartesianPoint curr(currentPosition.getX(),currentPosition.getY());
  return canSeeSegment(laserEndpoints, curr, point1, point2);
}

//Sees if the laser scan intersects with a segment created by 2
//trailpoints
bool AgentState::canSeeSegment(vector<CartesianPoint> givenLaserEndpoints, CartesianPoint laserPos, CartesianPoint point1, CartesianPoint point2){
  CartesianPoint intersection_point(0,0);
  bool canSeeSegment = false;
  for(int i = 0; i < givenLaserEndpoints.size(); i++){
    //this can be cleaned up or put into one function, but need to convert the distance to a linesegment to calculate
    //the distance from a point to a line
    CartesianPoint endpoint = givenLaserEndpoints[i];
    
    LineSegment distance_vector_line = LineSegment(laserPos, endpoint);
    LineSegment trail_segment = LineSegment(point1, point2);
    if(do_intersect(distance_vector_line, trail_segment, intersection_point)){
      canSeeSegment = true;
	break;
    }
  }
  //else, not visible
  return canSeeSegment;
}


//returns true if there is a point that is "visible" by the wall distance vectors to some epsilon.  
//A point is visible if the distance to a wall distance vector line is < epsilon.
bool AgentState::canSeePoint(vector<CartesianPoint> givenLaserEndpoints, CartesianPoint laserPos, CartesianPoint point, double distanceLimit){
  // //// RCLCPP_DEBUG_STREAM(this->get_logger(), "AgentState:canSeePoint() , robot pos " << laserPos.get_x() << "," << laserPos.get_y() << " target " << point.get_x() << "," << point.get_y()); 
  // double epsilon = canSeePointEpsilon;
  // //// RCLCPP_DEBUG_STREAM(this->get_logger(), "Number of laser endpoints " << givenLaserEndpoints.size()); 
  // bool canSeePoint = false;
  // double ab = laserPos.get_distance(point);
  // for(int i = 0; i < givenLaserEndpoints.size(); i++){
  //   //// RCLCPP_DEBUG_STREAM(this->get_logger(), "Laser endpoint : " << givenLaserEndpoints[i].get_x() << "," << givenLaserEndpoints[i].get_y());
  //   double ac = laserPos.get_distance(givenLaserEndpoints[i]);
  //   double bc = givenLaserEndpoints[i].get_distance(point);
  //   if(((ab + bc) - ac) < epsilon){
  //     //cout << "Distance vector endpoint visible: ("<<laserEndpoints[i].get_x()<<","<< laserEndpoints[i].get_y()<<")"<<endl; 
  //     //cout << "Distance: "<<distance_to_point<<endl;
  //     canSeePoint = true;
  //     break;
  //   }
  // }
  // //else, not visible
  // return canSeePoint;
  return canAccessPoint(givenLaserEndpoints, laserPos, point, distanceLimit);
}

// //returns true if there is a point that is "visible" by the wall distance vectors.  
// //A point is visible if the distance to the nearest wall distance vector lines is > distance to the point.
// bool AgentState::canAccessPoint(vector<CartesianPoint> givenLaserEndpoints, CartesianPoint laserPos, CartesianPoint point, double distanceLimit){
//   //// RCLCPP_DEBUG_STREAM(this->get_logger(), "AgentState:canAccessPoint() , robot pos " << laserPos.get_x() << "," << laserPos.get_y() << " target " << point.get_x() << "," << point.get_y()); 
//   //// RCLCPP_DEBUG_STREAM(this->get_logger(), "Number of laser endpoints " << givenLaserEndpoints.size()); 
//   bool canAccessPoint = false;
//   double distLaserPosToPoint = laserPos.get_distance(point);
//   if(distLaserPosToPoint > distanceLimit){
//     return false;
//   }
//   double point_direction = atan2((point.get_y() - laserPos.get_y()), (point.get_x() - laserPos.get_x()));
//   int index = 0;
//   double min_angle = 100000;

//   for(int i = 0; i < givenLaserEndpoints.size(); i++){
//     //// RCLCPP_DEBUG_STREAM(this->get_logger(), "Laser endpoint : " << givenLaserEndpoints[i].get_x() << "," << givenLaserEndpoints[i].get_y());
//     double laser_direction = atan2((givenLaserEndpoints[i].get_y() - laserPos.get_y()), (givenLaserEndpoints[i].get_x() - laserPos.get_x()));
//     if(abs(laser_direction - point_direction) < min_angle){
//       //// RCLCPP_DEBUG_STREAM(this->get_logger(), "Laser Direction : " << laser_direction << ", Point Direction : " << point_direction);
//       min_angle = abs(laser_direction - point_direction);
//       index = i;
//     }
//   }
//   while (index-2 < 0){
//     index = index + 1;
//   }
//   while (index+2 > givenLaserEndpoints.size()-1){
//     index = index - 1;
//   }
//   //// RCLCPP_DEBUG_STREAM(this->get_logger(), "Min angle : " << min_angle << ", " << index);
//   int numFree = 0;
//   for(int i = -2; i < 3; i++) {
//     double distLaserEndPointToLaserPos = givenLaserEndpoints[index+i].get_distance(laserPos);
//     //// RCLCPP_DEBUG_STREAM(this->get_logger(), "Distance Laser EndPoint to Laser Pos : " << distLaserEndPointToLaserPos << ", Distance Laser Pos to Point : " << distLaserPosToPoint);
//     if (distLaserEndPointToLaserPos > distLaserPosToPoint) {
//       numFree++;
//     }
//   }
//   //// RCLCPP_DEBUG_STREAM(this->get_logger(), "Number farther than point : " << numFree);
//   if (numFree > 3) {
//     canAccessPoint = true;
//   }
//   else{
//     return false;
//   }
//   //else, not visible
//   //return canAccessPoint;
//   double epsilon = canSeePointEpsilon;
//   bool canSeePoint = false;
//   double ab = laserPos.get_distance(point);
//   for(int i = -2; i < 3; i++) {
//     //// RCLCPP_DEBUG_STREAM(this->get_logger(), "Laser endpoint : " << givenLaserEndpoints[i].get_x() << "," << givenLaserEndpoints[i].get_y());
//     double ac = laserPos.get_distance(givenLaserEndpoints[i]);
//     double bc = givenLaserEndpoints[i].get_distance(point);
//     if(((ab + bc) - ac) < epsilon){
//       //cout << "Distance vector endpoint visible: ("<<laserEndpoints[i].get_x()<<","<< laserEndpoints[i].get_y()<<")"<<endl; 
//       //cout << "Distance: "<<distance_to_point<<endl;
//       canSeePoint = true;
//       break;
//     }
//   }
//   if(canSeePoint == false){
//     for(int i = 0; i < givenLaserEndpoints.size(); i++){
//       //// RCLCPP_DEBUG_STREAM(this->get_logger(), "Laser endpoint : " << givenLaserEndpoints[i].get_x() << "," << givenLaserEndpoints[i].get_y());
//       double ac = laserPos.get_distance(givenLaserEndpoints[i]);
//       double bc = givenLaserEndpoints[i].get_distance(point);
//       if(((ab + bc) - ac) < epsilon){
//         //cout << "Distance vector endpoint visible: ("<<laserEndpoints[i].get_x()<<","<< laserEndpoints[i].get_y()<<")"<<endl; 
//         //cout << "Distance: "<<distance_to_point<<endl;
//         canSeePoint = true;
//         break;
//       }
//     }
//   }
//   if(canSeePoint and canAccessPoint){
//     return true;
//   }
//   else{
//     return false;
//   }
// }


std::pair < std::vector<CartesianPoint>, std::vector< vector<CartesianPoint> > > AgentState::getCleanedTrailMarkers(){
	std::pair < std::vector<CartesianPoint>, std::vector<vector<CartesianPoint> > > cleanedMarker;
	// Change position history into CartesianPoint format
	vector <CartesianPoint> pos_history;
	for(std::vector<Position>::iterator it = currentTask->getPositionHistory()->begin() ; it != currentTask->getPositionHistory()->end(); ++it){
		CartesianPoint pos(it->getX(),it->getY());
		pos_history.push_back(pos);
	}
	// Initialize trail vectors
	std::vector<CartesianPoint> trailPositions;
	std::vector< vector<CartesianPoint> > trailLaserEndpoints;
	//Get laser history
	std::vector< vector<CartesianPoint> > laser_endpoints = *(currentTask->getLaserHistory());
	// Push first point in path to trail
	trailPositions.push_back(pos_history[0]);
	trailLaserEndpoints.push_back(laser_endpoints[0]);
	// Find the furthest point on path that can be seen from current position, push that point to trail and then move to that point
	for(int i = 0; i < pos_history.size(); i++){
		//cout << "First point: " << pos_history[i].get_x() << " " << pos_history[i].get_y() << endl;
		for(int j = pos_history.size()-1; j > i; j--){
			//cout << pos_history[j].get_x() << " " << pos_history[j].get_y() << endl;
			//if(canSeePoint(laser_endpoints[i], pos_history[i], pos_history[j])) {
      //if(canSeePoint(laser_endpoints[i], pos_history[i], pos_history[j]) and canSeePoint(laser_endpoints[j], pos_history[j], pos_history[i])) {
      if(canAccessPoint(laser_endpoints[i], pos_history[i], pos_history[j], 5)) {
				//cout << "CanAccessPoint is true" << endl;
				//cout << "Next point: " << pos_history[j].get_x() << " " << pos_history[j].get_y() << endl;
				trailPositions.push_back(pos_history[j]);
				trailLaserEndpoints.push_back(laser_endpoints[j]);
				i = j-1;
			}
		}
	}
  if (pos_history[pos_history.size()-1].get_x() != trailPositions[trailPositions.size()-1].get_x() or pos_history[pos_history.size()-1].get_y() != trailPositions[trailPositions.size()-1].get_y()) {
    trailPositions.push_back(pos_history.back());
    trailLaserEndpoints.push_back(laser_endpoints.back());
  }
	//cout << pos_history[pos_history.size()-1].get_x() << " " << pos_history[pos_history.size()-1].get_y() << endl;
	//cout << trailPositions[trailPositions.size()-1].get_x() << " " << trailPositions[trailPositions.size()-1].get_y() << endl;
	
	cleanedMarker.first = trailPositions;
	cleanedMarker.second = trailLaserEndpoints;
	return cleanedMarker;
}


//returns true if there is a point that is "visible" by the wall distance vectors to some epsilon.  
//A point is visible if the distance to a wall distance vector line is < epsilon.
bool AgentState::canSeePoint(CartesianPoint point, double distanceLimit){
  CartesianPoint curr(currentPosition.getX(),currentPosition.getY());
  //return canSeePoint(laserEndpoints, curr, point);
  return canAccessPoint(laserEndpoints, curr, point, distanceLimit);
}

bool AgentState::canSeeRegion(CartesianPoint center, double radius, double distanceLimit){
  CartesianPoint laserPos(currentPosition.getX(),currentPosition.getY());
  bool canAccessPoint = false, canAccessRegion = false;
  double distLaserPosToPoint = laserPos.get_distance(center);
  if(distLaserPosToPoint - radius > distanceLimit){
    return false;
  }
  for(int i = 0; i < laserEndpoints.size(); i++){
    //// RCLCPP_DEBUG_STREAM(this->get_logger(), "Laser endpoint : " << laserEndpoints[i].get_x() << "," << laserEndpoints[i].get_y());
    if(do_intersect(Circle(center, radius), LineSegment(laserPos, laserEndpoints[i]))){
      canAccessRegion = true;
      break;
    }
  }
  if(canSeePoint(center, distanceLimit)){
    canAccessPoint = true;
  }
  if(canAccessRegion == true and canAccessPoint == true){
    return true;
  }
  else{
    return false;
  }
}

void AgentState::transformToEndpoints(){
    // RCLCPP_DEBUG(this->get_logger(), "Convert laser scan to endpoints");
    double start_angle = currentLaserScan.angle_min;
    double increment = currentLaserScan.angle_increment;
    laserEndpoints.clear();
    // cout << "start_angle " << start_angle << " increment " << increment << endl;

    double r_x = currentPosition.getX();
    double r_y = currentPosition.getY();
    double r_ang = currentPosition.getTheta();
    CartesianPoint current_point(r_x,r_y);
  
    for(int i = 0 ; i < currentLaserScan.ranges.size(); i++){
       Vector v = Vector(current_point, start_angle + r_ang, currentLaserScan.ranges[i]);
       CartesianPoint endpoint = v.get_endpoint();
       start_angle = start_angle + increment;
       laserEndpoints.push_back(endpoint);
    }    
}

vector<CartesianPoint> AgentState::transformToEndpoints(Position p, sensor_msgs::msg::LaserScan scan){
    // // RCLCPP_DEBUG(this->get_logger(), "Convert laser scan to endpoints");
    double start_angle = scan.angle_min;
    double increment = scan.angle_increment;
    vector<CartesianPoint> les;

    double r_x = p.getX();
    double r_y = p.getY();
    double r_ang = p.getTheta();
    CartesianPoint current_point(r_x,r_y);
  
    for(int i = 0 ; i < scan.ranges.size(); i++){
       Vector v = Vector(current_point, start_angle + r_ang, scan.ranges[i]);
       CartesianPoint endpoint = v.get_endpoint();
       start_angle = start_angle + increment;
       les.push_back(endpoint);
    }
    return les;
}

double AgentState::getDistanceToObstacle(double rotation_angle){
	// // RCLCPP_DEBUG(this->get_logger(), "In getDistanceToObstacle");
	// one increment in the laser range scan is 1/3 degrees, i.e 0.005817 in radians  
	int index = (int)(rotation_angle/(laserScanRadianIncrement));
  // cout << "index " << index << endl;
	//// RCLCPP_DEBUG(this->get_logger(), "In getDistanceToObstacle after index");
	//shift the index in the positive
	index = index + 330;
	//// RCLCPP_DEBUG(this->get_logger(), "In getDistanceToObstacle after index shift");
	if(index < 0) index = 0;
	//// RCLCPP_DEBUG(this->get_logger(), "In getDistanceToObstacle after first if");
	if(index > 659) index = 659;
	//// RCLCPP_DEBUG(this->get_logger(), "In getDistanceToObstacle after second if");
	// cout << index << " " << currentLaserScan.ranges.size() << endl;
	if(currentLaserScan.ranges.size() == 0) { return maxLaserRange; }
	// cout << currentLaserScan.ranges[index] << endl;

  double min_distance = 25;
  int start_index = index - 2;
  if(start_index < 0)
    start_index = 0;
  int end_index = index + 2;
  if(end_index > 659)
    end_index = 659;

  for(int i = start_index; i <= end_index; i++){
    if(currentLaserScan.ranges[i] < min_distance){
      min_distance = currentLaserScan.ranges[i];
    }
  }
  return min_distance;
	// double r_x = currentPosition.getX();
	// double r_y = currentPosition.getY();
	// double r_ang = currentPosition.getTheta();
	// CartesianPoint current_point(r_x,r_y);
	// double r = robotFootPrint+robotFootPrintBuffer; // fetch robot's footprint plus 0.1 meter buffer
	
	// Vector v1 = Vector(current_point, r_ang+(M_PI/2), r);
	// Vector v2 = Vector(current_point, r_ang-(M_PI/2), r);
	
	// Vector parallel1 = Vector(v1.get_endpoint(), r_ang, maxLaserRange);
	// Vector parallel2 = Vector(v2.get_endpoint(), r_ang, maxLaserRange);
	
	// Vector laserVector = Vector(current_point, r_ang+rotation_angle, maxLaserRange);
	
	// CartesianPoint intersectionPoint = CartesianPoint();
	
	// if(do_intersect(parallel1, laserVector, intersectionPoint)){
	// 	if(intersectionPoint.get_distance(current_point) < currentLaserScan.ranges[index]){
	// 		//cout << "25" << endl;
	// 		return maxLaserRange;
	// 	}
	// 	else {
	// 		//cout << currentLaserScan.ranges[index] << endl;
	// 		return currentLaserScan.ranges[index];
	// 	}
	// }
	// else if(do_intersect(parallel2, laserVector, intersectionPoint)){
	// 	if(intersectionPoint.get_distance(current_point) < currentLaserScan.ranges[index]){
	// 		//cout << "25" << endl;
	// 		return maxLaserRange;
	// 	}
	// 	else {
	// 		//cout << currentLaserScan.ranges[index] << endl;
	// 		return currentLaserScan.ranges[index];
	// 	}
	// }
	// else {
	// 	//cout << currentLaserScan.ranges[index] << endl;
	// 	return currentLaserScan.ranges[index];
	// }
}


double AgentState::getDistanceToObstacle(Position initialPosition, vector<CartesianPoint> initialLaser, double rotation_angle){
  //// RCLCPP_DEBUG(this->get_logger(), "In getDistanceToObstacle");
  // one increment in the laser range scan is 1/3 degrees, i.e 0.005817 in radians  
  int index = (int)(rotation_angle/(laserScanRadianIncrement));
  //// RCLCPP_DEBUG(this->get_logger(), "In getDistanceToObstacle after index");
  //shift the index in the positive
  index = index + 330;
  //// RCLCPP_DEBUG(this->get_logger(), "In getDistanceToObstacle after index shift");
  if(index < 0) index = 0;
  //// RCLCPP_DEBUG(this->get_logger(), "In getDistanceToObstacle after first if");
  if(index > 659) index = 659;
  //// RCLCPP_DEBUG(this->get_logger(), "In getDistanceToObstacle after second if");
  //cout << index << " " << currentLaserScan.ranges.size() << endl;
  if(initialLaser.size() == 0) { return maxLaserRange; }
  //cout << currentLaserScan.ranges[index] << endl;

  double min_distance = 25;
  int start_index = index - 2;
  if(start_index < 0)
    start_index = 0;
  int end_index = index + 2;
  if(end_index > 659)
    end_index = 659;

  for(int i = start_index; i <= end_index; i++){
    double distance_to_laser = initialPosition.getDistance(initialLaser[i].get_x(), initialLaser[i].get_y());
    if(distance_to_laser < min_distance){
      min_distance = distance_to_laser;
    }
  }
  return min_distance;
  
  // double r_x = initialPosition.getX();
  // double r_y = initialPosition.getY();
  // double r_ang = initialPosition.getTheta();
  // CartesianPoint current_point(r_x,r_y);
  // double r = robotFootPrint+robotFootPrintBuffer; // fetch robot's footprint plus 0.1 meter buffer
  
  // Vector v1 = Vector(current_point, r_ang+(M_PI/2), r);
  // Vector v2 = Vector(current_point, r_ang-(M_PI/2), r);
  
  // Vector parallel1 = Vector(v1.get_endpoint(), r_ang, maxLaserRange);
  // Vector parallel2 = Vector(v2.get_endpoint(), r_ang, maxLaserRange);
  
  // Vector laserVector = Vector(current_point, r_ang+rotation_angle, maxLaserRange);
  
  // CartesianPoint intersectionPoint = CartesianPoint();
  // double distance_to_laser = initialPosition.getDistance(initialLaser[index].get_x(), initialLaser[index].get_y());
  
  // if(do_intersect(parallel1, laserVector, intersectionPoint)){
  //   if(intersectionPoint.get_distance(current_point) < distance_to_laser){
  //     //cout << "25" << endl;
  //     return maxLaserRange;
  //   }
  //   else {
  //     //cout << currentLaserScan.ranges[index] << endl;
  //     return distance_to_laser;
  //   }
  // }
  // else if(do_intersect(parallel2, laserVector, intersectionPoint)){
  //   if(intersectionPoint.get_distance(current_point) < distance_to_laser){
  //     //cout << "25" << endl;
  //     return maxLaserRange;
  //   }
  //   else {
  //     //cout << currentLaserScan.ranges[index] << endl;
  //     return distance_to_laser;
  //   }
  // }
  // else {
  //   //cout << currentLaserScan.ranges[index] << endl;
  //   return distance_to_laser;
  // }
}


FORRAction AgentState::maxForwardAction(){
 	//// RCLCPP_DEBUG(this->get_logger(), "In maxforwardaction");
	double error_margin = maxForwardActionBuffer; // margin from obstacles
	double view = maxForwardActionSweepAngle; // +view radians to -view radians view
	//double view = 0.7854;
	double min_distance = getDistanceToObstacle(view);
	//cout << min_distance << endl;
	for(double angle = (-1)*view; angle < view; angle = angle + 0.005){
		//cout << angle << endl;
		double distance = getDistanceToObstacle(angle);
		if(distance < min_distance){
			min_distance = distance;
		} 
	}
	
	min_distance = min_distance - error_margin;

	// RCLCPP_DEBUG_STREAM(this->get_logger(), "Forward obstacle distance " << min_distance);
  for (int i = numMoves-1; i > 0; i--) {
    if(min_distance > move[i]) {
      return FORRAction(FORWARD,i);
    }
  }
  return FORRAction(FORWARD,0);
	/*if(min_distance > move[5])
		return FORRAction(FORWARD,5);
	else if(min_distance > move[4] && min_distance <= move[5])
		return FORRAction(FORWARD,4);
	else if(min_distance > move[3] && min_distance <= move[4])
		return FORRAction(FORWARD,3);
	else if(min_distance > move[2] && min_distance <= move[3])
		return FORRAction(FORWARD,2);
	else if(min_distance > move[1] && min_distance <= move[2])
		return FORRAction(FORWARD,1);
	else
		return FORRAction(FORWARD,0);*/

}

FORRAction AgentState::maxForwardAction(Position initialPosition, vector<CartesianPoint> initialLaser){
  //// RCLCPP_DEBUG(this->get_logger(), "In maxforwardaction");
  double error_margin = maxForwardActionBuffer; // margin from obstacles
  double view = maxForwardActionSweepAngle; // +view radians to -view radians view
  //double view = 0.7854;
  double min_distance = getDistanceToObstacle(initialPosition, initialLaser, view);
  //cout << min_distance << endl;
  for(double angle = (-1)*view; angle < view; angle = angle + 0.005){
    //cout << angle << endl;
    double distance = getDistanceToObstacle(initialPosition, initialLaser, angle);
    if(distance < min_distance){
      min_distance = distance;
    } 
  }
  
  min_distance = min_distance - error_margin;

  // RCLCPP_DEBUG_STREAM(this->get_logger(), "Forward obstacle distance " << min_distance);
  for (int i = numMoves-1; i > 0; i--) {
    if(min_distance > move[i]) {
      return FORRAction(FORWARD,i);
    }
  }
  return FORRAction(FORWARD,0);
  /*if(min_distance > move[5])
    return FORRAction(FORWARD,5);
  else if(min_distance > move[4] && min_distance <= move[5])
    return FORRAction(FORWARD,4);
  else if(min_distance > move[3] && min_distance <= move[4])
    return FORRAction(FORWARD,3);
  else if(min_distance > move[2] && min_distance <= move[3])
    return FORRAction(FORWARD,2);
  else if(min_distance > move[1] && min_distance <= move[2])
    return FORRAction(FORWARD,1);
  else
    return FORRAction(FORWARD,0);*/

}

FORRAction AgentState::get_max_allowed_forward_move(){
  FORRAction max_forward(FORWARD, numMoves-1);
  cout << " Number of vetoed actions : " << vetoedActions->size() << endl;
  for(int intensity = 1; intensity <= numMoves; intensity++){
    if(vetoedActions->find(FORRAction(FORWARD,intensity)) != vetoedActions->end()){
      max_forward.type = FORWARD;
      max_forward.parameter = intensity - 1;
      break;
    }
  }
  return max_forward;
}


// returns an Action that takes the robot closest to the target
FORRAction AgentState::moveTowards(CartesianPoint target){
    // RCLCPP_DEBUG(this->get_logger(), "AgentState :: In moveTowards");
    double distance_from_target = currentPosition.getDistance(target.get_x(), target.get_y());
    // RCLCPP_DEBUG_STREAM(this->get_logger(), "Distance from target : " << distance_from_target);
    // compute the angular difference between the direction to the target and the current robot direction
    double robot_direction = currentPosition.getTheta();
    double goal_direction = atan2((target.get_y() - currentPosition.getY()), (target.get_x() - currentPosition.getX()));
    
    double required_rotation = goal_direction - robot_direction;

    //// RCLCPP_DEBUG_STREAM(this->get_logger(), "Robot direction : " << robot_direction << ", Goal Direction : " << goal_direction << ", Required rotation : " << required_rotation);
    if(required_rotation > M_PI)
      required_rotation = required_rotation - (2*M_PI);
    if(required_rotation < -M_PI)
      required_rotation = required_rotation + (2*M_PI);
    //cout << "Robot direction : " << robot_direction << ", Goal Direction : " << goal_direction << ", Required rotation : " << required_rotation << endl;
    // RCLCPP_DEBUG_STREAM(this->get_logger(), "Robot direction : " << robot_direction << ", Goal Direction : " << goal_direction << ", Required rotation : " << required_rotation);
    // if the angular difference is greater than smallest turn possible 
    // pick the right turn to allign itself to the target
    
    FORRAction decision;
    int rotIntensity=0;
    while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
      rotIntensity++;
    }
    // RCLCPP_DEBUG_STREAM(this->get_logger(), "Rotation Intensity : " << rotIntensity);
    int max_allowed = maxForwardAction().parameter;
    if (rotIntensity > 1) {
      if (required_rotation < 0){
        decision = FORRAction(RIGHT_TURN, rotIntensity-1);
      }
      else {
        decision = FORRAction(LEFT_TURN, rotIntensity-1);
      }
    }
    else if(max_allowed == 0) {
      if (required_rotation < 0){
        decision = FORRAction(RIGHT_TURN, 1);
      }
      else {
        decision = FORRAction(LEFT_TURN, 1);
      }
    }
    else {
      int intensity=0;
      while(distance_from_target > move[intensity] and intensity < numMoves) {
        intensity++;
      }
      if(intensity > 1)
        intensity--;
      // RCLCPP_DEBUG_STREAM(this->get_logger(), "Move Intensity : " << intensity);
      // int max_allowed = maxForwardAction().parameter;
      if(intensity > max_allowed){
          intensity = max_allowed;
      }
      decision = FORRAction(FORWARD, intensity);
    }

    /*if(fabs(required_rotation) > rotate[1]){
      if( required_rotation > rotate[1] && required_rotation <= rotate[2])
        decision = FORRAction(LEFT_TURN, 1);
      else if( required_rotation > rotate[2] && required_rotation <= rotate[3])
        decision = FORRAction(LEFT_TURN, 2);
      else if(required_rotation > rotate[3] && required_rotation <= rotate[4])
        decision = FORRAction(LEFT_TURN, 3);
      else if(required_rotation > rotate[4])
        decision = FORRAction(LEFT_TURN, 4);
      else if( required_rotation < -rotate[1] && required_rotation >= -rotate[2])
        decision = FORRAction(RIGHT_TURN, 1);
      else if( required_rotation < -rotate[2] && required_rotation >= -rotate[3])
        decision = FORRAction(RIGHT_TURN, 2);
      else if(required_rotation < -rotate[3] && required_rotation >= -rotate[4])
        decision = FORRAction(RIGHT_TURN, 3);
      else if(required_rotation < -rotate[4])
        decision = FORRAction(RIGHT_TURN, 4);
    }
    else{
      int intensity;
      if(distance_from_target <= move[1])
        intensity = 0;
      else if(distance_from_target <= move[2])
        intensity = 1;
      else if(distance_from_target <= move[3])
        intensity = 2;
      else if(distance_from_target <= move[4])
        intensity = 3;
      else if(distance_from_target <= move[5])
        intensity = 4;
      else
        intensity = 5;
      int max_allowed = maxForwardAction().parameter;
      if(intensity > max_allowed){
        intensity = max_allowed;
      }
      decision = FORRAction(FORWARD, intensity);
    }*/
    
    // RCLCPP_INFO_STREAM(this->get_logger(), "Action choosen : " << decision.type << "," << decision.parameter);
    return decision;
}

  void AgentState::setAgentStateParameters(double val1, double val2, double val3, double val4, double val5, double val6, double val7){
    canSeePointEpsilon = val1;
    laserScanRadianIncrement = val2;
    robotFootPrint = val3;
    robotFootPrintBuffer = val4;
    maxLaserRange = val5;
    maxForwardActionBuffer = val6;
    maxForwardActionSweepAngle = val7;
  }

bool AgentState::getRobotConfined(int decisionLimit, double distanceLimit){
  // RCLCPP_DEBUG(this->get_logger(), "AgentState :: In getRobotConfined");
  cout << "decisionLimit " << decisionLimit << " distanceLimit " << distanceLimit << endl;
  Position current_position = currentPosition;
  vector<Position> *pos_hist = currentTask->getPositionHistory();
  if(pos_hist->size() < decisionLimit){
    robotConfined = false;
    return robotConfined;
  }

  int startPosition = 0;
  if(pos_hist->size() > decisionLimit+1){
    startPosition = pos_hist->size() - decisionLimit - 1;
  }

  vector< vector <CartesianPoint> > *laser_hist = currentTask->getLaserHistory();
  cout << "laser_hist " << laser_hist->size() << endl;
  int dimension = currentTask->getDimension();
  vector< vector<int> > total_coverages;
  for(int k = startPosition; k < laser_hist->size(); k++){
    vector< vector<int> > grid;
    for(int i = 0; i < dimension; i++){
      vector<int> col;
      for(int j = 0; j < dimension; j++){
        col.push_back(0);
      }
      grid.push_back(col);
    }
    for(int l = 0; l < (*laser_hist)[k].size(); l++){
      double x1 = (*pos_hist)[k].getX();
      double y1 = (*pos_hist)[k].getY();
      double x2 = (*laser_hist)[k][l].get_x();
      double y2 = (*laser_hist)[k][l].get_y();
      double step_size = 0.1;
      double tx,ty;
      for(double step = 0; step <= 1; step += step_size){
        tx = (int)((x2 * step) + (x1 * (1-step)));
        ty = (int)((y2 * step) + (y1 * (1-step)));
        if(tx >= 0 and tx < dimension and ty >= 0 and ty < dimension){
          grid[tx][ty] = 1;
        }
      }
    }
    vector<int> coverage;
    for(int i = 0; i < dimension; i++){
      for(int j = 0; j < dimension; j++){
        coverage.push_back(grid[i][j]);
        // cout << grid[i][j] << " ";
      }
      // cout << endl;
    }
    // cout << endl;
    total_coverages.push_back(coverage);
  }
  cout << "total_coverages " << total_coverages.size() << endl;
  vector<int> total_coverage_previous;
  for(int i = 0; i < total_coverages[0].size(); i++){
    int total_cell = 0;
    for(int j = 0; j < total_coverages.size()-1; j++){
      total_cell += total_coverages[j][i];
    }
    total_coverage_previous.push_back(total_cell);
    // cout << total_cell << " ";
  }
  // cout << endl;
  double filled_cells = 0;
  double filled_cells_alot = 0;
  for(int i = 0; i < total_coverage_previous.size(); i++){
    if(total_coverage_previous[i] > 0){
      filled_cells++;
      if(total_coverage_previous[i] > 3){
        filled_cells_alot++;
      }
    }
  }
  double percent_alot = filled_cells_alot / filled_cells;
  cout << "filled_cells " << filled_cells << " filled_cells_alot " << filled_cells_alot << " percent_alot " << percent_alot << endl;
  double num_new = 0;
  for(int i = 0; i < total_coverage_previous.size(); i++){
    if(total_coverages[total_coverages.size()-1][i] > 0 and total_coverage_previous[i] == 0){
      num_new++;
    }
  }
  cout << "num_new " << num_new << endl;
  if(percent_alot >= 0.75 and num_new <= 1){
    robotConfined = true;
  }
  else{
    robotConfined = false;
  }
  cout << "robotConfined " << robotConfined << endl;
  return robotConfined;
}

vector <Position> AgentState::getCrowdPositions(geometry_msgs::msg::PoseArray crowdpose){
  vector <Position> crowdPositions;
  for(int i = 0; i < crowdpose.poses.size(); i++){
    double x = crowdpose.poses[i].position.x;
    double y = crowdpose.poses[i].position.y;
    tf2::Quaternion q(crowdpose.poses[i].orientation.x,crowdpose.poses[i].orientation.y,crowdpose.poses[i].orientation.z,crowdpose.poses[i].orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    crowdPositions.push_back(Position(x,y,yaw));
  }
  return crowdPositions;
}

bool AgentState::crowdModelLearned(){
  std::vector<double> densities = crowdModel.densities;
  for(int i = 0; i < densities.size() ; i++){
    if(densities[i]>0){
      return true;
    }
  }
  return false;
}

bool AgentState::riskModelLearned(){
  std::vector<double> risk = crowdModel.risk;
  for(int i = 0; i < risk.size() ; i++){
    if(risk[i]>0){
      return true;
    }
  }
  return false;
}

bool AgentState::flowModelLearned(){
  std::vector<double> left = crowdModel.left;
  for(int i = 0; i < left.size() ; i++){
    if(left[i]>0){
      return true;
    }
  }
  std::vector<double> right = crowdModel.right;
  for(int i = 0; i < right.size() ; i++){
    if(right[i]>0){
      return true;
    }
  }
  std::vector<double> up = crowdModel.up;
  for(int i = 0; i < up.size() ; i++){
    if(up[i]>0){
      return true;
    }
  }
  std::vector<double> down = crowdModel.down;
  for(int i = 0; i < down.size() ; i++){
    if(down[i]>0){
      return true;
    }
  }
  std::vector<double> up_left = crowdModel.up_left;
  for(int i = 0; i < up_left.size() ; i++){
    if(up_left[i]>0){
      return true;
    }
  }
  std::vector<double> up_right = crowdModel.up_right;
  for(int i = 0; i < up_right.size() ; i++){
    if(up_right[i]>0){
      return true;
    }
  }
  std::vector<double> down_left = crowdModel.down_left;
  for(int i = 0; i < down_left.size() ; i++){
    if(down_left[i]>0){
      return true;
    }
  }
  std::vector<double> down_right = crowdModel.down_right;
  for(int i = 0; i < down_right.size() ; i++){
    if(down_right[i]>0){
      return true;
    }
  }
  return false;
}

double AgentState::getGridValue(double x, double y){
  int resolution = crowdModel.resolution;
  int height = crowdModel.height;
  int width = crowdModel.width;
  //std::vector<double> densities = crowdModel.densities;
  double gridValue = crowdModel.densities[(floor(y/resolution)*width)+floor(x/resolution)];
  //cout << "resolution = " << resolution << " height = " << height << " width = " << width << " gridValue = " << gridValue << endl;
  return gridValue;
}

double AgentState::getRiskValue(double x, double y){
  int resolution = crowdModel.resolution;
  int height = crowdModel.height;
  int width = crowdModel.width;
  //std::vector<double> risk = crowdModel.risk;
  double riskValue = crowdModel.risk[(floor(y/resolution)*width)+floor(x/resolution)];
  //cout << "resolution = " << resolution << " height = " << height << " width = " << width << " riskValue = " << riskValue << endl;
  return riskValue;
}

double AgentState::getFlowValue(double x, double y, double theta){
  int resolution = crowdModel.resolution;
  int height = crowdModel.height;
  int width = crowdModel.width;
  //std::vector<double> left = crowdModel.left;
  double leftValue = crowdModel.left[(floor(y/resolution)*width)+floor(x/resolution)];
  //std::vector<double> right = crowdModel.right;
  double rightValue = crowdModel.right[(floor(y/resolution)*width)+floor(x/resolution)];
  //std::vector<double> up = crowdModel.up;
  double upValue = crowdModel.up[(floor(y/resolution)*width)+floor(x/resolution)];
  //std::vector<double> down = crowdModel.down;
  double downValue = crowdModel.down[(floor(y/resolution)*width)+floor(x/resolution)];
  //std::vector<double> up_left = crowdModel.up_left;
  double up_leftValue = crowdModel.up_left[(floor(y/resolution)*width)+floor(x/resolution)];
  //std::vector<double> up_right = crowdModel.up_right;
  double up_rightValue = crowdModel.up_right[(floor(y/resolution)*width)+floor(x/resolution)];
  //std::vector<double> down_left = crowdModel.down_left;
  double down_leftValue = crowdModel.down_left[(floor(y/resolution)*width)+floor(x/resolution)];
  //std::vector<double> down_right = crowdModel.down_right;
  double down_rightValue = crowdModel.down_right[(floor(y/resolution)*width)+floor(x/resolution)];

  double totalX = leftValue*cos(M_PI) + rightValue*cos(0) + upValue*cos(M_PI/2) + downValue*cos(3*M_PI/2) + up_leftValue*cos(3*M_PI/4) + up_rightValue*cos(M_PI/4) + down_leftValue*cos(5*M_PI/4) + down_rightValue*cos(7*M_PI/4);
  double totalY = leftValue*sin(M_PI) + rightValue*sin(0) + upValue*sin(M_PI/2) + downValue*sin(3*M_PI/2) + up_leftValue*sin(3*M_PI/4) + up_rightValue*sin(M_PI/4) + down_leftValue*sin(5*M_PI/4) + down_rightValue*sin(7*M_PI/4);
  double flowMagnitude = sqrt(totalX*totalX + totalY*totalY);
  double flowTheta = atan2(totalY, totalX);

  double angleDiff = min(abs(flowTheta - theta),(2*M_PI) - abs(flowTheta - theta));
  double flowValue = 0;
  if(angleDiff<=M_PI/4){
    flowValue = flowMagnitude;
  }
  else if(angleDiff<=M_PI/2){
    flowValue = flowMagnitude/2;
  }
  else if(angleDiff<=3*M_PI/4){
    flowValue = -flowMagnitude/2;
  }
  else if(angleDiff>3*M_PI/4){
    flowValue = -flowMagnitude;
  }

  //cout << "resolution = " << resolution << " height = " << height << " width = " << width << " flowValue = " << flowValue << endl;
  return flowValue;
}

double AgentState::getCrowdObservation(double x, double y){
  int resolution = crowdModel.resolution;
  int height = crowdModel.height;
  int width = crowdModel.width;
  double crowdObservationValue = crowdModel.crowd_observations[(floor(y/resolution)*width)+floor(x/resolution)];
  //cout << "resolution = " << resolution << " height = " << height << " width = " << width << " crowdObservationValue = " << crowdObservationValue << endl;
  return crowdObservationValue;
}

double AgentState::getRiskExperience(double x, double y){
  int resolution = crowdModel.resolution;
  int height = crowdModel.height;
  int width = crowdModel.width;
  double riskExperienceValue = crowdModel.risk_experiences[(floor(y/resolution)*width)+floor(x/resolution)];
  //cout << "resolution = " << resolution << " height = " << height << " width = " << width << " riskExperienceValue = " << riskExperienceValue << endl;
  return riskExperienceValue;
}

double AgentState::getFLowObservation(double x, double y){
  int resolution = crowdModel.resolution;
  int height = crowdModel.height;
  int width = crowdModel.width;
  double leftValue = crowdModel.left[(floor(y/resolution)*width)+floor(x/resolution)];
  double rightValue = crowdModel.right[(floor(y/resolution)*width)+floor(x/resolution)];
  double upValue = crowdModel.up[(floor(y/resolution)*width)+floor(x/resolution)];
  double downValue = crowdModel.down[(floor(y/resolution)*width)+floor(x/resolution)];
  double up_leftValue = crowdModel.up_left[(floor(y/resolution)*width)+floor(x/resolution)];
  double up_rightValue = crowdModel.up_right[(floor(y/resolution)*width)+floor(x/resolution)];
  double down_leftValue = crowdModel.down_left[(floor(y/resolution)*width)+floor(x/resolution)];
  double down_rightValue = crowdModel.down_right[(floor(y/resolution)*width)+floor(x/resolution)];

  double totalX = leftValue*cos(M_PI) + rightValue*cos(0) + upValue*cos(M_PI/2) + downValue*cos(3*M_PI/2) + up_leftValue*cos(3*M_PI/4) + up_rightValue*cos(M_PI/4) + down_leftValue*cos(5*M_PI/4) + down_rightValue*cos(7*M_PI/4);
  double totalY = leftValue*sin(M_PI) + rightValue*sin(0) + upValue*sin(M_PI/2) + downValue*sin(3*M_PI/2) + up_leftValue*sin(3*M_PI/4) + up_rightValue*sin(M_PI/4) + down_leftValue*sin(5*M_PI/4) + down_rightValue*sin(7*M_PI/4);
  double flowMagnitude = sqrt(totalX*totalX + totalY*totalY);
  double crowdObservationValue = crowdModel.crowd_observations[(floor(y/resolution)*width)+floor(x/resolution)];
  return flowMagnitude*crowdObservationValue;
}

void AgentState::dfs(int x, int y, int current_label, vector<int> dx, vector<int> dy, int row_count, int col_count, vector< vector<int> > *label, vector< vector<int> > *m) {
  if (x < 0 || x == row_count) return; // out of bounds
  if (y < 0 || y == col_count) return; // out of bounds
  if ((*label)[x][y] != 0 || (*m)[x][y] < 0) return; // already labeled or not marked with 1 in m

  // mark the current cell
  (*label)[x][y] = current_label;

  // recursively mark the neighbors
  for (int direction = 0; direction < dx.size(); ++direction)
    dfs(x + dx[direction], y + dy[direction], current_label, dx, dy, row_count, col_count, label, m);
}