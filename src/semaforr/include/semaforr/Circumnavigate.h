#ifndef CIRCUMNAVIGATE_H
#define CIRCUMNAVIGATE_H

/**!
  * Circumnavigate.h
  * 
  * /author: Raj Korpan
  *
  *          Circumnavigate to get to goal
  */

#include <vector>
#include "FORRAction.h"
#include "Beliefs.h"

using namespace std;

class Circumnavigate{
public:
	Circumnavigate(int l, int h, double arrMove[], double arrRotate[], int moveArrMax, int rotateArrMax, Beliefs *b, PathPlanner *s){
		length = l;
		height = h;
		numMoves = moveArrMax;
		numRotates = rotateArrMax;
		for(int i = 0 ; i < numMoves ; i++) move[i] = arrMove[i];
		for(int i = 0 ; i < numRotates ; i++) rotate[i] = arrRotate[i];
		beliefs = b;
		skeleton_planner = s;
		for(int i = 0; i < length; i++){
			vector<int> col;
			for(int j = 0; j < height; j ++){
				col.push_back(-1);
			}
			visited_grid.push_back(col);
		}
		foundAlignmentPoint = false;
		currentAlignmentPoint = Position(0,0,0);
		gotToAlignmentPoint = false;
		checkedForOpeningPoint = false;
		foundOpeningPoint = false;
		currentOpeningPoint = Position(0,0,0);
		currOpeningPointAlign = Position(0,0,0);
		openingPointAligned = false;
		followingPerpDirection = false;
		currentPerpDirection = -1;
	};
	~Circumnavigate(){};

	void resetCircumnavigate(){
		visited_grid.clear();
		for(int i = 0; i < length; i++){
			vector<int> col;
			for(int j = 0; j < height; j ++){
				col.push_back(-1);
			}
			visited_grid.push_back(col);
		}
		foundAlignmentPoint = false;
		currentAlignmentPoint = Position(0,0,0);
		gotToAlignmentPoint = false;
		checkedForOpeningPoint = false;
		foundOpeningPoint = false;
		currentOpeningPoint = Position(0,0,0);
		currOpeningPointAlign = Position(0,0,0);
		openingPointAligned = false;
		followingPerpDirection = false;
		currentPerpDirection = -1;
		alignmentPoints.clear();
		openingPoints.clear();
		openingPointsAligns.clear();
		perpendicularDirections.clear();
	}

	void addToStack(Position new_pose, sensor_msgs::msg::LaserScan new_laser){
		double min_distance = 1000;
		for(int i = 0; i < new_laser.ranges.size(); i++){
			if(new_laser.ranges[i] < min_distance){
				min_distance = new_laser.ranges[i];
			}
		}
		cout << "min_distance " << min_distance << endl;
		visited_grid[(int)(new_pose.getX())][(int)(new_pose.getY())] = 1;
		visited_grid[(int)(new_pose.getX()+min_distance)][(int)(new_pose.getY())] = 1;
		visited_grid[(int)(new_pose.getX())][(int)(new_pose.getY()+min_distance)] = 1;
		visited_grid[(int)(new_pose.getX()-min_distance)][(int)(new_pose.getY())] = 1;
		visited_grid[(int)(new_pose.getX())][(int)(new_pose.getY()-min_distance)] = 1;
		visited_grid[(int)(new_pose.getX()+min_distance)][(int)(new_pose.getY()+min_distance)] = 1;
		visited_grid[(int)(new_pose.getX()+min_distance)][(int)(new_pose.getY()-min_distance)] = 1;
		visited_grid[(int)(new_pose.getX()-min_distance)][(int)(new_pose.getY()+min_distance)] = 1;
		visited_grid[(int)(new_pose.getX()-min_distance)][(int)(new_pose.getY()-min_distance)] = 1;
		cout << "Visited grid" << endl;
		for(int i = 0; i < visited_grid[0].size(); i++){
			for(int j = 0; j < visited_grid.size(); j++){
				cout << visited_grid[j][i] << " ";
			}
			cout << endl;
		}
	}

	bool advisorCircumnavigate(FORRAction *decision){
		bool decisionMade = false;
		CartesianPoint task(beliefs->getAgentState()->getCurrentTask()->getTaskX(),beliefs->getAgentState()->getCurrentTask()->getTaskY());
		Position currentPosition = beliefs->getAgentState()->getCurrentPosition();
		cout << "Target = " << task.get_x() << " " << task.get_y() << " Current Position = " << currentPosition.getX() << " " << currentPosition.getY() << " " << currentPosition.getTheta() << endl;
		addToStack(beliefs->getAgentState()->getCurrentPosition(), beliefs->getAgentState()->getCurrentLaserScan());
		if(foundAlignmentPoint == false){
			CartesianPoint potentialAlignmentPoint1 = CartesianPoint(task.get_x(), currentPosition.getY());
			CartesianPoint potentialAlignmentPoint2 = CartesianPoint(currentPosition.getX(), task.get_y());
			bool can_see_1 = beliefs->getAgentState()->canSeePoint(potentialAlignmentPoint1, 25);
			bool can_see_2 = beliefs->getAgentState()->canSeePoint(potentialAlignmentPoint2, 25);
			if(can_see_1){
				foundAlignmentPoint = true;
				alignmentPoints.push_back(Position(potentialAlignmentPoint1.get_x(), potentialAlignmentPoint1.get_y(), 0));
				cout << "alignmentPoint 1 = " << potentialAlignmentPoint1.get_x() << " " << potentialAlignmentPoint1.get_y() << endl;
			}
			if(can_see_2){
				foundAlignmentPoint = true;
				alignmentPoints.push_back(Position(potentialAlignmentPoint2.get_x(), potentialAlignmentPoint2.get_y(), 0));
				cout << "alignmentPoint 2 = " << potentialAlignmentPoint2.get_x() << " " << potentialAlignmentPoint2.get_y() << endl;
			}
			if(foundAlignmentPoint == true){
				currentAlignmentPoint = alignmentPoints[0];
				alignmentPoints.erase(alignmentPoints.begin());
			}
		}
		if(foundAlignmentPoint == true and gotToAlignmentPoint == false){
			set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
			double robot_direction = currentPosition.getTheta();
			double goal_direction = atan2((task.get_y() - currentPosition.getY()), (task.get_x() - currentPosition.getX()));
			double required_rotation = goal_direction - robot_direction;
			if(required_rotation > M_PI)
				required_rotation = required_rotation - (2*M_PI);
			if(required_rotation < -M_PI)
				required_rotation = required_rotation + (2*M_PI);
			cout << "robot_direction " << robot_direction << " goal_direction " << goal_direction << " required_rotation " << required_rotation << " distance " << currentPosition.getDistance(currentAlignmentPoint) << endl;
			if(currentPosition.getDistance(currentAlignmentPoint) > 0.75){
				cout << "move towards currentAlignmentPoint" << endl;
				CartesianPoint align(currentAlignmentPoint.getX(),currentAlignmentPoint.getY());
				(*decision) = beliefs->getAgentState()->moveTowards(align);
				FORRAction forward = beliefs->getAgentState()->maxForwardAction();
				Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
				if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
					if(vetoed_actions->find(*decision) != vetoed_actions->end()){
						decisionMade = false;
					}
					else{
						cout << "Circumnavigate advisor to take decision" << endl;
						decisionMade = true;
					}
				}
			}
			else if(fabs(required_rotation) > 0.174532925){
				cout << "rotate to face target" << endl;
				int rotIntensity=0;
				while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
					rotIntensity++;
				}
				int max_allowed = beliefs->getAgentState()->maxForwardAction().parameter;
				if (rotIntensity > 1) {
					if (required_rotation < 0){
						(*decision) = FORRAction(RIGHT_TURN, rotIntensity-1);
					}
					else {
						(*decision) = FORRAction(LEFT_TURN, rotIntensity-1);
					}
					if(vetoed_actions->find(*decision) != vetoed_actions->end()){
						decisionMade = false;
					}
					else{
						cout << "Circumnavigate advisor to take decision" << endl;
						decisionMade = true;
					}
				}
			}
			else{
				cout << "on currentAlignmentPoint and facing target" << endl;
				gotToAlignmentPoint = true;
			}
		}
		if(foundAlignmentPoint == true and gotToAlignmentPoint == true and checkedForOpeningPoint == false){
			cout << "on alignmentpoint, look for opening" << endl;
			set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
			double robot_direction = currentPosition.getTheta();
			double goal_direction = atan2((task.get_y() - currentPosition.getY()), (task.get_x() - currentPosition.getX()));
			double required_rotation = goal_direction - robot_direction;
			if(required_rotation > M_PI)
				required_rotation = required_rotation - (2*M_PI);
			if(required_rotation < -M_PI)
				required_rotation = required_rotation + (2*M_PI);
			vector<CartesianPoint> currentLaserEndpoints = beliefs->getAgentState()->getCurrentLaserEndpoints();
			vector < vector<int> > laserGrid;
			for(int i = 0; i < length; i++){
				vector<int> col;
				for(int j = 0; j < height; j ++){
					col.push_back(0);
				}
				laserGrid.push_back(col);
			}
			for(int i = 0; i < currentLaserEndpoints.size(); i++){
				laserGrid[(int)(currentLaserEndpoints[i].get_x())][(int)(currentLaserEndpoints[i].get_y())] = 1;
			}
			for(int i = 0; i < laserGrid.size(); i++){
				for(int j = 0; j < laserGrid[i].size(); j++){
					if(i == (int)(currentPosition.getX()) and j == (int)(currentPosition.getY())){
						cout << 2 << " ";
					}
					else if(i == (int)(task.get_x()) and j == (int)(task.get_y())){
						cout << 3 << " ";
					}
					else{
						cout << laserGrid[i][j] << " ";
					}
				}
				cout << endl;
			}
			if(fabs(robot_direction) > M_PI/4 and fabs(robot_direction) < 3*M_PI/4){
				cout << "facing north or south" << endl;
				int start_row = (int)(currentLaserEndpoints[(int)(currentLaserEndpoints.size()/2)].get_x());
				int col = (int)(currentLaserEndpoints[(int)(currentLaserEndpoints.size()/2)].get_y());
				int end_row_r = start_row;
				int end_row_l = start_row;
				int value = laserGrid[start_row][col];
				while(value > 0 and end_row_r < laserGrid.size()-1){
					end_row_r = end_row_r + 1;
					value = laserGrid[end_row_r][col];
				}
				end_row_r = end_row_r - 1;
				value = laserGrid[start_row][col];
				while(value > 0 and end_row_l > 0){
					end_row_l = end_row_l - 1;
					value = laserGrid[end_row_l][col];
				}
				end_row_l = end_row_l + 1;
				cout << "col " << col << " start_row " << start_row << " end_row_l " << end_row_l << " end_row_r " << end_row_r << endl;
				if(laserGrid[end_row_r+1][col-1] == 0 and laserGrid[end_row_r+1][col] == 0 and laserGrid[end_row_r+1][col+1] == 0){
					cout << "found an opening 1 " << end_row_r+1 << " " << col << endl;
					foundOpeningPoint = true;
					openingPoints.push_back(Position(end_row_r+1, col, 0));
					openingPointsAligns.push_back(Position(end_row_r+1, currentPosition.getY(), currentPosition.getTheta()));
				}
				else if(laserGrid[end_row_l-1][col-1] == 0 and laserGrid[end_row_l-1][col] == 0 and laserGrid[end_row_l-1][col+1] == 0){
					cout << "found an opening 2 " << end_row_l-1 << " " << col << endl;
					foundOpeningPoint = true;
					openingPoints.push_back(Position(end_row_l-1, col, 0));
					openingPointsAligns.push_back(Position(end_row_l-1, currentPosition.getY(), currentPosition.getTheta()));
				}
				else{
					cout << "no opening visible" << endl;
					foundOpeningPoint = false;
				}
			}
			else{
				cout << "facing east or west" << endl;
				int row = (int)(currentLaserEndpoints[(int)(currentLaserEndpoints.size()/2)].get_x());
				int start_col = (int)(currentLaserEndpoints[(int)(currentLaserEndpoints.size()/2)].get_y());
				int end_col_r = start_col;
				int end_col_l = start_col;
				int value = laserGrid[row][start_col];
				while(value > 0 and end_col_r < laserGrid[0].size()-1){
					end_col_r = end_col_r + 1;
					value = laserGrid[row][end_col_r];
				}
				end_col_r = end_col_r - 1;
				value = laserGrid[row][start_col];
				while(value > 0 and end_col_l > 0){
					end_col_l = end_col_l - 1;
					value = laserGrid[row][end_col_l];
				}
				end_col_l = end_col_l + 1;
				cout << "row " << row << " start_col " << start_col << " end_col_l " << end_col_l << " end_col_r " << end_col_r << endl;
				if(laserGrid[row-1][end_col_l-1] == 0 and laserGrid[row][end_col_l-1] == 0 and laserGrid[row+1][end_col_l-1] == 0){
					cout << "found an opening 1 " << row << " " << end_col_l-1 << endl;
					foundOpeningPoint = true;
					openingPoints.push_back(Position(row, end_col_l-1, 0));
					openingPointsAligns.push_back(Position(currentPosition.getX(), end_col_l-1, currentPosition.getTheta()));
				}
				else if(laserGrid[row-1][end_col_r+1] == 0 and laserGrid[row][end_col_r+1] == 0 and laserGrid[row+1][end_col_r+1] == 0){
					cout << "found an opening 2" << endl;
					foundOpeningPoint = true;
					openingPoints.push_back(Position(row, end_col_r+1, 0));
					openingPointsAligns.push_back(Position(currentPosition.getX(), end_col_r+1, currentPosition.getTheta()));
				}
				else{
					cout << "no opening visible" << endl;
					foundOpeningPoint = false;
				}
			}
			checkedForOpeningPoint = true;
			if(foundOpeningPoint == true){
				currentOpeningPoint = openingPoints[0];
				openingPoints.erase(openingPoints.begin());
				currOpeningPointAlign = openingPointsAligns[0];
				openingPointsAligns.erase(openingPointsAligns.begin());
			}
		}
		if(foundAlignmentPoint == true and gotToAlignmentPoint == true and checkedForOpeningPoint == true and foundOpeningPoint == true){
			set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
			if(openingPointAligned == false){
				set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
				cout << "distance " << currentPosition.getDistance(currOpeningPointAlign) << endl;
				if(currentPosition.getDistance(currOpeningPointAlign) > 0.5 and beliefs->getAgentState()->canSeePoint(CartesianPoint(currOpeningPointAlign.getX(),currOpeningPointAlign.getY()), 25)){
					cout << "move towards currOpeningPointAlign" << endl;
					CartesianPoint align(currOpeningPointAlign.getX(),currOpeningPointAlign.getY());
					(*decision) = beliefs->getAgentState()->moveTowards(align);
					FORRAction forward = beliefs->getAgentState()->maxForwardAction();
					Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
					if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
						if(vetoed_actions->find(*decision) != vetoed_actions->end()){
							decisionMade = false;
						}
						else{
							cout << "Circumnavigate advisor to take decision" << endl;
							decisionMade = true;
						}
					}
				}
				else if(currentPosition.getDistance(currOpeningPointAlign) <= 0.5){
					cout << "on currOpeningPointAlign" << endl;
					openingPointAligned = true;
				}
				else if(!beliefs->getAgentState()->canSeePoint(CartesianPoint(currOpeningPointAlign.getX(),currOpeningPointAlign.getY()), 25)){
					cout << "move forward" << endl;
					CartesianPoint align;
					if(currentOpeningPoint.getX() == currOpeningPointAlign.getX()){
						align = CartesianPoint(currOpeningPointAlign.getX(), currentPosition.getY());
					}
					else if(currentOpeningPoint.getX() == currOpeningPointAlign.getX()){
						align = CartesianPoint(currOpeningPointAlign.getX(), currentPosition.getY());
					}
					// CartesianPoint align(currentOpeningPoint.getX(),currentOpeningPoint.getY());
					(*decision) = beliefs->getAgentState()->moveTowards(align);
					FORRAction forward = beliefs->getAgentState()->maxForwardAction();
					Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
					if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
						if(vetoed_actions->find(*decision) != vetoed_actions->end()){
							decisionMade = false;
						}
						else{
							cout << "Circumnavigate advisor to take decision" << endl;
							decisionMade = true;
						}
					}
				}
			}
			else{
				cout << "move towards currentOpeningPoint" << endl;
				CartesianPoint align(currentOpeningPoint.getX(),currentOpeningPoint.getY());
				(*decision) = beliefs->getAgentState()->moveTowards(align);
				FORRAction forward = beliefs->getAgentState()->maxForwardAction();
				Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
				if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
					if(vetoed_actions->find(*decision) != vetoed_actions->end()){
						decisionMade = false;
					}
					else{
						cout << "Circumnavigate advisor to take decision" << endl;
						decisionMade = true;
					}
				}
			}
		}
		else if(foundAlignmentPoint == true and gotToAlignmentPoint == true and checkedForOpeningPoint == true and foundOpeningPoint == false){
			cout << "no opening point" << endl;
		}










		
		
		
		
		// if(foundAlignmentPoint == true and goToStart == false and foundOpeningPoint == true and decisionMade == false and movingTowardsTarget == false and movingPerpendicularTarget == false){
		// 	cout << "go towards opening point" << endl;
		// 	set<FORRAction> *vetoed_actions = beliefs->getAgentState()->getVetoedActions();
		// 	double robot_direction = currentPosition.getTheta();
		// 	double goal_direction = atan2((task.get_y() - currentPosition.getY()), (task.get_x() - currentPosition.getX()));
		// 	double required_rotation = goal_direction - robot_direction;
		// 	if(required_rotation > M_PI)
		// 		required_rotation = required_rotation - (2*M_PI);
		// 	if(required_rotation < -M_PI)
		// 		required_rotation = required_rotation + (2*M_PI);
		// 	cout << "robot_direction " << robot_direction << " goal_direction " << goal_direction << " required_rotation " << required_rotation << " distance " << currentPosition.getDistance(alignmentPoint) << endl;
		// 	if(currentPosition.getDistance(openingPoint) > 0.75){
		// 		cout << "move towards openingPoint" << endl;
		// 		CartesianPoint align(openingPoint.getX(),openingPoint.getY());
		// 		(*decision) = beliefs->getAgentState()->moveTowards(align);
		// 		FORRAction forward = beliefs->getAgentState()->maxForwardAction();
		// 		Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
		// 		if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
		// 			if(vetoed_actions->find(*decision) != vetoed_actions->end()){
		// 				decisionMade = false;
		// 			}
		// 			else{
		// 				cout << "Circumnavigate advisor to take decision" << endl;
		// 				decisionMade = true;
		// 			}
		// 		}
		// 	}
		// 	else{
		// 		cout << "reached openingPoint" << endl;
		// 		if((targetDirection == 0 and beliefs->getAgentState()->canSeePoint(CartesianPoint(currentPosition.getX(),currentPosition.getY()+0.5), 25)) or (targetDirection == 1 and beliefs->getAgentState()->canSeePoint(CartesianPoint(currentPosition.getX(),currentPosition.getY()-0.5), 25)) or (targetDirection == 2 and beliefs->getAgentState()->canSeePoint(CartesianPoint(currentPosition.getX()+0.5,currentPosition.getY()), 25)) or (targetDirection == 3 and beliefs->getAgentState()->canSeePoint(CartesianPoint(currentPosition.getX()-0.5,currentPosition.getY()), 25))){
		// 			movingTowardsTarget = true;
		// 			movingPerpendicularTarget = false;
		// 		}
		// 		else{
		// 			movingTowardsTarget = false;
		// 			movingPerpendicularTarget = true;
		// 		}
		// 	}
		// }
		// if(foundAlignmentPoint == true and goToStart == false and foundOpeningPoint == false and decisionMade == false and movingTowardsTarget == false and movingPerpendicularTarget == false){
		// 	cout << "move perpendicular because no opening point" << endl;
		// 	movingTowardsTarget = false;
		// 	movingPerpendicularTarget = true;
		// }
		// if(foundAlignmentPoint == true and goToStart == false and movingPerpendicularTarget == true and decisionMade == false and movingTowardsTarget == false){
		// 	if(perpDirection == 0){
		// 		cout << "move north" << endl;
		// 		CartesianPoint align(currentPosition.getX(),currentPosition.getY()+0.5);
		// 		(*decision) = beliefs->getAgentState()->moveTowards(align);
		// 		FORRAction forward = beliefs->getAgentState()->maxForwardAction();
		// 		Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
		// 		if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
		// 			if(vetoed_actions->find(*decision) != vetoed_actions->end()){
		// 				decisionMade = false;
		// 			}
		// 			else{
		// 				cout << "Circumnavigate advisor to take decision" << endl;
		// 				decisionMade = true;
		// 			}
		// 		}
		// 	}
		// 	else if(perpDirection == 1){
		// 		cout << "move south" << endl;
		// 		CartesianPoint align(currentPosition.getX(),currentPosition.getY()-0.5);
		// 		(*decision) = beliefs->getAgentState()->moveTowards(align);
		// 		FORRAction forward = beliefs->getAgentState()->maxForwardAction();
		// 		Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
		// 		if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
		// 			if(vetoed_actions->find(*decision) != vetoed_actions->end()){
		// 				decisionMade = false;
		// 			}
		// 			else{
		// 				cout << "Circumnavigate advisor to take decision" << endl;
		// 				decisionMade = true;
		// 			}
		// 		}
		// 	}
		// 	else if(perpDirection == 2){
		// 		cout << "move east" << endl;
		// 		CartesianPoint align(currentPosition.getX()+0.5,currentPosition.getY());
		// 		(*decision) = beliefs->getAgentState()->moveTowards(align);
		// 		FORRAction forward = beliefs->getAgentState()->maxForwardAction();
		// 		Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
		// 		if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
		// 			if(vetoed_actions->find(*decision) != vetoed_actions->end()){
		// 				decisionMade = false;
		// 			}
		// 			else{
		// 				cout << "Circumnavigate advisor to take decision" << endl;
		// 				decisionMade = true;
		// 			}
		// 		}
		// 	}
		// 	else if(perpDirection == 3){
		// 		cout << "move west" << endl;
		// 		CartesianPoint align(currentPosition.getX()-0.5,currentPosition.getY());
		// 		(*decision) = beliefs->getAgentState()->moveTowards(align);
		// 		FORRAction forward = beliefs->getAgentState()->maxForwardAction();
		// 		Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
		// 		if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
		// 			if(vetoed_actions->find(*decision) != vetoed_actions->end()){
		// 				decisionMade = false;
		// 			}
		// 			else{
		// 				cout << "Circumnavigate advisor to take decision" << endl;
		// 				decisionMade = true;
		// 			}
		// 		}
		// 	}
		// 	else{
		// 		movingPerpendicularTarget = false;
		// 	}
		// 	if(decisionMade == false){
		// 		movingPerpendicularTarget = false;
		// 	}
		// 	if((targetDirection == 0 and beliefs->getAgentState()->canSeePoint(CartesianPoint(currentPosition.getX(),currentPosition.getY()+1), 25)) or (targetDirection == 1 and beliefs->getAgentState()->canSeePoint(CartesianPoint(currentPosition.getX(),currentPosition.getY()-1), 25)) or (targetDirection == 2 and beliefs->getAgentState()->canSeePoint(CartesianPoint(currentPosition.getX()+1,currentPosition.getY()), 25)) or (targetDirection == 3 and beliefs->getAgentState()->canSeePoint(CartesianPoint(currentPosition.getX()-1,currentPosition.getY()), 25))){
		// 		movingTowardsTarget = true;
		// 		movingPerpendicularTarget = false;
		// 		decisionMade = false;
		// 	}
		// }
		// if(foundAlignmentPoint == true and goToStart == false and movingTowardsTarget == true and decisionMade == false and movingPerpendicularTarget == false){
		// 	if(targetDirection == 0){
		// 		cout << "move north" << endl;
		// 		CartesianPoint align(currentPosition.getX(),currentPosition.getY()+0.5);
		// 		(*decision) = beliefs->getAgentState()->moveTowards(align);
		// 		FORRAction forward = beliefs->getAgentState()->maxForwardAction();
		// 		Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
		// 		if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
		// 			if(vetoed_actions->find(*decision) != vetoed_actions->end()){
		// 				decisionMade = false;
		// 			}
		// 			else{
		// 				cout << "Circumnavigate advisor to take decision" << endl;
		// 				decisionMade = true;
		// 			}
		// 		}
		// 	}
		// 	else if(targetDirection == 1){
		// 		cout << "move south" << endl;
		// 		CartesianPoint align(currentPosition.getX(),currentPosition.getY()-0.5);
		// 		(*decision) = beliefs->getAgentState()->moveTowards(align);
		// 		FORRAction forward = beliefs->getAgentState()->maxForwardAction();
		// 		Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
		// 		if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
		// 			if(vetoed_actions->find(*decision) != vetoed_actions->end()){
		// 				decisionMade = false;
		// 			}
		// 			else{
		// 				cout << "Circumnavigate advisor to take decision" << endl;
		// 				decisionMade = true;
		// 			}
		// 		}
		// 	}
		// 	else if(targetDirection == 2){
		// 		cout << "move east" << endl;
		// 		CartesianPoint align(currentPosition.getX()+0.5,currentPosition.getY());
		// 		(*decision) = beliefs->getAgentState()->moveTowards(align);
		// 		FORRAction forward = beliefs->getAgentState()->maxForwardAction();
		// 		Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
		// 		if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
		// 			if(vetoed_actions->find(*decision) != vetoed_actions->end()){
		// 				decisionMade = false;
		// 			}
		// 			else{
		// 				cout << "Circumnavigate advisor to take decision" << endl;
		// 				decisionMade = true;
		// 			}
		// 		}
		// 	}
		// 	else if(targetDirection == 3){
		// 		cout << "move west" << endl;
		// 		CartesianPoint align(currentPosition.getX()-0.5,currentPosition.getY());
		// 		(*decision) = beliefs->getAgentState()->moveTowards(align);
		// 		FORRAction forward = beliefs->getAgentState()->maxForwardAction();
		// 		Position expectedPosition = beliefs->getAgentState()->getExpectedPositionAfterAction((*decision));
		// 		if(((decision->type == RIGHT_TURN or decision->type == LEFT_TURN) or (forward.parameter >= decision->parameter)) and decision->parameter != 0 and expectedPosition.getDistance(beliefs->getAgentState()->getCurrentPosition()) >= 0.1){
		// 			if(vetoed_actions->find(*decision) != vetoed_actions->end()){
		// 				decisionMade = false;
		// 			}
		// 			else{
		// 				cout << "Circumnavigate advisor to take decision" << endl;
		// 				decisionMade = true;
		// 			}
		// 		}
		// 	}
		// 	else{
		// 		movingTowardsTarget = false;
		// 	}
		// 	if(decisionMade == false){
		// 		movingTowardsTarget = false;
		// 	}
		// 	if((perpDirection == 0 and beliefs->getAgentState()->canSeePoint(CartesianPoint(currentPosition.getX(),currentPosition.getY()+1), 25)) or (perpDirection == 1 and beliefs->getAgentState()->canSeePoint(CartesianPoint(currentPosition.getX(),currentPosition.getY()-1), 25)) or (perpDirection == 2 and beliefs->getAgentState()->canSeePoint(CartesianPoint(currentPosition.getX()+1,currentPosition.getY()), 25)) or (perpDirection == 3 and beliefs->getAgentState()->canSeePoint(CartesianPoint(currentPosition.getX()-1,currentPosition.getY()), 25))){
		// 		movingTowardsTarget = false;
		// 		movingPerpendicularTarget = true;
		// 		decisionMade = false;
		// 	}
		// }
		cout << "decisionMade " << decisionMade << endl;
		return decisionMade;
	}

private:
	int length;
	int height;
	double move[300];  
	double rotate[300];
	int numMoves, numRotates;
	vector< vector<int> > visited_grid;
	Beliefs *beliefs;
	PathPlanner *skeleton_planner;
	bool foundAlignmentPoint;
	vector<Position> alignmentPoints;
	bool gotToAlignmentPoint;
	Position currentAlignmentPoint;
	bool checkedForOpeningPoint;
	bool foundOpeningPoint;
	vector<Position> openingPoints;
	Position currentOpeningPoint;
	vector<Position> openingPointsAligns;
	Position currOpeningPointAlign;
	bool openingPointAligned;
	bool followingPerpDirection;
	vector<int> perpendicularDirections;
	int currentPerpDirection;
};

#endif