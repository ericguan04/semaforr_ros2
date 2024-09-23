#ifndef FRONTIEREXPLORE_H
#define FRONTIEREXPLORE_H

/**!
  * FrontierExplore.h
  * 
  * /author: Raj Korpan
  *
  *          Construct Frontier Graph
  */

#include <FORRGeometry.h>
#include <Position.h>
#include <FORRAction.h>
#include <vector>
#include <string>
#include <math.h>
#include <iostream>
#include <fstream>
#include <cmath>        //for atan2 and M_PI
#include <algorithm>
#include <map>
#include <set>
#include <numeric>
// #include <queue>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std;

// struct FrontierGridNode{
// 	int x;
// 	int y;
// 	vector<FrontierGridNode> nodeSequence;
// 	int nodeCost;
// 	int distanceTarget;
// 	FrontierGridNode(): x(0), y(0), nodeCost(0), distanceTarget(0) { }
// 	FrontierGridNode(int xval, int yval, int cost, int distval){
// 		x = xval;
// 		y = yval;
// 		nodeCost = cost;
// 		distanceTarget = distval;
// 	}
// 	void addToNodeSequence(FrontierGridNode n){
// 		nodeSequence.push_back(n);
// 	}
// 	void setNodeSequence(vector<FrontierGridNode> ns){
// 		nodeSequence = ns;
// 	}
// 	bool operator==(const FrontierGridNode n) {
// 		if(x == n.x and y == n.y){
// 			return true;
// 		}
// 		else{
// 			return false;
// 		}
// 	}
// 	bool operator < (const FrontierGridNode n) const{
// 		// if((nodeCost + distanceTarget) < (n.nodeCost + n.distanceTarget)){
// 		if((distanceTarget) > (n.distanceTarget)){
// 			return true;
// 		}
// 		else{
// 			return false;
// 		}
// 	}
// 	bool operator > (const FrontierGridNode n) const{
// 		// if((nodeCost + distanceTarget) > (n.nodeCost + n.distanceTarget)){
// 		if((distanceTarget) < (n.distanceTarget)){
// 			return true;
// 		}
// 		else{
// 			return false;
// 		}
// 	}
// };

class FrontierExplorer{
public:
	FrontierExplorer(int l, int h, double tthreshold, double decthreshold, double arrMove[], double arrRotate[], int moveArrMax, int rotateArrMax){
		time_threshold = tthreshold;
		length = l;
		height = h;
		numMoves = moveArrMax;
		numRotates = rotateArrMax;
		for(int i = 0 ; i < numMoves ; i++) move[i] = arrMove[i];
		for(int i = 0 ; i < numRotates ; i++) rotate[i] = arrRotate[i];
		// cout << "Frontier length " << length << " height " << height << endl;
		// -1 for unmarked, 1 for occupied, 0 for unoccupied
		for(int i = 0; i < l; i++){
			vector<int> col;
			for(int j = 0; j < h; j ++){
				col.push_back(-1);
			}
			frontier_grid.push_back(col);
		}
		for(int i = 0; i < l; i++){
			vector<int> col;
			for(int j = 0; j < h; j ++){
				col.push_back(-1);
			}
			traveled_grid.push_back(col);
		}
		for(int i = 0; i < l; i++){
			vector<int> col;
			for(int j = 0; j < h; j ++){
				col.push_back(-1);
			}
			stack_grid.push_back(col);
		}
		for(int i = 0; i < l; i++){
			vector<int> col;
			for(int j = 0; j < h; j ++){
				col.push_back(0);
			}
			passed_grid.push_back(col);
		}
		for(int i = 0; i < l; i++){
			vector<int> col;
			for(int j = 0; j < h; j ++){
				col.push_back(0);
			}
			hit_grid.push_back(col);
		}
		frontiers_complete = false;
		go_to_top_point = false;
		top_point_decisions = 0;
		decision_limit = decthreshold;
		start_rotations = 0;
	};
	~FrontierExplorer(){};

	bool getFrontiersComplete(){return frontiers_complete;}
	void setFrontiersComplete(double time){
		if(time >= time_threshold){
			frontiers_complete = true;
			if(time <= time_threshold+10){
				cout << "Frontier grid" << endl;
				for(int i = 0; i < frontier_grid[0].size(); i++){
					cout << "final_grid ";
					for(int j = 0; j < frontier_grid.size(); j++){
						cout << frontier_grid[j][i] << " ";
					}
					cout << endl;
				}
			}
		}
	}

	vector< vector<int> > getFrontierGrid(){return frontier_grid;}

	vector< vector<double> > getFrontierPath(){
		if(go_to_top_point){
			return path_to_top_point;
		}
		else if(path_to_current_target.size() > 0){
			return path_to_current_target;
		}
		vector< vector<double> > path;
		return path;
	}

	vector<Position> getFrontierStack(){
		vector<Position> hwst;
		for(int i = 0; i < frontier_stack.size(); i++){
			hwst.push_back(frontier_stack[i]);
		}
		return hwst;
	}

	vector< vector<Position> > getRemainingFrontierStack(){
		cout << "getRemainingFrontierStack " << frontier_stack.size() << endl;
		vector< vector<Position> > hwst;
		for(int i = 0; i < frontier_stack.size(); i++){
			vector<Position> pair_points;
			pair_points.push_back(frontier_stack_view[i]);
			pair_points.push_back(frontier_stack[i]);
			hwst.push_back(pair_points);
		}
		cout << "hwst " << hwst.size() << endl;
		return hwst;
	}

	Position getFrontierTarget(){
		if(go_to_top_point == true){
			return top_point;
		}
		else{
			return current_target;
		}
	}

	int getLength(){return length;}
	int getHeight(){return height;}

	FORRAction exploreDecision(Position current_point, sensor_msgs::msg::LaserScan current_laser){
		Position current_position = current_point;
		cout << "current_position " << current_position.getX() << " " << current_position.getY() << endl;
		position_history.push_back(current_position);
		traveled_grid[(int)(current_position.getX())][(int)(current_position.getY())] = 1;
		cout << "after traveled_grid" << endl;
		double start_angle = current_laser.angle_min;
		double increment = current_laser.angle_increment;
		double r_x = current_position.getX();
		double r_y = current_position.getY();
		double r_ang = current_position.getTheta();
		double middle_distance = 0;
		double middle_distance_min = 50;
		vector<CartesianPoint> laserEndpoints;
		cout << "before laserEndpoints" << endl;
		for(int i = 0; i < current_laser.ranges.size(); i++){
			double angle = start_angle + r_ang;
			laserEndpoints.push_back(CartesianPoint(r_x + current_laser.ranges[i]*cos(angle), r_y + current_laser.ranges[i]*sin(angle)));
			if(i >= 195 and i <= 465){
				middle_distance += current_laser.ranges[i];
				if(current_laser.ranges[i] < middle_distance_min){
					middle_distance_min = current_laser.ranges[i];
				}
			}
			start_angle = start_angle + increment;
		}
		cout << "after laserEndpoints " << laserEndpoints.size() << endl;
		middle_distance = middle_distance / 271.0;
		cout << "middle_distance " << middle_distance << " middle_distance_min " << middle_distance_min << endl;
		laserEndpoints_history.push_back(laserEndpoints);
		int startx = (int)(current_position.getX());
		int starty = (int)(current_position.getY());
		cout << "startx " << startx << " starty " << starty << endl;
		for(int j = 0; j < laserEndpoints.size(); j++){
			int ex = (int)(laserEndpoints[j].get_x());
			int ey = (int)(laserEndpoints[j].get_y());
			int ea = starty - ey;
			int eb = ex - startx;
			int ec = (startx - ex) * starty + (ey - starty) * startx;
			// cout << "ex " << ex << " ey " << ey << " ea " << ea << " eb " << eb << " ec " << ec << endl;
			if(ex >= 0 and ey >= 0 and ex < passed_grid.size() and ey < passed_grid[0].size()){
				if(startx > ex){
					for(int i = startx; i > ex; i--){
						if(CartesianPoint(current_position.getX(), current_position.getY()).get_distance(CartesianPoint(i, (-(ea * i + ec) / eb))) <= 20){
							passed_grid[i][(int)(-(ea * i + ec) / eb)] = passed_grid[i][(int)(-(ea * i + ec) / eb)] + 1;
						}
					}
				}
				else if(startx < ex){
					for(int i = startx; i < ex; i++){
						if(CartesianPoint(current_position.getX(), current_position.getY()).get_distance(CartesianPoint(i, (-(ea * i + ec) / eb))) <= 20){
							passed_grid[i][(int)(-(ea * i + ec) / eb)] = passed_grid[i][(int)(-(ea * i + ec) / eb)] + 1;
						}
					}
				}
				else{
					if(starty > ey){
						for(int i = starty; i > ey; i--){
							if(CartesianPoint(current_position.getX(), current_position.getY()).get_distance(CartesianPoint((-(eb * i + ec) / ea), i)) <= 20){
								passed_grid[(int)(-(eb * i + ec) / ea)][i] = passed_grid[(int)(-(eb * i + ec) / ea)][i] + 1;
							}
						}
					}
					else if(starty < ey){
						for(int i = starty; i < ey; i++){
							if(CartesianPoint(current_position.getX(), current_position.getY()).get_distance(CartesianPoint((-(eb * i + ec) / ea), i)) <= 20){
								passed_grid[(int)(-(eb * i + ec) / ea)][i] = passed_grid[(int)(-(eb * i + ec) / ea)][i] + 1;
							}
						}
					}
					else{
						passed_grid[startx][starty] = passed_grid[startx][starty] + 1;
					}
				}
				if(laserEndpoints[j].get_distance(CartesianPoint(current_position.getX(), current_position.getY())) <= 20){
					hit_grid[ex][ey] = hit_grid[ex][ey] + 1;
				}
			}
		}
		cout << "before ratio_grid" << endl;
		vector< vector<double> > ratio_grid;
		for(int i = 0; i < length; i++){
			vector<double> col;
			for(int j = 0; j < height; j ++){
				col.push_back(0.0);
			}
			ratio_grid.push_back(col);
		}
		for(int i = 0; i < passed_grid.size(); i++){
			for(int j = 0; j < passed_grid[i].size(); j++){
				if(passed_grid[i][j] > 0 or hit_grid[i][j] > 0){
					ratio_grid[i][j] = (double)(hit_grid[i][j]) / ((double)(hit_grid[i][j]) + (double)(passed_grid[i][j]));
				}
				else{
					ratio_grid[i][j] = -1.0;
				}
				// cout << i << " " << j << " passed_grid " << passed_grid[i][j] << " hit_grid " << hit_grid[i][j] << " ratio_grid " << ratio_grid[i][j] << endl;
			}
		}
		cout << "after ratio_grid" << endl;
		for(int i = 0; i < ratio_grid.size(); i++){
			for(int j = 0; j < ratio_grid[i].size(); j++){
				if(ratio_grid[i][j] >= 0.75 and frontier_grid[i][j] == -1){
					frontier_grid[i][j] = 1;
				}
				else if(ratio_grid[i][j] >= 0.0 and ratio_grid[i][j] <= 0.25 and frontier_grid[i][j] == -1){
					frontier_grid[i][j] = 0;
				}
				else if(ratio_grid[i][j] >= 0.95 and frontier_grid[i][j] == 0){
					frontier_grid[i][j] = 1;
				}
				else if(ratio_grid[i][j] >= 0 and ratio_grid[i][j] <= 0.05 and frontier_grid[i][j] == 1){
					frontier_grid[i][j] = 0;
				}
			}
		}
		cout << "updated frontier_grid" << endl;
		for(int i = 0; i < frontier_grid.size(); i++){
			for(int j = 0; j < frontier_grid[i].size(); j++){
				if(stack_grid[i][j] == -1 and frontier_grid[i][j] == 0 and traveled_grid[i][j] == -1){
					bool any_open = false;
					if(i-1 >= 0){
						if(frontier_grid[i-1][j] == -1){
							any_open = true;
						}
					}
					if(i+1 < frontier_grid.size()){
						if(frontier_grid[i+1][j] == -1){
							any_open = true;
						}
					}
					if(j-1 >= 0){
						if(frontier_grid[i][j-1] == -1){
							any_open = true;
						}
					}
					if(j+1 < frontier_grid[i].size()){
						if(frontier_grid[i][j+1] == -1){
							any_open = true;
						}
					}
					if(any_open == true){
						frontier_stack.push_back(Position(i,j,0));
						frontier_stack_view.push_back(current_position);
						stack_grid[i][j] = 1;
					}
				}
			}
		}
		cout << "updated stack_grid " << frontier_stack.size() << endl;
		
		// If the beginning then spin 360 degrees to add to stack
		if(start_rotations < 72){
			cout << "start_rotations " << start_rotations << endl;
			start_rotations = start_rotations + 1;
			return FORRAction(RIGHT_TURN, 1);
		}
		else if(frontier_stack.size() > 0 and start_rotations == 72){
			start_rotations = start_rotations + 1;
			top_point = frontier_stack_view[0];
			current_target = frontier_stack[0];
			cout << "Top point " << top_point.getX() << " " << top_point.getY() << endl;
			frontier_stack.erase(frontier_stack.begin());
			frontier_stack_view.erase(frontier_stack_view.begin());
			top_point_decisions = 0;
			return goTowardsPoint(current_position, current_target, middle_distance_min);
		}
		else if(frontier_stack.size() == 0){
			frontiers_complete = true;
			return FORRAction(FORWARD, 0);
		}
		// Once there are items on the stack, pop the top and start to go towards
		cout << "current_target " << current_target.getX() << " " << current_target.getY() << endl;
		cout << "top_point_decisions " << top_point_decisions << endl;
		cout << "frontier_stack.size() " << frontier_stack.size() << endl;
		cout << "position_history " << position_history.size() << endl;

		double dist_to_current_target = current_target.getDistance(current_position);

		if(((dist_to_current_target <= 0.5 or middle_distance <= 0.5 or middle_distance_min <= 0.3) and go_to_top_point == false) or top_point_decisions == decision_limit){
			cout << "Decision limit reached " << top_point_decisions << endl;
			cout << "Reached current target " << dist_to_current_target << endl;
			cout << "Too close in front " << middle_distance << endl;
			// Stop current point and go to next on stack
			cout << "Frontier grid" << endl;
			for(int i = 0; i < frontier_grid[0].size(); i++){
				for(int j = 0; j < frontier_grid.size(); j++){
					if(i == (int)(current_position.getY()) and j == (int)(current_position.getX())){
						cout << "[" << frontier_grid[j][i] << "] "; 
					}
					else{
						cout << frontier_grid[j][i] << " ";
					}
				}
				cout << endl;
			}
			cout << "stack_grid" << endl;
			for(int i = 0; i < stack_grid[0].size(); i++){
				for(int j = 0; j < stack_grid.size(); j++){
					if(i == (int)(current_position.getY()) and j == (int)(current_position.getX())){
						cout << "[" << stack_grid[j][i] << "] "; 
					}
					else{
						cout << stack_grid[j][i] << " ";
					}
				}
				cout << endl;
			}
			cout << "traveled_grid" << endl;
			for(int i = 0; i < traveled_grid[0].size(); i++){
				for(int j = 0; j < traveled_grid.size(); j++){
					if(i == (int)(current_position.getY()) and j == (int)(current_position.getX())){
						cout << "[" << traveled_grid[j][i] << "] "; 
					}
					else{
						cout << traveled_grid[j][i] << " ";
					}
				}
				cout << endl;
			}
			// After finishing point on stack, pop next one
			if(frontier_stack.size() > 0){
				cout << "Going to top point on stack" << endl;
				bool visited = true;
				while(visited and frontier_stack.size() > 0){
					top_point = frontier_stack_view[0];
					current_target = frontier_stack[0];
					frontier_stack.erase(frontier_stack.begin());
					frontier_stack_view.erase(frontier_stack_view.begin());
					if(traveled_grid[(int)(current_target.getX())][(int)(current_target.getY())] == -1 and frontier_grid[(int)(current_target.getX())][(int)(current_target.getY())] == 0){
						bool any_open = false;
						if((int)(current_target.getX())-1 >= 0){
							if(frontier_grid[(int)(current_target.getX())-1][(int)(current_target.getY())] == -1){
								any_open = true;
							}
						}
						if((int)(current_target.getX())+1 < frontier_grid.size()){
							if(frontier_grid[(int)(current_target.getX())+1][(int)(current_target.getY())] == -1){
								any_open = true;
							}
						}
						if((int)(current_target.getY())-1 >= 0){
							if(frontier_grid[(int)(current_target.getX())][(int)(current_target.getY())-1] == -1){
								any_open = true;
							}
						}
						if((int)(current_target.getY())+1 < frontier_grid[(int)(current_target.getX())].size()){
							if(frontier_grid[(int)(current_target.getX())][(int)(current_target.getY())+1] == -1){
								any_open = true;
							}
						}
						if(any_open == true){
							visited = false;
						}
					}
				}
				top_point_decisions = 0;
				go_to_top_point = true;
				path_to_top_point.clear();
				path_to_current_target.clear();
				start_rotations = 0;
				return FORRAction(RIGHT_TURN, 1);
			}
			else{
				cout << "No more in stack" << endl;
				go_to_top_point = false;
				frontiers_complete = true;
				return FORRAction(FORWARD, 0);
			}
		}

		if(go_to_top_point == true){
			// cout << "Go to top point" << endl;
			cout << "Top point " << top_point.getX() << " " << top_point.getY() << endl;
			double dist_to_top_point = top_point.getDistance(current_position);
			bool can_access_top_point = canAccessPoint(laserEndpoints, CartesianPoint(current_position.getX(), current_position.getY()), CartesianPoint(top_point.getX(), top_point.getY()), 5);
			cout << "Distance to top point " << dist_to_top_point << " current theta " << current_position.getTheta() << endl;

			if(dist_to_top_point <= 0.5){
				cout << "Top point achieved, go towards current_target" << endl;
				go_to_top_point = false;
				top_point_decisions = 0;
				return goTowardsPoint(current_position, current_target, middle_distance_min);
			}
			else if(dist_to_top_point <= 1 and can_access_top_point){
				cout << "Top point close, go towards" << endl;
				top_point_decisions++;
				return goTowardsPoint(current_position, top_point, middle_distance_min);
			}
			else if(path_to_top_point.size() > 0){
				cout << "Top point not in range, go to top point by following path" << endl;
				waypointAchieved(current_position);
				if(path_to_top_point.size() == 0){
					top_point_decisions++;
					return FORRAction(FORWARD, 0);
				}
				else{
					bool can_access_waypoint = canAccessPoint(laserEndpoints, CartesianPoint(current_position.getX(), current_position.getY()), CartesianPoint(path_to_top_point[0][0], path_to_top_point[0][1]), 10);
					if(can_access_waypoint){
						cout << "Can Access Current waypoint " << path_to_top_point[0][0] << " " << path_to_top_point[0][1] << endl;
						top_point_decisions++;
						return goTowardsPoint(current_position, Position(path_to_top_point[0][0], path_to_top_point[0][1], 0), middle_distance_min);
					}
					else{
						int num = -1;
						while(!can_access_waypoint and num < path_to_top_point.size()){
							num = num + 1;
							can_access_waypoint = canAccessPoint(laserEndpoints, CartesianPoint(current_position.getX(), current_position.getY()), CartesianPoint(path_to_top_point[num][0], path_to_top_point[num][1]), 10);
						}
						cout << "num " << num << " can_access_waypoint " << can_access_waypoint << endl;
						if(can_access_waypoint){
							cout << "New Access waypoint " << path_to_top_point[num][0] << " " << path_to_top_point[num][1] << endl;
							top_point_decisions++;
							return goTowardsPoint(current_position, Position(path_to_top_point[num][0], path_to_top_point[num][1], 0), middle_distance_min);
						}
						else{
							cout << "Try Current waypoint " << path_to_top_point[0][0] << " " << path_to_top_point[0][1] << endl;
							top_point_decisions++;
							return goTowardsPoint(current_position, Position(path_to_top_point[0][0], path_to_top_point[0][1], 0), middle_distance_min);
						}
					}
				}
			}
			else{
				cout << "find path to top point" << endl;
				findPathFromHistory(current_position, top_point);
				// cout << "Current waypoint " << path_to_top_point[0][0] << " " << path_to_top_point[0][1] << endl;
				if(path_to_top_point.size() > 0){
					waypointAchieved(current_position);
					if(path_to_top_point.size() == 0){
						top_point_decisions++;
						return FORRAction(FORWARD, 0);
					}
					else{
						top_point_decisions++;
						return goTowardsPoint(current_position, Position(path_to_top_point[0][0], path_to_top_point[0][1], 0), middle_distance_min);
					}
				}
			}
		}
		else{
			cout << "Going to current_target " << current_target.getX() << " " << current_target.getY() << endl;
			// cout << "Current grid value " << frontier_grid[(int)(current_position.getX())][(int)(current_position.getY())] << endl;
			bool can_access_current_target = canAccessPoint(laserEndpoints, CartesianPoint(current_position.getX(), current_position.getY()), CartesianPoint(current_target.getX(), current_target.getY()), 5);
			if(can_access_current_target){
				cout << "Can access current target" << endl;
				top_point_decisions++;
				return goTowardsPoint(current_position, current_target, middle_distance_min);
			}
			else if(path_to_current_target.size() > 0){
				cout << "current target not in range, go to current target by following path" << endl;
				waypointAchieved(current_position);
				if(path_to_current_target.size() == 0){
					top_point_decisions++;
					return FORRAction(FORWARD, 0);
				}
				else{
					bool can_access_waypoint = canAccessPoint(laserEndpoints, CartesianPoint(current_position.getX(), current_position.getY()), CartesianPoint(path_to_current_target[0][0], path_to_current_target[0][1]), 10);
					if(can_access_waypoint){
						cout << "Can Access Current waypoint " << path_to_current_target[0][0] << " " << path_to_current_target[0][1] << endl;
						top_point_decisions++;
						return goTowardsPoint(current_position, Position(path_to_current_target[0][0], path_to_current_target[0][1], 0), middle_distance_min);
					}
					else{
						int num = -1;
						while(!can_access_waypoint and num < path_to_current_target.size()){
							num = num + 1;
							can_access_waypoint = canAccessPoint(laserEndpoints, CartesianPoint(current_position.getX(), current_position.getY()), CartesianPoint(path_to_current_target[num][0], path_to_current_target[num][1]), 5);
						}
						cout << "num " << num << " can_access_waypoint " << can_access_waypoint << endl;
						if(can_access_waypoint){
							cout << "New Access waypoint " << path_to_current_target[num][0] << " " << path_to_current_target[num][1] << endl;
							top_point_decisions++;
							return goTowardsPoint(current_position, Position(path_to_current_target[num][0], path_to_current_target[num][1], 0), middle_distance_min);
						}
						else{
							cout << "Try Current waypoint " << path_to_current_target[0][0] << " " << path_to_current_target[0][1] << endl;
							top_point_decisions++;
							return goTowardsPoint(current_position, Position(path_to_current_target[0][0], path_to_current_target[0][1], 0), middle_distance_min);
						}
					}
				}
			}
			else{
				cout << "Follow open grid to try to get to target" << endl;
				findPathOnGrid(current_position, current_target);
				// cout << "Current waypoint " << path_to_current_target[0][0] << " " << path_to_current_target[0][1] << endl;
				if(path_to_current_target.size() > 0){
					waypointAchieved(current_position);
					if(path_to_current_target.size() == 0){
						top_point_decisions++;
						return FORRAction(FORWARD, 0);
					}
					else{
						top_point_decisions++;
						return goTowardsPoint(current_position, Position(path_to_current_target[0][0], path_to_current_target[0][1], 0), middle_distance_min);
					}
				}
			}
		}
	}

	void findPathFromHistory(Position current_point, Position target_point){
		path_to_top_point.clear();
		cout << "current_point " << current_point.getX() << " " << current_point.getY() << " target_point " << target_point.getX() << " " << target_point.getY() << endl;
		int target_point_index = -1;
		int closest_index = -1;
		double dist_to_top = 50;
		double min_dist_to_top = 50;
		for(int i = 0; i < position_history.size(); i++){
			dist_to_top = position_history[i].getDistance(target_point);
			if(position_history[i] == target_point){
				target_point_index = i;
				break;
			}
			else if(dist_to_top < 0.25){
				target_point_index = i;
				break;
			}
			if(dist_to_top < min_dist_to_top){
				min_dist_to_top = dist_to_top;
				closest_index = i;
			}
		}
		cout << "target_point_index " << target_point_index << endl;
		if(target_point_index == -1){
			target_point_index = closest_index;
		}
		vector<Position> trailPositions;
		trailPositions.push_back(position_history[target_point_index]);
		for(int i = target_point_index; i < position_history.size(); i++){
			for(int n = position_history.size()-1; n > i; n--){
				if(canAccessPoint(laserEndpoints_history[i], CartesianPoint(position_history[i].getX(), position_history[i].getY()), CartesianPoint(position_history[n].getX(), position_history[n].getY()), 2)) {
					trailPositions.push_back(position_history[n]);
					i = n-1;
				}
			}
		}
		trailPositions.push_back(position_history[position_history.size()-1]);
		cout << "path_to_top_point " << trailPositions.size() << endl;
		for(int i = trailPositions.size()-1; i >= 0; i--){
			vector<double> marker;
			marker.push_back(trailPositions[i].getX());
			marker.push_back(trailPositions[i].getY());
			cout << marker[0] << " " << marker[1] << endl;
			path_to_top_point.push_back(marker);
		}
	}

	void findPathOnGrid(Position current_point, Position target_point){
		path_to_current_target.clear();
		cout << "current_point " << current_point.getX() << " " << current_point.getY() << " target_point " << target_point.getX() << " " << target_point.getY() << endl;
		int current_x = (int)(current_point.getX());
		int current_y = (int)(current_point.getY());
		int target_x = (int)(target_point.getX());
		int target_y = (int)(target_point.getY());
		if(current_x == target_x and current_y == target_y){
			vector<double> current;
			current.push_back(target_point.getX());
			current.push_back(target_point.getY());
			path_to_current_target.push_back(current);
			return;
		}
		int closest_x = current_x;
		int closest_y = current_y;
		int min_dist = 100;
		for(int i = current_x - 2; i <= current_x + 2; i++){
			for(int j = current_y - 2; j <= current_y + 2; j++){
				if(i >= 0 and i < frontier_grid.size() and j >= 0 and j < frontier_grid[0].size() and i != current_x and j != current_y){
					int dist_target = abs(target_x - i) + abs(target_y - j);
					if(frontier_grid[i][j] == 0 and dist_target < min_dist){
						closest_x = i;
						closest_y = j;
						min_dist = dist_target;
					}
				}
			}
		}
		vector<double> current;
		current.push_back(closest_x);
		current.push_back(closest_y);
		path_to_current_target.push_back(current);

		// priority_queue<FrontierGridNode> rn_queue;
		// // vector<FrontierGridNode> already_searched;
		// // vector<FrontierGridNode> already_on_queue;
		// vector< vector<int> > searched_grid;
		// for(int i = 0; i < length; i++){
		// 	vector<int> col;
		// 	for(int j = 0; j < height; j ++){
		// 		col.push_back(0);
		// 	}
		// 	searched_grid.push_back(col);
		// }
		// FrontierGridNode start_rn = FrontierGridNode(current_x, current_y, 0, (abs(current_x - target_x) + abs(current_y - target_y)));
		// if(current_x-1 >= 0){
		// 	if(frontier_grid[current_x-1][current_y] == 0){
		// 		FrontierGridNode neighbor = FrontierGridNode(current_x-1, current_y, start_rn.nodeCost+1, (abs(current_x-1 - target_x) + abs(current_y - target_y)));
		// 		neighbor.addToNodeSequence(start_rn);
		// 		rn_queue.push(neighbor);
		// 		// already_on_queue.push_back(neighbor);
		// 	}
		// }
		// if(current_x+1 < frontier_grid.size()){
		// 	if(frontier_grid[current_x+1][current_y] == 0){
		// 		FrontierGridNode neighbor = FrontierGridNode(current_x+1, current_y, start_rn.nodeCost+1, (abs(current_x+1 - target_x) + abs(current_y - target_y)));
		// 		neighbor.addToNodeSequence(start_rn);
		// 		rn_queue.push(neighbor);
		// 		// already_on_queue.push_back(neighbor);
		// 	}
		// }
		// if(current_y-1 >= 0){
		// 	if(frontier_grid[current_x][current_y-1] == 0){
		// 		FrontierGridNode neighbor = FrontierGridNode(current_x, current_y-1, start_rn.nodeCost+1, (abs(current_x - target_x) + abs(current_y-1 - target_y)));
		// 		neighbor.addToNodeSequence(start_rn);
		// 		rn_queue.push(neighbor);
		// 		// already_on_queue.push_back(neighbor);
		// 	}
		// }
		// if(current_y+1 < frontier_grid[0].size()){
		// 	if(frontier_grid[current_x][current_y+1] == 0){
		// 		FrontierGridNode neighbor = FrontierGridNode(current_x, current_y+1, start_rn.nodeCost+1, (abs(current_x - target_x) + abs(current_y+1 - target_y)));
		// 		neighbor.addToNodeSequence(start_rn);
		// 		rn_queue.push(neighbor);
		// 		// already_on_queue.push_back(neighbor);
		// 	}
		// }
		// // already_searched.push_back(start_rn);
		// searched_grid[start_rn.x][start_rn.y] = 1;
		// cout << "rn_queue " << rn_queue.size() << endl;
		// FrontierGridNode target_rn = FrontierGridNode(target_x, target_y, 0, 0);
		// FrontierGridNode final_rn = start_rn;
		// int count = 0;
		// while(rn_queue.size() > 0 and count < 5000){
		// 	FrontierGridNode current_neighbor = rn_queue.top();
		// 	cout << "current_neighbor " << current_neighbor.x << " " << current_neighbor.y << " cost " << current_neighbor.nodeCost << " distanceTarget " << current_neighbor.distanceTarget << " " << current_neighbor.nodeSequence.size() << endl;
		// 	searched_grid[current_neighbor.x][current_neighbor.y] = 1;
		// 	// already_searched.push_back(current_neighbor);
		// 	rn_queue.pop();
		// 	cout << "rn_queue " << rn_queue.size() << endl;
		// 	if(current_neighbor == target_rn){
		// 		final_rn = current_neighbor;
		// 		cout << "final_rn " << final_rn.x << " " << final_rn.y << " " << final_rn.nodeSequence.size() << endl;
		// 		break;
		// 	}
		// 	if(current_neighbor.x-1 >= 0){
		// 		if(frontier_grid[current_neighbor.x-1][current_neighbor.y] == 0){
		// 			FrontierGridNode neighbor = FrontierGridNode(current_neighbor.x-1, current_neighbor.y, current_neighbor.nodeCost+1, (abs(current_neighbor.x-1 - target_x) + abs(current_neighbor.y - target_y)));
		// 			// bool foundAlreadySearched = false;
		// 			// for(int a = 0; a < already_searched.size(); a++){
		// 			// 	// cout << "neighbor " << neighbor.x << " " << neighbor.y << " already_searched " << already_searched[a].x << " " << already_searched[a].y << " " << (already_searched[a] == neighbor) << endl;
		// 			// 	if(already_searched[a] == neighbor){
		// 			// 		foundAlreadySearched = true;
		// 			// 		break;
		// 			// 	}
		// 			// }
		// 			// bool foundAlreadyOnQueue = false;
		// 			// if(foundAlreadySearched == false){
		// 			// 	for(int a = 0; a < already_on_queue.size(); a++){
		// 			// 		if(already_on_queue[a] == neighbor and already_on_queue[a].nodeCost == neighbor.nodeCost){
		// 			// 			foundAlreadyOnQueue = true;
		// 			// 			break;
		// 			// 		}
		// 			// 	}
		// 			// }
		// 			// cout << "foundAlreadySearched " << foundAlreadySearched << " foundAlreadyOnQueue " << foundAlreadyOnQueue << endl;
		// 			if(searched_grid[neighbor.x][neighbor.y] == 0){
		// 				neighbor.setNodeSequence(current_neighbor.nodeSequence);
		// 				neighbor.addToNodeSequence(current_neighbor);
		// 				cout << "neighbor added " << neighbor.x << " " << neighbor.y << " " << neighbor.nodeSequence.size() << endl;
		// 				rn_queue.push(neighbor);
		// 				// already_on_queue.push_back(neighbor);
		// 			}
		// 		}
		// 	}
		// 	if(current_neighbor.x+1 < frontier_grid.size()){
		// 		if(frontier_grid[current_neighbor.x+1][current_neighbor.y] == 0){
		// 			FrontierGridNode neighbor = FrontierGridNode(current_neighbor.x+1, current_neighbor.y, current_neighbor.nodeCost+1, (abs(current_neighbor.x+1 - target_x) + abs(current_neighbor.y - target_y)));
		// 			// bool foundAlreadySearched = false;
		// 			// for(int a = 0; a < already_searched.size(); a++){
		// 			// 	// cout << "neighbor " << neighbor.x << " " << neighbor.y << " already_searched " << already_searched[a].x << " " << already_searched[a].y << " " << (already_searched[a] == neighbor) << endl;
		// 			// 	if(already_searched[a] == neighbor){
		// 			// 		foundAlreadySearched = true;
		// 			// 		break;
		// 			// 	}
		// 			// }
		// 			// bool foundAlreadyOnQueue = false;
		// 			// if(foundAlreadySearched == false){
		// 			// 	for(int a = 0; a < already_on_queue.size(); a++){
		// 			// 		if(already_on_queue[a] == neighbor and already_on_queue[a].nodeCost == neighbor.nodeCost){
		// 			// 			foundAlreadyOnQueue = true;
		// 			// 			break;
		// 			// 		}
		// 			// 	}
		// 			// }
		// 			// cout << "foundAlreadySearched " << foundAlreadySearched << " foundAlreadyOnQueue " << foundAlreadyOnQueue << endl;
		// 			if(searched_grid[neighbor.x][neighbor.y] == 0){
		// 				neighbor.setNodeSequence(current_neighbor.nodeSequence);
		// 				neighbor.addToNodeSequence(current_neighbor);
		// 				cout << "neighbor added " << neighbor.x << " " << neighbor.y << " " << neighbor.nodeSequence.size() << endl;
		// 				rn_queue.push(neighbor);
		// 				// already_on_queue.push_back(neighbor);
		// 			}
		// 		}
		// 	}
		// 	if(current_neighbor.y-1 >= 0){
		// 		if(frontier_grid[current_neighbor.x][current_neighbor.y-1] == 0){
		// 			FrontierGridNode neighbor = FrontierGridNode(current_neighbor.x, current_neighbor.y-1, current_neighbor.nodeCost+1, (abs(current_neighbor.x - target_x) + abs(current_neighbor.y-1 - target_y)));
		// 			// bool foundAlreadySearched = false;
		// 			// for(int a = 0; a < already_searched.size(); a++){
		// 			// 	// cout << "neighbor " << neighbor.x << " " << neighbor.y << " already_searched " << already_searched[a].x << " " << already_searched[a].y << " " << (already_searched[a] == neighbor) << endl;
		// 			// 	if(already_searched[a] == neighbor){
		// 			// 		foundAlreadySearched = true;
		// 			// 		break;
		// 			// 	}
		// 			// }
		// 			// bool foundAlreadyOnQueue = false;
		// 			// if(foundAlreadySearched == false){
		// 			// 	for(int a = 0; a < already_on_queue.size(); a++){
		// 			// 		if(already_on_queue[a] == neighbor and already_on_queue[a].nodeCost == neighbor.nodeCost){
		// 			// 			foundAlreadyOnQueue = true;
		// 			// 			break;
		// 			// 		}
		// 			// 	}
		// 			// }
		// 			// cout << "foundAlreadySearched " << foundAlreadySearched << " foundAlreadyOnQueue " << foundAlreadyOnQueue << endl;
		// 			if(searched_grid[neighbor.x][neighbor.y] == 0){
		// 				neighbor.setNodeSequence(current_neighbor.nodeSequence);
		// 				neighbor.addToNodeSequence(current_neighbor);
		// 				cout << "neighbor added " << neighbor.x << " " << neighbor.y << " " << neighbor.nodeSequence.size() << endl;
		// 				rn_queue.push(neighbor);
		// 				// already_on_queue.push_back(neighbor);
		// 			}
		// 		}
		// 	}
		// 	if(current_neighbor.y+1 < frontier_grid[0].size()){
		// 		if(frontier_grid[current_neighbor.x][current_neighbor.y+1] == 0){
		// 			FrontierGridNode neighbor = FrontierGridNode(current_neighbor.x, current_neighbor.y+1, current_neighbor.nodeCost+1, (abs(current_neighbor.x - target_x) + abs(current_neighbor.y+1 - target_y)));
		// 			// bool foundAlreadySearched = false;
		// 			// for(int a = 0; a < already_searched.size(); a++){
		// 			// 	// cout << "neighbor " << neighbor.x << " " << neighbor.y << " already_searched " << already_searched[a].x << " " << already_searched[a].y << " " << (already_searched[a] == neighbor) << endl;
		// 			// 	if(already_searched[a] == neighbor){
		// 			// 		foundAlreadySearched = true;
		// 			// 		break;
		// 			// 	}
		// 			// }
		// 			// bool foundAlreadyOnQueue = false;
		// 			// if(foundAlreadySearched == false){
		// 			// 	for(int a = 0; a < already_on_queue.size(); a++){
		// 			// 		if(already_on_queue[a] == neighbor and already_on_queue[a].nodeCost == neighbor.nodeCost){
		// 			// 			foundAlreadyOnQueue = true;
		// 			// 			break;
		// 			// 		}
		// 			// 	}
		// 			// }
		// 			// cout << "foundAlreadySearched " << foundAlreadySearched << " foundAlreadyOnQueue " << foundAlreadyOnQueue << endl;
		// 			if(searched_grid[neighbor.x][neighbor.y] == 0){
		// 				neighbor.setNodeSequence(current_neighbor.nodeSequence);
		// 				neighbor.addToNodeSequence(current_neighbor);
		// 				cout << "neighbor added " << neighbor.x << " " << neighbor.y << " " << neighbor.nodeSequence.size() << endl;
		// 				rn_queue.push(neighbor);
		// 				// already_on_queue.push_back(neighbor);
		// 			}
		// 		}
		// 	}
		// 	count = count + 1;
		// 	cout << "rn_queue " << rn_queue.size() << " count " << count << endl;
		// }
		// if(final_rn == start_rn){
		// 	cout << "final_rn is still start_rn" << endl;
		// 	vector<double> current;
		// 	current.push_back(target_point.getX());
		// 	current.push_back(target_point.getY());
		// 	path_to_current_target.push_back(current);
		// 	return;
		// }
		// else{
		// 	cout << "found target_rn, traverse links to build path" << endl;
		// 	cout << "final_rn " << final_rn.x << " " << final_rn.y << " " << final_rn.nodeSequence.size() << endl;
		// 	for(int i = 0; i < final_rn.nodeSequence.size(); i++){
		// 		vector<double> current;
		// 		current.push_back(final_rn.nodeSequence[i].x);
		// 		current.push_back(final_rn.nodeSequence[i].y);
		// 		path_to_current_target.push_back(current);
		// 	}
		// 	cout << "final path_to_current_target " << endl;
		// 	for(int i = 0; i < path_to_current_target.size(); i++){
		// 		cout << path_to_current_target[i][0] << " " << path_to_current_target[i][1] << endl;
		// 	}
		// 	return;
		// }
	}

	void waypointAchieved(Position current_position){
		cout << "In waypointAchieved" << endl;
		bool erase_waypoint = true;
		if(go_to_top_point){
			while(erase_waypoint and path_to_top_point.size() > 0){
				Position current_waypoint = Position(path_to_top_point[0][0], path_to_top_point[0][1], 0);
				double dist_to_current_waypoint = current_position.getDistance(current_waypoint);
				if(dist_to_current_waypoint <= 0.75){
					cout << "Waypoint Achieved, removing waypoint" << endl;
					path_to_top_point.erase(path_to_top_point.begin());
				}
				else{
					erase_waypoint = false;
				}
			}
		}
		else if(path_to_current_target.size() > 0){
			while(erase_waypoint and path_to_current_target.size() > 0){
				Position current_waypoint = Position(path_to_current_target[0][0], path_to_current_target[0][1], 0);
				double dist_to_current_waypoint = current_position.getDistance(current_waypoint);
				if(dist_to_current_waypoint <= 0.75){
					cout << "Waypoint Achieved, removing waypoint" << endl;
					path_to_current_target.erase(path_to_current_target.begin());
				}
				else{
					erase_waypoint = false;
				}
			}
		}
	}

	FORRAction goTowardsPoint(Position current_position, Position target_position, double middle_distance_min){
		cout << "In goTowardsPoint" << endl;
		double distance_from_target = current_position.getDistance(target_position);
		cout << "Distance from target : " << distance_from_target << endl;

		// compute the angular difference between the direction to the target and the current robot direction
		double robot_direction = current_position.getTheta();
		double goal_direction = atan2((target_position.getY() - current_position.getY()), (target_position.getX() - current_position.getX()));
		double required_rotation = goal_direction - robot_direction;

		if(required_rotation > M_PI)
			required_rotation = required_rotation - (2*M_PI);
		if(required_rotation < -M_PI)
			required_rotation = required_rotation + (2*M_PI);
		cout << "Robot direction : " << robot_direction << ", Goal Direction : " << goal_direction << ", Required rotation : " << required_rotation << endl;
		// if the angular difference is greater than smallest turn possible 
		// pick the right turn to allign itself to the target

		FORRAction decision;
		int rotIntensity=0;
		while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
			rotIntensity++;
		}
		cout << "Rotation Intensity : " << rotIntensity << endl;
		if (rotIntensity > 1) {
			if (required_rotation < 0){
				decision = FORRAction(RIGHT_TURN, rotIntensity-1);
			}
			else {
				decision = FORRAction(LEFT_TURN, rotIntensity-1);
			}
		}
		else {
			decision = FORRAction(FORWARD, 4);
			int intensity=0;
			while(distance_from_target > move[intensity] and intensity < numMoves) {
				intensity++;
			}
			if(intensity > 1)
				intensity--;
			cout << "Move Intensity : " << intensity << endl;

			while(move[intensity] > middle_distance_min){
				intensity--;
			}
			cout << "Move Intensity : " << intensity << endl;
			if(go_to_top_point == true and intensity > 0)
				decision = FORRAction(FORWARD, intensity);
		}
		cout << "Action choosen : " << decision.type << "," << decision.parameter << endl;
		return decision;
	}


private:
	int length;
	int height;
	double time_threshold;
	double move[300];  
	double rotate[300];
	int numMoves, numRotates;
	int top_point_decisions;
	int decision_limit;
	int start_rotations;
	vector< vector<int> > frontier_grid;
	vector<Position> frontier_stack;
	vector<Position> frontier_stack_view;
	vector<Position> position_history;
	vector< vector<CartesianPoint> > laserEndpoints_history;
	Position current_target;
	Position top_point;
	vector< vector<int> > stack_grid;
	vector< vector<int> > traveled_grid;
	vector< vector<int> > passed_grid;
	vector< vector<int> > hit_grid;
	vector< vector<double> > path_to_top_point;
	vector< vector<double> > path_to_current_target;
	bool frontiers_complete;
	bool go_to_top_point;
};

#endif