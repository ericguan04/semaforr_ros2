#ifndef LOCALEXPLORE_H
#define LOCALEXPLORE_H

/**!
  * LocalExplore.h
  * 
  * /author: Raj Korpan
  *
  *          Conduct local exploration
  */

#include <FORRGeometry.h>
#include <Position.h>
#include "PathPlanner.h"
#include <vector>
#include <math.h>
#include <iostream>
#include <cmath>        //for atan2 and M_PI
#include <algorithm>
#include <map>
#include <set>
#include <numeric>
#include <queue>

using namespace std;

struct PotentialPoints{
	LineSegment pair;
	CartesianPoint start;
	CartesianPoint end;
	double dist_to_goal;
	double start_dist_to_goal;
	double end_dist_to_goal;
	double index;
	PotentialPoints(): pair(LineSegment()), start(CartesianPoint()), end(CartesianPoint()), dist_to_goal(0), start_dist_to_goal(0), end_dist_to_goal(0), index(-1) { }
	PotentialPoints(LineSegment p, CartesianPoint target, double ind){
		pair = p;
		start = pair.get_endpoints().first;
		end = pair.get_endpoints().second;
		dist_to_goal = distance(target, pair);
		start_dist_to_goal = start.get_distance(target);
		end_dist_to_goal = end.get_distance(target);
		index = ind;
		// printDetails();
	}
	void printDetails(){
		cout << "start " << start.get_x() << " " << start.get_y() << " end " << end.get_x() << " " << end.get_y() << " dist_to_goal " << dist_to_goal << " start_dist_to_goal " << start_dist_to_goal << " end_dist_to_goal " << end_dist_to_goal << endl;
	}
	bool operator==(const PotentialPoints p) {
		if((start == p.start and end == p.end) or end.get_distance(p.end) < 0.75){
			return true;
		}
		else{
			return false;
		}
	}
	bool operator < (const PotentialPoints p) const{
		if(dist_to_goal < 3.5 and p.dist_to_goal < 3.5){
			if(index < p.index){
				return false;
			}
			else{
				return true;
			}
		}
		else if(dist_to_goal >= 3.5 and p.dist_to_goal >= 3.5){
			if(index < p.index){
				return false;
			}
			else{
				return true;
			}
		}
		else if(dist_to_goal < 3.5 and p.dist_to_goal >= 3.5){
			return true;
		}
		else if(dist_to_goal >= 3.5 and p.dist_to_goal < 3.5){
			return false;
		}
		else{
			return true;
		}
		// if(dist_to_goal <= p.dist_to_goal and index >= p.index){
		// 	return true;
		// }
		// else if(dist_to_goal >= p.dist_to_goal and index <= p.index){
		// 	return false;
		// }
		// else{
		// 	if((start_dist_to_goal >= p.start_dist_to_goal or end_dist_to_goal >= p.end_dist_to_goal) and index <= p.index){
		// 		return false;
		// 	}
		// 	else{
		// 		return false;
		// 	}
		// }
	}
	bool operator > (const PotentialPoints p) const{
		if(dist_to_goal < 3.5 and p.dist_to_goal < 3.5){
			if(index < p.index){
				return true;
			}
			else{
				return false;
			}
		}
		else if(dist_to_goal >= 3.5 and p.dist_to_goal >= 3.5){
			if(index < p.index){
				return true;
			}
			else{
				return false;
			}
		}
		else if(dist_to_goal < 3.5 and p.dist_to_goal >= 3.5){
			return false;
		}
		else if(dist_to_goal >= 3.5 and p.dist_to_goal < 3.5){
			return true;
		}
		else{
			return false;
		}
		// if(dist_to_goal >= p.dist_to_goal and index <= p.index){
		// 	return true;
		// }
		// else if(dist_to_goal <= p.dist_to_goal and index >= p.index){
		// 	return false;
		// }
		// else{
		// 	if((start_dist_to_goal <= p.start_dist_to_goal or end_dist_to_goal <= p.end_dist_to_goal) and index >= p.index){
		// 		return false;
		// 	}
		// 	else{
		// 		return true;
		// 	}
		// }
	}
};

class LocalExplorer{
public:
	LocalExplorer(){
		already_started = false;
		start_of_potential = false;
		finished_potentials = false;
		started_random = false;
		found_new_potentials = false;
		distance_threshold = 2;
		start_index = 0;
		initial_coverage = 0;
		current_coverage = 0;
	};
	~LocalExplorer(){};
	bool getAlreadyStarted() { return already_started; }
	bool getStartedRandom() { return started_random; }
	bool getAtStartOfPotential() { return start_of_potential; }
	bool getFinishedPotentials() {
		// cout << "potential_queue " << potential_queue.size() << endl;
		if(potential_queue.size() > 0){
			bool picked_new = false;
			int count = potential_queue.size()-1;
			// cout << "count " << count << endl;
			while(!picked_new and count > 0){
				current_potential = potential_queue.top();
				if(!alreadyInStack(current_potential)){
					picked_new = true;
				}
				potential_exploration.push_back(current_potential);
				potential_queue.pop();
				count = count - 1;
				// cout << "potential_queue " << potential_queue.size() << " count " << count << " picked_new " << picked_new << endl;
			}
			if(picked_new == true){
				// cout << "current_potential ";
				// current_potential.printDetails();
				// potential_queue.pop();
				finished_potentials = false;
			}
			else{
				finished_potentials = true;
			}
		}
		else{
			finished_potentials = true;
		}
		return finished_potentials;
	}
	void resetLocalExplorer(){
		already_started = false;
		start_of_potential = false;
		finished_potentials = false;
		started_random = false;
		found_new_potentials = false;
		task = CartesianPoint();
		potential_exploration.clear();
		potential_queue = priority_queue<PotentialPoints, vector<PotentialPoints>, greater<PotentialPoints> >();
		random_queue = priority_queue<PotentialPoints, vector<PotentialPoints>, greater<PotentialPoints> >();
		current_potential = PotentialPoints();
		laser_to_explore = PotentialPoints();
		start_index = 0;
		initial_coverage = 0;
		current_coverage = 0;
	}
	void setPathPlanner(PathPlanner *planner){
		pathPlanner = planner;
	}
	void setQueue(CartesianPoint goal, vector< LineSegment > pairs){
		// cout << "inside setQueue " << pairs.size() << endl;
		task = goal;
		for(int i = 0; i < pairs.size(); i++){
			// cout << "pair " << i << " " << pairs[i].get_length() << endl;
			if(pairs[i].get_length() >= distance_threshold){
				potential_queue.push(PotentialPoints(pairs[i], task, start_index));
			}
		}
		current_potential = potential_queue.top();
		potential_exploration.push_back(current_potential);
		// cout << "current_potential ";
		// current_potential.printDetails();
		potential_queue.pop();
		already_started = true;
	}
	void addToQueue(vector< LineSegment > pairs){
		// cout << "inside addToQueue " << pairs.size() << endl;
		start_index++;
		int count_before = potential_queue.size();
		for(int i = 0; i < pairs.size(); i++){
			// cout << "pair " << i << " " << pairs[i].get_length() << endl;
			if(pairs[i].get_length() >= distance_threshold){
				potential_queue.push(PotentialPoints(pairs[i], task, start_index));
			}
		}
		int count_after = potential_queue.size();
		if(count_before == 0 and count_after > 0 and started_random == true){
			found_new_potentials = true;
		}
	}
	void atStartOfPotential(CartesianPoint current){
		if(current.get_distance(current_potential.start) < 0.75){
			start_of_potential = true;
		}
		else{
			start_of_potential = false;
		}
	}
	bool atEndOfPotential(CartesianPoint current){
		if(current.get_distance(current_potential.end) < 0.75 or found_new_potentials == true){
			found_new_potentials = false;
			return true;
		}
		else{
			return false;
		}
	}
	CartesianPoint getStartOfPotential(){
		return current_potential.start;
	}
	CartesianPoint getEndOfPotential(){
		return current_potential.end;
	}
	bool alreadyInStack(PotentialPoints segment){
		bool inCompleted = false;
		for(int i = 0; i < potential_exploration.size(); i++){
			if(segment == potential_exploration[i]){
				inCompleted = true;
				break;
			}
		}
		bool alreadyCovered = false;
		if(coverage[(int)(segment.end.get_x())][(int)(segment.end.get_y())] == 0){
			alreadyCovered = true;
		}
		if(inCompleted or alreadyCovered){
			return true;
		}
		else{
			return false;
		}
	}
	vector<CartesianPoint> getPathToStart(CartesianPoint current){
		// cout << "in getPathToStart " << endl;
		if(pathPlanner->getName() == "skeleton"){
			pathPlanner->resetPath();
		}
		else if(pathPlanner->getName() == "hallwayskel"){
			pathPlanner->resetOrigPath();
		}
		vector<CartesianPoint> waypoints;
		// cout << current.get_x() << " " << current.get_y() << endl;
		Node s(1, current.get_x()*100, current.get_y()*100);
		pathPlanner->setSource(s);
		// cout << current_potential.start.get_x() << " " << current_potential.start.get_y() << endl;
		Node t(1, current_potential.start.get_x()*100, current_potential.start.get_y()*100);
		pathPlanner->setTarget(t);
		cout << "plan generation status" << pathPlanner->calcPath(true) << endl;
		list<int> waypointInd;
		if(pathPlanner->getName() == "skeleton"){
			waypointInd = pathPlanner->getPath();
		}
		else if(pathPlanner->getName() == "hallwayskel"){
			waypointInd = pathPlanner->getOrigPaths()[2];
		}
		if(waypointInd.size() > 0){
			int step = -1;
			int max_step = waypointInd.size()-1;
			list<int>::iterator it;
			for ( it = waypointInd.begin(); it != waypointInd.end(); it++ ){
				step = step + 1;
				// cout << "node " << (*it) << " step " << step << endl;
				Graph *navGraph;
				if(pathPlanner->getName() == "skeleton"){
					navGraph = pathPlanner->getGraph();
				}
				else if(pathPlanner->getName() == "hallwayskel"){
					navGraph = pathPlanner->getOrigGraph();
				}
				double r_x = navGraph->getNode(*it).getX()/100.0;
				double r_y = navGraph->getNode(*it).getY()/100.0;
				double r = navGraph->getNode(*it).getRadius();
				CartesianPoint waypoint(r_x,r_y);
				waypoints.push_back(waypoint);
				list<int>::iterator itr1; 
				itr1 = it; 
				advance(itr1, 1);
				int forward_step = step + 1;
				// cout << "max_step " << max_step << " forward_step " << forward_step << endl;
				if(forward_step <= max_step){
					vector<CartesianPoint> path_from_edge = navGraph->getEdge(*it, *itr1)->getEdgePath(true);
					if(CartesianPoint(r_x,r_y).get_distance(path_from_edge[0]) <= r){
						for(int i = 0; i < path_from_edge.size(); i++){
							waypoints.push_back(path_from_edge[i]);
						}
					}
					else{
						std::reverse(path_from_edge.begin(),path_from_edge.end());
						for(int i = 0; i < path_from_edge.size(); i++){
							waypoints.push_back(path_from_edge[i]);
						}
					}
				}
				// cout << "num of waypoints " << skeleton_waypoints.size() << endl;
			}
		}
		else{
			waypoints.push_back(current);
		}
		waypoints.push_back(current_potential.start);
		if(pathPlanner->getName() == "skeleton"){
			pathPlanner->resetPath();
		}
		else if(pathPlanner->getName() == "hallwayskel"){
			pathPlanner->resetOrigPath();
		}
		return waypoints;
	}

	vector<CartesianPoint> getPathToEnd(){
		vector<CartesianPoint> waypoints;
		double tx, ty;
		for(double j = 0; j <= 1; j += 0.05){
			tx = (current_potential.end.get_x() * j) + (current_potential.start.get_x() * (1 - j));
			ty = (current_potential.end.get_y() * j) + (current_potential.start.get_y() * (1 - j));
			waypoints.push_back(CartesianPoint(tx, ty));
		}
		return waypoints;
	}
	vector<CartesianPoint> randomExploration(CartesianPoint current, vector<CartesianPoint> laserEndpoints, CartesianPoint goal){
		already_started = true;
		started_random = true;
		// cout << "randomExploration " << current.get_x() << " " << current.get_y() << " " << goal.get_x() << " " << goal.get_y() << endl;
		task = goal;
		// cout << "coverage" << endl;
		// for(int i = 0; i < coverage.size(); i++){
		// 	for(int j = 0; j < coverage[i].size(); j++){
		// 		cout << coverage[i][j] << " ";
		// 	}
		// 	cout << endl;
		// }
		double dist_to_goal = task.get_distance(current);
		double max_search_radius = (int)(dist_to_goal)+1.0;
		double min_search_radius = 1.0;
		// cout << "dist_to_goal " << dist_to_goal << " min_search_radius " << min_search_radius << " max_search_radius " << max_search_radius << endl;
		vector<double> search_radii;
		if(max_search_radius >= min_search_radius){
			for(double i = min_search_radius; i <= max_search_radius; i += 1.0){
				search_radii.push_back(i);
			}
		}
		else{
			search_radii.push_back(min_search_radius);
		}
		// cout << "search_radii " << search_radii.size() << endl;
		vector<bool> search_access;
		vector<int> laser_index;
		for(int i = 0; i < search_radii.size(); i++){
			bool canAccessRegion = false;
			double distLaserPosToPoint = current.get_distance(task);
			if(distLaserPosToPoint - search_radii[i] > 20){
				// cout << search_radii[i] << " 0" << endl;
				search_access.push_back(false);
				laser_index.push_back(-1);
			}
			else{
				int ind = -1;
				for(int j = 0; j < laserEndpoints.size(); j++){
					//// RCLCPP_DEBUG_STREAM(this->get_logger(), "Laser endpoint : " << laserEndpoints[j].get_x() << "," << laserEndpoints[j].get_y());
					if(do_intersect(Circle(task, search_radii[i]), LineSegment(current, laserEndpoints[j]))){
						canAccessRegion = true;
						ind = j;
						break;
					}
				}
				// cout << search_radii[i] << " " << canAccessRegion << endl;
				search_access.push_back(canAccessRegion);
				laser_index.push_back(ind);
			}
		}
		for(int i = 0; i < search_radii.size(); i++){
			if(search_access[i] and coverage[(int)(laserEndpoints[laser_index[i]].get_x())][(int)(laserEndpoints[laser_index[i]].get_y())] < 0){
				// cout << "closest visible radii " << search_radii[i] << endl;
				random_queue.push(PotentialPoints(LineSegment(current, laserEndpoints[laser_index[i]]), task, 0));
			}
		}
		if(random_queue.size() > 0){
			// cout << "random_queue " << random_queue.size() << endl;
			bool picked_new = false;
			if(random_queue.size() > 0){
				int count = random_queue.size()-1;
				// cout << "count " << count << endl;
				while(!picked_new and count > 0){
					laser_to_explore = random_queue.top();
					if(!alreadyInStack(laser_to_explore)){
						picked_new = true;
					}
					potential_exploration.push_back(laser_to_explore);
					random_queue.pop();
					count = count - 1;
					// cout << "random_queue " << random_queue.size() << " count " << count << " picked_new " << picked_new << endl;
				}
			}
			if(picked_new == true){
				vector<CartesianPoint> waypoints;
				if(!(laser_to_explore.start == current)){
					if(pathPlanner->getName() == "skeleton"){
						pathPlanner->resetPath();
					}
					else if(pathPlanner->getName() == "hallwayskel"){
						pathPlanner->resetOrigPath();
					}
					// cout << current.get_x() << " " << current.get_y() << endl;
					Node s(1, current.get_x()*100, current.get_y()*100);
					pathPlanner->setSource(s);
					// cout << laser_to_explore.start.get_x() << " " << laser_to_explore.start.get_y() << endl;
					Node t(1, laser_to_explore.start.get_x()*100, laser_to_explore.start.get_y()*100);
					pathPlanner->setTarget(t);
					cout << "plan generation status" << pathPlanner->calcPath(true) << endl;
					list<int> waypointInd;
					if(pathPlanner->getName() == "skeleton"){
						waypointInd = pathPlanner->getPath();
					}
					else if(pathPlanner->getName() == "hallwayskel"){
						waypointInd = pathPlanner->getOrigPaths()[2];
					}
					if(waypointInd.size() > 0){
						int step = -1;
						int max_step = waypointInd.size()-1;
						list<int>::iterator it;
						for ( it = waypointInd.begin(); it != waypointInd.end(); it++ ){
							step = step + 1;
							// cout << "node " << (*it) << " step " << step << endl;
							Graph *navGraph;
							if(pathPlanner->getName() == "skeleton"){
								navGraph = pathPlanner->getGraph();
							}
							else if(pathPlanner->getName() == "hallwayskel"){
								navGraph = pathPlanner->getOrigGraph();
							}
							double r_x = navGraph->getNode(*it).getX()/100.0;
							double r_y = navGraph->getNode(*it).getY()/100.0;
							double r = navGraph->getNode(*it).getRadius();
							CartesianPoint waypoint(r_x,r_y);
							waypoints.push_back(waypoint);
							list<int>::iterator itr1; 
							itr1 = it; 
							advance(itr1, 1);
							int forward_step = step + 1;
							// cout << "max_step " << max_step << " forward_step " << forward_step << endl;
							if(forward_step <= max_step){
								vector<CartesianPoint> path_from_edge = navGraph->getEdge(*it, *itr1)->getEdgePath(true);
								if(CartesianPoint(r_x,r_y).get_distance(path_from_edge[0]) <= r){
									for(int i = 0; i < path_from_edge.size(); i++){
										waypoints.push_back(path_from_edge[i]);
									}
								}
								else{
									std::reverse(path_from_edge.begin(),path_from_edge.end());
									for(int i = 0; i < path_from_edge.size(); i++){
										waypoints.push_back(path_from_edge[i]);
									}
								}
							}
							// cout << "num of waypoints " << skeleton_waypoints.size() << endl;
						}
					}
					else{
						waypoints.push_back(current);
					}
					waypoints.push_back(laser_to_explore.start);
					if(pathPlanner->getName() == "skeleton"){
						pathPlanner->resetPath();
					}
					else if(pathPlanner->getName() == "hallwayskel"){
						pathPlanner->resetOrigPath();
					}
				}
				double tx, ty;
				for(double j = 0; j <= 1; j += 0.1){
					tx = (laser_to_explore.end.get_x() * j) + (laser_to_explore.start.get_x() * (1 - j));
					ty = (laser_to_explore.end.get_y() * j) + (laser_to_explore.start.get_y() * (1 - j));
					waypoints.push_back(CartesianPoint(tx, ty));
				}
				return waypoints;
			}
		}
		// cout << "go to closest_coverage" << endl;
		int min_distance = 10000;
		int task_x = (int)(task.get_x());
		int task_y = (int)(task.get_y());
		int closest_x, closest_y;
		for(int i = 0; i < coverage.size(); i++){
			for(int j = 0; j < coverage[i].size(); j++){
				int dist_to_task = abs(task_x - i) + abs(task_y - j);
				if(coverage[i][j] > 0 and dist_to_task < min_distance){
					min_distance = dist_to_task;
					closest_x = i;
					closest_y = j;
				}
			}
		}
		// cout << "closest_x " << closest_x << " closest_y " << closest_y << endl;
		PotentialPoints closest_coverage = PotentialPoints(LineSegment(CartesianPoint(closest_x, closest_y), CartesianPoint(closest_x, closest_y)), task, 0);
		if(pathPlanner->getName() == "skeleton"){
			pathPlanner->resetPath();
		}
		else if(pathPlanner->getName() == "hallwayskel"){
			pathPlanner->resetOrigPath();
		}
		vector<CartesianPoint> waypoints;
		// cout << current.get_x() << " " << current.get_y() << endl;
		Node s(1, current.get_x()*100, current.get_y()*100);
		pathPlanner->setSource(s);
		// cout << closest_coverage.start.get_x() << " " << closest_coverage.start.get_y() << endl;
		Node t(1, closest_coverage.start.get_x()*100, closest_coverage.start.get_y()*100);
		pathPlanner->setTarget(t);
		cout << "plan generation status" << pathPlanner->calcPath(true) << endl;
		list<int> waypointInd;
		if(pathPlanner->getName() == "skeleton"){
			waypointInd = pathPlanner->getPath();
		}
		else if(pathPlanner->getName() == "hallwayskel"){
			waypointInd = pathPlanner->getOrigPaths()[2];
		}
		if(waypointInd.size() > 0){
			int step = -1;
			int max_step = waypointInd.size()-1;
			list<int>::iterator it;
			for ( it = waypointInd.begin(); it != waypointInd.end(); it++ ){
				step = step + 1;
				// cout << "node " << (*it) << " step " << step << endl;
				Graph *navGraph;
				if(pathPlanner->getName() == "skeleton"){
					navGraph = pathPlanner->getGraph();
				}
				else if(pathPlanner->getName() == "hallwayskel"){
					navGraph = pathPlanner->getOrigGraph();
				}
				double r_x = navGraph->getNode(*it).getX()/100.0;
				double r_y = navGraph->getNode(*it).getY()/100.0;
				double r = navGraph->getNode(*it).getRadius();
				CartesianPoint waypoint(r_x,r_y);
				waypoints.push_back(waypoint);
				list<int>::iterator itr1; 
				itr1 = it; 
				advance(itr1, 1);
				int forward_step = step + 1;
				// cout << "max_step " << max_step << " forward_step " << forward_step << endl;
				if(forward_step <= max_step){
					vector<CartesianPoint> path_from_edge = navGraph->getEdge(*it, *itr1)->getEdgePath(true);
					if(CartesianPoint(r_x,r_y).get_distance(path_from_edge[0]) <= r){
						for(int i = 0; i < path_from_edge.size(); i++){
							waypoints.push_back(path_from_edge[i]);
						}
					}
					else{
						std::reverse(path_from_edge.begin(),path_from_edge.end());
						for(int i = 0; i < path_from_edge.size(); i++){
							waypoints.push_back(path_from_edge[i]);
						}
					}
				}
				// cout << "num of waypoints " << skeleton_waypoints.size() << endl;
			}
		}
		else{
			waypoints.push_back(current);
		}
		waypoints.push_back(closest_coverage.start);
		if(pathPlanner->getName() == "skeleton"){
			pathPlanner->resetPath();
		}
		else if(pathPlanner->getName() == "hallwayskel"){
			pathPlanner->resetOrigPath();
		}
		return waypoints;
	}

	void updateCoverage(CartesianPoint current){
		// cout << "updated Coverage " << (int)(current.get_x()) << " " << (int)(current.get_y()) << endl;
		if(coverage[(int)(current.get_x())][(int)(current.get_y())] == -1){
			current_coverage = current_coverage + 1;
		}
		coverage[(int)(current.get_x())][(int)(current.get_y())] = 0;
	}

	vector< vector<int> > getCoverage() { return coverage; }

	void setCoverage(vector< vector<int> > coverage_grid){
		coverage = coverage_grid;
		double count = 0;
		for(int i = 0; i < coverage.size(); i++){
			for(int j = 0; j < coverage[i].size(); j++){
				if(coverage[i][j] >= 0){
					count = count + 1;
				}
			}
		}
		initial_coverage = count;
		current_coverage = count;
	}

	bool triggerLearning(){
		if(initial_coverage == 0){
			return false;
		}
		else if(current_coverage / initial_coverage - 1 > 0.1){
			initial_coverage = current_coverage;
			return true;
		}
		else{
			return false;
		}
	}

private:
	double distance_threshold;
	bool already_started;
	CartesianPoint task;
	vector< PotentialPoints > potential_exploration;
	priority_queue<PotentialPoints, vector<PotentialPoints>, greater<PotentialPoints> > potential_queue;
	priority_queue<PotentialPoints, vector<PotentialPoints>, greater<PotentialPoints> > random_queue;
	PotentialPoints current_potential;
	PotentialPoints laser_to_explore;
	bool start_of_potential;
	PathPlanner *pathPlanner;
	bool finished_potentials;
	bool started_random;
	vector< vector<int> > coverage;
	bool found_new_potentials;
	double start_index;
	int initial_coverage, current_coverage;
};

#endif