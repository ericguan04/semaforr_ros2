#ifndef HIGHWAYEXPLORE_H
#define HIGHWAYEXPLORE_H

/**!
  * HighwayExplore.h
  * 
  * /author: Raj Korpan
  *
  *          Construct Highway Graph
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

struct DecisionPoint{
	Position point;
	sensor_msgs::msg::LaserScan laser;
	vector<CartesianPoint> laserEndpoints;
	double farthest_view_left, farthest_view_middle, farthest_view_right;
	// double farthest_angle_left, farthest_angle_middle, farthest_angle_right;
	double farthest_distance_left, farthest_distance_middle, farthest_distance_right;
	double middle_distance, middle_distance_min, left_distance, right_distance, overall_avg_distance, overall_max_distance, overall_min_distance, overall_median_distance, overall_stdev_distance;
	double left_width, right_width;
	// double left_width_max, right_width_max;
	bool direction;
	Position middle_point;
	Position left_point;
	Position right_point;
	DecisionPoint(): point(Position()), laser(sensor_msgs::msg::LaserScan()) { }
	DecisionPoint(Position p, sensor_msgs::msg::LaserScan ls, bool dir = false){
		point = p;
		laser = ls;
		direction = dir;
		double start_angle = ls.angle_min;
		double increment = ls.angle_increment;
		double r_x = p.getX();
		double r_y = p.getY();
		double r_ang = p.getTheta();
		farthest_view_left = 0;
		farthest_view_middle = 329;
		farthest_view_right = 659;
		farthest_distance_left = 0;
		farthest_distance_middle = 0;
		farthest_distance_right = 0;
		middle_distance = 0;
		middle_distance_min = 0;
		left_distance = 0;
		right_distance = 0;
		overall_avg_distance = 0;
		overall_max_distance = 0;
		overall_min_distance = 0;
		overall_median_distance = 0;
		overall_stdev_distance = 0;
		double max_value_left = 0;
		double max_value_middle = 0;
		double max_value_right = 0;
		double min_value_middle = 50;
		double max_value_overall = 0;
		double min_value_overall = 50;
		double min_left_left = 50;
		double min_left_right = 50;
		double min_right_left = 50;
		double min_right_right = 50;
		int max_left_ind, max_right_ind;
		double max_left_left = 0;
		double max_left_right = 0;
		double max_right_left = 0;
		double max_right_right = 0;
		vector<double> sorted_ranges;
		double left_x = 0, left_y = 0, right_x = 0, right_y = 0, middle_x = 0, middle_y = 0;
		double min_left_left_x = 0, min_left_right_x = 0, min_right_left_x = 0, min_right_right_x = 0, min_left_left_y = 0, min_left_right_y = 0, min_right_left_y = 0, min_right_right_y = 0;
		// double max_left_left_x = 0, max_left_right_x = 0, max_right_left_x = 0, max_right_right_x = 0, max_left_left_y = 0, max_left_right_y = 0, max_right_left_y = 0, max_right_right_y = 0;
		for(int i = 0; i < ls.ranges.size(); i++){
			sorted_ranges.push_back(ls.ranges[i]);
			double angle = start_angle + r_ang;
			if(i >= 580 and i <= 620){
				left_distance += ls.ranges[i];
				if(ls.ranges[i] > max_value_left){
					// farthest_distance_left = ls.ranges[i];
					farthest_view_left = i;
					// farthest_angle_left = angle;
					max_value_left = ls.ranges[i];
					// left_point = Position(r_x + ls.ranges[i]*cos(angle), r_y + ls.ranges[i]*sin(angle), 0);
				}
				left_x += r_x + ls.ranges[i]*cos(angle);
				left_y += r_y + ls.ranges[i]*sin(angle);
			}
			else if(i >= 40 and i <= 80){
				right_distance += ls.ranges[i];
				if(ls.ranges[i] > max_value_right){
					// farthest_distance_right = ls.ranges[i];
					farthest_view_right = i;
					// farthest_angle_right = angle;
					max_value_right = ls.ranges[i];
					// right_point = Position(r_x + ls.ranges[i]*cos(angle), r_y + ls.ranges[i]*sin(angle), 0);
				}
				right_x += r_x + ls.ranges[i]*cos(angle);
				right_y += r_y + ls.ranges[i]*sin(angle);
			}
			else if(i >= 195 and i <= 465){
				middle_distance += ls.ranges[i];
				if(ls.ranges[i] > max_value_middle){
					// farthest_distance_middle = ls.ranges[i];
					farthest_view_middle = i;
					// farthest_angle_middle = angle;
					max_value_middle = ls.ranges[i];
					// middle_point = Position(r_x + ls.ranges[i]*cos(angle), r_y + ls.ranges[i]*sin(angle), 0);
				}
				middle_x += r_x + ls.ranges[i]*cos(angle);
				middle_y += r_y + ls.ranges[i]*sin(angle);
				if(ls.ranges[i] < min_value_middle){
					middle_distance_min = ls.ranges[i];
					min_value_middle = ls.ranges[i];
				}
			}
			overall_avg_distance += ls.ranges[i];
			if(ls.ranges[i] > max_value_overall){
				max_value_overall = ls.ranges[i];
				overall_max_distance = ls.ranges[i];
			}
			if(ls.ranges[i] < min_value_overall){
				min_value_overall = ls.ranges[i];
				overall_min_distance = ls.ranges[i];
			}
			laserEndpoints.push_back(CartesianPoint(r_x + ls.ranges[i]*cos(angle), r_y + ls.ranges[i]*sin(angle)));
			if(i <= 135){
				if(ls.ranges[i] > max_right_right){
					max_right_right = ls.ranges[i];
					max_right_ind = i;
				}
			}
			else if(i >= 524){
				if(ls.ranges[i] > max_left_left){
					max_left_left = ls.ranges[i];
					max_left_ind = i;
				}
			}
			start_angle = start_angle + increment;
		}
		for(int i = farthest_view_right - 3; i <= farthest_view_right + 3; i++){
			farthest_distance_right += ls.ranges[i];
		}
		farthest_distance_right = farthest_distance_right / 7.0;
		for(int i = farthest_view_left - 3; i <= farthest_view_left + 3; i++){
			farthest_distance_left += ls.ranges[i];
		}
		farthest_distance_left = farthest_distance_left / 7.0;
		for(int i = farthest_view_middle - 3; i <= farthest_view_middle + 3; i++){
			farthest_distance_middle += ls.ranges[i];
		}
		farthest_distance_middle = farthest_distance_middle / 7.0;
		if(max_right_ind == 0)
			max_right_ind = 1;
		if(max_right_ind == 135)
			max_right_ind = 134;
		if(max_left_ind == 524)
			max_left_ind = 525;
		if(max_left_ind == ls.ranges.size()-1)
			max_left_ind = ls.ranges.size()-2;
		double min_right_right_thr = 0, min_right_left_thr = 0, min_left_right_thr = 0, min_left_left_thr = 0;
		int min_right_right_ind = 0, min_right_left_ind = 0, min_left_right_ind = 0, min_left_left_ind = 0;
		start_angle = ls.angle_min;
		for(int i = 0; i < ls.ranges.size(); i++){
			double angle = start_angle + r_ang;
			if(i < max_right_ind){
				if(ls.ranges[i] < min_right_left){
					min_right_left = ls.ranges[i];
					min_right_left_x = r_x + ls.ranges[i]*cos(angle);
					min_right_left_y = r_y + ls.ranges[i]*sin(angle);
					min_right_left_ind = i;
					min_right_left_thr = ls.ranges[i] * 0.1;
				}
				// else if(ls.ranges[i] > max_right_left){
				// 	max_right_left = ls.ranges[i];
				// 	max_right_left_x = r_x + ls.ranges[i]*cos(angle);
				// 	max_right_left_y = r_y + ls.ranges[i]*sin(angle);
				// }
			}
			else if(i > max_right_ind and i <= 135){
				if(ls.ranges[i] < min_right_right){
					min_right_right = ls.ranges[i];
					min_right_right_x = r_x + ls.ranges[i]*cos(angle);
					min_right_right_y = r_y + ls.ranges[i]*sin(angle);
					min_right_right_ind = i;
					min_right_right_thr = ls.ranges[i] * 0.1;
				}
				// else if(ls.ranges[i] > max_right_right){
				// 	max_right_right = ls.ranges[i];
				// 	max_right_right_x = r_x + ls.ranges[i]*cos(angle);
				// 	max_right_right_y = r_y + ls.ranges[i]*sin(angle);
				// }
			}
			else if(i >= 524 and i < max_left_ind){
				if(ls.ranges[i] < min_left_left){
					min_left_left = ls.ranges[i];
					min_left_left_x = r_x + ls.ranges[i]*cos(angle);
					min_left_left_y = r_y + ls.ranges[i]*sin(angle);
					min_left_left_ind = i;
					min_left_left_thr = ls.ranges[i] * 0.1;
				}
				// else if(ls.ranges[i] > max_left_left){
				// 	max_left_left = ls.ranges[i];
				// 	max_left_left_x = r_x + ls.ranges[i]*cos(angle);
				// 	max_left_left_y = r_y + ls.ranges[i]*sin(angle);
				// }
			}
			else if(i > max_left_ind){
				if(ls.ranges[i] < min_left_right){
					min_left_right = ls.ranges[i];
					min_left_right_x = r_x + ls.ranges[i]*cos(angle);
					min_left_right_y = r_y + ls.ranges[i]*sin(angle);
					min_left_right_ind = i;
					min_left_right_thr = ls.ranges[i] * 0.1;
				}
				// else if(ls.ranges[i] > max_left_right){
				// 	max_left_right = ls.ranges[i];
				// 	max_left_right_x = r_x + ls.ranges[i]*cos(angle);
				// 	max_left_right_y = r_y + ls.ranges[i]*sin(angle);
				// }
			}
			start_angle = start_angle + increment;
		}
		start_angle = ls.angle_min;
		for(int i = 0; i < ls.ranges.size(); i++){
			double angle = start_angle + r_ang;
			if(i > min_right_left_ind and i < max_right_ind){
				if(abs(ls.ranges[min_right_left_ind] - ls.ranges[i]) <= min_right_left_thr or (abs(min_right_left_x - r_x + ls.ranges[i]*cos(angle)) + abs(min_right_left_y - r_y + ls.ranges[i]*sin(angle))) <= min_right_left_thr){
					min_right_left = ls.ranges[i];
					min_right_left_x = r_x + ls.ranges[i]*cos(angle);
					min_right_left_y = r_y + ls.ranges[i]*sin(angle);
					min_right_left_ind = i;
					min_right_left_thr = ls.ranges[i] * 0.1;
				}
			}
			else if(i > min_left_left_ind and i < max_left_ind){
				if(abs(ls.ranges[min_left_left_ind] - ls.ranges[i]) <= min_left_left_thr or (abs(min_left_left_x - r_x + ls.ranges[i]*cos(angle)) + abs(min_left_left_y - r_y + ls.ranges[i]*sin(angle))) <= min_left_left_thr){
					min_left_left = ls.ranges[i];
					min_left_left_x = r_x + ls.ranges[i]*cos(angle);
					min_left_left_y = r_y + ls.ranges[i]*sin(angle);
					min_left_left_ind = i;
					min_left_left_thr = ls.ranges[i] * 0.1;
				}
			}
			start_angle = start_angle + increment;
		}
		start_angle = ls.angle_min;
		for(int i = ls.ranges.size()-1; i >= 0; i--){
			double angle = start_angle + r_ang;
			if(i > max_right_ind and i < min_right_right_ind){
				if(abs(ls.ranges[min_right_right_ind] - ls.ranges[i]) <= min_right_right_thr or (abs(min_right_right_x - r_x + ls.ranges[i]*cos(angle)) + abs(min_right_right_y - r_y + ls.ranges[i]*sin(angle))) <= min_right_right_thr){
					min_right_right = ls.ranges[i];
					min_right_right_x = r_x + ls.ranges[i]*cos(angle);
					min_right_right_y = r_y + ls.ranges[i]*sin(angle);
					min_right_right_ind = i;
					min_right_right_thr = ls.ranges[i] * 0.1;
				}
			}
			else if(i > max_left_ind and i < min_left_right_ind){
				if(abs(ls.ranges[min_left_right_ind] - ls.ranges[i]) <= min_left_right_thr or (abs(min_left_right_x - r_x + ls.ranges[i]*cos(angle)) + abs(min_left_right_y - r_y + ls.ranges[i]*sin(angle))) <= min_left_right_thr){
					min_left_right = ls.ranges[i];
					min_left_right_x = r_x + ls.ranges[i]*cos(angle);
					min_left_right_y = r_y + ls.ranges[i]*sin(angle);
					min_left_right_ind = i;
					min_left_right_thr = ls.ranges[i] * 0.1;
				}
			}
			start_angle = start_angle + increment;
		}
		middle_distance = middle_distance / 271.0;
		left_distance = left_distance / 41.0;
		right_distance = right_distance / 41.0;
		overall_avg_distance = overall_avg_distance / ls.ranges.size();
		if(left_x < 0)
			left_x = 0;
		if(left_y < 0)
			left_y = 0;
		if(right_x < 0)
			right_x = 0;
		if(right_y < 0)
			right_y = 0;
		if(middle_x < 0)
			middle_x = 0;
		if(middle_y < 0)
			middle_y = 0;
		left_point = Position(left_x/ 41.0, left_y/ 41.0, 0);
		right_point = Position(right_x/ 41.0, right_y/ 41.0, 0);
		middle_point = Position(middle_x/ 271.0, middle_y/ 271.0, 0);
		sort(sorted_ranges.begin(), sorted_ranges.end());
		overall_median_distance = (sorted_ranges[sorted_ranges.size() / 2 - 1] + sorted_ranges[sorted_ranges.size() / 2]) / 2;
		double sqSum = inner_product(sorted_ranges.begin(), sorted_ranges.end(), sorted_ranges.begin(), 0.0);
        overall_stdev_distance = sqrt(sqSum / sorted_ranges.size() - overall_avg_distance * overall_avg_distance);
        left_width = sqrt((min_left_right_x - min_left_left_x) * (min_left_right_x - min_left_left_x) + (min_left_right_y - min_left_left_y) * (min_left_right_y - min_left_left_y));
        right_width = sqrt((min_right_right_x - min_right_left_x) * (min_right_right_x - min_right_left_x) + (min_right_right_y - min_right_left_y) * (min_right_right_y - min_right_left_y));
        // left_width_max = sqrt((max_left_right_x - max_left_left_x) * (max_left_right_x - max_left_left_x) + (max_left_right_y - max_left_left_y) * (max_left_right_y - max_left_left_y));
        // right_width_max = sqrt((max_right_right_x - max_right_left_x) * (max_right_right_x - max_right_left_x) + (max_right_right_y - max_right_left_y) * (max_right_right_y - max_right_left_y));
	}
	bool operator==(const DecisionPoint p) {
		return (point == p.point);
	}
	bool operator < (const DecisionPoint p) const{
		double dist1 = 0;
		if(direction == true){
			dist1 = right_distance;
		}
		else{
			dist1 = left_distance;
		}
		double dist2 = 0;
		if(p.direction == true){
			dist2 = p.right_distance;
		}
		else{
			dist2 = p.left_distance;
		}
		if(dist1 < dist2)
			return true;
		else
			return false;
		// if(farthest_distance_left < p.farthest_distance_left or farthest_distance_middle < p.farthest_distance_middle or farthest_distance_right < p.farthest_distance_right)
		// 	return true;
		// else
		// 	return false;
	}
};

class Highway{
public:
	Highway(DecisionPoint point){
		highway_points.push_back(point);
		start_point = point;
		avg_theta = point.point.getTheta();
	}
	// vector<DecisionPoint> getHighwayPoints(){return highway_points;}
	int getSizeHighway(){return highway_points.size();}
	double getAvgTheta(){return avg_theta;}
	DecisionPoint getStartPoint(){return start_point;}

	void addPointToHighway(DecisionPoint new_point){
		std::vector<DecisionPoint>::const_iterator it;
		it = find(highway_points.begin(), highway_points.end(), new_point);
		if(it == highway_points.end()){
			highway_points.push_back(new_point);
		}
		if(highway_points.size() >= 40){
			double avg_s = 0;
			double avg_c = 0;
			for(int i = highway_points.size() - 40; i < highway_points.size(); i++){
				avg_s += sin(highway_points[i].point.getTheta());
				avg_c += cos(highway_points[i].point.getTheta());
			}
			avg_s = avg_s/40.0;
			avg_c = avg_c/40.0;
			avg_theta = atan2(avg_s, avg_c);
		}
		else{
			avg_theta = new_point.point.getTheta();
		}
	}

	// DecisionPoint getClosestPointOnHighway(Position point){
	// 	DecisionPoint closest;
	// 	double min_distance = 10000000;
	// 	for(int i = 0; i < highway_points.size(); i++){
	// 		double distance = highway_points[i].point.getDistance(point);
	// 		if(distance < min_distance){
	// 			min_distance = distance;
	// 			closest = highway_points[i];
	// 		}
	// 	}
	// 	return closest;
	// }

private:
	vector<DecisionPoint> highway_points;
	DecisionPoint start_point;
	double avg_theta;
};

class HighwayExplorer{
public:
	HighwayExplorer(int l, int h, double dthreshold, double tthreshold, double decthreshold, double arrMove[], double arrRotate[], int moveArrMax, int rotateArrMax){
		distance_threshold = dthreshold;
		time_threshold = tthreshold;
		length = l;
		height = h;
		numMoves = moveArrMax;
		numRotates = rotateArrMax;
		for(int i = 0 ; i < numMoves ; i++) move[i] = arrMove[i];
		for(int i = 0 ; i < numRotates ; i++) rotate[i] = arrRotate[i];
		// cout << "Highway length " << length << " height " << height << endl;
		for(int i = 0; i < l; i++){
			vector<int> col;
			for(int j = 0; j < h; j ++){
				col.push_back(-1);
			}
			highway_grid.push_back(col);
		}
		for(int i = 0; i < l; i++){
			vector<int> col;
			for(int j = 0; j < h; j ++){
				col.push_back(0);
			}
			traveled_grid.push_back(col);
		}
		for(int i = 0; i < l; i++){
			vector< vector< pair<int, int> > > col;
			for(int j = 0; j < h; j ++){
				vector< pair<int, int> > values;
				col.push_back(values);
			}
			highway_grid_connections.push_back(col);
		}
		highways = vector<Highway>();
		highways_complete = false;
		go_to_top_point = false;
		go_to_farthest_on_grid = false;
		too_close = false;
		// too_close_front = false;
		top_point_decisions = 0;
		decision_limit = decthreshold;
		start_rotations = 0;
	};
	~HighwayExplorer(){};

	bool getHighwaysComplete(){return highways_complete;}
	void setHighwaysComplete(double time){
		if(time >= time_threshold){
			highways_complete = true;
			if(time <= time_threshold+10){
				cout << "Highway grid" << endl;
				for(int i = 0; i < highway_grid[0].size(); i++){
					cout << "final_grid ";
					for(int j = 0; j < highway_grid.size(); j++){
						cout << highway_grid[j][i] << " ";
					}
					cout << endl;
				}
			}
		}
	}

	vector< vector<int> > getHighwayGrid(){return highway_grid;}

	// vector< vector< vector< pair<int, int> > > > getHighwayGridConnections(){return highway_grid_connections;}

	vector< vector<double> > getHighwayPath(){return path_to_top_point;}

	vector<Position> getHighwayStack(){
		vector<Position> hwst;
		for(int i = 0; i < highway_stack_longest.size(); i++){
			hwst.push_back(highway_stack_longest[i].point);
		}
		for(int i = 0; i < highway_stack.size(); i++){
			hwst.push_back(highway_stack[i].point);
		}
		return hwst;
	}

	vector< vector<Position> > getRemainingHighwayStack(){
		cout << "getRemainingHighwayStack " << highway_stack_longest.size() << " " << highway_stack.size() << endl;
		vector< vector<Position> > hwst;
		for(int i = 0; i < highway_stack_longest.size(); i++){
			if(pointAlreadyInStack(highway_stack_longest[i], true) == false){
				if(highway_stack_longest[i].direction == true){
					vector<Position> pair_points;
					pair_points.push_back(highway_stack_longest[i].point);
					pair_points.push_back(highway_stack_longest[i].right_point);
					hwst.push_back(pair_points);
				}
				else{
					vector<Position> pair_points;
					pair_points.push_back(highway_stack_longest[i].point);
					pair_points.push_back(highway_stack_longest[i].left_point);
					hwst.push_back(pair_points);
				}
			}
		}
		for(int i = 0; i < highway_stack.size(); i++){
			if(pointAlreadyInStack(highway_stack[i], true) == false){
				if(highway_stack[i].direction == true){
					vector<Position> pair_points;
					pair_points.push_back(highway_stack[i].point);
					pair_points.push_back(highway_stack[i].right_point);
					hwst.push_back(pair_points);
				}
				else{
					vector<Position> pair_points;
					pair_points.push_back(highway_stack[i].point);
					pair_points.push_back(highway_stack[i].left_point);
					hwst.push_back(pair_points);
				}
			}
		}
		cout << "hwst " << hwst.size() << endl;
		return hwst;
	}

	Position getHighwayTarget(){
		if(go_to_top_point == true){
			return top_point.point;
		}
		else{
			return current_target;
		}
	}

	int getLength(){return length;}
	int getHeight(){return height;}

	FORRAction exploreDecision(Position current_point, sensor_msgs::msg::LaserScan current_laser){
		DecisionPoint current_position = DecisionPoint(current_point, current_laser);
		position_history.push_back(current_position);
		cout << "current_position " << current_position.point.getX() << " " << current_position.point.getY() << " " << current_position.point.getTheta() << " mid avg " << current_position.middle_distance << " mid min " << current_position.middle_distance_min << " mid max " << current_position.farthest_distance_middle << " left avg " << current_position.left_distance << " left max " << current_position.farthest_distance_left << " right avg " << current_position.right_distance << " right max " << current_position.farthest_distance_right << endl;
		cout << "left_width " << current_position.left_width << " right_width " << current_position.right_width << endl;
		// cout << " left_width_max " << current_position.left_width_max << " right_width_max " << current_position.right_width_max << endl;
		// cout << "middle_point " << current_position.middle_point.getX() << " " << current_position.middle_point.getY() << " left_point " << current_position.left_point.getX() << " " << current_position.left_point.getY() << " right_point " << current_position.right_point.getX() << " " << current_position.right_point.getY() << endl;
		if(current_position.right_distance >= distance_threshold){
			DecisionPoint right_position = current_position;
			right_position.direction = true;
			if(pointAlreadyInStack(right_position) == false){
				// cout << "Adding to stack " << highway_stack.size() << " distance " << current_position.right_distance << " view " << current_position.farthest_view_right << endl;
				if(current_position.farthest_distance_right >= 2*distance_threshold){
					cout << "Adding to top of longest stack " << highway_stack_longest.size() << " farthest right distance " << current_position.farthest_distance_right << " x " << right_position.point.getX() << " y " << right_position.point.getY() << endl;
					highway_stack_longest.insert(highway_stack_longest.begin(), right_position);
					stack_longest_index.insert(stack_longest_index.begin(), position_history.size()-1);
				}
				else{
					cout << "Adding to top of shorter stack " << highway_stack.size() << " farthest right distance " << current_position.farthest_distance_right << " x " << right_position.point.getX() << " y " << right_position.point.getY() << endl;
					highway_stack.insert(highway_stack.begin(), right_position);
					stack_index.insert(stack_index.begin(), position_history.size()-1);
				}
			}
		}
		if(current_position.left_distance >= distance_threshold){
			DecisionPoint left_position = current_position;
			left_position.direction = false;
			if(pointAlreadyInStack(left_position) == false){
				// cout << "Adding to stack " << highway_stack.size() << " distance " << current_position.left_distance << " view " << current_position.farthest_view_left << endl;
				if(current_position.farthest_distance_left >= 2*distance_threshold){
					cout << "Adding to top of longest stack " << highway_stack_longest.size() << " farthest left distance " << current_position.farthest_distance_left << " x " << left_position.point.getX() << " y " << left_position.point.getY() << endl;
					highway_stack_longest.insert(highway_stack_longest.begin(), left_position);
					stack_longest_index.insert(stack_longest_index.begin(), position_history.size()-1);
				}
				else{
					cout << "Adding to top of shorter stack " << highway_stack.size() << " farthest left distance " << current_position.farthest_distance_left << " x " << left_position.point.getX() << " y " << left_position.point.getY() << endl;
					highway_stack.insert(highway_stack.begin(), left_position);
					stack_index.insert(stack_index.begin(), position_history.size()-1);
				}
			}
		}
		// If the beginning then spin 360 degrees to add to stack
		if(highways.size() == 0 or start_rotations < 72){
			last_position = current_position;
			// cout << "last_position " << last_position.point.getX() << " " << last_position.point.getY() << " " << last_position.point.getTheta() << " " << last_highway << endl;
			// if(highway_stack_longest.size() == 0 and highway_stack.size() == 0){
			if(start_rotations < 72){
				start_rotations = start_rotations + 1;
				return FORRAction(RIGHT_TURN, 1);
			}
			else if(highway_stack_longest.size() > 0 or highway_stack.size() > 0){
				if(highway_stack_longest.size() > 0){
					top_point = highway_stack_longest[0];
					top_point_index = stack_longest_index[0];
					// cout << "Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
					highway_stack_completed.push_back(top_point);
					highway_stack_longest.erase(highway_stack_longest.begin());
					stack_longest_index.erase(stack_longest_index.begin());
				}
				else{
					top_point = highway_stack[0];
					top_point_index = stack_index[0];
					// cout << "Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
					highway_stack_completed.push_back(top_point);
					highway_stack.erase(highway_stack.begin());
					stack_index.erase(stack_index.begin());
				}
				Highway new_highway = Highway(top_point);
				highways.push_back(new_highway);
				last_highway = 0;
				top_point_decisions = 0;
				if(top_point.direction == true){
					current_target = top_point.right_point;
					dist_travelled_so_far = 0;
					avg_left = 0;
					avg_right = 0;
					avg_count = 0;
					return goTowardsPoint(current_position, current_target);
				}
				else{
					current_target = top_point.left_point;
					dist_travelled_so_far = 0;
					avg_left = 0;
					avg_right = 0;
					avg_count = 0;
					return goTowardsPoint(current_position, current_target);
				}
			}
			else{
				highways_complete = true;
				return FORRAction(FORWARD, 0);
			}
		}
		// Once there are items on the stack, pop the top and start to go towards
		// cout << "last_position " << last_position.point.getX() << " " << last_position.point.getY() << " " << last_position.point.getTheta() << " " << last_highway << endl;
		cout << "current_target " << current_target.getX() << " " << current_target.getY() << endl;
		cout << "top_point_decisions " << top_point_decisions << endl;
		cout << "highway_stack_longest.size() " << highway_stack_longest.size() << " highway_stack.size() " << highway_stack.size() << endl;

		// Check if you can go further, what the width-to-length ratio is, how close the walls
		// dist_travelled_so_far += last_position.point.getDistance(current_position.point);
		dist_travelled_so_far = highways[last_highway].getStartPoint().point.getDistance(current_position.point);
		avg_left += current_position.farthest_distance_left;
		avg_right += current_position.farthest_distance_right;
		avg_count++;
		double length_width_ratio = (current_position.farthest_distance_middle + dist_travelled_so_far) / (avg_left/avg_count + avg_right/avg_count);
		double angle_to_avg_theta = 0;
		if(highways[last_highway].getSizeHighway() >= 40){
			angle_to_avg_theta = highways[last_highway].getAvgTheta() - current_position.point.getTheta();
			if(angle_to_avg_theta > M_PI)
				angle_to_avg_theta = angle_to_avg_theta - (2*M_PI);
			if(angle_to_avg_theta < -M_PI)
				angle_to_avg_theta = angle_to_avg_theta + (2*M_PI);
			angle_to_avg_theta = fabs(angle_to_avg_theta);
		}
		double goal_direction = atan2((current_target.getY() - current_position.point.getY()), (current_target.getX() - current_position.point.getX()));
		double angle_to_current_target = goal_direction - current_position.point.getTheta();
		if(angle_to_current_target > M_PI)
			angle_to_current_target = angle_to_current_target - (2*M_PI);
		if(angle_to_current_target < -M_PI)
			angle_to_current_target = angle_to_current_target + (2*M_PI);
		angle_to_current_target = fabs(angle_to_current_target);
		double middle_direction = atan2((current_position.middle_point.getY() - current_position.point.getY()), (current_position.middle_point.getX() - current_position.point.getX()));
		double angle_to_middle_point = middle_direction - current_position.point.getTheta();
		if(angle_to_middle_point > M_PI)
			angle_to_middle_point = angle_to_middle_point - (2*M_PI);
		if(angle_to_middle_point < -M_PI)
			angle_to_middle_point = angle_to_middle_point + (2*M_PI);
		angle_to_middle_point = fabs(angle_to_middle_point);
		cout << "dist_travelled_so_far " << dist_travelled_so_far << " avg_left " << avg_left << " avg_right " << avg_right << " avg_count " << avg_count << " length_width_ratio " << length_width_ratio << " angle_to_avg_theta " << angle_to_avg_theta << " angle_to_current_target " << angle_to_current_target << endl;
		if((current_position.farthest_distance_middle > last_position.middle_distance or current_position.middle_distance > 0.5) and angle_to_current_target < 0.3490658504 and angle_to_middle_point < 0.3490658504){
			// cout << "More space in front from current position" << endl;
			// Update current target
			current_target = current_position.middle_point;
			// cout << "New current_target " << current_target.getX() << " " << current_target.getY() << endl;
		}
		double dist_to_current_target = current_target.getDistance(current_position.point);

		if(go_to_farthest_on_grid == true and top_point.point.getDistance(current_position.point) <= 0.5){
			top_point_decisions = decision_limit;
			dist_travelled_so_far = 0;
			avg_left = 0;
			avg_right = 0;
			avg_count = 0;
			go_to_top_point = false;
			top_point_decisions = 0;
		}

		if((((length_width_ratio < 1 and dist_travelled_so_far > 1.5*current_position.farthest_distance_middle) or angle_to_avg_theta >= 1.5707963268/2.0 or dist_to_current_target <= 0.5 or current_position.middle_distance <= 0.1) and go_to_top_point == false) or top_point_decisions == decision_limit){
			cout << "Too wide compared to length " << length_width_ratio << " Distance travelled so far " << dist_travelled_so_far << endl;
			cout << "Turn too big " << angle_to_avg_theta << endl;
			cout << "Decision limit reached " << top_point_decisions << endl;
			cout << "Reached current target " << dist_to_current_target << endl;
			cout << "Too close in front " << current_position.middle_distance << endl;
			if(length_width_ratio < 1 and dist_travelled_so_far > 1.5*current_position.farthest_distance_middle){
				cout << "PassageExplorationEnded length_width_ratio" << endl;
			}
			else if(angle_to_avg_theta >= 1.5707963268/2.0){
				cout << "PassageExplorationEnded angle_to_avg_theta" << endl;
			}
			else if(dist_to_current_target <= 0.5){
				cout << "PassageExplorationEnded dist_to_current_target" << endl;
			}
			else if(current_position.middle_distance <= 0.1){
				cout << "PassageExplorationEnded middle_distance" << endl;
			}
			else if(top_point_decisions == decision_limit){
				cout << "PassageExplorationEnded top_point_decisions" << endl;
			}
			// Stop current point and go to next on stack
			// cout << "Highway grid" << endl;
			for(int i = 0; i < highway_grid[0].size(); i++){
				for(int j = 0; j < highway_grid.size(); j++){
					if(i == (int)(current_position.point.getY()) and j == (int)(current_position.point.getX())){
						cout << "[" << highway_grid[j][i] << "] "; 
					}
					else{
						cout << highway_grid[j][i] << " ";
					}
				}
				cout << endl;
			}
			dist_travelled_so_far = 0;
			avg_left = 0;
			avg_right = 0;
			avg_count = 0;
			// After finishing point on stack, pop next one
			if(highway_stack_longest.size() > 0 or highway_stack.size() > 0){
				// cout << "Going to top point on stack" << endl;
				int start_highway = 0;
				int end_highway = 0;
				int middle_highway = 0;
				bool already_completed = true;
				// bool already_sorted = false;
				while((((start_highway >= 0 and end_highway >= 0) or (start_highway >= 0 and middle_highway >= 0) or (middle_highway >= 0 and end_highway >= 0)) or already_completed) and (highway_stack_longest.size() > 0 or highway_stack.size() > 0)){
					// if(already_sorted == false and (highway_stack[0].direction == true and highway_stack[0].right_distance < 2*distance_threshold) or (highway_stack[0].direction == false and highway_stack[0].left_distance < 2*distance_threshold)){
					// 	sort(highway_stack.begin(), highway_stack.end());
					// 	reverse(highway_stack.begin(), highway_stack.end());
					// 	already_sorted = true;
					// }
					if(highway_stack_longest.size() > 0){
						top_point = highway_stack_longest[0];
						top_point_index = stack_longest_index[0];
						// cout << "Potential Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
						highway_stack_longest.erase(highway_stack_longest.begin());
						stack_longest_index.erase(stack_longest_index.begin());
					}
					else{
						top_point = highway_stack[0];
						top_point_index = stack_index[0];
						// cout << "Potential Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
						highway_stack.erase(highway_stack.begin());
						stack_index.erase(stack_index.begin());
					}
					start_highway = highway_grid[(int)(top_point.point.getX())][(int)(top_point.point.getY())];
					if(top_point.direction == true){
						end_highway = highway_grid[(int)(top_point.right_point.getX())][(int)(top_point.right_point.getY())];
						middle_highway = highway_grid[(int)((top_point.point.getX() + top_point.right_point.getX())/2.0)][(int)((top_point.point.getY() + top_point.right_point.getY())/2.0)];
					}
					else{
						end_highway = highway_grid[(int)(top_point.left_point.getX())][(int)(top_point.left_point.getY())];
						middle_highway = highway_grid[(int)((top_point.point.getX() + top_point.left_point.getX())/2.0)][(int)((top_point.point.getY() + top_point.left_point.getY())/2.0)];
					}
					if(middle_highway == -2 and end_highway == -2){
						middle_highway = 0;
						end_highway = 0;
					}
					already_completed = anySimilarOnList(top_point, highway_stack_completed);
					highway_stack_completed.push_back(top_point);
					// cout << "start_highway " << start_highway  << " middle_highway " << middle_highway << " end_highway " << end_highway << endl;
				}
				// cout << "Final Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
				Highway new_highway = Highway(top_point);
				highways.push_back(new_highway);
				last_highway = last_highway + 1;
				top_point_decisions = 0;
				go_to_top_point = true;
				path_to_top_point.clear();
				go_to_farthest_on_grid = false;
			}
			// else if(go_to_farthest_on_grid == false){
			// 	double dist_to_farthest = 0;
			// 	for(int i = 0; i < highway_grid.size(); i++){
			// 		for(int j = 0; j < highway_grid[i].size(); j++){
			// 			if(highway_grid[i][j] >= 0){
			// 				double dist_to_grid = current_position.point.getDistance(Position(i,j,0));
			// 				if(dist_to_grid > dist_to_farthest){
			// 					top_point = highways[highway_grid[i][j]].getClosestPointOnHighway(Position(i,j,0));
			// 					top_point_index = -1;
			// 					dist_to_farthest = dist_to_grid;
			// 				}
			// 			}
			// 		}
			// 	}
			// 	top_point_decisions = 0;
			// 	go_to_top_point = true;
			// 	path_to_top_point.clear();
			// 	go_to_farthest_on_grid = true;
			// }
			else{
				// cout << "No more in stack" << endl;
				if(highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] >= 0){
					highways[highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())]].addPointToHighway(current_position);
				}
				last_position = current_position;
				go_to_top_point = false;
				highways_complete = true;
				return FORRAction(FORWARD, 0);
			}
		}

		if(current_position.left_distance <= 0.2 or current_position.right_distance <= 0.2 or too_close == true){
			cout << "Too close on sides, turn away" << endl;
			// Move away from sides
			if(highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] >= 0){
				highways[highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())]].addPointToHighway(current_position);
			}
			if(too_close == true){
				if(last_position.left_distance <= 0.2){
					// last_position = current_position;
					top_point_decisions++;
					// return FORRAction(RIGHT_TURN, 2);
					too_close = false;
					return goTowardsPoint(current_position, last_position.right_point);
				}
				else if(last_position.right_distance <= 0.2){
					// last_position = current_position;
					top_point_decisions++;
					// return FORRAction(LEFT_TURN, 2);
					too_close = false;
					return goTowardsPoint(current_position, last_position.left_point);
				}
			}
			else if(current_position.left_distance <= 0.2){
				// last_position = current_position;
				top_point_decisions++;
				// return FORRAction(RIGHT_TURN, 2);
				too_close = true;
				return goTowardsPoint(current_position, current_position.right_point);
			}
			else if(current_position.right_distance <= 0.2){
				// last_position = current_position;
				top_point_decisions++;
				// return FORRAction(LEFT_TURN, 2);
				too_close = true;
				return goTowardsPoint(current_position, current_position.left_point);
			}
		}
		else if(go_to_top_point == true){
			// cout << "Go to top point" << endl;
			if(highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] >= 0){
				highways[highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())]].addPointToHighway(current_position);
			}
			cout << "Top point " << top_point.point.getX() << " " << top_point.point.getY() << endl;
			double dist_to_top_point = top_point.point.getDistance(current_position.point);
			bool can_access_top_point = canAccessPoint(current_position.laserEndpoints, CartesianPoint(current_position.point.getX(), current_position.point.getY()), CartesianPoint(top_point.point.getX(), top_point.point.getY()), 5);
			// cout << "Distance to top point " << dist_to_top_point << " current theta " << current_position.point.getTheta() << " top point angles " << top_point.farthest_angle_left << " " << top_point.farthest_angle_right << endl;
			if(dist_to_top_point <= 0.5){
				cout << "Top point achieved, turn towards stretch" << endl;
				dist_travelled_so_far = 0;
				avg_left = 0;
				avg_right = 0;
				avg_count = 0;
				go_to_top_point = false;
				top_point_decisions = 0;
				if(top_point.direction == true){
					current_target = top_point.right_point;
					return goTowardsPoint(current_position, current_target);
				}
				else{
					current_target = top_point.left_point;
					return goTowardsPoint(current_position, current_target);
				}
			}
			else if(dist_to_top_point <= 1 and can_access_top_point){
				cout << "Top point close, go towards" << endl;
				top_point_decisions++;
				return goTowardsPoint(current_position, top_point.point);
			}
			else if(path_to_top_point.size() > 0){
				cout << "Top point not in range, go to top point by following path" << endl;
				waypointAchieved(current_position);
				if(path_to_top_point.size() == 0){
					last_position = current_position;
					top_point_decisions++;
					return FORRAction(FORWARD, 0);
				}
				else{
					bool can_access_waypoint = canAccessPoint(current_position.laserEndpoints, CartesianPoint(current_position.point.getX(), current_position.point.getY()), CartesianPoint(path_to_top_point[0][0], path_to_top_point[0][1]), 5);
					if(can_access_waypoint){
						cout << "Can Access Current waypoint " << path_to_top_point[0][0] << " " << path_to_top_point[0][1] << endl;
						top_point_decisions++;
						return goTowardsPoint(current_position, Position(path_to_top_point[0][0], path_to_top_point[0][1], 0));
					}
					else{
						int num = -1;
						while(!can_access_waypoint and num < path_to_top_point.size()){
							num = num + 1;
							can_access_waypoint = canAccessPoint(current_position.laserEndpoints, CartesianPoint(current_position.point.getX(), current_position.point.getY()), CartesianPoint(path_to_top_point[num][0], path_to_top_point[num][1]), 5);
						}
						cout << "num " << num << " can_access_waypoint " << can_access_waypoint << endl;
						if(can_access_waypoint){
							cout << "New Access waypoint " << path_to_top_point[num][0] << " " << path_to_top_point[num][1] << endl;
							top_point_decisions++;
							return goTowardsPoint(current_position, Position(path_to_top_point[num][0], path_to_top_point[num][1], 0));
						}
						// else if(top_point_decisions % 100 == 0 and path_to_top_point.size() >= 2){
						// 	// cout << "Erase current waypoint and assign next one" << endl;
						// 	path_to_top_point.erase(path_to_top_point.begin());
						// 	// cout << "New waypoint " << path_to_top_point[0][0] << " " << path_to_top_point[0][1] << endl;
						// 	top_point_decisions++;
						// 	return goTowardsPoint(current_position, Position(path_to_top_point[0][0], path_to_top_point[0][1], 0));
						// }
						else{
							cout << "Try Current waypoint " << path_to_top_point[0][0] << " " << path_to_top_point[0][1] << endl;
							top_point_decisions++;
							return goTowardsPoint(current_position, Position(path_to_top_point[0][0], path_to_top_point[0][1], 0));
						}
					}
					// else if(top_point_decisions % 100 == 0 and path_to_top_point.size() >= 2){
					// 	// cout << "Erase current waypoint and assign next one" << endl;
					// 	path_to_top_point.erase(path_to_top_point.begin());
					// 	// cout << "New waypoint " << path_to_top_point[0][0] << " " << path_to_top_point[0][1] << endl;
					// 	top_point_decisions++;
					// 	return goTowardsPoint(current_position, Position(path_to_top_point[0][0], path_to_top_point[0][1], 0));
					// }
					// else{
					// 	// cout << "Current waypoint " << path_to_top_point[0][0] << " " << path_to_top_point[0][1] << endl;
					// 	top_point_decisions++;
					// 	return goTowardsPoint(current_position, Position(path_to_top_point[0][0], path_to_top_point[0][1], 0));
					// }
				}
			}
			else{
				findPathOnGrid(current_position, top_point, top_point_index);
				// cout << "Current waypoint " << path_to_top_point[0][0] << " " << path_to_top_point[0][1] << endl;
				top_point_decisions++;
				return goTowardsPoint(current_position, Position(path_to_top_point[0][0], path_to_top_point[0][1], 0));
			}
		}
		else{
			// cout << "Going to current_target " << current_target.getX() << " " << current_target.getY() << endl;
			highways[last_highway].addPointToHighway(current_position);
			// cout << "Current grid value " << highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] << endl;
			if(highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] == -1){
				highway_grid[(int)(current_position.point.getX())][(int)(current_position.point.getY())] = last_highway;
				double multiplier = 10;
				vector< vector<int> > passed_grid;
				for(int i = 0; i < length*multiplier; i++){
					vector<int> col;
					for(int j = 0; j < height*multiplier; j ++){
						col.push_back(0);
					}
					passed_grid.push_back(col);
				}
				vector< vector<int> > hit_grid;
				for(int i = 0; i < length*multiplier; i++){
					vector<int> col;
					for(int j = 0; j < height*multiplier; j ++){
						col.push_back(0);
					}
					hit_grid.push_back(col);
				}
				int startx = (int)(current_position.point.getX()*multiplier);
				int starty = (int)(current_position.point.getY()*multiplier);
				int rightx = (int)(current_position.right_point.getX()*multiplier);
				int righty = (int)(current_position.right_point.getY()*multiplier);
				int leftx = (int)(current_position.left_point.getX()*multiplier);
				int lefty = (int)(current_position.left_point.getY()*multiplier);
				double righta = starty - righty;
				double rightb = rightx - startx;
				double rightc = (startx - rightx) * starty + (righty - starty) * startx;
				double lefta = starty - lefty;
				double leftb = leftx - startx;
				double leftc = (startx - leftx) * starty + (lefty - starty) * startx;
				if(startx > rightx){
					for(int i = startx; i > rightx; i--){
						passed_grid[i][-(righta * i + rightc) / rightb] = passed_grid[i][-(righta * i + rightc) / rightb] + 1;
					}
				}
				else if(startx < rightx){
					for(int i = startx; i < rightx; i++){
						passed_grid[i][-(righta * i + rightc) / rightb] = passed_grid[i][-(righta * i + rightc) / rightb] + 1;
					}
				}
				else{
					if(starty > righty){
						for(int i = starty; i > righty; i--){
							passed_grid[-(rightb * i + rightc) / righta][i] = passed_grid[-(rightb * i + rightc) / righta][i] + 1;
						}
					}
					else if(starty < righty){
						for(int i = starty; i < righty; i++){
							passed_grid[-(rightb * i + rightc) / righta][i] = passed_grid[-(rightb * i + rightc) / righta][i] + 1;
						}
					}
					else{
						passed_grid[startx][starty] = passed_grid[startx][starty] + 1;
					}
				}
				if(startx > leftx){
					for(int i = startx; i > leftx; i--){
						passed_grid[i][-(lefta * i + leftc) / leftb] = passed_grid[i][-(lefta * i + leftc) / leftb] + 1;
					}
				}
				else if(startx < leftx){
					for(int i = startx; i < leftx; i++){
						passed_grid[i][-(lefta * i + leftc) / leftb] = passed_grid[i][-(lefta * i + leftc) / leftb] + 1;
					}
				}
				else{
					if(starty > lefty){
						for(int i = starty; i > lefty; i--){
							passed_grid[-(leftb * i + leftc) / lefta][i] = passed_grid[-(leftb * i + leftc) / lefta][i] + 1;
						}
					}
					else if(starty < lefty){
						for(int i = starty; i < lefty; i++){
							passed_grid[-(leftb * i + leftc) / lefta][i] = passed_grid[-(leftb * i + leftc) / lefta][i] + 1;
						}
					}
					else{
						passed_grid[startx][starty] = passed_grid[startx][starty] + 1;
					}
				}
				hit_grid[rightx][righty] = 1;
				hit_grid[leftx][lefty] = 1;
				vector< vector<double> > ratio_grid;
				for(int i = 0; i < length*multiplier; i++){
					vector<double> col;
					for(int j = 0; j < height*multiplier; j ++){
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
							ratio_grid[i][j] = -1;
						}
					}
				}
				vector< vector<double> > count_grid;
				for(int i = 0; i < length; i++){
					vector<double> col;
					for(int j = 0; j < height; j ++){
						col.push_back(0.0);
					}
					count_grid.push_back(col);
				}
				vector< vector<double> > ratio_sum_grid;
				for(int i = 0; i < length; i++){
					vector<double> col;
					for(int j = 0; j < height; j ++){
						col.push_back(0.0);
					}
					ratio_sum_grid.push_back(col);
				}
				vector< vector<double> > distance_sum_grid;
				for(int i = 0; i < length; i++){
					vector<double> col;
					for(int j = 0; j < height; j ++){
						col.push_back(0.0);
					}
					distance_sum_grid.push_back(col);
				}
				for(int i = 0; i < ratio_grid.size(); i++){
					for(int j = 0; j < ratio_grid[i].size(); j++){
						if(ratio_grid[i][j] >= 0){
							int indx = (int)((double)(i)/multiplier);
							int indy = (int)((double)(j)/multiplier);
							count_grid[indx][indy] = count_grid[indx][indy] + 1;
							ratio_sum_grid[indx][indy] = ratio_sum_grid[indx][indy] + ratio_grid[i][j];
							distance_sum_grid[indx][indy] = distance_sum_grid[indx][indy] + sqrt((double)((i-startx)*(i-startx)+(j-starty)*(j-starty)))/multiplier;
						}
					}
				}
				vector< vector<double> > ratio_avg_grid;
				for(int i = 0; i < length; i++){
					vector<double> col;
					for(int j = 0; j < height; j ++){
						col.push_back(0.0);
					}
					ratio_avg_grid.push_back(col);
				}
				vector< vector<double> > distance_avg_grid;
				for(int i = 0; i < length; i++){
					vector<double> col;
					for(int j = 0; j < height; j ++){
						col.push_back(0.0);
					}
					distance_avg_grid.push_back(col);
				}
				for(int i = 0; i < count_grid.size(); i++){
					for(int j = 0; j < count_grid[i].size(); j++){
						if(count_grid[i][j] > 0){
							ratio_avg_grid[i][j] = ratio_sum_grid[i][j] / count_grid[i][j];
							distance_avg_grid[i][j] = distance_sum_grid[i][j] / count_grid[i][j];
						}
						else{
							ratio_avg_grid[i][j] = -1;
							distance_avg_grid[i][j] = -1;
						}
					}
				}
				for(int i = 0; i < ratio_avg_grid.size(); i++){
					for(int j = 0; j < ratio_avg_grid[i].size(); j++){
						if(distance_avg_grid[i][j] <= 2 and ratio_avg_grid[i][j] <= 0.5 and distance_avg_grid[i][j] >= 0 and ratio_avg_grid[i][j] >= 0){
							highway_grid[i][j] = last_highway;
							pair<int, int> connection_ab (i, j);
							highway_grid_connections[(int)(current_position.point.getX())][(int)(current_position.point.getY())].push_back(connection_ab);
							pair<int, int> connection_ba ((int)(current_position.point.getX()), (int)(current_position.point.getY()));
							highway_grid_connections[i][j].push_back(connection_ba);
						}
						else if(distance_avg_grid[i][j] <= 4 and ratio_avg_grid[i][j] > 0.5 and distance_avg_grid[i][j] >= 0 and ratio_avg_grid[i][j] >= 0 and highway_grid[i][j] < 0){
							highway_grid[i][j] = -2;
						}
					}
				}
				// if(current_position.left_distance >= 0.1){
				// 	double step_size = 0.01;
				// 	double tx,ty;
				// 	vector < vector <int> > neighbor_grids;
				// 	vector <double> neighbor_distances;
				// 	for(double step = 0; step <= 1; step += step_size){
				// 		tx = (current_position.left_point.getX() * step) + (current_position.point.getX() * (1-step));
				// 		ty = (current_position.left_point.getY() * step) + (current_position.point.getY() * (1-step));
				// 		double dist_to_current_from_t = current_position.point.getDistance(Position(tx, ty, 0));
				// 		vector<double> neighbor;
				// 		neighbor.push_back((int)(tx));
				// 		neighbor.push_back((int)(ty));
				// 		if(find(neighbor_grids.begin(), neighbor_grids.end(), neighbor) != neighbor_grids.end()){
				// 			continue;
				// 		}
				// 		else{
				// 			neighbor_grids.push_back(neighbor);
				// 			neighbor_distances.push_back(dist_to_current_from_t);
				// 		}
				// 	}
				// 	if(neighbor_grids.size() == 1){
				// 		continue;
				// 	}
				// 	else if(neighbor_grids.size() == 2){
				// 		if(highway_grid[neighbor_grids[1][0]][neighbor_grids[1][1]] == -1){
				// 			highway_grid[neighbor_grids[1][0]][neighbor_grids[1][1]] = -2;
				// 		}
				// 	}

				// 	for(int i = 1; i < neighbor_grids.size()-1; i++){
				// 		if(neighbor_grids[i][0] != neighbor_grids[i-1][0] and neighbor_grids[i][1] != neighbor_grids[i-1][1] and neighbor_distances[1] <= 2 and neighbor_distances[1] <= current_position.left_distance and highway_grid[neighbor_grids[i][0]][neighbor_grids[i][1]] == -1){
				// 			highway_grid[neighbor_grids[i][0]][neighbor_grids[i][1]] = last_highway;
				// 			pair<int, int> connection_ab (neighbor_grids[i][0], neighbor_grids[i][1]);
				// 			highway_grid_connections[(int)(current_position.point.getX())][(int)(current_position.point.getY())].push_back(connection_ab);
				// 			pair<int, int> connection_ba ((int)(current_position.point.getX()), (int)(current_position.point.getY()));
				// 			highway_grid_connections[neighbor_grids[i][0]][neighbor_grids[i][1]].push_back(connection_ba);
				// 		}
				// 	}
				// 	// double step_size = 0.1;
				// 	// double tx,ty;
				// 	// for(double step = 0; step <= 1; step += step_size){
				// 	// 	tx = (current_position.left_point.getX() * step) + (current_position.point.getX() * (1-step));
				// 	// 	ty = (current_position.left_point.getY() * step) + (current_position.point.getY() * (1-step));
				// 	// 	if(current_position.point.getDistance(Position(tx, ty, 0)) <= 2 and current_position.point.getDistance(Position(tx, ty, 0)) <= current_position.left_distance){
				// 	// 		if(highway_grid[(int)(tx)][(int)(ty)] == -1){
				// 	// 			highway_grid[(int)(tx)][(int)(ty)] = last_highway;
				// 	// 			pair<int, int> connection_ab ((int)(tx), (int)(ty));
				// 	// 			highway_grid_connections[(int)(current_position.point.getX())][(int)(current_position.point.getY())].push_back(connection_ab);
				// 	// 			pair<int, int> connection_ba ((int)(current_position.point.getX()), (int)(current_position.point.getY()));
				// 	// 			highway_grid_connections[(int)(tx)][(int)(ty)].push_back(connection_ba);
				// 	// 		}
				// 	// 	}
				// 	// }
				// }
				// if(current_position.right_distance >= 2){
				// 	double step_size = 0.1;
				// 	double tx,ty;
				// 	for(double step = 0; step <= 1; step += step_size){
				// 		tx = (current_position.right_point.getX() * step) + (current_position.point.getX() * (1-step));
				// 		ty = (current_position.right_point.getY() * step) + (current_position.point.getY() * (1-step));
				// 		if(current_position.point.getDistance(Position(tx, ty, 0)) <= 2 and current_position.point.getDistance(Position(tx, ty, 0)) <= current_position.right_distance){
				// 			if(highway_grid[(int)(tx)][(int)(ty)] == -1){
				// 				highway_grid[(int)(tx)][(int)(ty)] = last_highway;
				// 				pair<int, int> connection_ab ((int)(tx), (int)(ty));
				// 				highway_grid_connections[(int)(current_position.point.getX())][(int)(current_position.point.getY())].push_back(connection_ab);
				// 				pair<int, int> connection_ba ((int)(current_position.point.getX()), (int)(current_position.point.getY()));
				// 				highway_grid_connections[(int)(tx)][(int)(ty)].push_back(connection_ba);
				// 			}
				// 		}
				// 	}
				// }
			}
			if((int)(current_position.point.getX()) != (int)(last_position.point.getX()) or (int)(current_position.point.getY()) != (int)(last_position.point.getY())){
				pair<int, int> connection_ab ((int)(last_position.point.getX()), (int)(last_position.point.getY()));
				std::vector< pair<int, int> >::const_iterator it;
				it = find(highway_grid_connections[(int)(current_position.point.getX())][(int)(current_position.point.getY())].begin(), highway_grid_connections[(int)(current_position.point.getX())][(int)(current_position.point.getY())].end(), connection_ab);
				if(it == highway_grid_connections[(int)(current_position.point.getX())][(int)(current_position.point.getY())].end()){
					highway_grid_connections[(int)(current_position.point.getX())][(int)(current_position.point.getY())].push_back(connection_ab);
				}
				pair<int, int> connection_ba ((int)(current_position.point.getX()), (int)(current_position.point.getY()));
				it = find(highway_grid_connections[(int)(last_position.point.getX())][(int)(last_position.point.getY())].begin(), highway_grid_connections[(int)(last_position.point.getX())][(int)(last_position.point.getY())].end(), connection_ba);
				if(it == highway_grid_connections[(int)(last_position.point.getX())][(int)(last_position.point.getY())].end()){
					highway_grid_connections[(int)(last_position.point.getX())][(int)(last_position.point.getY())].push_back(connection_ba);
				}
			}
			return goTowardsPoint(current_position, current_target);
		}
	}

	void findPathOnGrid(DecisionPoint current_point, DecisionPoint target_point, int target_point_index){
		path_to_top_point.clear();
		cout << "current_point " << current_point.point.getX() << " " << current_point.point.getY() << " target_point " << target_point.point.getX() << " " << target_point.point.getY() << " target_point_index " << target_point_index << endl;
		if(target_point_index == -1){
			double dist_to_top = 50;
			for(int i = 0; i < position_history.size(); i++){
				dist_to_top = position_history[i].point.getDistance(target_point.point);
				if(position_history[i] == target_point){
					target_point_index = i;
					break;
				}
				else if(dist_to_top < 0.25){
					target_point_index = i;
					break;
				}
			}
			cout << "new target_point_index " << target_point_index << endl;
		}
		vector<DecisionPoint> trailPositions;
		trailPositions.push_back(position_history[target_point_index]);
		for(int i = target_point_index; i < position_history.size(); i++){
			for(int n = position_history.size()-1; n > i; n--){
				if(canAccessPoint(position_history[i].laserEndpoints, CartesianPoint(position_history[i].point.getX(), position_history[i].point.getY()), CartesianPoint(position_history[n].point.getX(), position_history[n].point.getY()), 2)) {
					trailPositions.push_back(position_history[n]);
					i = n-1;
				}
			}
		}
		trailPositions.push_back(position_history[position_history.size()-1]);
		for(int i = trailPositions.size()-1; i >= 0; i--){
			vector<double> marker;
			marker.push_back(trailPositions[i].point.getX());
			marker.push_back(trailPositions[i].point.getY());
			path_to_top_point.push_back(marker);
		}

		// int current_x = (int)(current_point.point.getX());
		// int current_y = (int)(current_point.point.getY());
		// int target_x = (int)(target_point.point.getX());
		// int target_y = (int)(target_point.point.getY());
		// path_to_top_point.clear();
		// // cout << "Current point coordinates in grid " << current_x << " " << current_y << " target_point " << target_x << " " << target_y << endl;
		// // cout << "Current highway " << highway_grid[current_x][current_y] << " Target highway " << highway_grid[target_x][target_y] << endl;
		// if(current_x == target_x and current_y == target_y){
		// 	vector<double> current;
		// 	current.push_back(target_point.point.getX());
		// 	current.push_back(target_point.point.getY());
		// 	path_to_top_point.push_back(current);
		// 	return;
		// }
		// traveled_grid.clear();
		// for(int i = 0; i < length; i++){
		// 	vector<int> col;
		// 	for(int j = 0; j < height; j ++){
		// 		col.push_back(0);
		// 	}
		// 	traveled_grid.push_back(col);
		// }
		// if(highway_grid[current_x][current_y] < 0){
		// 	double min_dist = 10000000;
		// 	DecisionPoint closest_to_current = current_point;
		// 	for(int i = 0; i < highways.size(); i++){
		// 		DecisionPoint close_point = highways[i].getClosestPointOnHighway(current_point.point);
		// 		if(close_point.point.getDistance(current_point.point) < min_dist and highway_grid[(int)(close_point.point.getX())][(int)(close_point.point.getY())] >= 0 and canAccessPoint(current_point.laserEndpoints, CartesianPoint(current_point.point.getX(), current_point.point.getY()), CartesianPoint(target_point.point.getX(), target_point.point.getY()), 5)){
		// 			min_dist = close_point.point.getDistance(current_point.point);
		// 			closest_to_current = close_point;
		// 		}
		// 	}
		// 	// cout << "Closest to current " << (int)(closest_to_current.point.getX()) << " " << (int)(closest_to_current.point.getY()) << endl;
		// 	current_x = (int)(closest_to_current.point.getX());
		// 	current_y = (int)(closest_to_current.point.getY());
		// }
		// if(highway_grid[target_x][target_y] < 0){
		// 	double min_dist = 10000000;
		// 	DecisionPoint closest_to_current = target_point;
		// 	for(int i = 0; i < highways.size(); i++){
		// 		DecisionPoint close_point = highways[i].getClosestPointOnHighway(target_point.point);
		// 		if(close_point.point.getDistance(target_point.point) < min_dist and highway_grid[(int)(close_point.point.getX())][(int)(close_point.point.getY())] >= 0){
		// 			min_dist = close_point.point.getDistance(target_point.point);
		// 			closest_to_current = close_point;
		// 		}
		// 	}
		// 	// cout << "Closest to target " << (int)(closest_to_current.point.getX()) << " " << (int)(closest_to_current.point.getY()) << endl;
		// 	target_x = (int)(closest_to_current.point.getX());
		// 	target_y = (int)(closest_to_current.point.getY());
		// }
		// traveled_grid[current_x][current_y] = 1;
		// vector< vector<int> > added_points;
		// vector< vector<int> > previous_cell;
		// for(int i = 0; i < highway_grid_connections[current_x][current_y].size(); i++){
		// 	int c_x = highway_grid_connections[current_x][current_y][i].first;
		// 	int c_y = highway_grid_connections[current_x][current_y][i].second;
		// 	if(traveled_grid[c_x][c_y] != 1){
		// 		traveled_grid[c_x][c_y] = 1;
		// 		vector<int> new_id;
		// 		new_id.push_back(c_x);
		// 		new_id.push_back(c_y);
		// 		added_points.push_back(new_id);
		// 		new_id.push_back(current_x);
		// 		new_id.push_back(current_y);
		// 		previous_cell.push_back(new_id);
		// 		// cout << "Neighbor " << c_x << " " << c_y << " Previous " << current_x << " " << current_y << endl;
		// 	}
		// }
		// // cout << "Added points " << added_points.size() << endl;
		// while(added_points.size() > 0){
		// 	int new_x = added_points[0][0];
		// 	int new_y = added_points[0][1];
		// 	if(new_x == target_x and new_y == target_y){
		// 		// cout << "Found target point" << endl;
		// 		break;
		// 	}
		// 	added_points.erase(added_points.begin());
		// 	for(int i = 0; i < highway_grid_connections[new_x][new_y].size(); i++){
		// 		int c_x = highway_grid_connections[new_x][new_y][i].first;
		// 		int c_y = highway_grid_connections[new_x][new_y][i].second;
		// 		if(traveled_grid[c_x][c_y] != 1){
		// 			traveled_grid[c_x][c_y] = 1;
		// 			vector<int> new_id;
		// 			new_id.push_back(c_x);
		// 			new_id.push_back(c_y);
		// 			added_points.push_back(new_id);
		// 			new_id.push_back(new_x);
		// 			new_id.push_back(new_y);
		// 			previous_cell.push_back(new_id);
		// 			// cout << "Neighbor " << c_x << " " << c_y << " Previous " << new_x << " " << new_y << endl;
		// 		}
		// 	}
		// 	// cout << "Added points " << added_points.size() << endl;
		// }
		// // cout << "Previous cell " << previous_cell.size() << endl;
		// vector< vector<int> > path;
		// if(traveled_grid[target_x][target_y] == 1){
		// 	vector<int> target;
		// 	target.push_back(target_x);
		// 	target.push_back(target_y);
		// 	path.insert(path.begin(), target);
		// 	// cout << "Path " << path.size() << " target " << target_x << " " << target_y << endl;
		// 	bool found = true;
		// 	while(found){
		// 		int point_x = path[0][0];
		// 		int point_y = path[0][1];
		// 		found = false;
		// 		// cout << "Path " << path.size() << " point " << point_x << " " << point_y << endl; 
		// 		for(int i = 0; i < previous_cell.size(); i++){
		// 			if(previous_cell[i][0] == point_x and previous_cell[i][1] == point_y){
		// 				// cout << "Match " << previous_cell[i][0] << " " << previous_cell[i][1] << endl;
		// 				vector<int> new_target;
		// 				new_target.push_back(previous_cell[i][2]);
		// 				new_target.push_back(previous_cell[i][3]);
		// 				path.insert(path.begin(), new_target);
		// 				// cout << "Previous " << previous_cell[i][2] << " " << previous_cell[i][3] << endl;
		// 				found = true;
		// 				break;
		// 			}
		// 		}
		// 		// cout << path[0][0] << " " << current_x << " " << path[0][1] << " " << current_y << endl;
		// 		if(path[0][0] == current_x and path[0][1] == current_y){
		// 			break;
		// 		}
		// 	}
		// 	vector< vector<double> > path_from_highway;
		// 	// cout << "Path " << path.size() << endl;
		// 	for(int i = 0; i < path.size(); i++){
		// 		// cout << path[i][0] << " " << path[i][1] << " " << highway_grid[path[i][0]][path[i][1]] << endl;
		// 		if(highway_grid[path[i][0]][path[i][1]] >= 0){
		// 			if(i > 0){
		// 				DecisionPoint closest_prev = highways[highway_grid[path[i][0]][path[i][1]]].getClosestPointOnHighway(Position(path[i-1][0], path[i-1][1], 0));
		// 				// cout << closest_prev.point.getX() << " " << closest_prev.point.getY() << endl;
		// 				vector<double> closest_point_prev;
		// 				closest_point_prev.push_back(closest_prev.point.getX());
		// 				closest_point_prev.push_back(closest_prev.point.getY());
		// 				path_from_highway.push_back(closest_point_prev);
		// 			}
		// 			DecisionPoint closest = highways[highway_grid[path[i][0]][path[i][1]]].getClosestPointOnHighway(Position(path[i][0], path[i][1], 0));
		// 			// cout << closest.point.getX() << " " << closest.point.getY() << endl;
		// 			vector<double> closest_point;
		// 			closest_point.push_back(closest.point.getX());
		// 			closest_point.push_back(closest.point.getY());
		// 			path_from_highway.push_back(closest_point);
		// 		}
		// 	}
		// 	vector<double> current;
		// 	current.push_back(target_point.point.getX());
		// 	current.push_back(target_point.point.getY());
		// 	path_from_highway.push_back(current);
		// 	// path_to_top_point = path;
		// 	path_to_top_point = path_from_highway;
		// }
		// else{
		// 	vector< vector<double> > path_from_highway;
		// 	vector<double> current;
		// 	current.push_back(target_point.point.getX());
		// 	current.push_back(target_point.point.getY());
		// 	path_from_highway.push_back(current);
		// 	// path_to_top_point = path;
		// 	path_to_top_point = path_from_highway;
		// }
	}

	void waypointAchieved(DecisionPoint current_position){
		cout << "In waypointAchieved" << endl;
		bool erase_waypoint = true;
		while(erase_waypoint){
			Position current_waypoint = Position(path_to_top_point[0][0], path_to_top_point[0][1], 0);
			double dist_to_current_waypoint = current_position.point.getDistance(current_waypoint);
			if(dist_to_current_waypoint <= 0.25){
				cout << "Waypoint Achieved, removing waypoint" << endl;
				path_to_top_point.erase(path_to_top_point.begin());
			}
			else{
				erase_waypoint = false;
			}
		}
	}

	FORRAction goTowardsPoint(DecisionPoint current_position, Position target_position){
		// cout << "In goTowardsPoint" << endl;
		double distance_from_target = current_position.point.getDistance(target_position);
		// cout << "Distance from target : " << distance_from_target << endl;

		// compute the angular difference between the direction to the target and the current robot direction
		double robot_direction = current_position.point.getTheta();
		double goal_direction = atan2((target_position.getY() - current_position.point.getY()), (target_position.getX() - current_position.point.getX()));
		double required_rotation = goal_direction - robot_direction;

		if(required_rotation > M_PI)
			required_rotation = required_rotation - (2*M_PI);
		if(required_rotation < -M_PI)
			required_rotation = required_rotation + (2*M_PI);
		// cout << "Robot direction : " << robot_direction << ", Goal Direction : " << goal_direction << ", Required rotation : " << required_rotation << endl;
		// if the angular difference is greater than smallest turn possible 
		// pick the right turn to allign itself to the target

		FORRAction decision;
		int rotIntensity=0;
		while(fabs(required_rotation) > rotate[rotIntensity] and rotIntensity < numRotates) {
			rotIntensity++;
		}
		// cout << "Rotation Intensity : " << rotIntensity << endl;
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
			// cout << "Move Intensity : " << intensity << endl;

			while(move[intensity] > current_position.middle_distance_min){
				intensity--;
			}
			// cout << "Move Intensity : " << intensity << endl;
			if(go_to_top_point == true and intensity > 0)
				decision = FORRAction(FORWARD, intensity);
		}
		// cout << "Action choosen : " << decision.type << "," << decision.parameter << endl;
		last_position = current_position;
		return decision;
	}

	bool pointAlreadyInStack(DecisionPoint new_point, bool onlyCompleted = false){
		cout << "Check if point in stack" << endl;
		int start_highway = highway_grid[(int)(new_point.point.getX())][(int)(new_point.point.getY())];
		int end_highway = -1;
		int middle_highway = -1;
		double width_length = 0;
		// double near_far_width = 0;
		if(new_point.direction == true){
			end_highway = highway_grid[(int)(new_point.right_point.getX())][(int)(new_point.right_point.getY())];
			middle_highway = highway_grid[(int)((new_point.point.getX() + new_point.right_point.getX())/2.0)][(int)((new_point.point.getY() + new_point.right_point.getY())/2.0)];
			width_length = new_point.right_distance / (new_point.right_width);
			// near_far_width = new_point.right_width / new_point.right_width_max;
		}
		else{
			end_highway = highway_grid[(int)(new_point.left_point.getX())][(int)(new_point.left_point.getY())];
			middle_highway = highway_grid[(int)((new_point.point.getX() + new_point.left_point.getX())/2.0)][(int)((new_point.point.getY() + new_point.left_point.getY())/2.0)];
			width_length = new_point.left_distance / (new_point.left_width);
			// near_far_width = new_point.left_width / new_point.left_width_max;
		}
		int numInvalid = 0;
		if(start_highway >= 0 or start_highway == -2)
			numInvalid++;
		if(middle_highway >= 0 or middle_highway == -2)
			numInvalid++;
		if(end_highway >= 0 or end_highway == -2)
			numInvalid++;
		cout << "width_length " << width_length << " overall_avg_distance " << new_point.overall_avg_distance << " overall_max_distance " << new_point.overall_max_distance << " overall_min_distance " << new_point.overall_min_distance << " overall_median_distance " << new_point.overall_median_distance << " overall_stdev_distance " << new_point.overall_stdev_distance << endl;
		cout << "start_highway " << start_highway << " middle_highway " << middle_highway << " end_highway " << end_highway << " numInvalid " << numInvalid << endl;
		if(width_length < 1){
			cout << "pointAlreadyInStack rejected width_length" << endl;
			return true;
		}
		// else if(near_far_width < 0.5){
		// 	cout << "pointAlreadyInStack rejected near_far_width" << endl;
		// 	return true;
		// }
		else if(new_point.overall_avg_distance > 4.75 and new_point.overall_max_distance <= 16.5 and new_point.overall_median_distance > 3.9 and new_point.overall_stdev_distance <= 4.25){
			cout << "pointAlreadyInStack rejected cluster1" << endl;
			return true;
		}
		else if(new_point.overall_avg_distance > 4.3 and new_point.overall_max_distance <= 16.5 and new_point.overall_median_distance <= 3.9 and new_point.overall_min_distance > 1.6){
			cout << "pointAlreadyInStack rejected cluster2" << endl;
			return true;
		}
		else if(numInvalid >= 2){
			cout << "pointAlreadyInStack rejected atleast_two_labeled" << endl;
			return true;
		}
		else if(anySimilarOnList(new_point, highway_stack) == true and !onlyCompleted){
			cout << "pointAlreadyInStack rejected similar_stack" << endl;
			return true;
		}
		else if(anySimilarOnList(new_point, highway_stack_longest) == true and !onlyCompleted){
			cout << "pointAlreadyInStack rejected similar_stack_longest" << endl;
			return true;
		}
		else if(anySimilarOnList(new_point, highway_stack_completed) == true){
			cout << "pointAlreadyInStack rejected similar_stack_completed" << endl;
			return true;
		}
		else if(start_highway == -1 and (middle_highway == -1 or end_highway == -1)){
			cout << "pointAlreadyInStack accepted start_mid_or_end" << endl;
			return false;
		}
		else if(start_highway >= 0 and middle_highway == -1 and end_highway == -1){
			cout << "pointAlreadyInStack accepted mid_end" << endl;
			return false;
		}
		else{
			cout << "pointAlreadyInStack rejected other" << endl;
			return true;
		}
	}

	bool anySimilarOnList(DecisionPoint new_point, vector<DecisionPoint> stack){
		for(int i = 0; i < stack.size(); i++){
			Position new_point_start = new_point.point;
			Position new_point_end;
			Position exist_point_start;
			Position exist_point_end;
			if(new_point.direction == true){
				new_point_end = new_point.right_point;
				exist_point_start = stack[i].point;
				exist_point_end = stack[i].right_point;
			}
			else{
				new_point_end = new_point.left_point;
				exist_point_start = stack[i].point;
				exist_point_end = stack[i].left_point;
			}
			double new_dist = new_point_start.getDistance(new_point_end);
			double exist_dist = exist_point_start.getDistance(exist_point_end);
			double start_to_start_dist = new_point_start.getDistance(exist_point_start);
			double end_to_end_dist = new_point_end.getDistance(exist_point_end);
			double start_to_end_dist = new_point_start.getDistance(exist_point_end);
			double end_to_start_dist = new_point_end.getDistance(exist_point_start);
			if((start_to_start_dist <= 1 and end_to_end_dist <= 1) or (start_to_end_dist <= 1 and end_to_start_dist <= 1) or (start_to_start_dist <= 1 and start_to_end_dist <= 1) or (end_to_end_dist <= 1 and end_to_start_dist <= 1)){
				return true;
			}
			double x = new_point_start.getX();
			double y = new_point_start.getY();
			double x1 = exist_point_start.getX();
			double y1 = exist_point_start.getY();
			double x2 = exist_point_end.getX();
			double y2 = exist_point_end.getY();
			double A = x - x1;
			double B = y - y1;
			double C = x2 - x1;
			double D = y2 - y1;
			double dot = A * C + B * D;
			double len_sq = C * C + D * D;
			double param = -1;
			if (len_sq != 0) //in case of 0 length line
				param = dot / len_sq;
			double xx, yy;
			if (param < 0) {
				xx = x1;
				yy = y1;
			}
			else if (param > 1) {
				xx = x2;
				yy = y2;
			}
			else {
				xx = x1 + param * C;
				yy = y1 + param * D;
			}
			double dx = x - xx;
			double dy = y - yy;
			double start_to_closest_exist = sqrt(dx * dx + dy * dy);
			Position start_to_closest_on_exist(xx, yy, 0);
			x = new_point_end.getX();
			y = new_point_end.getY();
			x1 = exist_point_start.getX();
			y1 = exist_point_start.getY();
			x2 = exist_point_end.getX();
			y2 = exist_point_end.getY();
			A = x - x1;
			B = y - y1;
			C = x2 - x1;
			D = y2 - y1;
			dot = A * C + B * D;
			len_sq = C * C + D * D;
			param = -1;
			if (len_sq != 0) //in case of 0 length line
				param = dot / len_sq;
			if (param < 0) {
				xx = x1;
				yy = y1;
			}
			else if (param > 1) {
				xx = x2;
				yy = y2;
			}
			else {
				xx = x1 + param * C;
				yy = y1 + param * D;
			}
			dx = x - xx;
			dy = y - yy;
			double end_to_closest_exist = sqrt(dx * dx + dy * dy);
			Position end_to_closest_on_exist(xx, yy, 0);
			x = exist_point_start.getX();
			y = exist_point_start.getY();
			x1 = new_point_start.getX();
			y1 = new_point_start.getY();
			x2 = new_point_end.getX();
			y2 = new_point_end.getY();
			A = x - x1;
			B = y - y1;
			C = x2 - x1;
			D = y2 - y1;
			dot = A * C + B * D;
			len_sq = C * C + D * D;
			param = -1;
			if (len_sq != 0) //in case of 0 length line
				param = dot / len_sq;
			if (param < 0) {
				xx = x1;
				yy = y1;
			}
			else if (param > 1) {
				xx = x2;
				yy = y2;
			}
			else {
				xx = x1 + param * C;
				yy = y1 + param * D;
			}
			dx = x - xx;
			dy = y - yy;
			double start_to_closest_new = sqrt(dx * dx + dy * dy);
			Position start_to_closest_on_new(xx, yy, 0);
			x = exist_point_end.getX();
			y = exist_point_end.getY();
			x1 = new_point_start.getX();
			y1 = new_point_start.getY();
			x2 = new_point_end.getX();
			y2 = new_point_end.getY();
			A = x - x1;
			B = y - y1;
			C = x2 - x1;
			D = y2 - y1;
			dot = A * C + B * D;
			len_sq = C * C + D * D;
			param = -1;
			if (len_sq != 0) //in case of 0 length line
				param = dot / len_sq;
			if (param < 0) {
				xx = x1;
				yy = y1;
			}
			else if (param > 1) {
				xx = x2;
				yy = y2;
			}
			else {
				xx = x1 + param * C;
				yy = y1 + param * D;
			}
			dx = x - xx;
			dy = y - yy;
			double end_to_closest_new = sqrt(dx * dx + dy * dy);
			Position end_to_closest_on_new(xx, yy, 0);

			if((end_to_closest_exist <= 1 and start_to_closest_new <= 1 and end_to_closest_on_exist.getDistance(start_to_closest_on_new) >= ((new_dist + exist_dist)/2.0)/3.0) or (end_to_closest_new <= 1 and start_to_closest_exist <= 1 and end_to_closest_on_new.getDistance(start_to_closest_on_exist) >= ((new_dist + exist_dist)/2.0)/3.0) or (start_to_start_dist <= 1 and end_to_closest_exist <= 1) or (start_to_start_dist <= 1 and end_to_closest_new <= 1) or (start_to_closest_exist <= 1 and end_to_closest_exist <= 1) or (start_to_closest_new <= 1 and end_to_closest_new <= 1) or (end_to_end_dist <= 1 and start_to_closest_exist <= 1) or (end_to_end_dist <= 1 and start_to_closest_new <= 1)){
				return true;
			}
		}
		return false;
	}

private:
	int length;
	int height;
	double distance_threshold;
	double time_threshold;
	double move[300];  
	double rotate[300];
	int numMoves, numRotates;
	int top_point_decisions;
	int decision_limit;
	int start_rotations;
	vector< vector<int> > highway_grid;
	vector< vector< vector< pair<int, int> > > > highway_grid_connections;
	vector<Highway> highways;
	// priority_queue<DecisionPoint> highway_queue;
	vector<DecisionPoint> highway_stack;
	vector<DecisionPoint> highway_stack_longest;
	vector<int> stack_index;
	vector<int> stack_longest_index;
	vector<DecisionPoint> highway_stack_completed;
	DecisionPoint last_position;
	vector<DecisionPoint> position_history;
	int last_highway;
	Position current_target;
	Position middle_of_highway;
	DecisionPoint top_point;
	int top_point_index;
	vector< vector<int> > traveled_grid;
	vector< vector<double> > path_to_top_point;
	bool highways_complete;
	bool go_to_top_point;
	bool go_to_farthest_on_grid;
	bool too_close;
	// bool too_close_front;
	double dist_travelled_so_far;
	double avg_left;
	double avg_right;
	double avg_count;
};

#endif