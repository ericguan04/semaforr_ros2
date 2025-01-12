/**!
  * Task.h
  * 
  * /author: Anoop Aroor
  *
  *          Defines tasks as simple target and stores its status and statistics of completion
  */

#ifndef TASK_H
#define TASK_H

#include "FORRAction.h"
#include "FORRGeometry.h"
#include "Position.h"
#include "PathPlanner.h"
#include <vector>
#include <map>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sensor_msgs/msg/laser_scan.hpp>

/*
 * Struct for skeleton waypoint.
 */
struct sk_waypoint {
	int type; // 0 = region, 1 = subtrail, 2 = intersection, 3 = passage
    FORRRegion region;
    vector<CartesianPoint> path;
    vector<CartesianPoint> original_path;
    vector< vector<int> > passage_points;
    CartesianPoint passage_centroid;
    int passage_label;
    int passage_orientation; // 0 = vertical, 1 = horizontal
    int creator; // 0 = skeleton planner, 1 = passage planner, 2 = out, 3 = LLE
    sk_waypoint(): type(-1), region(), path(), original_path(), passage_points(), passage_centroid(), passage_label(-1), passage_orientation(-1), creator(-1) { }
    sk_waypoint(int b, FORRRegion r, vector<CartesianPoint> p, vector< vector<int> > pp, int cr): type(b), region(r), path(p), original_path(p), passage_points(pp), creator(cr) { }

    int getType() { return type; }
    FORRRegion getRegion() { return region; }
    vector<CartesianPoint> getPath() { return path; }
    CartesianPoint getPathEnd() { return path[path.size()-1]; }
    CartesianPoint getPathBegin() { return path[0]; }
    void setPath(vector<CartesianPoint> np) { path = np; }
    vector< vector<int> > getPassagePoints() { return passage_points; }
    void setPassageLabel(int lab) { passage_label = lab; }
    int getPassageLabel() { return passage_label; }
    void setPassageCentroid(CartesianPoint p) { passage_centroid = p; }
    CartesianPoint getPassageCentroid() { return passage_centroid; }
    void setPassageOrientation(int ori) { passage_orientation = ori; }
    int getPassageOrientation() { return passage_orientation; }
    int getCreator() { return creator; }
    void setCreator(int cr) { creator = cr; }
};

class Task {
  
 public:
  
  Task(double x_in, double y_in, int length, int height)  
    {
      x = x_in;
      wx = x_in; // current waypoint x
      y = y_in;  
      wy = y_in; // current waypoint y
      wr = sk_waypoint();
      decision_count = 0;
      isPlanActive = false;
      isPlanComplete = true;
      plannerName = "none";
      decisionSequence = new std::vector<FORRAction>;
      pos_hist = new vector<Position>();
      laser_hist = new vector< vector<CartesianPoint> >();
      laser_scan_hist = new vector< sensor_msgs::msg::LaserScan >();
      dimension = 200;
      if(length > dimension){
        dimension = length;
      }
      if(height > dimension){
        dimension = height;
      }
      for(int i = 0; i < dimension; i++){
      	vector<int> col;
      	for(int j = 0; j < dimension; j++){
      		col.push_back(0);
      	}
      	planPositions.push_back(col);
      }
    }

  double getTaskX(){ return x;}
  double getTaskY(){ return y;}
  int getDimension(){ return dimension;}

  double getX() { 
	if(isPlanActive == false){
		return x;
	}
	else{
		if(plannerName != "skeleton" and plannerName != "hallwayskel"){
			return wx;
		}
		else if(plannerName == "skeleton"){
			if(wr.getType() == 0){
				return wr.getRegion().getCenter().get_x();
			}
			else{
				return wr.getPathBegin().get_x();
			}
		}
		else if(plannerName == "hallwayskel"){
			if(wr.getType() == 0){
				return wr.getRegion().getCenter().get_x();
			}
			else if(wr.getType() == 1){
				return wr.getPathBegin().get_x();
			}
			else if(wr.getType() == 2){
				return wr.getPassageCentroid().get_x();
			}
			else if(wr.getType() == 3){
				// return wr.getPassageCentroid().get_x();
				return wr.getPathBegin().get_x();
			}
		}
	}
  }
  
  double getY(){
	if(isPlanActive == false){
		return y; 
	}
	else{
		if(plannerName != "skeleton" and plannerName != "hallwayskel"){
			return wy;
		}
		else if(plannerName == "skeleton"){
			if(wr.getType() == 0){
				return wr.getRegion().getCenter().get_y();
			}
			else{
				return wr.getPathBegin().get_y();
			}
		}
		else if(plannerName == "hallwayskel"){
			if(wr.getType() == 0){
				return wr.getRegion().getCenter().get_y();
			}
			else if(wr.getType() == 1){
				return wr.getPathBegin().get_y();
			}
			else if(wr.getType() == 2){
				return wr.getPassageCentroid().get_y();
			}
			else if(wr.getType() == 3){
				// return wr.getPassageCentroid().get_y();
				return wr.getPathBegin().get_y();
			}
		}
	}
  }

  sk_waypoint getSkeletonWaypoint(){
  	if(plannerName == "skeleton" or plannerName == "hallwayskel"){
		return wr;
  	}
  	else{
  		vector<CartesianPoint> path;
  		if(isPlanActive == false){
  			path.push_back(CartesianPoint(x,y));
  		}
  		else{
  			path.push_back(CartesianPoint(wx,wy));
  		}
  		return sk_waypoint(1, FORRRegion(), path, vector< vector<int> >(), -1);
  	}
  }

  vector<sk_waypoint> getSkeletonWaypoints(){
  	return skeleton_waypoints;
  }

  bool getIsPlanActive(){return isPlanActive;}
  void setIsPlanActive(bool status){isPlanActive = status;}
  bool getIsPlanComplete(){return isPlanComplete;}
  
  int getDecisionCount(){return decision_count;} 
 
  int incrementDecisionCount() {decision_count += 1;}

  std::vector<FORRAction> getPreviousDecisions(){
	return *decisionSequence;
  }

  FORRAction saveDecision(FORRAction decision){
	decisionSequence->push_back(decision);
	//cout << "After decisionToPush" << endl;
  }

  vector<Position> *getPositionHistory(){return pos_hist;}

  void clearPositionHistory(){pos_hist->clear();}

  void saveSensor(Position currentPosition, vector<CartesianPoint> laserEndpoints, sensor_msgs::msg::LaserScan ls){
  	pos_hist->push_back(currentPosition);
  	laser_hist->push_back(laserEndpoints);
  	laser_scan_hist->push_back(ls);
	// if(pos_hist->size() < 1){
	// 	pos_hist->push_back(currentPosition);
	// 	laser_hist->push_back(laserEndpoints);
	// }
	// else{	
 //     		Position pos = pos_hist->back();
 //     		if(!(pos == currentPosition)) {
	// 		pos_hist->push_back(currentPosition);
	// 		laser_hist->push_back(laserEndpoints);
	// 	}
	// }
  }

  vector< vector <CartesianPoint> > *getLaserHistory(){return laser_hist;}

  vector< sensor_msgs::msg::LaserScan > *getLaserScanHistory(){return laser_scan_hist;}

  vector<CartesianPoint> getWaypoints(){
  	// cout << "in getWaypoints" << endl;
  	if(plannerName != "skeleton" and plannerName != "hallwayskel"){
  		return waypoints;
  	}
  	else if(plannerName == "skeleton"){
  		vector<CartesianPoint> points;
  		// cout << "number of skeleton_waypoints " << skeleton_waypoints.size() << endl;
  		for(int i = 0; i < skeleton_waypoints.size(); i++){
  			if(skeleton_waypoints[i].getType() == 0){
  				points.push_back(skeleton_waypoints[i].getRegion().getCenter());
  				// cout << "region waypoint " << i << " " << skeleton_waypoints[i].getRegion().getCenter().get_x() << " " << skeleton_waypoints[i].getRegion().getCenter().get_y() << " " << skeleton_waypoints[i].getRegion().getRadius() << endl;
  			}
  			// else{
  			// 	vector<CartesianPoint> pathBetween = skeleton_waypoints[i].getPath();
  			// 	for(int j = 0; j < pathBetween.size(); j++){
  			// 		points.push_back(pathBetween[j]);
  			// 		// cout << "path waypoint " << i << " " << j << " " << pathBetween[j].get_x() << " " << pathBetween[j].get_y() << endl;
  			// 	}
  			// }
  		}
  		// cout << "number of points " << points.size() << endl;
  		return points;
  	}
  	else if(plannerName == "hallwayskel"){
  		vector<CartesianPoint> points;
  		// cout << "number of skeleton_waypoints " << skeleton_waypoints.size() << endl;
  		for(int i = 0; i < skeleton_waypoints.size(); i++){
  			if(skeleton_waypoints[i].getType() == 0 and skeleton_waypoints[i].getCreator() == 0){
  				points.push_back(skeleton_waypoints[i].getRegion().getCenter());
  				// cout << "region waypoint " << i << " " << skeleton_waypoints[i].getRegion().getCenter().get_x() << " " << skeleton_waypoints[i].getRegion().getCenter().get_y() << " " << skeleton_waypoints[i].getRegion().getRadius() << endl;
  			}
  			// else if(skeleton_waypoints[i].getType() == 1 or skeleton_waypoints[i].getType() == 3){
  			// 	vector<CartesianPoint> pathBetween = skeleton_waypoints[i].getPath();
  			// 	for(int j = 0; j < pathBetween.size(); j++){
  			// 		points.push_back(pathBetween[j]);
  			// 		// cout << "path waypoint " << i << " " << j << " " << pathBetween[j].get_x() << " " << pathBetween[j].get_y() << endl;
  			// 	}
  			// }
  			// else 
  			else if(skeleton_waypoints[i].getType() == 2 and skeleton_waypoints[i].getCreator() == 1){
  				points.push_back(skeleton_waypoints[i].getPassageCentroid());
  			}
  		}
  		// cout << "number of points " << points.size() << endl;
  		return points;
  	}
  }
  vector<CartesianPoint> getOrigWaypoints(){
  	if(plannerName != "hallwayskel"){
  		return origWaypoints;
  	}
  	else{
      vector<CartesianPoint> points;
  		for(int i = 0; i < alternate_skeleton_waypoints.size(); i++){
  			if(alternate_skeleton_waypoints[i].getType() == 0 and alternate_skeleton_waypoints[i].getCreator() == 0){
  				points.push_back(alternate_skeleton_waypoints[i].getRegion().getCenter());
  				// cout << "region waypoint " << i << " " << alternate_skeleton_waypoints[i].getRegion().getCenter().get_x() << " " << alternate_skeleton_waypoints[i].getRegion().getCenter().get_y() << " " << alternate_skeleton_waypoints[i].getRegion().getRadius() << endl;
  			}
  			// else if(alternate_skeleton_waypoints[i].getType() == 1 or alternate_skeleton_waypoints[i].getType() == 3){
  			// 	vector<CartesianPoint> pathBetween = alternate_skeleton_waypoints[i].getPath();
  			// 	for(int j = 0; j < pathBetween.size(); j++){
  			// 		points.push_back(pathBetween[j]);
  			// 		// cout << "path waypoint " << i << " " << j << " " << pathBetween[j].get_x() << " " << pathBetween[j].get_y() << endl;
  			// 	}
  			// }
  			// else 
  			else if(alternate_skeleton_waypoints[i].getType() == 2 and alternate_skeleton_waypoints[i].getCreator() == 1){
  				points.push_back(alternate_skeleton_waypoints[i].getPassageCentroid());
  			}
  		}
  		// cout << "number of points " << points.size() << endl;
  		return points;
  	}
  }
  int getPlanSize(){
  	if(plannerName != "skeleton" and plannerName != "hallwayskel"){
  		return waypoints.size();
  	}
  	else{
  		return skeleton_waypoints.size();
  	}
  }

  string getPlannerName(){ return plannerName; }

  PathPlanner *getPathPlanner(){ return pathPlanner; }

  void clearWaypoints(int type){
  	waypoints.clear();
  	tierTwoWaypoints.clear();
  	origWaypoints.clear();
  	wx = x;
  	wy = y;
  	if(skeleton_waypoints.size() > 0 and type >= 0){
  		vector<sk_waypoint> temp = skeleton_waypoints;
  		skeleton_waypoints.clear();
  		for(int i = 0; i < temp.size(); i++){
  			if(temp[i].getCreator() != type){
  				skeleton_waypoints.push_back(temp[i]);
  			}
  		}
  		wr = skeleton_waypoints[0];
  	}
  	else{
  		skeleton_waypoints.clear();
  		alternate_skeleton_waypoints.clear();
  		wr = sk_waypoint();
  	}
  }

  // list<int> getWaypointInds(){return waypointInd;}
  vector< list<int> > getPlansInds(){return plansInds;}

  // generates new waypoints given currentposition and a planner
  bool generateWaypoints(Position source, PathPlanner *planner){
	waypoints.clear();
	tierTwoWaypoints.clear();
	skeleton_waypoints.clear();
	alternate_skeleton_waypoints.clear();
	pathCostInNavGraph = 0;
	pathCostInNavOrigGraph = 0;
	origPathCostInNavGraph = 0;
	origPathCostInOrigNavGraph = 0;
	//a_star planner works in cms so all units are converts into cm
	//once plan is generated waypoints are stored in meters
	Node s(1, source.getX()*100, source.getY()*100);
	planner->setSource(s);
	Node t(1, x*100, y*100);
	planner->setTarget(t);

	cout << "plan generation status" << planner->calcPath(true) << endl;

	// waypointInd = planner->getPath();
	plansInds = planner->getPaths();
	if(planner->getName() == "hallwayskel"){
		origNavGraph = planner->getOrigGraph();
		origPlansInds = planner->getOrigPaths();
		// cout << "origPlansInds " << origPlansInds.size() << " ";
		// if(origPlansInds.size() > 0){
		// 	cout << origPlansInds[0].size() << " " << origPlansInds[1].size();
		// }
		// cout << endl;
		planner->resetOrigPath();
	}
	// Graph *navGraph = planner->getGraph();
	// if(waypointInd.size() > 0){
	// 	isPlanActive = true;
	// 	isPlanComplete = false;
	// }
	// else{
	// 	isPlanActive = false;
	// }
	/*list<int>::iterator it;
	for ( it = waypointInd.begin(); it != waypointInd.end(); it++ ){
		double x = navGraph->getNode(*it).getX()/100.0;
		double y = navGraph->getNode(*it).getY()/100.0;
		cout << x << " " << y << endl;
		CartesianPoint waypoint(x,y);
		waypoints.push_back(waypoint);
		//if atleast one point is generated
		cout << "Plan active is true" << endl;
		isPlanActive = true;
	}
	setupNextWaypoint(source);*/
	planner->resetPath();
	pathPlanner = planner;
	cout << "plan generation complete" << endl;
  }

  bool generateOriginalWaypoints(Position source, PathPlanner *planner){
  	if(planner->getName() != "hallwayskel" and planner->getName() != "skeleton"){
		origWaypoints.clear();
		//a_star planner works in cms so all units are converts into cm
		//once plan is generated waypoints are stored in meters
		Node s(1, source.getX()*100, source.getY()*100);
		planner->setSource(s);
		Node t(1, x*100, y*100);
		planner->setTarget(t);

		cout << "plan generation status" << planner->calcOrigPath(true) << endl;

		list<int> path = planner->getOrigPath();
		navGraph = planner->getOrigGraph(); // we don't have to worry about this line
		list<int>::iterator it;
		for ( it = path.begin(); it != path.end(); it++ ){
			double x = navGraph->getNode(*it).getX()/100.0;
			double y = navGraph->getNode(*it).getY()/100.0;
			CartesianPoint waypoint(x,y);
			origWaypoints.push_back(waypoint);
	  	}
	  	origPathCostInOrigNavGraph = planner->getOrigPathCost();
	  	origPathCostInNavGraph = planner->calcPathCost(path);
	  	/*vector<CartesianPoint> skippedwaypoints;
		for(int i = 0; i < origWaypoints.size(); i+=4){
			skippedwaypoints.push_back(origWaypoints[i]);
		}
		origWaypoints = skippedwaypoints;*/
		planner->resetOrigPath();
		cout << "plan generation complete" << endl;
	}
  }

  bool generateWaypointsFromInds(Position source, vector<CartesianPoint> currentLaserEndpoints, PathPlanner *planner, list<int> indices, vector<FORRRegion> regions){
  	// cout << "Inside generateWaypointsFromInds" << endl;
	waypoints.clear();
	tierTwoWaypoints.clear();
	skeleton_waypoints.clear();
	alternate_skeleton_waypoints.clear();
	//a_star planner works in cms so all units are converts into cm
	//once plan is generated waypoints are stored in meters
	//Node s(1, source.getX()*100, source.getY()*100);
	//planner->setSource(s);
	//Node t(1, x*100, y*100);
	//planner->setTarget(t);

	//cout << "plan generation status" << planner->calcPath(true) << endl;
	waypointInd = indices;
	navGraph = planner->getGraph();
	pathPlanner = planner;
	plannerName = planner->getName();
	if(plannerName != "skeleton" and plannerName != "hallwayskel"){
		list<int>::iterator it;
		for ( it = waypointInd.begin(); it != waypointInd.end(); it++ ){
			// cout << "node " << (*it) << endl;
			double r_x = navGraph->getNode(*it).getX()/100.0;
			double r_y = navGraph->getNode(*it).getY()/100.0;
			// cout << r_x << " " << r_y << endl;
			CartesianPoint waypoint(r_x,r_y);
			waypoints.push_back(waypoint);
			//if atleast one point is generated
			// cout << "Plan active is true" << endl;
			isPlanActive = true;
			isPlanComplete = false;
		}
		pathCostInNavGraph = planner->getPathCost();
		vector<CartesianPoint> skippedwaypoints;
		for(int i = 0; i < waypoints.size(); i+=1){
			skippedwaypoints.push_back(waypoints[i]);
		}
		waypoints = skippedwaypoints;
		tierTwoWaypoints = skippedwaypoints;
		pathCostInNavOrigGraph = planner->calcOrigPathCost(waypointInd);
	}
	else if(plannerName == "skeleton"){
		int step = -1;
		int max_step = waypointInd.size()-1;
		double s_x = navGraph->getNode(waypointInd.front()).getX()/100.0;
		double s_y = navGraph->getNode(waypointInd.front()).getY()/100.0;
		double s_r = navGraph->getNode(waypointInd.front()).getRadius();
		bool s_vis = false;
		LineSegment s_lineseg;
		if(FORRRegion(CartesianPoint(s_x,s_y), s_r).inRegion(source.getX(), source.getY()) == false){
			int vRegion=-1;
			double vDist=1000000;
			for(int i = 0; i < regions.size() ; i++){
				if(regions[i].visibleFromRegion(CartesianPoint(source.getX(), source.getY()), 20) and regions[i].getMinExits().size() > 0){
					double dist_to_region = regions[i].getCenter().get_distance(CartesianPoint(source.getX(), source.getY()));
					if(dist_to_region < vDist){
						// cout << "Region " << i << " visible to point and distance " << dist_to_region << endl;
						vRegion = i;
						vDist = dist_to_region;
					}
				}
			}
			if(vRegion >= 0){
				s_lineseg = regions[vRegion].visibleLineSegmentFromRegion(CartesianPoint(source.getX(), source.getY()), 20);
				s_vis = true;
			}
		}
		double e_x = navGraph->getNode(waypointInd.back()).getX()/100.0;
		double e_y = navGraph->getNode(waypointInd.back()).getY()/100.0;
		double e_r = navGraph->getNode(waypointInd.back()).getRadius();
		bool e_vis = false;
		LineSegment e_lineseg;
		if(FORRRegion(CartesianPoint(e_x,e_y), e_r).inRegion(x, y) == false){
			int vRegion=-1;
			double vDist=1000000;
			for(int i = 0; i < regions.size() ; i++){
				if(regions[i].visibleFromRegion(CartesianPoint(x, y), 20) and regions[i].getMinExits().size() > 0){
					double dist_to_region = regions[i].getCenter().get_distance(CartesianPoint(x, y));
					if(dist_to_region < vDist){
						// cout << "Region " << i << " visible to point and distance " << dist_to_region << endl;
						vRegion = i;
						vDist = dist_to_region;
					}
				}
			}
			if(vRegion >= 0){
				e_lineseg = regions[vRegion].visibleLineSegmentFromRegion(CartesianPoint(x, y), 20);
				e_vis = true;
			}
		}
		if(s_vis){
			vector<CartesianPoint> path_from_s;
			CartesianPoint start = s_lineseg.get_endpoints().second;
			CartesianPoint end = s_lineseg.get_endpoints().first;
			double tx, ty;
			for(double j = 0; j <= 1; j += 0.05){
				tx = (end.get_x() * j) + (start.get_x() * (1 - j));
				ty = (end.get_y() * j) + (start.get_y() * (1 - j));
				path_from_s.push_back(CartesianPoint(tx, ty));
			}
			skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_s, vector< vector<int> >(), 0));
		}
		list<int>::iterator it;
		for ( it = waypointInd.begin(); it != waypointInd.end(); it++ ){
			step = step + 1;
			// cout << "node " << (*it) << " step " << step << endl;
			double r_x = navGraph->getNode(*it).getX()/100.0;
			double r_y = navGraph->getNode(*it).getY()/100.0;
			double r = navGraph->getNode(*it).getRadius();
			// cout << r_x << " " << r_y << " " << r << endl;
			skeleton_waypoints.push_back(sk_waypoint(0, FORRRegion(CartesianPoint(r_x,r_y), r), vector<CartesianPoint>(), vector< vector<int> >(), 0));
			list<int>::iterator itr1; 
			itr1 = it; 
			advance(itr1, 1);
			int forward_step = step + 1;
			// cout << "max_step " << max_step << " forward_step " << forward_step << endl;
			if(forward_step <= max_step){
				// if(navGraph->getEdge(*it, *itr1)->getFrom() == *it){
				// 	skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), navGraph->getEdge(*it, *itr1)->getEdgePath(true)));
				// }
				// else if(navGraph->getEdge(*it, *itr1)->getFrom() == *itr1){
				// 	skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), navGraph->getEdge(*it, *itr1)->getEdgePath(false)));
				// }
				vector<CartesianPoint> path_from_edge = navGraph->getEdge(*it, *itr1)->getEdgePath(true);
				if(FORRRegion(CartesianPoint(r_x,r_y), r).inRegion(path_from_edge[0])){
					skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_edge, vector< vector<int> >(), 0));
				}
				else{
					std::reverse(path_from_edge.begin(),path_from_edge.end());
					skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_edge, vector< vector<int> >(), 0));
				}
			}
			// cout << "num of waypoints " << skeleton_waypoints.size() << endl;
		}
		if(e_vis){
			vector<CartesianPoint> path_from_e;
			CartesianPoint start = e_lineseg.get_endpoints().first;
			CartesianPoint end = e_lineseg.get_endpoints().second;
			double tx, ty;
			for(double j = 0; j <= 1; j += 0.05){
				tx = (end.get_x() * j) + (start.get_x() * (1 - j));
				ty = (end.get_y() * j) + (start.get_y() * (1 - j));
				path_from_e.push_back(CartesianPoint(tx, ty));
			}
			skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_e, vector< vector<int> >(), 0));
		}
		if(skeleton_waypoints.size() > 0){
			cout << "Plan active is true" << endl;
			isPlanActive = true;
			isPlanComplete = false;
		}
		pathCostInNavGraph = planner->getPathCost();
		pathCostInNavOrigGraph = planner->calcOrigPathCost(waypointInd);
	}
	else if(plannerName == "hallwayskel"){
		// origNavGraph = planner->getOrigGraph();
		// origPlansInds = planner->getOrigPaths();
		// cout << "Generate hallwayskel plan " << waypointInd.size() << endl;
		cout << "origPlansInds " << origPlansInds.size() << " " << origPlansInds[0].size() << " " << origPlansInds[1].size() << " " << origPlansInds[2].size() << endl;
		if(origPlansInds[0].size() > 0){
			// cout << "prologue creation" << endl;
			int step = -1;
			int max_step = origPlansInds[0].size()-1;
			double s_x = origNavGraph->getNode(origPlansInds[0].front()).getX()/100.0;
			double s_y = origNavGraph->getNode(origPlansInds[0].front()).getY()/100.0;
			double s_r = origNavGraph->getNode(origPlansInds[0].front()).getRadius();
			bool s_vis = false;
			LineSegment s_lineseg;
			if(FORRRegion(CartesianPoint(s_x,s_y), s_r).inRegion(source.getX(), source.getY()) == false){
				int vRegion=-1;
				double vDist=1000000;
				for(int i = 0; i < regions.size() ; i++){
					if(regions[i].visibleFromRegion(CartesianPoint(source.getX(), source.getY()), 20) and regions[i].getMinExits().size() > 0){
						double dist_to_region = regions[i].getCenter().get_distance(CartesianPoint(source.getX(), source.getY()));
						if(dist_to_region < vDist){
							// cout << "Region " << i << " visible to point and distance " << dist_to_region << endl;
							vRegion = i;
							vDist = dist_to_region;
						}
					}
				}
				if(vRegion >= 0){
					s_lineseg = regions[vRegion].visibleLineSegmentFromRegion(CartesianPoint(source.getX(), source.getY()), 20);
					s_vis = true;
				}
			}
			// cout << "s_vis " << s_vis << endl;
			if(s_vis){
				vector<CartesianPoint> path_from_s;
				CartesianPoint start = s_lineseg.get_endpoints().second;
				CartesianPoint end = s_lineseg.get_endpoints().first;
				double tx, ty;
				for(double j = 0; j <= 1; j += 0.05){
					tx = (end.get_x() * j) + (start.get_x() * (1 - j));
					ty = (end.get_y() * j) + (start.get_y() * (1 - j));
					path_from_s.push_back(CartesianPoint(tx, ty));
				}
				skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_s, vector< vector<int> >(), 1));
				// cout << "added s_vis " << path_from_s.size() << endl;
			}
			list<int>::iterator it;
			for ( it = origPlansInds[0].begin(); it != origPlansInds[0].end(); it++ ){
				step = step + 1;
				// cout << "node " << (*it) << " step " << step << endl;
				double r_x = origNavGraph->getNode(*it).getX()/100.0;
				double r_y = origNavGraph->getNode(*it).getY()/100.0;
				double r = origNavGraph->getNode(*it).getRadius();
				// cout << r_x << " " << r_y << " " << r << endl;
				skeleton_waypoints.push_back(sk_waypoint(0, FORRRegion(CartesianPoint(r_x,r_y), r), vector<CartesianPoint>(), vector< vector<int> >(), 1));
				list<int>::iterator itr1; 
				itr1 = it; 
				advance(itr1, 1);
				int forward_step = step + 1;
				// cout << "max_step " << max_step << " forward_step " << forward_step << endl;
				if(forward_step <= max_step){
					vector<CartesianPoint> path_from_edge = origNavGraph->getEdge(*it, *itr1)->getEdgePath(true);
					if(FORRRegion(CartesianPoint(r_x,r_y), r).inRegion(path_from_edge[0])){
						skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_edge, vector< vector<int> >(), 1));
					}
					else{
						std::reverse(path_from_edge.begin(),path_from_edge.end());
						skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_edge, vector< vector<int> >(), 1));
					}
				}
				// cout << "num of waypoints " << skeleton_waypoints.size() << endl;
			}
		}
		// cout << "passage plan creation" << endl;
		int regionID = -1;
		for(int i = 0; i < regions.size() ; i++){
			if(origPlansInds[0].size() > 0){
				if(regions[i].inRegion(skeleton_waypoints[skeleton_waypoints.size()-1].getRegion().getCenter())){
					regionID = i;
					break;
				}
			}
		}
		// cout << "final region in prologue " << regionID << endl;
		// if(regionID == -1){
		// 	int nRegion = -1;
		// 	for(int i = 0; i < regions.size() ; i++){
		// 		if(regions[i].inRegion(CartesianPoint(source.getX(),source.getY())) and regions[i].getMinExits().size() > 0){
		// 			// cout << "nRegion " << i << endl;
		// 			nRegion = i;
		// 		}
		// 		if(nRegion >= 0){
		// 			break;
		// 		}
		// 	}
		// 	if(nRegion == -1){
		// 		int vRegion = -1;
		// 		double vDist=1000000;
		// 		for(int i = 0; i < regions.size() ; i++){
		// 			if(regions[i].visibleFromRegion(CartesianPoint(source.getX(),source.getY()), 20) and regions[i].getMinExits().size() > 0){
		// 				double dist_to_region = regions[i].getCenter().get_distance(CartesianPoint(source.getX(),source.getY()));
		// 				if(dist_to_region < vDist){
		// 					// cout << "vRegion " << i << " visible to point and distance " << dist_to_region << endl;
		// 					vRegion = i;
		// 					vDist = dist_to_region;
		// 				}
		// 			}
		// 		}
		// 		nRegion = vRegion;
		// 	}
		// 	if(nRegion == -1){
		// 		int cRegion = -1;
		// 		double max_score = -100000000.0;
		// 		for(int i = 0; i < regions.size() ; i++){
		// 			double d = -3.0 * (regions[i].getCenter().get_distance(CartesianPoint(source.getX(),source.getY())) - regions[i].getRadius());
		// 			double neighbors = regions[i].getMinExits().size();
		// 			double score = d + neighbors;
		// 			if(score > max_score){
		// 				// cout << "cRegion " << i << " with score " << score << endl;
		// 				cRegion = i;
		// 				max_score = score;
		// 			}
		// 		}
		// 		nRegion = cRegion;
		// 	}
		// 	regionID = nRegion;
		// }
		int nPassage = passage_grid[(int)(source.getX())][(int)(source.getY())];
		int start_intersection = navGraph->getNode(waypointInd.front()).getIntersectionID();
		// cout << "nPassage " << nPassage << " start_intersection " << start_intersection << endl;
		if(origPlansInds[0].size() > 0){
			// nPassage = passage_grid[(int)(skeleton_waypoints[skeleton_waypoints.size()-1].getRegion().getCenter().get_x())][(int)(skeleton_waypoints[skeleton_waypoints.size()-1].getRegion().getCenter().get_y())];
			vector<int> passageValues = regions[regionID].getPassageValues();
			for(int i = 0; i < passageValues.size(); i++){
				if(passageValues[i] == start_intersection){
					nPassage = start_intersection;
					break;
				}
				else{
					nPassage = passageValues[i];
				}
			}
		}
		// cout << "nPassage " << nPassage << " start_intersection " << start_intersection << endl;
		if(passage_graph_nodes.count(nPassage) == 0 and nPassage > 0 and nPassage != start_intersection){
			// cout << "final region in passage, create sequence of regions" << endl;
			// vector<int> startregionIDs;
			// for(int i = 0; i < regions.size(); i++){
			// 	for(int j = 0; j < regions[i].getPassageValues().size(); j++){
			// 		if(regions[i].getPassageValues()[j] == start_intersection){
			// 			startregionIDs.push_back(i);
			// 		}
			// 	}
			// }
			// if(startregionIDs.size() == 0){
			// 	double min_dist = 10000000;
			// 	int startid = -1;
			// 	for(int i = 0; i < regions.size(); i++){
			// 		double dist_to_start = average_passage[start_intersection-1].get_distance(regions[i].getCenter());
			// 		double dist_to_region = regions[regionID].getCenter().get_distance(regions[i].getCenter());
			// 		bool edge_into_passage = false;
			// 		for(int j = 0; j < regions[i].getMinExits().size(); j++){
			// 			if(find(regions[regions[i].getMinExits()[j].getExitRegion()].getPassageValues().begin(), regions[regions[i].getMinExits()[j].getExitRegion()].getPassageValues().end(), nPassage) != regions[regions[i].getMinExits()[j].getExitRegion()].getPassageValues().end()){
			// 				edge_into_passage = true;
			// 				break;
			// 			}
			// 		}
			// 		if(10*dist_to_start + dist_to_region < min_dist and (find(regions[i].getPassageValues().begin(), regions[i].getPassageValues().end(), nPassage) != regions[i].getPassageValues().end() or edge_into_passage == true)){
			// 			min_dist = 10*dist_to_start + dist_to_region;
			// 			startid = i;
			// 		}
			// 	}
			// 	startregionIDs.push_back(startid);
			// }
			double min_dist = 10000000;
			int startid = -1;
			for(int i = 0; i < regions.size(); i++){
				double dist_to_start = average_passage[start_intersection-1].get_distance(regions[i].getCenter());
				bool foundPassage = false;
				vector<int> passageValues = regions[i].getPassageValues();
				for(int p = 0; p < passageValues.size(); p++){
					if(passageValues[p] == nPassage){
						foundPassage = true;
						break;
					}
				}
				if(dist_to_start < min_dist and foundPassage == true and regions[i].getMinExits().size() > 0){
					min_dist = dist_to_start;
					startid = i;
				}
			}
			// if(startid > -1){
			// 	// startregionIDs.push_back(startid);
			// 	cout << "startid " << startid << endl;
			// }
			// else{
			// 	cout << "could not find region close to intersection" << endl;
			// 	// continue;
			// }
			if(regionID == startid or startid == -1){
				// cout << "same region, just add that one " << regions[regionID].getCenter().get_x() << " " << regions[regionID].getCenter().get_y() << " " << regions[regionID].getRadius() << endl;
				skeleton_waypoints.push_back(sk_waypoint(0, regions[regionID], vector<CartesianPoint>(), vector< vector<int> >(), 1));
			}
			else{
				priority_queue<RegionNode, vector<RegionNode>, greater<RegionNode> > rn_queue;
				RegionNode start_rn = RegionNode(regions[regionID], regionID, 0);
				// cout << "regionID exits " << regions[regionID].getMinExits().size() << endl;
				for(int i = 0; i < regions[regionID].getMinExits().size(); i++){
					RegionNode neighbor = RegionNode(regions[regions[regionID].getMinExits()[i].getExitRegion()], regions[regionID].getMinExits()[i].getExitRegion(), regions[regionID].getMinExits()[i].getExitDistance());
					neighbor.addToRegionSequence(start_rn);
					// cout << "neighbor " << i << " ID " << neighbor.regionID << " regionSequence " << neighbor.regionSequence.size() << endl;
					bool foundPassage = false;
					bool foundIntersection = false;
					vector<int> passageValues = neighbor.region.getPassageValues();
					for(int p = 0; p < passageValues.size(); p++){
						if(passageValues[p] == nPassage){
							foundPassage = true;
						}
						if(passageValues[p] == start_intersection){
							foundIntersection = true;
						}
						if(foundPassage and foundIntersection){
							break;
						}
					}
					if(foundPassage == true or foundIntersection == true or neighbor.regionID == startid){
						rn_queue.push(neighbor);
					}
				}
				vector<int> already_searched;
				already_searched.push_back(start_rn.regionID);
				// cout << "rn_queue " << rn_queue.size() << " already_searched " << already_searched.size() << endl;
				RegionNode final_rn;
				int count = 0;
				while(rn_queue.size() > 0 and count < 15){
					RegionNode current_neighbor = rn_queue.top();
					// cout << "current_neighbor " << current_neighbor.regionID << " cost " << current_neighbor.nodeCost << endl;
					already_searched.push_back(current_neighbor.regionID);
					rn_queue.pop();
					if(current_neighbor.regionID == startid){
						final_rn = current_neighbor;
						break;
					}
					for(int i = 0; i < current_neighbor.region.getMinExits().size(); i++){
						RegionNode eRegion = RegionNode(regions[current_neighbor.region.getMinExits()[i].getExitRegion()], current_neighbor.region.getMinExits()[i].getExitRegion(), current_neighbor.nodeCost + current_neighbor.region.getMinExits()[i].getExitDistance());
						eRegion.setRegionSequence(current_neighbor.regionSequence);
						eRegion.addToRegionSequence(current_neighbor);
						// cout << "eRegion " << i << " ID " << eRegion.regionID << " cost " << eRegion.nodeCost << " regionSequence " << eRegion.regionSequence.size() << endl;
						bool foundPassage = false;
						bool foundIntersection = false;
						vector<int> passageValues = eRegion.region.getPassageValues();
						for(int p = 0; p < passageValues.size(); p++){
							if(passageValues[p] == nPassage){
								foundPassage = true;
							}
							if(passageValues[p] == start_intersection){
								foundIntersection = true;
							}
							if(foundPassage and foundIntersection){
								break;
							}
						}
						bool foundAlreadySearched = false;
						for(int a = 0; a < already_searched.size(); a++){
							if(already_searched[a] == eRegion.regionID){
								foundAlreadySearched = true;
								break;
							}
						}
						if((foundPassage == true or foundIntersection == true or eRegion.regionID == startid) and foundAlreadySearched == false){
							rn_queue.push(eRegion);
						}
					}
					// cout << "rn_queue " << rn_queue.size() << " already_searched " << already_searched.size() << endl;
					count = count + 1;
				}
				// cout << "final_rn " << final_rn.regionID << " regionSequence " << final_rn.regionSequence.size() << " rn_queue " << rn_queue.size() << " already_searched " << already_searched.size() << endl;
				if(final_rn.regionID == -1){
					// cout << "cannot find connection, just add regionid and startid " << regions[regionID].getCenter().get_x() << " " << regions[regionID].getCenter().get_y() << " " << regions[regionID].getRadius() << " " << regions[startid].getCenter().get_x() << " " << regions[startid].getCenter().get_y() << " " << regions[startid].getRadius() << endl;
					skeleton_waypoints.push_back(sk_waypoint(0, regions[regionID], vector<CartesianPoint>(), vector< vector<int> >(), 1));
					skeleton_waypoints.push_back(sk_waypoint(0, regions[startid], vector<CartesianPoint>(), vector< vector<int> >(), 1));
				}
				else{
					vector<RegionNode> region_sequence = final_rn.regionSequence;
					for(int i = 0; i < region_sequence.size()-1; i++){
						skeleton_waypoints.push_back(sk_waypoint(0, region_sequence[i].region, vector<CartesianPoint>(), vector< vector<int> >(), 1));
						// cout << region_sequence[i].region.getCenter().get_x() << " " << region_sequence[i].region.getCenter().get_y() << " " << region_sequence[i].region.getRadius() << endl;
						for(int j = 0; j < region_sequence[i].region.getMinExits().size(); j++){
							if(region_sequence[i].region.getMinExits()[j].getExitRegion() == region_sequence[i+1].regionID){
								if(region_sequence[i].region.inRegion(region_sequence[i].region.getMinExits()[j].getConnectionPoints()[0])){
									// cout << region_sequence[i].region.getMinExits()[j].getConnectionPoints().size() << endl;
									skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), region_sequence[i].region.getMinExits()[j].getConnectionPoints(), vector< vector<int> >(), 1));
								}
								else{
									// cout << region_sequence[i].region.getMinExits()[j].getConnectionPoints().size() << endl;
									vector<CartesianPoint> path_from_edge = region_sequence[i].region.getMinExits()[j].getConnectionPoints();
									std::reverse(path_from_edge.begin(),path_from_edge.end());
									skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_edge, vector< vector<int> >(), 1));
								}
								break;
							}
						}
					}
					// cout << "num of waypoints " << skeleton_waypoints.size() << endl;
					skeleton_waypoints.push_back(sk_waypoint(0, region_sequence[region_sequence.size()-1].region, vector<CartesianPoint>(), vector< vector<int> >(), 1));
					// cout << region_sequence[region_sequence.size()-1].region.getCenter().get_x() << " " << region_sequence[region_sequence.size()-1].region.getCenter().get_y() << " " << region_sequence[region_sequence.size()-1].region.getRadius() << endl;
					for(int j = 0; j < region_sequence[region_sequence.size()-1].region.getMinExits().size(); j++){
						if(region_sequence[region_sequence.size()-1].region.getMinExits()[j].getExitRegion() == final_rn.regionID){
							if(region_sequence[region_sequence.size()-1].region.inRegion(region_sequence[region_sequence.size()-1].region.getMinExits()[j].getConnectionPoints()[0])){
								// cout << region_sequence[region_sequence.size()-1].region.getMinExits()[j].getConnectionPoints().size() << endl;
								skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), region_sequence[region_sequence.size()-1].region.getMinExits()[j].getConnectionPoints(), vector< vector<int> >(), 1));
							}
							else{
								// cout << region_sequence[region_sequence.size()-1].region.getMinExits()[j].getConnectionPoints().size() << endl;
								vector<CartesianPoint> path_from_edge = region_sequence[region_sequence.size()-1].region.getMinExits()[j].getConnectionPoints();
								std::reverse(path_from_edge.begin(),path_from_edge.end());
								skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_edge, vector< vector<int> >(), 1));
							}
							break;
						}
					}
					skeleton_waypoints.push_back(sk_waypoint(0, final_rn.region, vector<CartesianPoint>(), vector< vector<int> >(), 1));
					// cout << final_rn.region.getCenter().get_x() << " " << final_rn.region.getCenter().get_y() << " " << final_rn.region.getRadius() << endl;
					// cout << "num of waypoints " << skeleton_waypoints.size() << endl;
					// sk_waypoint new_passage = sk_waypoint(3, FORRRegion(), vector<CartesianPoint>(), passage_graph_edges[nPassage], 1);
					// new_passage.setPassageLabel(nPassage);
					// new_passage.setPassageCentroid(average_passage[nPassage-1]);
					// new_passage.setPassageOrientation(passage_graph_edges_orientation[nPassage]);
					// skeleton_waypoints.push_back(new_passage);
				}
			}
		}
		// cout << "create plan in passage graph" << endl;
		int step = -1;
		int max_step = waypointInd.size()-1;
		int start_passage = skeleton_waypoints.size();
		list<int>::iterator it;
		for ( it = waypointInd.begin(); it != waypointInd.end(); it++ ){
			step = step + 1;
			// cout << "node " << (*it) << " step " << step << endl;
			int intersection1 = navGraph->getNode(*it).getIntersectionID();
			// cout << "intersection1 " << intersection1 << endl;
			sk_waypoint new_waypoint = sk_waypoint(2, FORRRegion(), vector<CartesianPoint>(), passage_graph_nodes[intersection1], 1);
			new_waypoint.setPassageLabel(intersection1);
			new_waypoint.setPassageCentroid(average_passage[intersection1-1]);
			skeleton_waypoints.push_back(new_waypoint);
			// cout << new_waypoint.getPassageCentroid().get_x() << " " << new_waypoint.getPassageCentroid().get_y() << endl;
			list<int>::iterator itr1;
			itr1 = it;
			advance(itr1, 1);
			int forward_step = step + 1;
			// cout << "max_step " << max_step << " forward_step " << forward_step << endl;
			if(forward_step <= max_step){
				int intersection2 = navGraph->getNode(*itr1).getIntersectionID();
				// cout << "intersection2 " << intersection2 << endl;
				int passage12;
				vector<CartesianPoint> passage_path;
				for(int i = 0; i < passage_graph.size(); i++){
					// cout << "passage_graph " << passage_graph[i][0] << " " << passage_graph[i][1] << " " << passage_graph[i][2] << endl;
					if(passage_graph[i][0] == intersection1 and passage_graph[i][2] == intersection2){
						passage12 = passage_graph[i][1];
						passage_path = graph_trails[i];
						break;
					}
					else if(passage_graph[i][0] == intersection2 and passage_graph[i][2] == intersection1){
						passage12 = passage_graph[i][1];
						passage_path = graph_trails[i];
						std::reverse(passage_path.begin(), passage_path.end());
						break;
					}
				}
				// cout << "passage12 " << passage12 << endl;
				// vector<int> regionID1;
				// vector<int> regionID2;
				vector<int> regionP12;
				for(int i = 0; i < regions.size(); i++){
					vector<int> passageValues = regions[i].getPassageValues();
					for(int j = 0; j < passageValues.size(); j++){
						// if(regions[i].getPassageValues()[j] == intersection1){
						// 	regionID1.push_back(i);
						// }
						// if(regions[i].getPassageValues()[j] == intersection2){
						// 	regionID2.push_back(i);
						// }
						if(passageValues[j] == passage12){
							regionP12.push_back(i);
							// cout << i << endl;
							break;
						}
					}
				}
				// cout << "regionP12 " << regionP12.size() << endl;
				// if(regionID1.size() == 0){
				// 	double min_dist = 10000000;
				// 	int startid = -1;
				// 	for(int i = 0; i < regions.size(); i++){
				// 		double dist_to_start = average_passage[intersection1-1].get_distance(regions[i].getCenter());
				// 		double dist_to_second = average_passage[intersection2-1].get_distance(regions[i].getCenter());
				// 		bool edge_into_passage = false;
				// 		for(int j = 0; j < regions[i].getMinExits().size(); j++){
				// 			if(find(regions[regions[i].getMinExits()[j].getExitRegion()].getPassageValues().begin(), regions[regions[i].getMinExits()[j].getExitRegion()].getPassageValues().end(), passage12) != regions[regions[i].getMinExits()[j].getExitRegion()].getPassageValues().end()){
				// 				edge_into_passage = true;
				// 				break;
				// 			}
				// 		}
				// 		if(10*dist_to_start + dist_to_second < min_dist and (find(regions[i].getPassageValues().begin(), regions[i].getPassageValues().end(), passage12) != regions[i].getPassageValues().end() or edge_into_passage == true)){
				// 			min_dist = 10*dist_to_start + dist_to_second;
				// 			startid = i;
				// 		}
				// 	}
				// 	regionID1.push_back(startid);
				// }
				// if(regionID2.size() == 0){
				// 	double min_dist = 10000000;
				// 	int startid = -1;
				// 	for(int i = 0; i < regions.size(); i++){
				// 		double dist_to_start = average_passage[intersection2-1].get_distance(regions[i].getCenter());
				// 		double dist_to_second = average_passage[intersection1-1].get_distance(regions[i].getCenter());
				// 		bool edge_into_passage = false;
				// 		for(int j = 0; j < regions[i].getMinExits().size(); j++){
				// 			if(find(regions[regions[i].getMinExits()[j].getExitRegion()].getPassageValues().begin(), regions[regions[i].getMinExits()[j].getExitRegion()].getPassageValues().end(), passage12) != regions[regions[i].getMinExits()[j].getExitRegion()].getPassageValues().end()){
				// 				edge_into_passage = true;
				// 				break;
				// 			}
				// 		}
				// 		if(10*dist_to_start + dist_to_second < min_dist and (find(regions[i].getPassageValues().begin(), regions[i].getPassageValues().end(), passage12) != regions[i].getPassageValues().end() or edge_into_passage == true)){
				// 			min_dist = 10*dist_to_start + dist_to_second;
				// 			startid = i;
				// 		}
				// 	}
				// 	regionID2.push_back(startid);
				// }
				if(regionP12.size() == 0){
					// cout << "no available regions in passage, use trail instead " << passage_path.size() << endl;
					sk_waypoint new_passage = sk_waypoint(3, FORRRegion(), passage_path, passage_graph_edges[passage12], 1);
					new_passage.setPassageLabel(passage12);
					new_passage.setPassageCentroid(average_passage[passage12-1]);
					new_passage.setPassageOrientation(passage_graph_edges_orientation[passage12]);
					skeleton_waypoints.push_back(new_passage);
				}
				else{
					double min_dist_s = 10000000;
					double min_dist_e = 10000000;
					int startid = -1;
					int endid = -1;
					for(int i = 0; i < regionP12.size(); i++){
						double dist_to_start = average_passage[intersection1-1].get_distance(regions[regionP12[i]].getCenter());
						double dist_to_end = average_passage[intersection2-1].get_distance(regions[regionP12[i]].getCenter());
						if(dist_to_start < min_dist_s){
							min_dist_s = dist_to_start;
							startid = regionP12[i];
						}
						if(dist_to_end < min_dist_e){
							min_dist_e = dist_to_end;
							endid = regionP12[i];
						}
					}
					// cout << "startid " << startid << " endid " << endid << endl;
					if(startid == endid){
						// cout << "same region, just add that one " << regions[startid].getCenter().get_x() << " " << regions[startid].getCenter().get_y() << " " << regions[startid].getRadius() << endl;
						skeleton_waypoints.push_back(sk_waypoint(0, regions[startid], vector<CartesianPoint>(), vector< vector<int> >(), 1));
					}
					else{
						priority_queue<RegionNode, vector<RegionNode>, greater<RegionNode> > rn_queue;
						vector<int> already_searched;
						RegionNode start_rn = RegionNode(regions[startid], startid, 0);
						// cout << "startid exits " << regions[startid].getMinExits().size() << endl;
						for(int i = 0; i < regions[startid].getMinExits().size(); i++){
							RegionNode neighbor = RegionNode(regions[regions[startid].getMinExits()[i].getExitRegion()], regions[startid].getMinExits()[i].getExitRegion(), regions[startid].getMinExits()[i].getExitDistance());
							neighbor.addToRegionSequence(start_rn);
							// cout << "neighbor " << i << " ID " << neighbor.regionID << " regionSequence " << neighbor.regionSequence.size() << endl;
							bool inPassage = false;
							for(int p = 0; p < regionP12.size(); p++){
								if(regionP12[p] == neighbor.regionID){
									inPassage = true;
									break;
								}
							}
							if(inPassage == true){
								rn_queue.push(neighbor);
							}
						}
						already_searched.push_back(start_rn.regionID);
						// cout << "rn_queue " << rn_queue.size() << " already_searched " << already_searched.size() << endl;
						RegionNode final_rn;
						int count = 0;
						while(rn_queue.size() > 0 and count < 15){
							RegionNode current_neighbor = rn_queue.top();
							// cout << "current_neighbor " << current_neighbor.regionID << " cost " << current_neighbor.nodeCost << endl;
							already_searched.push_back(current_neighbor.regionID);
							rn_queue.pop();
							if(current_neighbor.regionID == endid){
								final_rn = current_neighbor;
								break;
							}
							for(int i = 0; i < current_neighbor.region.getMinExits().size(); i++){
								RegionNode eRegion = RegionNode(regions[current_neighbor.region.getMinExits()[i].getExitRegion()], current_neighbor.region.getMinExits()[i].getExitRegion(), current_neighbor.nodeCost + current_neighbor.region.getMinExits()[i].getExitDistance());
								eRegion.setRegionSequence(current_neighbor.regionSequence);
								eRegion.addToRegionSequence(current_neighbor);
								// cout << "eRegion " << i << " ID " << eRegion.regionID << " cost " << eRegion.nodeCost << " regionSequence " << eRegion.regionSequence.size() << endl;
								bool inPassage = false;
								for(int p = 0; p < regionP12.size(); p++){
									if(regionP12[p] == eRegion.regionID){
										inPassage = true;
										break;
									}
								}
								bool foundAlreadySearched = false;
								for(int a = 0; a < already_searched.size(); a++){
									if(already_searched[a] == eRegion.regionID){
										foundAlreadySearched = true;
										break;
									}
								}
								if(inPassage == true and foundAlreadySearched == false){
									rn_queue.push(eRegion);
								}
							}
							// cout << "rn_queue " << rn_queue.size() << " already_searched " << already_searched.size() << endl;
							count = count + 1;
						}
						// cout << "final_rn " << final_rn.regionID << " regionSequence " << final_rn.regionSequence.size() << " rn_queue " << rn_queue.size() << " already_searched " << already_searched.size() << endl;
						if(final_rn.regionID == -1){
							// cout << "cannot find connection, just add start and end " << regions[startid].getCenter().get_x() << " " << regions[startid].getCenter().get_y() << " " << regions[startid].getRadius() << " " << regions[endid].getCenter().get_x() << " " << regions[endid].getCenter().get_y() << " " << regions[endid].getRadius() << endl;
							skeleton_waypoints.push_back(sk_waypoint(0, regions[startid], vector<CartesianPoint>(), vector< vector<int> >(), 1));
							skeleton_waypoints.push_back(sk_waypoint(0, regions[endid], vector<CartesianPoint>(), vector< vector<int> >(), 1));
						}
						else{
							vector<RegionNode> region_sequence = final_rn.regionSequence;
							for(int i = 0; i < region_sequence.size()-1; i++){
								skeleton_waypoints.push_back(sk_waypoint(0, region_sequence[i].region, vector<CartesianPoint>(), vector< vector<int> >(), 1));
								// cout << region_sequence[i].region.getCenter().get_x() << " " << region_sequence[i].region.getCenter().get_y() << " " << region_sequence[i].region.getRadius() << endl;
								for(int j = 0; j < region_sequence[i].region.getMinExits().size(); j++){
									if(region_sequence[i].region.getMinExits()[j].getExitRegion() == region_sequence[i+1].regionID){
										if(region_sequence[i].region.inRegion(region_sequence[i].region.getMinExits()[j].getConnectionPoints()[0])){
											// cout << region_sequence[i].region.getMinExits()[j].getConnectionPoints().size() << endl;
											skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), region_sequence[i].region.getMinExits()[j].getConnectionPoints(), vector< vector<int> >(), 1));
										}
										else{
											// cout << region_sequence[i].region.getMinExits()[j].getConnectionPoints().size() << endl;
											vector<CartesianPoint> path_from_edge = region_sequence[i].region.getMinExits()[j].getConnectionPoints();
											std::reverse(path_from_edge.begin(),path_from_edge.end());
											skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_edge, vector< vector<int> >(), 1));
										}
										break;
									}
								}
							}
							// cout << "num of waypoints " << skeleton_waypoints.size() << endl;
							skeleton_waypoints.push_back(sk_waypoint(0, region_sequence[region_sequence.size()-1].region, vector<CartesianPoint>(), vector< vector<int> >(), 1));
							// cout << region_sequence[region_sequence.size()-1].region.getCenter().get_x() << " " << region_sequence[region_sequence.size()-1].region.getCenter().get_y() << " " << region_sequence[region_sequence.size()-1].region.getRadius() << endl;
							for(int j = 0; j < region_sequence[region_sequence.size()-1].region.getMinExits().size(); j++){
								if(region_sequence[region_sequence.size()-1].region.getMinExits()[j].getExitRegion() == final_rn.regionID){
									if(region_sequence[region_sequence.size()-1].region.inRegion(region_sequence[region_sequence.size()-1].region.getMinExits()[j].getConnectionPoints()[0])){
										// cout << region_sequence[region_sequence.size()-1].region.getMinExits()[j].getConnectionPoints().size() << endl;
										skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), region_sequence[region_sequence.size()-1].region.getMinExits()[j].getConnectionPoints(), vector< vector<int> >(), 1));
									}
									else{
										// cout << region_sequence[region_sequence.size()-1].region.getMinExits()[j].getConnectionPoints().size() << endl;
										vector<CartesianPoint> path_from_edge = region_sequence[region_sequence.size()-1].region.getMinExits()[j].getConnectionPoints();
										std::reverse(path_from_edge.begin(),path_from_edge.end());
										skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_edge, vector< vector<int> >(), 1));
									}
									break;
								}
							}
							skeleton_waypoints.push_back(sk_waypoint(0, final_rn.region, vector<CartesianPoint>(), vector< vector<int> >(), 1));
							// cout << final_rn.region.getCenter().get_x() << " " << final_rn.region.getCenter().get_y() << " " << final_rn.region.getRadius() << endl;
							// cout << "num of waypoints " << skeleton_waypoints.size() << endl;
						}
					}
				}
				// sk_waypoint new_passage = sk_waypoint(3, FORRRegion(), vector<CartesianPoint>(), passage_graph_edges[passage12], 1);
				// new_passage.setPassageLabel(passage12);
				// new_passage.setPassageCentroid(average_passage[passage12-1]);
				// new_passage.setPassageOrientation(passage_graph_edges_orientation[passage12]);
				// skeleton_waypoints.push_back(new_passage);
				// cout << new_passage.getPassageCentroid().get_x() << " " << new_passage.getPassageCentroid().get_y() << endl;
			}
			// cout << "num of waypoints " << skeleton_waypoints.size() << endl;
		}
		// cout << "num of waypoints " << skeleton_waypoints.size() << endl;
		// int end_passage = skeleton_waypoints.size();
		// for(int i = start_passage+1; i < end_passage-1; i+=3){
		// 	for(int j = 0; j < graph_through_intersections.size(); j++){
		// 		if(graph_through_intersections[j][0] == skeleton_waypoints[i].getPassageLabel() and graph_through_intersections[j][1] == skeleton_waypoints[i+1].getPassageLabel() and graph_through_intersections[j][2] == skeleton_waypoints[i+2].getPassageLabel()){
		// 			for(int k = 0; k < graph_intersection_trails[j].size(); k++){
		// 				skeleton_waypoints[i].getPath().push_back(graph_intersection_trails[j][k]);
		// 			}
		// 			break;
		// 		}
		// 		else if(graph_through_intersections[j][2] == skeleton_waypoints[i].getPassageLabel() and graph_through_intersections[j][1] == skeleton_waypoints[i+1].getPassageLabel() and graph_through_intersections[j][0] == skeleton_waypoints[i+2].getPassageLabel()){
		// 			for(int k = graph_intersection_trails[j].size()-1; k >= 0; k--){
		// 				skeleton_waypoints[i].getPath().push_back(graph_intersection_trails[j][k]);
		// 			}
		// 			break;
		// 		}
		// 	}
		// }
		// if(passage_graph_nodes.count(nPassage) == 0 and nPassage > 0){
		// 	sk_waypoint new_passage = sk_waypoint(3, FORRRegion(), vector<CartesianPoint>(), passage_graph_edges[nPassage], 1);
		// 	new_passage.setPassageLabel(nPassage);
		// 	new_passage.setPassageCentroid(average_passage[nPassage-1]);
		// 	new_passage.setPassageOrientation(passage_graph_edges_orientation[nPassage]);
		// 	skeleton_waypoints.push_back(new_passage);
		// }
		regionID = -1;
		for(int i = 0; i < regions.size() ; i++){
			if(origPlansInds[1].size() > 0){
				if(regions[i].inRegion(CartesianPoint(origNavGraph->getNode(origPlansInds[1].front()).getX()/100.0, origNavGraph->getNode(origPlansInds[1].front()).getY()/100.0))){
					regionID = i;
					break;
				}
			}
		}
		// cout << "first region in epilogue " << regionID << endl;
		// if(regionID == -1){
		// 	int nRegion = -1;
		// 	for(int i = 0; i < regions.size() ; i++){
		// 		if(regions[i].inRegion(CartesianPoint(x,y)) and regions[i].getMinExits().size() > 0){
		// 			// cout << "nRegion " << i << endl;
		// 			nRegion = i;
		// 		}
		// 		if(nRegion >= 0){
		// 			break;
		// 		}
		// 	}
		// 	if(nRegion == -1){
		// 		int vRegion = -1;
		// 		double vDist=1000000;
		// 		for(int i = 0; i < regions.size() ; i++){
		// 			if(regions[i].visibleFromRegion(CartesianPoint(x,y), 20) and regions[i].getMinExits().size() > 0){
		// 				double dist_to_region = regions[i].getCenter().get_distance(CartesianPoint(x,y));
		// 				if(dist_to_region < vDist){
		// 					// cout << "vRegion " << i << " visible to point and distance " << dist_to_region << endl;
		// 					vRegion = i;
		// 					vDist = dist_to_region;
		// 				}
		// 			}
		// 		}
		// 		nRegion = vRegion;
		// 	}
		// 	if(nRegion == -1){
		// 		int cRegion = -1;
		// 		double max_score = -100000000.0;
		// 		for(int i = 0; i < regions.size() ; i++){
		// 			double d = -3.0 * (regions[i].getCenter().get_distance(CartesianPoint(x,y)) - regions[i].getRadius());
		// 			double neighbors = regions[i].getMinExits().size();
		// 			double score = d + neighbors;
		// 			if(score > max_score){
		// 				// cout << "cRegion " << i << " with score " << score << endl;
		// 				cRegion = i;
		// 				max_score = score;
		// 			}
		// 		}
		// 		nRegion = cRegion;
		// 	}
		// 	regionID = nRegion;
		// }
		nPassage = passage_grid[(int)(x)][(int)(y)];
		int end_intersection = navGraph->getNode(waypointInd.back()).getIntersectionID();
		// cout << "nPassage " << nPassage << " end_intersection " << end_intersection << endl;
		if(origPlansInds[1].size() > 0){
			// nPassage = passage_grid[(int)(origNavGraph->getNode(origPlansInds[1].front()).getX()/100.0)][(int)(origNavGraph->getNode(origPlansInds[1].front()).getY()/100.0)];
			vector<int> passageValues = regions[regionID].getPassageValues();
			for(int i = 0; i < passageValues.size(); i++){
				if(passageValues[i] == end_intersection){
					nPassage = end_intersection;
					break;
				}
				else{
					nPassage = passageValues[i];
				}
			}
		}
		// cout << "nPassage " << nPassage << " end_intersection " << end_intersection << endl;
		if(passage_graph_nodes.count(nPassage) == 0 and nPassage > 0 and nPassage != end_intersection){
			// cout << "first region in passage, create sequence of regions" << endl;
			// vector<int> endregionIDs;
			// for(int i = 0; i < regions.size(); i++){
			// 	for(int j = 0; j < regions[i].getPassageValues().size(); j++){
			// 		if(regions[i].getPassageValues()[j] == end_intersection){
			// 			endregionIDs.push_back(i);
			// 		}
			// 	}
			// }
			// if(endregionIDs.size() == 0){
			// 	double min_dist = 10000000;
			// 	int endid = -1;
			// 	for(int i = 0; i < regions.size(); i++){
			// 		double dist_to_end = average_passage[end_intersection-1].get_distance(regions[i].getCenter());
			// 		double dist_to_region = regions[regionID].getCenter().get_distance(regions[i].getCenter());
			// 		bool edge_into_passage = false;
			// 		for(int j = 0; j < regions[i].getMinExits().size(); j++){
			// 			if(find(regions[regions[i].getMinExits()[j].getExitRegion()].getPassageValues().begin(), regions[regions[i].getMinExits()[j].getExitRegion()].getPassageValues().end(), nPassage) != regions[regions[i].getMinExits()[j].getExitRegion()].getPassageValues().end()){
			// 				edge_into_passage = true;
			// 				break;
			// 			}
			// 		}
			// 		if(10*dist_to_end + dist_to_region < min_dist and (find(regions[i].getPassageValues().begin(), regions[i].getPassageValues().end(), nPassage) != regions[i].getPassageValues().end() or edge_into_passage == true)){
			// 			min_dist = 10*dist_to_end + dist_to_region;
			// 			endid = i;
			// 		}
			// 	}
			// 	endregionIDs.push_back(endid);
			// }
			double min_dist = 10000000;
			int endid = -1;
			for(int i = 0; i < regions.size(); i++){
				double dist_to_end = average_passage[end_intersection-1].get_distance(regions[i].getCenter());
				bool foundPassage = false;
				vector<int> passageValues = regions[i].getPassageValues();
				for(int p = 0; p < passageValues.size(); p++){
					if(passageValues[p] == nPassage){
						foundPassage = true;
						break;
					}
				}
				if(dist_to_end < min_dist and foundPassage == true and regions[i].getMinExits().size() > 0){
					min_dist = dist_to_end;
					endid = i;
				}
			}
			// if(endid > -1){
			// 	// endregionIDs.push_back(endid);
			// 	cout << "endid " << endid << endl;
			// }
			// else{
			// 	cout << "could not find region close to intersection" << endl;
			// 	// continue;
			// }
			if(regionID == endid or endid == -1){
				// cout << "same region, just add that one " << regions[regionID].getCenter().get_x() << " " << regions[regionID].getCenter().get_y() << " " << regions[regionID].getRadius() << endl;
				skeleton_waypoints.push_back(sk_waypoint(0, regions[regionID], vector<CartesianPoint>(), vector< vector<int> >(), 1));
			}
			else{
				priority_queue<RegionNode, vector<RegionNode>, greater<RegionNode> > rn_queue;
				RegionNode start_rn = RegionNode(regions[regionID], regionID, 0);
				// cout << "regionID exits " << regions[regionID].getMinExits().size() << endl;
				for(int i = 0; i < regions[regionID].getMinExits().size(); i++){
					RegionNode neighbor = RegionNode(regions[regions[regionID].getMinExits()[i].getExitRegion()], regions[regionID].getMinExits()[i].getExitRegion(), regions[regionID].getMinExits()[i].getExitDistance());
					neighbor.addToRegionSequence(start_rn);
					// cout << "neighbor " << i << " ID " << neighbor.regionID << " regionSequence " << neighbor.regionSequence.size() << endl;
					bool foundPassage = false;
					bool foundIntersection = false;
					vector<int> passageValues = neighbor.region.getPassageValues();
					for(int p = 0; p < passageValues.size(); p++){
						if(passageValues[p] == nPassage){
							foundPassage = true;
						}
						if(passageValues[p] == end_intersection){
							foundIntersection = true;
						}
						if(foundPassage and foundIntersection){
							break;
						}
					}
					if(foundPassage == true or foundIntersection == true or neighbor.regionID == endid){
						rn_queue.push(neighbor);
					}
				}
				vector<int> already_searched;
				already_searched.push_back(start_rn.regionID);
				// cout << "rn_queue " << rn_queue.size() << " already_searched " << already_searched.size() << endl;
				RegionNode final_rn;
				int count = 0;
				while(rn_queue.size() > 0 and count < 15){
					RegionNode current_neighbor = rn_queue.top();
					// cout << "current_neighbor " << current_neighbor.regionID << " cost " << current_neighbor.nodeCost << endl;
					already_searched.push_back(current_neighbor.regionID);
					rn_queue.pop();
					if(current_neighbor.regionID == endid){
						final_rn = current_neighbor;
						break;
					}
					for(int i = 0; i < current_neighbor.region.getMinExits().size(); i++){
						RegionNode eRegion = RegionNode(regions[current_neighbor.region.getMinExits()[i].getExitRegion()], current_neighbor.region.getMinExits()[i].getExitRegion(), current_neighbor.nodeCost + current_neighbor.region.getMinExits()[i].getExitDistance());
						eRegion.setRegionSequence(current_neighbor.regionSequence);
						eRegion.addToRegionSequence(current_neighbor);
						// cout << "eRegion " << i << " ID " << eRegion.regionID << " cost " << eRegion.nodeCost << " regionSequence " << eRegion.regionSequence.size() << endl;
						bool foundPassage = false;
						bool foundIntersection = false;
						vector<int> passageValues = eRegion.region.getPassageValues();
						for(int p = 0; p < passageValues.size(); p++){
							if(passageValues[p] == nPassage){
								foundPassage = true;
							}
							if(passageValues[p] == end_intersection){
								foundIntersection = true;
							}
							if(foundPassage and foundIntersection){
								break;
							}
						}
						bool foundAlreadySearched = false;
						for(int a = 0; a < already_searched.size(); a++){
							if(already_searched[a] == eRegion.regionID){
								foundAlreadySearched = true;
								break;
							}
						}
						if((foundPassage == true or foundIntersection == true or eRegion.regionID == endid) and foundAlreadySearched == false){
							rn_queue.push(eRegion);
						}
					}
					// cout << "rn_queue " << rn_queue.size() << " already_searched " << already_searched.size() << endl;
					count = count + 1;
				}
				// cout << "final_rn " << final_rn.regionID << " regionSequence " << final_rn.regionSequence.size() << " rn_queue " << rn_queue.size() << " already_searched " << already_searched.size() << endl;
				if(final_rn.regionID == -1){
					// cout << "cannot find connection, just add regionid and endid " << regions[regionID].getCenter().get_x() << " " << regions[regionID].getCenter().get_y() << " " << regions[regionID].getRadius() << " " << regions[endid].getCenter().get_x() << " " << regions[endid].getCenter().get_y() << " " << regions[endid].getRadius() << endl;
					skeleton_waypoints.push_back(sk_waypoint(0, regions[endid], vector<CartesianPoint>(), vector< vector<int> >(), 1));
					skeleton_waypoints.push_back(sk_waypoint(0, regions[regionID], vector<CartesianPoint>(), vector< vector<int> >(), 1));
				}
				else{
					skeleton_waypoints.push_back(sk_waypoint(0, final_rn.region, vector<CartesianPoint>(), vector< vector<int> >(), 1));
					// cout << final_rn.region.getCenter().get_x() << " " << final_rn.region.getCenter().get_y() << " " << final_rn.region.getRadius() << endl;
					vector<RegionNode> region_sequence = final_rn.regionSequence;
					std::reverse(region_sequence.begin(),region_sequence.end());
					for(int j = 0; j < region_sequence[0].region.getMinExits().size(); j++){
						if(region_sequence[0].region.getMinExits()[j].getExitRegion() == final_rn.regionID){
							if(final_rn.region.inRegion(region_sequence[0].region.getMinExits()[j].getConnectionPoints()[0])){
								// cout << region_sequence[0].region.getMinExits()[j].getConnectionPoints().size() << endl;
								skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), region_sequence[0].region.getMinExits()[j].getConnectionPoints(), vector< vector<int> >(), 1));
							}
							else{
								// cout << region_sequence[0].region.getMinExits()[j].getConnectionPoints().size() << endl;
								vector<CartesianPoint> path_from_edge = region_sequence[0].region.getMinExits()[j].getConnectionPoints();
								std::reverse(path_from_edge.begin(),path_from_edge.end());
								skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_edge, vector< vector<int> >(), 1));
							}
							break;
						}
					}
					for(int i = 0; i < region_sequence.size()-1; i++){
						skeleton_waypoints.push_back(sk_waypoint(0, region_sequence[i].region, vector<CartesianPoint>(), vector< vector<int> >(), 1));
						// cout << region_sequence[i].region.getCenter().get_x() << " " << region_sequence[i].region.getCenter().get_y() << " " << region_sequence[i].region.getRadius() << endl;
						for(int j = 0; j < region_sequence[i].region.getMinExits().size(); j++){
							if(region_sequence[i].region.getMinExits()[j].getExitRegion() == region_sequence[i+1].regionID){
								if(region_sequence[i].region.inRegion(region_sequence[i].region.getMinExits()[j].getConnectionPoints()[0])){
									// cout << region_sequence[i].region.getMinExits()[j].getConnectionPoints().size() << endl;
									skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), region_sequence[i].region.getMinExits()[j].getConnectionPoints(), vector< vector<int> >(), 1));
								}
								else{
									// cout << region_sequence[i].region.getMinExits()[j].getConnectionPoints().size() << endl;
									vector<CartesianPoint> path_from_edge = region_sequence[i].region.getMinExits()[j].getConnectionPoints();
									std::reverse(path_from_edge.begin(),path_from_edge.end());
									skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_edge, vector< vector<int> >(), 1));
								}
								break;
							}
						}
					}
					// cout << "num of waypoints " << skeleton_waypoints.size() << endl;
					skeleton_waypoints.push_back(sk_waypoint(0, region_sequence[region_sequence.size()-1].region, vector<CartesianPoint>(), vector< vector<int> >(), 1));
					// cout << region_sequence[region_sequence.size()-1].region.getCenter().get_x() << " " << region_sequence[region_sequence.size()-1].region.getCenter().get_y() << " " << region_sequence[region_sequence.size()-1].region.getRadius() << endl;
					// cout << "num of waypoints " << skeleton_waypoints.size() << endl;
				}
			}
		}
		if(origPlansInds[1].size() > 0){
			// cout << "epilogue creation" << endl;
			int step = -1;
			int max_step = origPlansInds[1].size()-1;
			list<int>::iterator it;
			for ( it = origPlansInds[1].begin(); it != origPlansInds[1].end(); it++ ){
				step = step + 1;
				// cout << "node " << (*it) << " step " << step << endl;
				double r_x = origNavGraph->getNode(*it).getX()/100.0;
				double r_y = origNavGraph->getNode(*it).getY()/100.0;
				double r = origNavGraph->getNode(*it).getRadius();
				// cout << r_x << " " << r_y << " " << r << endl;
				skeleton_waypoints.push_back(sk_waypoint(0, FORRRegion(CartesianPoint(r_x,r_y), r), vector<CartesianPoint>(), vector< vector<int> >(), 1));
				list<int>::iterator itr1; 
				itr1 = it; 
				advance(itr1, 1);
				int forward_step = step + 1;
				// cout << "max_step " << max_step << " forward_step " << forward_step << endl;
				if(forward_step <= max_step){
					vector<CartesianPoint> path_from_edge = origNavGraph->getEdge(*it, *itr1)->getEdgePath(true);
					if(FORRRegion(CartesianPoint(r_x,r_y), r).inRegion(path_from_edge[0])){
						skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_edge, vector< vector<int> >(), 1));
					}
					else{
						std::reverse(path_from_edge.begin(),path_from_edge.end());
						skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_edge, vector< vector<int> >(), 1));
					}
				}
				// cout << "num of waypoints " << skeleton_waypoints.size() << endl;
			}
			double e_x = origNavGraph->getNode(origPlansInds[1].back()).getX()/100.0;
			double e_y = origNavGraph->getNode(origPlansInds[1].back()).getY()/100.0;
			double e_r = origNavGraph->getNode(origPlansInds[1].back()).getRadius();
			bool e_vis = false;
			LineSegment e_lineseg;
			if(FORRRegion(CartesianPoint(e_x,e_y), e_r).inRegion(x, y) == false){
				int vRegion=-1;
				double vDist=1000000;
				for(int i = 0; i < regions.size() ; i++){
					if(regions[i].visibleFromRegion(CartesianPoint(x, y), 20) and regions[i].getMinExits().size() > 0){
						double dist_to_region = regions[i].getCenter().get_distance(CartesianPoint(x, y));
						if(dist_to_region < vDist){
							// cout << "Region " << i << " visible to point and distance " << dist_to_region << endl;
							vRegion = i;
							vDist = dist_to_region;
						}
					}
				}
				if(vRegion >= 0){
					e_lineseg = regions[vRegion].visibleLineSegmentFromRegion(CartesianPoint(x, y), 20);
					e_vis = true;
				}
			}
			// cout << "e_vis " << e_vis << endl;
			if(e_vis){
				vector<CartesianPoint> path_from_e;
				CartesianPoint start = e_lineseg.get_endpoints().first;
				CartesianPoint end = e_lineseg.get_endpoints().second;
				double tx, ty;
				for(double j = 0; j <= 1; j += 0.05){
					tx = (end.get_x() * j) + (start.get_x() * (1 - j));
					ty = (end.get_y() * j) + (start.get_y() * (1 - j));
					path_from_e.push_back(CartesianPoint(tx, ty));
				}
				skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_e, vector< vector<int> >(), 0));
				// cout << "added e_vis " << path_from_e.size() << endl;
			}
		}
		cout << "final passage num of waypoints " << skeleton_waypoints.size() << endl;
		// vector<sk_waypoint> alternate_skeleton_waypoints;
		if(origPlansInds[2].size() > 0){
			cout << "generate plan in skeleton graph" << endl;
			int step = -1;
			int max_step = origPlansInds[2].size()-1;
			double s_x = origNavGraph->getNode(origPlansInds[2].front()).getX()/100.0;
			double s_y = origNavGraph->getNode(origPlansInds[2].front()).getY()/100.0;
			double s_r = origNavGraph->getNode(origPlansInds[2].front()).getRadius();
			bool s_vis = false;
			LineSegment s_lineseg;
			if(FORRRegion(CartesianPoint(s_x,s_y), s_r).inRegion(source.getX(), source.getY()) == false){
				int vRegion=-1;
				double vDist=1000000;
				for(int i = 0; i < regions.size() ; i++){
					if(regions[i].visibleFromRegion(CartesianPoint(source.getX(), source.getY()), 20) and regions[i].getMinExits().size() > 0){
						double dist_to_region = regions[i].getCenter().get_distance(CartesianPoint(source.getX(), source.getY()));
						if(dist_to_region < vDist){
							// cout << "Region " << i << " visible to point and distance " << dist_to_region << endl;
							vRegion = i;
							vDist = dist_to_region;
						}
					}
				}
				if(vRegion >= 0){
					s_lineseg = regions[vRegion].visibleLineSegmentFromRegion(CartesianPoint(source.getX(), source.getY()), 20);
					s_vis = true;
				}
			}
			double e_x = origNavGraph->getNode(origPlansInds[2].back()).getX()/100.0;
			double e_y = origNavGraph->getNode(origPlansInds[2].back()).getY()/100.0;
			double e_r = origNavGraph->getNode(origPlansInds[2].back()).getRadius();
			bool e_vis = false;
			LineSegment e_lineseg;
			if(FORRRegion(CartesianPoint(e_x,e_y), e_r).inRegion(x, y) == false){
				int vRegion=-1;
				double vDist=1000000;
				for(int i = 0; i < regions.size() ; i++){
					if(regions[i].visibleFromRegion(CartesianPoint(x, y), 20) and regions[i].getMinExits().size() > 0){
						double dist_to_region = regions[i].getCenter().get_distance(CartesianPoint(x, y));
						if(dist_to_region < vDist){
							// cout << "Region " << i << " visible to point and distance " << dist_to_region << endl;
							vRegion = i;
							vDist = dist_to_region;
						}
					}
				}
				if(vRegion >= 0){
					e_lineseg = regions[vRegion].visibleLineSegmentFromRegion(CartesianPoint(x, y), 20);
					e_vis = true;
				}
			}
			// cout << "s_vis " << s_vis << " e_vis " << e_vis << endl;
			if(s_vis){
				vector<CartesianPoint> path_from_s;
				CartesianPoint start = s_lineseg.get_endpoints().second;
				CartesianPoint end = s_lineseg.get_endpoints().first;
				double tx, ty;
				for(double j = 0; j <= 1; j += 0.05){
					tx = (end.get_x() * j) + (start.get_x() * (1 - j));
					ty = (end.get_y() * j) + (start.get_y() * (1 - j));
					path_from_s.push_back(CartesianPoint(tx, ty));
				}
				alternate_skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_s, vector< vector<int> >(), 0));
				// cout << "added s_vis " << path_from_s.size() << endl;
			}
			list<int>::iterator it;
			for ( it = origPlansInds[2].begin(); it != origPlansInds[2].end(); it++ ){
				step = step + 1;
				// cout << "node " << (*it) << " step " << step << endl;
				double r_x = origNavGraph->getNode(*it).getX()/100.0;
				double r_y = origNavGraph->getNode(*it).getY()/100.0;
				double r = origNavGraph->getNode(*it).getRadius();
				// cout << r_x << " " << r_y << " " << r << endl;
				alternate_skeleton_waypoints.push_back(sk_waypoint(0, FORRRegion(CartesianPoint(r_x,r_y), r), vector<CartesianPoint>(), vector< vector<int> >(), 0));
				list<int>::iterator itr1; 
				itr1 = it; 
				advance(itr1, 1);
				int forward_step = step + 1;
				// cout << "max_step " << max_step << " forward_step " << forward_step << endl;
				if(forward_step <= max_step){
					// if(origNavGraph->getEdge(*it, *itr1)->getFrom() == *it){
					// 	alternate_skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), origNavGraph->getEdge(*it, *itr1)->getEdgePath(true)));
					// }
					// else if(origNavGraph->getEdge(*it, *itr1)->getFrom() == *itr1){
					// 	alternate_skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), origNavGraph->getEdge(*it, *itr1)->getEdgePath(false)));
					// }
					vector<CartesianPoint> path_from_edge = origNavGraph->getEdge(*it, *itr1)->getEdgePath(true);
					if(FORRRegion(CartesianPoint(r_x,r_y), r).inRegion(path_from_edge[0])){
						alternate_skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_edge, vector< vector<int> >(), 0));
					}
					else{
						std::reverse(path_from_edge.begin(),path_from_edge.end());
						alternate_skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_edge, vector< vector<int> >(), 0));
					}
				}
				// cout << "final skeleton num of waypoints " << alternate_skeleton_waypoints.size() << endl;
			}
			if(e_vis){
				vector<CartesianPoint> path_from_e;
				CartesianPoint start = e_lineseg.get_endpoints().first;
				CartesianPoint end = e_lineseg.get_endpoints().second;
				double tx, ty;
				for(double j = 0; j <= 1; j += 0.05){
					tx = (end.get_x() * j) + (start.get_x() * (1 - j));
					ty = (end.get_y() * j) + (start.get_y() * (1 - j));
					path_from_e.push_back(CartesianPoint(tx, ty));
				}
				alternate_skeleton_waypoints.push_back(sk_waypoint(1, FORRRegion(), path_from_e, vector< vector<int> >(), 0));
				// cout << "added e_vis " << path_from_e.size() << endl;
			}
			// cout << "num of waypoints " << alternate_skeleton_waypoints.size() << endl;
		}
		if(alternate_skeleton_waypoints.size() > 0){
			double main_path_cost = 0;
			for(int i = 0; i < skeleton_waypoints.size()-1; i++){
				// cout << "type1 " << skeleton_waypoints[i].getType() << " type2 " << skeleton_waypoints[i+1].getType() << endl;
				if(skeleton_waypoints[i].getType() == 1 and skeleton_waypoints[i+1].getType() == 0){
					vector<CartesianPoint> sw_path = skeleton_waypoints[i].getPath();
					for(int j = 0; j < sw_path.size()-1; j++){
						main_path_cost += sw_path[j].get_distance(sw_path[j+1]);
					}
					main_path_cost += sw_path[sw_path.size()-1].get_distance(skeleton_waypoints[i+1].getRegion().getCenter());
				}
				else if(skeleton_waypoints[i].getType() == 0 and skeleton_waypoints[i+1].getType() == 1){
					vector<CartesianPoint> sw_path = skeleton_waypoints[i+1].getPath();
					main_path_cost += sw_path[0].get_distance(skeleton_waypoints[i].getRegion().getCenter());
					for(int j = 0; j < sw_path.size()-1; j++){
						main_path_cost += sw_path[j].get_distance(sw_path[j+1]);
					}
				}
				else if(skeleton_waypoints[i].getType() == 0 and skeleton_waypoints[i+1].getType() == 0){
					main_path_cost += skeleton_waypoints[i].getRegion().getCenter().get_distance(skeleton_waypoints[i+1].getRegion().getCenter());
				}
				else if(skeleton_waypoints[i].getType() == 0 and skeleton_waypoints[i+1].getType() == 2){
					main_path_cost += skeleton_waypoints[i].getRegion().getCenter().get_distance(skeleton_waypoints[i+1].getPassageCentroid());
				}
				else if(skeleton_waypoints[i].getType() == 0 and skeleton_waypoints[i+1].getType() == 3){
					main_path_cost += skeleton_waypoints[i].getRegion().getCenter().get_distance(skeleton_waypoints[i+1].getPassageCentroid());
				}
				else if(skeleton_waypoints[i].getType() == 2 and skeleton_waypoints[i+1].getType() == 0){
					main_path_cost += skeleton_waypoints[i+1].getRegion().getCenter().get_distance(skeleton_waypoints[i].getPassageCentroid());
				}
				else if(skeleton_waypoints[i].getType() == 2 and skeleton_waypoints[i+1].getType() == 3){
					main_path_cost += skeleton_waypoints[i].getPassageCentroid().get_distance(skeleton_waypoints[i+1].getPassageCentroid());
				}
				else if(skeleton_waypoints[i].getType() == 3 and skeleton_waypoints[i+1].getType() == 0){
					main_path_cost += skeleton_waypoints[i+1].getRegion().getCenter().get_distance(skeleton_waypoints[i].getPassageCentroid());
				}
				else if(skeleton_waypoints[i].getType() == 3 and skeleton_waypoints[i+1].getType() == 2){
					main_path_cost += skeleton_waypoints[i].getPassageCentroid().get_distance(skeleton_waypoints[i+1].getPassageCentroid());
				}
			}
			cout << "skeleton_waypoints " << skeleton_waypoints.size() << " cost " << main_path_cost << endl;
			double alternate_path_cost = 0;
			for(int i = 0; i < alternate_skeleton_waypoints.size()-1; i++){
				if(alternate_skeleton_waypoints[i].getType() == 1 and alternate_skeleton_waypoints[i+1].getType() == 0){
					vector<CartesianPoint> asw_path = alternate_skeleton_waypoints[i].getPath();
					for(int j = 0; j < asw_path.size()-1; j++){
						alternate_path_cost += asw_path[j].get_distance(asw_path[j+1]);
					}
					alternate_path_cost += asw_path[asw_path.size()-1].get_distance(alternate_skeleton_waypoints[i+1].getRegion().getCenter());
				}
				else if(alternate_skeleton_waypoints[i].getType() == 0 and alternate_skeleton_waypoints[i+1].getType() == 1){
					vector<CartesianPoint> asw_path = alternate_skeleton_waypoints[i+1].getPath();
					alternate_path_cost += asw_path[0].get_distance(alternate_skeleton_waypoints[i].getRegion().getCenter());
					for(int j = 0; j < asw_path.size()-1; j++){
						alternate_path_cost += asw_path[j].get_distance(asw_path[j+1]);
					}
				}
			}
			cout << "alternate_skeleton_waypoints " << alternate_skeleton_waypoints.size() << " cost " << alternate_path_cost << endl;
			pathCostInNavGraph = main_path_cost;
			pathCostInNavOrigGraph = planner->getPathCost() + planner->getOrigPathCost();
			origPathCostInOrigNavGraph = planner->getOrigPathCosts()[2];
      origPathCostInNavGraph = alternate_path_cost;
			if(alternate_path_cost < main_path_cost){
				vector<sk_waypoint> temp_skeleton_waypoints;
				for(int i = 0; i < skeleton_waypoints.size(); i++){
					temp_skeleton_waypoints.push_back(skeleton_waypoints[i]);
				}
				skeleton_waypoints = alternate_skeleton_waypoints;
				alternate_skeleton_waypoints = temp_skeleton_waypoints;
				pathCostInNavGraph = alternate_path_cost;
				pathCostInNavOrigGraph = planner->getOrigPathCosts()[2];
				origPathCostInOrigNavGraph = planner->getPathCost() + planner->getOrigPathCost();
        origPathCostInNavGraph = main_path_cost;
			}
		}
		else{
			pathCostInNavGraph = 0;
			pathCostInNavOrigGraph = planner->getPathCost() + planner->getOrigPathCost();
			origPathCostInOrigNavGraph = planner->getOrigPathCosts()[2];
      origPathCostInNavGraph = 0;
		}
		if(skeleton_waypoints.size() > 0){
			cout << "Plan active is true" << endl;
			isPlanActive = true;
			isPlanComplete = false;
		}
	}
	//setupNextWaypoint(source);
	setupNearestWaypoint(source, currentLaserEndpoints);
	//planner->resetPath();
	//cout << "plan generation complete" << endl;
  }


   double planCost(vector<CartesianPoint> waypoints, PathPlanner *planner, Position source, Position target){
   	double cost = planner->calcPathCost(waypoints, source, target);
   	return cost;
   }

 //   void setupNextWaypoint(Position currentPosition){
 //   	cout << "inside setup next waypoint" << endl;
	// double dis;
	// while(waypoints.size() >= 1){
	// 	cout << "inside while" << endl;
	// 	wx = waypoints[0].get_x();
	// 	wy = waypoints[0].get_y();
	// 	dis = currentPosition.getDistance(wx, wy);
	// 	if(dis < 0.75){
	// 		cout << "found waypoint with dist < 0.75" << endl;
	// 		waypoints.erase(waypoints.begin());
	// 	}
	// 	else{
	// 		break;
	// 	} 	
	// }
	// cout << "check plan active: " << waypoints.size() << endl;
	// if(waypoints.size() > 0){
	// 	isPlanActive = true;
	// }
	// else{
	// 	isPlanActive = false;
	// }
	// cout << "end setup next waypoint" << endl;
 //   }

	void createNewWaypoint(CartesianPoint new_point, int type){
		if(plannerName != "skeleton" and plannerName != "hallwayskel"){
			waypoints.insert(waypoints.begin(), new_point);
			// waypoints.push_back(new_point);
			isPlanActive = true;
			wx = new_point.get_x();
			wy = new_point.get_y();
			// cout << wx << " " << wy << endl;
		}
		else{
			// vector<CartesianPoint> addPoint;
			// addPoint.push_back(new_point);
			skeleton_waypoints.insert(skeleton_waypoints.begin(), sk_waypoint(0, FORRRegion(new_point, 0.75), vector<CartesianPoint>(), vector< vector<int> >(), type));
			// skeleton_waypoints.insert(skeleton_waypoints.begin(), sk_waypoint(1, FORRRegion(), addPoint, vector< vector<int> >()));
			isPlanActive = true;
			wr = skeleton_waypoints[0];
			// cout << wr.getRegion().getCenter().get_x() << " " << wr.getRegion().getCenter().get_y() << endl;
		}
	}

   void setupNearestWaypoint(Position currentPosition, vector<CartesianPoint> currentLaserEndpoints){
   	// cout << "inside setup nearest waypoint" << endl;
   	if(plannerName != "skeleton" and plannerName != "hallwayskel"){
		double dis;
		int farthest = -1;
		// cout << "waypoints size: " << waypoints.size() << endl;
		for (int i = 0; i < waypoints.size(); i++){
			dis = currentPosition.getDistance(waypoints[i].get_x(), waypoints[i].get_y());
			if(dis < 0.75){
				// cout << "found waypoint with dist < 0.75: " << i << endl;
				farthest = i;
			}
		}
		if(farthest == 0){
			if(waypoints.begin() == tierTwoWaypoints.begin()){
				tierTwoWaypoints.erase(tierTwoWaypoints.begin());
			}
			waypoints.erase(waypoints.begin());
		}
		else if(farthest > 0){
			for(int i = 0; i <= farthest; i++){
				for(int j = 0; j < tierTwoWaypoints.size(); j++){
					if(waypoints[i] == tierTwoWaypoints[j]){
						tierTwoWaypoints.erase(tierTwoWaypoints.begin()+j);
						j--;
					}
				}
			}
			waypoints.erase(waypoints.begin(), waypoints.begin()+farthest);
		}
		wx = waypoints[0].get_x();
		wy = waypoints[0].get_y();
		//cout << "check plan active: " << waypoints.size() << endl;
		if(waypoints.size() > 0){
			isPlanActive = true;
		}
		else{
			isPlanActive = false;
		}
		if(tierTwoWaypoints.size() == 0){
			isPlanComplete = true;
		}
	}
	else if(plannerName == "skeleton"){
		int farthest = -1;
		int farthest_path = -1;
		// cout << "skeleton_waypoints size: " << skeleton_waypoints.size() << " finished_sk_waypoints size: " << finished_sk_waypoints.size() << endl;
		for (int i = 0; i < skeleton_waypoints.size(); i++){
			if(skeleton_waypoints[i].getType() == 0){
				// cout << "region " << i << endl;
				if(skeleton_waypoints[i].getRegion().inRegion(CartesianPoint(currentPosition.getX(), currentPosition.getY())) and skeleton_waypoints[i].getRegion().getCenter().get_distance(CartesianPoint(currentPosition.getX(), currentPosition.getY())) < 0.75){
					// cout << "found skeleton region waypoint: " << i << endl;
					farthest = i;
				}
			}
			else{
				vector<CartesianPoint> pathBetween = skeleton_waypoints[i].getPath();
				double dis;
				// cout << "pathBetween " << i  << " size: " << pathBetween.size() << endl;
				int farthest_here = -1;
				for (int j = 0; j < pathBetween.size()-1; j++){
					dis = currentPosition.getDistance(pathBetween[j].get_x(), pathBetween[j].get_y());
					if(dis < 0.5 and canAccessPoint(currentLaserEndpoints, CartesianPoint(currentPosition.getX(), currentPosition.getY()), pathBetween[j+1], 20)){
						// cout << "found pathBetween with dist < 0.5: " << j << endl;
						farthest_here = j;
					}
				}
				if(farthest_here > -1 or currentPosition.getDistance(pathBetween[pathBetween.size()-1].get_x(), pathBetween[pathBetween.size()-1].get_y()) < 0.5){
					// cout << "found skeleton region waypoint: " << i << " farthest_here " << farthest_here << endl;
					farthest = i;
					farthest_path = farthest_here;
				}
			}
		}
		// cout << "farthest " << farthest << " farthest_path " << farthest_path << endl;
		if(farthest == 0 and (skeleton_waypoints[0].getType() == 0 or (!(skeleton_waypoints[0].getType() == 0) and farthest_path == skeleton_waypoints[0].getPath().size()-1))){
			// cout << "eliminate first" << endl;
			finished_sk_waypoints.push_back(skeleton_waypoints[0]);
			skeleton_waypoints.erase(skeleton_waypoints.begin());
		}
		else if(farthest == 0){
			// cout << "eliminate first path up to farthest_path" << endl;
			vector<CartesianPoint> new_path;
			for(int i = farthest_path+1; i < skeleton_waypoints[0].getPath().size(); i++){
				new_path.push_back(skeleton_waypoints[0].getPath()[i]);
			}
			skeleton_waypoints[0].setPath(new_path);
		}
		else if(farthest > 0 and skeleton_waypoints[farthest].getType() == 0){
			// cout << "eliminate up to farthest region" << endl;
			for(int i = 0; i <= farthest; i++){
				finished_sk_waypoints.push_back(skeleton_waypoints[i]);
			}
			skeleton_waypoints.erase(skeleton_waypoints.begin(), skeleton_waypoints.begin()+farthest);
		}
		else if(farthest > 0 and !(skeleton_waypoints[farthest].getType() == 0)){
			// cout << "eliminate up to farthest path up to farthest_path" << endl;
			if(farthest_path == skeleton_waypoints[farthest].getPath().size()-1){
				// cout << "farthest_path is equal to last in path " << skeleton_waypoints[farthest].getPath().size()-1 << endl;
				for(int i = 0; i <= farthest; i++){
					finished_sk_waypoints.push_back(skeleton_waypoints[i]);
				}
				skeleton_waypoints.erase(skeleton_waypoints.begin(), skeleton_waypoints.begin()+farthest);
			}
			else{
				// cout << "eliminate up to one before farthest path" << endl;
				for(int i = 0; i <= farthest-1; i++){
					finished_sk_waypoints.push_back(skeleton_waypoints[i]);
				}
				skeleton_waypoints.erase(skeleton_waypoints.begin(), skeleton_waypoints.begin()+(farthest-1));
				vector<CartesianPoint> new_path;
				for(int i = farthest_path+1; i < skeleton_waypoints[0].getPath().size(); i++){
					new_path.push_back(skeleton_waypoints[0].getPath()[i]);
				}
				skeleton_waypoints[0].setPath(new_path);
			}
		}
		// cout << "skeleton_waypoints size: " << skeleton_waypoints.size() << " finished_sk_waypoints size: " << finished_sk_waypoints.size() << endl;
		
		cout << "check plan active: " << skeleton_waypoints.size() << endl;
		if(skeleton_waypoints.size() > 0){
			wr = skeleton_waypoints[0];
			cout << "new current waypoint " << this->getX() << " " << this->getY() << endl;
			isPlanActive = true;
		}
		else{
			isPlanActive = false;
		}
	}
	else if(plannerName == "hallwayskel"){
		int farthest = -1;
		int farthest_path = -1;
		int farthest_passage = -1;
		// cout << "skeleton_waypoints size: " << skeleton_waypoints.size() << " finished_sk_waypoints size: " << finished_sk_waypoints.size() << endl;
		for (int i = 0; i < skeleton_waypoints.size(); i++){
			if(skeleton_waypoints[i].getType() == 0){
				// cout << "region " << i << endl;
				if(skeleton_waypoints[i].getRegion().inRegion(CartesianPoint(currentPosition.getX(), currentPosition.getY())) and skeleton_waypoints[i].getRegion().getCenter().get_distance(CartesianPoint(currentPosition.getX(), currentPosition.getY())) < 0.75){
					// cout << "found skeleton region waypoint: " << i << endl;
					farthest = i;
				}
			}
			else if(skeleton_waypoints[i].getType() == 1){
				vector<CartesianPoint> pathBetween = skeleton_waypoints[i].getPath();
				double dis;
				// cout << "pathBetween " << i  << " size: " << pathBetween.size() << endl;
				int farthest_here = -1;
				for (int j = 0; j < pathBetween.size()-1; j++){
					dis = currentPosition.getDistance(pathBetween[j].get_x(), pathBetween[j].get_y());
					if(dis < 0.5 and canAccessPoint(currentLaserEndpoints, CartesianPoint(currentPosition.getX(), currentPosition.getY()), pathBetween[j+1], 20)){
						// cout << "found pathBetween with dist < 0.5: " << j << endl;
						farthest_here = j;
					}
				}
				if(farthest_here > -1 or currentPosition.getDistance(pathBetween[pathBetween.size()-1].get_x(), pathBetween[pathBetween.size()-1].get_y()) < 0.5){
					// cout << "found skeleton region waypoint: " << i << " farthest_here " << farthest_here << endl;
					farthest = i;
					farthest_path = farthest_here;
				}
			}
			else if(skeleton_waypoints[i].getType() == 2){
				// cout << "intersection " << i << endl;
				// vector< vector<int> > grid_points = skeleton_waypoints[i].getPassagePoints();
				// for(int j = 0; j < grid_points.size(); j++){
				// 	if((int)(currentPosition.getX()) == grid_points[j][0] and (int)(currentPosition.getY()) == grid_points[j][1]){
				// 		farthest = i;
				// 		break;
				// 	}
				// }
				if(average_passage[skeleton_waypoints[i].getPassageLabel()-1].get_distance(CartesianPoint(currentPosition.getX(), currentPosition.getY())) < 0.75){
					farthest = i;
				}
			}
			else if(skeleton_waypoints[i].getType() == 3){
				// cout << "passage " << i << endl;
				// vector< vector<int> > grid_points = skeleton_waypoints[i].getPassagePoints();
				// for(int j = 0; j < grid_points.size(); j++){
				// 	if((int)(currentPosition.getX()) == grid_points[j][0] and (int)(currentPosition.getY()) == grid_points[j][1]){
				// 		farthest = i;
				// 		break;
				// 	}
				// }
				vector<CartesianPoint> pathBetween = skeleton_waypoints[i].getPath();
				double dis;
				// cout << "pathBetween " << i  << " size: " << pathBetween.size() << endl;
				int farthest_here = -1;
				for (int j = 0; j < pathBetween.size()-1; j++){
					dis = currentPosition.getDistance(pathBetween[j].get_x(), pathBetween[j].get_y());
					if(dis < 0.75 and canAccessPoint(currentLaserEndpoints, CartesianPoint(currentPosition.getX(), currentPosition.getY()), pathBetween[j+1], 20)){
						// cout << "found pathBetween with dist < 0.5: " << j << endl;
						farthest_here = j;
					}
				}
				if(farthest_here > -1 or currentPosition.getDistance(pathBetween[pathBetween.size()-1].get_x(), pathBetween[pathBetween.size()-1].get_y()) < 0.75){
					// cout << "found skeleton region waypoint: " << i << " farthest_here " << farthest_here << endl;
					farthest = i;
					farthest_passage = farthest_here;
				}
			}
		}
		// cout << "farthest " << farthest << " farthest_path " << farthest_path << " farthest_passage " << farthest_passage << endl;
		if(farthest == 0 and (skeleton_waypoints[0].getType() == 0 or (skeleton_waypoints[0].getType() == 1 and farthest_path == skeleton_waypoints[0].getPath().size()-1) or skeleton_waypoints[0].getType() == 2 or (skeleton_waypoints[0].getType() == 3 and farthest_passage == skeleton_waypoints[0].getPath().size()-1))){
			// cout << "eliminate first" << endl;
			finished_sk_waypoints.push_back(skeleton_waypoints[0]);
			skeleton_waypoints.erase(skeleton_waypoints.begin());
		}
		else if(farthest == 0 and skeleton_waypoints[0].getType() == 1){
			// cout << "eliminate first path up to farthest_path" << endl;
			vector<CartesianPoint> new_path;
			for(int i = farthest_path+1; i < skeleton_waypoints[0].getPath().size(); i++){
				new_path.push_back(skeleton_waypoints[0].getPath()[i]);
			}
			skeleton_waypoints[0].setPath(new_path);
		}
		else if(farthest == 0 and skeleton_waypoints[0].getType() == 3){
			// cout << "eliminate first passage path up to farthest_passage" << endl;
			vector<CartesianPoint> new_path;
			for(int i = farthest_passage+1; i < skeleton_waypoints[0].getPath().size(); i++){
				new_path.push_back(skeleton_waypoints[0].getPath()[i]);
			}
			skeleton_waypoints[0].setPath(new_path);
		}
		else if(farthest > 0 and (skeleton_waypoints[farthest].getType() == 0 or skeleton_waypoints[farthest].getType() == 2)){
			// cout << "eliminate up to farthest region or intersection" << endl;
			for(int i = 0; i <= farthest; i++){
				finished_sk_waypoints.push_back(skeleton_waypoints[i]);
			}
			skeleton_waypoints.erase(skeleton_waypoints.begin(), skeleton_waypoints.begin()+farthest);
		}
		else if(farthest > 0 and skeleton_waypoints[farthest].getType() == 1){
			// cout << "eliminate up to farthest path up to farthest_path" << endl;
			if(farthest_path == skeleton_waypoints[farthest].getPath().size()-1){
				// cout << "farthest_path is equal to last in path " << skeleton_waypoints[farthest].getPath().size()-1 << endl;
				for(int i = 0; i <= farthest; i++){
					finished_sk_waypoints.push_back(skeleton_waypoints[i]);
				}
				skeleton_waypoints.erase(skeleton_waypoints.begin(), skeleton_waypoints.begin()+farthest);
			}
			else{
				// cout << "eliminate up to one before farthest path" << endl;
				for(int i = 0; i <= farthest-1; i++){
					finished_sk_waypoints.push_back(skeleton_waypoints[i]);
				}
				skeleton_waypoints.erase(skeleton_waypoints.begin(), skeleton_waypoints.begin()+(farthest-1));
				vector<CartesianPoint> new_path;
				for(int i = farthest_path+1; i < skeleton_waypoints[0].getPath().size(); i++){
					new_path.push_back(skeleton_waypoints[0].getPath()[i]);
				}
				skeleton_waypoints[0].setPath(new_path);
			}
		}
		else if(farthest > 0 and skeleton_waypoints[farthest].getType() == 3){
			// cout << "eliminate up to farthest path up to farthest_passage" << endl;
			if(farthest_passage == skeleton_waypoints[farthest].getPath().size()-1){
				// cout << "farthest_passage is equal to last in path " << skeleton_waypoints[farthest].getPath().size()-1 << endl;
				for(int i = 0; i <= farthest; i++){
					finished_sk_waypoints.push_back(skeleton_waypoints[i]);
				}
				skeleton_waypoints.erase(skeleton_waypoints.begin(), skeleton_waypoints.begin()+farthest);
			}
			else{
				// cout << "eliminate up to one before farthest path" << endl;
				for(int i = 0; i <= farthest-1; i++){
					finished_sk_waypoints.push_back(skeleton_waypoints[i]);
				}
				skeleton_waypoints.erase(skeleton_waypoints.begin(), skeleton_waypoints.begin()+(farthest-1));
				vector<CartesianPoint> new_path;
				for(int i = farthest_passage+1; i < skeleton_waypoints[0].getPath().size(); i++){
					new_path.push_back(skeleton_waypoints[0].getPath()[i]);
				}
				skeleton_waypoints[0].setPath(new_path);
			}
		}
		// else if(farthest > 0 and skeleton_waypoints[farthest].getType() == 3){
		// 	for(int i = 0; i <= (farthest-1); i++){
		// 		finished_sk_waypoints.push_back(skeleton_waypoints[i]);
		// 	}
		// 	if(farthest-1 == 0){
		// 		skeleton_waypoints.erase(skeleton_waypoints.begin());
		// 	}
		// 	else{
		// 		skeleton_waypoints.erase(skeleton_waypoints.begin(), skeleton_waypoints.begin()+(farthest-1));
		// 	}
		// }
		// cout << "skeleton_waypoints size: " << skeleton_waypoints.size() << " finished_sk_waypoints size: " << finished_sk_waypoints.size() << endl;
		cout << "check plan active: " << skeleton_waypoints.size() << endl;
		if(skeleton_waypoints.size() > 0){
			wr = skeleton_waypoints[0];
			cout << "new current waypoint " << this->getX() << " " << this->getY() << endl;
			isPlanActive = true;
		}
		else{
			isPlanActive = false;
		}
	}
	//cout << "end setup next waypoint" << endl;
   }

  void skipWaypoint(){
  	if(plannerName == "skeleton" or plannerName == "hallwayskel"){
  		finished_sk_waypoints.push_back(skeleton_waypoints[0]);
		skeleton_waypoints.erase(skeleton_waypoints.begin());
		if(skeleton_waypoints.size() > 0){
			wr = skeleton_waypoints[0];
			cout << "new current waypoint " << this->getX() << " " << this->getY() << endl;
			isPlanActive = true;
		}
		else{
			isPlanActive = false;
		}
  	}
  }

  bool isTaskComplete(Position currentPosition){
	bool status = false;
	double dis = currentPosition.getDistance(x, y);
	if (dis < 1){
		status = true;
	}
	return status;
  }


 //   bool isWaypointComplete(Position currentPosition){
	// bool status = false;
	// double dis = currentPosition.getDistance(wx, wy);
	// if (isPlanActive && (dis < 0.75)){
	// 	status = true;
	// }
	// return status;
 //   }

   bool isAnyWaypointComplete(Position currentPosition, vector<CartesianPoint> currentLaserEndpoints){
	bool status = false;
	if(isPlanActive && plannerName != "skeleton" && plannerName != "hallwayskel"){
		for (int i = 0; i < waypoints.size(); i++){
			double dis = currentPosition.getDistance(waypoints[i].get_x(), waypoints[i].get_y());
			if ((dis < 0.75)){
				status = true;
				break;
			}
		}
	}
	else if(isPlanActive && plannerName == "skeleton"){
		for (int i = 0; i < skeleton_waypoints.size(); i++){
			if(skeleton_waypoints[i].getType() == 0){
				if(skeleton_waypoints[i].getRegion().inRegion(CartesianPoint(currentPosition.getX(), currentPosition.getY())) and skeleton_waypoints[i].getRegion().getCenter().get_distance(CartesianPoint(currentPosition.getX(), currentPosition.getY())) < 0.75){
					status = true;
					break;
				}
			}
			else{
				vector<CartesianPoint> pathBetween = skeleton_waypoints[i].getPath();
				double dis;
				int farthest_path = -1;
				// cout << "pathBetween size: " << pathBetween.size() << endl;
				for (int j = 0; j < pathBetween.size()-1; j++){
					dis = currentPosition.getDistance(pathBetween[j].get_x(), pathBetween[j].get_y());
					if(dis < 0.5 and canAccessPoint(currentLaserEndpoints, CartesianPoint(currentPosition.getX(), currentPosition.getY()), pathBetween[j+1], 20)){
						// cout << "found pathBetween with dist < 0.5: " << j << endl;
						farthest_path = j;
					}
				}
				if(farthest_path > -1 or currentPosition.getDistance(pathBetween[pathBetween.size()-1].get_x(), pathBetween[pathBetween.size()-1].get_y()) < 0.5){
					status = true;
					break;
				}
			}
		}
	}
	else if(isPlanActive && plannerName == "hallwayskel"){
		for (int i = 0; i < skeleton_waypoints.size(); i++){
			if(skeleton_waypoints[i].getType() == 0){
				if(skeleton_waypoints[i].getRegion().inRegion(CartesianPoint(currentPosition.getX(), currentPosition.getY())) and skeleton_waypoints[i].getRegion().getCenter().get_distance(CartesianPoint(currentPosition.getX(), currentPosition.getY())) < 0.75){
					status = true;
					break;
				}
			}
			else if(skeleton_waypoints[i].getType() == 1){
				vector<CartesianPoint> pathBetween = skeleton_waypoints[i].getPath();
				double dis;
				int farthest_path = -1;
				// cout << "pathBetween size: " << pathBetween.size() << endl;
				for (int j = 0; j < pathBetween.size()-1; j++){
					dis = currentPosition.getDistance(pathBetween[j].get_x(), pathBetween[j].get_y());
					if(dis < 0.5 and canAccessPoint(currentLaserEndpoints, CartesianPoint(currentPosition.getX(), currentPosition.getY()), pathBetween[j+1], 20)){
						// cout << "found pathBetween with dist < 0.5: " << j << endl;
						farthest_path = j;
					}
				}
				if(farthest_path > -1 or currentPosition.getDistance(pathBetween[pathBetween.size()-1].get_x(), pathBetween[pathBetween.size()-1].get_y()) < 0.5){
					status = true;
					break;
				}
			}
			else if(skeleton_waypoints[i].getType() == 2){
				if(average_passage[skeleton_waypoints[i].getPassageLabel()-1].get_distance(CartesianPoint(currentPosition.getX(), currentPosition.getY())) < 0.75){
					status = true;
					break;
				}
			}
			else if(skeleton_waypoints[i].getType() == 3){
				vector<CartesianPoint> pathBetween = skeleton_waypoints[i].getPath();
				double dis;
				int farthest_path = -1;
				// cout << "pathBetween size: " << pathBetween.size() << endl;
				for (int j = 0; j < pathBetween.size()-1; j++){
					dis = currentPosition.getDistance(pathBetween[j].get_x(), pathBetween[j].get_y());
					if(dis < 0.75 and canAccessPoint(currentLaserEndpoints, CartesianPoint(currentPosition.getX(), currentPosition.getY()), pathBetween[j+1], 20)){
						// cout << "found pathBetween with dist < 0.5: " << j << endl;
						farthest_path = j;
					}
				}
				if(farthest_path > -1 or currentPosition.getDistance(pathBetween[pathBetween.size()-1].get_x(), pathBetween[pathBetween.size()-1].get_y()) < 0.75){
					status = true;
					break;
				}
			}
			// else if(skeleton_waypoints[i].getType() == 3){
			// 	if(i > 0){
			// 		vector< vector<int> > grid_points = skeleton_waypoints[i].getPassagePoints();
			// 		for(int j = 0; j < grid_points.size(); j++){
			// 			if((int)(currentPosition.getX()) == grid_points[j][0] and (int)(currentPosition.getY()) == grid_points[j][1]){
			// 				status = true;
			// 				break;
			// 			}
			// 		}
			// 	}
			// }
		}
	}
	return status;
   }

  double getPathCostInNavGraph(){return pathCostInNavGraph;}
  double getPathCostInNavOrigGraph(){return pathCostInNavOrigGraph;}
  double getOrigPathCostInOrigNavGraph(){return origPathCostInOrigNavGraph;}
  double getOrigPathCostInNavGraph(){return origPathCostInNavGraph;}

  void updatePlanPositions(double p_x, double p_y){
  	cout << "In updatePlanPositions p_x " << p_x << " p_y " << p_y << endl;
  	int floor_x = (int)(floor(p_x))-1;
  	int floor_y = (int)(floor(p_y))-1;
  	int ceil_x = (int)(ceil(p_x))+1;
  	int ceil_y = (int)(ceil(p_y))+1;
  	if(floor_x < 0)
  		floor_x = 0;
  	if(floor_y < 0)
  		floor_y = 0;
  	if(ceil_x >= planPositions[0].size())
  		ceil_x = planPositions[0].size()-1;
  	if(ceil_y >= planPositions[0].size())
  		ceil_y = planPositions[0].size()-1;
  	cout << "Updating planPositions floor " << floor_x << " " << floor_y << " ceil " << ceil_x << " " << ceil_y << endl;
  	for(int i = floor_x; i <= ceil_x; i++){
  		for(int j = floor_y; j <= ceil_y; j++){
  			planPositions[i][j] = 1;
  		}
  	}
  	// if(floor_x >= 0 and floor_y >= 0 and floor_x < planPositions[0].size() and floor_y < planPositions[0].size()){
  	// 	planPositions[floor_x][floor_y] = 1;
  	// }
  	// if(floor_x >= 0 and ceil_y >= 0 and floor_x < planPositions[0].size() and ceil_y < planPositions[0].size()){
  	// 	planPositions[floor_x][ceil_y] = 1;
  	// }
  	// if(ceil_x >= 0 and floor_y >= 0 and ceil_x < planPositions[0].size() and floor_y < planPositions[0].size()){
  	// 	planPositions[ceil_x][floor_y] = 1;
  	// }
  	// if(ceil_x >= 0 and ceil_y >= 0 and ceil_x < planPositions[0].size() and ceil_y < planPositions[0].size()){
  	// 	planPositions[ceil_x][ceil_y] = 1;
  	// }
  }

  bool getPlanPositionValue(double p_x, double p_y){
  	cout << "In getPlanPositionValue" << endl;
  	int floor_x = (int)(floor(p_x));
  	int floor_y = (int)(floor(p_y));
  	int ceil_x = (int)(ceil(p_x));
  	int ceil_y = (int)(ceil(p_y));
  	if(floor_x >= 0 and floor_y >= 0 and floor_x < planPositions[0].size() and floor_y < planPositions[0].size()){
  		// cout << "planPositions[" << floor_x << "][" << floor_y << "] = " << planPositions[floor_x][floor_y] << endl;
  		if(planPositions[floor_x][floor_y] == 1)
  			return true;
  	}
  	if(floor_x >= 0 and ceil_y >= 0 and floor_x < planPositions[0].size() and ceil_y < planPositions[0].size()){
  		// cout << "planPositions[" << floor_x << "][" << ceil_y << "] = " << planPositions[floor_x][ceil_y] << endl;
  		if(planPositions[floor_x][ceil_y] == 1)
  			return true;
  	}
  	if(ceil_x >= 0 and floor_y >= 0 and ceil_x < planPositions[0].size() and floor_y < planPositions[0].size()){
  		// cout << "planPositions[" << ceil_x << "][" << floor_y << "] = " << planPositions[ceil_x][floor_y] << endl;
  		if(planPositions[ceil_x][floor_y] == 1)
  			return true;
  	}
  	if(ceil_x >= 0 and ceil_y >= 0 and ceil_x < planPositions[0].size() and ceil_y < planPositions[0].size()){
  		// cout << "planPositions[" << ceil_x << "][" << ceil_y << "] = " << planPositions[ceil_x][ceil_y] << endl;
  		if(planPositions[ceil_x][ceil_y] == 1)
  			return true;
  	}
  	return false;
  }

  void resetPlanPositions(){
  	cout << "In resetPlanPositions" << endl;
  	vector< vector<int> > grid;
  	// int dimension = 330;
	for(int i = 0; i < dimension; i++){
		vector<int> col;
		for(int j = 0; j < dimension; j++){
			col.push_back(0);
		}
		grid.push_back(col);
	}
	planPositions = grid;
  }

  void setPassageValues(vector< vector<int> > pg, map<int, vector< vector<int> > > pgn, map<int, vector< vector<int> > > pge, vector< vector<int> > pgr, vector< vector<int> > ap, vector< vector<CartesianPoint> > gt, vector< vector<int> > gti, vector< vector<CartesianPoint> > git){
  	// cout << "inside setPassageValues" << endl;
	passage_grid = pg;
	passage_graph_nodes = pgn;
	passage_graph_edges = pge;
	passage_graph = pgr;
	// cout << "before average_passage calc" << endl;
	for(int i = 0; i < ap.size(); i++){
		if(ap[i].size() > 0){
			average_passage.push_back(CartesianPoint(((double)(ap[i][0]))/100.0, ((double)(ap[i][1]))/100.0));
		}
		else{
			average_passage.push_back(CartesianPoint(0,0));
		}
	}
	// cout << "calculated average_passage" << endl;
	for(int i = 0; i < passage_graph.size(); i++){
		vector< vector<int> > passage_points = passage_graph_edges[passage_graph[i][1]];
		int min_x = 100000, max_x = 0, min_y = 100000, max_y = 0;
		for(int j = 0; j < passage_points.size(); j++){
			if(passage_points[j][0] < min_x){
				min_x = passage_points[j][0];
			}
			if(passage_points[j][0] > max_x){
				max_x = passage_points[j][0];
			}
			if(passage_points[j][1] < min_y){
				min_y = passage_points[j][1];
			}
			if(passage_points[j][1] > max_y){
				max_y = passage_points[j][1];
			}
		}
		if((max_x - min_x) > (max_y - min_y)){
			passage_graph_edges_orientation[passage_graph[i][1]] = 1;
		}
		else{
			passage_graph_edges_orientation[passage_graph[i][1]] = 0;
		}
	}
	// cout << "calculated passage_graph_edges_orientation" << endl;
	graph_trails = gt;
	graph_through_intersections = gti;
    graph_intersection_trails = git;
  }
   
 private:
  
  int dimension;

  // Current plan generated by A*, stored as waypoints , index 0 being the beginning of the plan
  vector<CartesianPoint> waypoints;
  vector<CartesianPoint> tierTwoWaypoints;
  vector<CartesianPoint> origWaypoints;
  double pathCostInNavGraph, pathCostInNavOrigGraph;
  double origPathCostInOrigNavGraph, origPathCostInNavGraph;

  vector<sk_waypoint> skeleton_waypoints;
  vector<sk_waypoint> alternate_skeleton_waypoints;
  vector<sk_waypoint> finished_sk_waypoints;

  vector< vector<int> > passage_grid;
  map<int, vector< vector<int> > > passage_graph_nodes;
  map<int, vector< vector<int> > > passage_graph_edges;
  map<int, int> passage_graph_edges_orientation;
  vector< vector<int> > passage_graph;
  vector<CartesianPoint> average_passage;
  vector< vector<CartesianPoint> > graph_trails;
  vector< vector<int> > graph_through_intersections;
  vector< vector<CartesianPoint> > graph_intersection_trails;

  list<int> waypointInd;
  vector< list<int> > plansInds;
  vector< list<int> > origPlansInds;

  CartesianPoint currentWaypoint;

  bool isPlanActive;
  bool isPlanComplete;
  PathPlanner *pathPlanner;
  string plannerName;
  Graph *navGraph;
  Graph *origNavGraph;
  
  //<! expected task execution time in seconds 
  float time_taken;

  // distance covered by the robot to get to the target
  float distance_travelled; 

  // Sequence of decisions made while pursuing the target
  std::vector<FORRAction> *decisionSequence;

  // Position History as is: Set of all unique positions the robot has been in , while pursuing the target
  std::vector<Position> *pos_hist; 

  // Laser scan history as is:
  vector< vector<CartesianPoint> > *laser_hist; 

  // Laser scan history sensor
  vector< sensor_msgs::msg::LaserScan > *laser_scan_hist;

  // Cleaned Position History, along with its corresponding laser scan data : Set of cleaned positions
  std::pair < std::vector<CartesianPoint>, std::vector<vector<CartesianPoint> > > *cleaned_trail;

  // decision count
  int decision_count;

  // t1, t2, t3 counters
  int tier1_decisions, tier2_decisions, tier3_decisions;

  //<! The point in the map, that the robot needs to go in order to execute this task 
  double x,y,wx,wy;
  sk_waypoint wr;

  // Plan positions
  vector< vector<int> > planPositions;

};

#endif