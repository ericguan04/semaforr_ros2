// Publish robot status into Rviz for visualization and debugging


#include <iostream>
#include <stdlib.h>
#include "Beliefs.h"
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/transform_datatypes.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using namespace std;


class Visualizer
{
private:
	//! We will be publishing to the "target_point" topic to display target point on rviz
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr all_targets_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr remaining_targets_pub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr region_pub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr exits_pub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr skeleton_pub_;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr conveyor_pub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr hallway1_pub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr hallway2_pub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr hallway3_pub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr hallway4_pub_;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trails_pub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoints_pub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr original_plan_pub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr nodes1_pub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr nodes2_pub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr edges_pub_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr edges_cost_pub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr doors_pub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr barriers_pub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr walls_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr highway_pub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr highway_plan_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr highway_target_pub_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr highway_stack_pub_;

	std::shared_ptr<rclcpp::Node> node_;

	Controller *con;
	Beliefs *beliefs;
	int visualized;

public:
	//! ROS node initialization
	Visualizer(rclcpp::Node::SharedPtr node, Controller *c) : node_(node)
	{
		visualized = false;

		// Setup publishers using create_publisher
		target_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>("target_point", 10);
		waypoint_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>("waypoint", 10);
		all_targets_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("all_targets", 10);
		remaining_targets_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("remaining_targets", 10);
		plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("plan", 10);
		waypoints_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("waypoints", 10);
		original_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("original_plan", 10);
		conveyor_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("conveyor", 10);
		hallway1_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("hallway1", 10);
		hallway2_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("hallway2", 10);
		hallway3_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("hallway3", 10);
		hallway4_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("hallway4", 10);
		occupancy_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy", 10);
		region_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("region", 10);
		exits_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("exits", 10);
		skeleton_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("skeleton", 10);
		nodes1_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("nodes1", 10);
		nodes2_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("nodes2", 10);
		edges_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("edges", 10);
		edges_cost_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("edges_cost", 10);
		trails_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("trail", 10);
		stats_pub_ = node_->create_publisher<std_msgs::msg::String>("decision_log", 10);
		doors_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("door", 10);
		barriers_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("barrier", 10);
		walls_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("walls", 10);
		pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("decision_pose", 10);
		laser_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>("decision_laser", 10);
		highway_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("highway", 10);
		highway_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("highway_plan", 10);
		highway_target_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>("highway_target_point", 10);
		highway_stack_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("highway_stack", 10);

		// Setup controller and beliefs
		con = c;
		beliefs = con->getBeliefs();
	}


  void publish(){
	if(beliefs->getAgentState()->getCurrentTask() != NULL and con->getHighwayFinished()){
		publish_next_target();
		publish_next_waypoint();
		publish_plan();
		publish_original_plan();
		publish_all_targets();
		publish_remaining_targets();
	}
	// publish_nodes();
	// publish_reachable_nodes();
	// publish_edges();
        /*if(visualized < 5){
		publish_nodes();
		publish_reachable_nodes();
		//publish_edges();
		visualized++;
	}*/
	//publish_edges_cost();
	publish_conveyor();
	publish_hallway1();
	publish_hallway2();
	publish_hallway3();
	publish_hallway4();
	publish_region();
	publish_exits();
	publish_skeleton();
	publish_trails();
	publish_doors();
	// publish_barriers();
	publish_walls();
	// //publish_occupancy();
	publish_highway();
	publish_highway_plan();
	publish_highway_target();
	publish_highway_stack();
  }


  void publish_nodes(){
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = "map";
    	marker.header.stamp = node_->now();
	marker.ns = "basic_shapes";
	marker.type = visualization_msgs::msg::Marker::POINTS;
    	marker.pose.position.x = 0;
    	marker.pose.position.y = 0;
    	marker.pose.position.z = 0;
    	marker.pose.orientation.x = 0.0;
    	marker.pose.orientation.y = 0.0;
    	marker.pose.orientation.z = 0.0;
    	marker.pose.orientation.w = 1.0;
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = marker.scale.y = 0.15;
	marker.scale.z = 0.15;
	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 0.5;
	marker.lifetime = rclcpp::Duration::from_seconds(0.0);

	vector<Edge*> edges = con->getPlanners()[0]->getGraph()->getEdges();
	vector<Node*> nodes = con->getPlanners()[0]->getGraph()->getNodes();

	for( int i = 0; i < nodes.size(); i++ ){
		if(nodes[i]->getInBuffer()){
			float x = nodes[i]->getX()/100.0;
			float y = nodes[i]->getY()/100.0;
			geometry_msgs::msg::Point point;
			point.x = x;
			point.y = y;
			point.z = 0;
			marker.points.push_back(point);
		}
	}
	nodes1_pub_->publish(marker);
  }

  void publish_reachable_nodes(){
	// cout << "publish nodes" << endl;
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = "map";
    	marker.header.stamp = node_->now();
	marker.ns = "basic_shapes";
	marker.type = visualization_msgs::msg::Marker::POINTS;
    	marker.pose.position.x = 0;
    	marker.pose.position.y = 0;
    	marker.pose.position.z = 0;
    	marker.pose.orientation.x = 0.0;
    	marker.pose.orientation.y = 0.0;
    	marker.pose.orientation.z = 0.0;
    	marker.pose.orientation.w = 1.0;
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = marker.scale.y = 0.15;
	marker.scale.z = 0.15;
	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 0.5;
	marker.lifetime = rclcpp::Duration::from_seconds(0.0);

	vector<Edge*> edges = con->getPlanners()[0]->getGraph()->getEdges();
	vector<Node*> nodes = con->getPlanners()[0]->getGraph()->getNodes();

	for( int i = 0; i < nodes.size(); i++ ){
		if(!nodes[i]->getInBuffer()){
			float x = nodes[i]->getX()/100.0;
			float y = nodes[i]->getY()/100.0;
			geometry_msgs::msg::Point point;
			point.x = x;
			point.y = y;
			point.z = 0;
			marker.points.push_back(point);
		}
	}
	nodes2_pub_->publish(marker);
  }

  void publish_edges_cost(){
	visualization_msgs::msg::MarkerArray markerArrayCost;

	Graph *graph = con->getPlanners()[0]->getGraph();
	vector<Edge*> edges = graph->getEdges();
	vector<Node*> nodes = graph->getNodes();

	for( int i = 0; i < edges.size(); i++ ){
		visualization_msgs::msg::Marker costMarker;

		costMarker.header.frame_id = "map";
	    	costMarker.header.stamp = node_->now();
		costMarker.ns = "basic_shapes";
		costMarker.id = i;
		costMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
		costMarker.action = visualization_msgs::msg::Marker::ADD;
	    	costMarker.pose.orientation.x = 0.0;
	    	costMarker.pose.orientation.y = 0.0;
	    	costMarker.pose.orientation.z = 0.0;
	    	costMarker.pose.orientation.w = 1.0;
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		costMarker.scale.x = 1;

		// Set the color -- be sure to set alpha to something non-zero!
		costMarker.color.r = 0.0f;
		costMarker.color.g = 1.0f;
		costMarker.color.b = 0.0f;
		costMarker.color.a = 0.5;
		costMarker.lifetime = rclcpp::Duration::from_seconds(0.0);


		Node *from = graph->getNodePtr(edges[i]->getFrom());
		Node *to = graph->getNodePtr(edges[i]->getTo());
		geometry_msgs::msg::Point f;
		f.x = from->getX()/100.0;
		f.y = from->getY()/100.0;
		f.z = 0;
		geometry_msgs::msg::Point t;
		t.x = to->getX()/100.0;
		t.y = to->getY()/100.0;
		t.z = 0;

		costMarker.pose.position.x = (f.x + t.x)/2.0;
		costMarker.pose.position.y = (f.y + t.y)/2.0;
		costMarker.pose.position.z = 0;
		//to_string(edges[i]->getCost(true)) + " " + to_string(edges[i]->getCost(false));
		costMarker.text = "a";

		markerArrayCost.markers.push_back(costMarker);
	}
	edges_cost_pub_->publish(markerArrayCost);
  }

  void publish_edges(){
	visualization_msgs::msg::MarkerArray markerArray;

	Graph *graph = con->getPlanners()[0]->getGraph();
	vector<Edge*> edges = graph->getEdges();
	vector<Node*> nodes = graph->getNodes();

	for( int i = 0; i < edges.size(); i++ ){
		visualization_msgs::msg::Marker marker;
		marker.header.frame_id = "map";
	    	marker.header.stamp = node_->now();
		marker.ns = "basic_shapes";
		marker.id = i;
		marker.type = visualization_msgs::msg::Marker::LINE_LIST;
	    	marker.pose.position.x = 0;
	    	marker.pose.position.y = 0;
	    	marker.pose.position.z = 0;
	    	marker.pose.orientation.x = 0.0;
	    	marker.pose.orientation.y = 0.0;
	    	marker.pose.orientation.z = 0.0;
	    	marker.pose.orientation.w = 1.0;
		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = 0.1;

		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 0.5;
		marker.lifetime = rclcpp::Duration::from_seconds(0.0);

		Node *from = graph->getNodePtr(edges[i]->getFrom());
		Node *to = graph->getNodePtr(edges[i]->getTo());
		geometry_msgs::msg::Point f;
		f.x = from->getX()/100.0;
		f.y = from->getY()/100.0;
		f.z = 0;
		geometry_msgs::msg::Point t;
		t.x = to->getX()/100.0;
		t.y = to->getY()/100.0;
		t.z = 0;
		marker.points.push_back(f);
		marker.points.push_back(t);

		markerArray.markers.push_back(marker);
	}
	edges_pub_->publish(markerArray);
  }

  void publishLog(FORRAction decision, double overallTimeSec, double computationTimeSec){
	publish_log(decision, overallTimeSec, computationTimeSec);
  }

  void publish_next_target(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside visualization tool!!");
	geometry_msgs::msg::PointStamped target;
	target.header.frame_id = "map";
	target.header.stamp = node_->now();
	target.point.x = beliefs->getAgentState()->getCurrentTask()->getTaskX();
	target.point.y = beliefs->getAgentState()->getCurrentTask()->getTaskY();
	target.point.z = 0;
	target_pub_->publish(target);
  }

  void publish_highway_target(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside visualization tool!!");
	geometry_msgs::msg::PointStamped target;
	target.header.frame_id = "map";
	target.header.stamp = node_->now();
	if(con->getHighwaysOn() == 1){
		target.point.x = con->gethighwayExploration()->getHighwayTarget().getX();
		target.point.y = con->gethighwayExploration()->getHighwayTarget().getY();
		target.point.z = 0;
		highway_target_pub_->publish(target);
	}
	else if(con->getHighwaysOn() == 2){
		target.point.x = con->getfrontierExploration()->getFrontierTarget().getX();
		target.point.y = con->getfrontierExploration()->getFrontierTarget().getY();
		target.point.z = 0;
		highway_target_pub_->publish(target);
	}
  }

  void publish_next_waypoint(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside visualization tool!!");
	geometry_msgs::msg::PointStamped waypoint;
	waypoint.header.frame_id = "map";
	waypoint.header.stamp = node_->now();
	waypoint.point.x = beliefs->getAgentState()->getCurrentTask()->getX();
	waypoint.point.y = beliefs->getAgentState()->getCurrentTask()->getY();
	waypoint.point.z = 0;
	waypoint_pub_->publish(waypoint);
  }


  void publish_plan(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish plan!!");
	nav_msgs::msg::Path path;
	path.header.frame_id = "map";
	path.header.stamp = node_->now();

	vector <CartesianPoint> waypoints = beliefs->getAgentState()->getCurrentTask()->getWaypoints();
	// double pathCostInNavGraph = beliefs->getAgentState()->getCurrentTask()->getPathCostInNavGraph();
	// double pathCostInNavOrigGraph = beliefs->getAgentState()->getCurrentTask()->getPathCostInNavOrigGraph();
	// std::stringstream output;
	// output << pathCostInNavGraph << "\t" << pathCostInNavOrigGraph;
	// path.header.frame_id = output.str();

	for(int i = 0; i < waypoints.size(); i++){
		geometry_msgs::msg::PoseStamped poseStamped;
		poseStamped.header.frame_id = "map";
		poseStamped.header.stamp = path.header.stamp;
		poseStamped.pose.position.x = waypoints[i].get_x();
		poseStamped.pose.position.y = waypoints[i].get_y();
		path.poses.push_back(poseStamped);
	}
	plan_pub_->publish(path);

	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = node_->now();
	marker.ns = "basic_shapes";
	marker.type = visualization_msgs::msg::Marker::POINTS;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = marker.scale.y = 0.25;
	marker.scale.z = 0.25;
	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 0.5;
	marker.lifetime = rclcpp::Duration::from_seconds(0.0);

	for(int i = 0; i < waypoints.size(); i++){
		float x = waypoints[i].get_x();
		float y = waypoints[i].get_y();
		geometry_msgs::msg::Point point;
		point.x = x;
		point.y = y;
		point.z = 0;
		marker.points.push_back(point);
	}

  	waypoints_pub_->publish(marker);
 }

 void publish_highway_plan(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish highway plan!!");
	nav_msgs::msg::Path path;
	path.header.frame_id = "map";
	path.header.stamp = node_->now();
	vector< vector<double> > waypoints;
	if(con->getHighwaysOn() == 1){
		waypoints = con->gethighwayExploration()->getHighwayPath();
	}
	else if(con->getHighwaysOn() == 2){
		waypoints = con->getfrontierExploration()->getFrontierPath();
	}
	for(int i = 0; i < waypoints.size(); i++){
		geometry_msgs::msg::PoseStamped poseStamped;
		poseStamped.header.frame_id = "map";
		poseStamped.header.stamp = path.header.stamp;
		poseStamped.pose.position.x = waypoints[i][0];
		poseStamped.pose.position.y = waypoints[i][1];
		path.poses.push_back(poseStamped);
	}
	highway_plan_pub_->publish(path);
 }

 void publish_original_plan(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish original plan!!");
	nav_msgs::msg::Path path;
	path.header.frame_id = "map";
	path.header.stamp = node_->now();

	vector <CartesianPoint> waypoints = beliefs->getAgentState()->getCurrentTask()->getOrigWaypoints();
	// double origPathCostInNavGraph = beliefs->getAgentState()->getCurrentTask()->getOrigPathCostInNavGraph();
	// double origPathCostInOrigNavGraph = beliefs->getAgentState()->getCurrentTask()->getOrigPathCostInOrigNavGraph();
	// std::stringstream output;
	// output << origPathCostInNavGraph << "\t" << origPathCostInOrigNavGraph;
	// path.header.frame_id = output.str();

	for(int i = 0; i < waypoints.size(); i++){
		geometry_msgs::msg::PoseStamped poseStamped;
		poseStamped.header.frame_id = "map";
		poseStamped.header.stamp = path.header.stamp;
		poseStamped.pose.position.x = waypoints[i].get_x();
		poseStamped.pose.position.y = waypoints[i].get_y();
		path.poses.push_back(poseStamped);
	}
	original_plan_pub_->publish(path);
 }

  void publish_conveyor(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish conveyor");
	nav_msgs::msg::OccupancyGrid grid;

	grid.header.frame_id = "map";
	grid.header.stamp = node_->now();
	grid.info.map_load_time = node_->now();

	grid.info.origin.orientation.w = 0;
	grid.info.resolution = beliefs->getSpatialModel()->getConveyors()->getGranularity();
	grid.info.width = beliefs->getSpatialModel()->getConveyors()->getBoxWidth();
	grid.info.height = beliefs->getSpatialModel()->getConveyors()->getBoxHeight();

	vector< vector<int> > conveyors = beliefs->getSpatialModel()->getConveyors()->getConveyors();
	for(int j = 0; j < grid.info.height; j++){
		for(int i = 0; i < grid.info.width; i++){
			grid.data.push_back(conveyors[i][j]);
		}
	}
	conveyor_pub_->publish(grid);
  }

  void publish_hallway1(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish hallway1");
	visualization_msgs::msg::Marker marker;
	vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
	// cout << "There are currently " << hallways.size() << " hallways" << endl;
	marker.header.frame_id = "map";
	marker.header.stamp = node_->now();
	marker.ns = "basic_shapes";
	marker.type = visualization_msgs::msg::Marker::POINTS;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.lifetime = rclcpp::Duration::from_seconds(0.0);
	for(int i = 0; i < hallways.size(); i++){
		vector<CartesianPoint> points = hallways[i].getPoints();
		int hallway_type = hallways[i].getHallwayType();
		// cout << "Number of points = " << points.size() << " Hallway type = " << hallway_type << endl;
		if(hallway_type == 0){
			for(int j = 0; j < points.size(); j++){
				float x = points[j].get_x();
				float y = points[j].get_y();
				geometry_msgs::msg::Point point;
				point.x = x;
				point.y = y;
				point.z = 0;
				marker.points.push_back(point);
				std_msgs::msg::ColorRGBA color;
				color.a = 0.5;
				color.r = 1.0;
				color.g = 0.0;
				color.b = 0.0;
				marker.colors.push_back(color);
			}
		}
	}
	hallway1_pub_->publish(marker);
  }

  void publish_hallway2(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish hallway2");
	visualization_msgs::msg::Marker marker;
	vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
	//cout << "There are currently " << hallways.size() << " hallways" << endl;
	marker.header.frame_id = "map";
	marker.header.stamp = node_->now();
	marker.ns = "basic_shapes";
	marker.type = visualization_msgs::msg::Marker::POINTS;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.lifetime = rclcpp::Duration::from_seconds(0.0);
	for(int i = 0; i < hallways.size(); i++){
		vector<CartesianPoint> points = hallways[i].getPoints();
		int hallway_type = hallways[i].getHallwayType();
		//cout << "Number of points = " << points.size() << " Hallway type = " << hallway_type << endl;
		if(hallway_type == 1){
			for(int j = 0; j < points.size(); j++){
				float x = points[j].get_x();
				float y = points[j].get_y();
				geometry_msgs::msg::Point point;
				point.x = x;
				point.y = y;
				point.z = 0;
				marker.points.push_back(point);
				std_msgs::msg::ColorRGBA color;
				color.a = 0.5;
				color.r = 0.0;
				color.g = 1.0;
				color.b = 0.0;
				marker.colors.push_back(color);
			}
		}
	}
	hallway2_pub_->publish(marker);
  }

  void publish_hallway3(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish hallway3");
	visualization_msgs::msg::Marker marker;
	vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
	//cout << "There are currently " << hallways.size() << " hallways" << endl;
	marker.header.frame_id = "map";
	marker.header.stamp = node_->now();
	marker.ns = "basic_shapes";
	marker.type = visualization_msgs::msg::Marker::POINTS;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.lifetime = rclcpp::Duration::from_seconds(0.0);
	for(int i = 0; i < hallways.size(); i++){
		vector<CartesianPoint> points = hallways[i].getPoints();
		int hallway_type = hallways[i].getHallwayType();
		//cout << "Number of points = " << points.size() << " Hallway type = " << hallway_type << endl;
		if(hallway_type == 2){
			for(int j = 0; j < points.size(); j++){
				float x = points[j].get_x();
				float y = points[j].get_y();
				geometry_msgs::msg::Point point;
				point.x = x;
				point.y = y;
				point.z = 0;
				marker.points.push_back(point);
				std_msgs::msg::ColorRGBA color;
				color.a = 0.5;
				color.r = 0.0;
				color.g = 0.0;
				color.b = 1.0;
				marker.colors.push_back(color);
			}
		}
	}
	hallway3_pub_->publish(marker);
  }

  void publish_hallway4(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish hallway4");
	visualization_msgs::msg::Marker marker;
	vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();
	//cout << "There are currently " << hallways.size() << " hallways" << endl;
	marker.header.frame_id = "map";
	marker.header.stamp = node_->now();
	marker.ns = "basic_shapes";
	marker.type = visualization_msgs::msg::Marker::POINTS;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.lifetime = rclcpp::Duration::from_seconds(0.0);
	for(int i = 0; i < hallways.size(); i++){
		vector<CartesianPoint> points = hallways[i].getPoints();
		int hallway_type = hallways[i].getHallwayType();
		//cout << "Number of points = " << points.size() << " Hallway type = " << hallway_type << endl;
		if(hallway_type == 3){
			for(int j = 0; j < points.size(); j++){
				float x = points[j].get_x();
				float y = points[j].get_y();
				geometry_msgs::msg::Point point;
				point.x = x;
				point.y = y;
				point.z = 0;
				marker.points.push_back(point);
				std_msgs::msg::ColorRGBA color;
				color.a = 0.5;
				color.r = 0.5;
				color.g = 0.5;
				color.b = 0.5;
				marker.colors.push_back(color);
			}
		}
	}
	hallway4_pub_->publish(marker);
  }

  void publish_occupancy(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish occupancy");
	nav_msgs::msg::OccupancyGrid grid;

	grid.header.frame_id = "map";
	grid.header.stamp = node_->now();
	grid.info.map_load_time = node_->now();

	grid.info.origin.orientation.w = 0;
	vector< vector <bool> > occupancyGrid = con->getPlanner()->getMap()->getOccupancyGrid();
	int gridSize = con->getPlanner()->getMap()->getOccupancySize();
	grid.info.resolution = gridSize / 100.0;
	grid.info.width = (int)(beliefs->getSpatialModel()->getConveyors()->getMapWidth() / grid.info.resolution);
	grid.info.height = (int)(beliefs->getSpatialModel()->getConveyors()->getMapHeight() / grid.info.resolution);

	for(int j = 0; j < grid.info.height; j++){
	    for(int i = 0; i < grid.info.width; i++){
		if(occupancyGrid[i][j]){
      			grid.data.push_back(50);
		}
		else{
			grid.data.push_back(0);
		}
    	    }
  	}
	occupancy_pub_->publish(grid);
  }

  void publish_region(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish regions");

	visualization_msgs::msg::MarkerArray markerArray;
	vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
	cout << "There are currently " << regions.size() << " regions" << endl;
	for(int i = 0 ; i < regions.size(); i++){
		//regions[i].print();
		visualization_msgs::msg::Marker marker;
		marker.header.frame_id = "map";
    		marker.header.stamp = node_->now();
		marker.ns = "basic_shapes";
    		marker.id = i;
		marker.type = visualization_msgs::msg::Marker::CYLINDER;

		marker.pose.position.x = regions[i].getCenter().get_x();
    		marker.pose.position.y = regions[i].getCenter().get_y();
    		marker.pose.position.z = 0;
    		marker.pose.orientation.x = 0.0;
    		marker.pose.orientation.y = 0.0;
    		marker.pose.orientation.z = 0.0;
    		marker.pose.orientation.w = 1.0;

    		// Set the scale of the marker -- 1x1x1 here means 1m on a side
    		marker.scale.x = marker.scale.y = regions[i].getRadius()*2;
    		marker.scale.z = 1.0;

    		// Set the color -- be sure to set alpha to something non-zero!
    		marker.color.r = 0.0f;
    		marker.color.g = 1.0f;
    		marker.color.b = 0.0f;
    		marker.color.a = 0.5;

    		marker.lifetime = rclcpp::Duration::from_seconds(0.0);
		markerArray.markers.push_back(marker);

	}
	region_pub_->publish(markerArray);
  }

  void publish_exits(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish exits");
	vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
	// cout << "There are currently " << regions.size() << " regions" << endl;
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = node_->now();
	marker.ns = "basic_shapes";
	marker.type = visualization_msgs::msg::Marker::POINTS;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = marker.scale.y = 0.15;
	marker.scale.z = 0.15;
	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 0.5;
	marker.lifetime = rclcpp::Duration::from_seconds(0.0);

	for(int i = 0 ; i < regions.size(); i++){
		vector<FORRExit> exits = regions[i].getExits();
		for(int j = 0; j < exits.size() ; j++){
			float x = exits[j].getExitPoint().get_x();
			float y = exits[j].getExitPoint().get_y();
			geometry_msgs::msg::Point point;
			point.x = x;
			point.y = y;
			point.z = 0;
			marker.points.push_back(point);
		}
	}
  	exits_pub_->publish(marker);
  }
  void publish_skeleton(){
  	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish skeleton");
  	vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
	// cout << "There are currently " << regions.size() << " regions" << endl;
	visualization_msgs::msg::Marker line_list;
	line_list.header.frame_id = "map";
	line_list.header.stamp = node_->now();
	line_list.ns = "basic_shapes";
	line_list.action = visualization_msgs::msg::Marker::ADD;
	line_list.id = 1;
	line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
	line_list.pose.orientation.w = 1.0;
	line_list.scale.x = 0.1;
	line_list.color.r = 1.0;
	line_list.color.a = 1.0;
	for(int i = 0 ; i < regions.size(); i++){
		vector<FORRExit> exits = regions[i].getExits();
		for(int j = 0; j < exits.size() ; j++){
			geometry_msgs::msg::Point p1, p2;
			p1.x = regions[i].getCenter().get_x();
			p1.y = regions[i].getCenter().get_y();
			p1.z = 0;

			p2.x = regions[exits[j].getExitRegion()].getCenter().get_x();
			p2.y = regions[exits[j].getExitRegion()].getCenter().get_y();
			p2.z = 0;

			line_list.points.push_back(p1);
			line_list.points.push_back(p2);
		}
	}
  	skeleton_pub_->publish(line_list);
  }

  void publish_trails(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish trail");
	//Goal here is to publish all trails

	FORRTrails *trails = beliefs->getSpatialModel()->getTrails();
	/*nav_msgs::msg::Path path;
	path.header.frame_id = "map";
	path.header.stamp = node_->now();

	for(int i = 0; i < trails->getSize(); i++){
		vector<TrailMarker> trail = trails->getTrail(i);
		//fill up the path using the trail
		for(int j = 0; j < trail.size(); j++){
			double x = trail[j].coordinates.get_x();
			double y = trail[j].coordinates.get_y();
			geometry_msgs::msg::PoseStamped poseStamped;
			poseStamped.header.frame_id = "map";
			poseStamped.header.stamp = path.header.stamp;
			poseStamped.pose.position.x = x;
			poseStamped.pose.position.y = y;
			path.poses.push_back(poseStamped);
		}
	}*/

	visualization_msgs::msg::Marker line_list;
	line_list.header.frame_id = "map";
	line_list.header.stamp = node_->now();
	line_list.ns = "basic_shapes";
	line_list.action = visualization_msgs::msg::Marker::ADD;
	line_list.id = 1;
	line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
	line_list.pose.orientation.w = 1.0;
	line_list.scale.x = 0.1;
	line_list.color.r = 1.0;
	line_list.color.a = 1.0;
	// cout << "There are currently " << trails->getSize() << " trails" << endl;
	for(int i = 0 ; i < trails->getSize(); i++){
		vector<TrailMarker> trail = trails->getTrail(i);
		for(int j = 0; j < trail.size()-1; j++){
			geometry_msgs::msg::Point p1, p2;
			p1.x = trail[j].coordinates.get_x();
			p1.y = trail[j].coordinates.get_y();
			p1.z = 0;

			p2.x = trail[j+1].coordinates.get_x();
			p2.y = trail[j+1].coordinates.get_y();
			p2.z = 0;

			line_list.points.push_back(p1);
			line_list.points.push_back(p2);
		}
	}
	trails_pub_->publish(line_list);

  }

  void publish_doors(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish doors");

	std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
	// cout << "There are currently " << doors.size() << " regions" << endl;
	visualization_msgs::msg::Marker line_list;
	line_list.header.frame_id = "map";
	line_list.header.stamp = node_->now();
	line_list.ns = "basic_shapes";
	line_list.action = visualization_msgs::msg::Marker::ADD;
	line_list.id = 1;
	line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
	line_list.pose.orientation.w = 1.0;
	line_list.scale.x = 0.3;
	line_list.color.r = 1.0;
	line_list.color.a = 1.0;

	for(int i = 0 ; i < doors.size(); i++){
		for(int j = 0; j < doors[i].size(); j++){
			geometry_msgs::msg::Point p1, p2;
			p1.x = doors[i][j].startPoint.getExitPoint().get_x();
			p1.y = doors[i][j].startPoint.getExitPoint().get_y();
			p1.z = 0;

			p2.x = doors[i][j].endPoint.getExitPoint().get_x();
			p2.y = doors[i][j].endPoint.getExitPoint().get_y();
			p2.z = 0;

			line_list.points.push_back(p1);
			line_list.points.push_back(p2);
		}
	}
	doors_pub_->publish(line_list);
  }

    void publish_barriers(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish barriers");

	// std::vector<LineSegment> barriers = beliefs->getSpatialModel()->getBarriers()->getBarriers();
	// cout << "There are currently " << barriers.size() << " barriers" << endl;
	// visualization_msgs::msg::Marker line_list;
	// line_list.header.frame_id = "map";
	// line_list.header.stamp = node_->now();
	// line_list.ns = "basic_shapes";
	// line_list.action = visualization_msgs::msg::Marker::ADD;
	// line_list.id = 1;
	// line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
	// line_list.pose.orientation.w = 1.0;
	// line_list.scale.x = 0.1;
	// line_list.color.r = 1.0;
	// line_list.color.a = 1.0;

	// for(int i = 0 ; i < barriers.size(); i++){
	// 	geometry_msgs::msg::Point p1, p2;
	// 	p1.x = barriers[i].get_endpoints().first.get_x();
	// 	p1.y = barriers[i].get_endpoints().first.get_y();
	// 	p1.z = 0;

	// 	p2.x = barriers[i].get_endpoints().second.get_x();
	// 	p2.y = barriers[i].get_endpoints().second.get_y();
	// 	p2.z = 0;

	// 	line_list.points.push_back(p1);
	// 	line_list.points.push_back(p2);
	// }
	// barriers_pub_->publish(line_list);
    visualization_msgs::msg::Marker cube;
	cube.header.frame_id = "map";
	cube.header.stamp = node_->now();
	cube.ns = "basic_shapes";
	cube.action = visualization_msgs::msg::Marker::ADD;
	cube.id = 1;
	cube.type = visualization_msgs::msg::Marker::CUBE;
	cube.pose.orientation.w = 1.0;
	cube.scale.x = 40;
	cube.scale.y = 40;
	cube.scale.z = 0.1;
	cube.color.r = 1;
	cube.color.g = 1;
	cube.color.b = 1;
	cube.color.a = 1;
	cube.pose.position.x = 12;
	cube.pose.position.y = 12;
	cube.pose.position.z = -1;
	barriers_pub_->publish(cube);
  }

  void publish_walls(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish walls");
	vector<Wall> walls = con->getPlanner()->getMap()->getWalls();
	cout << "There are currently " << walls.size() << " walls" << endl;
	visualization_msgs::msg::Marker line_list;
	line_list.header.frame_id = "map";
    	line_list.header.stamp = node_->now();
	line_list.ns = "basic_shapes";
	line_list.action = visualization_msgs::msg::Marker::ADD;
    	line_list.id = 1;
	line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
	line_list.pose.orientation.w = 1.0;
	line_list.scale.x = 0.3;
	line_list.color.r = 0.5;
	line_list.color.a = 0.5;

	for(int i = 0 ; i < walls.size(); i++){
		geometry_msgs::msg::Point p1, p2;
		p1.x = walls[i].x1/100.0;
		p1.y = walls[i].y1/100.0;
		p1.z = 0;
		p2.x = walls[i].x2/100.0;
		p2.y = walls[i].y2/100.0;
		p2.z = 0;

		line_list.points.push_back(p1);
		line_list.points.push_back(p2);
	}
	walls_pub_->publish(line_list);

	// visualization_msgs::msg::Marker cube_list;
	// cube_list.header.frame_id = "map";
	// cube_list.header.stamp = node_->now();
	// cube_list.ns = "basic_shapes";
	// cube_list.action = visualization_msgs::msg::Marker::ADD;
	// cube_list.id = 1;
	// cube_list.type = visualization_msgs::msg::Marker::CUBE_LIST;
	// cube_list.pose.orientation.w = 1.0;
	// cube_list.scale.x = 0.1;
	// cube_list.scale.y = 0.1;
	// cube_list.scale.z = 5;
	// cube_list.color.r = 0.5;
	// cube_list.color.a = 1;

	// for(int i = 0 ; i < walls.size(); i++){
	// 	geometry_msgs::msg::Point p1, p2;
	// 	p1.x = walls[i].x1/100.0;
	// 	p1.y = walls[i].y1/100.0;
	// 	p1.z = 0;
	// 	p2.x = walls[i].x2/100.0;
	// 	p2.y = walls[i].y2/100.0;
	// 	p2.z = 0;

	// 	cube_list.points.push_back(p1);
	// 	cube_list.points.push_back(p2);
	// 	double length = sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
	// 	if(length > 0.1){
	// 		if(length <= 0.4){
	// 			geometry_msgs::msg::Point p3;
	// 			p3.x = (p1.x + p2.x)/2;
	// 			p3.y = (p1.y + p2.y)/2;
	// 			cube_list.points.push_back(p3);
	// 		}
	// 		else{
	// 			if(p1.x == p2.x){
	// 				if(p1.y < p2.y){
	// 					for(double i = p1.y+0.1; i <= p2.y; i += 0.1){
	// 						geometry_msgs::msg::Point p3;
	// 						p3.x = p1.x;
	// 						p3.y = i;
	// 						cube_list.points.push_back(p3);
	// 					}
	// 				}
	// 				else{
	// 					for(double i = p2.y+0.1; i <= p1.y; i += 0.1){
	// 						geometry_msgs::msg::Point p3;
	// 						p3.x = p2.x;
	// 						p3.y = i;
	// 						cube_list.points.push_back(p3);
	// 					}
	// 				}
	// 			}
	// 			else if(p1.y == p2.y){
	// 				if(p1.x < p2.x){
	// 					for(double i = p1.x+0.1; i <= p2.x; i += 0.1){
	// 						geometry_msgs::msg::Point p3;
	// 						p3.x = i;
	// 						p3.y = p1.y;
	// 						cube_list.points.push_back(p3);
	// 					}
	// 				}
	// 				else{
	// 					for(double i = p2.x+0.1; i <= p1.x; i += 0.1){
	// 						geometry_msgs::msg::Point p3;
	// 						p3.x = i;
	// 						p3.y = p2.y;
	// 						cube_list.points.push_back(p3);
	// 					}
	// 				}
	// 			}
	// 			else{
	// 				double slope = (p1.y - p2.y) / (p1.x - p2.x);
	// 				double intercept = p1.y - slope * p1.x;
	// 				if(abs(p1.x - p2.x) > abs(p1.y - p2.y)){
	// 					if(p1.x < p2.x){
	// 						for(double i = p1.x+0.1; i <= p2.x; i += 0.1){
	// 							geometry_msgs::msg::Point p3;
	// 							p3.x = i;
	// 							p3.y = slope * i + intercept;
	// 							cube_list.points.push_back(p3);
	// 						}
	// 					}
	// 					else{
	// 						for(double i = p2.x+0.1; i <= p1.x; i += 0.1){
	// 							geometry_msgs::msg::Point p3;
	// 							p3.x = i;
	// 							p3.y = slope * i + intercept;
	// 							cube_list.points.push_back(p3);
	// 						}
	// 					}
	// 				}
	// 				else{
	// 					if(p1.y < p2.y){
	// 						for(double i = p1.y+0.1; i <= p2.y; i += 0.1){
	// 							geometry_msgs::msg::Point p3;
	// 							p3.x = (i - intercept) / slope;
	// 							p3.y = i;
	// 							cube_list.points.push_back(p3);
	// 						}
	// 					}
	// 					else{
	// 						for(double i = p2.y+0.1; i <= p1.y; i += 0.1){
	// 							geometry_msgs::msg::Point p3;
	// 							p3.x = (i - intercept) / slope;
	// 							p3.y = i;
	// 							cube_list.points.push_back(p3);
	// 						}
	// 					}
	// 				}
	// 			}
	// 		}
	// 	}
	// }
	// walls_pub_->publish(cube_list);
  }

  void publish_all_targets(){
	// // RCLCPP_DEBUG(this->get_logger(), "Publish All targets as pose array!!");
	geometry_msgs::msg::PoseArray targets;
	targets.header.frame_id = "map";
	targets.header.stamp = node_->now();

	list<Task*> agenda = beliefs->getAgentState()->getAllAgenda();
	for(list<Task*>::iterator it = agenda.begin(); it != agenda.end(); it++){
		double x = (*it)->getX();
		double y = (*it)->getY();
		geometry_msgs::msg::Pose pose;
		pose.position.x = x;
		pose.position.y = y;
		targets.poses.push_back(pose);
	}
	all_targets_pub_->publish(targets);
  }

  void publish_remaining_targets(){
	// // RCLCPP_DEBUG(this->get_logger(), "Publish remaining targets as pose array!!");
	geometry_msgs::msg::PoseArray targets;
	targets.header.frame_id = "map";
	targets.header.stamp = node_->now();

	list<Task*> agenda = beliefs->getAgentState()->getAgenda();
	for(list<Task*>::iterator it = agenda.begin(); it != agenda.end(); it++){
		double x = (*it)->getX();
		double y = (*it)->getY();
		geometry_msgs::msg::Pose pose;
		pose.position.x = x;
		pose.position.y = y;
		targets.poses.push_back(pose);
	}
	remaining_targets_pub_->publish(targets);
  }

  void publish_highway(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish highway");
	nav_msgs::msg::OccupancyGrid grid;

	grid.header.frame_id = "map";
	grid.header.stamp = node_->now();
	grid.info.map_load_time = node_->now();

	grid.info.origin.orientation.w = 0;
	grid.info.resolution = 1;
	if(con->getHighwaysOn() == 1){
		grid.info.width = con->gethighwayExploration()->getLength();
		grid.info.height = con->gethighwayExploration()->getHeight();
		vector< vector<int> > highways;
		if(con->getHighwayFinished()){
			highways = beliefs->getAgentState()->getPassageGrid();
		}
		else{
			highways = con->gethighwayExploration()->getHighwayGrid();
		}
		if(highways.size() > 0){
			for(int j = 0; j < grid.info.height; j++){
				for(int i = 0; i < grid.info.width; i++){
					if(highways[i][j] < 0){
						grid.data.push_back(-1);
					}
					else if(highways[i][j] * 2 < 100){
						grid.data.push_back(highways[i][j] * 2);
					}
					else{
						grid.data.push_back(100);
					}
				}
			}
			highway_pub_->publish(grid);
		}
	}
	else if(con->getHighwaysOn() == 2){
		grid.info.width = con->getfrontierExploration()->getLength();
		grid.info.height = con->getfrontierExploration()->getHeight();
		vector< vector<int> > highways;
		if(con->getFrontierFinished()){
			highways = beliefs->getAgentState()->getPassageGrid();
		}
		else{
			highways = con->getfrontierExploration()->getFrontierGrid();
		}
		if(highways.size() > 0){
			for(int j = 0; j < grid.info.height; j++){
				for(int i = 0; i < grid.info.width; i++){
					if(highways[i][j] < 0){
						grid.data.push_back(-1);
					}
					else if(highways[i][j] * 2 < 100){
						grid.data.push_back(highways[i][j] * 2);
					}
					else{
						grid.data.push_back(100);
					}
				}
			}
			highway_pub_->publish(grid);
		}
	}
  }

  void publish_highway_stack(){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish highway_stack");
	vector< Position > highway_stack;
	if(con->getHighwaysOn() == 1){
		highway_stack = con->gethighwayExploration()->getHighwayStack();
	}
	else if(con->getHighwaysOn() == 2){
		highway_stack = con->getfrontierExploration()->getFrontierStack();
	}
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = node_->now();
	marker.ns = "basic_shapes";
	marker.type = visualization_msgs::msg::Marker::POINTS;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = marker.scale.y = 0.5;
	marker.scale.z = 0.5;
	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 0.5;
	marker.lifetime = rclcpp::Duration::from_seconds(0.0);

	for(int i = 0 ; i < highway_stack.size(); i++){
		float x = highway_stack[i].getX();
		float y = highway_stack[i].getY();
		geometry_msgs::msg::Point point;
		point.x = x;
		point.y = y;
		point.z = 0;
		marker.points.push_back(point);
	}
	for(int i = 0 ; i < highway_stack.size(); i++){
		std_msgs::msg::ColorRGBA c;
		double percent = float(i)/float(highway_stack.size()-1);
		c.r = 1.0 - percent;
		c.g = 0;
		c.b = percent - 1.0;
		c.a = 1.0;
		marker.colors.push_back(c);
	}
  	highway_stack_pub_->publish(marker);
  }

  void publish_log(FORRAction decision, double overallTimeSec, double computationTimeSec){
	// // RCLCPP_DEBUG(this->get_logger(), "Inside publish decision log!!");
	std_msgs::msg::String log;
	double robotX = beliefs->getAgentState()->getCurrentPosition().getX();
	double robotY = beliefs->getAgentState()->getCurrentPosition().getY();
	double targetX;
	double targetY;
	double robotTheta = beliefs->getAgentState()->getCurrentPosition().getTheta();

	geometry_msgs::msg::PoseStamped poseStamped;
	poseStamped.header.frame_id = "map";
	poseStamped.header.stamp = node_->now();
	poseStamped.pose.position.x = robotX;
	poseStamped.pose.position.y = robotY;
	poseStamped.pose.position.z = 0;
	poseStamped.pose.orientation = tf2::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, robotTheta);
	pose_pub_->publish(poseStamped);

	if(beliefs->getAgentState()->getCurrentTask() != NULL) {
		targetX = beliefs->getAgentState()->getCurrentTask()->getTaskX();
		targetY = beliefs->getAgentState()->getCurrentTask()->getTaskY();
	} else {
		targetX = 0;
		targetY = 0;
	}
	vector<CartesianPoint> laserEndpoints = beliefs->getAgentState()->getCurrentLaserEndpoints();
	sensor_msgs::msg::LaserScan laserScan = beliefs->getAgentState()->getCurrentLaserScan();
	laser_pub_->publish(laserScan);

	FORRAction max_forward = beliefs->getAgentState()->maxForwardAction();
	// // RCLCPP_DEBUG(this->get_logger(), "After max_forward");
	//vector< vector<CartesianPoint> > allTrace = beliefs->getAgentState()->getAllTrace();
	list<Task*>& agenda = beliefs->getAgentState()->getAgenda();
	list<Task*>& all_agenda = beliefs->getAgentState()->getAllAgenda();
	// // RCLCPP_DEBUG(this->get_logger(), "After all_agenda");
	vector<FORRRegion> regions = beliefs->getSpatialModel()->getRegionList()->getRegions();
	vector< vector< CartesianPoint> > trails =  beliefs->getSpatialModel()->getTrails()->getTrailsPoints();
	// // RCLCPP_DEBUG(this->get_logger(), "After trails");
	FORRActionType chosenActionType = decision.type;
	int chosenActionParameter = decision.parameter;
	double decisionTier = con->getCurrentDecisionStats()->decisionTier;
	string vetoedActions = con->getCurrentDecisionStats()->vetoedActions;
	string advisors = con->getCurrentDecisionStats()->advisors;
	string advisorComments = con->getCurrentDecisionStats()->advisorComments;
	string advisorInfluence = con->getCurrentDecisionStats()->advisorInfluence;
	double planningComputationTime = con->getCurrentDecisionStats()->planningComputationTime;
	double learningComputationTime = con->getCurrentDecisionStats()->learningComputationTime;
	double graphingComputationTime = con->getCurrentDecisionStats()->graphingComputationTime;
	string chosenPlanner = con->getCurrentDecisionStats()->chosenPlanner;
	string plannerComments = con->getCurrentDecisionStats()->plannerComments;
	// cout << "vetoedActions = " << vetoedActions << " decisionTier = " << decisionTier << " advisors = " << advisors << " advisorComments = " << advisorComments << endl;
	vector< vector<int> > conveyors = beliefs->getSpatialModel()->getConveyors()->getConveyors();
	std::vector< std::vector<Door> > doors = beliefs->getSpatialModel()->getDoors()->getDoors();
	vector<Aggregate> hallways = beliefs->getSpatialModel()->getHallways()->getHallways();

	// // RCLCPP_DEBUG(this->get_logger(), "After decision statistics");
	int decisionCount = -1;
	int currentTask = -1;
	if(!agenda.empty()){
		currentTask = all_agenda.size() - agenda.size();
  		//if(currentTask != 0)
		decisionCount = beliefs->getAgentState()->getCurrentTask()->getDecisionCount();
	}
	// // RCLCPP_DEBUG(this->get_logger(), "After decisionCount");

	cout << "Current task " << currentTask << " and decision number " << decisionCount << " with overall time " << overallTimeSec << " and computation time " << computationTimeSec << endl;

	std::stringstream lep;
	for(int i = 0; i < laserEndpoints.size(); i++){
		double x = laserEndpoints[i].get_x();
 		double y = laserEndpoints[i].get_y();
		lep << x << "," << y << ";";
	}
	// // RCLCPP_DEBUG(this->get_logger(), "After laserEndpoints");


	std::stringstream ls;
	double min_laser_scan = 25; //meters
	for(int i = 0; i < laserScan.ranges.size(); i++){
		double length = laserScan.ranges[i];
		if(length < min_laser_scan){
			min_laser_scan = length;
		}
		ls << length << ",";
	}
	// // RCLCPP_DEBUG(this->get_logger(), "After laserScan");
	/*int totalSize = 0;
	for(int i = 0; i < allTrace.size(); i++){
		totalSize += allTrace[i].size();
	}*/


	std::stringstream regionsstream;
	for(int i = 0; i < regions.size(); i++){
		regionsstream << regions[i].getCenter().get_x() << " " << regions[i].getCenter().get_y() << " " << regions[i].getRadius();
		vector<FORRExit> exits = regions[i].getExits();
		for(int j = 0; j < exits.size() ; j++){
			regionsstream << " " << exits[j].getExitPoint().get_x() << " "  << exits[j].getExitPoint().get_y() << " "  << exits[j].getExitRegion() << " "  << exits[j].getMidPoint().get_x() << " "  << exits[j].getMidPoint().get_y() << " "  << exits[j].getExitRegionPoint().get_x() << " "  << exits[j].getExitRegionPoint().get_y() << " "  << exits[j].getExitDistance() << " "  << exits[j].getConnectionPath();
		}
		regionsstream << ";";
	}
	// // RCLCPP_DEBUG(this->get_logger(), "After regions");


	std::stringstream trailstream;
	for(int i = 0; i < trails.size(); i++){
		for(int j = 0; j < trails[i].size(); j++){
			trailstream << trails[i][j].get_x() << " " << trails[i][j].get_y() << " ";
		}
		trailstream << ";";
	}
	// // RCLCPP_DEBUG(this->get_logger(), "After trails");

	std::stringstream conveyorStream;
	for(int j = 0; j < conveyors.size()-1; j++){
		for(int i = 0; i < conveyors[j].size(); i++){
			conveyorStream << conveyors[j][i] << " ";
		}
		conveyorStream << ";";
	}
	// // RCLCPP_DEBUG(this->get_logger(), "After conveyors");
	

	std::stringstream doorStream;
	for(int i = 0; i < doors.size(); i++){
		for(int j = 0; j < doors[i].size(); j++){
			doorStream << doors[i][j].startPoint.getExitPoint().get_x() << " " << doors[i][j].startPoint.getExitPoint().get_y() << " " << doors[i][j].endPoint.getExitPoint().get_x() << " " << doors[i][j].endPoint.getExitPoint().get_y() << " " << doors[i][j].str << ", ";
		}
		doorStream << ";";
	}

	// // RCLCPP_DEBUG(this->get_logger(), "After doors");

	std::stringstream hallwayStream;
	for(int i = 0; i < hallways.size(); i++){
		vector<CartesianPoint> points = hallways[i].getPoints();
		hallwayStream << hallways[i].getHallwayType();
		for(int j = 0; j < points.size(); j++){
			hallwayStream << " " << points[j].get_x() << " " << points[j].get_y();
		}
		hallwayStream << ";";
	}

	// // RCLCPP_DEBUG(this->get_logger(), "After hallways");

	std::stringstream planStream;
	if(beliefs->getAgentState()->getCurrentTask() != NULL){
		vector <CartesianPoint> waypoints = beliefs->getAgentState()->getCurrentTask()->getWaypoints();
		double pathCostInNavGraph = beliefs->getAgentState()->getCurrentTask()->getPathCostInNavGraph();
		double pathCostInNavOrigGraph = beliefs->getAgentState()->getCurrentTask()->getPathCostInNavOrigGraph();
		planStream << pathCostInNavGraph << " " << pathCostInNavOrigGraph << ";";

		for(int i = 0; i < waypoints.size(); i++){
			planStream << waypoints[i].get_x() << " " << waypoints[i].get_y();
			planStream << ";";
		}

		//double plancost = beliefs->getAgentState()->getCurrentTask()->planCost(waypoints, con->getPlanner(), beliefs->getAgentState()->getCurrentPosition(), Position(targetX,targetY,0));
		//planStream << "\t" << plancost;
	}
	// // RCLCPP_DEBUG(this->get_logger(), "After planStream");

	std::stringstream origPlanStream;
	if(beliefs->getAgentState()->getCurrentTask() != NULL){
		vector <CartesianPoint> waypoints = beliefs->getAgentState()->getCurrentTask()->getOrigWaypoints();
		double origPathCostInNavGraph = beliefs->getAgentState()->getCurrentTask()->getOrigPathCostInNavGraph();
		double origPathCostInOrigNavGraph = beliefs->getAgentState()->getCurrentTask()->getOrigPathCostInOrigNavGraph();
		origPlanStream << origPathCostInNavGraph << " " << origPathCostInOrigNavGraph<< ";";

		for(int i = 0; i < waypoints.size(); i++){
			origPlanStream << waypoints[i].get_x() << " " << waypoints[i].get_y();
			origPlanStream << ";";		
		}

		//double plancost = beliefs->getAgentState()->getCurrentTask()->planCost(waypoints, con->getPlanner(), beliefs->getAgentState()->getCurrentPosition(), Position(targetX,targetY,0));
		//origPlanStream << "\t" << plancost;
	}
	//// RCLCPP_DEBUG(this->get_logger(), "After origPlanStream");

	std::stringstream crowdStream;
	geometry_msgs::msg::PoseArray crowdpose = beliefs->getAgentState()->getCrowdPose();

	for(int i = 0; i < crowdpose.poses.size(); i++){
		crowdStream << crowdpose.poses[i].position.x << " " << crowdpose.poses[i].position.y << " " << crowdpose.poses[i].orientation.x 
		<< " " << crowdpose.poses[i].orientation.y << " " << crowdpose.poses[i].orientation.z << " " << crowdpose.poses[i].orientation.w;
		crowdStream << ";";
	}
	//// RCLCPP_DEBUG(this->get_logger(), "After crowdStream");

	std::stringstream allCrowdStream;
	geometry_msgs::msg::PoseArray crowdposeall = beliefs->getAgentState()->getCrowdPoseAll();

	for(int i = 0; i < crowdposeall.poses.size(); i++){
		allCrowdStream << crowdposeall.poses[i].position.x << " " << crowdposeall.poses[i].position.y << " " << crowdposeall.poses[i].orientation.x
		<< " " << crowdposeall.poses[i].orientation.y << " " << crowdposeall.poses[i].orientation.z << " " << crowdposeall.poses[i].orientation.w;
		allCrowdStream << ";";
	}
	//// RCLCPP_DEBUG(this->get_logger(), "After all crowdStream");

	std::stringstream crowdModel;
	semaforr::CrowdModel model = con->getPlanner()->getCrowdModel();
	int resolution = model.resolution;
	int height = model.height;
	int width = model.width;
	std::vector<double> densities = model.densities;
	std::vector<double> risk = model.risk;
	crowdModel << height << " " << width << " " << resolution << ";";
	for(int i = 0; i < densities.size() ; i++){
		crowdModel << densities[i] << " ";
	}
	crowdModel << "\t";
	for(int i = 0; i < risk.size() ; i++){
		crowdModel << risk[i] << " ";
	}
	crowdModel << "\t";
	std::vector<double> left = model.left;
	for(int i = 0; i < left.size() ; i++){
		crowdModel << left[i] << " ";
	}
	crowdModel << "\t";
	std::vector<double> right = model.right;
	for(int i = 0; i < right.size() ; i++){
		crowdModel << right[i] << " ";
	}
	crowdModel << "\t";
	std::vector<double> up = model.up;
	for(int i = 0; i < up.size() ; i++){
		crowdModel << up[i] << " ";
	}
	crowdModel << "\t";
	std::vector<double> down = model.down;
	for(int i = 0; i < down.size() ; i++){
		crowdModel << down[i] << " ";
	}
	crowdModel << "\t";
	std::vector<double> up_left = model.up_left;
	for(int i = 0; i < up_left.size() ; i++){
		crowdModel << up_left[i] << " ";
	}
	crowdModel << "\t";
	std::vector<double> up_right = model.up_right;
	for(int i = 0; i < up_right.size() ; i++){
		crowdModel << up_right[i] << " ";
	}
	crowdModel << "\t";
	std::vector<double> down_left = model.down_left;
	for(int i = 0; i < down_left.size() ; i++){
		crowdModel << down_left[i] << " ";
	}
	crowdModel << "\t";
	std::vector<double> down_right = model.down_right;
	for(int i = 0; i < down_right.size() ; i++){
		crowdModel << down_right[i] << " ";
	}

	//// RCLCPP_DEBUG(this->get_logger(), "After all crowd model");

	std::stringstream passageStream;
	if(currentTask > 0 and decisionCount == 1){
		vector< vector<int> > highways;
		if(con->getHighwaysOn() == 1){
			if(con->getHighwayFinished()){
				highways = beliefs->getAgentState()->getPassageGrid();
			}
			else{
				highways = con->gethighwayExploration()->getHighwayGrid();
			}
		}
		else if(con->getHighwaysOn() == 2){
			if(con->getFrontierFinished()){
				highways = beliefs->getAgentState()->getPassageGrid();
			}
			else{
				highways = con->getfrontierExploration()->getFrontierGrid();
			}
		}
		if(highways.size() > 0){
			for(int j = 0; j < highways.size()-1; j++){
				for(int i = 0; i < highways[j].size(); i++){
					passageStream << highways[j][i] << " ";
				}
				passageStream << ";";
			}
		}
		else{
			passageStream << " ";
		}
	}
	else{
		passageStream << " ";
	}

	// // RCLCPP_DEBUG(this->get_logger(), "After conveyors");

	std::stringstream output;

	output << currentTask << "\t" << decisionCount << "\t" << overallTimeSec << "\t" << computationTimeSec << "\t" << targetX << "\t" << targetY << "\t" << robotX << "\t" << robotY << "\t" << robotTheta << "\t" << max_forward.parameter << "\t" << decisionTier << "\t" << vetoedActions << "\t" << chosenActionType << "\t" << chosenActionParameter << "\t" << advisors << "\t" << advisorComments << "\t" << planStream.str() << "\t" << origPlanStream.str() << "\t" << regionsstream.str() << "\t" << trailstream.str() << "\t" << doorStream.str() << "\t" << conveyorStream.str() << "\t" << hallwayStream.str() << "\t" << planningComputationTime << "\t" << learningComputationTime << "\t" << chosenPlanner << "\t" << graphingComputationTime<< "\t" << passageStream.str() << "\t" << plannerComments  << "\t" << lep.str() << "\t" << ls.str();// << "\t" << crowdModel.str() << "\t" << crowdStream.str() << "\t" << allCrowdStream.str() << "\t" << advisorInfluence;

	//output << currentTask << "\t" << decisionCount << "\t" << targetX << "\t" << targetY << "\t" << robotX << "\t" << robotY << "\t" << robotTheta << "\t" << lep.str() << "\t" << ls.str();

	//output << currentTask << "\t" << decisionCount << "\t" << overallTimeSec << "\t" << computationTimeSec << "\t" << targetX << "\t" << targetY << "\t" << robotX << "\t" << robotY << "\t" << robotTheta << "\t" << max_forward.parameter << "\t" << decisionTier << "\t" << vetoedActions << "\t" << chosenActionType << "\t" << chosenActionParameter << "\t" << advisors << "\t" << advisorComments << "\t" << lep.str() << "\t" << ls.str() << "\t" << crowdStream.str() << "\t" << allCrowdStream.str() << "\t" << crowdModel.str() << "\t" << planStream.str();

	log.data = output.str();
	stats_pub_->publish(log);
	con->clearCurrentDecisionStats();
  }
};
