/* \mainpage ROSiffied semaforr Documentation
 * \brief RobotDriver governs low-level actions of a robot.
 *
 *  
 * \author Eric Guan, Raj Korpan
 * \version Semaforr ROS 2.0
 * \Hunter College Fall 2024
 * 
 */

#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <sys/time.h>

#include "Controller.h"
#include "FORRAction.h"
#include "Visualizer.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Python.h>

using namespace std;

// Main interface between SemaFORR and ROS, all ROS related information should be in RobotDriver
class RobotDriver : public rclcpp::Node, public std::enable_shared_from_this<RobotDriver>
{
private:
    //! We will be publishing to the "cmd_vel" topic to issue commands
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    //! We will be listening to /pose, /laserscan and /crowd_model, /crowd_pose topics
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;
    rclcpp::Subscription<semaforr__msg__CrowdModel>::SharedPtr sub_crowd_model_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_crowd_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_crowd_pose_all_;

    // Current position and previous stopping position of the robot
    Position current, previous;
    // Current and previous laser scan
    sensor_msgs::msg::LaserScan laserscan;
    // Current crowd_model
    semaforr__msg__CrowdModel crowdModel;
    // Current crowd_pose
    geometry_msgs::msg::PoseArray crowdPose, crowdPoseAll;
    // Controller
    Controller *controller;
    // Pos received
    bool init_pos_received, init_laser_received;
    // Add noise to pose
    bool add_noise;
    // Visualization 
    Visualizer *viz_;

public:
    //! ROS node initialization
    RobotDriver() : Node("semaforr")
    {
        // Pass in arguments for SemaFORR Controller
        this->declare_parameter("semaforr_path", rclcpp::PARAMETER_STRING);
        this->declare_parameter("target_set", rclcpp::PARAMETER_STRING);
        this->declare_parameter("map_config", rclcpp::PARAMETER_STRING);
        this->declare_parameter("map_dimensions", rclcpp::PARAMETER_STRING);
        this->declare_parameter("advisors", rclcpp::PARAMETER_STRING);
        this->declare_parameter("params", rclcpp::PARAMETER_STRING);

        // Unpack the passed in arguments
        rclcpp::Parameter semaforr_path_param = this->get_parameter("semaforr_path");
        rclcpp::Parameter target_set_param = this->get_parameter("target_set");
        rclcpp::Parameter map_config_param = this->get_parameter("map_config");
        rclcpp::Parameter map_dimensions_param = this->get_parameter("map_dimensions");
        rclcpp::Parameter advisors_param = this->get_parameter("advisors");
        rclcpp::Parameter params_param = this->get_parameter("params");

        std::string semaforr_path = semaforr_path_param.as_string();
        std::string target_set = target_set_param.as_string();
        std::string map_config = map_config_param.as_string();
        std::string map_dimensions = map_dimensions_param.as_string();
        std::string advisors = advisors_param.as_string();
        std::string params = params_param.as_string();

        controller = new Controller(advisors, params, map_config, target_set, map_dimensions);

        std::cout << "starting to declare pubs and subs" << std::endl;
        // Set up the publisher for the cmd_vel topic
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        
        cout << "cmd_vel_pub successful" << endl;
        
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose", 10, std::bind(&RobotDriver::updatePose, this, std::placeholders::_1));

        cout << "sub_pose successful" << endl;

        sub_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan_raw", 10, std::bind(&RobotDriver::updateLaserScan, this, std::placeholders::_1));
        
        cout << "sub_laser successful" << endl;

        // sub_crowd_model_ = this->create_subscription<semaforr__msg__CrowdModel>(
        //     "crowd_model", 10, std::bind(&RobotDriver::updateCrowdModel, this, std::placeholders::_1));
        
        // cout << "sub_crowd_model successful" << endl;
        
        // sub_crowd_pose_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        //     "crowd_pose", 10, std::bind(&RobotDriver::updateCrowdPose, this, std::placeholders::_1));
        
        // cout << "sub_crowd_pose successful" << endl;
        
        // sub_crowd_pose_all_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        //     "crowd_pose_all", 10, std::bind(&RobotDriver::updateCrowdPoseAll, this, std::placeholders::_1));

        // cout << "sub_crowd_pose_all successful" << endl;
        
        std::cout << "finished declaring pubs and subs" << std::endl;

        // Declare and create a controller with task, action and advisor configuration
        // controller = new Controller(advisors, params, map_config, target_set, map_dimensions);
        // RCLCPP_INFO(node->get_logger(), "Controller Initialized");
        // controller = con;
        init_pos_received = false;
        init_laser_received = false;
        current.setX(0);
        current.setY(0);
        current.setTheta(0);
        add_noise = false;
        previous.setX(0);
        previous.setY(0);
        previous.setTheta(0);
    }

    // initialize the visualizer
    void initialize_viz(rclcpp::Node::SharedPtr node_ptr) {
        std::cout << "start of visualizer and after node_ptr" << std::endl;
        viz_ = new Visualizer(node_ptr, controller);
        std::cout << "visualizer successfully created" << std::endl;
    }

    // Callback function for crowd pose message
    void updateCrowdPose(const geometry_msgs::msg::PoseArray &crowd_pose)
    {
        crowdPose = crowd_pose;
    }

    // Callback function for crowd pose all message
    void updateCrowdPoseAll(const geometry_msgs::msg::PoseArray &crowd_pose_all)
    {
        crowdPoseAll = crowd_pose_all;
    }

    // Callback function for crowd model message
    void updateCrowdModel(const semaforr__msg__CrowdModel &crowd_model)
    {
        controller->getPlanner()->setCrowdModel(crowd_model);
        controller->updatePlannersModels(crowd_model);
        controller->getBeliefs()->getAgentState()->setCrowdModel(crowd_model);
    }

    // Callback function for pose message
    void updatePose(const geometry_msgs::msg::PoseStamped &pose)
    {
        double x = pose.pose.position.x;
        double y = pose.pose.position.y;
        tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        Position currentPose(x, y, yaw);
        if (add_noise)
        {
            double new_x = currentPose.getX() + ((float(rand()) / float(RAND_MAX)) * (0.5 - -0.5)) + -0.5;
            double new_y = currentPose.getY() + ((float(rand()) / float(RAND_MAX)) * (0.5 - -0.5)) + -0.5;
            double new_theta = currentPose.getTheta() + ((float(rand()) / float(RAND_MAX)) * (0.0872665 - -0.0872665)) + -0.0872665;
            if (new_theta < -M_PI)
            {
                new_theta = new_theta + 2 * M_PI;
            }
            else if (new_theta > M_PI)
            {
                new_theta = new_theta - 2 * M_PI;
            }
            currentPose = Position(new_x, new_y, new_theta);
        }
        if (!init_pos_received)
        {
            init_pos_received = true;
            previous = currentPose;
        }
        current = currentPose;
    }

    // Callback function for laser_scan message
    void updateLaserScan(const sensor_msgs::msg::LaserScan &scan)
    {
        laserscan = scan;
        init_laser_received = true;
    }

    // Collect initial sensor data from robot
    void initialize(rclcpp::Node::SharedPtr node_ptr)
    {
        rclcpp::spin_some(node_ptr);
        std::cout << "initialize successful" << std::endl;
        previous = current;
    }

    // Call semaforr and execute decisions, until mission is successful
    void run(rclcpp::Node::SharedPtr node_ptr)
    {
        std::cout << "started run" << std::endl;

        Py_Initialize();
        geometry_msgs::msg::Twist base_cmd;

        rclcpp::Rate rate(30.0);
        double epsilon_move = 0.06; // Meters
        double epsilon_turn = 0.11; // Radians
        bool action_complete = true;
        bool mission_complete = false;
        FORRAction semaforr_action;
        double overallTimeSec = 0.0, computationTimeSec = 0.0, actionTimeSec = 0.0, action_start_time = 0.0, action_end_time = 0.0;
        timeval tv, cv, atv;
        double start_time, start_timecv;
        double end_time, end_timecv;
        gettimeofday(&tv, NULL);
        start_time = tv.tv_sec + (tv.tv_usec / 1000000.0);
        bool firstMessageReceived;

        std::cout << "run declarations completed" << std::endl;

        while (rclcpp::ok())
        {
            std::cout << "within ok() while loop" << std::endl;
            while (init_pos_received == false or init_laser_received == false)
            {
                RCLCPP_INFO(node_ptr->get_logger(), "Waiting for first pose and laser");
                rate.sleep();
                rclcpp::spin_some(node_ptr);
                firstMessageReceived = true;
            }
            gettimeofday(&tv, NULL);
            end_time = tv.tv_sec + (tv.tv_usec / 1000000.0);
            overallTimeSec = (end_time - start_time);

            if (action_complete)
            {
                RCLCPP_INFO(node_ptr->get_logger(), "Action completed. Save sensor info, Current position: %f %f %f", current.getX(), current.getY(), current.getTheta());
                if (firstMessageReceived == true)
                {
                    firstMessageReceived = false;
                }
                else
                {
                    viz_->publishLog(semaforr_action, overallTimeSec, computationTimeSec);
                    controller->gethighwayExploration()->setHighwaysComplete(overallTimeSec);
                    controller->getfrontierExploration()->setFrontiersComplete(overallTimeSec);
                }
                gettimeofday(&cv,NULL);
				start_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
				controller->updateState(current, laserscan, crowdPose, crowdPoseAll);
				// ROS_DEBUG("Finished UpdateState");
				viz_->publish();
				// ROS_DEBUG("Finished Publish");
				previous = current;
				//ROS_DEBUG("Check if mission is complete");
				mission_complete = controller->isMissionComplete();
                
                if(mission_complete)
                {
					RCLCPP_INFO(node_ptr->get_logger(), "Mission completed");
					gettimeofday(&cv,NULL);
					end_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
					computationTimeSec = (end_timecv-start_timecv);
					viz_->publishLog(semaforr_action, overallTimeSec, computationTimeSec);
					controller->gethighwayExploration()->setHighwaysComplete(overallTimeSec);
					controller->getfrontierExploration()->setFrontiersComplete(overallTimeSec);
					break;
				}
				else
                {
					RCLCPP_INFO(node_ptr->get_logger(), "Mission still in progress, invoke semaforr");
					semaforr_action = controller->decide();
					RCLCPP_INFO_STREAM(node_ptr->get_logger(), "SemaFORRdecision is " << semaforr_action.type << " " << semaforr_action.parameter); 
					base_cmd = convert_to_vel(semaforr_action);
					action_complete = false;
					actionTimeSec=0.0;
					gettimeofday(&atv,NULL);
					action_start_time = atv.tv_sec + (atv.tv_usec/1000000.0);
				}
				gettimeofday(&cv,NULL);
				end_timecv = cv.tv_sec + (cv.tv_usec/1000000.0);
				computationTimeSec = (end_timecv-start_timecv);
			}
			//send the drive command 
			cmd_vel_pub_->publish(base_cmd);
			//ROS_INFO_STREAM("Published base_cmd : " << overallTimeSec);
			//wait for some time
			rate.sleep();
			// Sense input 
			rclcpp::spin_some(node_ptr);
			gettimeofday(&atv,NULL);
			action_end_time = atv.tv_sec + (atv.tv_usec/1000000.0);
			actionTimeSec = (action_end_time - action_start_time);
			//ROS_INFO_STREAM("Action Time (sec) : " << actionTimeSec);
			// Check if the action is complete
			action_complete = testActionCompletion(semaforr_action, current, previous, epsilon_move, epsilon_turn, actionTimeSec);
			//action_complete = true;
		}
		Py_Finalize();
	}


	//! Drive the robot according to the semaforr action, episilon = 0 to 1 indicating percent of task completion
	// Need to improve this
	bool testActionCompletion(FORRAction action, Position current, Position previous, double epsilon_move, double epsilon_rotate, double elapsed_time)
	{
		// ROS_DEBUG("Testing if action has been completed by the robot");
		// ROS_DEBUG_STREAM("Current position " << current.getX() << " " << current.getY() << " " << current.getTheta()); 
		// ROS_DEBUG_STREAM("Previous position " << previous.getX() << " " << previous.getY() << " " << previous.getTheta());
		bool actionComplete = false;
		//ROS_DEBUG_STREAM("Position expected " << expected.getX() << " " << expected.getY() << " " << expected.getTheta());
		if (action.type == FORWARD){
			double distance_travelled = previous.getDistance(current);
			double expected_travel = controller->getBeliefs()->getAgentState()->getMovement(action.parameter);
			//if((abs(distance_travelled - expected_travel) < epsilon_move)){
			// if ((elapsed_time >= 0.01 and action.parameter == 0) or (elapsed_time >= expected_travel*0.6692+0.0111) or (fabs(distance_travelled - expected_travel) < epsilon_move)){
			if ((elapsed_time >= 0.01 and action.parameter == 0) or (elapsed_time >= expected_travel) or (fabs(distance_travelled - expected_travel) < epsilon_move)){
				// ROS_INFO_STREAM("elapsed_time : " << elapsed_time << " " << fabs(distance_travelled - expected_travel));
				actionComplete = true;
			}
			else{
				//ROS_INFO_STREAM("elapsed_time : " << elapsed_time << " " << abs(distance_travelled - expected_travel));
				actionComplete = false;  
			}
		}
		else if(action.type == RIGHT_TURN or action.type == LEFT_TURN){
			double turn_completed = current.getTheta() - previous.getTheta();
		
			if(turn_completed > M_PI)
				turn_completed = turn_completed - (2*M_PI);
			if(turn_completed < -M_PI)
				turn_completed = turn_completed + (2*M_PI);

			double turn_expected = fabs(controller->getBeliefs()->getAgentState()->getRotation(action.parameter));
			//if(action.type == RIGHT_TURN) 
			//	turn_expected = (-1)*turn_expected;
			//if((abs(turn_completed - turn_expected) < epsilon_rotate)){
			// if((elapsed_time >= (-0.0385*pow(turn_expected,2))+(0.7916*turn_expected)) or (fabs(fabs(turn_completed) - turn_expected) < epsilon_rotate)){
			if((elapsed_time >= turn_expected/0.5) or (fabs(fabs(turn_completed) - turn_expected) < epsilon_rotate) and fabs(turn_completed) > 0){
				// ROS_INFO_STREAM("elapsed_time : " << elapsed_time << " " << fabs(fabs(turn_completed) - turn_expected));
				actionComplete = true;
			}
			else {
				//ROS_INFO_STREAM("elapsed_time : " << elapsed_time << " " << abs(turn_completed - turn_expected));
				actionComplete = false;
			}
		}
		else if((action.type == PAUSE) or (elapsed_time >= 0.01)){
			//ROS_INFO_STREAM("elapsed_time : " << elapsed_time);
			actionComplete = true;
		}
		//ROS_DEBUG_STREAM(" Action Completed ? : " << actionComplete);
		return actionComplete;
	}

	
	// Function that converts FORRAction into ROS compatible base_cmd

	geometry_msgs::msg::Twist convert_to_vel(FORRAction action){

		geometry_msgs::msg::Twist base_cmd;

		base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0; 
		if(action.type == FORWARD){
			base_cmd.linear.x = 0.5; //0.5 meters per second
		}   
		else if(action.type == RIGHT_TURN){
			base_cmd.linear.x = 0.01;
			base_cmd.angular.z = -0.05; //-0.05 radians per second
		}
		else if(action.type == LEFT_TURN){
			base_cmd.linear.x = 0.01;
			base_cmd.angular.z = 0.05; //0.05 radians per second
		}
		else if(action.type == PAUSE){
			base_cmd.linear.x = 0;
		}
		return base_cmd;
	}

};

int main(int argc, char **argv) {
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotDriver>();
    node->initialize_viz(node);

    RCLCPP_INFO(node->get_logger(), "Starting... semaforr");

    // if (argc != 7) {
    //     RCLCPP_ERROR(node->get_logger(), "Not enough parameters. Expected 6, got %d", argc);
    //     return 1;  // Exit if not enough parameters
    // }

    // std::string path(argv[1]);
    // std::string target_set(argv[2]);
    // std::string map_config(argv[3]);
    // std::string map_dimensions(argv[4]);
    // std::string advisors(argv[5]);
    // std::string params(argv[6]);

    // std::string advisor_config = path + advisors;
    // std::string params_config = path + params;

    // Create the controller
    // Controller *controller = new Controller(advisor_config, params_config, map_config, target_set, map_dimensions);
    
    // testing robot driver init
    //auto driver_node = std::make_shared<RobotDriver>(RobotDriver(controller));
    //rclcpp::spin(node);

    // Create the RobotDriver and pass the node to it
    // cout << "Trying to construct the ROS2 Node" << endl;
    // RobotDriver driver(controller);
    // cout << "Succeeded in creating the ROS2 Node" << endl;

    // std::cout << "Trying to init the node" << std::endl;
    node->initialize(node);
    // std::cout << "initialize successful" << std::endl;
    RCLCPP_INFO(node->get_logger(), "Robot Driver Initialized");

    // Run the driver
    node->run(node);  //pass in node param

    // RCLCPP_INFO(node->get_logger(), "Mission Accomplished!");

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}