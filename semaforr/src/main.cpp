/* \mainpage ROSiffied semaforr Documentation
 * \brief RobotDriver governs low-level actions of a robot.
 *
 * \author Anoop Aroor, Raj Korpan and others.
 *
 * \version SEMAFORR ROS 1.0
 *
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
class RobotDriver : public rclcpp::Node
{
private:
    //! We will be publishing to the "cmd_vel" topic to issue commands
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    //! We will be listening to /pose, /laserscan and /crowd_model, /crowd_pose topics
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_;
    rclcpp::Subscription<semaforr::msg::CrowdModel>::SharedPtr sub_crowd_model_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_crowd_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_crowd_pose_all_;

    // Current position and previous stopping position of the robot
    Position current, previous;
    // Current and previous laser scan
    sensor_msgs::msg::LaserScan laserscan;
    // Current crowd_model
    semaforr::msg::CrowdModel crowdModel;
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
    RobotDriver(Controller *con)
        : Node("semaforr")
        : rclcpp::Node
    {
        // Set up the publisher for the cmd_vel topic
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "pose", 10, std::bind(&RobotDriver::updatePose, this, std::placeholders::_1));
        sub_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "base_scan", 10, std::bind(&RobotDriver::updateLaserScan, this, std::placeholders::_1));
        sub_crowd_model_ = this->create_subscription<semaforr::msg::CrowdModel>(
            "crowd_model", 10, std::bind(&RobotDriver::updateCrowdModel, this, std::placeholders::_1));
        sub_crowd_pose_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "crowd_pose", 10, std::bind(&RobotDriver::updateCrowdPose, this, std::placeholders::_1));
        sub_crowd_pose_all_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "crowd_pose_all", 10, std::bind(&RobotDriver::updateCrowdPoseAll, this, std::placeholders::_1));

        // Declare and create a controller with task, action and advisor configuration
        controller = con;
        init_pos_received = false;
        init_laser_received = false;
        current.setX(0);
        current.setY(0);
        current.setTheta(0);
        add_noise = false;
        previous.setX(0);
        previous.setY(0);
        previous.setTheta(0);
        auto node_ptr = shared_from_this();
        classB = std::make_shared<Visualizer>(node_ptr);
        viz_ = new Visualizer(node_ptr, con);
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
    void updateCrowdModel(const semaforr::msg::CrowdModel &crowd_model)
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
    void initialize()
    {
        rclcpp::spin_some(shared_from_this());
        previous = current;
    }

    // Call semaforr and execute decisions, until mission is successful
    void run()
    {
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
        bool firstMessageReceived = false;

        while (rclcpp::ok())
        {
            while (!init_pos_received || !init_laser_received)
            {
                // RCLCPP_DEBUG(this->get_logger(), "Waiting for first message or laser");
                rate.sleep();
                rclcpp::spin_some(shared_from_this());
                firstMessageReceived = true;
            }
            gettimeofday(&tv, NULL);
            end_time = tv.tv_sec + (tv.tv_usec / 1000000.0);
            overallTimeSec = (end_time - start_time);

            if (action_complete)
            {
                // RCLCPP_INFO(this->get_logger(), "Action completed. Save sensor info, Current position: %f %f %f", current.getX(), current.getY(), current.getTheta());
                if (firstMessageReceived)
                {
                    firstMessageReceived = false;
                }
                else
                {
                    viz_->publishLog(semaforr_action, overallTimeSec, computationTimeSec);
                    controller->gethighwayExploration()->setHighwaysComplete(overallTimeSec);
                    controller->getfrontierExploration()->setFrontiersComplete(overallTimeSec);
                }
                gettimeofday(&cv, NULL);
                start_timecv = cv.tv_sec + (cv.tv_usec / 1000000.0);
                controller->updateBeliefs(current, laserscan, crowdPose, crowdPoseAll, overallTimeSec);
                semaforr_action = controller->deliberate();
                gettimeofday(&cv, NULL);
                end_timecv = cv.tv_sec + (cv.tv_usec / 1000000.0);
                computationTimeSec = (end_timecv - start_timecv);
                if (controller->getBeliefs()->goalReached(current))
                {
                    mission_complete = true;
                    // RCLCPP_INFO(this->get_logger(), "Mission completed, goal reached.");
                }
                else if (semaforr_action.getCode() == "Nothing")
                {
                    mission_complete = true;
                    // RCLCPP_INFO(this->get_logger(), "Mission failed, semaforr did not generate an action.");
                }
                else
                {
                    action_complete = false;
                    action_start_time = overallTimeSec;
                }
            }
            else
            {
                if (semaforr_action.getCode() == "Move")
                {
                    // RCLCPP_INFO(this->get_logger(), "Move command: Distance remaining %f %f", semaforr_action.getParameter(0), epsilon_move);
                    if (semaforr_action.getParameter(0) < epsilon_move)
                    {
                        action_complete = true;
                        action_end_time = overallTimeSec;
                        actionTimeSec = (action_end_time - action_start_time);
                        controller->updateExploration(current, previous, actionTimeSec, overallTimeSec, true);
                    }
                    else
                    {
                        base_cmd.linear.x = min(0.8, semaforr_action.getParameter(0));
                    }
                }
                else if (semaforr_action.getCode() == "Turn")
                {
                    // RCLCPP_INFO(this->get_logger(), "Turn command: Distance remaining %f %f", semaforr_action.getParameter(0), epsilon_turn);
                    if (semaforr_action.getParameter(0) < epsilon_turn)
                    {
                        action_complete = true;
                        action_end_time = overallTimeSec;
                        actionTimeSec = (action_end_time - action_start_time);
                        controller->updateExploration(current, previous, actionTimeSec, overallTimeSec, false);
                    }
                    else
                    {
                        base_cmd.angular.z = min(1.0, semaforr_action.getParameter(0));
                    }
                }
                else
                {
                    action_complete = true;
                }
            }
            previous = current;
            cmd_vel_pub_->publish(base_cmd);
            base_cmd.linear.x = 0;
            base_cmd.angular.z = 0;
            rclcpp::spin_some(shared_from_this());
            if (mission_complete)
            {
                break;
            }
            rate.sleep();
        }
        Py_Finalize();
    }
};

int main(int argc, char **argv)
{
    Py_Initialize();
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting... semaforr");
    Controller *controller = new Controller();
    auto robot_driver = std::make_shared<RobotDriver>(controller);
    robot_driver->initialize();
    robot_driver->run();
    Py_Finalize();
    rclcpp::shutdown();
    return 0;
}
