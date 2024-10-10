/**
 * @file Person.hpp
 * @brief Person class definition for SemaFORR's social context model
 *
 * Detailed description:
 * This file declares the Person class, which is used to represent people during social robot navigation
 * Includes pose, velocity, history data members
 *
 * @author Eric Guan
 * @date 2024-10-09
 * @version 1.0
 *
 * @note
 * - This class is a work in progress
 */

#include <vector>

struct Pose
{
    // (x,y) coordinate values, (theta) angle orientation
    int x;
    int y;
    double theta;
};

class Person
{
private:
    Pose pose_cur;
    std::vector<Pose> history;
    double velocity;
    // facial recognition?
    Pose pose_pred;
    bool in_camera;
};